#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

// ── PID ──────────────────────────────────────────────────────
class PIDController
{
public:
  PIDController(double p, double i, double d, double mn, double mx)
    : kp(p), ki(i), kd(d), min_out(mn), max_out(mx) {}

  void reset() { integral = 0.0; prev_error = 0.0; }

  double compute(double error, double dt) {
    if (dt <= 0.0) return 0.0;
    integral = std::clamp(integral + error * dt, -10.0, 10.0);
    double out = kp * error + ki * integral + kd * (error - prev_error) / dt;
    prev_error = error;
    return std::clamp(out, min_out, max_out);
  }

private:
  double kp, ki, kd, min_out, max_out;
  double integral = 0.0, prev_error = 0.0;
};

// ── Hareket Durumları ────────────────────────────────────────
enum class MotionState { IDLE, TURNING, MOVING };

// ── Node ─────────────────────────────────────────────────────
class MazeMotionNode : public rclcpp::Node
{
public:
  MazeMotionNode() : Node("maze_motion_node")
  {
    // Publishers
    vel_pub_    = this->create_publisher<geometry_msgs::msg::TwistStamped>(
                    "/a200_0000/cmd_vel", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
                    "/maze_motion_status", 10);

    // Subscribers
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
      "/maze_motion_cmd", 10,
      std::bind(&MazeMotionNode::cmd_callback, this, _1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/a200_0000/sensors/lidar2d_0/scan",
      rclcpp::SensorDataQoS(),
      std::bind(&MazeMotionNode::scan_callback, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/a200_0000/platform/odom/filtered", 10,
      std::bind(&MazeMotionNode::odom_callback, this, _1));

    timer_ = this->create_wall_timer(
      50ms, std::bind(&MazeMotionNode::motion_loop, this));

    last_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "MazeMotionNode başlatıldı.");
  }

private:
  // ── Sabitler ──────────────────────────────────────────────
  const double CELL_SIZE          = 2.05;
  const double MOVE_TOLERANCE     = 0.01;   // metre (mesafe hatası eşiği)
  const double ANGLE_TOLERANCE    = 0.5;    // derece
  const double WINDOW_DEGREES     = 20.0;
  const double MAX_SENSOR_RANGE   = 30.0;
  const double WALL_THRESHOLD     = 1.6;
  const double WALL_LOST_THRESHOLD= 1.5;
  const double EMERGENCY_STOP_DIST= 0.3;
  const double IDEAL_DIST         = 0.9;

  // ── Lidar-Odometri Füzyon ─────────────────────────────────
  // FUSION_ALPHA: odom ağırlığı (0=saf lidar, 1=saf odom)
  // Önde duvar yoksa lidar güvenilmez → otomatik olarak α=1.0'a geçer
  const double FUSION_ALPHA            = 0.0;
  const double LIDAR_RELIABLE_MAX_DIST = 4.0;   // bu değerin üstünde lidar pas geçilir

  // ── PID'ler ───────────────────────────────────────────────
  PIDController pid_turn_    {0.04, 0.001, 0.005, -0.6,  0.6 };
  PIDController pid_distance_{3.0,  0.005, 0.1,    0.0,  5.0 };
  PIDController pid_center_  {0.8,  0.0,   0.2,   -0.5,  0.5 };

  // ── Durum ─────────────────────────────────────────────────
  MotionState motion_state_ = MotionState::IDLE;

  // Hedef bilgileri (Decision node'dan gelir)
  double target_yaw_deg_  = 0.0;
  double start_move_x_    = 0.0;
  double start_move_y_    = 0.0;
  double front_dist_at_start_ = 0.0;   // MOVING'e girerken lidar snapshot
  bool   lidar_fusion_valid_  = false;  // başlangıçta önde duvar var mıydı?
  bool   has_target_      = false;

  // Sensörler
  double front_dist_      = 0.0;
  double left_dist_       = 0.0;
  double right_dist_      = 0.0;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

  double current_x_       = 0.0;
  double current_y_       = 0.0;
  double current_yaw_deg_ = 0.0;
  bool   scan_received_   = false;
  bool   odom_received_   = false;

  rclcpp::Time last_time_;

  // ── Yardımcılar ──────────────────────────────────────────

  double normalize_angle(double a) {
    while (a >  180.0) a -= 360.0;
    while (a < -180.0) a += 360.0;
    return a;
  }

  double getAverageRange(const sensor_msgs::msg::LaserScan::SharedPtr& msg, double angle_rad)
  {
    double min_dist = MAX_SENSOR_RANGE;
    bool found = false;
    double half = (WINDOW_DEGREES / 2.0) * M_PI / 180.0;
    int s = std::max(0, (int)((angle_rad - half - msg->angle_min) / msg->angle_increment));
    int e = std::min((int)msg->ranges.size()-1,
                     (int)((angle_rad + half - msg->angle_min) / msg->angle_increment));
    for (int i = s; i <= e; ++i) {
      double r = msg->ranges[i];
      if (!std::isnan(r) && !std::isinf(r) && r >= msg->range_min && r <= msg->range_max)
        if (r < min_dist) { min_dist = r; found = true; }
    }
    return found ? min_dist : MAX_SENSOR_RANGE;
  }

  // Duvara paralel açı hatası (sol veya sağ duvar)
  double calculate_wall_angle(bool is_left)
  {
    if (!last_scan_) return 0.0;

    double angle_front_deg = is_left ?  70.0 : -70.0;
    double angle_back_deg  = is_left ? 110.0 : -110.0;

    auto get_point = [&](double target_deg) -> std::pair<double,double>
    {
      double tr = target_deg * M_PI / 180.0;
      double wr = 5.0 * M_PI / 180.0;
      int s = std::max(0, (int)((tr - wr - last_scan_->angle_min) / last_scan_->angle_increment));
      int e = std::min((int)last_scan_->ranges.size()-1,
                       (int)((tr + wr - last_scan_->angle_min) / last_scan_->angle_increment));
      double sx = 0, sy = 0;
      int cnt = 0;
      for (int i = s; i <= e; ++i) {
        double r = last_scan_->ranges[i];
        if (!std::isnan(r) && !std::isinf(r) && r > 0.1 && r < 2.5) {
          double a = last_scan_->angle_min + i * last_scan_->angle_increment;
          sx += r * std::cos(a); sy += r * std::sin(a); cnt++;
        }
      }
      return cnt ? std::make_pair(sx/cnt, sy/cnt) : std::make_pair(0.0, 0.0);
    };

    auto pf = get_point(angle_front_deg);
    auto pb = get_point(angle_back_deg);
    if ((pf.first == 0 && pf.second == 0) || (pb.first == 0 && pb.second == 0))
      return 0.0;
    return std::atan2(pf.second - pb.second, pf.first - pb.first);
  }

  void publish_status(const std::string& status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Status: %s", status.c_str());
  }

  void stop_robot()
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp    = this->now();
    cmd.header.frame_id = "base_link";
    vel_pub_->publish(cmd);
  }

  // ── Callback'ler ─────────────────────────────────────────

  void cmd_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
  {
    // Yeni hareket komutu: önce dön, sonra ilerle
    target_yaw_deg_ = msg->theta;
    pid_turn_.reset();
    motion_state_ = MotionState::TURNING;
    has_target_   = true;

    RCLCPP_INFO(this->get_logger(),
      "Yeni komut: Grid(%.0f,%.0f) yaw=%.1f°",
      msg->x, msg->y, msg->theta);
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_scan_    = msg;
    scan_received_= true;
    front_dist_   = getAverageRange(msg, 0.0);
    left_dist_    = getAverageRange(msg, M_PI_2);
    right_dist_   = getAverageRange(msg, -M_PI_2);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_received_ = true;
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double r, p, yaw;
    tf2::Matrix3x3(q).getRPY(r, p, yaw);
    current_yaw_deg_ = yaw * 180.0 / M_PI;
  }

  // ── Hareket döngüsü ──────────────────────────────────────

  void motion_loop()
  {
    if (!scan_received_ || !odom_received_ || !has_target_) return;

    rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;
    if (dt <= 0.0 || dt > 0.2) return;

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp    = now;
    cmd.header.frame_id = "base_link";

    switch (motion_state_)
    {
    // ── TURNING ──────────────────────────────────────────
    case MotionState::TURNING:
    {
      double error = normalize_angle(target_yaw_deg_ - current_yaw_deg_);

      if (std::abs(error) < ANGLE_TOLERANCE) {
        stop_robot();
        pid_turn_.reset();
        start_move_x_  = current_x_;
        start_move_y_  = current_y_;
        // Lidar snapshot: önde güvenilir mesafede duvar varsa füzyon aktif
        front_dist_at_start_ = front_dist_;
        lidar_fusion_valid_  = (front_dist_ < LIDAR_RELIABLE_MAX_DIST);
        if (lidar_fusion_valid_)
          RCLCPP_INFO(this->get_logger(),
            "Lidar füzyon AKTİF — başlangıç front=%.3fm", front_dist_at_start_);
        else
          RCLCPP_INFO(this->get_logger(),
            "Lidar füzyon KAPALI — önde duvar yok, saf odom.");
        pid_distance_.reset();
        pid_center_.reset();
        motion_state_  = MotionState::MOVING;
      }
      else {
        double w = pid_turn_.compute(error, dt);
        if (std::abs(w) < 0.15) w = (error > 0) ? 0.15 : -0.15;
        cmd.twist.angular.z = w;
        cmd.twist.linear.x  = 0.0;
        vel_pub_->publish(cmd);
      }
    }
    break;

    // ── MOVING ───────────────────────────────────────────
    case MotionState::MOVING:
    {
      // Acil fren
      if (front_dist_ < EMERGENCY_STOP_DIST) {
        stop_robot();
        RCLCPP_WARN(this->get_logger(), "ACİL FREN! Engel algılandı.");
        publish_status("DONE");
        motion_state_ = MotionState::IDLE;
        has_target_   = false;
        return;
      }

      // ── Mesafe hesabı: Lidar-Odometri Füzyonu ──────────────
      // Odometri: hypot daha doğru (max yerine)
      double odom_dist = std::hypot(current_x_ - start_move_x_,
                                    current_y_ - start_move_y_);

      // Lidar: başlangıçtaki front mesafesinden ne kadar yaklaştık?
      double lidar_dist = front_dist_at_start_ - front_dist_;
      lidar_dist = std::clamp(lidar_dist, 0.0, CELL_SIZE + 0.3);  // outlier koruması

      // Önde güvenilir duvar yoksa α=1 (saf odom), varsa FUSION_ALPHA
      double alpha = lidar_fusion_valid_ ? FUSION_ALPHA : 1.0;
      double dist_moved = alpha * odom_dist + (1.0 - alpha) * lidar_dist;

      double dist_error = CELL_SIZE - dist_moved;

      if (dist_error <= MOVE_TOLERANCE) {
        stop_robot();
        pid_distance_.reset();
        pid_center_.reset();
        publish_status("DONE");
        motion_state_ = MotionState::IDLE;
        has_target_   = false;
      }
      else {
        cmd.twist.linear.x = pid_distance_.compute(dist_error, dt);

        // Duvar hizalama
        bool lw = (left_dist_  < WALL_LOST_THRESHOLD);
        bool rw = (right_dist_ < WALL_LOST_THRESHOLD);
        double corr = 0.0;

        if (lw && rw) {
          corr = (left_dist_ - right_dist_) + (calculate_wall_angle(true) + calculate_wall_angle(false));
        }
        else if (rw) {
          corr = (IDEAL_DIST - right_dist_) + calculate_wall_angle(false);
        }
        else if (lw) {
          corr = (left_dist_ - IDEAL_DIST)+ calculate_wall_angle(true);
        }
        else {
          corr = normalize_angle(target_yaw_deg_ - current_yaw_deg_) * (M_PI / 180.0);
        }

        cmd.twist.angular.z = pid_center_.compute(corr, dt);
        vel_pub_->publish(cmd);
      }
    }
    break;

    case MotionState::IDLE:
    default:
      break;
    }
  }

  // ── ROS üyeleri ──────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr    cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr   scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// ── main ─────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MazeMotionNode>());
  rclcpp::shutdown();
  return 0;
}