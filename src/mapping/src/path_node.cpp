#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <vector>
#include <cmath>
#include <algorithm>

struct Pose2D { double x, y, yaw; };

enum class RecoveryState { NONE, BACKING_UP };

static inline double normAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

class PathFollowerNode : public rclcpp::Node
{
public:
    PathFollowerNode() : Node("path_follower_node")
    {
        using std::placeholders::_1;

        // ── Nav2 RPP Tarzı Parametreler ───────────────────────────────────────
        declare_parameter("stop_distance_m",       0.25);  
        declare_parameter("lookahead_dist_m",      0.40);  
        declare_parameter("goal_tolerance_m",      0.10);  
        declare_parameter("max_linear_vel",        2.55);  
        declare_parameter("min_linear_vel",        0.40);  
        declare_parameter("max_angular_vel",       1.00);  
        
        // Regülasyon Parametreleri
        declare_parameter("approach_dist_m",       1.50);  
        declare_parameter("rotate_to_heading_ang", 0.80);  // 0.60'tan 0.80'e çıkarıldı (Daha esnek dönüş toleransı)
        declare_parameter("regulated_vel_scaling", 0.35);  

        // Stuck Parametreleri
        declare_parameter("stuck_timeout_s",       10.0);  
        declare_parameter("stuck_dist_m",          0.10);  

        stop_dist_m_     = get_parameter("stop_distance_m").as_double();
        lookahead_dist_  = get_parameter("lookahead_dist_m").as_double();
        goal_tol_        = get_parameter("goal_tolerance_m").as_double();
        max_lin_         = get_parameter("max_linear_vel").as_double();
        min_lin_         = get_parameter("min_linear_vel").as_double();
        max_ang_         = get_parameter("max_angular_vel").as_double();
        
        approach_dist_   = get_parameter("approach_dist_m").as_double();
        rotate_to_ang_   = get_parameter("rotate_to_heading_ang").as_double();
        reg_vel_scaling_ = get_parameter("regulated_vel_scaling").as_double();

        stuck_timeout_   = get_parameter("stuck_timeout_s").as_double();
        stuck_dist_      = get_parameter("stuck_dist_m").as_double();

        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/plan", 10, std::bind(&PathFollowerNode::pathCb, this, _1));
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/corrected_pose", 10, std::bind(&PathFollowerNode::poseCb, this, _1));
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/a200_0000/sensors/lidar2d_0/scan", 10, std::bind(&PathFollowerNode::scanCb, this, _1));

        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/a200_0000/cmd_vel", 10);

        // Timer 100ms'den 50ms'e çekildi (Overshoot önleme)
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PathFollowerNode::controlLoop, this));

        last_pose_time_ = now();
        last_stuck_check_pose_ = {0, 0, 0};
    }

private:
    Pose2D  pose_{};
    bool    pose_ready_  = false;
    std::vector<Pose2D> path_;                   

    bool   obstacle_ahead_ = false;
    double closest_front_  = 1e9;

    rclcpp::Time  last_pose_time_;
    Pose2D        last_stuck_check_pose_{};
    rclcpp::Time  last_moved_time_;
    bool          stuck_check_init_ = false;

    RecoveryState recovery_state_ = RecoveryState::NONE;
    double        recovery_turn_dir_ = 1.0;
    rclcpp::Time  recovery_start_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr            path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr  cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double stop_dist_m_, lookahead_dist_, goal_tol_;
    double max_lin_, min_lin_, max_ang_;
    double approach_dist_, rotate_to_ang_, reg_vel_scaling_;
    double stuck_timeout_, stuck_dist_;

    void pathCb(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (recovery_state_ != RecoveryState::NONE) return;

        if (!path_.empty()) {
            return; 
        }

        for (const auto& p : msg->poses) {
            path_.push_back({p.pose.position.x, p.pose.position.y, 0.0});
        }
        
        RCLCPP_INFO(get_logger(), "Yeni rota alindi ve kilitlendi. Hedef bitene veya engel cikana kadar baska rota alinmayacak.");
    }

    void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        pose_.x   = msg->pose.position.x;
        pose_.y   = msg->pose.position.y;
        double z  = msg->pose.orientation.z;
        double w  = msg->pose.orientation.w;
        pose_.yaw = std::atan2(2.0 * z * w, 1.0 - 2.0 * z * z);
        pose_ready_ = true;

        if (!stuck_check_init_) {
            last_stuck_check_pose_ = pose_;
            last_moved_time_       = now();
            stuck_check_init_      = true;
        }
    }

    void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        obstacle_ahead_ = false;
        closest_front_  = 1e9;
        
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float r = msg->ranges[i];
            if (r < msg->range_min || r > msg->range_max || std::isnan(r)) continue;

            float a = msg->angle_min + i * msg->angle_increment;
            if (std::fabs(a) > 0.80) continue; 

            if (r < closest_front_) closest_front_ = r;
            if (r < stop_dist_m_)   obstacle_ahead_ = true;
        }
    }

    bool isStuck()
    {
        if (!stuck_check_init_) return false;
        double moved = std::hypot(pose_.x - last_stuck_check_pose_.x,
                                  pose_.y - last_stuck_check_pose_.y);
        auto now_t = now();
        if (moved > stuck_dist_) {
            last_stuck_check_pose_ = pose_;
            last_moved_time_       = now_t;
        }
        double elapsed = (now_t - last_moved_time_).seconds();
        return elapsed > stuck_timeout_;
    }

    void controlLoop()
    {
        if (!pose_ready_) return;

        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp    = now();
        cmd.header.frame_id = "base_link";

        if (recovery_state_ == RecoveryState::BACKING_UP) {
            if ((now() - recovery_start_).seconds() < 1.5) {
                cmd.twist.linear.x = -0.15; 
                cmd.twist.angular.z = recovery_turn_dir_ * 0.8;
                cmd_vel_pub_->publish(cmd);
                return;
            } else {
                recovery_state_ = RecoveryState::NONE;
                last_moved_time_ = now();
                path_.clear(); 
            }
        }

        if (isStuck() && !path_.empty()) {
            recovery_turn_dir_ = 1.0;
            recovery_state_ = RecoveryState::BACKING_UP;
            recovery_start_ = now();
            return;
        }

        if (obstacle_ahead_) {
            cmd.twist.linear.x  = 0.0;
            cmd.twist.angular.z = (closest_front_ > 0.0) ? 0.7 : -0.7; 
            cmd_vel_pub_->publish(cmd);
            path_.clear(); 
            return;
        }

        if (path_.empty()) {
            cmd_vel_pub_->publish(cmd); 
            return;
        }

        // ── 1. En yakın noktayı bul ve geride kalan "hayalet" noktaları sil ──
        if (!path_.empty()) {
            size_t closest_idx = 0;
            double min_dist = 1e9;
            for (size_t i = 0; i < path_.size(); ++i) {
                double d = std::hypot(path_[i].x - pose_.x, path_[i].y - pose_.y);
                if (d < min_dist) {
                    min_dist = d;
                    closest_idx = i;
                }
            }
            path_.erase(path_.begin(), path_.begin() + closest_idx);
        }

        // ── 2. Kalan rotanın son noktasına ulaşıldı mı? ──
        if (!path_.empty() && std::hypot(path_.back().x - pose_.x, path_.back().y - pose_.y) < goal_tol_) {
            path_.clear();
            cmd_vel_pub_->publish(cmd);
            return;
        }
        
        if (path_.empty()) { cmd_vel_pub_->publish(cmd); return; }

        // ── 3. Lookahead (İleri Bakış) Noktasını Bulma ────────────────────────
        size_t target_idx = path_.size() - 1;
        for (size_t i = 0; i < path_.size(); ++i) {
            if (std::hypot(path_[i].x - pose_.x, path_[i].y - pose_.y) >= lookahead_dist_) {
                target_idx = i; break;
            }
        }

        double tx2 = path_[target_idx].x;
        double ty2 = path_[target_idx].y;
        double dx = tx2 - pose_.x;
        double dy = ty2 - pose_.y;
        
        double dist_to_final_goal = std::hypot(path_.back().x - pose_.x, path_.back().y - pose_.y);
        double alpha = normAngle(std::atan2(dy, dx) - pose_.yaw);
        double lookahead_l = std::hypot(dx, dy);

        // ── 4. Rotate to Heading (Olduğun Yerde Dön) ──────────────────────────
        if (std::fabs(alpha) > rotate_to_ang_) {
            cmd.twist.linear.x = 0.0;
            double rot_vel = std::clamp(1.5 * alpha, -max_ang_, max_ang_);
            if (std::fabs(rot_vel) < 0.2) rot_vel = std::copysign(0.2, alpha);
            cmd.twist.angular.z = rot_vel;
            cmd_vel_pub_->publish(cmd);
            return;
        }

        // ── 5. Nav2 Pure Pursuit Matematiği (Curvature) ───────────────────────
        double curvature = 0.0;
        if (lookahead_l > 0.001) {
            curvature = 2.0 * std::sin(alpha) / lookahead_l;
        }

        // ── 6. Regulated Linear Velocity (Hız Regülasyonu) ────────────────────
        double v_target = max_lin_;

        if (dist_to_final_goal < approach_dist_) {
            v_target = max_lin_ * (dist_to_final_goal / approach_dist_);
        }

        if (std::fabs(curvature) > 0.1) {
            double safe_vel_for_curve = reg_vel_scaling_ / std::fabs(curvature);
            v_target = std::min(v_target, safe_vel_for_curve);
        }

        cmd.twist.linear.x = std::clamp(v_target, min_lin_, max_lin_);

        // ── 7. Açısal Hızı Hesapla (w = v * k) ────────────────────────────────
        cmd.twist.angular.z = cmd.twist.linear.x * curvature;
        cmd.twist.angular.z = std::clamp(cmd.twist.angular.z, -max_ang_, max_ang_);

        cmd_vel_pub_->publish(cmd);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollowerNode>());
    rclcpp::shutdown();
    return 0;
}