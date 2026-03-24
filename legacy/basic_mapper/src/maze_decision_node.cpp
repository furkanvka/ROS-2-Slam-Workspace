#include <cmath>
#include <algorithm>
#include <map>
#include <set>
#include <stack>
#include <queue>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

// ── Veri yapıları ────────────────────────────────────────────
struct GridPoint {
  int x = 0, y = 0;
  bool operator<(const GridPoint& o) const { return x != o.x ? x < o.x : y < o.y; }
  bool operator==(const GridPoint& o) const { return x == o.x && y == o.y; }
};

struct CellInfo {
  bool visited    = false;
  bool wall_north = false;
  bool wall_south = false;
  bool wall_east  = false;
  bool wall_west  = false;
};

using GridMap = std::map<GridPoint, CellInfo>;

enum class State {
  IDLE,
  SCANNING,
  WAITING_MOTION,
  PLANNING,
  FOLLOW_PATH
};

// ── Node ─────────────────────────────────────────────────────
class MazeDecisionNode : public rclcpp::Node
{
public:
  MazeDecisionNode() : Node("maze_decision_node")
  {
    // ── Publishers ──────────────────────────────────────────
    motion_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>(
      "/maze_motion_cmd", 10);
    viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/maze_viz", 10);

    // ── Subscribers ─────────────────────────────────────────
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/a200_0000/sensors/lidar2d_0/scan",
      rclcpp::SensorDataQoS(),
      std::bind(&MazeDecisionNode::scan_callback, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/a200_0000/platform/odom/filtered", 10,
      std::bind(&MazeDecisionNode::odom_callback, this, _1));

    motion_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/maze_motion_status", 10,
      std::bind(&MazeDecisionNode::motion_status_callback, this, _1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/maze_goal", 10,
      [this](const geometry_msgs::msg::Point::SharedPtr msg) {
        user_goal_ = { static_cast<int>(msg->x), static_cast<int>(msg->y) };
        has_user_goal_ = true;
        RCLCPP_INFO(this->get_logger(), "Yeni hedef: Grid(%d,%d)", user_goal_.x, user_goal_.y);
        current_state_ = State::PLANNING;
        publish_viz();
      });

    // Karar döngüsü: 100ms
    timer_ = this->create_wall_timer(
      100ms, std::bind(&MazeDecisionNode::decision_loop, this));

    // Viz döngüsü: 500ms — karar döngüsünden bağımsız
    viz_timer_ = this->create_wall_timer(
      500ms, std::bind(&MazeDecisionNode::publish_viz, this));

    grid_map_[{0,0}].visited = true;
    RCLCPP_INFO(this->get_logger(), "MazeDecisionNode başlatıldı.");
  }

private:
  // ── Sabitler ──────────────────────────────────────────────
  const double CELL_SIZE       = 2.05;   // grid → world dönüşümü
  const double WALL_THICKNESS  = 0.08;
  const double WALL_HEIGHT     = 0.5;
  const double CELL_VIZ_SIZE   = 1.8;   // hücre marker boyutu
  const double WALL_THRESHOLD  = 1.6;
  const double MAX_SENSOR_RANGE= 30.0;
  const double WINDOW_DEGREES  = 20.0;

  // ── Durum ─────────────────────────────────────────────────
  State     current_state_ = State::IDLE;
  GridPoint current_grid_  = {0, 0};
  GridPoint target_grid_   = {0, 0};
  GridMap   grid_map_;
  std::stack<GridPoint> path_stack_;

  std::vector<GridPoint> planned_path_;
  size_t    path_index_    = 0;
  GridPoint user_goal_     = {0, 0};
  bool      has_user_goal_ = false;

  double front_dist_      = 0.0;
  double left_dist_       = 0.0;
  double right_dist_      = 0.0;
  double current_yaw_deg_ = 0.0;
  bool   scan_received_   = false;
  bool   odom_received_   = false;
  bool   motion_done_     = false;
  bool   motion_failed_   = false;

  // ── ROS üyeleri ──────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr           motion_cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr       scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr           odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr             motion_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr         goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr viz_timer_;

  // ════════════════════════════════════════════════════════
  //  VİZUALİZASYON
  // ════════════════════════════════════════════════════════

  geometry_msgs::msg::Point grid_to_world(int gx, int gy, double z = 0.0)
  {
    geometry_msgs::msg::Point p;
    p.x = gx * CELL_SIZE;
    p.y = gy * CELL_SIZE;
    p.z = z;
    return p;
  }

  visualization_msgs::msg::Marker make_marker(
    const std::string& ns, int id, int type,
    double r, double g, double b, double a = 0.8)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id    = "odom";
    m.header.stamp       = this->now();
    m.ns                 = ns;
    m.id                 = id;
    m.type               = type;
    m.action             = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    m.lifetime           = rclcpp::Duration(0, 0);  // kalıcı
    return m;
  }

  void publish_viz()
  {
    visualization_msgs::msg::MarkerArray arr;
    auto now = this->now();

    // ── Önce tüm namespace'leri temizle ───────────────────
    for (const auto& ns : {"cells", "walls", "robot", "goal", "path"})
    {
      visualization_msgs::msg::Marker del;
      del.header.frame_id = "odom";
      del.header.stamp    = now;
      del.ns              = ns;
      del.id              = 0;
      del.action          = visualization_msgs::msg::Marker::DELETEALL;
      arr.markers.push_back(del);
    }

    // ── 1. ZİYARET EDİLEN HÜCRELER (yeşil zemin plakası) ─
    int cell_id = 0;
    for (auto& [gp, cell] : grid_map_)
    {
      if (!cell.visited) continue;
      auto m = make_marker("cells", cell_id++,
                           visualization_msgs::msg::Marker::CUBE,
                           0.2, 0.75, 0.2, 0.3);
      m.header.stamp      = now;
      m.pose.position     = grid_to_world(gp.x, gp.y, 0.0);
      m.scale.x = CELL_VIZ_SIZE;
      m.scale.y = CELL_VIZ_SIZE;
      m.scale.z = 0.04;
      arr.markers.push_back(m);
    }

    // ── 2. DUVARLAR (kırmızı paneller) ────────────────────
    int wall_id = 0;
    for (auto& [gp, cell] : grid_map_)
    {
      double wx   = gp.x * CELL_SIZE;
      double wy   = gp.y * CELL_SIZE;
      double half = CELL_SIZE / 2.0;

      // {duvar var mı, offset_x, offset_y, scale_x, scale_y}
      struct WD { bool on; double ox, oy, sx, sy; };
      std::vector<WD> ws = {
        { cell.wall_north,  0,    +half, CELL_SIZE,    WALL_THICKNESS },
        { cell.wall_south,  0,    -half, CELL_SIZE,    WALL_THICKNESS },
        { cell.wall_east,  +half,  0,   WALL_THICKNESS, CELL_SIZE    },
        { cell.wall_west,  -half,  0,   WALL_THICKNESS, CELL_SIZE    },
      };

      for (auto& w : ws) {
        if (!w.on) continue;
        auto m = make_marker("walls", wall_id++,
                             visualization_msgs::msg::Marker::CUBE,
                             0.85, 0.1, 0.1, 0.9);
        m.header.stamp        = now;
        m.pose.position.x     = wx + w.ox;
        m.pose.position.y     = wy + w.oy;
        m.pose.position.z     = WALL_HEIGHT / 2.0;
        m.scale.x = w.sx;
        m.scale.y = w.sy;
        m.scale.z = WALL_HEIGHT;
        arr.markers.push_back(m);
      }
    }

    // ── 3. ROBOT KONUMU (mavi küp + yazı) ─────────────────
    {
      auto m = make_marker("robot", 0,
                           visualization_msgs::msg::Marker::CUBE,
                           0.1, 0.4, 1.0, 0.95);
      m.header.stamp      = now;
      m.pose.position     = grid_to_world(current_grid_.x, current_grid_.y, 0.2);
      m.scale.x = 0.55; m.scale.y = 0.55; m.scale.z = 0.4;
      arr.markers.push_back(m);

      auto txt = make_marker("robot", 1,
                             visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
                             1.0, 1.0, 1.0, 1.0);
      txt.header.stamp    = now;
      txt.pose.position   = grid_to_world(current_grid_.x, current_grid_.y, 0.7);
      txt.scale.z         = 0.28;
      txt.text = "R(" + std::to_string(current_grid_.x) +
                 "," + std::to_string(current_grid_.y) + ")";
      arr.markers.push_back(txt);
    }

    // ── 4. HEDEF (sarı silindir + yazı) ───────────────────
    if (has_user_goal_)
    {
      auto m = make_marker("goal", 0,
                           visualization_msgs::msg::Marker::CYLINDER,
                           1.0, 0.85, 0.0, 0.9);
      m.header.stamp      = now;
      m.pose.position     = grid_to_world(user_goal_.x, user_goal_.y, 0.05);
      m.scale.x = 1.1; m.scale.y = 1.1; m.scale.z = 0.1;
      arr.markers.push_back(m);

      auto txt = make_marker("goal", 1,
                             visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
                             1.0, 1.0, 0.0, 1.0);
      txt.header.stamp    = now;
      txt.pose.position   = grid_to_world(user_goal_.x, user_goal_.y, 0.55);
      txt.scale.z         = 0.28;
      txt.text = "HEDEF(" + std::to_string(user_goal_.x) +
                 "," + std::to_string(user_goal_.y) + ")";
      arr.markers.push_back(txt);
    }

    // ── 5. BFS ROTASI (turuncu çizgi + adım numaraları) ───
    if (planned_path_.size() > 1 && path_index_ < planned_path_.size())
    {
      auto line = make_marker("path", 0,
                              visualization_msgs::msg::Marker::LINE_STRIP,
                              1.0, 0.5, 0.0, 1.0);
      line.header.stamp = now;
      line.scale.x      = 0.12;

      for (size_t i = path_index_; i < planned_path_.size(); ++i) {
        geometry_msgs::msg::Point p;
        p.x = planned_path_[i].x * CELL_SIZE;
        p.y = planned_path_[i].y * CELL_SIZE;
        p.z = 0.25;
        line.points.push_back(p);
      }
      arr.markers.push_back(line);

      // Adım numaraları
      for (size_t i = path_index_; i < planned_path_.size(); ++i) {
        auto txt = make_marker("path", static_cast<int>(i) + 1,
                               visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
                               1.0, 0.75, 0.1, 1.0);
        txt.header.stamp  = now;
        txt.pose.position = grid_to_world(planned_path_[i].x, planned_path_[i].y, 0.45);
        txt.scale.z       = 0.2;
        txt.text          = std::to_string(i - path_index_ + 1);
        arr.markers.push_back(txt);
      }
    }

    viz_pub_->publish(arr);
  }

  // ════════════════════════════════════════════════════════
  //  YARDIMCILAR
  // ════════════════════════════════════════════════════════

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

  double normalize_angle(double a) {
    while (a >  180.0) a -= 360.0;
    while (a < -180.0) a += 360.0;
    return a;
  }

  GridPoint get_global_target(int local_dir, int orientation)
  {
    int g = ((orientation + local_dir * 90) % 360 + 360) % 360;
    GridPoint p = current_grid_;
    if      (g ==   0) p.x += 1;
    else if (g ==  90) p.y += 1;
    else if (g == 180) p.x -= 1;
    else if (g == 270) p.y -= 1;
    return p;
  }

  void map_wall_info(GridPoint p, int global_angle, bool is_wall)
  {
    int a = ((global_angle % 360) + 360) % 360;
    if      (a ==   0) grid_map_[p].wall_east  = is_wall;
    else if (a ==  90) grid_map_[p].wall_north = is_wall;
    else if (a == 180) grid_map_[p].wall_west  = is_wall;
    else if (a == 270) grid_map_[p].wall_south = is_wall;
  }

  double compute_target_yaw(GridPoint target)
  {
    double dx = target.x - current_grid_.x;
    double dy = target.y - current_grid_.y;
    double ideal    = std::atan2(dy, dx) * 180.0 / M_PI;
    double oriented = std::round(current_yaw_deg_ / 90.0) * 90.0;
    return normalize_angle(current_yaw_deg_ + normalize_angle(ideal - oriented));
  }

  void send_motion_cmd(GridPoint tgt, double yaw_deg)
  {
    geometry_msgs::msg::Pose2D cmd;
    cmd.x     = static_cast<double>(tgt.x);
    cmd.y     = static_cast<double>(tgt.y);
    cmd.theta = yaw_deg;
    motion_cmd_pub_->publish(cmd);
    motion_done_   = false;
    motion_failed_ = false;
    current_state_ = State::WAITING_MOTION;
    RCLCPP_INFO(this->get_logger(),
      "→ Motion CMD: Grid(%d,%d) yaw=%.1f°", tgt.x, tgt.y, yaw_deg);
  }

  std::vector<GridPoint> run_bfs(GridPoint start, GridPoint goal)
  {
    std::queue<GridPoint> q;
    std::map<GridPoint, GridPoint> parent;
    std::set<GridPoint> visited;
    q.push(start); visited.insert(start);
    while (!q.empty()) {
      auto cur = q.front(); q.pop();
      if (cur == goal) break;
      const auto& cell = grid_map_[cur];
      std::vector<std::pair<GridPoint,bool>> nb = {
        {{cur.x+1, cur.y}, !cell.wall_east},
        {{cur.x-1, cur.y}, !cell.wall_west},
        {{cur.x, cur.y+1}, !cell.wall_north},
        {{cur.x, cur.y-1}, !cell.wall_south}
      };
      for (auto& n : nb)
        if (n.second && !visited.count(n.first) && grid_map_.count(n.first)) {
          visited.insert(n.first);
          parent[n.first] = cur;
          q.push(n.first);
        }
    }
    std::vector<GridPoint> path;
    GridPoint cur = goal;
    if (!parent.count(cur) && !(cur == start)) return path;
    while (!(cur == start)) { path.push_back(cur); cur = parent[cur]; }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
  }

  // ── Callback'ler ─────────────────────────────────────────

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    scan_received_ = true;
    front_dist_    = getAverageRange(msg, 0.0);
    left_dist_     = getAverageRange(msg, M_PI_2);
    right_dist_    = getAverageRange(msg, -M_PI_2);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_received_ = true;
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double r, p, yaw;
    tf2::Matrix3x3(q).getRPY(r, p, yaw);
    current_yaw_deg_ = yaw * 180.0 / M_PI;
  }

  void motion_status_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if      (msg->data == "DONE")   motion_done_   = true;
    else if (msg->data == "FAILED") motion_failed_ = true;
  }

  // ════════════════════════════════════════════════════════
  //  KARAR DÖNGÜSÜ
  // ════════════════════════════════════════════════════════

  void decision_loop()
  {
    if (!scan_received_ || !odom_received_) return;

    switch (current_state_)
    {
    case State::IDLE:
      current_state_ = State::SCANNING;
      break;

    case State::SCANNING:
    {
      int ori = ((int)std::round(current_yaw_deg_ / 90.0) * 90 % 360 + 360) % 360;

      grid_map_[current_grid_].visited = true;
      map_wall_info(current_grid_, ori,       front_dist_ <= WALL_THRESHOLD);
      map_wall_info(current_grid_, ori + 90,  left_dist_  <= WALL_THRESHOLD);
      map_wall_info(current_grid_, ori - 90,  right_dist_ <= WALL_THRESHOLD);

      GridPoint p_right = get_global_target(-1, ori);
      GridPoint p_front = get_global_target( 0, ori);
      GridPoint p_left  = get_global_target( 1, ori);

      bool found = false;
      GridPoint next;
      double turn = 0.0;

      if      (right_dist_ > WALL_THRESHOLD && !grid_map_[p_right].visited)
        { next = p_right; turn = -90.0; found = true; }
      else if (front_dist_ > WALL_THRESHOLD && !grid_map_[p_front].visited)
        { next = p_front; turn =   0.0; found = true; }
      else if (left_dist_  > WALL_THRESHOLD && !grid_map_[p_left].visited)
        { next = p_left;  turn =  90.0; found = true; }

      if (found) {
        path_stack_.push(current_grid_);
        grid_map_[next].visited = true;
        target_grid_ = next;
        send_motion_cmd(target_grid_, normalize_angle(current_yaw_deg_ + turn));
      }
      else if (!path_stack_.empty()) {
        target_grid_ = path_stack_.top();
        path_stack_.pop();
        send_motion_cmd(target_grid_, compute_target_yaw(target_grid_));
      }
      else {
        if (has_user_goal_) {
          RCLCPP_INFO(this->get_logger(), "Keşif bitti → PLANNING.");
          current_state_ = State::PLANNING;
        } else {
          RCLCPP_INFO(this->get_logger(), "Keşif tamamlandı → IDLE.");
          current_state_ = State::IDLE;
        }
      }
    }
    break;

    case State::WAITING_MOTION:
    {
      if (motion_done_) {
        current_grid_ = target_grid_;
        motion_done_  = false;
        if (!planned_path_.empty() && path_index_ < planned_path_.size())
          { path_index_++; current_state_ = State::FOLLOW_PATH; }
        else
          current_state_ = State::SCANNING;
      }
      else if (motion_failed_) {
        motion_failed_ = false;
        RCLCPP_WARN(this->get_logger(), "Hareket başarısız! Taramaya dönülüyor.");
        current_state_ = State::SCANNING;
      }
    }
    break;

    case State::PLANNING:
    {
      if (!has_user_goal_) { current_state_ = State::IDLE; break; }
      planned_path_ = run_bfs(current_grid_, user_goal_);
      if (planned_path_.size() > 1) {
        path_index_ = 1;
        RCLCPP_INFO(this->get_logger(), "BFS hazır. %ld adım.", planned_path_.size()-1);
        current_state_ = State::FOLLOW_PATH;
        publish_viz();  // rota çizilince anında güncelle
      } else {
        RCLCPP_WARN(this->get_logger(), "BFS: yol bulunamadı.");
        has_user_goal_ = false;
        planned_path_.clear();
        current_state_ = State::IDLE;
      }
    }
    break;

    case State::FOLLOW_PATH:
    {
      if (path_index_ >= planned_path_.size()) {
        RCLCPP_INFO(this->get_logger(), "Hedefe ulaşıldı!");
        has_user_goal_ = false;
        planned_path_.clear();
        current_state_ = State::IDLE;
        publish_viz();
        break;
      }
      target_grid_ = planned_path_[path_index_];
      send_motion_cmd(target_grid_, compute_target_yaw(target_grid_));
    }
    break;

    default: break;
    }
  }
};

// ── main ─────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MazeDecisionNode>());
  rclcpp::shutdown();
  return 0;
}