#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

using std::placeholders::_1;

class GoalBridgeNode : public rclcpp::Node
{
public:
  GoalBridgeNode() : Node("goal_bridge_node")
  {
    // ── Subscriber: RViz'den gelen tıklama ──────────────
    clicked_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10,
      std::bind(&GoalBridgeNode::clicked_callback, this, _1));

    // ── Publishers ──────────────────────────────────────
    goal_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
      "/maze_goal", 10);

    preview_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/maze_viz_goal_preview", 10);

    RCLCPP_INFO(this->get_logger(),
      "GoalBridgeNode başlatıldı.");
    RCLCPP_INFO(this->get_logger(),
      "RViz → 'Publish Point' butonuna bas, haritada bir noktaya tıkla.");
    RCLCPP_INFO(this->get_logger(),
      "Hücre boyutu: %.2f m  |  Orijin: (0,0) grid = (0.0, 0.0) world", CELL_SIZE);
  }

private:
  const double CELL_SIZE = 2.05;  // maze_decision_node ile aynı olmalı

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr            goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr      preview_pub_;

  // ── Dünya → Grid dönüşümü ────────────────────────────
  // round() kullanılıyor: tıklanan nokta hangi hücrenin
  // merkezine daha yakınsa oraya snap'leniyor.
  int world_to_grid(double world) const
  {
    return static_cast<int>(std::round(world / CELL_SIZE));
  }

  // ── Onay görseli: yeşil ok + yazı ───────────────────
  void publish_preview(double wx, double wy, int gx, int gy)
  {
    auto now = this->now();

    // Aşağı bakan ok (dünya koordinatı)
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id    = "odom";
    arrow.header.stamp       = now;
    arrow.ns                 = "goal_preview";
    arrow.id                 = 0;
    arrow.type               = visualization_msgs::msg::Marker::ARROW;
    arrow.action             = visualization_msgs::msg::Marker::ADD;
    arrow.pose.position.x    = gx * CELL_SIZE;   // grid merkezine snap
    arrow.pose.position.y    = gy * CELL_SIZE;
    arrow.pose.position.z    = 1.5;
    // Quaternion: z ekseni boyunca aşağı bakan ok
    arrow.pose.orientation.x =  0.7071;
    arrow.pose.orientation.y =  0.0;
    arrow.pose.orientation.z =  0.0;
    arrow.pose.orientation.w =  0.7071;
    arrow.scale.x = 1.0;
    arrow.scale.y = 0.12;
    arrow.scale.z = 0.12;
    arrow.color.r = 0.0;
    arrow.color.g = 0.9;
    arrow.color.b = 0.2;
    arrow.color.a = 1.0;
    arrow.lifetime = rclcpp::Duration(5, 0);  // 5 saniye görünür
    preview_pub_->publish(arrow);

    // Yazı: tıklanan dünya koordinatı + hesaplanan grid
    visualization_msgs::msg::Marker txt;
    txt.header.frame_id  = "odom";
    txt.header.stamp     = now;
    txt.ns               = "goal_preview";
    txt.id               = 1;
    txt.type             = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    txt.action           = visualization_msgs::msg::Marker::ADD;
    txt.pose.position.x  = gx * CELL_SIZE;
    txt.pose.position.y  = gy * CELL_SIZE;
    txt.pose.position.z  = 2.2;
    txt.pose.orientation.w = 1.0;
    txt.scale.z          = 0.3;
    txt.color.r = 0.2; txt.color.g = 1.0; txt.color.b = 0.2; txt.color.a = 1.0;
    txt.lifetime         = rclcpp::Duration(5, 0);

    // İki satır: world koordinatı + grid koordinatı
    char buf[128];
    std::snprintf(buf, sizeof(buf),
      "world:(%.1f, %.1f)\ngrid:(%d, %d)",
      wx, wy, gx, gy);
    txt.text = buf;
    preview_pub_->publish(txt);
  }

  // ── Ana callback ─────────────────────────────────────
  void clicked_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    double wx = msg->point.x;
    double wy = msg->point.y;

    int gx = world_to_grid(wx);
    int gy = world_to_grid(wy);

    RCLCPP_INFO(this->get_logger(),
      "Tıklama → World:(%.2f, %.2f)  →  Grid:(%d, %d)",
      wx, wy, gx, gy);

    // /maze_goal yayınla
    geometry_msgs::msg::Point goal_msg;
    goal_msg.x = static_cast<double>(gx);
    goal_msg.y = static_cast<double>(gy);
    goal_msg.z = 0.0;
    goal_pub_->publish(goal_msg);

    // RViz'de onay görseli göster
    publish_preview(wx, wy, gx, gy);

    RCLCPP_INFO(this->get_logger(),
      "✓ /maze_goal → Grid(%d, %d) gönderildi.", gx, gy);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
