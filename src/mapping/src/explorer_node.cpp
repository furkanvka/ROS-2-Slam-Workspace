#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <vector>
#include <cmath>
#include <queue>
#include <unordered_set>

struct Pose2D { double x, y, yaw; };

class ExplorerNode : public rclcpp::Node
{
public:
    ExplorerNode() : Node("explorer_node")
    {
        using std::placeholders::_1;

        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10,
            std::bind(&ExplorerNode::mapCb, this, _1));

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/corrected_pose", 10,
            std::bind(&ExplorerNode::poseCb, this, _1));

        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/a200_0000/cmd_vel", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ExplorerNode::controlLoop, this));
    }

private:
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    Pose2D pose_{};

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        pose_.x = msg->pose.position.x;
        pose_.y = msg->pose.position.y;
        double z = msg->pose.orientation.z;
        double w = msg->pose.orientation.w;
        pose_.yaw = std::atan2(2.0 * z * w, 1.0 - 2.0 * z * z);
    }

    void mapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_ = msg;
    }

   

    void controlLoop()
    {
    }

    int worldToGridX(double x) {
        if (!map_) return 0;
        return static_cast<int>((x - map_->info.origin.position.x) / map_->info.resolution);
    }

    int worldToGridY(double y) {
        if (!map_) return 0;
        return static_cast<int>((y - map_->info.origin.position.y) / map_->info.resolution);
    }

    double gridToWorldX(int gx) {
        if (!map_) return 0.0;
        return map_->info.origin.position.x + (gx + 0.5) * map_->info.resolution;
    }

    double gridToWorldY(int gy) {
        if (!map_) return 0.0;
        return map_->info.origin.position.y + (gy + 0.5) * map_->info.resolution;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExplorerNode>());
    rclcpp::shutdown();
}
