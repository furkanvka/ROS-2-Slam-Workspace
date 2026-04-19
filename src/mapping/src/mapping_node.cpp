#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <cmath>
#include <functional>

struct Pose2D { double x, y, yaw; };

class MappingNode : public rclcpp::Node
{
public:
    MappingNode() : Node("mapping_node")
    {
        using std::placeholders::_1;

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/a200_0000/sensors/lidar2d_0/scan", 10,
            std::bind(&MappingNode::scanCb, this, _1));

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/corrected_pose", 10,
            std::bind(&MappingNode::poseCb, this, _1));

        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

        initMap();
    }

private:
    Pose2D pose_{};
    Pose2D prev_pose_{};
    bool   pose_ready_ = false;

    std::vector<float>           log_odds_;
    nav_msgs::msg::OccupancyGrid map_;

    // ── Log-odds parametreleri ────────────────────────────────────────────
    static constexpr float  LOG_ODDS_HIT  =  0.9f;
    static constexpr float  LOG_ODDS_FREE = -0.4f;
    static constexpr float  LOG_ODDS_MIN  = -2.0f;
    static constexpr float  LOG_ODDS_MAX  =  5.0f;
    static constexpr double RAY_SIGMA     =  2.0;

    inline int8_t toOccupancy(float lo)
    {
        if (lo >  0.5f) return 100;
        if (lo < -0.5f) return 0;
        return -1;
    }

    inline void updateFree(int idx, double ray_len)
    {
        double weight  = std::exp(-ray_len * ray_len / (2.0 * RAY_SIGMA * RAY_SIGMA));
        float  delta   = static_cast<float>(LOG_ODDS_FREE * weight);
        log_odds_[idx] = std::max(LOG_ODDS_MIN, log_odds_[idx] + delta);
    }

    inline void updateHit(int idx)
    {
        log_odds_[idx] = std::min(LOG_ODDS_MAX, log_odds_[idx] + LOG_ODDS_HIT);
    }

    void bresenham(int x0, int y0, int x1, int y1,
                   std::function<void(int, int)> visit)
    {
        int dx = std::abs(x1-x0), sx = x0 < x1 ? 1 : -1;
        int dy = std::abs(y1-y0), sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        while (true) {
            visit(x0, y0);
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 <  dx) { err += dx; y0 += sy; }
        }
    }

    void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        pose_.x   = msg->pose.position.x;
        pose_.y   = msg->pose.position.y;
        double z  = msg->pose.orientation.z;
        double w  = msg->pose.orientation.w;
        pose_.yaw = std::atan2(2.0*z*w, 1.0 - 2.0*z*z);

        if (!pose_ready_) {
            prev_pose_  = pose_;
            pose_ready_ = true;
        }
    }

    void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!pose_ready_) return;

        // FIX: hareketsizken sadece stamp güncelle, map işlemi yapma
        double dx   = pose_.x   - prev_pose_.x;
        double dy   = pose_.y   - prev_pose_.y;
        double dyaw = pose_.yaw - prev_pose_.yaw;
        while (dyaw >  M_PI) dyaw -= 2.0 * M_PI;
        while (dyaw < -M_PI) dyaw += 2.0 * M_PI;

        if (std::hypot(dx, dy) < 0.01 && std::abs(dyaw) < 0.005) {
            map_.header.stamp = now();
            map_pub_->publish(map_);
            return;
        }

        prev_pose_ = pose_;

        int W  = static_cast<int>(map_.info.width);
        int H  = static_cast<int>(map_.info.height);
        int rx = worldToGridX(pose_.x);
        int ry = worldToGridY(pose_.y);

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float r = msg->ranges[i];
            if (r < msg->range_min || r > msg->range_max) continue;

            float  a  = msg->angle_min + i * msg->angle_increment + pose_.yaw;
            double wx = pose_.x + r * std::cos(a);
            double wy = pose_.y + r * std::sin(a);

            int gx = worldToGridX(wx);
            int gy = worldToGridY(wy);

            bresenham(rx, ry, gx, gy, [&](int cx, int cy) {
                if (cx < 0 || cx >= W || cy < 0 || cy >= H) return;
                if (cx == gx && cy == gy) return;
                double cell_wx = map_.info.origin.position.x + (cx + 0.5) * map_.info.resolution;
                double cell_wy = map_.info.origin.position.y + (cy + 0.5) * map_.info.resolution;
                double ray_len = std::hypot(cell_wx - pose_.x, cell_wy - pose_.y);
                int idx = cy * W + cx;
                updateFree(idx, ray_len);
                map_.data[idx] = toOccupancy(log_odds_[idx]);
            });

            if (gx < 0 || gx >= W || gy < 0 || gy >= H) continue;
            int idx = gy * W + gx;
            updateHit(idx);
            map_.data[idx] = toOccupancy(log_odds_[idx]);
        }

        map_.header.stamp = now();
        map_pub_->publish(map_);
    }

    int worldToGridX(double x) {
        return static_cast<int>((x - map_.info.origin.position.x) / map_.info.resolution);
    }
    int worldToGridY(double y) {
        return static_cast<int>((y - map_.info.origin.position.y) / map_.info.resolution);
    }

    void initMap()
    {
        map_.header.frame_id = "odom";
        map_.info.resolution = 0.05;
        map_.info.width      = 400;
        map_.info.height     = 400;
        map_.info.origin.position.x = -10.0;
        map_.info.origin.position.y = -10.0;
        map_.info.origin.orientation.w = 1.0;
        int N = 400 * 400;
        map_.data.resize(N, -1);
        log_odds_.resize(N, 0.0f);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr     scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr        map_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MappingNode>());
    rclcpp::shutdown();
}