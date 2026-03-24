#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <cmath>
#include <limits>

struct Point2D { double x, y; };
struct Pose2D  { double x, y, yaw; };

std::vector<Point2D> scanToPoints(const sensor_msgs::msg::LaserScan & scan, const Pose2D & pose)
{
    std::vector<Point2D> pts;
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float r = scan.ranges[i];
        if (r < scan.range_min || r > scan.range_max) continue;
        float angle = scan.angle_min + i * scan.angle_increment + pose.yaw;
        float wx = pose.x + r * std::cos(angle);
        float wy = pose.y + r * std::sin(angle);
        pts.push_back({wx, wy});
    }
    return pts;
}

Pose2D icp(const std::vector<Point2D> & source, const std::vector<Point2D> & target)
{
    Pose2D delta{0.0, 0.0, 0.0};
    if (source.empty() || target.empty()) return delta;

    std::vector<Point2D> src = source;

    for (int iter = 0; iter < 50; ++iter) {

        // Adaptive eşik: ilk iterasyonda 1.0m, son iterasyonda 0.5m
        double thresh = 1.0 - iter * (0.5 / 50.0);
        double best_sq = thresh * thresh;

        std::vector<std::pair<Point2D,Point2D>> pairs;
        for (auto & s : src) {
            double best = best_sq;
            Point2D closest{};
            bool found = false;
            for (auto & t : target) {
                double d = (s.x-t.x)*(s.x-t.x) + (s.y-t.y)*(s.y-t.y);
                if (d < best) { best = d; closest = t; found = true; }
            }
            if (found) pairs.push_back({s, closest});
        }

        if (pairs.empty()) break;

        Point2D cs{0,0}, ct{0,0};
        for (auto & [s,t] : pairs) { cs.x+=s.x; cs.y+=s.y; ct.x+=t.x; ct.y+=t.y; }
        cs.x /= pairs.size(); cs.y /= pairs.size();
        ct.x /= pairs.size(); ct.y /= pairs.size();

        double sxy=0, sxx=0;
        for (auto & [s,t] : pairs) {
            sxy += (s.x-cs.x)*(t.y-ct.y) - (s.y-cs.y)*(t.x-ct.x);
            sxx += (s.x-cs.x)*(t.x-ct.x) + (s.y-cs.y)*(t.y-ct.y);
        }
        double d_yaw = std::atan2(sxy, sxx);
        double d_x   = ct.x - (cs.x*std::cos(d_yaw) - cs.y*std::sin(d_yaw));
        double d_y   = ct.y - (cs.x*std::sin(d_yaw) + cs.y*std::cos(d_yaw));

        for (auto & p : src) {
            double nx = p.x*std::cos(d_yaw) - p.y*std::sin(d_yaw) + d_x;
            double ny = p.x*std::sin(d_yaw) + p.y*std::cos(d_yaw) + d_y;
            p.x = nx; p.y = ny;
        }
        delta.x   += d_x;
        delta.y   += d_y;
        delta.yaw += d_yaw;
        if (std::hypot(d_x, d_y) < 1e-4 && std::abs(d_yaw) < 1e-4) break;
    }
    return delta;
}

class Occupncy : public rclcpp::Node
{
public:
    Occupncy() : Node("occupancy_mapper")
    {
        using std::placeholders::_1;

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/a200_0000/sensors/lidar2d_0/scan", 10,
            std::bind(&Occupncy::lidar_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/a200_0000/platform/odom/filtered", 10,
            std::bind(&Occupncy::odom_callback, this, _1));

        map_pub_  = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_pose", 10);

        init_map();
    }

private:

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto & q = msg->pose.pose.orientation;
        odom_pose_.x   = msg->pose.pose.position.x;
        odom_pose_.y   = msg->pose.pose.position.y;
        odom_pose_.yaw = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!prev_scan_pts_.empty()) {

            // 1. Odom'dan delta hesapla
            double dx   = odom_pose_.x   - prev_odom_pose_.x;
            double dy   = odom_pose_.y   - prev_odom_pose_.y;
            double dyaw = odom_pose_.yaw - prev_odom_pose_.yaw;

            // yaw farkını [-pi, pi] aralığına çek
            while (dyaw >  M_PI) dyaw -= 2.0 * M_PI;
            while (dyaw < -M_PI) dyaw += 2.0 * M_PI;

            // 2. Odom delta'sı ile initial guess oluştur
            Pose2D guess;
            guess.x   = corrected_.x   + dx;
            guess.y   = corrected_.y   + dy;
            guess.yaw = corrected_.yaw + dyaw;

            // 3. ICP'yi guess pose'undan başlat
            auto pts_from_guess = scanToPoints(*msg, guess);
            Pose2D delta = icp(pts_from_guess, prev_scan_pts_);

            // 4. ICP düzeltmesini guess üzerine uygula
            corrected_.x   = guess.x   + delta.x;
            corrected_.y   = guess.y   + delta.y;
            corrected_.yaw = guess.yaw + delta.yaw;

            RCLCPP_INFO(this->get_logger(),
                "pos: x=%.3f  y=%.3f  yaw=%.3f",
                corrected_.x, corrected_.y, corrected_.yaw);

            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp    = this->now();
            pose_msg.header.frame_id = "odom";
            pose_msg.pose.position.x = corrected_.x;
            pose_msg.pose.position.y = corrected_.y;
            pose_msg.pose.position.z = 0.0;
            pose_msg.pose.orientation.z = std::sin(corrected_.yaw / 2.0);
            pose_msg.pose.orientation.w = std::cos(corrected_.yaw / 2.0);
            pose_pub_->publish(pose_msg);
        }

        prev_odom_pose_ = odom_pose_;

        auto current_pts = scanToPoints(*msg, corrected_);
        prev_scan_pts_ = current_pts;

        map_.header.stamp = this->now();
        for (auto & pt : current_pts)
        {
            int gx = worldToGridX(pt.x);
            int gy = worldToGridY(pt.y);
            if (gx < 0 || gx >= static_cast<int>(map_.info.width) ||
                gy < 0 || gy >= static_cast<int>(map_.info.height)) continue;
            map_.data[gy * map_.info.width + gx] = 100;
        }
        map_pub_->publish(map_);
    }

    int worldToGridX(float wx)
    {
        return static_cast<int>((wx - map_.info.origin.position.x) / map_.info.resolution);
    }

    int worldToGridY(float wy)
    {
        return static_cast<int>((wy - map_.info.origin.position.y) / map_.info.resolution);
    }

    void init_map()
    {
        map_.header.frame_id = "odom";
        map_.info.resolution = 0.1;
        map_.info.width      = 200;
        map_.info.height     = 200;
        map_.info.origin.position.x = -10.0;
        map_.info.origin.position.y = -10.0;
        map_.info.origin.orientation.w = 1.0;
        map_.data.resize(map_.info.width * map_.info.height, -1);
    }

    Pose2D corrected_     {0.0, 0.0, 0.0};
    Pose2D odom_pose_     {0.0, 0.0, 0.0};
    Pose2D prev_odom_pose_{0.0, 0.0, 0.0};

    std::vector<Point2D> prev_scan_pts_;
    nav_msgs::msg::OccupancyGrid map_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr    map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Occupncy>());
    rclcpp::shutdown();
    return 0;
}