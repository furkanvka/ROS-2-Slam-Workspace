#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <cmath>
#include <functional>
#include <algorithm>
#include <numeric>
#include <limits>

struct Point2D { double x, y; };
struct Pose2D  { double x, y, yaw; };

// ─── Kd-Tree ────────────────────────────────────────────────────────────────

struct KdNode {
    Point2D point;
    int     left  = -1;
    int     right = -1;
};

class KdTree
{
public:
    void build(const std::vector<Point2D> & pts)
    {
        nodes_.clear();
        if (pts.empty()) return;
        std::vector<int> idx(pts.size());
        std::iota(idx.begin(), idx.end(), 0);
        pts_ = &pts;
        build_recursive(idx, 0);
    }

    bool nearest(const Point2D & q, double thresh_sq, Point2D & out) const
    {
        double best_dist = thresh_sq;
        search(0, q, 0, out, best_dist);
        return best_dist < thresh_sq;
    }

    bool empty() const { return nodes_.empty(); }

private:
    const std::vector<Point2D> * pts_ = nullptr;
    std::vector<KdNode>          nodes_;

    int build_recursive(std::vector<int> & idx, int depth)
    {
        if (idx.empty()) return -1;

        int    axis = depth % 2;
        size_t mid  = idx.size() / 2;
        std::nth_element(idx.begin(), idx.begin() + mid, idx.end(),
            [&](int a, int b) {
                return axis == 0 ? (*pts_)[a].x < (*pts_)[b].x
                                 : (*pts_)[a].y < (*pts_)[b].y;
            });

        KdNode node;
        node.point = (*pts_)[idx[mid]];
        int node_idx = static_cast<int>(nodes_.size());
        nodes_.push_back(node);

        std::vector<int> left_idx (idx.begin(),            idx.begin() + mid);
        std::vector<int> right_idx(idx.begin() + mid + 1,  idx.end());

        nodes_[node_idx].left  = build_recursive(left_idx,  depth + 1);
        nodes_[node_idx].right = build_recursive(right_idx, depth + 1);

        return node_idx;
    }

    void search(int node_idx, const Point2D & q, int depth,
                Point2D & best, double & best_dist) const
    {
        if (node_idx < 0 || node_idx >= static_cast<int>(nodes_.size())) return;

        const KdNode & node = nodes_[node_idx];
        int axis = depth % 2;

        double dx   = q.x - node.point.x;
        double dy   = q.y - node.point.y;
        double dist = dx*dx + dy*dy;

        if (dist < best_dist) { best_dist = dist; best = node.point; }

        double diff     = (axis == 0) ? dx : dy;
        int    near_idx = (diff < 0) ? node.left  : node.right;
        int    far_idx  = (diff < 0) ? node.right : node.left;

        search(near_idx, q, depth + 1, best, best_dist);
        if (diff * diff < best_dist)
            search(far_idx, q, depth + 1, best, best_dist);
    }
};

// ─── Scan → World Points ────────────────────────────────────────────────────

std::vector<Point2D> scanToPoints(const sensor_msgs::msg::LaserScan & scan,
                                   const Pose2D & pose)
{
    std::vector<Point2D> pts;
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float r = scan.ranges[i];
        if (r < scan.range_min || r > scan.range_max) continue;
        float angle = scan.angle_min + i * scan.angle_increment + pose.yaw;
        pts.push_back({ pose.x + r * std::cos(angle),
                        pose.y + r * std::sin(angle) });
    }
    return pts;
}

// ─── ICP ────────────────────────────────────────────────────────────────────

Pose2D icp(const std::vector<Point2D> & source, const KdTree & tree)
{
    Pose2D delta{0.0, 0.0, 0.0};
    if (source.empty() || tree.empty()) return delta;

    std::vector<Point2D> src = source;

    for (int iter = 0; iter < 50; ++iter) {

        double thresh    = 1.0 - iter * (0.5 / 50.0);
        double thresh_sq = thresh * thresh;

        std::vector<std::pair<Point2D,Point2D>> pairs;
        pairs.reserve(src.size());
        for (auto & s : src) {
            Point2D closest;
            if (tree.nearest(s, thresh_sq, closest))
                pairs.push_back({s, closest});
        }
        if (pairs.empty()) break;

        Point2D cs{0,0}, ct{0,0};
        for (auto & [s,t] : pairs) { cs.x+=s.x; cs.y+=s.y; ct.x+=t.x; ct.y+=t.y; }
        double n = static_cast<double>(pairs.size());
        cs.x/=n; cs.y/=n; ct.x/=n; ct.y/=n;

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

        delta.x  += d_x;
        delta.y  += d_y;
        delta.yaw = std::atan2(std::sin(delta.yaw + d_yaw),
                               std::cos(delta.yaw + d_yaw));

        if (std::hypot(d_x, d_y) < 1e-4 && std::abs(d_yaw) < 1e-4) break;
    }
    return delta;
}

// ─── Node ───────────────────────────────────────────────────────────────────

class OccupancyMapper : public rclcpp::Node
{
public:
    OccupancyMapper() : Node("occupancy_mapper")
    {
        using std::placeholders::_1;

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/a200_0000/sensors/lidar2d_0/scan", 10,
            std::bind(&OccupancyMapper::lidar_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/a200_0000/platform/odom/filtered", 10,
            std::bind(&OccupancyMapper::odom_callback, this, _1));

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
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z));

        if (!odom_initialized_) {
            corrected_        = odom_pose_;
            prev_odom_pose_   = odom_pose_;
            odom_initialized_ = true;
        }
    }

    void bresenham(int x0, int y0, int x1, int y1,
                   std::function<void(int,int)> visit)
    {
        int dx = std::abs(x1-x0), sx = x0<x1 ? 1 : -1;
        int dy = std::abs(y1-y0), sy = y0<y1 ? 1 : -1;
        int err = dx - dy;
        while (true) {
            visit(x0, y0);
            if (x0==x1 && y0==y1) break;
            int e2 = 2*err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 <  dx) { err += dx; y0 += sy; }
        }
    }

    bool isFarEnough(const Point2D & p)
    {
        for (auto & m : map_pts_)
            if ((p.x-m.x)*(p.x-m.x)+(p.y-m.y)*(p.y-m.y) < MAP_PTS_MIN_DIST_SQ)
                return false;
        return true;
    }

    // Mesafeye göre ağırlıklı free güncelleme — uzak raylar duvarı silemez
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

    inline int8_t toOccupancy(float lo)
    {
        if (lo >  0.5f) return 100;
        if (lo < -0.5f) return 0;
        return -1;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!odom_initialized_) return;

        double dx   = odom_pose_.x   - prev_odom_pose_.x;
        double dy   = odom_pose_.y   - prev_odom_pose_.y;
        double dyaw = odom_pose_.yaw - prev_odom_pose_.yaw;
        while (dyaw >  M_PI) dyaw -= 2.0*M_PI;
        while (dyaw < -M_PI) dyaw += 2.0*M_PI;

        if (map_pts_.size() >= MIN_MAP_PTS) {

            if (std::hypot(dx, dy) < 0.01 && std::abs(dyaw) < 0.005) {
                map_.header.stamp = this->now();
                map_pub_->publish(map_);
                return;
            }

            Pose2D guess;
            guess.x   = corrected_.x   + dx;
            guess.y   = corrected_.y   + dy;
            guess.yaw = corrected_.yaw + dyaw;

            auto pts_from_guess = scanToPoints(*msg, guess);
            Pose2D delta = icp(pts_from_guess, map_tree_);

            corrected_.x   = guess.x + delta.x;
            corrected_.y   = guess.y + delta.y;
            corrected_.yaw = std::atan2(
                std::sin(guess.yaw + delta.yaw),
                std::cos(guess.yaw + delta.yaw));

            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp    = this->now();
            pose_msg.header.frame_id = "odom";
            pose_msg.pose.position.x = corrected_.x;
            pose_msg.pose.position.y = corrected_.y;
            pose_msg.pose.position.z = 0.0;
            pose_msg.pose.orientation.z = std::sin(corrected_.yaw / 2.0);
            pose_msg.pose.orientation.w = std::cos(corrected_.yaw / 2.0);
            pose_pub_->publish(pose_msg);

        } else {
            corrected_.x   += dx;
            corrected_.y   += dy;
            corrected_.yaw  = std::atan2(
                std::sin(corrected_.yaw + dyaw),
                std::cos(corrected_.yaw + dyaw));
        }

        prev_odom_pose_ = odom_pose_;

        auto current_pts = scanToPoints(*msg, corrected_);

        bool tree_dirty = false;
        for (auto & pt : current_pts) {
            if (map_pts_.size() < MAP_PTS_MAX && isFarEnough(pt)) {
                map_pts_.push_back(pt);
                tree_dirty = true;
            }
        }
        if (tree_dirty || map_tree_.empty())
            map_tree_.build(map_pts_);

        int rx = worldToGridX(corrected_.x);
        int ry = worldToGridY(corrected_.y);
        int W  = static_cast<int>(map_.info.width);
        int H  = static_cast<int>(map_.info.height);

        map_.header.stamp = this->now();

        for (auto & pt : current_pts) {
            int gx = worldToGridX(pt.x);
            int gy = worldToGridY(pt.y);

            // Ray boyunca free güncelle — mesafeye göre ağırlıklı
            bresenham(rx, ry, gx, gy, [&](int cx, int cy) {
                if (cx < 0 || cx >= W || cy < 0 || cy >= H) return;
                if (cx == gx && cy == gy) return;
                int idx = cy * W + cx;

                // Hücrenin robot'a olan mesafesi
                double cell_wx = map_.info.origin.position.x + (cx + 0.5) * map_.info.resolution;
                double cell_wy = map_.info.origin.position.y + (cy + 0.5) * map_.info.resolution;
                double ray_len = std::hypot(cell_wx - corrected_.x, cell_wy - corrected_.y);

                updateFree(idx, ray_len);
                map_.data[idx] = toOccupancy(log_odds_[idx]);
            });

            // Hit hücresini güncelle
            if (gx < 0 || gx >= W || gy < 0 || gy >= H) continue;
            int idx = gy * W + gx;
            updateHit(idx);
            map_.data[idx] = toOccupancy(log_odds_[idx]);
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
        int cells = map_.info.width * map_.info.height;
        map_.data.resize(cells, -1);
        log_odds_.resize(cells, 0.0f);
    }

    // Log-odds parametreleri
    static constexpr float  LOG_ODDS_HIT       =  0.9f;
    static constexpr float  LOG_ODDS_FREE      = -0.4f;
    static constexpr float  LOG_ODDS_MIN       = -2.0f;
    static constexpr float  LOG_ODDS_MAX       =  5.0f;  // yüksek → duvarı silmek zor
    static constexpr double RAY_SIGMA          =  2.0;   // m — uzak free etkisini bastır

    // Map nokta bulutu parametreleri
    static constexpr double MAP_PTS_MIN_DIST_SQ = 0.05 * 0.05;
    static constexpr size_t MAP_PTS_MAX         = 20000;
    static constexpr size_t MIN_MAP_PTS         = 50;

    bool   odom_initialized_ = false;
    Pose2D corrected_        {0.0, 0.0, 0.0};
    Pose2D odom_pose_        {0.0, 0.0, 0.0};
    Pose2D prev_odom_pose_   {0.0, 0.0, 0.0};

    std::vector<Point2D>         map_pts_;
    KdTree                       map_tree_;
    std::vector<float>           log_odds_;
    nav_msgs::msg::OccupancyGrid map_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr    map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyMapper>());
    rclcpp::shutdown();
    return 0;
}