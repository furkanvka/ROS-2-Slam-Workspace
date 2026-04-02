#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <vector>
#include <cmath>
#include <numeric>
#include <algorithm>

// ─── STRUCTS ─────────────────────────────────────────────────────────────────
struct Point2D { double x, y; };
struct Pose2D  { double x, y, yaw; };

// ─── KD-TREE ─────────────────────────────────────────────────────────────────
struct KdNode { Point2D point; int left = -1, right = -1; };

class KdTree {
public:
    void build(const std::vector<Point2D>& pts) {
        nodes_.clear();
        if (pts.empty()) return;
        std::vector<int> idx(pts.size());
        std::iota(idx.begin(), idx.end(), 0);
        pts_ = &pts;
        buildRec(idx, 0);
    }

    bool nearest(const Point2D& q, double thresh_sq, Point2D& out) const {
        double best = thresh_sq;
        search(0, q, 0, out, best);
        return best < thresh_sq;
    }

    bool empty() const { return nodes_.empty(); }

private:
    const std::vector<Point2D>* pts_ = nullptr;
    std::vector<KdNode> nodes_;

    int buildRec(std::vector<int>& idx, int d) {
        if (idx.empty()) return -1;
        int    axis = d % 2;
        size_t mid  = idx.size() / 2;
        std::nth_element(idx.begin(), idx.begin() + mid, idx.end(),
            [&](int a, int b) {
                return axis == 0 ? (*pts_)[a].x < (*pts_)[b].x
                                 : (*pts_)[a].y < (*pts_)[b].y;
            });
        KdNode n; n.point = (*pts_)[idx[mid]];
        int id = static_cast<int>(nodes_.size());
        nodes_.push_back(n);
        std::vector<int> L(idx.begin(), idx.begin() + mid);
        std::vector<int> R(idx.begin() + mid + 1, idx.end());
        nodes_[id].left  = buildRec(L, d + 1);
        nodes_[id].right = buildRec(R, d + 1);
        return id;
    }

    void search(int id, const Point2D& q, int d,
                Point2D& best, double& best_d) const {
        if (id < 0 || id >= static_cast<int>(nodes_.size())) return;
        const auto& n = nodes_[id];
        double dx = q.x - n.point.x, dy = q.y - n.point.y;
        double dist = dx*dx + dy*dy;
        if (dist < best_d) { best_d = dist; best = n.point; }
        int    axis = d % 2;
        double diff = (axis == 0) ? dx : dy;
        int    near = (diff < 0) ? n.left  : n.right;
        int    far  = (diff < 0) ? n.right : n.left;
        search(near, q, d + 1, best, best_d);
        if (diff * diff < best_d)
            search(far, q, d + 1, best, best_d);
    }
};

// ─── UTILS ───────────────────────────────────────────────────────────────────
std::vector<Point2D> scanToPoints(const sensor_msgs::msg::LaserScan& scan,
                                   const Pose2D& pose)
{
    std::vector<Point2D> pts;
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float r = scan.ranges[i];
        if (r < scan.range_min || r > scan.range_max) continue;
        float a = scan.angle_min + i * scan.angle_increment + pose.yaw;
        pts.push_back({ pose.x + r * std::cos(a), pose.y + r * std::sin(a) });
    }
    return pts;
}

// ─── ICP ─────────────────────────────────────────────────────────────────────
Pose2D icp(const std::vector<Point2D>& source, const KdTree& tree)
{
    Pose2D delta{0, 0, 0};
    if (source.empty() || tree.empty()) return delta;

    std::vector<Point2D> pts = source;

    for (int iter = 0; iter < 50; ++iter) {
        // FIX 1: coarse-to-fine threshold (eski davranış)
        double thresh    = 1.0 - iter * (0.5 / 50.0);
        double thresh_sq = thresh * thresh;

        std::vector<std::pair<Point2D,Point2D>> pairs;
        pairs.reserve(pts.size());
        for (auto& s : pts) {
            Point2D c;
            if (tree.nearest(s, thresh_sq, c))
                pairs.push_back({s, c});
        }
        if (static_cast<int>(pairs.size()) < 20) break;

        Point2D cs{0,0}, ct{0,0};
        for (auto& [s,t] : pairs) { cs.x+=s.x; cs.y+=s.y; ct.x+=t.x; ct.y+=t.y; }
        double n = static_cast<double>(pairs.size());
        cs.x/=n; cs.y/=n; ct.x/=n; ct.y/=n;

        double sxy=0, sxx=0;
        for (auto& [s,t] : pairs) {
            sxy += (s.x-cs.x)*(t.y-ct.y) - (s.y-cs.y)*(t.x-ct.x);
            sxx += (s.x-cs.x)*(t.x-ct.x) + (s.y-cs.y)*(t.y-ct.y);
        }

        double dyaw = std::atan2(sxy, sxx);
        double dx   = ct.x - (cs.x*std::cos(dyaw) - cs.y*std::sin(dyaw));
        double dy   = ct.y - (cs.x*std::sin(dyaw) + cs.y*std::cos(dyaw));

        for (auto& p : pts) {
            double nx = p.x*std::cos(dyaw) - p.y*std::sin(dyaw) + dx;
            double ny = p.x*std::sin(dyaw) + p.y*std::cos(dyaw) + dy;
            p.x = nx; p.y = ny;
        }

        delta.x   += dx;
        delta.y   += dy;
        delta.yaw  = std::atan2(std::sin(delta.yaw + dyaw),
                                std::cos(delta.yaw + dyaw));

        if (std::hypot(dx, dy) < 1e-4 && std::abs(dyaw) < 1e-4) break;
    }
    return delta;
}

// ─── NODE ─────────────────────────────────────────────────────────────────────
class LocalizationNode : public rclcpp::Node
{
public:
    LocalizationNode() : Node("localization_node")
    {
        using std::placeholders::_1;

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/a200_0000/sensors/lidar2d_0/scan", 10,
            std::bind(&LocalizationNode::scanCb, this, _1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/a200_0000/platform/odom/filtered", 10,
            std::bind(&LocalizationNode::odomCb, this, _1));

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/corrected_pose", 10);
    }

private:
    Pose2D odom_{}, prev_{}, corrected_{};
    bool initialized_ = false;

    std::vector<Point2D> map_pts_;
    KdTree               tree_;

    // FIX 2: minimum nokta mesafesi — KD-tree'yi gereksiz büyütme
    static constexpr double MAP_PTS_MIN_DIST_SQ = 0.05 * 0.05;
    static constexpr size_t MAP_PTS_MAX         = 20000;
    static constexpr size_t MIN_MAP_PTS         = 50;

    bool isFarEnough(const Point2D& p) const {
        for (auto& m : map_pts_)
            if ((p.x-m.x)*(p.x-m.x) + (p.y-m.y)*(p.y-m.y) < MAP_PTS_MIN_DIST_SQ)
                return false;
        return true;
    }

    void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto& q = msg->pose.pose.orientation;
        odom_.x   = msg->pose.pose.position.x;
        odom_.y   = msg->pose.pose.position.y;
        odom_.yaw = std::atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z));

        if (!initialized_) {
            corrected_ = odom_;
            prev_      = odom_;
            initialized_ = true;
        }
    }

    void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!initialized_) return;

        double dx   = odom_.x   - prev_.x;
        double dy   = odom_.y   - prev_.y;
        double dyaw = odom_.yaw - prev_.yaw;

        // FIX 3: dyaw normalize et
        while (dyaw >  M_PI) dyaw -= 2.0 * M_PI;
        while (dyaw < -M_PI) dyaw += 2.0 * M_PI;

        Pose2D guess{
            corrected_.x   + dx,
            corrected_.y   + dy,
            corrected_.yaw + dyaw
        };

        auto pts = scanToPoints(*msg, guess);

        if (map_pts_.size() >= MIN_MAP_PTS) {
            Pose2D d = icp(pts, tree_);
            corrected_.x   = guess.x   + d.x;
            corrected_.y   = guess.y   + d.y;
            corrected_.yaw = std::atan2(
                std::sin(guess.yaw + d.yaw),
                std::cos(guess.yaw + d.yaw));
        } else {
            corrected_ = guess;
        }

        prev_ = odom_;

        // FIX 4: sadece yeterince uzak noktaları ekle, dirty flag ile rebuild
        bool tree_dirty = false;
        for (auto& p : pts) {
            if (map_pts_.size() < MAP_PTS_MAX && isFarEnough(p)) {
                map_pts_.push_back(p);
                tree_dirty = true;
            }
        }
        if (tree_dirty || tree_.empty())
            tree_.build(map_pts_);

        // Publish
        geometry_msgs::msg::PoseStamped out;
        out.header.stamp    = now();
        out.header.frame_id = "odom";
        out.pose.position.x = corrected_.x;
        out.pose.position.y = corrected_.y;
        out.pose.orientation.z = std::sin(corrected_.yaw / 2.0);
        out.pose.orientation.w = std::cos(corrected_.yaw / 2.0);
        pose_pub_->publish(out);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationNode>());
    rclcpp::shutdown();
}