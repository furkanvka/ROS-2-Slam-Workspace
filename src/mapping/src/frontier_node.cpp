#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <vector>
#include <cmath>
#include <queue>
#include <algorithm>
#include <unordered_set>

// ─── Helpers ─────────────────────────────────────────────────────────────────

struct Pose2D { double x, y, yaw; };

// ─── Node ─────────────────────────────────────────────────────────────────────

class FrontierNode : public rclcpp::Node
{
public:
    FrontierNode() : Node("frontier_node")
    {
        using std::placeholders::_1;

        // ── Parameters (Sadece planlama ile ilgili olanlar) ───────────────────
        declare_parameter("robot_radius_m",      0.40);  
        declare_parameter("center_preference_m", 0.40);  

        robot_radius_m_ = get_parameter("robot_radius_m").as_double();
        center_pref_m_  = get_parameter("center_preference_m").as_double();

        // ── Subscriptions ────────────────────────────────────────────────────
        map_sub_  = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&FrontierNode::mapCb,  this, _1));
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/corrected_pose", 10, std::bind(&FrontierNode::poseCb, this, _1));

        // ── Publishers ───────────────────────────────────────────────────────
        path_pub_ = create_publisher<nav_msgs::msg::Path>("/plan", 10);

        // ── Timer (Planlama ağır işlemdir, 2 Hz yeterlidir) ──────────────────
        timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&FrontierNode::exploreLoop, this));
    }

private:
    // ── State ─────────────────────────────────────────────────────────────────
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    Pose2D  pose_{};
    bool    pose_ready_  = false;

    std::vector<Pose2D> path_;                   
    std::unordered_set<int> blacklisted_goals_;  

    // ROS handles
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr   map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr               path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double robot_radius_m_, center_pref_m_;

    // ── Callbacks ─────────────────────────────────────────────────────────────

    void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        pose_.x   = msg->pose.position.x;
        pose_.y   = msg->pose.position.y;
        double z  = msg->pose.orientation.z;
        double w  = msg->pose.orientation.w;
        pose_.yaw = std::atan2(2.0 * z * w, 1.0 - 2.0 * z * z);
        pose_ready_ = true;
    }

    void mapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { map_ = msg; }

    // ── Grid helpers ──────────────────────────────────────────────────────────

    int worldToGridX(double x) const { return static_cast<int>((x - map_->info.origin.position.x) / map_->info.resolution); }
    int worldToGridY(double y) const { return static_cast<int>((y - map_->info.origin.position.y) / map_->info.resolution); }
    double gridToWorldX(int gx) const { return map_->info.origin.position.x + (gx + 0.5) * map_->info.resolution; }
    double gridToWorldY(int gy) const { return map_->info.origin.position.y + (gy + 0.5) * map_->info.resolution; }
    bool inBounds(int x, int y) const { return x >= 0 && x < (int)map_->info.width && y >= 0 && y < (int)map_->info.height; }

    // ── Costmap & Path Planning ───────────────────────────────────────────────
    
    std::vector<int> buildCostGrid() const
    {
        int W = map_->info.width, H = map_->info.height;
        int inflate_cells = static_cast<int>(std::ceil(robot_radius_m_ / map_->info.resolution));
        int penalty_cells = static_cast<int>(std::ceil((robot_radius_m_ + center_pref_m_) / map_->info.resolution));

        std::vector<int> grid(W * H, 0);

        for (int i = 0; i < W * H; ++i) {
            if (map_->data[i] == 100) grid[i] = 10000;
        }

        std::vector<int> result = grid;
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                if (grid[y * W + x] != 10000) continue; 
                
                for (int dy = -penalty_cells; dy <= penalty_cells; ++dy) {
                    for (int dx = -penalty_cells; dx <= penalty_cells; ++dx) {
                        int nx = x + dx, ny = y + dy;
                        if (!inBounds(nx, ny)) continue;
                        
                        double dist = std::hypot(dx, dy);
                        int idx = ny * W + nx;
                        
                        if (dist <= inflate_cells) {
                            result[idx] = 255; 
                        } else if (dist <= penalty_cells && result[idx] < 255) {
                            double penalty_weight = 15.0; 
                            int penalty = static_cast<int>(penalty_weight * (penalty_cells - dist));
                            result[idx] = std::max(result[idx], penalty);
                        }
                    }
                }
            }
        }
        return result;
    }

    bool planPath(int sx, int sy, int gx, int gy, const std::vector<int>& cost_grid)
    {
        path_.clear();
        if (!map_) return false;
        int W = map_->info.width, H = map_->info.height;
        if (!inBounds(sx, sy) || !inBounds(gx, gy)) return false;

        int s_idx = sy * W + sx;
        int g_idx = gy * W + gx;

        if (cost_grid[g_idx] == 255) {
            bool found_free = false;
            for (int r = 1; r <= 5 && !found_free; ++r) {
                for (int dy = -r; dy <= r && !found_free; ++dy) {
                    for (int dx = -r; dx <= r && !found_free; ++dx) {
                        int ng = (gy + dy) * W + (gx + dx);
                        if (inBounds(gx+dx, gy+dy) && cost_grid[ng] != 255) {
                            gx = gx + dx; gy = gy + dy;
                            g_idx = ng;
                            found_free = true;
                        }
                    }
                }
            }
            if (!found_free) return false;
        }

        struct ANode {
            int idx; float f;
            bool operator>(const ANode& o) const { return f > o.f; }
        };

        std::vector<float>  g_cost(W * H, 1e9f);
        std::vector<int>    came_from(W * H, -1);
        std::priority_queue<ANode, std::vector<ANode>, std::greater<ANode>> open;

        g_cost[s_idx] = 0.0f;
        open.push({s_idx, 0.0f});

        const int   dx8[] = {-1, 1, 0, 0, -1, -1, 1, 1};
        const int   dy8[] = { 0, 0,-1, 1, -1,  1,-1, 1};
        const float dc8[] = {1.f,1.f,1.f,1.f,1.414f,1.414f,1.414f,1.414f};

        while (!open.empty()) {
            auto [cur, _f] = open.top(); open.pop();
            if (cur == g_idx) break;

            int cx = cur % W, cy = cur / W;
            for (int i = 0; i < 8; ++i) {
                int nx = cx + dx8[i], ny = cy + dy8[i];
                if (!inBounds(nx, ny)) continue;
                int n_idx = ny * W + nx;

                if (cost_grid[n_idx] == 255) continue;

                float terrain_penalty = cost_grid[n_idx] * 0.5f; 
                float tg = g_cost[cur] + dc8[i] + terrain_penalty;
                
                if (tg < g_cost[n_idx]) {
                    came_from[n_idx] = cur;
                    g_cost[n_idx]    = tg;
                    float h = std::hypot(nx - gx, ny - gy);
                    open.push({n_idx, tg + h});
                }
            }
        }

        if (came_from[g_idx] == -1 && g_idx != s_idx) return false;

        for (int c = g_idx; c != s_idx; c = came_from[c]) {
            if (came_from[c] == -1) return false;
            path_.push_back({gridToWorldX(c % W), gridToWorldY(c / W), 0.0});
        }
        std::reverse(path_.begin(), path_.end());

        // --- ROTA YUMUŞATMA (MOVING AVERAGE FILTER) ---
        // Keskin A* köşelerini tıraşlayarak robotun salınımını önler
        if (path_.size() > 4) {
            std::vector<Pose2D> smoothed = path_;
            int window = 2; // Daha güçlü yumuşatma için 3 veya 4 yapılabilir
            for (size_t i = window; i < path_.size() - window; ++i) {
                double sum_x = 0, sum_y = 0;
                for (int j = -window; j <= window; ++j) {
                    sum_x += path_[i + j].x;
                    sum_y += path_[i + j].y;
                }
                smoothed[i].x = sum_x / (2 * window + 1);
                smoothed[i].y = sum_y / (2 * window + 1);
            }
            path_ = smoothed;
        }

        return !path_.empty();
    }

    bool findBestFrontier(double& out_x, double& out_y, const std::vector<int>& cost_grid)
    {
        if (!map_) return false;
        int W = map_->info.width, H = map_->info.height;

        struct Candidate { double wx, wy; double score; };
        std::vector<Candidate> cands;

        for (int y = 2; y < H - 2; ++y) {
            for (int x = 2; x < W - 2; ++x) {
                int idx = y * W + x;
                if (map_->data[idx] != 0)      continue;  
                if (cost_grid[idx]  == 255)    continue;  

                bool is_frontier = false;
                const int nx4[] = {-1,1,0,0};
                const int ny4[] = { 0,0,-1,1};
                int unknown_count = 0;
                for (int d = 0; d < 4; ++d) {
                    int ni = (y + ny4[d]) * W + (x + nx4[d]);
                    if (map_->data[ni] == -1) { is_frontier = true; ++unknown_count; }
                }
                if (!is_frontier) continue;

                double fx = gridToWorldX(x);
                double fy = gridToWorldY(y);
                double dist = std::hypot(fx - pose_.x, fy - pose_.y);
                if (dist < 0.5) continue; 

                double score = 1.0 * dist - 2.0 * unknown_count;
                cands.push_back({fx, fy, score});
            }
        }

        if (cands.empty()) return false;

        std::sort(cands.begin(), cands.end(),
                  [](const Candidate& a, const Candidate& b){ return a.score < b.score; });

        for (auto& c : cands) {
            int gx = worldToGridX(c.wx), gy = worldToGridY(c.wy);
            int key = gy * (int)map_->info.width + gx;
            if (blacklisted_goals_.count(key)) continue;
            out_x = c.wx; out_y = c.wy;
            return true;
        }
        return false;
    }

    void publishPath()
    {
        nav_msgs::msg::Path msg;
        msg.header.stamp    = now();
        msg.header.frame_id = map_ ? map_->header.frame_id : "odom";
        for (auto& p : path_) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = msg.header;
            ps.pose.position.x = p.x;
            ps.pose.position.y = p.y;
            ps.pose.orientation.w = 1.0;
            msg.poses.push_back(ps);
        }
        path_pub_->publish(msg);
    }

    void exploreLoop()
    {
        if (!map_ || !pose_ready_) return;

        auto cost_grid = buildCostGrid();
        double tx, ty;

        if (findBestFrontier(tx, ty, cost_grid)) {
            int sx = worldToGridX(pose_.x), sy = worldToGridY(pose_.y);
            int gx = worldToGridX(tx),      gy = worldToGridY(ty);
            
            if (planPath(sx, sy, gx, gy, cost_grid)) {
                RCLCPP_INFO(get_logger(), "New plan: %zu waypoints -> (%.2f, %.2f)", path_.size(), tx, ty);
                publishPath();
            } else {
                RCLCPP_WARN(get_logger(), "A* failed for frontier (%.2f, %.2f)", tx, ty);
                int key = gy * (int)map_->info.width + gx;
                blacklisted_goals_.insert(key); 
            }
        } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "No frontier found – exploration complete or map too small.");
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierNode>());
    rclcpp::shutdown();
    return 0;
}