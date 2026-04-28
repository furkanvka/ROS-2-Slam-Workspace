#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

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

    std::vector<float> scan_angles_cos_;
    std::vector<float> scan_angles_sin_;
    
    std::vector<uint32_t> last_free_update_;
    uint32_t              scan_count_ = 0;

    // ── GÜNCELLENMİŞ LOG-ODDS PARAMETRELERİ ──────────────────────────────
    // Hit değeri yüksek (duvar hemen oluşur), Free değeri düşük (zor silinir)
    static constexpr float LOG_ODDS_HIT  =  1.5f;  
    static constexpr float LOG_ODDS_FREE = -0.2f;  // Daha küçük bir negatif değer
    static constexpr float LOG_ODDS_MIN  = -3.0f;  
    static constexpr float LOG_ODDS_MAX  =  8.0f;  // Üst sınır yükseltildi (duvar doygunluğu)

    inline int8_t toOccupancy(float lo) const
    {
        // Duvarın haritada kalma toleransını artırmak için eşik düşürüldü
        if (lo > 0.3f)  return 100; 
        if (lo < -1.0f) return 0;
        return -1;
    }

    inline void updateFree(int idx)
    {
        if (last_free_update_[idx] == scan_count_) return; 
        
        log_odds_[idx] = std::max(LOG_ODDS_MIN, log_odds_[idx] + LOG_ODDS_FREE);
        last_free_update_[idx] = scan_count_;
        map_.data[idx] = toOccupancy(log_odds_[idx]);
    }

    inline void updateHit(int idx)
    {
        log_odds_[idx] = std::min(LOG_ODDS_MAX, log_odds_[idx] + LOG_ODDS_HIT);
        map_.data[idx] = toOccupancy(log_odds_[idx]);
    }

    template<typename Func>
    void bresenham(int x0, int y0, int x1, int y1, Func visit)
    {
        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
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

        double dx   = pose_.x   - prev_pose_.x;
        double dy   = pose_.y   - prev_pose_.y;
        double dyaw = pose_.yaw - prev_pose_.yaw;
        
        while (dyaw >  M_PI) dyaw -= 2.0 * M_PI;
        while (dyaw < -M_PI) dyaw += 2.0 * M_PI;

        // Çok küçük hareketlerde haritayı yormamak için güncelleme yapma
        if (std::hypot(dx, dy) < 0.02 && std::abs(dyaw) < 0.01) {
            return;
        }

        prev_pose_ = pose_;
        scan_count_++;

        int W  = static_cast<int>(map_.info.width);
        int H  = static_cast<int>(map_.info.height);
        int rx = worldToGridX(pose_.x);
        int ry = worldToGridY(pose_.y);

        if (scan_angles_cos_.empty()) {
            scan_angles_cos_.reserve(msg->ranges.size());
            scan_angles_sin_.reserve(msg->ranges.size());
            for (size_t i = 0; i < msg->ranges.size(); ++i) {
                float a = msg->angle_min + i * msg->angle_increment;
                scan_angles_cos_.push_back(std::cos(a));
                scan_angles_sin_.push_back(std::sin(a));
            }
        }

        double cos_yaw = std::cos(pose_.yaw);
        double sin_yaw = std::sin(pose_.yaw);

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float r = msg->ranges[i];
            
            // Maksimum menzil sınırında "max_range" verisini boş alan güncellemek için kullanabiliriz
            bool is_hit = true;
            if (r >= msg->range_max || std::isnan(r)) {
                r = msg->range_max - 0.1f; 
                is_hit = false;
            }
            if (r < msg->range_min) continue;

            double lx = r * scan_angles_cos_[i];
            double ly = r * scan_angles_sin_[i];
            
            double wx = pose_.x + (lx * cos_yaw - ly * sin_yaw);
            double wy = pose_.y + (lx * sin_yaw + ly * cos_yaw);

            int gx = worldToGridX(wx);
            int gy = worldToGridY(wy);

            // Yol boyu (Boş alan) güncellemesi
            bresenham(rx, ry, gx, gy, [&](int cx, int cy) {
                if (cx < 0 || cx >= W || cy < 0 || cy >= H) return;
                if (cx == gx && cy == gy && is_hit) return; 
                
                int idx = cy * W + cx;
                updateFree(idx);
            });

            // Engel (Hit) güncellemesi
            if (is_hit && gx >= 0 && gx < W && gy >= 0 && gy < H) {
                int idx = gy * W + gx;
                updateHit(idx);
            }
        }

        map_.header.stamp = now();
        map_pub_->publish(map_);
    }

    int worldToGridX(double x) const {
        return static_cast<int>((x - map_.info.origin.position.x) / map_.info.resolution);
    }
    int worldToGridY(double y) const {
        return static_cast<int>((y - map_.info.origin.position.y) / map_.info.resolution);
    }

    void initMap()
    {
        map_.header.frame_id = "odom";
        map_.info.resolution = 0.05;
        map_.info.width      = 600; // Biraz daha geniş bir alan
        map_.info.height     = 600;
        map_.info.origin.position.x = -15.0;
        map_.info.origin.position.y = -15.0;
        map_.info.origin.orientation.w = 1.0;
        
        int N = map_.info.width * map_.info.height;
        map_.data.resize(N, -1);
        log_odds_.resize(N, 0.0f);
        last_free_update_.resize(N, 0);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr     scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr       map_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MappingNode>());
    rclcpp::shutdown();
    return 0;
}