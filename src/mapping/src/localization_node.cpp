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

    bool nearestTwo(const Point2D& q, double thresh_sq, Point2D& out1, Point2D& out2) const {
        double best1_d = thresh_sq;
        double best2_d = thresh_sq;
        
        searchTwo(0, q, 0, out1, best1_d, out2, best2_d);
        
        // Eğer ikinci bir nokta bulabildiysek işlem başarılıdır
        return best2_d < thresh_sq;
    }

    bool empty() const { return nodes_.empty(); }

private:
    const std::vector<Point2D>* pts_ = nullptr;
    std::vector<KdNode> nodes_;


    void searchTwo(int id, const Point2D& q, int d,
                   Point2D& best1, double& best1_d,
                   Point2D& best2, double& best2_d) const {
        if (id < 0 || id >= static_cast<int>(nodes_.size())) return;
        
        const auto& n = nodes_[id];
        double dx = q.x - n.point.x;
        double dy = q.y - n.point.y;
        double dist = dx * dx + dy * dy;

        // Nokta en yakın 1. noktadan daha yakınsa
        if (dist < best1_d) {
            // Eski 1. noktayı 2. nokta yap (Aşağı kaydır)
            best2_d = best1_d;
            best2   = best1;
            
            // Yeni noktayı 1. nokta yap
            best1_d = dist;
            best1   = n.point;
        } 
        // Nokta 1. noktadan uzak ama 2. noktadan yakınsa
        // (Ayrıca aynı nokta üst üste binmişse diye > 1e-6 kontrolü yapıyoruz)
        else if (dist < best2_d && dist > 1e-6) {
            best2_d = dist;
            best2   = n.point;
        }

        int axis = d % 2;
        double diff = (axis == 0) ? dx : dy;
        int near = (diff < 0) ? n.left  : n.right;
        int far  = (diff < 0) ? n.right : n.left;

        searchTwo(near, q, d + 1, best1, best1_d, best2, best2_d);
        
        // Sınır kontrolünü artık best2_d'ye göre yapmalıyız ki 
        // 2. en yakın noktayı kaçırmayalım
        if (diff * diff < best2_d) {
            searchTwo(far, q, d + 1, best1, best1_d, best2, best2_d);
        }
    }

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
        if (i % 3 != 0) continue;
        float r = scan.ranges[i];
        if (r < scan.range_min || r > scan.range_max) continue;
        float a = scan.angle_min + i * scan.angle_increment + pose.yaw;
        pts.push_back({ pose.x + r * std::cos(a), pose.y + r * std::sin(a) });
    }
    return pts;
}


// 3x3 Hessian matrisini (H) ve 3x1 Gradient vektörünü (B) alıp, sonucu X'e yazar.
// Eğer matris tekil (singular) ise false döner.
bool solve3x3(const double H[3][3], const double B[3], double X[3]) {
    // 3x3 matrisin determinantı
    double det = H[0][0] * (H[1][1] * H[2][2] - H[1][2] * H[2][1])
               - H[0][1] * (H[1][0] * H[2][2] - H[1][2] * H[2][0])
               + H[0][2] * (H[1][0] * H[2][1] - H[1][1] * H[2][0]);

    // Determinant 0'a çok yakınsa sistem çözülemez (robot koridorda sıkışmış veya veri yetersiz)
    if (std::abs(det) < 1e-8) return false; 

    double invDet = 1.0 / det;

    // Cramer Kuralı ile X, Y ve Yaw (Theta) değerlerinin bulunması
    X[0] = (B[0] * (H[1][1] * H[2][2] - H[1][2] * H[2][1]) -
            H[0][1] * (B[1] * H[2][2] - H[1][2] * B[2]) +
            H[0][2] * (B[1] * H[2][1] - H[1][1] * B[2])) * invDet;

    X[1] = (H[0][0] * (B[1] * H[2][2] - H[1][2] * B[2]) -
            B[0] * (H[1][0] * H[2][2] - H[1][2] * H[2][0]) +
            H[0][2] * (H[1][0] * B[2] - B[1] * H[2][0])) * invDet;

    X[2] = (H[0][0] * (H[1][1] * B[2] - B[1] * H[2][1]) -
            H[0][1] * (H[1][0] * B[2] - B[1] * H[2][0]) +
            B[0] * (H[1][0] * H[2][1] - H[1][1] * H[2][0])) * invDet;

    return true;
}

// ─── ICP ─────────────────────────────────────────────────────────────────────
Pose2D icp(const std::vector<Point2D>& source, const KdTree& tree)
{
    Pose2D delta{0, 0, 0};
    if (source.empty() || tree.empty()) return delta;

    std::vector<Point2D> pts = source;

    for (int iter = 0; iter < 50; ++iter) {
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


Pose2D icp_point_to_line(const std::vector<Point2D>& source, const KdTree& tree)
{
    Pose2D delta{0, 0, 0};
    if (source.empty() || tree.empty()) return delta;

    std::vector<Point2D> pts = source;

    for (int iter = 0; iter < 50; ++iter) {
        double thresh    = 1.0 - iter * (0.5 / 50.0);
        double thresh_sq = thresh * thresh;

        // Denklem sistemimiz: H * x = B
        double H[3][3] = {{0}}; 
        double B[3]    = {0};   
        
        int valid_pairs = 0;

        for (auto& s : pts) {
            Point2D t1, t2;
            
            // Eğer KD-Tree bize yüzey oluşturacak en yakın 2 noktayı dönerse:
            if (tree.nearestTwo(s, thresh_sq, t1, t2)) {
                
                // 1. Hedef yüzeyin normal vektörünü hesapla
                double l_dx = t2.x - t1.x;
                double l_dy = t2.y - t1.y;
                double len = std::hypot(l_dx, l_dy);
                
                if (len < 1e-6) continue;
                
                // Vektörü 90 derece döndürüp normalize ediyoruz
                double nx = -l_dy / len;
                double ny =  l_dx / len;

                // 2. Jacobian matrisi elemanları (A matrisi satırı)
                double a1 = nx;
                double a2 = ny;
                double a3 = s.x * ny - s.y * nx; // Rotasyon etkisi
                
                // Hata metrigimiz (Noktanın doğruya olan dik uzaklığı)
                double b_i = (t1.x - s.x) * nx + (t1.y - s.y) * ny;

                // 3. Matrisleri biriktir (Accumulation)
                H[0][0] += a1 * a1; H[0][1] += a1 * a2; H[0][2] += a1 * a3;
                H[1][0] += a2 * a1; H[1][1] += a2 * a2; H[1][2] += a2 * a3;
                H[2][0] += a3 * a1; H[2][1] += a3 * a2; H[2][2] += a3 * a3;

                B[0] += a1 * b_i;
                B[1] += a2 * b_i;
                B[2] += a3 * b_i;
                
                valid_pairs++;
            }
        }

        if (valid_pairs < 20) break; // Yeterli eşleşme yoksa dur

        double X[3] = {0};
        
        // Matrisi çözmeye çalış
        if (!solve3x3(H, B, X)) {
            break; // Matris çözülemediyse iterasyonu bitir
        }

        double dx   = X[0];
        double dy   = X[1];
        double dyaw = X[2];

        // Bulunan küçük sapmaları (delta) kaynak noktalara uygula
        for (auto& p : pts) {
            double nx_pt = p.x * std::cos(dyaw) - p.y * std::sin(dyaw) + dx;
            double ny_pt = p.x * std::sin(dyaw) + p.y * std::cos(dyaw) + dy;
            p.x = nx_pt; 
            p.y = ny_pt;
        }

        // Toplam sapmayı güncelle
        delta.x   += dx;
        delta.y   += dy;
        delta.yaw  = std::atan2(std::sin(delta.yaw + dyaw), std::cos(delta.yaw + dyaw));

        // Eğer değişim çok küçüldüyse (yakınsama sağlandıysa) çık
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

    static constexpr double MAP_PTS_MIN_DIST_SQ = 0.05 * 0.05;
    static constexpr size_t MAP_PTS_MAX         = 20000;

    double normalizeAngle(double angle) const {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

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
        // Eski kod:
        // Pose2D d = icp(pts, tree_);

        // Yeni kod:
        Pose2D d = icp_point_to_line(pts, tree_);
        
        corrected_.x   = guess.x + d.x;
        corrected_.y   = guess.y + d.y;
        corrected_.yaw = normalizeAngle(guess.yaw + d.yaw);


        prev_ = odom_;

        // Sadece yeterince uzak noktaları ekle
        bool tree_dirty = false;
        for (auto& p : pts) {
            if (map_pts_.size() < MAP_PTS_MAX && isFarEnough(p)) {
                map_pts_.push_back(p);
                tree_dirty = true;
            }
        }

        // YENI EKLENEN: Sadece son noktalar kalsın
        if (map_pts_.size() > MAP_PTS_MAX) {
            map_pts_.erase(map_pts_.begin(), map_pts_.begin() + 3000);
            tree_dirty = true; // Noktalar silindiği için ağaç yeniden kurulmalı
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