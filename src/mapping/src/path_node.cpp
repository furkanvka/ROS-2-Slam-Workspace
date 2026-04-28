#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

// ─────────────────────────────────────────────────────────────────────────────
//  Yardımcı tipler
// ─────────────────────────────────────────────────────────────────────────────

struct Pose2D { double x, y, yaw; };

static inline double normAngle(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Sektörel Lidar Özeti  (frontal / sol-ön / sağ-ön ayrı ayrı)
// ─────────────────────────────────────────────────────────────────────────────

struct LidarSector {
    double front_min;      // ±15° içindeki en yakın mesafe
    double left_min;       // 15°–60° içindeki en yakın mesafe
    double right_min;      // −60° – −15° içindeki en yakın mesafe
    double front_wide_min; // ±60° içindeki genel en yakın (collision check)
};

LidarSector parseSectors(const sensor_msgs::msg::LaserScan& scan)
{
    LidarSector s{1e9, 1e9, 1e9, 1e9};
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float r = scan.ranges[i];
        if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;

        float a = scan.angle_min + static_cast<float>(i) * scan.angle_increment;

        if (std::fabs(a) <= 1.047f) {              // ±60°
            s.front_wide_min = std::min(s.front_wide_min, static_cast<double>(r));
            if (std::fabs(a) <= 0.262f)            // ±15°
                s.front_min = std::min(s.front_min, static_cast<double>(r));
            else if (a > 0.262f)                   // sol 15°–60°
                s.left_min  = std::min(s.left_min,  static_cast<double>(r));
            else                                   // sağ
                s.right_min = std::min(s.right_min, static_cast<double>(r));
        }
    }
    return s;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Trapezoidal hız rampası  (her döngüde çağrılır)
// ─────────────────────────────────────────────────────────────────────────────

struct VelRamp {
    double v_current = 0.0;
    double w_current = 0.0;

    void step(double v_target, double w_target,
              double max_acc_v, double max_acc_w, double dt)
    {
        auto clamp_step = [](double cur, double tgt, double step) {
            double diff = tgt - cur;
            if (std::fabs(diff) <= step) return tgt;
            return cur + std::copysign(step, diff);
        };
        v_current = clamp_step(v_current, v_target, max_acc_v * dt);
        w_current = clamp_step(w_current, w_target, max_acc_w * dt);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  PathFollowerNode
// ─────────────────────────────────────────────────────────────────────────────

class PathFollowerNode : public rclcpp::Node
{
public:
    PathFollowerNode() : Node("path_follower_node")
    {
        using std::placeholders::_1;

        // ── Parametreler ──────────────────────────────────────────────────────
        declare_parameter("max_linear_vel",     0.50);  // m/s
        declare_parameter("min_linear_vel",     0.08);  // m/s
        declare_parameter("max_angular_vel",    1.20);  // rad/s
        declare_parameter("max_acc_linear",     0.40);  // m/s²
        declare_parameter("max_acc_angular",    1.50);  // rad/s²

        // Adaptive lookahead: L = base + k*v, clamp [min, max]
        declare_parameter("lookahead_base_m",   0.35);
        declare_parameter("lookahead_k",        0.40);  // L'yi hıza göre büyüt
        declare_parameter("lookahead_min_m",    0.20);
        declare_parameter("lookahead_max_m",    1.00);

        // Goal
        declare_parameter("goal_tolerance_m",   0.10);
        declare_parameter("heading_tolerance",  0.10);  // rad, hedefe varınca hizalama

        // Obstacle / lidar
        declare_parameter("stop_dist_m",        0.30);  // bu mesafenin altında dur
        declare_parameter("slow_dist_m",        0.70);  // bu mesafeden itibaren yavaşla
        declare_parameter("lateral_margin_m",   0.25);  // sol/sağ tehdit eşiği
        declare_parameter("steer_away_gain",    0.60);  // yanal tehditten kaçış kazancı

        // Stuck & recovery
        declare_parameter("stuck_timeout_s",    6.0);
        declare_parameter("stuck_dist_m",       0.08);
        declare_parameter("backup_duration_s",  1.5);
        declare_parameter("backup_vel",        -0.12);

        loadParams();

        // ── Pub / Sub ─────────────────────────────────────────────────────────
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/plan", 10, std::bind(&PathFollowerNode::pathCb, this, _1));
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/corrected_pose", 10, std::bind(&PathFollowerNode::poseCb, this, _1));
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/a200_0000/sensors/lidar2d_0/scan", 10,
            std::bind(&PathFollowerNode::scanCb, this, _1));

        cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
            "/a200_0000/cmd_vel", 10);

        // Plan isteği için servis istemcisi (FrontierNode'a)
        plan_client_ = create_client<std_srvs::srv::Trigger>("/plan_path");

        // ── Kontrol döngüsü 20 Hz ─────────────────────────────────────────────
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PathFollowerNode::controlLoop, this));

        RCLCPP_INFO(get_logger(), "PathFollowerNode hazır.");
    }

private:
    // ── State ─────────────────────────────────────────────────────────────────
    Pose2D pose_{};
    bool   pose_ready_ = false;

    std::vector<Pose2D> path_;
    size_t              progress_idx_ = 0;   // geride kalmış son waypoint indeksi

    LidarSector sector_{1e9, 1e9, 1e9, 1e9};
    bool        scan_ready_ = false;

    VelRamp ramp_;

    // Stuck detection
    Pose2D       last_moved_pose_{};
    rclcpp::Time last_moved_time_;
    bool         stuck_init_ = false;

    // Recovery
    bool         in_recovery_  = false;
    rclcpp::Time recovery_end_;

    // Plan request cooldown
    rclcpp::Time last_plan_req_time_;
    bool         plan_req_sent_ = false;

    // ── ROS handles ──────────────────────────────────────────────────────────
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr             path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr     scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr   cmd_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr                plan_client_;
    rclcpp::TimerBase::SharedPtr                                     timer_;

    // ── Parametreler ──────────────────────────────────────────────────────────
    double max_lin_, min_lin_, max_ang_;
    double max_acc_v_, max_acc_w_;
    double lh_base_, lh_k_, lh_min_, lh_max_;
    double goal_tol_, heading_tol_;
    double stop_dist_, slow_dist_, lateral_margin_, steer_gain_;
    double stuck_timeout_, stuck_dist_, backup_dur_, backup_vel_;

    void loadParams()
    {
        max_lin_       = get_parameter("max_linear_vel").as_double();
        min_lin_       = get_parameter("min_linear_vel").as_double();
        max_ang_       = get_parameter("max_angular_vel").as_double();
        max_acc_v_     = get_parameter("max_acc_linear").as_double();
        max_acc_w_     = get_parameter("max_acc_angular").as_double();
        lh_base_       = get_parameter("lookahead_base_m").as_double();
        lh_k_          = get_parameter("lookahead_k").as_double();
        lh_min_        = get_parameter("lookahead_min_m").as_double();
        lh_max_        = get_parameter("lookahead_max_m").as_double();
        goal_tol_      = get_parameter("goal_tolerance_m").as_double();
        heading_tol_   = get_parameter("heading_tolerance").as_double();
        stop_dist_     = get_parameter("stop_dist_m").as_double();
        slow_dist_     = get_parameter("slow_dist_m").as_double();
        lateral_margin_= get_parameter("lateral_margin_m").as_double();
        steer_gain_    = get_parameter("steer_away_gain").as_double();
        stuck_timeout_ = get_parameter("stuck_timeout_s").as_double();
        stuck_dist_    = get_parameter("stuck_dist_m").as_double();
        backup_dur_    = get_parameter("backup_duration_s").as_double();
        backup_vel_    = get_parameter("backup_vel").as_double();
    }

    // ── Callbacks ─────────────────────────────────────────────────────────────

    void pathCb(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (in_recovery_) return;
        path_.clear();
        progress_idx_ = 0;
        for (const auto& ps : msg->poses)
            path_.push_back({ps.pose.position.x, ps.pose.position.y, 0.0});
        plan_req_sent_ = false;
        RCLCPP_INFO(get_logger(), "Yeni rota: %zu waypoint.", path_.size());
    }

    void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        pose_.x   = msg->pose.position.x;
        pose_.y   = msg->pose.position.y;
        double z  = msg->pose.orientation.z;
        double w  = msg->pose.orientation.w;
        pose_.yaw = std::atan2(2.0 * z * w, 1.0 - 2.0 * z * z);
        pose_ready_ = true;

        if (!stuck_init_) {
            last_moved_pose_ = pose_;
            last_moved_time_ = now();
            stuck_init_      = true;
        }
    }

    void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        sector_    = parseSectors(*msg);
        scan_ready_ = true;
    }

    // ── Yardımcılar ───────────────────────────────────────────────────────────

    void publishStop()
    {
        ramp_.step(0.0, 0.0, max_acc_v_, max_acc_w_, 0.05);
        // Tam durdurma — rampa sıfıra yakınsa doğrudan sıfırla
        ramp_.v_current = 0.0;
        ramp_.w_current = 0.0;
        sendCmd(0.0, 0.0);
    }

    void sendCmd(double v, double w)
    {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp    = now();
        cmd.header.frame_id = "base_link";
        cmd.twist.linear.x  = std::clamp(v, -max_lin_, max_lin_);
        cmd.twist.angular.z = std::clamp(w, -max_ang_, max_ang_);
        cmd_pub_->publish(cmd);
    }

    // Lidar engelinden kaynaklanan yanal sapma düzeltmesi
    // Sağ taraf yakınsa → sola dön (+w), sol taraf yakınsa → sağa dön (−w)
    double lateralSteerCorrection() const
    {
        if (!scan_ready_) return 0.0;
        double correction = 0.0;
        if (sector_.right_min < lateral_margin_)
            correction += steer_gain_ * (1.0 - sector_.right_min / lateral_margin_);
        if (sector_.left_min  < lateral_margin_)
            correction -= steer_gain_ * (1.0 - sector_.left_min  / lateral_margin_);
        return correction;
    }

    // Frontal mesafeye göre hız faktörü  [0, 1]
    double obstacleVelFactor() const
    {
        if (!scan_ready_) return 1.0;
        double d = sector_.front_min;
        if (d <= stop_dist_) return 0.0;
        if (d >= slow_dist_) return 1.0;
        return (d - stop_dist_) / (slow_dist_ - stop_dist_);
    }

    bool isStuck()
    {
        if (!stuck_init_) return false;
        double moved = std::hypot(pose_.x - last_moved_pose_.x,
                                  pose_.y - last_moved_pose_.y);
        if (moved > stuck_dist_) {
            last_moved_pose_ = pose_;
            last_moved_time_ = now();
        }
        return (now() - last_moved_time_).seconds() > stuck_timeout_;
    }

    void requestNewPlan()
    {
        if (!plan_client_->service_is_ready()) return;
        if (plan_req_sent_) return;

        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        plan_client_->async_send_request(req,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture fut) {
                auto res = fut.get();
                if (res->success)
                    RCLCPP_INFO(get_logger(), "Yeni plan alındı: %s", res->message.c_str());
                else
                    RCLCPP_WARN(get_logger(), "Plan isteği başarısız: %s", res->message.c_str());
                plan_req_sent_ = false;  // tekrar deneyebilir
            });
        plan_req_sent_ = true;
        RCLCPP_INFO(get_logger(), "Yeni plan istendi.");
    }

    // ── Ana Kontrol Döngüsü ───────────────────────────────────────────────────

    void controlLoop()
    {
        if (!pose_ready_) return;

        constexpr double DT = 0.05; // 20 Hz

        // ── 1. Recovery modu ─────────────────────────────────────────────────
        if (in_recovery_) {
            if (now() < recovery_end_) {
                ramp_.step(backup_vel_, 0.0, max_acc_v_, max_acc_w_, DT);
                sendCmd(ramp_.v_current, ramp_.w_current);
            } else {
                in_recovery_ = false;
                path_.clear();
                progress_idx_ = 0;
                last_moved_pose_ = pose_;
                last_moved_time_ = now();
                RCLCPP_INFO(get_logger(), "Recovery bitti, yeni plan isteniyor.");
                requestNewPlan();
            }
            return;
        }

        // ── 2. Frontal çarpışma — acil dur ──────────────────────────────────
        if (scan_ready_ && sector_.front_min <= stop_dist_ && !path_.empty()) {
            publishStop();
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Engel: %.2f m — duruyorum.", sector_.front_min);
            return;
        }

        // ── 3. Rota yok ──────────────────────────────────────────────────────
        if (path_.empty()) {
            publishStop();
            return;
        }

        // ── 4. Stuck tespiti ─────────────────────────────────────────────────
        if (isStuck()) {
            RCLCPP_WARN(get_logger(), "Stuck! Recovery başlıyor.");
            in_recovery_  = true;
            recovery_end_ = now() + rclcpp::Duration::from_seconds(backup_dur_);
            publishStop();
            return;
        }

        // ── 5. Progress indeksini ilerlet ────────────────────────────────────
        //    Robota en yakın waypoint'i bul, progress_idx_ geri gidemez
        {
            double best = 1e9;
            size_t best_i = progress_idx_;
            for (size_t i = progress_idx_; i < path_.size(); ++i) {
                double d = std::hypot(path_[i].x - pose_.x, path_[i].y - pose_.y);
                if (d < best) { best = d; best_i = i; }
            }
            progress_idx_ = best_i;
        }

        // ── 6. Hedefe varış kontrolü ─────────────────────────────────────────
        const Pose2D& goal = path_.back();
        double dist_to_goal = std::hypot(goal.x - pose_.x, goal.y - pose_.y);
        if (dist_to_goal < goal_tol_) {
            publishStop();
            path_.clear();
            RCLCPP_INFO(get_logger(), "Hedefe ulaşıldı.");
            requestNewPlan();
            return;
        }

        // ── 7. Adaptive lookahead mesafesi ───────────────────────────────────
        double v_now     = ramp_.v_current;
        double lookahead = std::clamp(lh_base_ + lh_k_ * v_now, lh_min_, lh_max_);

        // Engel varsa lookahead'i kısalt — temkinli davran
        if (scan_ready_ && sector_.front_wide_min < slow_dist_)
            lookahead = std::max(lh_min_, lookahead * (sector_.front_wide_min / slow_dist_));

        // ── 8. Lookahead noktasını bul ───────────────────────────────────────
        size_t target_idx = path_.size() - 1;
        for (size_t i = progress_idx_; i < path_.size(); ++i) {
            if (std::hypot(path_[i].x - pose_.x, path_[i].y - pose_.y) >= lookahead) {
                target_idx = i;
                break;
            }
        }

        const Pose2D& tp = path_[target_idx];
        double dx    = tp.x - pose_.x;
        double dy    = tp.y - pose_.y;
        double alpha = normAngle(std::atan2(dy, dx) - pose_.yaw);  // heading hatası
        double L     = std::hypot(dx, dy);                         // gerçek mesafe

        // ── 9. Pure Pursuit curvature → (v, ω) ──────────────────────────────
        //    κ = 2·sin(α) / L   →   ω = v · κ
        double curvature = (L > 1e-3) ? (2.0 * std::sin(alpha) / L) : 0.0;

        // ── 10. Hız profili ──────────────────────────────────────────────────

        // a) Hedefe yaklaşırken yavaşla (lineer)
        double v_target = max_lin_;
        if (dist_to_goal < slow_dist_)
            v_target = max_lin_ * std::max(0.2, dist_to_goal / slow_dist_);

        // b) Yüksek eğride yavaşla — keskin köşe koruması
        double abs_curv = std::fabs(curvature);
        if (abs_curv > 0.5)
            v_target *= std::max(0.3, 1.0 - 0.6 * (abs_curv - 0.5));

        // c) Frontal engel faktörü
        double obs_factor = obstacleVelFactor();
        v_target *= obs_factor;

        // d) Minimum hız (durana kadar) — çok küçük hız robot motorunu zorlar
        if (v_target > 0.0 && v_target < min_lin_) v_target = min_lin_;

        double w_target = v_target * curvature;

        // e) Yanal duvardan kaçış — ω'ya eklenir
        w_target += lateralSteerCorrection();
        w_target  = std::clamp(w_target, -max_ang_, max_ang_);

        // ── 11. Trapezoidal rampa ────────────────────────────────────────────
        ramp_.step(v_target, w_target, max_acc_v_, max_acc_w_, DT);
        sendCmd(ramp_.v_current, ramp_.w_current);

        RCLCPP_DEBUG(get_logger(),
            "v=%.2f w=%.2f α=%.2f L=%.2f lh=%.2f front=%.2f",
            ramp_.v_current, ramp_.w_current,
            alpha, L, lookahead, sector_.front_min);
    }
};

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollowerNode>());
    rclcpp::shutdown();
    return 0;
}