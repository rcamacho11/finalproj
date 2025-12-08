#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class HumanDetector : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    HumanDetector()
    : Node("human_detector_node"),
      rng_(std::random_device{}())
    {
        RCLCPP_INFO(this->get_logger(), "HumanDetector node starting...");

        // ===============================
        // CSV reset on launch
        // ===============================
        const char *home = std::getenv("HOME");
        if (home) {
            human_csv_path_ = std::string(home) + "/detected_humans.csv";
        } else {
            human_csv_path_ = "/tmp/detected_humans.csv";
        }
        reset_human_csv();

        // ===============================
        // TF listener
        // ===============================
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ===============================
        // Nav2 action client
        // ===============================
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // ===============================
        // Laser scan subscriber
        // ===============================
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&HumanDetector::scan_callback, this, std::placeholders::_1)
        );

        // ===============================
        // Map subscriber (TRANSIENT LOCAL → latched)
        // ===============================
        rclcpp::QoS map_qos(10);
        map_qos.transient_local().reliable();

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            map_qos,
            std::bind(&HumanDetector::map_callback, this, std::placeholders::_1)
        );

        // ===============================
        // Coverage navigation timer
        // ===============================
        coverage_timer_ = this->create_wall_timer(
            1s,
            std::bind(&HumanDetector::coverage_navigation_loop, this)
        );

        // ===============================
        // Cooldown timer for human detection
        // ===============================
        detection_cooldown_timer_ = this->create_wall_timer(
            5s,
            std::bind(&HumanDetector::cooldown_end, this)
        );
        detection_cooldown_timer_->cancel();

        RCLCPP_INFO(this->get_logger(), "HumanDetector initialized.");
    }

private:

    // ============================================
    // Human storage struct
    // ============================================
    struct HumanRecord {
        int   id;
        float x;
        float y;
    };

    // ============================================
    // Coverage zone struct
    // ============================================
    struct CoverageZone {
        double x;
        double y;
    };

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::TimerBase::SharedPtr coverage_timer_;
    rclcpp::TimerBase::SharedPtr detection_cooldown_timer_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Human detection
    bool human_recently_detected_ = false;
    int  human_count_ = 0;
    std::vector<HumanRecord> humans_;
    std::string human_csv_path_;

    // Coverage
    bool map_received_    = false;
    bool coverage_built_  = false;
    bool navigating_      = false;

    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    std::vector<geometry_msgs::msg::PoseStamped> coverage_path_;
    size_t current_wp_index_ = 0;

    // Coverage zones (to avoid re-visiting areas)
    std::vector<CoverageZone> covered_zones_;
    double coverage_radius_ = 4.0;   // updated from LiDAR
    bool coverage_radius_set_ = false;

    // Parameters
    const float detection_distance_threshold_ = 0.8f;
    const float zone_half_size_ = 1.0f;  
    const double stripe_spacing_m_ = 2.0;
    const double along_spacing_m_  = 1.0;
    const int8_t occupancy_threshold_ = 50;

    std::mt19937 rng_;

    // ==========================================================
    // CSV
    // ==========================================================
    void reset_human_csv()
    {
        humans_.clear();
        human_count_ = 0;

        std::ofstream out(human_csv_path_, std::ios::trunc);
        out << "id,x,y\n";
        RCLCPP_INFO(this->get_logger(), "Human CSV reset.");
    }

    void append_human_to_csv(const HumanRecord &rec)
    {
        std::ofstream out(human_csv_path_, std::ios::app);
        out << rec.id << "," << rec.x << "," << rec.y << "\n";
    }

    bool is_new_human(float x, float y)
    {
        for (const auto &h : humans_) {
            if (fabs(x - h.x) < zone_half_size_ &&
                fabs(y - h.y) < zone_half_size_) {
                return false;
            }
        }
        return true;
    }

    // ==========================================================
    // MAP CALLBACK → Build coverage path
    // ==========================================================
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        latest_map_ = msg;
        map_received_ = true;

        if (!coverage_built_) {
            RCLCPP_INFO(this->get_logger(), "Map received → building coverage path...");
            build_coverage_from_map(*msg);
        }
    }

    bool is_cell_free(const nav_msgs::msg::OccupancyGrid &map, int x, int y)
    {
        if (x < 0 || y < 0 ||
            x >= (int)map.info.width ||
            y >= (int)map.info.height)
            return false;

        int idx = y * map.info.width + x;
        int8_t v = map.data[idx];

        if (v < 0) return false;
        return v <= occupancy_threshold_;
    }

    geometry_msgs::msg::PoseStamped cell_to_pose(
        const nav_msgs::msg::OccupancyGrid &map, int x, int y)
    {
        geometry_msgs::msg::PoseStamped p;
        p.header.frame_id = "map";
        p.header.stamp = this->now();

        double res = map.info.resolution;

        p.pose.position.x = map.info.origin.position.x + (x + 0.5)*res;
        p.pose.position.y = map.info.origin.position.y + (y + 0.5)*res;
        p.pose.orientation.w = 1.0;

        return p;
    }

    void build_coverage_from_map(const nav_msgs::msg::OccupancyGrid &map)
    {
        coverage_path_.clear();
        current_wp_index_ = 0;

        int width = map.info.width;
        int height = map.info.height;
        double res = map.info.resolution;

        int row_step = std::max(1, (int)(stripe_spacing_m_ / res));
        int col_step = std::max(1, (int)(along_spacing_m_  / res));

        bool left_to_right = true;

        for (int y = 0; y < height; y += row_step) {

            bool added = false;

            if (left_to_right) {
                for (int x = 0; x < width; x += col_step) {
                    if (!is_cell_free(map, x, y)) continue;
                    coverage_path_.push_back(cell_to_pose(map, x, y));
                    added = true;
                }
            } else {
                for (int x = width - 1; x >= 0; x -= col_step) {
                    if (!is_cell_free(map, x, y)) continue;
                    coverage_path_.push_back(cell_to_pose(map, x, y));
                    added = true;
                }
            }

            if (added)
                left_to_right = !left_to_right;
        }

        if (coverage_path_.empty()) {
            RCLCPP_WARN(this->get_logger(),
                        "Coverage path empty — map has no free space!");
            return;
        }

        coverage_built_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "Coverage path built with %zu waypoints.",
                    coverage_path_.size());
    }

    // ==========================================================
    // COVERAGE ZONE LOGIC
    // ==========================================================
    bool is_in_covered_zone(double x, double y)
    {
        double r2 = coverage_radius_ * coverage_radius_;
        for (const auto &zone : covered_zones_) {
            double dx = x - zone.x;
            double dy = y - zone.y;
            if (dx*dx + dy*dy <= r2) {
                return true;
            }
        }
        return false;
    }

    // ==========================================================
    // COVERAGE NAVIGATION LOOP (A* via Nav2)
    // ==========================================================
    void coverage_navigation_loop()
    {
        if (human_recently_detected_) return;
        if (!map_received_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "Waiting for /map...");
            return;
        }
        if (!coverage_built_) return;
        if (navigating_) return;

        // Skip waypoints inside already-covered zones
        while (current_wp_index_ < coverage_path_.size()) {
            const auto &pos = coverage_path_[current_wp_index_].pose.position;
            if (!is_in_covered_zone(pos.x, pos.y)) {
                break;
            }
            current_wp_index_++;
        }

        if (current_wp_index_ >= coverage_path_.size()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                 "Coverage complete — all zones visited.");
            return;
        }

        if (!nav_client_->wait_for_action_server(1s)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Nav2 not ready.");
            return;
        }

        auto goal = NavigateToPose::Goal();
        goal.pose = coverage_path_[current_wp_index_];

        RCLCPP_INFO(this->get_logger(),
                    "Sending coverage goal %zu/%zu",
                    current_wp_index_+1,
                    coverage_path_.size());

        navigating_ = true;

        // =====================
        // Callbacks
        // =====================
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;

        // goal response
        opts.goal_response_callback =
            [this](std::shared_ptr<GoalHandleNav> handle)
            {
                if (!handle) {
                    RCLCPP_WARN(this->get_logger(), "Coverage goal rejected.");
                    navigating_ = false;
                }
            };

        // result
        opts.result_callback =
            [this](const GoalHandleNav::WrappedResult &result)
            {
                navigating_ = false;

                // Mark area covered
                if (current_wp_index_ < coverage_path_.size()) {
                    const auto &pos =
                        coverage_path_[current_wp_index_].pose.position;

                    covered_zones_.push_back({pos.x, pos.y});
                    RCLCPP_INFO(this->get_logger(),
                                "Marking zone (%.2f, %.2f) as covered (r=%.2f)",
                                pos.x, pos.y, coverage_radius_);
                }

                current_wp_index_++;
            };

        nav_client_->async_send_goal(goal, opts);
    }

    // ==========================================================
    // LASER → HUMAN DETECTION
    // ==========================================================
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // First time: set coverage radius from LiDAR
        if (!coverage_radius_set_) {
            if (std::isfinite(msg->range_max) && msg->range_max > 1.0) {
                coverage_radius_ = 0.6 * msg->range_max;
                RCLCPP_INFO(this->get_logger(),
                            "Coverage radius auto-set to %.2f from LiDAR range %.2f",
                            coverage_radius_, msg->range_max);
                coverage_radius_set_ = true;
            }
        }

        if (human_recently_detected_) return;

        float min_dist = std::numeric_limits<float>::max();
        int min_idx = -1;

        for (int i=0; i < (int)msg->ranges.size(); i++) {
            float r = msg->ranges[i];
            if (std::isfinite(r) && r < min_dist) {
                min_dist = r;
                min_idx = i;
            }
        }

        if (min_idx < 0) return;
        if (min_dist >= detection_distance_threshold_) return;

        float angle = msg->angle_min + min_idx * msg->angle_increment;

        geometry_msgs::msg::PointStamped p_laser, p_map;
        p_laser.header.frame_id = msg->header.frame_id;
        p_laser.header.stamp = msg->header.stamp;
        p_laser.point.x = min_dist * cos(angle);
        p_laser.point.y = min_dist * sin(angle);

        try {
            p_map = tf_buffer_->transform(p_laser, "map", 50ms);
        }
        catch (...) {
            return;
        }

        float x = p_map.point.x;
        float y = p_map.point.y;

        if (!is_new_human(x, y)) return;

        // New human detected
        human_recently_detected_ = true;
        human_count_++;

        HumanRecord rec{human_count_, x, y};
        humans_.push_back(rec);
        append_human_to_csv(rec);

        RCLCPP_WARN(this->get_logger(),
                    "NEW HUMAN %d detected at MAP (%.2f, %.2f)",
                    rec.id, x, y);

        detection_cooldown_timer_->reset();
    }

    // ==========================================================
    // COOLDOWN DONE → resume coverage search
    // ==========================================================
    void cooldown_end()
    {
        human_recently_detected_ = false;
        detection_cooldown_timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "Cooldown done — resuming coverage search.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanDetector>());
    rclcpp::shutdown();
    return 0;
}

