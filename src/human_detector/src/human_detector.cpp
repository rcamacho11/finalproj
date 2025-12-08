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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

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

    // Decide CSV path: $HOME/detected_humans.csv or /tmp fallback
    const char *home = std::getenv("HOME");
    if (home) {
      human_csv_path_ = std::string(home) + "/detected_humans.csv";
    } else {
      human_csv_path_ = "/tmp/detected_humans.csv";
    }
    RCLCPP_INFO(this->get_logger(), "Using human CSV path: %s", human_csv_path_.c_str());

    // Load any previously detected humans
    load_detected_humans_from_csv();

    // Action client for Nav2
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Subscribe to laser scan
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      10,
      std::bind(&HumanDetector::scan_callback, this, std::placeholders::_1)
    );

    // Simple random wandering timer
    wander_timer_ = this->create_wall_timer(
      3s,
      std::bind(&HumanDetector::wander_loop, this)
    );

    // Cooldown timer (created but immediately cancelled; we enable it when needed)
    detection_cooldown_timer_ = this->create_wall_timer(
      5s,
      std::bind(&HumanDetector::cooldown_end, this)
    );
    detection_cooldown_timer_->cancel();

    RCLCPP_INFO(this->get_logger(), "HumanDetector node initialized.");
  }

private:
  struct HumanRecord
  {
    int id;
    float x;
    float y;
  };

  // Subscribers / action clients / timers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::TimerBase::SharedPtr wander_timer_;
  rclcpp::TimerBase::SharedPtr detection_cooldown_timer_;

  // Random generator for wandering
  std::mt19937 rng_;

  // Human detection state
  bool human_recently_detected_ = false;
  int human_count_ = 0;

  // Stored human zones (both in memory & mirrored to CSV)
  std::vector<HumanRecord> humans_;
  std::string human_csv_path_;

  // Detection parameters
  const float detection_distance_threshold_ = 0.8f;   // meters
  const float zone_half_size_ = 1.0f;                // ±1m → 2m × 2m zone

  // -------------------------------------------------------------
  // CSV helpers
  // -------------------------------------------------------------

  void load_detected_humans_from_csv()
  {
    std::ifstream in(human_csv_path_);
    if (!in.is_open()) {
      RCLCPP_INFO(this->get_logger(),
                  "No existing human CSV found at %s. Starting fresh.",
                  human_csv_path_.c_str());
      human_count_ = 0;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Loading detected humans from CSV...");

    std::string line;
    bool first_line = true;
    int max_id = 0;

    while (std::getline(in, line)) {
      if (line.empty()) continue;

      // Skip header if present
      if (first_line) {
        first_line = false;
        if (line.find("id") != std::string::npos) {
          // Looks like a header, skip
          continue;
        }
      }

      std::stringstream ss(line);
      std::string token;
      HumanRecord rec;

      // id
      if (!std::getline(ss, token, ',')) continue;
      rec.id = std::stoi(token);

      // x
      if (!std::getline(ss, token, ',')) continue;
      rec.x = std::stof(token);

      // y
      if (!std::getline(ss, token, ',')) continue;
      rec.y = std::stof(token);

      humans_.push_back(rec);
      if (rec.id > max_id) {
        max_id = rec.id;
      }
    }

    human_count_ = max_id;
    RCLCPP_INFO(this->get_logger(),
                "Loaded %zu humans from CSV. Next id will be %d.",
                humans_.size(), human_count_ + 1);
  }

  void append_human_to_csv(const HumanRecord &rec)
  {
    bool file_exists = false;
    {
      std::ifstream test(human_csv_path_);
      file_exists = test.good();
    }

    std::ofstream out(human_csv_path_, std::ios::app);
    if (!out.is_open()) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to open CSV file for writing: %s",
                  human_csv_path_.c_str());
      return;
    }

    // If file did not exist, write header first
    if (!file_exists) {
      out << "id,x,y\n";
    }

    out << rec.id << "," << rec.x << "," << rec.y << "\n";
    out.flush();
  }

  // -------------------------------------------------------------
  // Check if a detection falls inside an existing human zone
  // (2m × 2m box centered at each stored (x, y))
  // These coordinates are in the robot's frame at time of detection.
  // -------------------------------------------------------------
  bool is_new_human(float x, float y)
  {
    for (const auto &h : humans_) {
      if (std::fabs(x - h.x) < zone_half_size_ &&
          std::fabs(y - h.y) < zone_half_size_) {
        // Already detected a human in this region
        return false;
      }
    }
    return true;
  }

  // -------------------------------------------------------------
  // LASER SCAN CALLBACK
  // -------------------------------------------------------------
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // If we are in cooldown, ignore new detections
    if (human_recently_detected_) {
      return;
    }

    float min_dist = std::numeric_limits<float>::max();
    int min_index = -1;

    // Find closest valid range
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float r = msg->ranges[i];
      if (std::isfinite(r) && r < min_dist) {
        min_dist = r;
        min_index = static_cast<int>(i);
      }
    }

    // No valid reading
    if (min_index < 0) {
      return;
    }

    // Too far → not considered a human
    if (min_dist >= detection_distance_threshold_) {
      return;
    }

    // Compute angle and convert to robot-frame coordinates
    float angle = msg->angle_min + min_index * msg->angle_increment;
    float x = min_dist * std::cos(angle);
    float y = min_dist * std::sin(angle);

    // Check if this is a new human zone
    if (!is_new_human(x, y)) {
      // Already have a human in this region
      return;
    }

    // This is a NEW human
    human_recently_detected_ = true;
    human_count_++;

    HumanRecord rec;
    rec.id = human_count_;
    rec.x  = x;
    rec.y  = y;

    humans_.push_back(rec);
    append_human_to_csv(rec);

    RCLCPP_WARN(
      this->get_logger(),
      "NEW HUMAN %d DETECTED at (x=%.2f, y=%.2f), dist=%.2f m. Total humans: %d",
      rec.id, rec.x, rec.y, min_dist, human_count_
    );

    // Stop wandering while we "handle" this human (e.g., for grading / inspection)
    if (wander_timer_) {
      wander_timer_->cancel();
    }

    // Start cooldown so we do not immediately detect again
    if (detection_cooldown_timer_) {
      detection_cooldown_timer_->reset();
    }
  }

  // -------------------------------------------------------------
  // COOLDOWN → resume searching
  // -------------------------------------------------------------
  void cooldown_end()
  {
    human_recently_detected_ = false;
    RCLCPP_INFO(this->get_logger(),
                "Cooldown complete → continuing human search.");

    if (wander_timer_) {
      wander_timer_->reset();
    }
    if (detection_cooldown_timer_) {
      detection_cooldown_timer_->cancel();
    }
  }

  // -------------------------------------------------------------
  // SIMPLE WANDERING
  // -------------------------------------------------------------
  void wander_loop()
  {
    // Optional: do not send goals while we are in cooldown
    if (human_recently_detected_) {
      return;
    }

    if (!nav_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN(this->get_logger(), "Nav2 action server unavailable.");
      return;
    }

    // Simple random goal around the robot (in map frame)
    std::uniform_real_distribution<float> dist(-2.0f, 2.0f);
    float x = dist(rng_);
    float y = dist(rng_);

    RCLCPP_INFO(this->get_logger(),
                "Wandering to random goal (%.2f, %.2f)", x, y);

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp    = this->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.orientation.w = 1.0;  // facing forward, no rotation

    nav_client_->async_send_goal(goal_msg);
  }
};

// -------------------------------------------------------------
// main()
// -------------------------------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HumanDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

