#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

struct HumanBox {
  float x;   // robot-relative x position
  float y;   // robot-relative y position
};

class HumanDetector : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  HumanDetector()
  : Node("human_detector")
  {
    RCLCPP_INFO(this->get_logger(), "Human Detector Node Started.");

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&HumanDetector::scan_callback, this, std::placeholders::_1));

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

    wander_timer_ = this->create_wall_timer(
      8s, std::bind(&HumanDetector::wander_loop, this));

    detection_cooldown_timer_ = this->create_wall_timer(
      3s, std::bind(&HumanDetector::cooldown_end, this));
    detection_cooldown_timer_->cancel();

    rng_.seed(std::random_device{}());
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::TimerBase::SharedPtr wander_timer_;
  rclcpp::TimerBase::SharedPtr detection_cooldown_timer_;

  std::mt19937 rng_;

  bool human_recently_detected_ = false;
  int human_count_ = 0;

  std::vector<HumanBox> humans;  // stores human zones

  // -------------------------------------------------------------
  // Check if a detection falls inside an existing 1m × 1m box
  // -------------------------------------------------------------
  bool is_new_human(float x, float y)
  {
    for (const auto &h : humans)
    {
      if (std::fabs(x - h.x) < 0.5 && std::fabs(y - h.y) < 0.5)
      {
        return false;  // same human region
      }
    }
    return true;
  }

  // -------------------------------------------------------------
  // LASER SCAN CALLBACK
  // -------------------------------------------------------------
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    float min_dist = std::numeric_limits<float>::max();
    int min_index = -1;

    // Find closest object
    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
      float r = msg->ranges[i];
      if (std::isfinite(r) && r < min_dist)
      {
        min_dist = r;
        min_index = i;
      }
    }

    if (min_dist >= 0.8) return; // no close object

    float angle = msg->angle_min + min_index * msg->angle_increment;

    // Convert to robot-relative coordinates
    float x = min_dist * std::cos(angle);
    float y = min_dist * std::sin(angle);

    // IGNORE if still inside cooldown
    if (human_recently_detected_) return;

    // Check if NEW human zone
    if (is_new_human(x, y))
    {
      human_recently_detected_ = true;
      human_count_++;

      // Store 1m box center
      humans.push_back({x, y});

      RCLCPP_WARN(
        this->get_logger(),
        "⚠ NEW HUMAN %d DETECTED at (x=%.2f, y=%.2f), dist=%.2f m",
        human_count_,
        x, y, min_dist
      );

      wander_timer_->cancel();
      detection_cooldown_timer_->reset();
    }
  }

  // -------------------------------------------------------------
  // COOLDOWN → resume searching
  // -------------------------------------------------------------
  void cooldown_end()
  {
    human_recently_detected_ = false;
    RCLCPP_INFO(this->get_logger(), "Cooldown complete → continuing human search.");
    wander_timer_->reset();
    detection_cooldown_timer_->cancel();
  }

  // -------------------------------------------------------------
  // SIMPLE WANDERING
  // -------------------------------------------------------------
  void wander_loop()
  {
    if (!nav_client_->wait_for_action_server(1s))
    {
      RCLCPP_WARN(this->get_logger(), "Nav2 unavailable");
      return;
    }

    std::uniform_real_distribution<float> dist(-2.0, 2.0);
    float x = dist(rng_);
    float y = dist(rng_);

    RCLCPP_INFO(this->get_logger(), "Wandering to (%.2f, %.2f)", x, y);

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.orientation.w = 1.0;

    nav_client_->async_send_goal(goal_msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HumanDetector>());
  rclcpp::shutdown();
  return 0;
}

