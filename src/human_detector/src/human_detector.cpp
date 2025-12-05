#include <chrono>
#include <cmath>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

class HumanDetector : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  HumanDetector()
  : Node("human_detector")
  {
    RCLCPP_INFO(this->get_logger(), "Human Detector Node Started.");

    // Laser scanner subscriber
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&HumanDetector::scan_callback, this, std::placeholders::_1));

    // Nav2 action client
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

    // Wandering timer
    wander_timer_ = this->create_wall_timer(
      10s, std::bind(&HumanDetector::wander_loop, this));

    rng_.seed(std::random_device{}());
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::TimerBase::SharedPtr wander_timer_;

  std::mt19937 rng_;

  // -----------------------------
  // LASER SCAN CALLBACK
  // -----------------------------
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Simple threshold to detect a “human-like” blob
    float min_dist = std::numeric_limits<float>::max();

    for (float r : msg->ranges)
    {
      if (std::isfinite(r) && r < min_dist)
        min_dist = r;
    }

    if (min_dist < 0.8)  // Something is close
    {
      RCLCPP_WARN(this->get_logger(), "⚠ HUMAN DETECTED approx %.2f m away!", min_dist);
      wander_timer_->cancel();  // Stop wandering
    }
  }

  // -----------------------------
  // SIMPLE WANDERING NAVIGATION
  // -----------------------------
  void wander_loop()
  {
    if (!nav_client_->wait_for_action_server(1s))
    {
      RCLCPP_WARN(this->get_logger(), "Nav2 action server not available yet.");
      return;
    }

    // Pick random point near the robot
    std::uniform_real_distribution<float> dist(-2.0, 2.0);

    float x = dist(rng_);
    float y = dist(rng_);

    RCLCPP_INFO(this->get_logger(), "Wandering to random position: (%.2f, %.2f)", x, y);

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

