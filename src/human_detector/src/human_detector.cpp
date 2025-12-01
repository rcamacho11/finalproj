#include <chrono>
#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// ✅ MRTP navigation library
#include "navigation/navigation.hpp"

using namespace std::chrono_literals;

class HumanDetector : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  HumanDetector()
  : Node("human_detector_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    RCLCPP_INFO(get_logger(), "HumanDetector node started.");

    // --- ACTION CLIENT (Nav2) ---
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

    // --- SUBSCRIPTIONS ---
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&HumanDetector::scan_callback, this, std::placeholders::_1));

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 1,
      std::bind(&HumanDetector::map_callback, this, std::placeholders::_1));

    amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10,
      std::bind(&HumanDetector::amcl_callback, this, std::placeholders::_1));

    // 🔁 Patrol waypoints – tune these in RViz so the robot can see both humans
    patrol_waypoints_ = {
      {  2.0, -17.0 },
      { 10.0, -17.0 },
      { 10.0, -10.0 },
      {  2.0, -10.0 }
    };

    timer_ = create_wall_timer(
      1s, std::bind(&HumanDetector::timer_callback, this));
  }

private:
  struct Human {
    double x;
    double y;
    bool   visited;
  };

  // Nav2
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  // Subs
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Map
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  bool have_map_ = false;

  // AMCL pose (for debugging / printing)
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_amcl_pose_;
  bool have_amcl_ = false;

  // Humans (dynamic obstacles not in map)
  std::vector<Human> humans_;

  // Patrol state
  std::vector<std::pair<double,double>> patrol_waypoints_;
  int current_waypoint_idx_ = 0;

  // Nav state
  bool nav_ready_   = false;
  bool goal_active_ = false;
  int  current_human_idx_ = -1;  // -1 means we're doing patrol, not human-visit

  // ===========================================
  //                 CALLBACKS
  // ===========================================

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = msg;
    have_map_ = true;
  }

  void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    last_amcl_pose_ = msg;
    have_amcl_ = true;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!have_map_) {
      // Can't compare with map yet
      return;
    }

    // Try to get transform map <- laser frame
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(
        "map", msg->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF lookup failed in scan_callback: %s", ex.what());
      return;
    }

    // Extract laser pose in map frame
    double tx = tf.transform.translation.x;
    double ty = tf.transform.translation.y;

    tf2::Quaternion q(
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z,
      tf.transform.rotation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Map info
    const auto & info = map_->info;
    double res   = info.resolution;
    double ox    = info.origin.position.x;
    double oy    = info.origin.position.y;
    int width    = info.width;
    int height   = info.height;

    // We’ll look at all beams; humans show up as "new obstacles in free cells".
    const auto & ranges = msg->ranges;
    double angle = msg->angle_min;

    for (size_t i = 0; i < ranges.size(); ++i, angle += msg->angle_increment)
    {
      double r = ranges[i];

      // Filter invalid or far ranges
      if (!std::isfinite(r) || r < 0.2 || r > 6.0) {
        continue;
      }

      // Only look roughly forward for “1 foot in front” message
      if (std::abs(angle) < 0.15 && r < 0.35) {
        RCLCPP_INFO(get_logger(),
          "👀 Something is ~1 foot in front (r = %.2f m)", r);
      }

      // Compute hit point in map frame
      double global_angle = yaw + angle;
      double wx = tx + r * std::cos(global_angle);
      double wy = ty + r * std::sin(global_angle);

      // Convert world (wx,wy) -> map indices (mx,my)
      int mx = static_cast<int>((wx - ox) / res);
      int my = static_cast<int>((wy - oy) / res);

      if (mx < 0 || mx >= width || my < 0 || my >= height) {
        continue;
      }
      int idx = my * width + mx;

      int8_t occ = map_->data[idx];
      // We only care about "unexpected obstacles" in cells that are FREE in the map.
      // In OccupancyGrid: 0 = free, 100 = occupied, -1 = unknown.
      if (occ != 0) {
        continue;  // shelves / walls / original humans that are in the static map
      }

      // Check if this is near an already-detected human (cluster)
      bool merged = false;
      for (auto & h : humans_) {
        double dx = h.x - wx;
        double dy = h.y - wy;
        if (std::sqrt(dx*dx + dy*dy) < 0.5) {
          // close to existing cluster, just nudge average
          h.x = 0.5 * (h.x + wx);
          h.y = 0.5 * (h.y + wy);
          merged = true;
          break;
        }
      }

      if (!merged) {
        Human new_h{wx, wy, false};
        humans_.push_back(new_h);
        RCLCPP_INFO(get_logger(),
          "👤 New dynamic obstacle (candidate human) at (%.2f, %.2f). Total=%zu",
          wx, wy, humans_.size());
      }
    }
  }

  // ===========================================
  //               NAVIGATION TIMER
  // ===========================================

  void timer_callback()
  {
    // Make sure Nav2 is ready
    if (!nav_ready_) {
      if (!nav_client_->wait_for_action_server(500ms)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Waiting for Nav2 action server...");
        return;
      }
      RCLCPP_INFO(get_logger(), "Nav2 action server is ready.");
      nav_ready_ = true;
    }

    if (goal_active_) {
      // already executing a goal
      return;
    }

    // If we have detected humans that we haven't visited yet, go to them first.
    int human_to_visit = get_unvisited_human_index();
    if (human_to_visit != -1) {
      current_human_idx_ = human_to_visit;
      auto & h = humans_[human_to_visit];
      RCLCPP_INFO(get_logger(),
        "Navigating to HUMAN #%d at (%.2f, %.2f)",
        human_to_visit + 1, h.x, h.y);

      send_goal(h.x, h.y);
      return;
    }

    // No unvisited humans yet: continue patrol to trigger more scans.
    if (current_waypoint_idx_ < static_cast<int>(patrol_waypoints_.size())) {
      double gx = patrol_waypoints_[current_waypoint_idx_].first;
      double gy = patrol_waypoints_[current_waypoint_idx_].second;
      RCLCPP_INFO(get_logger(),
        "Navigating to patrol waypoint %d at (%.2f, %.2f)",
        current_waypoint_idx_, gx, gy);

      current_human_idx_ = -1;  // goal is patrol, not human
      send_goal(gx, gy);
    } else {
      // Patrol finished
      int visited = count_visited_humans();
      RCLCPP_INFO(get_logger(),
        "Patrol finished. Humans detected=%zu, visited=%d",
        humans_.size(), visited);
      if (visited >= 2) {
        RCLCPP_INFO(get_logger(),
          "🎯 Mission complete: visited at least 2 humans.");
      }
      // Node keeps spinning so you can see info in logs / RViz.
    }
  }

  // ===========================================
  //          SEND GOAL + ACTION CALLBACKS
  // ===========================================

  void send_goal(double x, double y)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = now();

    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;
    goal.pose.orientation.w = 1.0;  // facing arbitrary direction

    NavigateToPose::Goal nav_goal;
    nav_goal.pose = goal;  // IMPORTANT: set the pose, or Nav2 rejects the goal

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;

    options.goal_response_callback =
      std::bind(&HumanDetector::goal_response_callback, this, std::placeholders::_1);

    options.result_callback =
      std::bind(&HumanDetector::result_callback, this, std::placeholders::_1);

    nav_client_->async_send_goal(nav_goal, options);
    goal_active_ = true;
  }

  void goal_response_callback(std::shared_ptr<GoalHandleNav> handle)
  {
    if (!handle) {
      RCLCPP_ERROR(get_logger(), "NavigateToPose goal was REJECTED by Nav2!");
      goal_active_ = false;
    } else {
      RCLCPP_INFO(get_logger(), "NavigateToPose goal ACCEPTED by Nav2.");
    }
  }

  void result_callback(const GoalHandleNav::WrappedResult & result)
  {
    goal_active_ = false;

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        if (current_human_idx_ >= 0 &&
            current_human_idx_ < static_cast<int>(humans_.size()))
        {
          humans_[current_human_idx_].visited = true;
          RCLCPP_INFO(get_logger(),
            "Reached HUMAN #%d at (%.2f, %.2f).",
            current_human_idx_ + 1,
            humans_[current_human_idx_].x,
            humans_[current_human_idx_].y);

          if (count_visited_humans() >= 2) {
            RCLCPP_INFO(get_logger(),
              "🎯 Both humans visited. Mission complete!");
          }
        } else {
          RCLCPP_INFO(get_logger(),
            "Reached patrol waypoint %d.",
            current_waypoint_idx_);
          current_waypoint_idx_++;
        }
        break;

      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(get_logger(), "NavigateToPose goal ABORTED.");
        if (current_human_idx_ == -1) {
          current_waypoint_idx_++;
        }
        break;

      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(get_logger(), "NavigateToPose goal CANCELED.");
        break;

      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code from Nav2.");
        break;
    }

    current_human_idx_ = -1;
  }

  // ===========================================
  //               HELPER FUNCTIONS
  // ===========================================

  int get_unvisited_human_index() const
  {
    for (size_t i = 0; i < humans_.size(); ++i) {
      if (!humans_[i].visited) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  int count_visited_humans() const
  {
    int c = 0;
    for (const auto & h : humans_) {
      if (h.visited) c++;
    }
    return c;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // --- MRTP Navigator ---
  // debug=false, verbose=false
  Navigator navigator(false, false);

  // --- Create initial pose as geometry_msgs::msg::Pose (NOT PoseStamped!) ---
  auto init_pose = std::make_shared<geometry_msgs::msg::Pose>();

  init_pose->position.x = 2.12;
  init_pose->position.y = -21.3;
  init_pose->position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 1.57);  // yaw = 1.57 radians
  init_pose->orientation = tf2::toMsg(q);

  // --- Set initial pose + wait for Nav2 to become active ---
  navigator.SetInitialPose(init_pose);
  navigator.WaitUntilNav2Active();

  RCLCPP_INFO(rclcpp::get_logger("main"),
              "Initial pose set via MRTP Navigator, Nav2 is active.");

  // --- Launch HumanDetector node ---
  auto node = std::make_shared<HumanDetector>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

