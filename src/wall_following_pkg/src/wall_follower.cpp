// std headers
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <string>

// third party headers
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// project headers

using std::chrono_literals;

/// @brief Enum to make code clearer when determining direction of wall
///        following in later code.
enum class WallFollowingDirection { LeftHandSide = 0, RightHandSide = 1 };

struct RobotState {
  geometry_msgs::msg::Pose pose;
  WallFollowingDirection direction_to_follow;
  std::vector<float> front_range;
  std::vector<float> left_range;
  std::vector<float> right_range;
  float min_front_range;
  float min_left_range;
  float min_right_range;
  float euclidean_dist_front;
  float euclidean_dist_left;
  float euclidean_dist_right;
};

/// @brief WallFollowerNode class that handles listening to the laser scanner
///        then determining next velocity command, and publishing it to the
///        correct topic.
///        The aim is the have the robot follow the wall at a distance of
///        approximately 0.2 -> 0.3 meters
class WallFollowerNode : public rclcpp::Node {
public:
  WallFollowerNode(std::string publisher_channel = "cmd_vel")
      : Node("wall_follower_node") {

    callback_group_1_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_3_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&WallFollowerNode::laser_scan_callback, this,
                  std::placeholders::_1),
        callback_group_1_);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&WallFollowerNode::odom_callback, this,
                  std::placeholders::_1),
        callback_group_2_);

    controller_timer = this->create_wall_timer(
        1s, std::bind(&WallFollowerNode::controller_callback, this),
        callback_group_3_);

    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  /// @brief   The controller callback in charge of publishing velocity values
  ///          based on the current state of the robot.
  auto controller_callback() -> void{};

  /// @brief   The odometry callback is in charge of determining the previous,
  ///          and current, state of the robot given sensor data
  auto odom_callback() -> void{};

  /// @brief The laser scan callback is tasked with updating the state object
  /// with
  ///        the correct Left, Right and Front ranges as well as minimum values
  ///        for each.
  auto laser_scan_callback() -> void{};

  RobotState curr_state, prev_state_;

  rclcpp::CallbackGroup::SharedPtr callback_group_1_, callback_group_2_,
      callback_group_3_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr controller_timer;
};

auto main(int argc, char *argv[]) -> int {
  rclcpp::init(argc, argv);

  auto wall_follower_node = std::make_shared<WallFollowerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(wall_follower_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}