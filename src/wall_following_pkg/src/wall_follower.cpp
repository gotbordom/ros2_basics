// std headers
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
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// project headers

/// @brief Enum to make code clearer when determining direction of wall
///        following in later code.
enum class WallFollowingDirection
{
  LeftHandSide = 0,
  RightHandSide = 1
};
struct RobotState
{
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
class WallFollowerNode : public rclcpp::Node
{
public:
  WallFollowerNode(WallFollowingDirection wall_to_follow =
                       WallFollowingDirection::LeftHandSide,
                   std::string node_name = "wall_follower_node",
                   std::string subscription_channel = "scan",
                   std::string publisher_channel = "cmd_vel")
      : Node(node_name), wall_to_follow_(wall_to_follow),
        subscription_channel_(subscription_channel),
        publisher_channel_(publisher_channel),
        estimated_orientation_degrees_(0.0)
  {

    // Create pub / sub
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        subscription_channel_, 10,
        std::bind(&WallFollowerNode::topic_callback, this,
                  std::placeholders::_1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        publisher_channel_, 10);

    // TODO - Lol... I need each of these in their own group so that they can
    // run in parallel
    side_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        "debug_side_scan", 10);
    front_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        "debug_front_scan", 10);
  }

private:
  /// @brief   The controller callback in charge of publishing velocity values
  ///          based on the current state of the robot.
  auto controller_callback() -> void {};

  /// @brief   The odometry callback is in charge of determining the previous,
  ///          and current, state of the robot given sensor data
  auto odom_callback() -> void {};

  /// @brief The laser scan callback is tasked with updating the state object with
  ///        the correct Left, Right and Front ranges as well as minimum values for each.
  auto laser_scan_callback() -> void {};

  std::string subscription_channel_, publisher_channel_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

auto main(int argc, char *argv[]) -> int
{
  // init the rcl
  rclcpp::init(argc, argv);
  // spin the node
  rclcpp::spin(std::make_shared<WallFollowerNode>());
  // shutdown the node
  rclcpp::shutdown();
  return 0;
}