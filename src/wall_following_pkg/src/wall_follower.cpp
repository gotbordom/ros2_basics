// std headers
#include <functional>
#include <memory>
#include <string>

// third party headers
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// project headers

/// @brief Enum to make code clearer when determining direction of wall following in later code.
enum class WallFollowingDirection
{
  LeftHandSide = 0,
  RightHandSide = 1
};

/// @brief WallFollowerNode class that handles listening to the laser scanner
///        then determining next velocity command, and publishing it to the correct topic.
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
        publisher_channel_(publisher_channel)
  {

    // Create pub / sub
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        subscription_channel_, 10,
        std::bind(&WallFollowerNode::topic_callback, this,
                  std::placeholders::_1));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        publisher_channel_, 10);

    debug_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("debug_scan", 10);
  }

private:
  /// @brief Topic callback function that will be called everytime this node sees a new message on the @subscriber_channel_
  ///        it will also publish data to the @publisher_channel_ indicating new drive velocities.
  /// @param msg the last published LaserScan object
  auto topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
      -> void
  {
    RCLCPP_INFO(this->get_logger(), "Got range of size: %d\n",
                msg->ranges.size());

    auto ranges = msg->ranges;
    auto total_ranges = msg->ranges.size();

    // ------------- DEBUGGING LASER SCAN MSG ------------------
    // NOTE: Unfortunately I can't find documentation on this laser scanner and if angle 0 is ranges[0]... angle 360 is ranges[360]
    // ASSUMPTION: For now I will assume ranges[0] is distance at theta 0 where directly forward and it all increments CCW
    // TODO (AT) DEBUGGING: I want to know where origin is.
    auto debug_scan_msg = msg.get();
    debug_scan_msg.ranges = debug_scan_msg.ranges[0];
    debug_publisher_.publish(debug_scan_msg);
    // ------------- DEBUGGING LASER SCAN MSG ------------------

    // split ranges into left

    // clean up range data
    auto it = std::remove_if(ranges.begin(), ranges.end(), [&msg](float num)
                             { return (num >= msg.range_max || num <= msg.range_min); });
    ranges.erase(it, ranges.end());
  }

  WallFollowingDirection wall_to_follow_;
  std::string subscription_channel_, publisher_channel_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr debug_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
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