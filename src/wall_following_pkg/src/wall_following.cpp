// std headers
#include <functional>

// third party headers
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// project headers

/// Indicate right or left wall following.
enum class WallToFollow { Left = 0, Right = 1 };

/// The wall following node that will use a laser scan to find the wall.
/// Subscribes to a Scan to detect walls.
/// Publishes Twist objects to drive the robot.
class WallFollowingNode : public rclcpp::Node {
public:
  WallFollowingNode(WallToFollow wall_to_follow)
      : Node("wall_following_node"), wall_to_follow_(wall_to_follow) {
    geometry_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&WallFollowingNode::topic_callback, this,
                  std::placeholders::_1));

    // subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    //     "cmd_vel", 10,
    //     std::bind(&SimpleSubscriber::topic_callback, this,
    //               std::placeholders::_1));
  }

private:
  auto topic_callback(const sensor_msgs::msg::LaserScan msg) -> void {}
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr geometry_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;

  WallToFollow wall_to_follow_;
};

/// This is a simple program that just listens to the laser and drives the
/// robot around the track.
auto main(int argc, char *argv[]) -> int {
  // start ros
  rclcpp::init(argc, argv);
  // spin node
  rclcpp::spin(std::make_shared<WallFollowingNode>(WallToFollow::Left));
  // shutdown
  rclcpp::shutdown();
  return 0;
}