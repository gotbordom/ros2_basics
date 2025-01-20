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

enum class WallFollowingAlgorithm { LeftHandSide = 0, RightHandSide = 1 };

class WallFollowerNode : public rclcpp::Node {
public:
  WallFollowerNode() : Node("wall_follower_node") {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "cmd_vel", 10,
        std::bind(&WallFollowerNode::topic_callback, this,
                  std::placeholders::_1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  auto topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
      -> void {
    RCLCPP_INFO(this->get_logger(), "Got range of size: %d\n",
                msg->ranges.size());
  }

  WallFollowingAlgorithm wall_following_algorithm_;
  std::string subscription_channel_, publisher_channel_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

auto main(int argc, char *argv[]) -> int {
  // init the rcl
  rclcpp::init(argc, argv);
  // spin the node
  rclcpp::spin(std::make_shared<WallFollowerNode>());
  // shutdown the node
  rclcpp::shutdown();
  return 0;
}