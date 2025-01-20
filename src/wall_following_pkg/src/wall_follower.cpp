// std headers
#include <functional>
#include <memory>

// third party headers
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// project headers

class SimpleSubscriber : public rclcpp::Node {
public:
  SimpleSubscriber() : Node("simple_subscriber") {
    // create a subscription
    // Question (AT) while most of this I was able to mostly cobble together,
    // I don't understand why I needed the placeholders::_1
    // Answer: They are placeholders for arguments being passed to the callback
    // function.
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "cmd_vel", 10,
        std::bind(&SimpleSubscriber::topic_callback, this,
                  std::placeholders::_1));
  }

private:
  auto topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
      -> void {
    RCLCPP_INFO(this->get_logger(), "DEBUGGING");
  }
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

auto main(int argc, char *argv[]) -> int {
  // init the rcl
  rclcpp::init(argc, argv);
  // spin the node
  rclcpp::spin(std::make_shared<SimpleSubscriber>());
  // shutdown the node
  rclcpp::shutdown();
  return 0;
}