// std headers

// third party headers
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

// project headers

class SimpleSubscriber : public rclcpp::Node {
public:
  SimpleSubscriber() : Node("simple_subscriber") {
    // create a subscription
    // Question (AT) while most of this I was able to mostly cobble together,
    // I don't understand why I needed the placeholders::_1
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&SimpleSubscriber::topic_callback, this,
                  std::placeholders::_1));
  }

private:
  auto topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) -> void {
    RCLCPP_INFO(this->get_logger(),
                "I heard: \n\tlinear: \n\t\tx: '%f'.\n\t\ty: '%f'.\n\t\tz: "
                "'%f'.\n\tangular: \n\t\tx: '%f'.\n\t\ty: '%f'.\n\t\tz: '%f'.",
                msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x,
                msg->angular.y, msg->angular.z);
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
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