// std headers
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/detail/int32__struct.hpp"
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdio.h>

// third party headers
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

// project headers

using namespace std::chrono_literals;

// Goal: Publish a message to the /cmd_vel topic
class MoveRobotPublisher : public rclcpp::Node {
public:
  MoveRobotPublisher() : Node("move_robot_publisher") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->timer_ = this->create_wall_timer(
        500ms, std::bind(&MoveRobotPublisher::timer_callback, this));
  }

private:
  auto timer_callback() -> void {
    auto message = geometry_msgs::msg::Twist();

    // We are just driving in a circle so we can just publish the same
    // message over and over.
    message.linear.x = 0.5;
    message.linear.y = 0.0;
    message.linear.z = 0.0;

    message.angular.x = 0.0;
    message.angular.y = 0.0;
    message.angular.z = 0.5;

    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

//
auto main(int argc, char *argv[]) -> int {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveRobotPublisher>());
  rclcpp::shutdown();
  return 0;
}