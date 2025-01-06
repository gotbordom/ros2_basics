// std headers
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include <chrono>
#include <functional>
#include <memory>

// third party headers
#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <services_quiz_srv/srv/spin.hpp>

using namespace std::chrono_literals;
using Spin = services_quiz_srv::srv::Spin;

class ServicesQuizServerNode : public rclcpp::Node {
public:
  ServicesQuizServerNode() : Node("service_quiz_server_node") {
    service_ = this->create_service<Spin>(
        "rotate", std::bind(&ServicesQuizServerNode::request_callback, this,
                            std::placeholders::_1, std::placeholders::_2));
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  auto request_callback(std::shared_ptr<Spin::Request> request,
                        std::shared_ptr<Spin::Response> response) -> void {
    // solution: easy - emit message one with direction and speed, sleep for
    // time, emit message to stop
    // solution: med  - create a wall timer, that emits a stop_cmd every
    // request->time

    auto drive_msg = geometry_msgs::msg::Twist();
    if (request->direction == "right") {
      drive_msg.angular.z = -1.0 * request->angular_velocity;
      publisher_->publish(drive_msg);
    } else if (request->direction == "left") {
      drive_msg.angular.z = request->angular_velocity;
      publisher_->publish(drive_msg);
    } else {
      // In this case the wrong data was sent and we need to exit
      response->success = false;
      return;
    }

    // sleep
    sleep(request->time);

    auto stop_msg = geometry_msgs::msg::Twist();
    stop_msg.angular.z = 0.0;
    publisher_->publish(stop_msg);

    response->success = true;
  }

  rclcpp::Service<Spin>::SharedPtr service_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

auto main(int argc, char *argv[]) -> int {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServicesQuizServerNode>());
  rclcpp::shutdown();
  return 0;
}