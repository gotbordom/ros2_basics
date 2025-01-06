
// std headers
#include <functional>
#include <memory>

// third party headers
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/utilities.hpp"
#include <custom_interfaces/srv/my_custom_service_message.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

using MyCustomServiceMessage = custom_interfaces::srv::MyCustomServiceMessage;

class MovementServer : public rclcpp::Node {
public:
  MovementServer() : Node("movement_server") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    service_ = this->create_service<MyCustomServiceMessage>(
        "movement", std::bind(&MovementServer::request_callback, this,
                              std::placeholders::_1, std::placeholders::_2));
  }

private:
  auto
  request_callback(std::shared_ptr<MyCustomServiceMessage::Request> request,
                   std::shared_ptr<MyCustomServiceMessage::Response> response)
      -> void {

    auto msg = geometry_msgs::msg::Twist();
    // NOTE: This is case sensitive
    // ideally this wouldn't even be a string but if it is we should wrap the
    // string check in an std::lower or std::upper to enforce that the important
    // part is the command not the string literal. i.e TuRn RiGhT == Turn Right.
    if (request->move == "Turn right") {
      // publish a message for turning right
      msg.linear.x = 0.2;
      msg.angular.z = -0.2;
      publisher_->publish(msg);

      response->success = false;
    } else if (request->move == "Turn left") {
      msg.linear.x = 0.2;
      msg.angular.z = 0.2;
      publisher_->publish(msg);

      response->success = false;
    } else if (request->move == "Stop") {
      msg.linear.x = 0.0;
      msg.angular.z = 0.0;
      publisher_->publish(msg);

      response->success = false;
    } else {
      // for now don't publish a new change, just let it keep doing what it was
      // doing

      // invalid request
      RCLCPP_ERROR(this->get_logger(), "Invalid input.");
      response->success = false;
    }
  }
  rclcpp::Service<MyCustomServiceMessage>::SharedPtr service_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

auto main(int argc, char *argv[]) -> int {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MovementServer>());
  rclcpp::shutdown();
  return 0;
}