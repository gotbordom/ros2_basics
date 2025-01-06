// std headers
#include <functional>
#include <memory>

// thrid party headers
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/utilities.hpp"
#include "std_srvs/srv/detail/set_bool__struct.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

// project headers

class Excersize42ServerNode : public rclcpp::Node {
public:
  Excersize42ServerNode() : Node("excersize42_service_node") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    service_ = this->create_service<std_srvs::srv::SetBool>(
        "moving_right",
        std::bind(&Excersize42ServerNode::request_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

private:
  // interesting that we still don't return anything here.
  auto
  request_callback(std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> response)
      -> void {

    // crate our default message
    auto msg = geometry_msgs::msg::Twist();

    // if true we turn right ( negative angular about z )
    if (request->data) {
      msg.linear.x = 0.2;
      msg.angular.z = -0.2;
      publisher_->publish(msg);

      // Set the response success variable to true and message
      response->success = true;
      response->message = "Turning to the right right right!";
    } else {
      msg.linear.x = 0.0;
      msg.angular.z = 0.0;
      publisher_->publish(msg);

      // Set the response success variable to true and message
      response->success = true;
      response->message = "Stopping!";
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};

auto main(int argc, char *argv[]) -> int {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Excersize42ServerNode>());
  rclcpp::shutdown();
  return 0;
}