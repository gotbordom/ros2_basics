// std headers
#include <chrono>

//  third party headers
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/timer.hpp"
#include "std_srvs/srv/detail/empty__struct.hpp"
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

// project headers

using namespace std::chrono_literals;

class ServerNode : public rclcpp::Node {
public:
  ServerNode() : Node("server_node") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    service_ = this->create_service<std_srv::srv::Empty>(
        "moving", std::bind(&ServerNode::service_callback, this,
                            std::placeholders::_1, std::placeholders::_2));
  }

private:
  // my guess is the timer checks IF there is reqeust?
  // yeah looks like no need for the timer perse
  // auto timer_callback() -> void {}

  //  whereas the service callback also checks for a request?
  // idk perhaps a server, like a subscriber, doesn't actually need a timer to
  // fully function?
  auto service_callback(const std::shared_ptr<Empty::Request> request,
                        const std::shared_ptr<Empty::Response> response)
      -> void {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.2;
    message.angular.z = 0.2;
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
};

auto main(int argc, char *argv[]) -> int {
  // This is service so the node just stays alive - std pattern
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}
