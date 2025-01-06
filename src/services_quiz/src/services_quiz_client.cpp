// std headers
#include <chrono>

// thrid party headers
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/timer.hpp"
#include <rclcpp/rclcpp.hpp>
#include <services_quiz_srv/srv/spin.hpp>

using namespace std::chrono_literals;
using Spin = services_quiz_srv::srv::Spin;

class ServicesQuizClientNode : public rclcpp::Node {
public:
  ServicesQuizClientNode() : Node("services_quiz_client_node") {
    client_ = this->create_client<Spin>("rotate");
    timer_ = this->create_wall_timer(
        1s, std::bind(&ServicesQuizClientNode::timer_callback, this));
  }

  auto is_service_done() const -> bool { return this->service_done_; }

private:
  auto timer_callback() -> void {
    if (!service_called_) {
      RCLCPP_INFO(this->get_logger(), "Send Async Request");
      async_send_request();
    } else {
      RCLCPP_INFO(this->get_logger(), "Timer Callback Executed");
    }
  }

  auto response_callback(rclcpp::Client<Spin>::SharedFuture future) -> void {
    // Get response value
    // Granted it will remain unused and likely cause a warn
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Response: success");
    service_done_ = true;
  }

  // helper methods
  auto async_send_request() -> void {
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Client interrupted while waiting for service. Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service Unavailable. Waiting for Service...");
    }

    auto request = std::make_shared<Spin::Request>();

    request->direction = "right";
    request->angular_velocity = 0.2;
    request->time = 10;

    auto result_future = client_->async_send_request(
        request, std::bind(&ServicesQuizClientNode::response_callback, this,
                           std::placeholders::_1));
    service_called_ = true;

    // Now check for the response after a timeout of 1 second
    // This looks like it is a timeout? but what if our response takes longer
    // than 1s... In this instance do we  only get a response not ready  yet.
    // Then we ...  try it all again?
    auto status = result_future.wait_for(1s);
    if (status != std::future_status::ready) {

      RCLCPP_WARN(this->get_logger(), "Response not ready yet.");
    }
  }

  rclcpp::Client<Spin>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool service_done_ = false;
  bool service_called_ = false;
};

auto main(int argc, char *argv[]) -> int {
  rclcpp::init(argc, argv);
  auto service_client = std::make_shared<ServicesQuizClientNode>();

  while (!service_client->is_service_done()) {
    rclcpp::spin_some(service_client);
  }

  rclcpp::shutdown();
  return 0;
}