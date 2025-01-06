// std headers
#include <chrono>

// third party headers
#include "rclcpp/client.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/utilities.hpp"
#include "std_srvs/srv/detail/empty__struct.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

// project headers

using namespace std::chrono_literals;

class ServiceClientV2 : public rclcpp::Node {
public:
  ServiceClientV2() : Node("serivce_client_v2") {
    client_ = this->create_client<std_srvs::srv::Empty>("moving");
    timer_ = this->create_wall_timer(
        1s, std::bind(&ServiceClientV2::timer_callback, this));
  }

  auto is_service_done() const -> int { return this->service_done_; }

private:
  // callback methods
  // so in the service / client pattern it is ideal to have callbacks for
  // timers, controlling making reuqests to the clients, and response callbacks
  // for handling service response objects ?
  // Whereas the pub/ sub pattern seems to have timer and subscriber callbacks,
  // where mainly the sub_callback is used to handle data similar to the
  // response_callback
  auto timer_callback() -> void {
    if (!service_called_) {
      RCLCPP_INFO(this->get_logger(), "Send Async Request");
      async_send_request();
    } else {
      RCLCPP_INFO(this->get_logger(), "Timer Callback Executed");
    }
  }

  auto
  response_callback(rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future)
      -> void {
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

    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result_future = client_->async_send_request(
        request, std::bind(&ServiceClientV2::response_callback, this,
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

  // members
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool service_done_ = false;
  bool service_called_ = false;
};

// Still no executor for the server->client patterns?
auto main(int argc, char *argv[]) -> int {
  rclcpp::init(argc, argv);
  // create the node
  auto service_client = std::make_shared<ServiceClientV2>();
  // spin the node - intersting though, why spin_some?
  while (!service_client->is_service_done()) {
    rclcpp::spin_some(service_client);
  }
  // shutdown
  rclcpp::shutdown();
  return 0;
}