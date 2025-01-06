// std headers
#include <chrono>

// third party headers
#include "rclcpp/executors.hpp"
#include "rclcpp/future_return_code.hpp"
#include "std_srvs/srv/detail/empty__struct.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

// project headers

using namespace std::chrono_literals;

auto main(int argc, char *argv[]) -> int {
  rclcpp::init(argc, argv);

  // creating a simple client to interact with /stop
  auto stop_node = std::make_shared<rclcpp::Node>("stop_client_node");
  auto client = stop_node->create_client<std_srvs::srv::Empty>("stop");

  // now check loop until we have access to the service
  while (!client->wait_for_service(1s)) {
    //  While we are waiting we need to ensure that we handle an attempted
    //  shutdown
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "service not available, waiting again...");
  }

  // now that we have access to the service we can await a request
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto response_future = client->async_send_request(request);

  // spin until futur is complete
  if (rclcpp::spin_until_future_complete(stop_node, response_future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto response = response_future.get();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The robot is moving");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service /moving");
  }
  return 0;
}