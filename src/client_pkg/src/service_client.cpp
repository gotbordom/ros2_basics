// std headers
#include <chrono>

// third party headers
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

// project headers

using namespace std::chrono_literals;

auto main(int argc, char *argv[]) -> int {

  // init rclcpp
  rclcpp::init(argc, argv);
  // create nodes
  // this is a simple example so just a basic node
  auto node = std::make_shared<rclcpp::Node>("client_node");
  // create service
  auto client = node->create_client<std_srvs::srv::Empty>("moving");
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();

  // NOTE: So... for a client we don't create an executor and spin that way?
  // I guess I will see as we go, but I don't know why we wouldn't.
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "service not available, waiting again...");
  }

  auto result_future = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result_future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The robot is moving");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service /moving");
  }

  // shutdown
  rclcpp::shutdown();
  return 0;
}