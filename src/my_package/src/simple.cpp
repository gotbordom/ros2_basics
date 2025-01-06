// std headers

// thrid party headers
#include "rclcpp/executors.hpp"
#include <rclcpp/rclcpp.hpp>

auto main(int argc, char *argv[]) -> int {
  // init ros2 comms
  rclcpp::init(argc, argv);

  // create ros2 node ObiWan
  auto node = rclcpp::Node::make_shared("ObiWan");

  // Create a rate object of 2hz
  rclcpp::WallRate loop_rate(2);

  // Question (AT) Isn't this also acheivable with rclcpp::spin() instead of
  // a hardcoded loop?
  while (rclcpp::ok()) {
    // Ros logger
    // Question (AT) about the ros client lib logger.
    // I need to read more about this logger and how the default nodes use them.
    // Seems really nice to have all of this rolled for me, but does it actually
    // do everything I need a logger to do?
    // - write data/logs to a live file of choice?
    // - thread safe?
    // - etc?
    RCLCPP_INFO(node->get_logger(), "Help me ObiWan, you're my only hope!");
    rclcpp::spin_some(node);

    // Sleep for a given rate
    loop_rate.sleep();
  }
  // Shutdown
  rclcpp::shutdown();
  return 0;
}