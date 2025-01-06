// std headers

// third party headers
#include "rclcpp/executors.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include <rclcpp/rclcpp.hpp>

// project headers

// So general pattern I have learned so far is:
// init rcl
// spin a node
// shutdown
// Executors seems to have a differnt pattern and I am unser why at thie time.
auto main(int argc, char *argv[]) -> int {
  // init rcl
  rclcpp::init(argc, argv);

  // create a node
  auto node = rclcpp::Node::make_shared("executor_example_1_node");
  RCLCPP_INFO(node->get_logger(), "Bacon pancakes, making bacon pancakes!");

  // add the node to the executor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // spin the executor
  executor.spin();

  // shutdown
  rclcpp::shutdown();
  return 0;
}