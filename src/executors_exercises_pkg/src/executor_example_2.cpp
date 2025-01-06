// std headers

// third party headers
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include <functional>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

// project headers

class OdomSubscriber : public rclcpp::Node {
public:
  OdomSubscriber(std::string odom_topic_name) : Node("simple_subscriber") {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_name, 10,
        std::bind(&OdomSubscriber::topic_callback, this,
                  std::placeholders::_1));
  }

private:
  auto topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) -> void {
    RCLCPP_INFO(this->get_logger(), "Odometry = ['%f', '%f', '%f']",
                msg->pose.pose.position.x, msg->pose.pose.position.y,
                msg->pose.pose.position.z);
  }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

// Another pattern using executors
auto main(int argc, char *argv[]) -> int {
  // init
  rclcpp::init(argc, argv);

  // create nodes
  auto node = std::make_shared<OdomSubscriber>("/box_bot_1/odom");
  RCLCPP_INFO(node->get_logger(), "Bacon pancakes, makin bacon pancakes");

  // create the executor
  rclcpp::executors::SingleThreadedExecutor executor;
  // add the node to the executor
  executor.add_node(node);
  // spin the executor
  executor.spin();

  rclcpp::shutdown();
  return 0;
}