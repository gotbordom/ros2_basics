// std headers
#include <chrono>

// third party headers
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include <functional>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

// project headers

using namespace std::chrono_literals;

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

class SlowTimer : public rclcpp::Node {
public:
  SlowTimer(float sleep_timer)
      : Node("slow_timer_subscriber"), wait_time_(sleep_timer) {
    timer_ = this->create_wall_timer(
        500ms, std::bind(&SlowTimer::timer_callback, this));
  }

private:
  auto timer_callback() -> void {
    sleep(this->wait_time_);
    RCLCPP_INFO(this->get_logger(), "TICK");
  }
  rclcpp::TimerBase::SharedPtr timer_;
  float wait_time_;
};

// Another pattern using executors
auto main(int argc, char *argv[]) -> int {
  // init
  rclcpp::init(argc, argv);

  // create nodes
  auto odom_subs_node = std::make_shared<OdomSubscriber>("/box_bot_1/odom");
  auto sleep_timer = 3.0;
  auto slow_timer_node = std::make_shared<SlowTimer>(sleep_timer);

  RCLCPP_INFO(odom_subs_node->get_logger(), "odom_subs_node INFO...");
  RCLCPP_INFO(slow_timer_node->get_logger(), "slow_timer_node INFO...");

  // create the executor
  rclcpp::executors::SingleThreadedExecutor executor;
  // add the node to the executor
  executor.add_node(odom_subs_node);
  executor.add_node(slow_timer_node);
  // spin the executor
  executor.spin();

  rclcpp::shutdown();
  return 0;
}