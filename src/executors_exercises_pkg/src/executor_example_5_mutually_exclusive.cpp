// std headers
#include <chrono>

// third party headers
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
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

class TwoTimer : public rclcpp::Node {
public:
  TwoTimer(float sleep_timer_1, float sleep_timer_2)
      : Node("slow_timer_subscriber"), wait_time_1_(sleep_timer_1),
        wait_time_2_(sleep_timer_2) {

    callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    timer_1_ = this->create_wall_timer(
        500ms, std::bind(&TwoTimer::timer_1_callback, this), callback_group_);
    timer_2_ = this->create_wall_timer(
        500ms, std::bind(&TwoTimer::timer_2_callback, this), callback_group_);
  }

private:
  auto timer_1_callback() -> void {
    RCLCPP_INFO(this->get_logger(), "Timer 1 Callback Start");
    sleep(this->wait_time_1_);
    RCLCPP_INFO(this->get_logger(), "Timer 1 Callback End");
  }
  auto timer_2_callback() -> void {
    RCLCPP_INFO(this->get_logger(), "Timer 2 Callback Start");
    sleep(this->wait_time_2_);
    RCLCPP_INFO(this->get_logger(), "Timer 2 Callback End");
  }
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr timer_1_, timer_2_;
  float wait_time_1_, wait_time_2_;
};

// Another pattern using executors
auto main(int argc, char *argv[]) -> int {
  // init
  rclcpp::init(argc, argv);

  // create nodes
  auto sleep_timer_1 = 1.0;
  auto sleep_timer_2 = 3.0;
  auto two_timer_node =
      std::make_shared<TwoTimer>(sleep_timer_1, sleep_timer_2);

  RCLCPP_INFO(two_timer_node->get_logger(), "two_timer_node INFO...");

  // create the executor ( Curious how multiple callbacks would work with ... a
  // SingleThread vs MultiThread)
  rclcpp::executors::MultiThreadedExecutor executor;
  // add the node to the executor
  executor.add_node(two_timer_node);
  // spin the executor
  executor.spin();

  rclcpp::shutdown();
  return 0;
}