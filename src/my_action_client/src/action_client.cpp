// std headers
#include "rclcpp/client.hpp"
#include "rclcpp/subscription.hpp"
#include <chrono>

// third party headers
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <t3_action_msg/action/move.hpp>

class MyActionClient : public rclcpp::Node {
public:
  MyActionClient()
      : Node("my_action_client"), goal_done_(false), goal_sent_(false) {
    client_ = this->create_client</* TODO */>("t3_action_server");
    subscription_ = this->create_subscription</* TODO */, typename CallbackT>(
        const std::string &topic_name, 10,
        std::bind(&MyActionClient::subscription_callback, this,
                  std::placeholders::_1));
  }

  auto is_goal_done() -> bool { return goal_done_; }

private:
  auto response_callback() -> void {}
  auto subscription_callback(/* TODO Fill in based on sub msg type */) -> void {
  }

  auto async_send_goal() -> void {}

  rclcpp::Client</* TODO Fill this in */>::SharedPtr client_;
  rclcpp::Subscription</*TODO Fill this in*/>::SharedPtr subscription_;
  bool goal_done_;
  bool goal_sent_;
}

auto main(int argc, char *argv[]) -> int {

  rclcpp::shutdown();
  return 0;
}
