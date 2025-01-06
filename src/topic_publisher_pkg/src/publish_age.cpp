// std headers
#include <chrono>

// third party headers
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include <rclcpp/rclcpp.hpp>

// project headers
#include <custom_interfaces/msg/age.hpp>

using namespace std::chrono_literals;

class PublishAge : public rclcpp::Node {
public:
  PublishAge() : Node("age_producer") {
    publisher_ = this->create_publisher<custom_interfaces::msg::Age>("age", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&PublishAge::timer_callback, this));
  }

private:
  auto timer_callback() -> void {
    auto message = custom_interfaces::msg::Age();
    message.years = 2025;
    message.months = 1;
    message.days = 2;

    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_interfaces::msg::Age>::SharedPtr publisher_;
};

auto main(int argc, char *argv[]) -> int {
  // init
  rclcpp::init(argc, argv);
  // spin
  rclcpp::spin(std::make_shared<PublishAge>());
  // shutdown
  rclcpp::shutdown();
  return 0;
}