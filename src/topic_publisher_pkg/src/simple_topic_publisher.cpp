// std headers
#include "rclcpp/rate.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/detail/int32__struct.hpp"
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdio.h>

// third party headers
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

// project headers

/*
The ROS2 way - with node composition
*/

using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node {
public:
  SimplePublisher() : Node("simple_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("counter", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&SimplePublisher::timer_callback, this));
  }

private:
  auto timer_callback() -> void {
    auto message = std_msgs::msg::Int32();
    message.data = count_;
    count_++;
    publisher_->publish(message);
  }

  int count_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
};

auto main(int argc, char *argv[]) -> int {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePublisher>());
  rclcpp::shutdown();
  return 0;
}

/*
The ROS1 way - without node composition
*/
// auto main(int argc, char *argv[]) -> int {
//   // start by initing it with hte argc and argv
//   rclcpp::init(argc, argv);

//   // Create my node
//   auto node = rclcpp::Node::make_shared("simple_publisher");

//   // create my publisher with name counter and queue size 10
//   auto publisher = node->create_publisher<std_msgs::msg::Int32>("counter",
//   10);

//   // create my message to send
//   auto message = std::make_shared<std_msgs::msg::Int32>();
//   message->data = 0;

//   // create a wall rate to send it
//   auto loop_rate = rclcpp::WallRate{2};

//   // while we are ok, send the  data
//   while (rclcpp::ok()) {
//     // publish the message
//     // NOTE: Ok so publish has a template for const shared pointers but not
//     non
//     // const... so I need to dereference this.
//     publisher->publish(*message);
//     // update the message
//     message->data++;
//     // Spin the node? doesn't the while loop handle this already?
//     rclcpp::spin_some(node);
//     // Because it needs to be slowed a bit
//     loop_rate.sleep();
//   }

//   rclcpp::shutdown();
//   return 0;
// }
