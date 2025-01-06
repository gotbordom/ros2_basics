// std headers
#include <algorithm>
#include <chrono>
#include <memory>
#include <numeric>
#include <vector>

// third party headers
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/create_publisher.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// project headers

using namespace std::chrono_literals;

class MoveRobotNode : public rclcpp::Node {
public:
  MoveRobotNode() : Node("move_robot_node") {
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&MoveRobotNode::topic_callback, this, std::placeholders::_1));
  }

private:
  auto topic_callback(const sensor_msgs::msg::LaserScan msg) -> void {
    // Check my subscription
    // 1. If front > 1 meter drive forward
    // 2. If right > 1 meter turn left
    // 3. If  left > 1 meter turn right
    // For starters there are traps in this that you can get stuck but thats
    // ok I am just solving what is asked.
    // Assumption 1 - I forgot how I get the sub data to the callback maybe
    // Placeholder?
    // Assumption 2 - first msg.range[0] is right most scan, mid is
    // forward, last if left?
    //   This assumption comes form looking at the data without googling or
    //   researching how it works

    auto cmd_vel_msg = geometry_msgs::msg::Twist();
    // auto ranges = std::vector<float>(
    //     msg.ranges, msg.ranges + sizeof(msg.ranges) / sizeof(msg.ranges[0]));
    auto ranges = msg.ranges;

    // Throw out data that isn't valid
    auto it = std::remove_if(ranges.begin(), ranges.end(), [&msg](float num) {
      return (num >= msg.range_max || num <= msg.range_min);
    });

    // Resize the vector to remove the elements at the end
    ranges.erase(it, ranges.end());

    // Split data  into left, center, and right ranges
    auto one_third = (ranges.size() - 1) / 3;
    auto right_scan_ranges =
        std::vector<float>(ranges.begin(), ranges.begin() + one_third);

    auto center_scan_ranges = std::vector<float>(ranges.begin() + one_third,
                                                 ranges.end() - one_third);
    auto left_scan_ranges =
        std::vector<float>(ranges.end() - one_third, ranges.end());

    // TODO (AT) This isn't perfect. This has many holes and should
    // instead perhaps an average / mean or median of some subset of
    // the data. Finish the simple solution using this first.
    auto left_scan =
        *std::min_element(left_scan_ranges.begin(), left_scan_ranges.end());

    auto right_scan =
        *std::min_element(right_scan_ranges.begin(), right_scan_ranges.end());
    ;
    auto center_scan =
        *std::min_element(center_scan_ranges.begin(), center_scan_ranges.end());
    ;

    // So dont over think it. Just finish the three asked for cases
    if (center_scan > 1 && left_scan > 1 && right_scan > 1) {
      // Publish message forward
      cmd_vel_msg.linear.x = 0.5;
    } else if (left_scan <= 1) {
      // stop driving and turn right
      cmd_vel_msg.angular.z = -0.5;
    } else {
      // stop driving and turn left ( basically all cases )
      cmd_vel_msg.angular.z = 0.5;
    }

    // NOTE: This still has bad boundary conditions.
    // NOTE: I don't believe I  am actually using all the range data. Likely
    // have an off by one.. or more.
    cmd_vel_publisher_->publish(cmd_vel_msg);
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
};

auto main(int argc, char *argv[]) -> int {
  // initilize ros
  rclcpp::init(argc, argv);
  // spin the node
  rclcpp::spin(std::make_shared<MoveRobotNode>());
  // shutdown the node
  rclcpp::shutdown();
  return 0;
}