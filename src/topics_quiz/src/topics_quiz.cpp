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

class TopicsQuizNode : public rclcpp::Node {
public:
  TopicsQuizNode() : Node("topics_quiz_node") {
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&TopicsQuizNode::topic_callback, this,
                  std::placeholders::_1));
  }

private:
  auto topic_callback(const sensor_msgs::msg::LaserScan msg) -> void {
    // Check my subscription
    // 1. If front > 1 meter drive forward
    // 2. If right > 1 meter turn left
    // 3. If  left > 1 meter turn right
    // For starters there are traps in this that you can get stuck but thats
    // ok I am just solving what is asked.

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

    // Calculate sizes for each vector
    int size = ranges.size();
    int partition1size = size / 3;
    int partition2Size = size / 3;
    // int partition3Size = size - partition1size - partition2Size;

    // Create new vectors assuming ranges are stored left->right
    // as in ranges[0] == left most
    auto left_scan_ranges =
        std::vector<double>(ranges.begin(), ranges.begin() + partition1size);
    auto center_scan_ranges =
        std::vector<double>(ranges.begin() + partition1size,
                            ranges.begin() + partition1size + partition2Size);
    auto right_scan_ranges = std::vector<double>(
        ranges.begin() + partition1size + partition2Size, ranges.end());

    // Approach 0: Just don't require turning right. Always turn left. Removes
    // an if  statement not a full solution though.
    // auto right_scan =
    //     *std::min_element(right_scan_ranges.begin(),
    //     right_scan_ranges.end());
    // auto center_scan =
    //     *std::min_element(center_scan_ranges.begin(),
    //     center_scan_ranges.end());
    // auto left_scan =
    //     *std::min_element(left_scan_ranges.begin(), left_scan_ranges.end());

    // Approach 1: Taking the min of each range gets me trapped in a attempt to
    // turn left or right
    auto left_scan =
        *std::min_element(left_scan_ranges.begin(), left_scan_ranges.end());
    auto center_scan =
        *std::min_element(center_scan_ranges.begin(), center_scan_ranges.end());
    auto right_scan =
        *std::min_element(right_scan_ranges.begin(), right_scan_ranges.end());

    // Approach 2:  Taking the average - Doesn't  really work easily.
    // auto right_scan = std::accumulate(right_scan_ranges.begin(),
    //                                   right_scan_ranges.end(), 0.0) /
    //                   right_scan_ranges.size();
    // auto center_scan = std::accumulate(center_scan_ranges.begin(),
    //                                    center_scan_ranges.end(), 0.0) /
    //                    center_scan_ranges.size();
    // auto left_scan = std::accumulate(left_scan_ranges.begin(),
    //                                  left_scan_ranges.end(), 0.0) /
    //                   left_scan_ranges.size();

    // Approach 3: Turn until we no longer have values in the left/ right ranges
    // below 1m.
    // TODO (AT) Haven't implemented this yet.

    // So dont over think it. Just finish the three asked for cases
    // Note: While this technically meets needs  for THIS sphere, if we had TWO
    // spheres close enough, we wouldn't be able to drive through as we stop to
    // turn. Instead of driving AND turning. Though  I think the instructions
    // wanted Stop to turn, then drive? butcouldn't quite tell.
    // If we were allowed to drive and turn... then make the default msg drive
    // forward. Then the if change to be If forward is <=1 linear.x = 0.0 and
    // the rest of the else / else if's become just if statements.
    // I might do this anyway.

    // Controller approach 1: Drive OR Turn not both at the same time.
    if (center_scan > 1 && left_scan > 1 && right_scan > 1) {
      // Publish message forward
      cmd_vel_msg.linear.x = 0.5;
    } else if (left_scan <= 1) {
      // turn right
      cmd_vel_msg.angular.z = -0.5;
    } else {
      // stop driving and default turn left.
      // this also covers the case that an object is on my right side.
      cmd_vel_msg.angular.z = 0.5;
    }

    // Controller Approach 2: Drive and/or turn.
    // TODO (AT) By default always drive forward

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
  rclcpp::spin(std::make_shared<TopicsQuizNode>());
  // shutdown the node
  rclcpp::shutdown();
  return 0;
}