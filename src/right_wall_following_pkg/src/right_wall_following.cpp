// std headers

// third party headers
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/utilities.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

// project headers

class WallFollowingNode : public rclcpp::Node {
public:
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist> publisher_;
};

/// This is a simple program that just listens to the laser and drives the
/// robot around the track.
auto main(int argc, char *argv[]) -> int {
  // start ros
  rclcpp::init(argc, argv);
  // spin node
  rclcpp::spin();
  // shutdown
  rclcpp::shutdown();
  return 0;
}