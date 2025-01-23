// std headers
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <string>

// third party headers
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// project headers

using namespace std::chrono_literals;

/// @brief Enum to make code clearer when determining direction of wall
///        following in later code.
enum class WallFollowingDirection { LeftHandSide = 0, RightHandSide = 1 };

struct RangeInfos {
  // Data that should only ever be updated in setup
  // range pairs are [ (inclusive), (exclusive) ]
  std::vector<std::pair<int, int>> front_idxs;
  std::vector<std::pair<int, int>> left_idxs;
  std::vector<std::pair<int, int>> right_idxs;
  std::pair<float, float> front_angle_range;
  std::pair<float, float> left_angle_range;
  std::pair<float, float> right_angle_range;
  // min, max pairs are [min, max]
  std::pair<float, float> angle_rad_min_max;
  std::pair<float, float> range_meter_min_max;
  float angle_rad_increment; // should be the same for all data ranges

  // Data that will be updated every iteration of a state.
  std::vector<float> front_range;
  std::vector<float> left_range;
  std::vector<float> right_range;
  float min_front_range;
  float min_left_range;
  float min_right_range;
};

struct RobotState {
  RangeInfos range_infos;
  geometry_msgs::msg::Pose pose;
  WallFollowingDirection direction_to_follow;
  RangeInfos ranges_info;
  float euclidean_dist_front;
  float euclidean_dist_left;
  float euclidean_dist_right;
};

/// @brief WallFollowerNode class that handles listening to the laser scanner
///        then determining next velocity command, and publishing it to the
///        correct topic.
///        The aim is the have the robot follow the wall at a distance of
///        approximately 0.2 -> 0.3 meters
class WallFollowerNode : public rclcpp::Node {
public:
  WallFollowerNode(std::string publisher_channel = "cmd_vel")
      : Node("wall_follower_node"), laser_scan_setup_done_(false),
        odom_setup_done_(false) {

    callback_group_1_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_3_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = callback_group_1_;
    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&WallFollowerNode::laser_scan_callback, this,
                  std::placeholders::_1),
        options1);

    rclcpp::SubscriptionOptions options2;
    options1.callback_group = callback_group_2_;
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&WallFollowerNode::odom_callback, this,
                  std::placeholders::_1),
        options2);

    controller_timer = this->create_wall_timer(
        1s, std::bind(&WallFollowerNode::controller_callback, this),
        callback_group_3_);

    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  /// @brief   The controller callback in charge of publishing velocity values
  ///          based on the current state of the robot.
  auto controller_callback() -> void {}

  /// @brief   The odometry callback is in charge of determining the previous,
  ///          and current, state of the robot given sensor data
  auto odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) -> void {}

  /// @brief The laser scan callback is tasked with updating the state object
  ///        with the correct Left, Right and Front ranges as well as minimum
  ///        values for each.
  /// @param msg the last published LaserScan object
  /// @note  Unfortunately I can't find documentation on this laser scanner and
  ///        if angle 0 is ranges[0]... angle 360 is ranges[360]

  /// ASSUMPTION:
  /// 1. 0 degrees is forward
  /// 2. 0 degress DIST is stored in msg->ranges[0]
  /// 3. Degrees increment CCW such that msg->ranges[180] would be backward
  ///    Looking at the data with a debugging topic and rviz2, assumptions 1-3
  ///    are true.
  auto laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
      -> void {
    if (!laser_scan_setup_done_) {
      laser_scan_setup_done_ = laser_scan_setup(msg);
    }
  }

  /// @brief   Run the setup for the laser scan callback. This will calculate
  ///          the correct index values, etc for each range and store them.
  /// @return  boolean - if configuration was successful.
  auto laser_scan_setup(const sensor_msgs::msg::LaserScan::SharedPtr msg)
      -> bool {

    curr_state_.range_infos.angle_rad_increment = msg->angle_increment;
    curr_state_.range_infos.angle_rad_min_max.first = msg->angle_min;
    curr_state_.range_infos.angle_rad_min_max.second = msg->angle_max;
    curr_state_.range_infos.range_meter_min_max.first = msg->range_min;
    curr_state_.range_infos.range_meter_min_max.second = msg->range_max;

    curr_state_.range_infos.front_idxs = {{0, 44}, {315, 359}};

    curr_state_.range_infos.left_idxs = {{45, 134}};

    curr_state_.range_infos.right_idxs = {{225, 314}};

    curr_state_.range_infos.right_range = std::vector<float>(
        msg->ranges.begin() + curr_state_.range_infos.right_idxs[0].first,
        msg->ranges.begin() + curr_state_.range_infos.right_idxs[0].second);

    curr_state_.range_infos.left_range = std::vector<float>(
        msg->ranges.begin() + curr_state_.range_infos.left_idxs[0].first,
        msg->ranges.begin() + curr_state_.range_infos.left_idxs[0].second);

    // Get front, left and right range vectors and save start/ end idx for each.
    // for (size_t i = 0; i < msg->ranges.size() l++ i)
    // {
    //   // TODO: If I end up needing to filter out improper data this could be
    //   an
    //   //       optimal time.
    //   float angle = msg->angle_min + i * msg->angle_increment;

    //   // Get range index values
    //   if (angle > PI / 4 && angle < PI * 3 / 4)
    //   {
    //     prev_state_.range_infos.left_range.push_back(msg->ranges[i]);
    //     curr_state_.range_infos.left_range.push_back(msg->ranges[i]);
    //   }

    //   // update right range values
    //   if (angle > PI * 5 / 4 && angle < PI * 7 / 4)
    //   {
    //     prev_state_.range_infos.right_range.push_back(msg->ranges[i]);
    //     curr_state_.range_infos.right_range.push_back(msg->ranges[i]);
    //   }

    //   // update front range values
    //   if (angle < PI / 4 || angle > PI * 7 / 4)
    //   {
    //     prev_state_.range_infos.front_range.push_back(msg->ranges[i]);
    //     curr_state_.range_infos.front_range.push_back(msg->ranges[i]);
    //   }
    // }

    // Save min/max scan angles and ranges

    // Save initial min value per range

    return true;
  }

  RobotState curr_state_, prev_state_;

  // TODO - I want think on who should own these. I don't believe they would be
  // a part of the robot state?
  //        my thought being that they are a part of the robot's controller
  //        state?
  bool laser_scan_setup_done_, odom_setup_done_;

  rclcpp::CallbackGroup::SharedPtr callback_group_1_, callback_group_2_,
      callback_group_3_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr controller_timer;
};

auto main(int argc, char *argv[]) -> int {
  rclcpp::init(argc, argv);

  auto wall_follower_node = std::make_shared<WallFollowerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(wall_follower_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}