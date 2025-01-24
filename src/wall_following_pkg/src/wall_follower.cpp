// std headers
#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <string>

// third party headers
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/logging.hpp"
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
  // NOTE: front is broken into left and because the it is [0, (1/4)pi] and
  // [(7/4)pi, 2pi] instead of handling a discontinuity I will just make two
  // vectors
  std::pair<int, int> idx_front_left;
  std::pair<int, int> idx_front_right;
  std::pair<int, int> idx_left;
  std::pair<int, int> idx_right;
  // min max pairs are [ min, max ]
  std::pair<float, float> angle_rad_min_max_front_left;
  std::pair<float, float> angle_rad_min_max_front_right;
  std::pair<float, float> angle_rad_min_max_left;
  std::pair<float, float> angle_rad_min_max_right;
  // should be the same for all data ranges
  std::pair<float, float> range_meter_min_max;
  float angle_rad_increment;

  // Data that will be updated every iteration of a state.
  std::vector<float> range_front_left;
  std::vector<float> range_front_right;
  std::vector<float> range_left;
  std::vector<float> range_right;

  // keep track of the min value, and its respective angle it was acquired at
  std::pair<float, float> min_front_left;
  std::pair<float, float> min_front_right;
  std::pair<float, float> min_left;
  std::pair<float, float> min_right;
};

struct RobotState {
  RangeInfos range_infos;
  geometry_msgs::msg::Pose pose;
  WallFollowingDirection direction_to_follow;
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
  WallFollowerNode()
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
  auto controller_callback() -> void {
    RCLCPP_INFO(this->get_logger(), "CONTROLLER CALLBACK: got called");
  }

  /// @brief   The odometry callback is in charge of determining the previous,
  ///          and current, state of the robot given sensor data
  auto odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) -> void {
    RCLCPP_INFO(this->get_logger(), "ODOMETRY CALLBACK: got called");
  }

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
      RCLCPP_INFO(this->get_logger(), "LASER SCAN CALLBACK: running setup");
      laser_scan_setup_done_ = laser_scan_setup(msg);
    } else {
      RCLCPP_INFO(this->get_logger(), "LASER SCAN CALLBACK: running non-setup");

      // First make sure to set previous state to the current state.
      // prev_state_ = curr_state_;

      // Then adjust current state range infos to new incoming data.

      // ranges
      curr_state_.range_infos.range_front_left = std::vector<float>(
          msg->ranges.begin() + curr_state_.range_infos.idx_front_left.first,
          msg->ranges.begin() + curr_state_.range_infos.idx_front_left.second);

      curr_state_.range_infos.range_front_right = std::vector<float>(
          msg->ranges.begin() + curr_state_.range_infos.idx_front_right.first,
          msg->ranges.begin() + curr_state_.range_infos.idx_front_right.second);

      curr_state_.range_infos.range_left = std::vector<float>(
          msg->ranges.begin() + curr_state_.range_infos.idx_left.first,
          msg->ranges.begin() + curr_state_.range_infos.idx_left.second);

      curr_state_.range_infos.range_right = std::vector<float>(
          msg->ranges.begin() + curr_state_.range_infos.idx_right.first,
          msg->ranges.begin() + curr_state_.range_infos.idx_right.second);

      // min values and their angles
      auto it = std::min_element(
          msg->ranges.begin() + curr_state_.range_infos.idx_front_left.first,
          msg->ranges.begin() + curr_state_.range_infos.idx_front_left.second);
      auto min_value_angle = std::distance(msg->ranges.begin(), it) *
                             curr_state_.range_infos.angle_rad_increment;
      curr_state_.range_infos.min_front_left = {*it, min_value_angle};

      it = std::min_element(
          msg->ranges.begin() + curr_state_.range_infos.idx_front_right.first,
          msg->ranges.begin() + curr_state_.range_infos.idx_front_right.second);
      min_value_angle = std::distance(msg->ranges.begin(), it) *
                        curr_state_.range_infos.angle_rad_increment;
      curr_state_.range_infos.min_front_right = {*it, min_value_angle};

      it = std::min_element(
          msg->ranges.begin() + curr_state_.range_infos.idx_left.first,
          msg->ranges.begin() + curr_state_.range_infos.idx_left.second);
      min_value_angle = std::distance(msg->ranges.begin(), it) *
                        curr_state_.range_infos.angle_rad_increment;
      curr_state_.range_infos.min_left = {*it, min_value_angle};

      it = std::min_element(
          msg->ranges.begin() + curr_state_.range_infos.idx_right.first,
          msg->ranges.begin() + curr_state_.range_infos.idx_right.second);
      min_value_angle = std::distance(msg->ranges.begin(), it) *
                        curr_state_.range_infos.angle_rad_increment;
      curr_state_.range_infos.min_right = {*it, min_value_angle};

      print_range_infos();
    }
  }

  //  TODO - This needs to be broken into smaller helper functions that can all
  //  get unit tested.
  /// @brief   Run the setup for the laser scan callback. This will calculate
  ///          the correct index values, etc for each range and store them.
  /// @return  boolean - if configuration was successful.
  auto laser_scan_setup(const sensor_msgs::msg::LaserScan::SharedPtr msg)
      -> bool {

    // Set the data we take directly from the msg
    curr_state_.range_infos.angle_rad_increment = msg->angle_increment;
    curr_state_.range_infos.range_meter_min_max.first = msg->range_min;
    curr_state_.range_infos.range_meter_min_max.second = msg->range_max;

    // Set the index values to use for each range.
    // Since each messaage gets a full 360 readings for a single 360 degree
    // rotation, these numbers are easier to use.
    curr_state_.range_infos.idx_front_left = {0, 45};
    curr_state_.range_infos.idx_front_right = {315, 360};
    curr_state_.range_infos.idx_left = {45, 135};
    curr_state_.range_infos.idx_right = {225, 315};

    // Set the radian min/max ranges encase I need to compute the degree of any
    // of my stored range's elements store the radiands start  and end values
    // for each range.
    const float PI = 3.14;
    const float degree_to_rad = PI / 180;
    curr_state_.range_infos.angle_rad_min_max_front_left = {
        degree_to_rad * curr_state_.range_infos.idx_front_left.first,
        degree_to_rad * curr_state_.range_infos.idx_front_left.second};
    curr_state_.range_infos.angle_rad_min_max_front_right = {
        degree_to_rad * curr_state_.range_infos.idx_front_right.first,
        degree_to_rad * curr_state_.range_infos.idx_front_right.second};
    curr_state_.range_infos.angle_rad_min_max_left = {
        degree_to_rad * curr_state_.range_infos.idx_left.first,
        degree_to_rad * curr_state_.range_infos.idx_left.second};
    curr_state_.range_infos.angle_rad_min_max_right = {
        degree_to_rad * curr_state_.range_infos.idx_right.first,
        degree_to_rad * curr_state_.range_infos.idx_right.second};

    // Store ranges
    curr_state_.range_infos.range_front_left = std::vector<float>(
        msg->ranges.begin() + curr_state_.range_infos.idx_front_left.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_front_left.second);

    curr_state_.range_infos.range_front_right = std::vector<float>(
        msg->ranges.begin() + curr_state_.range_infos.idx_front_right.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_front_right.second);

    curr_state_.range_infos.range_left = std::vector<float>(
        msg->ranges.begin() + curr_state_.range_infos.idx_left.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_left.second);

    curr_state_.range_infos.range_right = std::vector<float>(
        msg->ranges.begin() + curr_state_.range_infos.idx_right.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_right.second);

    // Save initial min value per range
    auto it = std::min_element(
        msg->ranges.begin() + curr_state_.range_infos.idx_front_left.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_front_left.second);
    auto min_value_angle = std::distance(msg->ranges.begin(), it) *
                           curr_state_.range_infos.angle_rad_increment;
    curr_state_.range_infos.min_front_left = {*it, min_value_angle};

    it = std::min_element(
        msg->ranges.begin() + curr_state_.range_infos.idx_front_right.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_front_right.second);
    min_value_angle = std::distance(msg->ranges.begin(), it) *
                      curr_state_.range_infos.angle_rad_increment;
    curr_state_.range_infos.min_front_right = {*it, min_value_angle};

    it = std::min_element(
        msg->ranges.begin() + curr_state_.range_infos.idx_left.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_left.second);
    min_value_angle = std::distance(msg->ranges.begin(), it) *
                      curr_state_.range_infos.angle_rad_increment;
    curr_state_.range_infos.min_left = {*it, min_value_angle};

    it = std::min_element(
        msg->ranges.begin() + curr_state_.range_infos.idx_right.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_right.second);
    min_value_angle = std::distance(msg->ranges.begin(), it) *
                      curr_state_.range_infos.angle_rad_increment;
    curr_state_.range_infos.min_right = {*it, min_value_angle};

    print_range_infos();
    return true;
  }

  // TODO - I should just overwrite the struct with an << operator or a print
  // function itself
  /// @brief Print out the current state range info
  auto print_range_infos() -> void {
    // const auto info = curr_state_.range_infos;
    RCLCPP_INFO(this->get_logger(),
                "CURRENT STATE\n"
                "=============\n"
                "LEFT:        SIZE = %d, MIN VALUE = %f, MIN ANGLE = %f\n"
                "RIGHT:       SIZE = %d, MIN VALUE = %f, MIN ANGLE = %f\n"
                "FRONT LEFT:  SIZE = %d, MIN VALUE = %f, MIN ANGLE = %f\n"
                "FRONT RIGHT: SIZE = %d, MIN VALUE = %f, MIN ANGLE = %f\n",
                curr_state_.range_infos.range_left.size(),
                curr_state_.range_infos.min_left.first,
                curr_state_.range_infos.min_left.second,
                curr_state_.range_infos.range_right.size(),
                curr_state_.range_infos.min_right.first,
                curr_state_.range_infos.min_right.second,
                curr_state_.range_infos.range_front_left.size(),
                curr_state_.range_infos.min_front_left.first,
                curr_state_.range_infos.min_front_left.second,
                curr_state_.range_infos.range_front_right.size(),
                curr_state_.range_infos.min_front_right.first,
                curr_state_.range_infos.min_front_right.second);
  }

  RobotState curr_state_, prev_state_;
  // TODO - I want think on who should own these. I don't believe they would be
  //        a part of the robot state? my intial thought being that they are a
  //        part of the robot's controller state?
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