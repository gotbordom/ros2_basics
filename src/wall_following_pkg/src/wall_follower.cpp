// std headers
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
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// project headers

// TODO: Currently neither are "Implemented" the SinglePointPoseEstimate is being run by default.
//       The TODO here is to actually refactor the code so that both either algorithm could be injected
//       into the code. Then write up the second idea and see how it does compared to the first.
/// @brief The aim of this is to break out the code into stages.
///        SinglePointPoseEstimate     is estimating the distance of the robot from
///                                    the wall and obstacles given a single point
///                                    from front_scan data and a single point from
///                                    side_scan data.
///        AveragePointPoseEstimate    is my idea of attempting to determine pose
///                                    based on all the data for either the side_scan
///                                    or the front_scan.
///        SpecificPointsPoseEstimate  There is likely a better name for this. I don't know it.
///                                    I want to track only the points in front that the robot
///                                    if continuing straight could actually drive into.
///                                    Then use those points to determine if there is an obstacle in the way.
enum class WallFollowingPoseEstimation
{
  SinglePointPoseEstimate = 0,
  AveragePointPoseEstimate = 1,
  SpecificPointsPoseEstimate = 2
};

/// @brief Enum to make code clearer when determining direction of wall
///        following in later code.
enum class WallFollowingDirection
{
  LeftHandSide = 0,
  RightHandSide = 1
};

/// @brief WallFollowerNode class that handles listening to the laser scanner
///        then determining next velocity command, and publishing it to the
///        correct topic.
///        The aim is the have the robot follow the wall at a distance of
///        approximately 0.2 -> 0.3 meters
class WallFollowerNode : public rclcpp::Node
{
public:
  WallFollowerNode(WallFollowingDirection wall_to_follow =
                       WallFollowingDirection::LeftHandSide,
                   std::string node_name = "wall_follower_node",
                   std::string subscription_channel = "scan",
                   std::string publisher_channel = "cmd_vel")
      : Node(node_name), wall_to_follow_(wall_to_follow),
        subscription_channel_(subscription_channel),
        publisher_channel_(publisher_channel),
        estimated_orientation_degrees_(0.0)
  {

    // Create pub / sub
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        subscription_channel_, 10,
        std::bind(&WallFollowerNode::topic_callback, this,
                  std::placeholders::_1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        publisher_channel_, 10);

    // TODO - Lol... I need each of these in their own group so that they can
    // run in parallel
    side_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        "debug_side_scan", 10);
    front_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        "debug_front_scan", 10);
  }

private:
  /// @brief Topic callback function that will be called everytime this node
  /// sees a new message on the @subscriber_channel_
  ///        it will also publish data to the @publisher_channel_ indicating new
  ///        drive velocities.
  /// @param msg the last published LaserScan object
  /// @note  Unfortunately I can't find documentation on this laser scanner and
  ///        if angle 0 is ranges[0]... angle 360 is ranges[360]

  /// ASSUMPTION:
  /// 1. 0 degrees is forward
  /// 2. 0 degress DIST is stored in msg->ranges[0]
  /// 3. Degrees increment CCW such that msg->ranges[180] would be backward
  ///    Looking at the data with a debugging topic and rviz2, assumptions 1-3
  ///    are true.
  /// 4. Robot is approx 1m x 1m
  /// 5. I only need to see about as much of the wall being followed as the
  ///    dimensions of the car, and I know we are aiming to stay within 0.2 -
  ///    0.3 meters I can assume that the angle needed to get "enough data"
  ///    would be 90 degrees.
  ///    sin(theta) = O / H
  ///    theta = arcsin( O / H )
  ///    where O = distance from the sensor to the front of the robot
  ///    where H = distance from the sensor to the wall perpendicular to motion
  ///    where theta = 1/2 the angle needed to scan the width of the robot
  ///    where sensor is placed center of mass of the robot (definetly an
  ///    assumption)
  ///    Then O = 0.5m, H = ~ (0.2 + 0.5) -> (0.3 + 0.5) ~= ( 0.8 + 0.7 ) / 2 =
  ///    0.75
  ///    Then arcsin(0.5 / 0.75) ~= 42 * 2 = 84 degrees worth of scanner
  ///    data. Rounded to 90 for simplicity and just extra data.

  /// If I am following the Left Hand Side, then we need data from
  /// [1/4 * pi, 3/4 * pi]
  /// If I am following the Right Hand Side, then we need data from
  /// [5/4 * pi, 7/4 * pi]
  /// Watching for the wall in front, or obstacles, should always be the data in
  /// [0,1/4 * pi] && [7/4 * pi, 2 * pi]
  auto topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
      -> void
  {

    const double PI = 3.14;
    auto side_scan = *msg;
    auto front_scan = *msg;

    // NOTE: Logic here uses ! because I want to set everything but
    // those regions to inf.
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
      float angle = msg->angle_min + i * msg->angle_increment;

      // Get distances to objects on the side we are following
      if (wall_to_follow_ == WallFollowingDirection::LeftHandSide)
      {
        if (!(angle > PI / 4 && angle < PI * 3 / 4))
        {
          side_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
        }
      }
      else if (wall_to_follow_ == WallFollowingDirection::RightHandSide)
      {
        if (!(angle > PI * 5 / 4 && angle < PI * 7 / 4))
        {
          side_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
        }
      }

      // Get distances to objects in front
      if (!(angle < PI / 4 || angle > PI * 7 / 4))
      {
        front_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      }
    }

    // Publish raw debugging output
    side_scan_publisher_->publish(side_scan);
    front_scan_publisher_->publish(front_scan);

    // TODO For optimization, I can now run filtering BEFORE splitting
    // my data into two datasets now that I know it is actually getting the
    // expected ranges. ( This would remove two for loops over 360+ data points)
    for (size_t i = 0; i < side_scan.ranges.size(); ++i)
    {
      // First remove the data if it is ourside the bounds of the hardware
      // per the published message.
      auto value_to_check = side_scan.ranges[i];
      auto value_nan = std::isnan(value_to_check);
      auto value_out_of_bounds = value_to_check < side_scan.range_min ||
                                 value_to_check > side_scan.range_max;
      if (!value_nan && value_out_of_bounds)
      {
        side_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      }
    }

    for (size_t i = 0; i < front_scan.ranges.size(); ++i)
    {
      // First remove the data if it is ourside the bounds of the hardware
      // per the published message.
      auto value_to_check = front_scan.ranges[i];
      auto value_nan = std::isnan(value_to_check);
      auto value_out_of_bounds = value_to_check < side_scan.range_min ||
                                 value_to_check > side_scan.range_max;
      if (!value_nan && value_out_of_bounds)
      {
        front_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      }
    }

    // TODO - Another optimization would be to do this ahead of time once i know
    // it all works. for now though I am still testing so I want all the data to
    // technically exist until I really don't need it.
    // Now lets actually erase all nan values.
    side_scan.ranges.erase(
        std::remove_if(side_scan.ranges.begin(), side_scan.ranges.end(),
                       [](float val)
                       { return std::isnan(val); }),
        side_scan.ranges.end());
    front_scan.ranges.erase(
        std::remove_if(front_scan.ranges.begin(), front_scan.ranges.end(),
                       [](float val)
                       { return std::isnan(val); }),
        front_scan.ranges.end());

    auto min_meters_from_wall = 0.2;
    auto max_meters_from_wall = 0.3;
    auto heading_velocities = geometry_msgs::msg::Twist();

    heading_velocities.linear.x = 0.01; // TODO: Make these parameters
    auto turning_velocity = 0.01;       // TODO: Make these parameters
    turning_velocity = wall_to_follow_ == WallFollowingDirection::LeftHandSide
                           ? -1 * turning_velocity
                           : turning_velocity;

    // This is where I am determining distance reading then translating it into a "pose estimate"
    // by updating how far I am from the wall and /or obstacles
    auto raw_side_scan_data = side_scan.ranges[side_scan.ranges.size() / 2];
    estimated_pose_from_wall_meters_ = raw_side_scan_data * std::cos(estimated_orientation_degrees_);

    // Am I in bounds
    auto driving_above_min_bound =
        estimated_pose_from_wall_meters_ > min_meters_from_wall;
    auto dirving_below_max_bound =
        estimated_pose_from_wall_meters_ < max_meters_from_wall;

    // Either we are in bounds or we are turning to get in bounds
    if (driving_above_min_bound && dirving_below_max_bound)
    {
      heading_velocities.angular.z =
          0.0; // just ensure that we don't need to turn
    }
    // To close to wall turn away
    else if (dirving_below_max_bound)
    {
      heading_velocities.angular.z = turning_velocity;
      estimated_orientation_degrees_ += turning_velocity;
    }
    // To far from wall turn towards
    else if (driving_above_min_bound)
    {
      heading_velocities.angular.z = -1 * turning_velocity;
      estimated_orientation_degrees_ += turning_velocity;
    }

    // Now that we are following the wall, let's see if we need to override data
    // to avoid the wall in front of us
    publisher_->publish(heading_velocities);
  }

  float estimated_orientation_degrees_, estimated_pose_from_wall_meters_, estimated_pose_from_obstacle_meters_;
  WallFollowingDirection wall_to_follow_;
  std::string subscription_channel_, publisher_channel_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

  // DEBUGGING OUTPUT
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
      side_scan_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
      front_scan_publisher_;
};

auto main(int argc, char *argv[]) -> int
{
  // init the rcl
  rclcpp::init(argc, argv);
  // spin the node
  rclcpp::spin(std::make_shared<WallFollowerNode>());
  // shutdown the node
  rclcpp::shutdown();
  return 0;
}