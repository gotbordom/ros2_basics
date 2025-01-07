
#include <functional>
#include <math.h>
#include <memory>
#include <thread>

#include "actions_quiz_msg/action/distance.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64.hpp"
#include <rclcpp/qos.hpp>

#define RAD2DEG 57.295779513

class QuizActionServer : public rclcpp::Node {
public:
  using Distance = actions_quiz_msg::action::Distance;
  using GoalHandleDistance = rclcpp_action::ServerGoalHandle<Distance>;

  explicit QuizActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("my_action_server", options) {
    using namespace std::placeholders;

    my_callback_group =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = my_callback_group;

    this->action_server_ = rclcpp_action::create_server<Distance>(
        this, "distance_as",
        std::bind(&QuizActionServer::handle_goal, this, _1, _2),
        std::bind(&QuizActionServer::handle_cancel, this, _1),
        std::bind(&QuizActionServer::handle_accepted, this, _1));

    rclcpp::QoS qos(10);
    qos.keep_last(10);
    qos.best_effort();
    qos.durability_volatile();

    // Create odom subscriber
    odom_subscriber = create_subscription<nav_msgs::msg::Odometry>(
        "odom", qos, std::bind(&QuizActionServer::odomCallback, this, _1),
        options1);

    distance_publisher =
        this->create_publisher<std_msgs::msg::Float64>("total_distance", 10);
  }

private:
  rclcpp_action::Server<Distance>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_publisher;
  rclcpp::CallbackGroup::SharedPtr my_callback_group;

  double previous_x = 0.0;
  double previous_y = 0.0;
  double latest_x = 0.0;
  double latest_y = 0.0;
  double current_x = 0.0;
  double current_y = 0.0;
  double total_distance;
  bool first_run = true;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Distance::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with secs %d",
                goal->seconds);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleDistance> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDistance> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&QuizActionServer::execute, this, _1), goal_handle}
        .detach();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Get x,y robot coordinates
    this->current_x = msg->pose.pose.position.x;
    this->current_y = msg->pose.pose.position.y;
  }

  void measure_dist() {

    if (this->first_run) {
      this->previous_x = this->current_x;
      this->previous_y = this->current_y;
    }

    this->latest_x = this->current_x;
    this->latest_y = this->current_y;

    double d_increment = sqrt(pow(this->latest_x - this->previous_x, 2) +
                              pow(this->latest_y - this->previous_y, 2));

    this->total_distance = this->total_distance + d_increment;
    this->previous_x = this->current_x;
    this->previous_y = this->current_y;
    this->first_run = false;
  }

  void execute(const std::shared_ptr<GoalHandleDistance> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Distance::Feedback>();
    auto &distance_message = feedback->current_dist;
    auto result = std::make_shared<Distance::Result>();
    auto pub = std::make_shared<std_msgs::msg::Float64>();
    rclcpp::Rate loop_rate(1);

    for (int i = 0; (i < goal->seconds) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->status = false;
        result->total_dist = total_distance;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Get measured distance
      measure_dist();
      distance_message = this->total_distance;
      // Publish distance into topic
      pub->data = this->total_distance;
      distance_publisher->publish(*pub);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->status = true;
      result->total_dist = this->total_distance;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
}; // class QuizActionServer

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<QuizActionServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}