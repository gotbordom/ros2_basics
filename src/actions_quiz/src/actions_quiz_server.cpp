// std headers
#include <chrono>

// third party headers
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_action/server.hpp"
#include "std_msgs/msg/detail/float64__struct.hpp"
#include <actions_quiz_msg/action/distance.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64.hpp>

class ActionQuizServer : public rclcpp::Node {
public:
  using Distance = actions_quiz_msg::action::Distance;
  using GoalHandleDistance = rclcpp_action::ServerGoalHandle<Distance>;

  explicit ActionQuizServer(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("action_quiz_server", node_options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Distance>(
        this, "distance_as",
        std::bind(&ActionQuizServer::handle_goal, this, _1, _2),
        std::bind(&ActionQuizServer::handle_cancel, this, _1),
        std::bind(&ActionQuizServer::handle_accepted, this, _1));

    feedback_publisher_ =
        this->create_publisher<std_msgs::msg::Float64>("total_distance", 10);
  }

private:
  auto handle_goal(const rclcpp_action::GoalUUID &uuid,
                   std::shared_ptr<const Distance::Goal> goal)
      -> rclcpp_action::GoalResponse {
    RCLCPP_INFO(this->get_logger(), "Received goal request with secs %d",
                goal->seconds);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  auto handle_cancel(const std::shared_ptr<GoalHandleDistance> goal_handle)
      -> rclcpp_action::CancelResponse {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  auto handle_accepted(const std::shared_ptr<GoalHandleDistance> goal_handle)
      -> void {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&ActionQuizServer::execute, this, _1), goal_handle}
        .detach();
  }

  // TODO Bulk of actual work, the rest is mostly boiler plate
  auto execute(const std::shared_ptr<GoalHandleDistance> goal_handle) -> void {
    // execute the goal and prep all the data
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Distance::Feedback>();
    auto result = std::make_shared<Distance::Result>();

    // Getting everything initialized
    auto publisher_msg = std_msgs::msg::Float64();
    publisher_msg.data = 0.0;
    feedback->current_dist = 0.0;
    result->total_dist = 0.0;
    result->status = false;
    rclcpp::Rate loop_rate(1);

    for (int i = 0; (i < goal->seconds) && rclcpp::ok(); ++i) {
      // TODO (AT) should we consider a canceled request as a status == false or
      // true?
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // Get distance traveled
      // update currnet_dist
      // DEBUGGING (AT) Just making sure this will compile and print out  data
      publisher_msg.data += 1.0;
      feedback->current_dist = publisher_msg.data;

      // publish the feedback message
      goal_handle->publish_feedback(feedback);
      feedback_publisher_->publish(publisher_msg);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->status = true;
      result->total_dist = publisher_msg.data;
      goal_handle->succeed(result);
      feedback_publisher_->publish(publisher_msg);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  // feedback publisher writes out the current distance traveled every second.
  rclcpp_action::Server<Distance>::SharedPtr action_server_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr feedback_publisher_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
};

auto main(int argc, char *argv[]) -> int {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<ActionQuizServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}