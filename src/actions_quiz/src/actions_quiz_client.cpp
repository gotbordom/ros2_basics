// std headers
#include <chrono>

// thrid party headers
#include "rclcpp_action/client.hpp"
#include "rclcpp_action/create_client.hpp"
#include <actions_quiz_msg/action/distance.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;

class ActionQuizClient : public rclcpp::Node {
public:
  using Distance = actions_quiz_msg::action::Distance;
  using GoalHandleDistance = rclcpp_action::ClientGoalHandle<Distance>;

  ActionQuizClient() : Node("action_quiz_client_node"), goal_done_(false) {
    action_client_ = rclcpp_action::create_client<Distance>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "distance_as");
    this->timer_ = this->create_wall_timer(
        500ms, std::bind(&ActionQuizClient::send_goal, this));
  }

  auto is_goal_done() const -> bool { return goal_done_; }
  auto send_goal() -> void {

    // We sent the goal now stop the timer, for now?
    this->timer_->cancel();
    this->goal_done_ = false;

    // first does the client actually exist
    if (!this->action_client_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    // if it exists give it a 10 sec timeout to be ready to use
    if (!this->action_client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      // well... not really but we couldn't accomplish it so we move on
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = Distance::Goal();
    goal_msg.seconds = 20;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    // Remembering to use a SendGoalOptions will remind me off all
    // the callbacks I need to write
    auto send_goal_options = rclcpp_action::Client<Distance>::SendGoalOptions();

    // TODO (AT) might actually be the response that needs to arguments given
    // that Distance object has status and total_dist
    send_goal_options.goal_response_callback = std::bind(
        &ActionQuizClient::goal_response_callback, this, std::placeholders::_1);

    send_goal_options.feedback_callback =
        std::bind(&ActionQuizClient::feedback_callback, this,
                  std::placeholders::_1, std::placeholders::_2);

    // TODO (AT) THis might need a second object placeholder
    send_goal_options.result_callback = std::bind(
        &ActionQuizClient::result_callback, this, std::placeholders::_1);

    // Here is the async send goal that i thought I needed to write.
    // turns out it is already implemented. How nice.
    auto goal_handle_future =
        this->action_client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  // Ok cool so the response callback mainly just tells us if the goal was
  // or was not accepted.
  auto goal_response_callback(const GoalHandleDistance::SharedPtr &goal_handle)
      -> void {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  // This is just a printer, leave the work up to the server on what gets put
  // into feeback and why
  auto
  feedback_callback(GoalHandleDistance::SharedPtr,
                    const std::shared_ptr<const Distance::Feedback> feedback)
      -> void {
    RCLCPP_INFO(this->get_logger(), "Feedback received: %f",
                feedback->current_dist);
  }

  // This looks to have more "logic" but really just likely to be a "what to do"
  // in each of the ResultCode cases
  auto result_callback(const GoalHandleDistance::WrappedResult &result)
      -> void {
    this->goal_done_ = true;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received: %s",
                result.result->status ? "true" : "false");
  }

  rclcpp_action::Client<Distance>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;
};

auto main(int argc, char *argv[]) -> int {
  rclcpp::init(argc, argv);

  // no arguments needed as the options have a default available
  auto action_client = std::make_shared<ActionQuizClient>();
  auto executor = rclcpp::executors::MultiThreadedExecutor();
  executor.add_node(action_client);

  while (!action_client->is_goal_done()) {
    executor.spin_some();
  }

  rclcpp::shutdown();
  return 0;
}