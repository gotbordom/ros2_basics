// std headers
#include <chrono>

// third party headers
#include "rclcpp/client.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <t3_action_msg/action/move.hpp>

using namespace std::chrono_literals;

class MyActionClient : public rclcpp::Node {
public:
  using Move = t3_action_msg::action::Move;
  using GoalHandleMove = rclcpp_action::ClientGoalHandle<Move>;

  explicit MyActionClient(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("my_action_client", node_options), goal_done_(false) {
    this->client_ptr_ = rclcpp_action::create_client<Move>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "move_robot_as");
    // As soon as I was writing out why I didn't agree, I realized it is because
    // we need to spin_some and attempt contacting the server.
    this->timer_ = this->create_wall_timer(
        500ms, std::bind(&MyActionClient::send_goal, this));
  }

  auto is_goal_done() const -> bool { return goal_done_; }
  auto send_goal() -> void {
    using namespace std::placeholders;

    // We sent the goal now stop the timer, for now?
    this->timer_->cancel();
    this->goal_done_ = false;

    // first does the client actually exist
    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    // if it exists give it a 10 sec timeout to be ready to use
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      // well... not really but we couldn't accomplish it so we move on
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = Move::Goal();
    goal_msg.secs = 5;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    // Remembering to use a SendGoalOptions will remind me off all
    // the callbacks I need to write
    auto send_goal_options = rclcpp_action::Client<Move>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&MyActionClient::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
        std::bind(&MyActionClient::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
        std::bind(&MyActionClient::result_callback, this, _1);

    // Here is the async send goal that i thought I needed to write.
    // turns out it is already implemented. How nice.
    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  // Ok cool so the response callback mainly just tells us if the goal was
  // or was not accepted.
  auto goal_response_callback(const GoalHandleMove::SharedPtr &goal_handle)
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
  auto feedback_callback(GoalHandleMove::SharedPtr,
                         const std::shared_ptr<const Move::Feedback> feedback)
      -> void {
    RCLCPP_INFO(this->get_logger(), "Feedback received: %s",
                feedback->feedback.c_str());
  }

  // This looks to have more "logic" but really just likely to be a "what to do"
  // in each of the ResultCode cases
  auto result_callback(const GoalHandleMove::WrappedResult &result) -> void {
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
                result.result->status.c_str());
  }

  // Cool so it looks like they have this object already created in
  // rclcpp_action. so instead of creating a client and sub from scratch just
  // use theirs
  rclcpp_action::Client<Move>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;
};

// Hey look we are using executors again? Why did we stop this? I think mainly
// because we were making clients and servers? Idk
auto main(int argc, char *argv[]) -> int {
  rclcpp::init(argc, argv);

  // no arguments needed as the options have a default available
  auto action_client = std::make_shared<MyActionClient>();
  auto executor = rclcpp::executors::MultiThreadedExecutor();
  executor.add_node(action_client);

  while (!action_client->is_goal_done()) {
    executor.spin_some();
  }

  rclcpp::shutdown();
  return 0;
}
