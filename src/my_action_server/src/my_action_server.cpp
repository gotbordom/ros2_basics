// std headers
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server.hpp"
#include <chrono>

// third party headers
#include </t3_action_msg/action/move.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class MyActionService : public rclcpp::Node {
public:
  using namespace std::placeholders;

  this->action_server_ = rclcpp_action::create_server<Move>(
      this, "move_robot_as_2",
      std::bind(&MyActionServer::handle_goal, this, _1, _2),
      std::bind(&MyActionServer::handle_cancel, this, _1),
      std::bind(&MyActionServer::handle_accepted, this, _1));

  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

private : rclcpp_action::Server<Move>::SharedPtr action_server_;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

auto handle_goal(const rclcpp_action::GoalUUID &uuid,
                 std::shared_ptr<const Move::Goal> goal)
    -> rclcpp_action::GoalResponse {
  RCLCPP_INFO(this->get_logger(), "Received goal request with secs %d",
              goal->secs);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

auto handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle)
    -> rclcpp_action::CancelResponse {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

auto handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle)
    -> void {
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a
  // new thread
  std::thread{std::bind(&MyActionServer::execute, this, _1), goal_handle}
      .detach();
}

auto execute(const std::shared_ptr<GoalHandleMove> goal_handle) -> void {
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Move::Feedback>();
  auto &message = feedback->feedback;
  message = "Starting movement...";
  auto result = std::make_shared<Move::Result>();
  auto move = geometry_msgs::msg::Twist();
  rclcpp::Rate loop_rate(1);

  for (int i = 0; (i < goal->secs) && rclcpp::ok(); ++i) {
    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      result->status = message;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
    // Move robot forward and send feedback
    message = "Moving forward...";
    move.linear.x = 0.3;
    publisher_->publish(move);
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Publish feedback");

    loop_rate.sleep();
  }

  // Check if goal is done
  if (rclcpp::ok()) {
    result->status = "Finished action server. Robot moved during 5 seconds";
    move.linear.x = 0.0;
    publisher_->publish(move);
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}; // class MyActionServer

auto main(int argc, char *argv[]) -> int {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<MyActionServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}