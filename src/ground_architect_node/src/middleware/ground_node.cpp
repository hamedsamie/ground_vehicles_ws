#include "ground_architect_node/ground_node.hpp"
#include <chrono>

namespace ground_architect
{

GroundNode::GroundNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("ground_node", options)
{
  RCLCPP_INFO(get_logger(), "Node constructed (initial ROS lifecycle state: unconfigured)");
}

GroundNode::CallbackReturn GroundNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_configure(): preparing node");

  // Reset internal FSM and move to IDLE (system loaded, not yet active)
  fsm_.reset();
  fsm_.transition(OpState::IDLE);

  // Create lifecycle-aware publisher (publishes only when activated)
  status_pub_ = this->create_publisher<std_msgs::msg::String>("status", rclcpp::QoS(10));

  // Prepare timer but keep it inactive until activation
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
    if (fsm_.state() == OpState::RUNNING && status_pub_ && status_pub_->is_activated())
    {
      std_msgs::msg::String msg;
      msg.data = "running";
      status_pub_->publish(msg);
    }
  });
  timer_->cancel();

  return CallbackReturn::SUCCESS;
}

GroundNode::CallbackReturn GroundNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_activate(): enabling publishers and timers");

  if (status_pub_)
    status_pub_->on_activate();

  // Move internal FSM to READY, then RUNNING for demo
  fsm_.transition(OpState::READY);
  fsm_.transition(OpState::RUNNING);

  // Start timer for periodic status publishing
  if (timer_)
    timer_->reset();

  return CallbackReturn::SUCCESS;
}

GroundNode::CallbackReturn GroundNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_deactivate(): disabling publishers and timers");

  if (timer_)
    timer_->cancel();
  if (status_pub_)
    status_pub_->on_deactivate();

  // Back to READY (prepared but not running)
  fsm_.transition(OpState::READY);

  return CallbackReturn::SUCCESS;
}

GroundNode::CallbackReturn GroundNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_cleanup(): releasing resources");

  timer_.reset();
  status_pub_.reset();
  fsm_.transition(OpState::IDLE);

  return CallbackReturn::SUCCESS;
}

GroundNode::CallbackReturn GroundNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_shutdown(): final cleanup");

  timer_.reset();
  status_pub_.reset();

  return CallbackReturn::SUCCESS;
}

} // namespace ground_architect
