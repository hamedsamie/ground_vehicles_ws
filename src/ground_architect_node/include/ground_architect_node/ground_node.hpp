#pragma once
#include "ground_architect_node/state_machine.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>

namespace ground_architect
{

/**
 * @brief ROS 2 LifecycleNode for the ground vehicle architecture.
 *
 * - Implements standard ROS lifecycle states (configure, activate, etc.)
 * - Manages an internal operational FSM for runtime behavior
 * - Publishes status messages only when activated and in RUNNING mode
 */
class GroundNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit GroundNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

protected:
  // Alias for cleaner callback signatures
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // Lifecycle state callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

private:
  StateMachine fsm_; ///< Internal operational FSM instance

  // Publisher for status messages (lifecycle-aware)
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_pub_;

  // Periodic timer (publishes status when running)
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace ground_architect
