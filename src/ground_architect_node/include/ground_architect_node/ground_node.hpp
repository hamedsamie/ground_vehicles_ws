#pragma once
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>

#include "ground_architect_node/state_machine.hpp"

namespace ground_architect
{

/**
 * @brief Lifecycle-aware ROS 2 node for the Ground Architect assignment.
 *
 * - Implements standard ROS lifecycle states (configure, activate, etc.)
 * - Manages an internal operational FSM for runtime behavior
 * - Publishes status messages only when activated and in RUNNING mode
 * - Runtime parameters (loop_hz, verbose) with validation and live updates.
 * - YAML-based initial parameter loading (via launch or --params-file).
 * - Timer period follows loop_hz; changes apply immediately when active.
 * - Subscriber: "cmd_in" (std_msgs/String)
 * - Lifecycle publisher: "telemetry" (std_msgs/String)
 * - Subscriber ignores messages unless node is ACTIVE and FSM is RUNNING.
 */
class GroundNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit GroundNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // Lifecycle callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

private:
  // -------------------------
  // Internal state & members
  // -------------------------
  StateMachine fsm_; // internal operational FSM

  // Lifecycle-aware publishers (only publish when activated)
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr telemetry_pub_;

  // Normal subscriber (we guard behavior in the callback)
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;

  // Periodic timer for work / status publishing
  rclcpp::TimerBase::SharedPtr timer_;

  // ----------
  // Parameters
  // ----------
  int loop_hz_{10};    // validated range: [1..100]
  bool verbose_{true}; // extra logs when true

  // Parameter callback handle must be kept alive
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_param_cb_;

  // Helpers
  void declare_and_load_parameters(); // declare and read initial values
  rcl_interfaces::msg::SetParametersResult
  on_param_set(const std::vector<rclcpp::Parameter> &params); // validation + apply
  void apply_parameters_no_throw();                           // use current member values
  void create_or_update_timer();                              // create timer from loop_hz_

  // ----------------
  // Pub/Sub handlers
  // ----------------
  void handle_cmd_msg(const std_msgs::msg::String::SharedPtr msg);
};

} // namespace ground_architect
