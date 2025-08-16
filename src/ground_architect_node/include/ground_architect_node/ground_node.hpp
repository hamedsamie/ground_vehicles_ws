#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ground_architect_node/action/execute_mission.hpp" // generated from action/
#include "ground_architect_node/algorithm/controller.hpp"
#include "ground_architect_node/state_machine.hpp"

namespace ground_architect
{

class GroundNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using ExecuteMission = ground_architect_node::action::ExecuteMission;
  using GoalHandleMission = rclcpp_action::ServerGoalHandle<ExecuteMission>;

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
  // ------------
  // Internal FSM
  // ------------
  StateMachine fsm_;

  // -------------------------
  // Pure algorithm module
  // -------------------------
  algo::Controller controller_;

  // -------------------------
  // Publishers / Subscribers
  // -------------------------
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr telemetry_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ----------
  // Parameters
  // ----------
  int loop_hz_{10};         // [1..100]
  bool verbose_{true};      // extra logs
  double gain_{1.0};        // algorithm parameter
  double saturation_{10.0}; // algorithm parameter

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_param_cb_;

  void declare_and_load_parameters();
  rcl_interfaces::msg::SetParametersResult
  on_param_set(const std::vector<rclcpp::Parameter> &params);
  void apply_parameters_no_throw();
  void create_or_update_timer();

  // --------
  // Services
  // --------
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_mode_srv_;

  void handle_reset(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void handle_set_mode(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                       std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  // -------------
  // Action Server
  // -------------
  rclcpp_action::Server<ExecuteMission>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                          std::shared_ptr<const ExecuteMission::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMission> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleMission> goal_handle);
  void execute_mission(const std::shared_ptr<GoalHandleMission> goal_handle);

  // ---------------
  // Pub/Sub handler
  // ---------------
  void handle_cmd_msg(const std_msgs::msg::String::SharedPtr msg);
  bool try_parse_u(const std::string &s, double &out_u) const;

  // Keep algorithm mode aligned with FSM
  void sync_algo_mode();
};

} // namespace ground_architect
