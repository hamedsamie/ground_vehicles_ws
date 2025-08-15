#include "ground_architect_node/ground_node.hpp"

#include <chrono>
#include <lifecycle_msgs/msg/state.hpp> // PRIMARY_STATE_ACTIVE
#include <thread>
#include <utility>

using namespace std::chrono_literals;

namespace ground_architect
{

GroundNode::GroundNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("ground_node", options)
{
  RCLCPP_INFO(get_logger(), "Constructed (ROS lifecycle: unconfigured)");

  // Register parameter set callback EARLY so that any set_parameters()
  // calls go through validation.
  on_set_param_cb_ = this->add_on_set_parameters_callback(
      std::bind(&GroundNode::on_param_set, this, std::placeholders::_1));
}

GroundNode::CallbackReturn GroundNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_configure(): declare/load parameters & create resources");

  // Reset the internal FSM and move to IDLE (configured but inactive)
  fsm_.reset();
  (void)fsm_.transition(OpState::IDLE);

  // Parameters
  declare_and_load_parameters();

  // Lifecycle-aware publishers
  status_pub_ = this->create_publisher<std_msgs::msg::String>("status", rclcpp::QoS(10));
  telemetry_pub_ = this->create_publisher<std_msgs::msg::String>("telemetry", rclcpp::QoS(10));

  // Subscriber: guard behavior in callback against lifecycle/FSM
  cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
      "cmd_in", rclcpp::QoS(10),
      std::bind(&GroundNode::handle_cmd_msg, this, std::placeholders::_1));

  // Services
  reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "reset",
      std::bind(&GroundNode::handle_reset, this, std::placeholders::_1, std::placeholders::_2));

  set_mode_srv_ = this->create_service<std_srvs::srv::SetBool>(
      "set_mode",
      std::bind(&GroundNode::handle_set_mode, this, std::placeholders::_1, std::placeholders::_2));

  // Action server
  action_server_ = rclcpp_action::create_server<ExecuteMission>(
      this->get_node_base_interface(), this->get_node_clock_interface(),
      this->get_node_logging_interface(), this->get_node_waitables_interface(), "execute_mission",
      std::bind(&GroundNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GroundNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&GroundNode::handle_accepted, this, std::placeholders::_1));

  // Timer prepared but paused until activation
  create_or_update_timer();
  if (timer_)
    timer_->cancel();

  if (verbose_)
  {
    RCLCPP_INFO(get_logger(), "Configured with loop_hz=%d verbose=%s", loop_hz_,
                verbose_ ? "true" : "false");
  }

  return CallbackReturn::SUCCESS;
}

GroundNode::CallbackReturn GroundNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_activate(): enabling publishers and timers");

  if (status_pub_)
    status_pub_->on_activate();
  if (telemetry_pub_)
    telemetry_pub_->on_activate();

  // Lifecycle activation → internal FSM READY → RUNNING
  (void)fsm_.transition(OpState::READY);
  (void)fsm_.transition(OpState::RUNNING);

  if (timer_)
    timer_->reset(); // start periodic work
  return CallbackReturn::SUCCESS;
}

GroundNode::CallbackReturn GroundNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_deactivate(): pausing activity");

  if (timer_)
    timer_->cancel();
  if (status_pub_)
    status_pub_->on_deactivate();
  if (telemetry_pub_)
    telemetry_pub_->on_deactivate();

  // Back to READY (configured and prepared, but not actively running)
  (void)fsm_.transition(OpState::READY);
  return CallbackReturn::SUCCESS;
}

GroundNode::CallbackReturn GroundNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_cleanup(): releasing resources");

  timer_.reset();
  status_pub_.reset();
  telemetry_pub_.reset();
  cmd_sub_.reset();
  reset_srv_.reset();
  set_mode_srv_.reset();
  action_server_.reset();

  // Back to IDLE
  (void)fsm_.transition(OpState::IDLE);
  return CallbackReturn::SUCCESS;
}

GroundNode::CallbackReturn GroundNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_shutdown(): final cleanup");
  timer_.reset();
  status_pub_.reset();
  telemetry_pub_.reset();
  cmd_sub_.reset();
  reset_srv_.reset();
  set_mode_srv_.reset();
  action_server_.reset();
  return CallbackReturn::SUCCESS;
}

// ----------
// Parameters
// ----------
void GroundNode::declare_and_load_parameters()
{
  // Declare with defaults if not already declared.
  this->declare_parameter<int>("loop_hz", 10);
  this->declare_parameter<bool>("verbose", true);

  // Read current values
  this->get_parameter("loop_hz", loop_hz_);
  this->get_parameter("verbose", verbose_);

  // Validate immediately
  if (loop_hz_ < 1 || loop_hz_ > 100)
  {
    RCLCPP_WARN(get_logger(),
                "Invalid 'loop_hz'=%d in parameters (expected 1..100). Clamping to 10.", loop_hz_);
    loop_hz_ = 10;
  }
}

rcl_interfaces::msg::SetParametersResult
GroundNode::on_param_set(const std::vector<rclcpp::Parameter> &params)
{
  int new_loop_hz = loop_hz_;
  bool new_verbose = verbose_;

  // Parameter validation
  for (const auto &p : params)
  {
    if (p.get_name() == "loop_hz")
    {
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        rcl_interfaces::msg::SetParametersResult r;
        r.successful = false;
        r.reason = "'loop_hz' must be an integer";
        return r;
      }
      int v = p.as_int();
      if (v < 1 || v > 100)
      {
        rcl_interfaces::msg::SetParametersResult r;
        r.successful = false;
        r.reason = "'loop_hz' out of range (1..100)";
        return r;
      }
      new_loop_hz = v;
    }
    else if (p.get_name() == "verbose")
    {
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
      {
        rcl_interfaces::msg::SetParametersResult r;
        r.successful = false;
        r.reason = "'verbose' must be a boolean";
        return r;
      }
      new_verbose = p.as_bool();
    }
  }

  // All validations passed → apply updates atomically
  loop_hz_ = new_loop_hz;
  verbose_ = new_verbose;
  apply_parameters_no_throw();

  if (verbose_)
  {
    RCLCPP_INFO(get_logger(), "Updated parameters: loop_hz=%d verbose=%s", loop_hz_,
                verbose_ ? "true" : "false");
  }

  rcl_interfaces::msg::SetParametersResult ok;
  ok.successful = true;
  ok.reason = "accepted";
  return ok;
}

void GroundNode::apply_parameters_no_throw()
{
  // Recreate timer with the new period. Safe to call in any lifecycle state.
  try
  {
    create_or_update_timer();
    const bool active =
        (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    if (active && fsm_.state() == OpState::RUNNING && timer_)
    {
      timer_->reset();
    }
    else if (timer_)
    {
      timer_->cancel();
    }
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(get_logger(), "apply_parameters_no_throw: %s", e.what());
  }
}

void GroundNode::create_or_update_timer()
{
  // Convert Hz to period (milliseconds).
  const auto period_ms = std::chrono::milliseconds(1000 / loop_hz_);
  auto cb = [this]() {
    if (fsm_.state() == OpState::RUNNING && status_pub_ && status_pub_->is_activated())
    {
      std_msgs::msg::String msg;
      msg.data = "running";
      status_pub_->publish(msg);
      if (verbose_)
      {
        RCLCPP_INFO(this->get_logger(), "Published status: %s", msg.data.c_str());
      }
    }
  };

  // If timer not created yet, make it. Otherwise, replace it.
  if (!timer_)
  {
    timer_ = this->create_wall_timer(period_ms, cb);
  }
  else
  {
    timer_ = this->create_wall_timer(period_ms, cb);
  }
  // Lifecycle state decides that whether to start teh timer or not.
  if (timer_)
    timer_->cancel();
}

// --------
// Services
// --------
void GroundNode::handle_reset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  // Stop any periodic work
  if (timer_)
    timer_->cancel();

  // Reset FSM to IDLE
  fsm_.reset();
  (void)fsm_.transition(OpState::IDLE);

  res->success = true;
  res->message = "FSM reset to IDLE; timer stopped.";
}

void GroundNode::handle_set_mode(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                 std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  // true  -> request RUNNING
  // false -> request READY
  const bool want_running = req->data;

  OpState target = want_running ? OpState::RUNNING : OpState::READY;
  if (!fsm_.can_transition(fsm_.state(), target))
  {
    res->success = false;
    res->message = std::string("Illegal transition from ") + to_string(fsm_.state()) + " to " +
                   to_string(target);
    return;
  }

  (void)fsm_.transition(target, want_running ? "set_mode=true" : "set_mode=false");

  // Respect lifecycle active state for timer control
  const bool active =
      (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  if (active && fsm_.state() == OpState::RUNNING && timer_)
  {
    timer_->reset();
  }
  else if (timer_)
  {
    timer_->cancel();
  }

  res->success = true;
  res->message = std::string("FSM moved to ") + to_string(fsm_.state());
}

// -------------
// Action Server
// -------------
rclcpp_action::GoalResponse
GroundNode::handle_goal(const rclcpp_action::GoalUUID &,
                        std::shared_ptr<const ExecuteMission::Goal> goal)
{

  const bool active =
      (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  if (!(active && fsm_.state() == OpState::RUNNING))
  {
    if (verbose_)
      RCLCPP_WARN(get_logger(), "Rejecting goal (node not active or FSM not RUNNING)");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!goal || goal->cycles <= 0)
  {
    if (verbose_)
      RCLCPP_WARN(get_logger(), "Rejecting goal: invalid cycles (%d)", goal ? goal->cycles : -1);
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (verbose_)
  {
    RCLCPP_INFO(get_logger(), "Accepting goal mission_id='%s' cycles=%d", goal->mission_id.c_str(),
                goal->cycles);
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GroundNode::handle_cancel(const std::shared_ptr<GoalHandleMission>)
{
  if (verbose_)
    RCLCPP_INFO(get_logger(), "Cancel request received");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GroundNode::handle_accepted(const std::shared_ptr<GoalHandleMission> goal_handle)
{
  // Execute in a background thread
  std::thread{std::bind(&GroundNode::execute_mission, this, goal_handle)}.detach();
}

void GroundNode::execute_mission(const std::shared_ptr<GoalHandleMission> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<ExecuteMission::Feedback>();
  auto result = std::make_shared<ExecuteMission::Result>();

  const int cycles = goal->cycles;
  for (int i = 1; i <= cycles; ++i)
  {
    // Check cancel
    if (goal_handle->is_canceling())
    {
      result->success = false;
      result->message = "Canceled";
      goal_handle->canceled(result);
      if (verbose_)
        RCLCPP_INFO(get_logger(), "Mission canceled");
      return;
    }

    // Simulate work: publish feedback
    feedback->progress = static_cast<float>(i) / static_cast<float>(cycles);
    feedback->phase = "working";
    goal_handle->publish_feedback(feedback);
    if (verbose_)
    {
      RCLCPP_INFO(get_logger(), "Mission feedback: progress=%.2f", feedback->progress);
    }

    std::this_thread::sleep_for(100ms);
  }

  result->success = true;
  result->message = "Mission completed";
  goal_handle->succeed(result);
  if (verbose_)
    RCLCPP_INFO(get_logger(), "Mission succeeded");
}

// -------
// Pub/Sub
// -------
void GroundNode::handle_cmd_msg(const std_msgs::msg::String::SharedPtr msg)
{
  // Ignore unless lifecycle is ACTIVE and FSM is RUNNING
  const bool active =
      (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  if (!(active && fsm_.state() == OpState::RUNNING))
  {
    if (verbose_)
    {
      RCLCPP_INFO(get_logger(), "Ignoring cmd '%s' (node not active or FSM not RUNNING)",
                  msg ? msg->data.c_str() : "<null>");
    }
    return;
  }

  if (verbose_)
  {
    RCLCPP_INFO(get_logger(), "Handling cmd: %s", msg->data.c_str());
  }

  // For now, simple echo → telemetry ack
  if (telemetry_pub_ && telemetry_pub_->is_activated())
  {
    auto out = std_msgs::msg::String();
    out.data = std::string("ack:") + msg->data;
    telemetry_pub_->publish(out);
  }
}

} // namespace ground_architect
