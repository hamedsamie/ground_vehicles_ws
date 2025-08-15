#include "ground_architect_node/ground_node.hpp"

#include <chrono>
#include <lifecycle_msgs/msg/state.hpp> // for PRIMARY_STATE_ACTIVE
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
  return CallbackReturn::SUCCESS;
}

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
    // Only start the timer if we’re already active and running.
    if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      if (fsm_.state() == OpState::RUNNING && timer_)
      {
        timer_->reset();
      }
      else if (timer_)
      {
        timer_->cancel();
      }
    }
    else if (timer_)
    {
      // Not active → keep timer paused
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
    // Replace with a new timer;
    timer_ = this->create_wall_timer(period_ms, cb);
  }
  // Lifecycle state decides that whether to start teh timer or not.
  if (timer_)
    timer_->cancel();
}

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
