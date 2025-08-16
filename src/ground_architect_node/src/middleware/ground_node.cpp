#include "ground_architect_node/ground_node.hpp"

#include <chrono>
#include <lifecycle_msgs/msg/state.hpp> // PRIMARY_STATE_ACTIVE
#include <sstream>
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

  // Initialize algorithm with current config; set mode to READY (inactive)
  controller_.init(algo::Config{gain_, saturation_});
  controller_.set_mode(algo::Mode::READY);

  // Publishers
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
    RCLCPP_INFO(get_logger(), "Configured with loop_hz=%d verbose=%s gain=%.3f sat=%.3f", loop_hz_,
                verbose_ ? "true" : "false", gain_, saturation_);
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

  (void)fsm_.transition(OpState::READY);
  (void)fsm_.transition(OpState::RUNNING);
  sync_algo_mode();

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

  (void)fsm_.transition(OpState::READY);
  sync_algo_mode();

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

  (void)fsm_.transition(OpState::IDLE);
  sync_algo_mode();

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
  this->declare_parameter<int>("loop_hz", 10);
  this->declare_parameter<bool>("verbose", true);
  this->declare_parameter<double>("gain", 1.0);
  this->declare_parameter<double>("saturation", 10.0);

  this->get_parameter("loop_hz", loop_hz_);
  this->get_parameter("verbose", verbose_);
  this->get_parameter("gain", gain_);
  this->get_parameter("saturation", saturation_);

  if (loop_hz_ < 1 || loop_hz_ > 100)
  {
    RCLCPP_WARN(get_logger(), "Invalid 'loop_hz'=%d (expected 1..100). Clamping to 10.", loop_hz_);
    loop_hz_ = 10;
  }
  if (saturation_ <= 0.0)
  {
    RCLCPP_WARN(get_logger(), "Invalid 'saturation'=%.3f (must be >0). Using 10.0.", saturation_);
    saturation_ = 10.0;
  }
}

rcl_interfaces::msg::SetParametersResult
GroundNode::on_param_set(const std::vector<rclcpp::Parameter> &params)
{
  int new_loop_hz = loop_hz_;
  bool new_verbose = verbose_;
  double new_gain = gain_;
  double new_sat = saturation_;

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
    else if (p.get_name() == "gain")
    {
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        rcl_interfaces::msg::SetParametersResult r;
        r.successful = false;
        r.reason = "'gain' must be a double";
        return r;
      }
      new_gain = p.as_double();
    }
    else if (p.get_name() == "saturation")
    {
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        rcl_interfaces::msg::SetParametersResult r;
        r.successful = false;
        r.reason = "'saturation' must be a double";
        return r;
      }
      double v = p.as_double();
      if (v <= 0.0)
      {
        rcl_interfaces::msg::SetParametersResult r;
        r.successful = false;
        r.reason = "'saturation' must be > 0";
        return r;
      }
      new_sat = v;
    }
  }

  loop_hz_ = new_loop_hz;
  verbose_ = new_verbose;
  gain_ = new_gain;
  saturation_ = new_sat;

  // Push new config to algorithm and update timer rate
  controller_.update_config(algo::Config{gain_, saturation_});
  apply_parameters_no_throw();

  if (verbose_)
  {
    RCLCPP_INFO(get_logger(), "Params updated: loop_hz=%d verbose=%s gain=%.3f sat=%.3f", loop_hz_,
                verbose_ ? "true" : "false", gain_, saturation_);
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
      // Publish status and algorithm y
      std_msgs::msg::String msg;
      std::ostringstream oss;
      oss << "running y=" << controller_.y();
      msg.data = oss.str();
      status_pub_->publish(msg);
      if (verbose_)
        RCLCPP_INFO(this->get_logger(), "Status: %s", msg.data.c_str());
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

  controller_.reset();
  controller_.set_mode(algo::Mode::IDLE);

  res->success = true;
  res->message = "FSM reset to IDLE; algorithm reset; timer stopped.";
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
  sync_algo_mode();

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
  res->message = std::string("FSM moved to ") + to_string(fsm_.state()) +
                 " ; algo=" + algo::to_string(controller_.mode());
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

    // Make the algorithm "do work" each cycle.
    (void)controller_.step(algo::Input{1.0});

    feedback->progress = static_cast<float>(i) / static_cast<float>(cycles);
    feedback->phase = "working";
    goal_handle->publish_feedback(feedback);
    if (verbose_)
    {
      RCLCPP_INFO(get_logger(), "Mission feedback: progress=%.2f y=%.3f", feedback->progress,
                  controller_.y());
    }

    std::this_thread::sleep_for(100ms);
  }

  result->success = true;
  result->message = "Mission completed";
  goal_handle->succeed(result);
  if (verbose_)
    RCLCPP_INFO(get_logger(), "Mission succeeded, y=%.3f", controller_.y());
}

// -------
// Pub/Sub
// -------
bool GroundNode::try_parse_u(const std::string &s, double &out_u) const
{
  // Accept formats like: "u:1.25" or "u=-0.5"
  const auto pos_colon = s.find(':');
  const auto pos_eq = s.find('=');
  size_t pos = std::string::npos;
  if (pos_colon != std::string::npos)
    pos = pos_colon;
  else if (pos_eq != std::string::npos)
    pos = pos_eq;
  if (pos == std::string::npos)
    return false;

  const auto key = s.substr(0, pos);
  if (key != "u")
    return false;

  try
  {
    out_u = std::stod(s.substr(pos + 1));
    return true;
  }
  catch (...)
  {
    return false;
  }
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

  double u = 0.0;
  bool parsed = msg && try_parse_u(msg->data, u);

  if (parsed)
  {
    auto out = controller_.step(algo::Input{u});
    if (telemetry_pub_ && telemetry_pub_->is_activated())
    {
      std_msgs::msg::String t;
      std::ostringstream oss;
      oss << "y=" << out.y << " note=" << out.note;
      t.data = oss.str();
      telemetry_pub_->publish(t);
    }
    if (verbose_)
    {
      RCLCPP_INFO(get_logger(), "cmd u=%.3f -> y=%.3f (%s)", u, controller_.y(), out.note.c_str());
    }
  }
  else
  {
    // Fallback
    if (telemetry_pub_ && telemetry_pub_->is_activated())
    {
      auto out = std_msgs::msg::String();
      out.data = std::string("ack:") + (msg ? msg->data : "");
      telemetry_pub_->publish(out);
    }
    if (verbose_)
    {
      RCLCPP_INFO(get_logger(), "cmd not parsed as 'u:<value>', ack only");
    }
  }
}

// -------
// Helpers
// -------
void GroundNode::sync_algo_mode()
{
  // Keep the algorithm mode aligned with our internal FSM.
  // Map missing/unknown states conservatively to IDLE or READY.
  algo::Mode m = algo::Mode::IDLE;

  switch (fsm_.state())
  {
  case OpState::UNINITIALIZED: // be conservative before first configure
    m = algo::Mode::IDLE;
    break;
  case OpState::IDLE:
    m = algo::Mode::IDLE;
    break;
  case OpState::READY:
    m = algo::Mode::READY;
    break;
  case OpState::RUNNING:
    m = algo::Mode::RUNNING;
    break;
  case OpState::ERROR:
    m = algo::Mode::READY;
    break;
  default:
    // Fallback for any future/unknown states
    m = algo::Mode::IDLE;
    break;
  }

  controller_.set_mode(m);
}

} // namespace ground_architect
