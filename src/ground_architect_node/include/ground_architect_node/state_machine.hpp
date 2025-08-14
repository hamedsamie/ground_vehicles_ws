#pragma once
#include <chrono>
#include <string>
#include <vector>

namespace ground_architect
{

// Internal runtime modes (not ROS lifecycle states)
enum class OpState
{
  UNINITIALIZED, // default before configuration
  IDLE,          // resources loaded, not ready to run
  READY,         // ready to run
  RUNNING,       // actively processing
  ERROR          // faulted
};

// Helper in order that logs and tests can print states
inline const char *to_string(OpState s)
{
  switch (s)
  {
  case OpState::UNINITIALIZED:
    return "UNINITIALIZED";
  case OpState::IDLE:
    return "IDLE";
  case OpState::READY:
    return "READY";
  case OpState::RUNNING:
    return "RUNNING";
  case OpState::ERROR:
    return "ERROR";
  default:
    return "UNKNOWN";
  }
}

/**
 * @brief ROS-free FSM.
 */
class StateMachine
{
public:
  StateMachine() = default;

  OpState state() const
  {
    return state_;
  }

  // What transitions are allowed from the given state?
  bool can_transition(OpState from, OpState to) const
  {
    if (from == OpState::UNINITIALIZED && to == OpState::IDLE)
      return true;

    if (from == OpState::IDLE)
    {
      return (to == OpState::READY || to == OpState::ERROR);
    }

    if (from == OpState::READY)
    {
      return (to == OpState::RUNNING || to == OpState::IDLE || to == OpState::ERROR);
    }

    if (from == OpState::RUNNING)
    {
      return (to == OpState::READY || to == OpState::ERROR);
    }

    if (from == OpState::ERROR)
    {
      return (to == OpState::IDLE); // Remark: require operator to recover to IDLE
    }

    return false;
  }

  // List of allowed next states from the current state
  std::vector<OpState> allowed_transitions() const
  {
    std::vector<OpState> out;
    for (OpState candidate :
         {OpState::UNINITIALIZED, OpState::IDLE, OpState::READY, OpState::RUNNING, OpState::ERROR})
    {
      if (candidate != state_ && can_transition(state_, candidate))
        out.push_back(candidate);
    }
    return out;
  }

  // Transition with an optional reason string
  bool transition(OpState target, const std::string &reason = "")
  {
    if (!can_transition(state_, target))
    {
      // don't update reason/timestamp on illegal request
      return false;
    }
    state_ = target;
    last_reason_ = reason;
    last_change_time_ = std::chrono::steady_clock::now();
    return true;
  }

  // Force an error state from anywhere
  bool set_error(const std::string &reason)
  {
    // If already in ERROR, do nothing
    if (state_ == OpState::ERROR)
      return false;
    state_ = OpState::ERROR;
    last_reason_ = reason;
    last_change_time_ = std::chrono::steady_clock::now();
    return true;
  }

  // Reset back to the initial state
  void reset()
  {
    state_ = OpState::UNINITIALIZED;
    last_reason_.clear();
    last_change_time_ = std::chrono::steady_clock::now();
  }

  // Explains the last change state
  const std::string &last_reason() const
  {
    return last_reason_;
  }

  // Time difference since the last change
  std::chrono::milliseconds time_since_last_change() const
  {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - last_change_time_);
  }

private:
  OpState state_{OpState::UNINITIALIZED};
  std::string last_reason_{};
  std::chrono::steady_clock::time_point last_change_time_{std::chrono::steady_clock::now()};
};

} // namespace ground_architect
