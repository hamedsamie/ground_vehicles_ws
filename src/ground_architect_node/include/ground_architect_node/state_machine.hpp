#pragma once
#include <string>

namespace ground_architect
{

/**
 * @brief Enumeration of internal operational states.
 *
 * NOT ROS lifecycle states.
 * They represent how our algorithm or system behaves internally.
 * They have been separated from ROS states to keep runtime behavior flexible.
 */
enum class OpState
{
  UNINITIALIZED, // Default state before configuration
  IDLE,          // System loaded but not ready to run
  READY,         // System ready to run
  RUNNING,       // System actively processing
  ERROR          // Fault detected
};

/**
 * @brief Simple FSM for internal operational modes.
 *
 * - Knows the current internal state
 * - Validates allowed transitions
 * - Updates the state when transitions are legal
 */
class StateMachine
{
public:
  /// @brief Return current operational state
  OpState state() const
  {
    return state_;
  }

  /**
   * @brief Check if a transition is legal from `from` → `to`.
   * @param from Current state
   * @param to Target state
   * @return true if transition allowed, false otherwise
   */
  bool can_transition(OpState from, OpState to) const
  {
    // Define allowed transitions
    if (from == OpState::UNINITIALIZED && to == OpState::IDLE)
      return true;
    if (from == OpState::IDLE && (to == OpState::READY || to == OpState::ERROR))
      return true;
    if (from == OpState::READY &&
        (to == OpState::RUNNING || to == OpState::IDLE || to == OpState::ERROR))
      return true;
    if (from == OpState::RUNNING && (to == OpState::READY || to == OpState::ERROR))
      return true;
    if (from == OpState::ERROR && to == OpState::IDLE)
      return true;
    return false;
  }

  /**
   * @brief Perform a state transition if legal.
   * @param target The desired state
   * @param reason Optional reason string (filled on failure)
   * @return true if transition performed, false otherwise
   */
  bool transition(OpState target, std::string *reason = nullptr)
  {
    if (can_transition(state_, target))
    {
      state_ = target;
      return true;
    }
    if (reason)
      *reason = "Illegal transition from current state.";
    return false;
  }

  /// @brief Reset state machine to UNINITIALIZED
  void reset()
  {
    state_ = OpState::UNINITIALIZED;
  }

private:
  OpState state_{OpState::UNINITIALIZED}; // Start in UNINITIALIZED
};

} // namespace ground_architect
