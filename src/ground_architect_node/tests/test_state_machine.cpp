#include "ground_architect_node/state_machine.hpp"
#include <gtest/gtest.h>
#include <thread>

using ground_architect::OpState;
using ground_architect::StateMachine;
using ground_architect::to_string;

TEST(StateMachine, StartsUninitialized)
{
  StateMachine sm;
  EXPECT_EQ(sm.state(), OpState::UNINITIALIZED);
}

TEST(StateMachine, LegalTransitionsBasicFlow)
{
  StateMachine sm;

  // UNINITIALIZED -> IDLE
  EXPECT_TRUE(sm.transition(OpState::IDLE, "configured ok"));
  EXPECT_EQ(sm.state(), OpState::IDLE);

  // IDLE -> READY
  EXPECT_TRUE(sm.transition(OpState::READY, "ready to run"));
  EXPECT_EQ(sm.state(), OpState::READY);

  // READY -> RUNNING
  EXPECT_TRUE(sm.transition(OpState::RUNNING, "start processing"));
  EXPECT_EQ(sm.state(), OpState::RUNNING);

  // RUNNING -> READY (pause)
  EXPECT_TRUE(sm.transition(OpState::READY, "pause"));
  EXPECT_EQ(sm.state(), OpState::READY);

  // READY -> IDLE (stop)
  EXPECT_TRUE(sm.transition(OpState::IDLE, "stop"));
  EXPECT_EQ(sm.state(), OpState::IDLE);
}

TEST(StateMachine, IllegalTransitionsAreRejected)
{
  StateMachine sm;

  // Can't jump to RUNNING from UNINITIALIZED
  EXPECT_FALSE(sm.transition(OpState::RUNNING, "illegal"));

  // Proper path to IDLE first
  EXPECT_TRUE(sm.transition(OpState::IDLE));
  // Then RUNNING is still illegal directly from IDLE (must go via READY)
  EXPECT_FALSE(sm.transition(OpState::RUNNING));

  // Go via READY then RUNNING
  EXPECT_TRUE(sm.transition(OpState::READY));
  EXPECT_TRUE(sm.transition(OpState::RUNNING));
}

TEST(StateMachine, ErrorHandling)
{
  StateMachine sm;
  EXPECT_TRUE(sm.transition(OpState::IDLE));
  EXPECT_TRUE(sm.transition(OpState::READY));
  EXPECT_TRUE(sm.transition(OpState::RUNNING));

  // Any state -> ERROR
  EXPECT_TRUE(sm.set_error("sensor fault"));
  EXPECT_EQ(sm.state(), OpState::ERROR);
  EXPECT_EQ(sm.last_reason(), std::string("sensor fault"));

  // ERROR -> only IDLE is legal
  EXPECT_FALSE(sm.transition(OpState::READY));
  EXPECT_FALSE(sm.transition(OpState::RUNNING));
  EXPECT_TRUE(sm.transition(OpState::IDLE, "operator recovered"));
  EXPECT_EQ(sm.state(), OpState::IDLE);
}

TEST(StateMachine, AllowedTransitionsList)
{
  StateMachine sm;

  // From UNINITIALIZED we only allow -> IDLE
  auto a0 = sm.allowed_transitions();
  ASSERT_EQ(a0.size(), 1u);
  EXPECT_EQ(a0[0], OpState::IDLE);

  EXPECT_TRUE(sm.transition(OpState::IDLE));
  auto a1 = sm.allowed_transitions();
  // From IDLE: READY or ERROR
  ASSERT_EQ(a1.size(), 2u);
}

TEST(StateMachine, TimestampUpdatesOnChange)
{
  StateMachine sm;
  auto t0 = sm.time_since_last_change();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_TRUE(sm.transition(OpState::IDLE, "configured"));
  auto t1 = sm.time_since_last_change();

  // We can't assert exact 0 due to scheduling, but it should be less than 10ms.
  EXPECT_LT(t1.count(), 10);
}
