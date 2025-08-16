#include "ground_architect_node/algorithm/controller.hpp"
#include <gtest/gtest.h>

using namespace ground_architect::algo;

TEST(Algorithm, ThrowsIfNotInitialized)
{
  Controller c;
  EXPECT_THROW(c.step(Input{1.0}), std::runtime_error);
}

TEST(Algorithm, InactiveDoesNotChangeY)
{
  Controller c;
  c.init(Config{1.0, 5.0});
  c.set_mode(Mode::READY);
  auto out = c.step(Input{2.0});
  EXPECT_EQ(out.y, 0.0);
  EXPECT_EQ(out.note, "inactive");
}

TEST(Algorithm, RunningUpdatesAndClamps)
{
  Controller c;
  c.init(Config{2.0, 5.0}); // gain=2, sat=5
  c.set_mode(Mode::RUNNING);

  auto o1 = c.step(Input{1.0}); // y += 2*1 = 2
  EXPECT_DOUBLE_EQ(o1.y, 2.0);

  auto o2 = c.step(Input{1.5}); // y += 2*1.5 = 3 -> total 5
  EXPECT_DOUBLE_EQ(o2.y, 5.0);

  auto o3 = c.step(Input{1.0}); // y += 2*1 = 2 -> clamp to 5
  EXPECT_DOUBLE_EQ(o3.y, 5.0);
}

TEST(Algorithm, ResetClearsY)
{
  Controller c;
  c.init(Config{1.0, 10.0});
  c.set_mode(Mode::RUNNING);
  (void)c.step(Input{3.0});
  EXPECT_DOUBLE_EQ(c.y(), 3.0);
  c.reset();
  EXPECT_DOUBLE_EQ(c.y(), 0.0);
}
