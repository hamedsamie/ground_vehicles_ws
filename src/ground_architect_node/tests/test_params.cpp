#include "ground_architect_node/ground_node.hpp"
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

using ground_architect::GroundNode;

/**
 * Helper: create a GroundNode that uses its own rclcpp::Context.
 * This avoids relying on the global default context and prevents
 * "context argument is null" errors in tests.
 */
static std::shared_ptr<GroundNode>
make_node_with_local_context(std::shared_ptr<rclcpp::Context> &out_ctx)
{
  // Create and initialize a local context
  out_ctx = std::make_shared<rclcpp::Context>();
  out_ctx->init(0, nullptr);

  // Bind the context into NodeOptions and pass it to the node
  rclcpp::NodeOptions opts;
  opts.context(out_ctx);
  return std::make_shared<GroundNode>(opts);
}

/**
 * Declare the parameters that the node expects (the node normally declares
 * them inside on_configure(); we mimic that here for a unit-test context).
 */
static void declare_expected_params(const std::shared_ptr<GroundNode> &node)
{
  // Use the same names/defaults as production code
  // (the defaults here don't matter for the tests, we just need them declared)
  node->declare_parameter<int>("loop_hz", 10);
  node->declare_parameter<bool>("verbose", true);
}

TEST(ParamTest, RejectsInvalidLoopHzType)
{
  std::shared_ptr<rclcpp::Context> ctx;
  auto node = make_node_with_local_context(ctx);

  // Mimic on_configure() parameter declarations
  declare_expected_params(node);

  auto results = node->set_parameters({rclcpp::Parameter("loop_hz", "not an int")});
  ASSERT_EQ(results.size(), 1u);
  EXPECT_FALSE(results[0].successful);

  node.reset();
  ctx->shutdown("test done");
}

TEST(ParamTest, RejectsOutOfRangeLoopHz)
{
  std::shared_ptr<rclcpp::Context> ctx;
  auto node = make_node_with_local_context(ctx);

  declare_expected_params(node);

  auto results = node->set_parameters({rclcpp::Parameter("loop_hz", 0)}); // too small
  ASSERT_EQ(results.size(), 1u);
  EXPECT_FALSE(results[0].successful);

  results = node->set_parameters({rclcpp::Parameter("loop_hz", 1000)}); // too large
  ASSERT_EQ(results.size(), 1u);
  EXPECT_FALSE(results[0].successful);

  node.reset();
  ctx->shutdown("test done");
}

TEST(ParamTest, AcceptsValidLoopHzAndVerbose)
{
  std::shared_ptr<rclcpp::Context> ctx;
  auto node = make_node_with_local_context(ctx);

  declare_expected_params(node);

  // Set verbose first
  auto results = node->set_parameters({rclcpp::Parameter("verbose", false)});
  ASSERT_EQ(results.size(), 1u);
  EXPECT_TRUE(results[0].successful);

  // Then set a valid loop_hz
  results = node->set_parameters({rclcpp::Parameter("loop_hz", 20)});
  ASSERT_EQ(results.size(), 1u);
  EXPECT_TRUE(results[0].successful);

  // Verify parameters really updated
  int loop_hz = 0;
  bool verbose = true;
  ASSERT_TRUE(node->get_parameter("loop_hz", loop_hz));
  ASSERT_TRUE(node->get_parameter("verbose", verbose));

  EXPECT_EQ(loop_hz, 20);
  EXPECT_FALSE(verbose);

  node.reset();
  ctx->shutdown("test done");
}
