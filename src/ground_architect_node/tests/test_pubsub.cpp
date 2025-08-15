#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ground_architect_node/ground_node.hpp"
#include <atomic>
#include <std_msgs/msg/string.hpp>
#include <thread>

using ground_architect::GroundNode;

// Make node on a local context
static std::shared_ptr<GroundNode>
make_node_with_local_context(std::shared_ptr<rclcpp::Context> &out_ctx)
{
  out_ctx = std::make_shared<rclcpp::Context>();
  out_ctx->init(0, nullptr);
  rclcpp::NodeOptions opts;
  opts.context(out_ctx);
  return std::make_shared<GroundNode>(opts);
}

// Trigger a lifecycle transition by numeric ID (Foxy API)
static void do_transition(const std::shared_ptr<GroundNode> &node, uint8_t transition_id)
{
  (void)node->trigger_transition(transition_id);
}

TEST(PubSub, AckIsPublishedOnlyWhenActive)
{
  // Create a shared local context for all entities in this test
  std::shared_ptr<rclcpp::Context> ctx;
  auto node = make_node_with_local_context(ctx);

  // Test-side node (publisher/subscriber) on the SAME context
  rclcpp::NodeOptions opts;
  opts.context(ctx);
  auto client = std::make_shared<rclcpp::Node>("test_client", opts);

  // Build the executor with the SAME context (otherwise it's null)
  rclcpp::ExecutorOptions ex_opts;
  ex_opts.context = ctx;
  rclcpp::executors::SingleThreadedExecutor exec(ex_opts);

  exec.add_node(node->get_node_base_interface());
  exec.add_node(client);

  std::atomic<bool> got_msg{false};
  std::string last;

  auto sub = client->create_subscription<std_msgs::msg::String>(
      "telemetry", 10, [&](std_msgs::msg::String::UniquePtr m) {
        got_msg = true;
        last = m->data;
      });

  auto pub = client->create_publisher<std_msgs::msg::String>("cmd_in", 10);

  // Spin asynchronously
  std::thread spin_thread([&exec]() { exec.spin(); });

  // Before configure/activate → ignored
  {
    auto m = std_msgs::msg::String();
    m.data = "go";
    pub->publish(m);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    EXPECT_FALSE(got_msg.load());
  }

  // Configure then Activate (numeric IDs per lifecycle_msgs/msg/Transition)
  do_transition(node, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  do_transition(node, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  rclcpp::sleep_for(std::chrono::milliseconds(50));

  // Publish and expect ack
  {
    got_msg = false;
    auto m = std_msgs::msg::String();
    m.data = "go";
    pub->publish(m);

    // wait up to 500 ms
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (!got_msg.load() && std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(got_msg.load());
    EXPECT_EQ(last, "ack:go");
  }

  // Cleanup
  exec.cancel();
  spin_thread.join();
  sub.reset();
  pub.reset();
  client.reset();
  node.reset();
  ctx->shutdown("test done");
}
