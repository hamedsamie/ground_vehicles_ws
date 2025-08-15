#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ground_architect_node/action/execute_mission.hpp"
#include "ground_architect_node/ground_node.hpp"

using ground_architect::GroundNode;
using ExecuteMission = ground_architect_node::action::ExecuteMission;
using ClientGoalHandleMission = rclcpp_action::ClientGoalHandle<ExecuteMission>;

static std::shared_ptr<GroundNode>
make_node_with_local_context(std::shared_ptr<rclcpp::Context> &out_ctx)
{
  out_ctx = std::make_shared<rclcpp::Context>();
  out_ctx->init(0, nullptr);
  rclcpp::NodeOptions opts;
  opts.context(out_ctx);
  return std::make_shared<GroundNode>(opts);
}

static void do_transition(const std::shared_ptr<GroundNode> &node, uint8_t id)
{
  (void)node->trigger_transition(id);
}

TEST(ServicesAction, ResetSetModeAndExecuteMission)
{
  std::shared_ptr<rclcpp::Context> ctx;
  auto node = make_node_with_local_context(ctx);

  // Client node
  rclcpp::NodeOptions opts;
  opts.context(ctx);
  auto client = std::make_shared<rclcpp::Node>("test_client_sa", opts);

  // Executor with the SAME context
  rclcpp::ExecutorOptions exo;
  exo.context = ctx;
  rclcpp::executors::SingleThreadedExecutor exec(exo);
  exec.add_node(node->get_node_base_interface());
  exec.add_node(client);

  // Service clients
  auto reset_cli = client->create_client<std_srvs::srv::Trigger>("reset");
  auto set_mode_cli = client->create_client<std_srvs::srv::SetBool>("set_mode");

  // Action client
  auto action_cli = rclcpp_action::create_client<ExecuteMission>(client, "execute_mission");

  // Spin executor in background
  std::thread spin_thread([&exec]() { exec.spin(); });

  // Configure & Activate
  do_transition(node, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  do_transition(node, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  // Wait for servers
  ASSERT_TRUE(reset_cli->wait_for_service(std::chrono::seconds(2)));
  ASSERT_TRUE(set_mode_cli->wait_for_service(std::chrono::seconds(2)));
  ASSERT_TRUE(action_cli->wait_for_action_server(std::chrono::seconds(2)));

  // Call set_mode false (READY) then true (RUNNING)
  {
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = false;
    auto fut = set_mode_cli->async_send_request(req);
    auto res = fut.get();
    ASSERT_TRUE(res->success);
  }
  {
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = true;
    auto fut = set_mode_cli->async_send_request(req);
    auto res = fut.get();
    ASSERT_TRUE(res->success);
  }

  // Send a small mission
  ExecuteMission::Goal goal;
  goal.mission_id = "demo";
  goal.cycles = 3;

  std::atomic<float> last_progress{0.0f};
  rclcpp_action::Client<ExecuteMission>::SendGoalOptions send_opts;
  send_opts.feedback_callback =
      [&](ClientGoalHandleMission::SharedPtr /*unused*/,
          const std::shared_ptr<const ExecuteMission::Feedback> feedback) {
        last_progress = feedback->progress;
      };

  auto gh_future = action_cli->async_send_goal(goal, send_opts);
  auto gh = gh_future.get();
  ASSERT_TRUE(gh);

  auto res_future = action_cli->async_get_result(gh);
  auto wrapped = res_future.get();
  ASSERT_EQ(wrapped.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(wrapped.result->success);
  ASSERT_GE(last_progress.load(), 1.0f);

  // Cleanup
  exec.cancel();
  spin_thread.join();
  client.reset();
  node.reset();
  ctx->shutdown("test done");
}
