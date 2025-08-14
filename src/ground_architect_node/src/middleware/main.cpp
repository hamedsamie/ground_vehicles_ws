#include "ground_architect_node/ground_node.hpp"
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Entry point for the node.
 *
 * Creates a ROS 2 executor, adds the LifecycleNode, and spins.
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ground_architect::GroundNode>();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
