// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hybrid/FSMNode.hpp"
#include "hybrid/BehaviorRunner.hpp"


#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;

  auto node = std::make_shared<rclcpp::Node>("bt_node");

  auto blackboard = BT::Blackboard::create();
  blackboard->clear();
  blackboard->set("node", node);

  auto fsm_node = std::make_shared<hybrid::FSMNode>(blackboard);
  fsm_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  fsm_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  std::vector<std::string> plugins;

  // BumpGo behavior
  plugins = {
    "forward_bt_node",
    "back_bt_node",
    "turn_bt_node",
    "is_obstacle_bt_node"
  };
  auto bump_go_node = std::make_shared<hybrid::BehaviorRunner>(
    blackboard,
    "bump_go",
    "/behavior_tree_xml/bumpgo.xml",
    plugins
  );

  // Navigation behavior
  plugins = {
    "move_bt_node",
    "getwp_bt_node"
  };
  auto navigation_node = std::make_shared<hybrid::BehaviorRunner>(
    blackboard,
    "navigate",
    "/behavior_tree_xml/navigate.xml",
    plugins
  );

  // Behaviors are configured now. They will be activated in cascade in the FSMNode
  bump_go_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  navigation_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  exec.add_node(fsm_node->get_node_base_interface());
  exec.add_node(bump_go_node->get_node_base_interface());
  exec.add_node(navigation_node->get_node_base_interface());


  RCLCPP_INFO(node->get_logger(), "bump_go_node state: %s",
    bump_go_node->get_current_state().label().c_str());
  RCLCPP_INFO(node->get_logger(), "navigation_node state: %s",
    navigation_node->get_current_state().label().c_str());
  while (rclcpp::ok()) {
    exec.spin_some();
    rclcpp::spin_some(node);

    if (fsm_node->get_state() == hybrid::State::STOP) {
      RCLCPP_INFO(node->get_logger(), "FSM stopped. Exiting...");

      fsm_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);

      break;
    }
  }
  RCLCPP_INFO(node->get_logger(), "bump_go_node state: %s",
    bump_go_node->get_current_state().label().c_str());
  RCLCPP_INFO(node->get_logger(), "navigation_node state: %s",
    navigation_node->get_current_state().label().c_str());

}
