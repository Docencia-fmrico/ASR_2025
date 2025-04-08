// Copyright 2024 Rodrigo Pérez-Rodríguez
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


#include "hybrid/BehaviorRunner.hpp"

namespace hybrid
{

BehaviorRunner::BehaviorRunner(
  BT::Blackboard::Ptr blackboard,
  const std::string & name,
  const std::string & xml_path,
  const std::vector<std::string> & plugins)
: CascadeLifecycleNode(name),
  blackboard_(blackboard),
  status_(BT::NodeStatus::IDLE)
{
  RCLCPP_INFO(get_logger(), "BehaviorRunner constructor (%s)", name.c_str());

  blackboard_->get("node", node_);

  xml_path_ = xml_path;
  plugins_ = plugins;

  RCLCPP_DEBUG(get_logger(), "XML path: %s", xml_path_.c_str());
  RCLCPP_DEBUG(get_logger(), "# plugins: %ld", plugins_.size());

  status_pub_ = create_publisher<std_msgs::msg::String>("behavior_status", 10);

  executed_ = false;
}

void
BehaviorRunner::control_cycle()
{
  std_msgs::msg::String msg;
  RCLCPP_INFO_ONCE(get_logger(), "Behavior tree (%s): control_cycle", get_name());

  if (!executed_) {
    status_ = tree_.rootNode()->executeTick();
  }


  switch (status_) {
    case BT::NodeStatus::SUCCESS:
      msg.data = "SUCCESS";
      status_pub_->publish(msg);
      RCLCPP_INFO_ONCE(get_logger(), "Behavior tree (%s): SUCCESS", get_name());
      executed_ = true;
      break;
    case BT::NodeStatus::RUNNING:
      msg.data = "RUNNING";
      status_pub_->publish(msg);
      RCLCPP_INFO_ONCE(get_logger(), "Behavior tree (%s): RUNNING", get_name());
      break;
    default:
      msg.data = "FAILURE";
      status_pub_->publish(msg);
      RCLCPP_INFO_ONCE(get_logger(), "Behavior tree (%s): FAILURE", get_name());
      executed_ = true;
      break;
  }

}

BT::NodeStatus
BehaviorRunner::get_bt_status()
{
  return status_;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BehaviorRunner::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "BehaviorRunner (%s) on_activate", get_name());

  std::string pkg_path = ament_index_cpp::get_package_share_directory("hybrid");
  std::string xml_file = pkg_path + xml_path_;

  RCLCPP_INFO(get_logger(), "XML file: %s", xml_file.c_str());

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  for (const auto & plugin : plugins_) {
    factory.registerFromPlugin(loader.getOSName(plugin));
    RCLCPP_DEBUG(get_logger(), "Plugin loaded: %s", plugin.c_str());
  }

  RCLCPP_DEBUG(get_logger(), "Getting node from blackboard");
  blackboard_->get("node", node_);
  RCLCPP_DEBUG(get_logger(), "Creating BT from XML");
  tree_ = factory.createTreeFromFile(xml_file, blackboard_);
  RCLCPP_DEBUG(get_logger(), "BT created from XML");

  status_pub_->on_activate();

  timer_ =
    create_wall_timer(10ms, std::bind(&BehaviorRunner::control_cycle, this));

  RCLCPP_DEBUG(get_logger(), "Timer created");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BehaviorRunner::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "BehaviorRunner(%s) on_deactivate", get_name());
  std_msgs::msg::String msg;
  msg.data = "DEACTIVATED";
  status_pub_->publish(msg);

  timer_ = nullptr;
  status_pub_->on_deactivate();

  refresh();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void
BehaviorRunner::refresh()
{
  executed_ = false;
  status_ = BT::NodeStatus::IDLE;

  tree_.rootNode()->halt();

  RCLCPP_INFO(get_logger(), "BehaviorRunner refreshed (%s)", get_name());
}

} // namespace hybrid
