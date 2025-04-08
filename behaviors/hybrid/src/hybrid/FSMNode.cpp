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


#include "hybrid/FSMNode.hpp"

namespace hybrid
{

FSMNode::FSMNode(BT::Blackboard::Ptr blackboard)
: CascadeLifecycleNode("fsm_node"),
  state_(State::BUMP_GO),
  last_status_(""),
  blackboard_(blackboard)
{

  RCLCPP_INFO(get_logger(), "FSMNode constructor");

  status_sub_ = create_subscription<std_msgs::msg::String>(
    "behavior_status", 10, std::bind(&FSMNode::status_callback, this, _1));

  go_to_state(state_);
}

void
FSMNode::status_callback(std_msgs::msg::String::UniquePtr msg)
{
  last_status_ = msg.get()->data;
  RCLCPP_DEBUG(get_logger(), "Status received: %s", last_status_.c_str());
  if (last_status_ == "DEACTIVATED") {
    last_status_ = "";
  }
}

void
FSMNode::control_cycle()
{
  std_msgs::msg::Float64 result_msg;

  switch (state_) {
    case State::BUMP_GO:
      if(last_status_ == "") {
        RCLCPP_INFO_ONCE(get_logger(), "[State - BUMP_GO]: BT not started yet");
        break;
      }
      if (check_behavior_finished()) {
        if (last_status_ == "SUCCESS") {
          RCLCPP_INFO(get_logger(), "[State - BUMP_GO]: Task done");
          go_to_state(State::NAVIGATE);
        } else {
          RCLCPP_INFO(get_logger(), "[State - BUMP_GO]: Error. Stopping FSM");
          go_to_state(State::STOP);
        }
      }
      break;
    case State::NAVIGATE:
      if (check_behavior_finished()) {
        if (last_status_ == "FAILURE") {
          RCLCPP_INFO(get_logger(), "[State - NAVIGATE]: Error. Stopping FSM");
          go_to_state(State::STOP);
        } else {
          RCLCPP_INFO(get_logger(), "[State - NAVIGATE]: Task done");
          go_to_state(State::BUMP_GO);
        }
      }
      break;
    case State::STOP:
      RCLCPP_INFO_ONCE(get_logger(), "[State - STOP]: FSM stopped");
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Unknown state");
      break;
  }
}

void
FSMNode::go_to_state(State state)
{
  state_ = state;
  last_status_ = "";

  switch (state_) {
    case State::BUMP_GO:
      start_time_ = now();
      RCLCPP_INFO(get_logger(), "State: BUMP_GO");
      remove_activation("navigate");
      add_activation("bump_go");
      RCLCPP_INFO(get_logger(), "bump_go activated");
      break;
    case State::NAVIGATE:
      RCLCPP_INFO(get_logger(), "State: NAVIGATE");
      remove_activation("bump_go");
      add_activation("navigate");
      RCLCPP_INFO(get_logger(), "navigate activated");
      break;
    case State::STOP:
      RCLCPP_INFO(get_logger(), "State: STOP");
      remove_activation("bump_go");
      remove_activation("navigate");
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Unknown state");
      break;
  }
}

bool
FSMNode::check_behavior_finished()
{
  if (state_ == State::BUMP_GO) {
    auto elapsed = now() - start_time_;
    RCLCPP_INFO(get_logger(), "Elapsed time: %.2f seconds", elapsed.seconds());
    if (elapsed > std::chrono::seconds(BUMP_GO_TIMEOUT)) {
      RCLCPP_INFO(get_logger(), "State %d. BUMP_GO timeout", static_cast<int>(state_));
      last_status_ = "SUCCESS";
      return true;
    }
  }

  RCLCPP_DEBUG(get_logger(), "State %d. Checking behavior finished: %s", static_cast<int>(state_),
      last_status_.c_str());
  return last_status_ == "FAILURE" || last_status_ == "SUCCESS";
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FSMNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "FSMNode on_activate");

  timer_ =
    create_wall_timer(10ms, std::bind(&FSMNode::control_cycle, this));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FSMNode::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "FSMNode on_deactivate");

  timer_ = nullptr;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

} // namespace hybrid
