/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <action_msgs/msg/goal_status.hpp>    // for GoalStatus
#include <behaviortree_cpp_v3/action_node.h>  // for ActionNodeBase
#include <behaviortree_ros2/action_client_interface.hpp>  // for ActionClientInterface<ActionT>

namespace BT::ros2 {
/// \brief Template for a ROS2 action client BT node refactored from
/// https://github.com/BehaviorTree/BehaviorTree.ROS/blob/bt_action_spin/include/behaviortree_ros/bt_action_node.h
/// and changed such that the class is separate from the actual ROS2
/// functionality
template <class ActionT>
class ActionClientNode : public BT::ActionNodeBase {
  // Declare necessary types
  using ActionFeedback = typename ActionT::Feedback;
  using ActionGoal = typename ActionT::Goal;
  using ActionGoalHandle = typename rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult =
      typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult;

 public:
  ActionClientNode(
      const std::string& name, const BT::NodeConfiguration& config,
      std::shared_ptr<ActionClientInterface<ActionT>> client_api_handle)
      : BT::ActionNodeBase(name, config),
        client_api_handle_(std::move(client_api_handle)),
        wrapped_result_{std::nullopt} {}

  /// \brief This function is executed every time the behavior tree executed
  /// reaches this node \return Resulting node state after the BT node got
  /// ticked
  BT::NodeStatus tick() final {
    // The action taken depends on the current NodeStatus
    switch (status()) {
      // On startup
      case BT::NodeStatus::IDLE: {
        // Reset action result
        wrapped_result_.reset();

        // Create action goal
        std::optional<ActionGoal> goal = defineGoal();
        // If defineGoal returns no goal the node fails
        if (!goal.has_value()) {
          // Set node status as failed and return failed
          return BT::NodeStatus::FAILURE;
        }

        // Send goal
        auto const sending_successful = client_api_handle_->sendGoal(
            goal.value(),
            // Feedback callback
            [this](std::shared_ptr<ActionGoalHandle> /*goal_handle*/,
                   std::shared_ptr<ActionFeedback const> const feedback) {
              // Handle feedback but shadow goal_handle from the calling entity
              handleFeedback(feedback);
            },
            // Result callback
            [this](WrappedResult const& wrapped_result) {
              // Just store the result in a member variable
              wrapped_result_ = wrapped_result;
            });

        // Check if sending of the goal was successful
        if (!sending_successful) {
          // Set node status as failed and return failed
          return BT::NodeStatus::FAILURE;
        }
        // Set node status running
        return BT::NodeStatus::RUNNING;
      }  // case BT::NodeStatus::IDLE
      // While waiting for a result
      case BT::NodeStatus::RUNNING: {  // Depending on the action client state
        if (!client_api_handle_->isActionActive()) {
          // Check if a result is received
          if (!wrapped_result_.has_value()) {
            // If no result is available, something has gone wrong so set node
            // to failure. The most likely reason for this is that the server
            // did not answer in time to a goal or cancel request
            return BT::NodeStatus::FAILURE;
          } else {
            // If a result is received, process it. This also determines the
            // final node status of an action iteration
            return processResult(wrapped_result_.value());
          }
        }
        break;  // insert 'break;' to avoid fall-through
      }         // case BT::NodeStatus::RUNNING:
      case BT::NodeStatus::FAILURE:
        break;
      case BT::NodeStatus::SUCCESS:
        break;
    }  // switch (status())
    // Return current node status (BT::NodeStatus::SUCCESS or
    // BT::NodeStatus::FAILURE)
    return status();
  }

  /// \brief This function is called when the BT tree is stopped
  void halt() override {
    // If the bt is terminated the running action should be canceled
    if (!client_api_handle_->cancelGoal()) {
      // If it is not possible to cancel the goal this node fails
      setStatus(BT::NodeStatus::FAILURE);  // TODO(sjahr) : Does this make sense
                                           // as the tree is terminated anyways
    }
  }

  // These functions need to be defined by the user:
  /// \brief This function is used to define an action goal. If the definition
  /// of a the goal somehow fails it returns nothing and the bT node fails
  /// \return Either a valid action goal or 'std::nullopt' (nothing) in case the
  /// construction of the action goal somehow failed e.g. because some required
  /// information is not available.
  [[nodiscard]] virtual std::optional<ActionGoal> defineGoal() = 0;

  /// \brief Handle feedback received from the action server.
  /// \param [in] feedback Action feedback message
  virtual void handleFeedback(
      std::shared_ptr<ActionFeedback const> const feedback) = 0;

  /// \brief Process the result received from the action server
  /// \param wrapped_result Action result received from the server containing
  /// the result and the final action status \return The resulting BT node
  /// status based on the evaluation of the action result
  [[nodiscard]] virtual BT::NodeStatus processResult(
      WrappedResult const& wrapped_result) = 0;

 private:
  // Handle to use the action client
  std::shared_ptr<ActionClientInterface<ActionT>> client_api_handle_;

  // Action result
  std::optional<WrappedResult> wrapped_result_;
};
}  // namespace BT::ros2
