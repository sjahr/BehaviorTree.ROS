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

#include <memory>                           // for std::shared_ptr
#include <rclcpp_action/rclcpp_action.hpp>  // for rclcpp_action::ClientGoalHandle

namespace BT::ros2
{
using ActionGoalStatus = action_msgs::msg::GoalStatus;

template <class ActionT>
/// \brief Interface template for an action client api used in a BT node. This interface class exists to make mocking
/// for testing purposes easier
struct ActionClientInterface
{
  using ActionGoal = typename ActionT::Goal;
  using ActionGoalHandle = typename rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult;
  using ActionFeedback = typename ActionT::Feedback;

  using FeedbackCallback = std::function<void(std::shared_ptr<ActionGoalHandle> goal_handle,
                                              std::shared_ptr<ActionFeedback const> const feedback)>;
  using ResultCallback = std::function<void(WrappedResult const& wrapped_result)>;

  virtual ~ActionClientInterface() = default;

  /// \brief Send the action goal to the action server and provide feedback and result callback functions
  /// \return 'True' if the goal was successfully send to the action server (This does not mean it got accepted!), 'False' if not.
  [[nodiscard]] virtual bool sendGoal(ActionGoal const& goal, FeedbackCallback const& feedback_callback,
                                      ResultCallback const& result_callback) = 0;

  /// \brief Cancel the current active goal
  /// \return 'True' if an action cancel request is successfully send to the action server (This does not mean that the
  /// action is successfully canceled!), 'False' if not.
  [[nodiscard]] virtual bool cancelGoal() = 0;

  /// \brief Check whether or not the action client is currently active (Waiting for a response or result)
  /// \return 'True' if the action is active, 'False' if not
  [[nodiscard]] virtual bool isActionActive() const = 0;
};
}  // namespace BT::ros2
