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

#include <behaviortree_ros2/bt_action_client.hpp>  // for ActionClientNode
#include "example_interfaces/action/fibonacci.hpp"       // for Fibonacci
#include <optional>                                      // for std::optional
#include <rclcpp/logger.hpp>                             // for rclcpp::Logger

namespace BT::ros2
{
using Fibonacci = example_interfaces::action::Fibonacci;
using FibonacciWrappedResult = rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult;

namespace
{
auto const LOGGER = rclcpp::get_logger("FibonacciClientNode");
}
class FibonacciClient : public ActionClientNode<Fibonacci>
{
public:
  using ActionClientNode<Fibonacci>::ActionClientNode;
  virtual ~FibonacciClient() = default;
  [[nodiscard]] std::optional<Fibonacci::Goal> defineGoal() override
  {
    // Create goal
    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;
    return goal_msg;
  };
  void handleFeedback(std::shared_ptr<Fibonacci::Feedback const> const /*feedback*/) override
  {
    RCLCPP_INFO_STREAM(LOGGER, "Waiting for result");
  };
  [[nodiscard]] BT::NodeStatus processResult(FibonacciWrappedResult const& wrapped_result) override
  {
    switch (wrapped_result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(LOGGER, "Goal was successful");
        return BT::NodeStatus::SUCCESS;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(LOGGER, "Goal was canceled");
        return BT::NodeStatus::FAILURE;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_INFO(LOGGER, "Goal was aborted");
        return BT::NodeStatus::FAILURE;
    }
    RCLCPP_ERROR_STREAM(LOGGER, "Action goal result code is UNKNOWN");
    return BT::NodeStatus::FAILURE;
  };

  void halt() override{
    // Do nothing
  };
};
}  // namespace BT::ros2
