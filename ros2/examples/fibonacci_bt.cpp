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

#include "example_interfaces/action/fibonacci.hpp"  // for Fibonacci
#include "fibonacci_action_client.hpp"

#include <chrono>    // for milliseconds
#include <iostream>  // for cin.get()
#include <thread>    // for sleep_for

#include <rclcpp/logger.hpp>  // for rclcpp::Loggers
#include <rclcpp/rclcpp.hpp>  // for Node, MultiThreadedExecutor, ok()

#include <behaviortree_cpp_v3/bt_factory.h>  // for BehaviorTreeFactory, NodeStatus
#include <behaviortree_ros2/ros2_action_api.hpp>  // for ActionClientApi

namespace {
using namespace std::chrono_literals;
auto const LOGGER = rclcpp::get_logger("BehaviorTree");
auto const SERVER_RESPONSE_TIME = 100ms;
constexpr auto TREE_RATE = 10ms;
constexpr auto BT_DESCRIPTION = R"(
    <?xml version="1.0"?>
    <root>
        <BehaviorTree>
            <Sequence>
                <FibonacciAction/>
            </Sequence>
        </BehaviorTree>
    </root>)";
}  // namespace

int main(int argc, char* argv[]) {
  // Initialize ROS2
  rclcpp::init(argc, argv);
  auto const shared_node =
      std::make_shared<rclcpp::Node>("example_node", rclcpp::NodeOptions{});

  // Create action API handle
  using Fibonacci = example_interfaces::action::Fibonacci;
  auto action_api_handle =
      std::make_shared<BT::ros2::ActionClientApi<Fibonacci>>(
          shared_node, "fibonacci", SERVER_RESPONSE_TIME);

  // Create BT
  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<BT::ros2::FibonacciClient>(
      "FibonacciAction",
      [&](const std::string& name, BT::NodeConfiguration const& config) {
        return std::make_unique<BT::ros2::FibonacciClient>(name, config,
                                                           action_api_handle);
      });
  auto tree = factory.createTreeFromText(BT_DESCRIPTION);

  // Halt the program to enable the user to start introspection tools (e.g.
  // Groot)
  RCLCPP_INFO_STREAM(LOGGER,
                     "Please start the action server for this example. "
                     "Afterward, hit enter to start the action");
  std::cin.get();

  // Create second thread which ticks the tree /TODO(sjahr): Remove this and let
  // the executor handle all threads
  std::thread tick_tree([&]() {
    // Keep ticking until it returns either SUCCESS or FAILURE
    while (rclcpp::ok()) {
      if (tree.tickRoot() != BT::NodeStatus::RUNNING) {
        break;
      }
      std::this_thread::sleep_for(TREE_RATE);
    }
  });

  // Start the node.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(shared_node);
  executor.spin();

  // Stop ticking thread
  tick_tree.join();
  // Shutdown ROS2 ecosystem
  rclcpp::shutdown();
}
