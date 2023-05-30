// Copyright (c) 2022 OUXT Polaris
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

// Headers in this package
#include <memory>
#include <ndt_matching_2d/ndt_localization_2d_component.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  RCLCPP_INFO(rclcpp::get_logger("ndt_localization_2d_node"), "ndt_localization_2d_node started!");
  auto component = std::make_shared<ndt_matching_2d::NdtLocalization2dComponent>(options);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(component);
  executor.spin();
  // rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
