// Copyright 2024 Team Re-Boot. All rights reserved.
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

#include <memory>

#include "rclcpp_components/component_manager.hpp"
#include "torch_util/executor.hpp"

int main(int argc, char * argv[])
{
  /// Component container with a single-threaded executor.
  rclcpp::init(argc, argv);
  auto exec = std::make_shared<torch_util::SingleThreadedExecutor>();
  auto node = std::make_shared<rclcpp_components::ComponentManager>(exec);
  exec->add_node(node);
  exec->spin();
}