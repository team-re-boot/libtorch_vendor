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
  /// Component container with a multi-threaded executor.
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp_components::ComponentManager>();
  if (node->has_parameter("thread_num")) {
    const size_t thread_num = node->get_parameter("thread_num").as_int();
    const size_t torch_num_threads = node->has_parameter("torch_num_threads")
                                       ? node->get_parameter("torch_num_threads").as_int()
                                       : 16;
    const size_t torch_num_interop_threads =
      node->has_parameter("torch_num_interop_threads")
        ? node->get_parameter("torch_num_interop_threads").as_int()
        : 12;
    auto exec = std::make_shared<torch_util::MultiThreadedExecutor>(
      torch_num_threads, torch_num_interop_threads, rclcpp::ExecutorOptions{}, thread_num);
    node->set_executor(exec);
    exec->add_node(node);
    exec->spin();
  } else {
    auto exec = std::make_shared<torch_util::MultiThreadedExecutor>();
    node->set_executor(exec);
  }
}