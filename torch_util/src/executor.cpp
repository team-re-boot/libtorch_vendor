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

#include <torch/torch.h>

#include <torch_util/executor.hpp>

namespace torch_util
{
MultiThreadedExecutor::MultiThreadedExecutor() : rclcpp::executors::MultiThreadedExecutor()
{
  torch::set_num_threads(16);
  torch::set_num_interop_threads(12);
}

MultiThreadedExecutor::MultiThreadedExecutor(
  const size_t torch_num_threads, const size_t torch_num_interop_threads,
  const rclcpp::ExecutorOptions & options, const size_t number_of_ros2_callback_threads)
: rclcpp::executors::MultiThreadedExecutor(options, number_of_ros2_callback_threads)
{
  torch::set_num_threads(torch_num_threads);
  torch::set_num_interop_threads(torch_num_interop_threads);
}

SingleThreadedExecutor::SingleThreadedExecutor()
{
  torch::set_num_threads(16);
  torch::set_num_interop_threads(12);
}

SingleThreadedExecutor::SingleThreadedExecutor(
  const size_t torch_num_threads, const size_t torch_num_interop_threads,
  const rclcpp::ExecutorOptions & options)
: rclcpp::executors::SingleThreadedExecutor(options)
{
  torch::set_num_threads(torch_num_threads);
  torch::set_num_interop_threads(torch_num_interop_threads);
}
}  // namespace torch_util
