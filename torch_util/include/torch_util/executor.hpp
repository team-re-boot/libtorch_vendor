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

#ifndef TORCH_UTIL__EXECUTOR_HPP_
#define TORCH_UTIL__EXECUTOR_HPP_

#include <rclcpp/rclcpp.hpp>

namespace torch_util
{
class MultiThreadedExecutor : public rclcpp::executors::MultiThreadedExecutor
{
public:
  explicit MultiThreadedExecutor();
  explicit MultiThreadedExecutor(
    const size_t torch_num_threads, const size_t torch_num_interop_threads,
    const rclcpp::ExecutorOptions & options, const size_t number_of_ros2_callback_threads);
};

class SingleThreadedExecutor : public rclcpp::executors::SingleThreadedExecutor
{
public:
  explicit SingleThreadedExecutor();
  explicit SingleThreadedExecutor(
    const size_t torch_num_threads, const size_t torch_num_interop_threads,
    const rclcpp::ExecutorOptions & options);
};
}  // namespace torch_util

#endif  // TORCH_UTIL__EXECUTOR_HPP_
