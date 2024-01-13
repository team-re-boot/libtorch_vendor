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

#include <gtest/gtest.h>

#include <torch_util/type_adapter.hpp>

class TestNode : public rclcpp::Node
{
  explicit TestNode(const rclcpp::NodeOptions & options) : Node("test", options) {}
};

TEST(Util, type_adapter) {}

TEST(Util, to_torch_tensor)
{
  torch_util::to_torch_tensor(torch_msgs::build<torch_msgs::msg::FP32Tensor>()
                                .is_cuda(false)
                                .data({1.1, 2.2, 3.3, 4.4})
                                .shape({2, 2}));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  rclcpp::shutdown();
  return RUN_ALL_TESTS();
}
