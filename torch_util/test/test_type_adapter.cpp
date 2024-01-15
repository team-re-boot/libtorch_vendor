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

namespace torch_util
{
class TestNode : public rclcpp::Node
{
  explicit TestNode(const rclcpp::NodeOptions & options) : Node("test", options) {}
};

TEST(Util, type_adapter) {}

TEST(Util, to_torch_tensor)
{
  const auto tensor = to_torch_tensor(torch_msgs::build<torch_msgs::msg::FP32Tensor>()
                                        .is_cuda(false)
                                        .data({1.1, 2.2, 3.3, 4.4})
                                        .shape({2, 2}));
  const auto msg = to_fp32_tensor_msg(tensor);
  EXPECT_EQ(msg.data.size(), static_cast<size_t>(4));
  EXPECT_FLOAT_EQ(msg.data[0], 1.1);
  EXPECT_FLOAT_EQ(msg.data[1], 2.2);
  EXPECT_FLOAT_EQ(msg.data[2], 3.3);
  EXPECT_FLOAT_EQ(msg.data[3], 4.4);
  EXPECT_EQ(msg.shape.size(), static_cast<size_t>(2));
  EXPECT_EQ(msg.shape[0], static_cast<int64_t>(2));
  EXPECT_EQ(msg.shape[1], static_cast<int64_t>(2));
}
}  // namespace torch_util

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  rclcpp::shutdown();
  return RUN_ALL_TESTS();
}
