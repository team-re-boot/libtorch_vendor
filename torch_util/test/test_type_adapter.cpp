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

#include <functional>
#include <torch_util/type_adapter.hpp>

namespace torch_util
{
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

using AdaptedType = rclcpp::TypeAdapter<torch::Tensor, torch_msgs::msg::FP32Tensor>;

class PubNode : public rclcpp::Node
{
public:
  explicit PubNode(const rclcpp::NodeOptions & options) : Node("test", options)
  {
    publisher_ = create_publisher<AdaptedType>("tensor", 1);
  }
  void publish(const torch::Tensor & tensor) { publisher_->publish(tensor); }

private:
  std::shared_ptr<rclcpp::Publisher<AdaptedType>> publisher_;
};

class SubNode : public rclcpp::Node
{
public:
  explicit SubNode(
    const rclcpp::NodeOptions & options, const std::function<void(const torch::Tensor &)> function)
  : Node("test", options)
  {
    subscriber_ = create_subscription<AdaptedType>("tensor", 1, function);
  }

private:
  std::shared_ptr<rclcpp::Subscription<AdaptedType>> subscriber_;
};

TEST(Util, type_adapter)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  const auto pub_tensor = torch::zeros({1, 2});
  bool tensor_recieved = false;
  auto sub_node = std::make_shared<SubNode>(options, [&](const torch::Tensor & tensor) {
    EXPECT_TRUE(pub_tensor.data_ptr() == tensor.data_ptr());
    tensor_recieved = true;
  });
  auto pub_node = std::make_shared<PubNode>(options);
  pub_node->publish(pub_tensor);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(sub_node);
  exec.add_node(pub_node);
  exec.spin_some();
  rclcpp::shutdown();
  EXPECT_TRUE(tensor_recieved);
}
}  // namespace torch_util

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
