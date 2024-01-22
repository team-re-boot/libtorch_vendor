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

#ifndef TORCH_UTIL__TYPE_TYPEADAPTER_HPP_
#define TORCH_UTIL__TYPE_TYPEADAPTER_HPP_

#include <cv_bridge/cv_bridge.h>
#include <torch/torch.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <torch_msgs/msg/fp32_tensor.hpp>
#include <torch_msgs/msg/fp64_tensor.hpp>
#include <torch_msgs/msg/int16_tensor.hpp>
#include <torch_msgs/msg/int32_tensor.hpp>
#include <torch_msgs/msg/int64_tensor.hpp>
#include <torch_msgs/msg/int8_tensor.hpp>
#include <torch_msgs/msg/uint8_tensor.hpp>
#include <tuple>

namespace torch_util
{
template <typename TENSOR>
bool check_shape(const TENSOR & tensor)
{
  if (tensor.shape.empty()) {
    return tensor.data.empty();
  } else {
    size_t vector_size = 1;
    for (const auto dimensiton : tensor.shape) {
      vector_size = vector_size * dimensiton;
    }
    return vector_size == tensor.data.size();
  }
}

#define DEFINE_TO_TORCH_TENSOR_FUNCTION(TENSOR_MSG, DTYPE)                                       \
  torch::Tensor to_torch_tensor(TENSOR_MSG msg)                                                  \
  {                                                                                              \
    assert(check_shape(msg));                                                                    \
    return torch::from_blob(                                                                     \
             msg.data.data(), torch::ArrayRef<int64_t>(msg.shape.data(), msg.shape.size()),      \
             torch::TensorOptions().dtype(DTYPE).device([&] {                                    \
               return (msg.is_cuda && torch::cuda::is_available()) ? torch::kCUDA : torch::kCPU; \
             }()))                                                                               \
      .clone();                                                                                  \
  }

DEFINE_TO_TORCH_TENSOR_FUNCTION(torch_msgs::msg::FP32Tensor, torch::kFloat32)
DEFINE_TO_TORCH_TENSOR_FUNCTION(torch_msgs::msg::FP64Tensor, torch::kFloat64)
DEFINE_TO_TORCH_TENSOR_FUNCTION(torch_msgs::msg::INT8Tensor, torch::kInt8)
DEFINE_TO_TORCH_TENSOR_FUNCTION(torch_msgs::msg::INT16Tensor, torch::kInt16)
DEFINE_TO_TORCH_TENSOR_FUNCTION(torch_msgs::msg::INT32Tensor, torch::kInt32)
DEFINE_TO_TORCH_TENSOR_FUNCTION(torch_msgs::msg::INT64Tensor, torch::kInt64)
DEFINE_TO_TORCH_TENSOR_FUNCTION(torch_msgs::msg::UINT8Tensor, torch::kUInt8)

#undef DEFINE_TO_TORCH_TENSOR_FUNCTION

#define DEFINE_TO_TENSOR_MSG_FUNCTION(MESSAGE_TYPE, DTYPE, CPP_DTYPE)                 \
  MESSAGE_TYPE to_##DTYPE##_tensor_msg(const torch::Tensor & tensor)                  \
  {                                                                                   \
    return torch_msgs::build<MESSAGE_TYPE>()                                          \
      .is_cuda(tensor.is_cuda())                                                      \
      .data(std::vector<CPP_DTYPE>(                                                   \
        tensor.data_ptr<CPP_DTYPE>(), tensor.data_ptr<CPP_DTYPE>() + tensor.numel())) \
      .shape(std::vector<int64_t>(tensor.sizes().begin(), tensor.sizes().end()));     \
  }
DEFINE_TO_TENSOR_MSG_FUNCTION(torch_msgs::msg::FP32Tensor, fp32, float)
DEFINE_TO_TENSOR_MSG_FUNCTION(torch_msgs::msg::FP64Tensor, fp64, double)
DEFINE_TO_TENSOR_MSG_FUNCTION(torch_msgs::msg::INT8Tensor, int8, int8_t)
DEFINE_TO_TENSOR_MSG_FUNCTION(torch_msgs::msg::INT16Tensor, int16, int16_t)
DEFINE_TO_TENSOR_MSG_FUNCTION(torch_msgs::msg::INT32Tensor, int32, int32_t)
DEFINE_TO_TENSOR_MSG_FUNCTION(torch_msgs::msg::INT64Tensor, int64, int64_t)
DEFINE_TO_TENSOR_MSG_FUNCTION(torch_msgs::msg::UINT8Tensor, uint8, uint8_t)

#undef DEFINE_TO_TENSOR_MSG_FUNCTION
}  // namespace torch_util

#define DEFINE_TYPE_ADAPTER(MESSAGE_TYPE, CPP_DTYPE)                                               \
  template <>                                                                                      \
  struct rclcpp::TypeAdapter<torch::Tensor, MESSAGE_TYPE>                                          \
  {                                                                                                \
    using is_specialized = std::true_type;                                                         \
    using custom_type = torch::Tensor;                                                             \
    using ros_message_type = MESSAGE_TYPE;                                                         \
    static void convert_to_ros_message(const custom_type & source, ros_message_type & destination) \
    {                                                                                              \
      destination = torch_util::to_##CPP_DTYPE##_tensor_msg(source);                               \
    }                                                                                              \
    static void convert_to_custom(const ros_message_type & source, custom_type & destination)      \
    {                                                                                              \
      destination = torch_util::to_torch_tensor(source);                                           \
    }                                                                                              \
  };
DEFINE_TYPE_ADAPTER(torch_msgs::msg::FP32Tensor, fp32)
DEFINE_TYPE_ADAPTER(torch_msgs::msg::FP64Tensor, fp64)
DEFINE_TYPE_ADAPTER(torch_msgs::msg::INT8Tensor, int8)
DEFINE_TYPE_ADAPTER(torch_msgs::msg::INT16Tensor, int16)
DEFINE_TYPE_ADAPTER(torch_msgs::msg::INT32Tensor, int32)
DEFINE_TYPE_ADAPTER(torch_msgs::msg::INT64Tensor, int64)
DEFINE_TYPE_ADAPTER(torch_msgs::msg::UINT8Tensor, uint8)

torch::Tensor to_torch_tensor(const sensor_msgs::msg::Image & image);
sensor_msgs::msg::Image to_image(
  const std_msgs::msg::Header & header, const torch::Tensor & tensor);

template <>
struct rclcpp::TypeAdapter<
  std::tuple<std_msgs::msg::Header, torch::Tensor>, sensor_msgs::msg::Image>
{
  using is_specialized = std::true_type;
  using custom_type = std::tuple<std_msgs::msg::Header, torch::Tensor>;
  using ros_message_type = sensor_msgs::msg::Image;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    destination = std::apply(to_image, source);
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    destination = {source.header, to_torch_tensor(source)};
  }
};

#endif  // TORCH_UTIL__TYPE_TYPEADAPTER_HPP_
