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

#include <torch_util/type_adapter.hpp>

namespace torch_util
{
torch::Tensor to_torch_tensor(const cv::Mat & image)
{
  /// cv::Mat (H, W, C) => (C, W, H) => (C, H, W) torch::Tensor
  return torch::from_blob(
           image.data, {image.rows, image.cols, 3},
           torch::TensorOptions().dtype(torch::kUInt8).device([&] {
             return (torch::cuda::is_available()) ? torch::kCUDA : torch::kCPU;
           }()))
    .transpose(0, 2)
    .transpose(1, 2);
}

cv::Mat to_cv_mat(const torch::Tensor & tensor)
{
  assert(tensor.dim() == 3);
  assert(tensor.size(2) == 3);
  assert(tensor.dtype() == torch::kUInt8);
  /// torch::Tensor (C, H, W) => (C, W, H) => (H, W, C) cv::Mat
  const auto transposed_tensor = tensor.transpose(1, 2).transpose(0, 2);
  return cv::Mat(
    cv::Size(transposed_tensor.size(0), transposed_tensor.size(1)), CV_8UC3,
    std::vector<uint8_t>(
      transposed_tensor.data_ptr<uint8_t>(),
      transposed_tensor.data_ptr<uint8_t>() + transposed_tensor.numel())
      .data());
}

torch::Tensor to_torch_tensor(const sensor_msgs::msg::Image & image)
{
  return to_torch_tensor(cv_bridge::toCvCopy(image, "rgb8")->image);
}

sensor_msgs::msg::Image to_image(const std_msgs::msg::Header & header, const torch::Tensor & tensor)
{
  sensor_msgs::msg::Image image_msg;
  cv_bridge::CvImage(header, "bgr8", to_cv_mat(tensor)).toImageMsg(image_msg);
  return image_msg;
}

torch::Tensor to_torch_tensor(const sensor_msgs::msg::CompressedImage & image)
{
  return to_torch_tensor(cv_bridge::toCvCopy(image, "rgb8")->image);
}

sensor_msgs::msg::CompressedImage to_compressed_image(
  const std_msgs::msg::Header & header, const torch::Tensor & tensor)
{
  sensor_msgs::msg::CompressedImage image_msg;
  cv_bridge::CvImage(header, "bgr8", to_cv_mat(tensor)).toCompressedImageMsg(image_msg);
  return image_msg;
}
}  // namespace torch_util