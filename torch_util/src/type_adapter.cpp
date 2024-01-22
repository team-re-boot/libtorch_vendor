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
torch::Tensor to_torch_tensor(const sensor_msgs::msg::Image & image)
{
  cv_bridge::CvImage bridge;
  const cv::Mat image_cv = cv_bridge::toCvCopy(image, "rgb8")->image;
  /// cv::Mat (H, W, C) => (C, W, H) => (C, H, W) torch::Tensor
  return torch::from_blob(image_cv.data, {image_cv.rows, image_cv.cols, 3})
    .transpose(0, 2)
    .transpose(1, 2);
}
}  // namespace torch_util