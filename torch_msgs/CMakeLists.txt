cmake_minimum_required(VERSION 3.8)
project(torch_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/FP32Tensor.msg
  msg/FP64Tensor.msg
  msg/INT8Tensor.msg
  msg/INT16Tensor.msg
  msg/INT32Tensor.msg
  msg/INT64Tensor.msg
  msg/UINT8Tensor.msg
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
