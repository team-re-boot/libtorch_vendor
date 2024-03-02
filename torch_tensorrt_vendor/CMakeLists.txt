cmake_minimum_required(VERSION 3.8)
project(torch_tensorrt_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(CUDA REQUIRED)

# In jetson, /usr
set(TENSORRT_ROOT $ENV{TENSORRT_ROOT})

if(${TENSORRT_ROOT})
  ament_vendor(${PROJECT_NAME}
    VCS_URL https://github.com/pytorch/TensorRT.git
    VCS_VERSION v2.2.0
    CMAKE_ARGS -DTensorRT_ROOT=${TENSORRT_ROOT}
  )
else()
  message(FATAL_ERROR "TensorRT not found, skip building " ${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()

