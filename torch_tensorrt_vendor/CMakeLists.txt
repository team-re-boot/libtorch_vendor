cmake_minimum_required(VERSION 3.8)
project(torch_tensorrt_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(CUDA REQUIRED)
find_package(CUDNN REQUIRED)
find_package(TENSORRT REQUIRED)

if(NOT (${CUDA_FOUND} AND ${CUDNN_FOUND} AND ${TENSORRT_FOUND}))
  message(WARNING "cuda, cudnn, tensorrt libraries are not found")
  return()
else()
  message("CUDA/cudnn/TensorRT found.")
  ament_vendor(${PROJECT_NAME}
    VCS_URL https://github.com/pytorch/TensorRT.git
    VCS_VERSION v2.2.0
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()

