cmake_minimum_required(VERSION 3.8)
project(torch_tensorrt_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(CUDA)
find_package(CUDNN)
find_package(TENSORRT)
ament_auto_find_build_dependencies()

if(${CUDA_FOUND} AND ${CUDNN_FOUND} AND ${TENSORRT_FOUND})
  message("CUDA/cudnn/TensorRT found.")
  ament_vendor(${PROJECT_NAME}
    VCS_URL https://github.com/pytorch/TensorRT.git
    VCS_VERSION v2.2.0
  )
else()
  if(NOT ${CUDA_FOUND})
    message(FATAL_ERROR "CUDA not found.")
  endif()
  # if(NOT ${CUDNN_FOUND})
  #   message(FATAL_ERROR "cudnn not found.")
  # endif()
  if(NOT ${TENSORRT_FOUND})
    message(FATAL_ERROR "TensorRT not found.")
  endif()
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()

