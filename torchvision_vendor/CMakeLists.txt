cmake_minimum_required(VERSION 3.8)
project(torchvision_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)
find_package(CUDA)

if(CUDA_FOUND)
  message("CUDA version: " ${CUDA_VERSION})
  message("CUDA toolkig root directory : " ${CUDA_TOOLKIT_ROOT_DIR})
  message("TORCH_CUDA_ARCH_LIST : " $ENV{TORCH_CUDA_ARCH_LIST})
  set($ENV{USE_DISTRIBUTED} 1)
  set(CMAKE_ARGS 
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install
    -DBUILD_SHARED_LIBS=ON 
    -DUSE_CUDA=ON
    -DUSE_CUDNN=ON
    -DUSE_MKLDNN=ON
    -DUSE_NCCL=OFF
    -DBUILD_PYTHON=OFF
    -DBUILD_CUSTOM_PROTOBUF=OFF
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    -DCMAKE_CUDA_COMPILER=${CUDA_TOOLKIT_ROOT_DIR}/bin/nvcc)
else()
  set(CMAKE_ARGS 
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install
    -DBUILD_SHARED_LIBS=ON 
    -DUSE_CUDA=OFF
    -DUSE_CUDNN=OFF
    -DUSE_MKLDNN=ON
    -DUSE_NCCL=OFF
    -DBUILD_PYTHON=OFF
    -DBUILD_CUSTOM_PROTOBUF=OFF
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON)
endif()

ament_vendor(torchvision
  VCS_URL https://github.com/pytorch/vision.git
  VCS_VERSION v0.16.2
  CMAKE_ARGS ${CMAKE_ARGS}
)

ament_export_include_directories(include)
ament_export_libraries(torchvision)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
