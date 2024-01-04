cmake_minimum_required(VERSION 3.8)
project(libtorch_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(CUDA REQUIRED)

include(ExternalProject)

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

message("CMake arguments : " ${CMAKE_ARGS})
externalproject_add(libtorch-ext
  GIT_REPOSITORY https://github.com/pytorch/pytorch.git
  GIT_TAG v2.1.0
  GIT_SUBMODULES_RECURSE TRUE
  CMAKE_ARGS ${CMAKE_ARGS}
  BUILD_COMMAND make -j24
)

install(
  DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/
  DESTINATION
    ${CMAKE_INSTALL_PREFIX}
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
