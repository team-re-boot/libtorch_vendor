cmake_minimum_required(VERSION 3.8)
project(libtorch_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(CUDA)

if(CUDA_FOUND)
  message("CUDA version: " ${CUDA_VERSION})
  message("CUDA toolkig root directory : " ${CUDA_TOOLKIT_ROOT_DIR})
  message("TORCH_CUDA_ARCH_LIST : " $ENV{TORCH_CUDA_ARCH_LIST})
  set($ENV{USE_DISTRIBUTED} 1)
  set(CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install;
    -DBUILD_SHARED_LIBS=ON;
    -DUSE_CUDA=ON;
    -DUSE_CUDNN=ON;
    -DUSE_MKLDNN=OFF;
    -DUSE_NCCL=OFF;
    -DBUILD_PYTHON=OFF;
    -DBUILD_CUSTOM_PROTOBUF=OFF;
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON;
    -DUSE_NATIVE_ARCH=OFF;
    -DCMAKE_CUDA_COMPILER=${CUDA_TOOLKIT_ROOT_DIR}/bin/nvcc)
else()
  set(CMAKE_ARGS 
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install;
    -DBUILD_SHARED_LIBS=ON;
    -DUSE_CUDA=OFF;
    -DUSE_CUDNN=OFF;
    -DUSE_MKLDNN=OFF;
    -DUSE_NCCL=OFF;
    -DBUILD_PYTHON=OFF;
    -DBUILD_CUSTOM_PROTOBUF=OFF;
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON;
    -DUSE_NATIVE_ARCH=OFF)
endif()

include(ExternalProject)
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
ament_export_include_directories(include/torch/csrc/api/include)

if(CUDA_FOUND)
  ament_export_libraries(c10_cuda)
  ament_export_libraries(tensorpipe_cuda)
endif()

ament_export_libraries(asmjit)
ament_export_libraries(c10)
ament_export_libraries(caffe2_nvrtc)
ament_export_libraries(clog)
ament_export_libraries(cpuinfo)
ament_export_libraries(dnnl)
ament_export_libraries(fbgemm)
ament_export_libraries(fmt)
ament_export_libraries(kineto)
ament_export_libraries(nnpack)
ament_export_libraries(pthreadpool)
ament_export_libraries(pytorch_qnnpack)
ament_export_libraries(qnnpack)
ament_export_libraries(sleef)
ament_export_libraries(tensorpipe)
ament_export_libraries(tensorpipe_uv)
ament_export_libraries(torch_cpu)
ament_export_libraries(torch_cuda)
ament_export_libraries(torch_global_deps)
ament_export_libraries(torch)
ament_export_libraries(XNNPACK)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
