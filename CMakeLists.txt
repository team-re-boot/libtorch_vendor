cmake_minimum_required(VERSION 3.8)
project(libtorch_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

macro(build_libtorch) 
  include(ExternalProject)
  externalproject_add(libtorch-ext
    GIT_REPOSITORY https://github.com/pytorch/pytorch.git
    GIT_TAG v2.1.0
    GIT_SUBMODULES_RECURSE TRUE
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install -DBUILD_SHARED_LIBS:BOOL=ON -DUSE_CUDA=ON -DCUDA_NVCC_FLAGS="-gencode=arch=compute_87,code=sm_87"
    BUILD_COMMAND make -j24
  )
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}
  )
endmacro()

build_libtorch()
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
