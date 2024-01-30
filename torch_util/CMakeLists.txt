cmake_minimum_required(VERSION 3.8)
project(torch_util)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# find_package(CUDA)

include_directories(include)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/type_adapter.cpp
  src/executor.cpp
)

ament_auto_add_executable(component_container_mt
  src/component_container_mt.cpp
)

ament_auto_add_executable(component_container
  src/component_container.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_auto_add_gtest(test_type_adaptation test/test_type_adapter.cpp)
  target_link_libraries(test_type_adaptation ${PROJECT_NAME})
endif()

ament_auto_package()
