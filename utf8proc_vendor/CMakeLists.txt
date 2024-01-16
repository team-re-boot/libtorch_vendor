cmake_minimum_required(VERSION 3.8)
project(utf8proc_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_vendor(utf8proc_vendor
  VCS_URL https://github.com/JuliaStrings/utf8proc.git
  VCS_VERSION v2.9.0
)

ament_export_include_directories(opt/utf8proc_vendor/include)
ament_export_libraries(utf8proc)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
