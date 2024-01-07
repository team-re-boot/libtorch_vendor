cmake_minimum_required(VERSION 3.8)
project(bert_tokenizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(
  bert_tokenizer SHARED
  src/tokenizer.cpp)

file(DOWNLOAD https://cdn.huggingface.co/bert-base-uncased-vocab.txt vocab/bert-base-uncased-vocab.txt)
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/vocab
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tokenizer test/test_tokenizer.cpp)
  target_link_libraries(test_tokenizer bert_tokenizer)
endif()

ament_auto_package()
