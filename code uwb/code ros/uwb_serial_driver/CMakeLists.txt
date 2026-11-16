cmake_minimum_required(VERSION 3.8)
project(uwb_serial_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(uwb_serial_core
  src/parser.cpp
  src/serial_port.cpp
)
target_include_directories(uwb_serial_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(uwb_serial_core PUBLIC nlohmann_json::nlohmann_json)

add_executable(uwb_serial_node src/uwb_serial_node.cpp)
target_include_directories(uwb_serial_node PRIVATE include)
ament_target_dependencies(uwb_serial_node rclcpp std_msgs)
target_link_libraries(uwb_serial_node uwb_serial_core)

install(TARGETS uwb_serial_core uwb_serial_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_parser test/test_parser.cpp)
  target_include_directories(test_parser PRIVATE include)
  target_link_libraries(test_parser uwb_serial_core)
endif()

ament_package()
