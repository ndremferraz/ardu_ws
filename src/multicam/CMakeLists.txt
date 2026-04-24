cmake_minimum_required(VERSION 3.8)
project(multicam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(multicam_node src/multicam_node.cpp)
ament_target_dependencies(multicam_node rclcpp sensor_msgs cv_bridge std_msgs OpenCV)
target_include_directories(multicam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(multicam_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS multicam_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
