cmake_minimum_required(VERSION 3.8)
project(mavros_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)
find_package(mavros_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Add Python node task_control
install(PROGRAMS
  ${PROJECT_NAME}/hover_test.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME hover_test
)

# Add Python node task_control
install(PROGRAMS
  ${PROJECT_NAME}/takeoff_and_wait.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME takeoff_and_wait
)

install(PROGRAMS
  ${PROJECT_NAME}/send_cmd_vel.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME send_cmd_vel
)

install(PROGRAMS
  ${PROJECT_NAME}/flight_test.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME flight_test
)

install(PROGRAMS
  ${PROJECT_NAME}/challenge2or3.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME challenge2or3
)

install(PROGRAMS
  ${PROJECT_NAME}/challenge2.1.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME challenge2.1
)

install(PROGRAMS
  ${PROJECT_NAME}/challenge2.2.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME challenge2.2
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # NOTE(vooon): Does not support our custom triple-license, tiered to make it to work.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)

  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# vim: ts=2 sw=2 et:
