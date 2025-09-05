cmake_minimum_required(VERSION 3.8)
project(webots_gen3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_lint_auto)

install(
  DIRECTORY config launch protos resource worlds controllers
  DESTINATION share/${PROJECT_NAME}
)

# Add C++ gripper mimic node
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(mimic_gripper_finger src/mimic_gripper_finger.cpp)
ament_target_dependencies(mimic_gripper_finger rclcpp std_msgs sensor_msgs)
install(TARGETS mimic_gripper_finger DESTINATION lib/${PROJECT_NAME})

ament_package()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_webots_moveit tests/test_webots_moveit.py)
endif()
