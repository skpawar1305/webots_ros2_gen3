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

ament_package()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_webots_moveit tests/test_webots_moveit.py)
endif()
