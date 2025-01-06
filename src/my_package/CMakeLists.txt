cmake_minimum_required(VERSION 3.8)
project(my_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Seems that the colcon build doesn't add specifics about files to be added
# yeah ok step 5 goes into this more.
add_executable(simple_node
	src/simple.cpp
)

ament_target_dependencies(simple_node 
	rclcpp
)

# move the executables to porject lib
install(
	TARGETS simple_node
	DESTINATION lib/${PROJECT_NAME}
)

# move the launch files to the workspace shared project
install(
	DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
