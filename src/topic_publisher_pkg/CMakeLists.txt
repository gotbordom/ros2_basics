cmake_minimum_required(VERSION 3.8)
project(topic_publisher_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

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

# create the exe target for a move robot
add_executable(simple_topic_node
	src/simple_topic_publisher.cpp
)

# create the exe target for a move robot
add_executable(move_robot_node
	src/move_robot.cpp
)

# create exe target for age publisher node
add_executable(age_publisher_node
	src/publish_age.cpp
)

# link libs to target
ament_target_dependencies(simple_topic_node
	rclcpp 
	std_msgs
	geometry_msgs
)

# link libs to target
ament_target_dependencies(move_robot_node
	rclcpp 
	std_msgs
	geometry_msgs
)

ament_target_dependencies(age_publisher_node
	rclcpp
	std_msgs
	custom_interfaces
)

# install exe objects
install(TARGETS 
	simple_topic_node
	move_robot_node
	age_publisher_node
	DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
