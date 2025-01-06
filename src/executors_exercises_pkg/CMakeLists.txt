cmake_minimum_required(VERSION 3.8)
project(executors_exercises_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

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

# add executable target
# Just a simple hello world style example
add_executable(executor_example_1_node
	src/executor_example_1.cpp
)

# add executable target
# A more complex example that drives  a topic
add_executable(executor_example_2_node
	src/executor_example_2.cpp
)

# add executable target
# A more complex example that drives multiple nodes on one thread.
add_executable(executor_example_3_node
	src/executor_example_3.cpp
)

# add executable target
# A more complex example that drives multiple nodes on multiple threads.
add_executable(executor_example_4_node
	src/executor_example_4.cpp
)

# add executable target
# A more complex example that drives multiple nodes on multple singe threaded executors.
add_executable(executor_example_4_single_threaded_node
	src/executor_example_4_single_threaded.cpp
)

# add executable target
# until now everything has been one callback PER node. Now we look at multiple callbacks PER node.
# This case the second callback is blocked by the first
add_executable(executor_example_5_node
	src/executor_example_5.cpp
)

# add executable target
# until now everything has been one callback PER node. Now we look at multiple callbacks PER node.
# This case we aim to make both callbacks reentrant
# allowing the executor to call them however it needs to.
add_executable(executor_example_5_reentrant_node
	src/executor_example_5_reentrant.cpp
)

# add executable target
# until now everything has been one callback PER node. Now we look at multiple callbacks PER node.
# This case we aim to make both callbacks mutually exclusive
# telling the executer to run each callback independantly
add_executable(executor_example_5_mutually_exclusive_node
	src/executor_example_5_mutually_exclusive.cpp
)

# add executable target
# until now everything has been one callback PER node. Now we look at multiple callbacks PER node.
# This case we aim to make both callbacks mutually exclusive
# While the other uses a single callback group this one uses two for mutual exclusion
add_executable(executor_example_5_mutually_exclusive_multiple_node
	src/executor_example_5_mutually_exclusive_multiple.cpp
)

# add executable target
add_executable(box_bot_manager_node
	src/box_bot_manager.cpp
)

# add ament target dependencies 
ament_target_dependencies(executor_example_1_node
	rclcpp
)

# add ament target dependencies 
ament_target_dependencies(executor_example_2_node
	rclcpp
	nav_msgs
)

# add ament target dependencies 
ament_target_dependencies(executor_example_3_node
	rclcpp
	nav_msgs
)

# add ament target dependencies 
ament_target_dependencies(executor_example_4_node
	rclcpp
	nav_msgs
)

# add ament target dependencies 
ament_target_dependencies(executor_example_4_single_threaded_node
	rclcpp
	nav_msgs
)

# add ament target dependencies 
ament_target_dependencies(executor_example_5_node
	rclcpp
	nav_msgs
)

# add ament target dependencies 
ament_target_dependencies(executor_example_5_reentrant_node
	rclcpp
	nav_msgs
)

# add ament target dependencies 
ament_target_dependencies(executor_example_5_mutually_exclusive_node
	rclcpp
	nav_msgs
)

# add ament target dependencies 
ament_target_dependencies(executor_example_5_mutually_exclusive_multiple_node
	rclcpp
	nav_msgs
)

# add ament target dependencies 
ament_target_dependencies(box_bot_manager_node
	rclcpp
	nav_msgs
)

# install target to lib
install(TARGETS
	executor_example_1_node
	executor_example_2_node
	executor_example_3_node
	executor_example_4_node
	executor_example_4_single_threaded_node
	executor_example_5_node
	executor_example_5_reentrant_node
	executor_example_5_mutually_exclusive_node
	executor_example_5_mutually_exclusive_multiple_node
	box_bot_manager_node
	DESTINATION lib/${PROJECT_NAME}
)

# install sharables to share
install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
