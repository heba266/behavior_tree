cmake_minimum_required(VERSION 3.8)
project(behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# C++17 required for BehaviorTree.CPP
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(std_msgs REQUIRED)
find_package(system_interfaces REQUIRED)

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


# ── BT plugin libraries (one per plugin file) ──────────────
# We will add these as we write each plugin
# For now just the executor

# ── Main BT executor ───────────────────────────────────────
add_executable(bt_executor
    src/bt_executor.cpp
)

target_include_directories(bt_executor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(bt_executor
    rclcpp
    behaviortree_cpp
    behaviortree_ros2
    nav2_msgs
    geometry_msgs
    nav_msgs
    robot_localization
    std_msgs
    system_interfaces
)
install(DIRECTORY
  launch
  config
  worlds
  urdf
  DESTINATION share/${PROJECT_NAME}
)
# Install the executable
install(TARGETS bt_executor
    DESTINATION lib/${PROJECT_NAME}
)

# Install the trees directory so XML is findable at runtime
install(DIRECTORY trees/
    DESTINATION share/${PROJECT_NAME}/trees
)

ament_package()
