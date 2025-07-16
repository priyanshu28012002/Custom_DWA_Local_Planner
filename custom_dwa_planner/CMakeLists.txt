cmake_minimum_required(VERSION 3.8)
project(custom_dwa_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(ROS2_CORE_DEPENDENCIES  rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs )

set(TF_DEPENDENCIES  tf2 tf2_ros tf2_geometry_msgs )

set(NAVIGATION_DEPENDENCIES  nav2_common nav2_util nav2_costmap_2d nav2_navfn_planner )

set(UTILITY_DEPENDENCIES yaml-cpp OpenCV nlohmann_json behaviortree_cpp_v3 )

set(ALL_DEPENDENCIES
  ${ROS2_CORE_DEPENDENCIES}
  ${TF_DEPENDENCIES}
  ${NAVIGATION_DEPENDENCIES}
  ${UTILITY_DEPENDENCIES}
)

foreach(DEPENDENCY ${ALL_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(dwa_planner_node src/dwa_planner_node.cpp)
ament_target_dependencies(dwa_planner_node
  ${ROS2_CORE_DEPENDENCIES}
  ${TF_DEPENDENCIES}
)
install(TARGETS dwa_planner_node DESTINATION lib/${PROJECT_NAME})

add_executable(global_path_planner_node src/global_path.cpp)
ament_target_dependencies(global_path_planner_node
  ${ROS2_CORE_DEPENDENCIES}
  ${TF_DEPENDENCIES}
  ${NAVIGATION_DEPENDENCIES}
)
install(TARGETS global_path_planner_node DESTINATION lib/${PROJECT_NAME})

add_executable(costmap_node src/costmap_node.cpp)
ament_target_dependencies(costmap_node
  ${ROS2_CORE_DEPENDENCIES}
  ${TF_DEPENDENCIES}
)
install(TARGETS costmap_node DESTINATION lib/${PROJECT_NAME})

add_executable(joy_controller_node src/joy_controller.cpp)
ament_target_dependencies(joy_controller_node
  ${ROS2_CORE_DEPENDENCIES}
)
install(TARGETS joy_controller_node DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
