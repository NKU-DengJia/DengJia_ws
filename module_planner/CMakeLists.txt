cmake_minimum_required(VERSION 3.8)
project(module_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
add_executable(NodeReadtxtAndPublish src/visualization/NodeReadtxtAndPublish.cpp)
ament_target_dependencies(NodeReadtxtAndPublish rclcpp visualization_msgs tf2 tf2_geometry_msgs geometry_msgs)

find_package(self_interface REQUIRED)
add_executable(NodeEgoMarkerPublish src/visualization/NodeEgotoMarker.cpp)
ament_target_dependencies(NodeEgoMarkerPublish rclcpp visualization_msgs self_interface)

add_executable(NodeTrajMarkerPublisher src/visualization/NodeTrajtoMarker.cpp)
ament_target_dependencies(NodeTrajMarkerPublisher rclcpp visualization_msgs self_interface)

find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
include_directories(planner/common)
add_executable(NodeTrajPublisher 
                src/planner/test_planner.cpp 
                src/planner/common/common.cpp
                src/planner/common/referenceline.cpp
                src/planner/common/ultis_math.cpp
                src/planner/common/cartesian_frenet_conversion.cpp
                src/planner/common/traj_sampling.cpp
                src/planner/common/traj_OSQP.cpp
                src/planner/common/self_OSQP.cpp
                src/planner/common/traj_self.cpp
                )
ament_target_dependencies(NodeTrajPublisher rclcpp self_interface visualization_msgs osqp OsqpEigen)

find_package(nav_msgs REQUIRED)
include_directories(planner/local_grid_map)
add_executable(NodeGridMap
                src/planner/local_grid_map/grid_map_node.cpp)
ament_target_dependencies(NodeGridMap rclcpp tf2 tf2_geometry_msgs self_interface nav_msgs)

install(TARGETS
  NodeReadtxtAndPublish
  NodeEgoMarkerPublish
  NodeTrajPublisher
  NodeTrajMarkerPublisher
  NodeGridMap
  DESTINATION lib/${PROJECT_NAME})


# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

ament_package()
