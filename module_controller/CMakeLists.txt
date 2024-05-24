cmake_minimum_required(VERSION 3.8)
project(module_controller)
set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find_package(rclcpp REQUIRED)
# find_package(self_interface REQUIRED)
# add_executable(ChassisCmdPublisherNode src/test_controller.cpp)
# ament_target_dependencies(ChassisCmdPublisherNode rclcpp self_interface)

include_directories(math)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(self_interface REQUIRED)
add_executable(ChassisCmdPublisherNode 
                src/controller_main.cpp
                src/controller_lat_lqr.cpp
                src/controller_lat_ESO_lqr.cpp
                src/math/lqr_solver.cpp
                src/math/ultis_math.cpp
                src/math/trajectory_analysis.cpp
                )
ament_target_dependencies(ChassisCmdPublisherNode Eigen3 rclcpp self_interface)

install(TARGETS
  ChassisCmdPublisherNode
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
