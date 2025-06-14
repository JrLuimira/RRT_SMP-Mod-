cmake_minimum_required(VERSION 3.8)
project(smf_move_base_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(smf_move_base_msgs REQUIRED)
find_package(pedsim_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_filters REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_sdf REQUIRED)
find_package(grid_map_visualization REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ompl REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
  SYSTEM)

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

set(dependencies
    rclcpp
    pedsim_msgs
    nav_msgs
    smf_move_base_msgs
    grid_map_ros
    grid_map_msgs
    tf2_geometry_msgs
    geometry_msgs
    OMPL
)

add_executable(smf_move_base_planner
src/planning_framework_main.cpp
src/state_validity_checker_grid_map_R2.cpp
src/local_state_validity_checker_grid_map_R2.cpp
src/state_cost_objective.cpp
src/informed_new_state_sampler.cpp
src/new_state_sampler.cpp
src/sampler/PathLengthDirectInfSamplerMod.cpp
src/planner/InformedRRTstarMod.cpp
src/planner/RRTstarMod.cpp
src/planner/RRT.cpp
src/planner/RRT_SMP.cpp
src/planner/RRTMod.cpp
src/goal_region_angle_tolerance.cpp

)
ament_target_dependencies(smf_move_base_planner ${dependencies})
target_link_libraries(smf_move_base_planner ${OMPL_LIBRARIES})

install(TARGETS
  smf_move_base_planner
  DESTINATION lib/${PROJECT_NAME})

# add_executable(test_rrt_smp
# src/planner/RRT_SMP.cpp
# )
# ament_target_dependencies(test_rrt_smp ${dependencies})
# target_link_libraries(test_rrt_smp ${OMPL_LIBRARIES})

# install(TARGETS
#   test_rrt_smp
#   DESTINATION lib/${PROJECT_NAME})



ament_package()
