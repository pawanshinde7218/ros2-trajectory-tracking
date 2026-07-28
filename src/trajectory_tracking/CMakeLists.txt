cmake_minimum_required(VERSION 3.8)
project(trajectory_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Custom message generation
# ---------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrajectoryPoint.msg"
  "msg/Trajectory.msg"
  DEPENDENCIES std_msgs
)


ament_python_install_package(script)

install(PROGRAMS
  script/waypoint_collector.py
  script/path_smoother.py
  script/trajectory_generator.py
  script/trajectory_controller.py
  script/rviz_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config script urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(unit_tests test/test_utils.py)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()