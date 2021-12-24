cmake_minimum_required(VERSION 3.5)
project(cpp_harvest_studio)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(harvest_studio_msg REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target} "rclcpp" "std_msgs" "sensor_msgs" "dynamixel_sdk" "geometry_msgs" "moveit_ros_planning_interface" "tf2_geometry_msgs" "harvest_studio_msg" "visualization_msgs")
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

custom_executable(change_joint_states)
custom_executable(camera2dynamixel)
custom_executable(dyna_angle_publisher)
custom_executable(generate_motion_point)
custom_executable(add_rviz_marker)
custom_executable(get_xarm_state)
custom_executable(generate_motion_point_random)
custom_executable(dyna_angle_control)
custom_executable(point_cloud_updater)
custom_executable(harvest_studio_control)


# ament_export_dependencies(rosidl_default_runtime)

ament_package()
