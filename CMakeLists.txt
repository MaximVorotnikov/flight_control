cmake_minimum_required(VERSION 3.0.2)
project(flight_control)

if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

# if("ALL" IN_LIST Boost_FIND_COMPONENTS)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(custom_ros_utils REQUIRED)
find_package(custom_timers_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(euler_angles_lib REQUIRED)


set(COLCON_VARS ament_cmake rclcpp tf2_ros geometry_msgs mavros_msgs drone_msgs sensor_msgs nav_msgs custom_timers_ros visualization_msgs tf2_geometry_msgs euler_angles_lib)

include_directories(include)

add_executable(node src/autopilot.cpp) # include/Quaternion_Solution.h include/wheeltec_robot.h
target_link_libraries(node euler_angles_lib::euler_angles custom_timers_ros::custom_timers_ros_lib)
ament_target_dependencies(node ${COLCON_VARS})

install(
    DIRECTORY include/${PROJECT_NAME}
    DESTINATION include
)

install(
    TARGETS node
    EXPORT node
    LIBRARY DESTINATION ${COLCON_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${COLCON_GLOBAL_LIB_DESTINATION}
    RUNTIME DESTINATION  ${COLCON_GLOBAL_LIB_DESTINATION}
    INCLUDES DESTINATION include
)

install(TARGETS node
DESTINATION lib/${PROJECT_NAME}/)

ament_export_targets(node HAS_LIBRARY_TARGET)

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

