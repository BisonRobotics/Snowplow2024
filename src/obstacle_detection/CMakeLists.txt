cmake_minimum_required(VERSION 3.8)
project(obstacle_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_interface_packages)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Armadillo REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CircleObstacle.msg"
  "msg/SegmentObstacle.msg"
  "msg/Obstacles.msg"
)

#add_message_files(FILES SegmentObstacle.msg )
#generate_messages(DEPENDENCIES std_msgs geometry_msgs)

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


add_executable(obstacle_detector src/obstacle_detector.cpp include/obstacle_detector.h
               include/point.h include/segment.h include/circle.h include/figure_fitting.h)
ament_target_dependencies(obstacle_detector rclcpp sensor_msgs tf2 tf2_geometry_msgs )

add_executable(obstacle_visualizer src/obstacle_visualizer.cpp include/obstacle_visualizer.h)
ament_target_dependencies(obstacle_visualizer rclcpp visualization_msgs)


#add_executable(scans_merger src/scans_merger.cpp include/scans_merger.h)
#add_dependencies(scans_merger ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(static_scan_publisher src/static_scan_publisher.cpp)
ament_target_dependencies(static_scan_publisher rclcpp sensor_msgs)


ament_package()
