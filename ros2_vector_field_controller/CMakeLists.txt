cmake_minimum_required(VERSION 3.8)
project(ros2_vector_field_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


file(GLOB SOURCES
    src/*.h
    src/*.cpp
)

add_executable(vector_field_controller_node ${SOURCES})
ament_target_dependencies(vector_field_controller_node  
  rclcpp
  sensor_msgs  
  tf2
  tf2_ros
  tf2_geometry_msgs
  rosgraph_msgs
  pcl_conversions #not used by merger but needed by tools.cpp file
  PCL #not used by merger but needed by tools.cpp file
  nav_msgs
  visualization_msgs
)

target_link_libraries(vector_field_controller_node
  ${Boost_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
)

#install nodes
install(TARGETS
  vector_field_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
launch
config
DESTINATION share/${PROJECT_NAME}
)

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
