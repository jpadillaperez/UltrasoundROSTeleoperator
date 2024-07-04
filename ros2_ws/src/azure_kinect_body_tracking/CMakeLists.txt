cmake_minimum_required(VERSION 3.8)
project(azure_kinect_body_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include
                    ${rclcpp_INCLUDE_DIRS}
                    /usr/include/k4a
                    /usr/include/k4abt)

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


add_executable(body_tracking_node src/body_tracking_node.cpp)
ament_target_dependencies(
   body_tracking_node
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  cv_bridge
  image_transport
  )
  
add_executable(joint_angle_node src/joint_angle_node.cpp)
ament_target_dependencies(joint_angle_node rclcpp sensor_msgs)

target_link_libraries(body_tracking_node
                      k4a
                      k4abt)

install(TARGETS
  body_tracking_node
  joint_angle_node
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
