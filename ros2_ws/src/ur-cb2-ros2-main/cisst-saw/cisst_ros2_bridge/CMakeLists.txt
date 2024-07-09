cmake_minimum_required(VERSION 3.10)
project(cisst_ros2_bridge VERSION 2.0.0)

find_package (ament_cmake REQUIRED)
find_package (rclcpp REQUIRED)
find_package (std_msgs REQUIRED)
find_package (std_srvs REQUIRED)
find_package (geometry_msgs REQUIRED)
find_package (sensor_msgs REQUIRED)
find_package (diagnostic_msgs REQUIRED)
find_package (cisst_msgs REQUIRED)
find_package (tf2_ros REQUIRED)
find_package (tf2_msgs REQUIRED)

set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
  )

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.1.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})
include (${CISST_USE_FILE})

include_directories (include)

# header
set (HEADER_FILES
  include/cisst_ros2_bridge/mtsROSBridge.h
  include/cisst_ros2_bridge/mtsCISSTToROS.h
  include/cisst_ros2_bridge/mtsROSToCISST.h)

# source
set (SOURCE_FILES
  src/mtsROSBridge.cpp
  src/mtsCISSTToROS.cpp
  src/mtsROSToCISST.cpp)

add_library (cisst_ros2_bridge ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries (cisst_ros2_bridge ${catkin_LIBRARIES})
cisst_target_link_libraries (cisst_ros2_bridge ${REQUIRED_CISST_LIBRARIES})
ament_target_dependencies (cisst_ros2_bridge rclcpp std_msgs std_srvs geometry_msgs sensor_msgs diagnostic_msgs cisst_msgs tf2_ros)

## Declare a cpp executable
add_executable (example_bridge example/main.cpp)
target_link_libraries (example_bridge cisst_ros2_bridge ${catkin_LIBRARIES})
cisst_target_link_libraries (example_bridge ${REQUIRED_CISST_LIBRARIES})
ament_target_dependencies (example_bridge
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  diagnostic_msgs
  cisst_msgs
  tf2_ros
  )

install (TARGETS cisst_ros2_bridge example_bridge
  DESTINATION lib/${PROJECT_NAME}
  )

install (DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  )

install (
  TARGETS cisst_ros2_bridge
  EXPORT cisst_ros2_bridge
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )
ament_export_targets (cisst_ros2_bridge HAS_LIBRARY_TARGET)

ament_package ()
