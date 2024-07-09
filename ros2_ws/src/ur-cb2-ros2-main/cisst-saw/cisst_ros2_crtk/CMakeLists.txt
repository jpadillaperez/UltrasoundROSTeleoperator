cmake_minimum_required(VERSION 3.10)
project(cisst_ros2_crtk VERSION 2.0.0)

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
find_package (crtk_msgs REQUIRED)
find_package (cisst_ros2_bridge REQUIRED)

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
  include/cisst_ros2_crtk/cisst_ros_crtk.h
  include/cisst_ros2_crtk/mts_ros_crtk_bridge.h # for backward compatibility
  include/cisst_ros2_crtk/mts_ros_crtk_bridge_provided.h
  include/cisst_ros2_crtk/mts_ros_crtk_bridge_required.h
  include/cisst_ros2_crtk/mtsCISSTToROS.h
  include/cisst_ros2_crtk/mtsROSToCISST.h)

# source
set (SOURCE_FILES
  src/cisst_ros_crtk.cpp
  src/mts_ros_crtk_bridge_provided.cpp
  src/mts_ros_crtk_bridge_required.cpp
  src/mtsCISSTToROS.cpp
  src/mtsROSToCISST.cpp)

add_library (cisst_ros2_crtk ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries (cisst_ros2_crtk ${catkin_LIBRARIES})
cisst_target_link_libraries (cisst_ros2_crtk ${REQUIRED_CISST_LIBRARIES})
ament_target_dependencies (cisst_ros2_crtk cisst_ros2_bridge crtk_msgs)


install (TARGETS cisst_ros2_crtk
  DESTINATION lib/${PROJECT_NAME}
  )

install (DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  )
install (
  TARGETS cisst_ros2_crtk
  EXPORT cisst_ros2_crtk
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )
ament_export_targets (cisst_ros2_crtk HAS_LIBRARY_TARGET)

ament_package ()
