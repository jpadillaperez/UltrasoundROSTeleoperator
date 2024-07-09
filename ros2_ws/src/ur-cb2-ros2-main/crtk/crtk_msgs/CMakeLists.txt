cmake_minimum_required (VERSION 3.10)
project (crtk_msgs VERSION 1.0.0)

if (NOT CMAKE_C_STANDARD)
  set (CMAKE_C_STANDARD 99)
endif ()

if (NOT CMAKE_CXX_STANDARD)
  set (CMAKE_CXX_STANDARD 14)
endif ()

find_package (ament_cmake REQUIRED)
find_package (rclcpp REQUIRED)
find_package (std_msgs REQUIRED)
find_package (geometry_msgs REQUIRED)

find_package (builtin_interfaces REQUIRED)
find_package (rosidl_default_generators REQUIRED)

rosidl_generate_interfaces (crtk_msgs
  "msg/OperatingState.msg"
  "msg/StringStamped.msg"
  "msg/CartesianImpedanceHalfPlaneGains.msg"
  "msg/CartesianImpedance.msg"
  "srv/TriggerOperatingState.srv"
  "srv/QueryForwardKinematics.srv"
  "srv/QueryInverseKinematics.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
  )

ament_package ()
