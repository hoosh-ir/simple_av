cmake_minimum_required(VERSION 3.5)
project(dm_pole_info_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(dm_object_info_msgs REQUIRED)

set(msg_files
  "msg/AreaId.msg"
  "msg/AreaName.msg"
  "msg/DeviceErrorInfo.msg"
  "msg/DeviceId.msg"
  "msg/DeviceMfrId.msg"
  "msg/DeviceOpStat.msg"
  "msg/DeviceStat.msg"
  "msg/DeviceTypeId.msg"
  "msg/DeviceUniqueId.msg"
  "msg/DirectionMax.msg"
  "msg/DirectionMin.msg"
  "msg/DistanceMax.msg"
  "msg/DistanceMin.msg"
  "msg/PoleData.msg"
  "msg/PoleHeader.msg"
  "msg/PoleId.msg"
  "msg/PoleInfo.msg"
  "msg/SecLvl.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES dm_object_info_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
