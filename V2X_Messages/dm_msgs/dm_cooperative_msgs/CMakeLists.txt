cmake_minimum_required(VERSION 3.8)
project(dm_cooperative_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dm_object_info_msgs REQUIRED)
find_package(dm_freespace_info_msgs REQUIRED)
find_package(dm_signal_info_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CooperativeObjectInfoMessage.msg"
  "msg/CooperativeFreespaceInfoMessage.msg"
  "msg/CooperativeSignalInfoMessage.msg"
  DEPENDENCIES
    "geometry_msgs"
    "dm_object_info_msgs"
    "dm_freespace_info_msgs"
    "dm_signal_info_msgs"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
