cmake_minimum_required(VERSION 3.16)
project(waypoint_navigation_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

add_definitions(-DQT_NO_KEYWORDS)

set(SOURCE_FILES
  src/waypoint_nav_frame.cpp
  src/waypoint_nav_tool.cpp
)

add_library(waypoint_nav_plugin SHARED ${SOURCE_FILES})

target_include_directories(waypoint_nav_plugin PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<INSTALL_INTERFACE:include>"
)

ament_target_dependencies(waypoint_nav_plugin
  rclcpp
  rviz_common
  rviz_rendering
  interactive_markers
  nav_msgs
  geometry_msgs
  visualization_msgs
  pluginlib
  tf2
)

target_link_libraries(waypoint_nav_plugin
  Qt5::Widgets
  yaml-cpp
)

pluginlib_export_plugin_description_file(rviz_common waypoint_nav_plugin_description.xml)

install(TARGETS waypoint_nav_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES waypoint_nav_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY media/ DESTINATION share/${PROJECT_NAME}/media)
install(DIRECTORY icons/ DESTINATION share/${PROJECT_NAME}/icons)

ament_package()
