cmake_minimum_required(VERSION 3.5)
project(bt_nav)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(nav2_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    behaviortree_cpp_v3
    ament_index_cpp
    nav2_msgs
    rclcpp_action
)

include_directories(include ${ZMQ_INCLUDE_DIRS})

add_library(move_bt_node SHARED src/bt_nav/Move.cpp)
add_library(getwp_bt_node SHARED src/bt_nav/GetWaypoint.cpp)

list(APPEND plugin_libs
  move_bt_node
  getwp_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(navigate src/bt_nav_main.cpp)
ament_target_dependencies(navigate ${dependencies})


install(TARGETS
  ${plugin_libs}
  navigate
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  behavior_tree_xml
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
