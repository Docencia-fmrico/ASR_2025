cmake_minimum_required(VERSION 3.8)
project(hybrid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bt_bumpgo REQUIRED)
find_package(bt_nav REQUIRED)

set(dependencies
  rclcpp
  ament_index_cpp
  rclcpp_cascade_lifecycle
  behaviortree_cpp_v3
  std_msgs
  bt_bumpgo
  bt_nav
)

include_directories(include)


add_library(${PROJECT_NAME} SHARED
  src/hybrid/BehaviorRunner.cpp
  src/hybrid/FSMNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(hybrid_behavior src/hybrid_behavior_main.cpp)
ament_target_dependencies(hybrid_behavior ${dependencies})
target_link_libraries(hybrid_behavior ${PROJECT_NAME})

install(TARGETS
  ${plugin_libs}
  ${PROJECT_NAME}
  hybrid_behavior
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch behavior_tree_xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()