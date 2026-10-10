cmake_minimum_required(VERSION 3.8)
project(airobot_action)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "task_frame_nav/action/NavigateTaskPoint.action"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(task_frame_nav_node task_frame_nav/src/task_frame_nav_node.cpp)
target_include_directories(task_frame_nav_node PRIVATE task_frame_nav/include)
ament_target_dependencies(task_frame_nav_node
  action_msgs
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  tf2
  tf2_geometry_msgs
  tf2_ros
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(task_frame_nav_node "${cpp_typesupport_target}")

install(TARGETS task_frame_nav_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY task_frame_nav/include/ DESTINATION include)
install(DIRECTORY task_frame_nav/launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY task_frame_nav/config
  DESTINATION share/${PROJECT_NAME}/task_frame_nav)
install(FILES task_frame_nav/README.md
  DESTINATION share/${PROJECT_NAME}/task_frame_nav)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(test_pose_math task_frame_nav/test/test_pose_math.cpp)
  target_include_directories(test_pose_math PRIVATE task_frame_nav/include)
  ament_target_dependencies(test_pose_math geometry_msgs tf2 tf2_geometry_msgs)

  ament_add_gtest(test_capture_state task_frame_nav/test/test_capture_state.cpp)
  target_include_directories(test_capture_state PRIVATE task_frame_nav/include)
  ament_target_dependencies(test_capture_state geometry_msgs nav_msgs tf2 tf2_geometry_msgs)

  ament_add_pytest_test(test_rviz_bringup task_frame_nav/test/test_rviz_bringup.py)
  add_launch_test(task_frame_nav/test/test_auto_goal_launch.py TIMEOUT 15)
endif()

ament_package()
