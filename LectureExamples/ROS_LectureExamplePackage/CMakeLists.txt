cmake_minimum_required(VERSION 3.8)
project(mtrx3760)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav2_msgs REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomMessage.msg"
  "action/MoveRobot.action"
  "srv/MoveForward.srv"
  DEPENDENCIES
)

ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(simplePublisher src/simple_publisher.cpp)
ament_target_dependencies(simplePublisher rclcpp geometry_msgs)

add_executable(simpleSubscriber src/simple_subscriber.cpp)
ament_target_dependencies(simpleSubscriber rclcpp geometry_msgs)

add_executable(simplePublisherCustom src/simple_publisher_custom.cpp)
ament_target_dependencies(simplePublisherCustom rclcpp)
target_link_libraries(simplePublisherCustom ${cpp_typesupport_target})

add_executable(simpleSubscriberCustom src/simple_subscriber_custom.cpp)
ament_target_dependencies(simpleSubscriberCustom rclcpp)
target_link_libraries(simpleSubscriberCustom ${cpp_typesupport_target})

add_executable(staticBroadcast src/static_broadcast.cpp)
ament_target_dependencies(staticBroadcast rclcpp tf2 tf2_ros)

add_executable(dynamicBroadcast src/dynamic_broadcast.cpp)
ament_target_dependencies(dynamicBroadcast rclcpp tf2 tf2_ros)

add_executable(frameListener src/frame_listener.cpp)
ament_target_dependencies(frameListener rclcpp tf2 tf2_ros)

add_executable(timeTraveller src/time_traveller.cpp)
ament_target_dependencies(timeTraveller rclcpp tf2 tf2_ros)

add_executable(nodeWithParam src/node_with_param.cpp)
ament_target_dependencies(nodeWithParam rclcpp tf2 tf2_ros)

add_executable(dynamicReconfiguration src/dynamic_reconfiguration.cpp)
ament_target_dependencies(dynamicReconfiguration rclcpp tf2 tf2_ros)

add_executable(serviceServer src/service_server.cpp)
ament_target_dependencies(serviceServer rclcpp example_interfaces)

add_executable(serviceClient src/service_client.cpp)
ament_target_dependencies(serviceClient rclcpp example_interfaces)

add_executable(actionServer src/action_server.cpp)
ament_target_dependencies(actionServer rclcpp action_msgs rclcpp_action)
target_link_libraries(actionServer ${cpp_typesupport_target})

add_executable(actionClient src/action_client.cpp)
ament_target_dependencies(actionClient rclcpp action_msgs rclcpp_action)
target_link_libraries(actionClient ${cpp_typesupport_target})

add_executable(lifecycleNode src/lifecycle_node.cpp)
ament_target_dependencies(lifecycleNode rclcpp_lifecycle std_msgs lifecycle_msgs)

add_executable(simpleCameraDisplay src/simple_camera_display.cpp)
ament_target_dependencies(simpleCameraDisplay rclcpp std_msgs sensor_msgs cv_bridge)
target_link_libraries(simpleCameraDisplay ${OpenCV_LIBRARIES})

add_executable(aprilTagDetection src/april_tag_detection.cpp)
ament_target_dependencies(aprilTagDetection rclcpp std_msgs sensor_msgs cv_bridge)
target_link_libraries(aprilTagDetection ${OpenCV_LIBRARIES})

add_executable(colourDetection src/colour_detection.cpp)
ament_target_dependencies(colourDetection rclcpp std_msgs sensor_msgs cv_bridge)
target_link_libraries(colourDetection ${OpenCV_LIBRARIES})

add_executable(navigate src/navigate.cpp)
ament_target_dependencies(navigate rclcpp action_msgs rclcpp_action nav2_msgs)

add_executable(moveForwardServer src/move_forward_service.cpp)
ament_target_dependencies(moveForwardServer rclcpp geometry_msgs)
target_link_libraries(moveForwardServer ${cpp_typesupport_target})

add_executable(moveForwardClient src/move_forward_client.cpp)
ament_target_dependencies(moveForwardClient rclcpp geometry_msgs)
target_link_libraries(moveForwardClient ${cpp_typesupport_target})

add_executable(decorator src/decorator.cpp)
ament_target_dependencies(decorator rclcpp geometry_msgs)

add_executable(factory src/factory.cpp)
ament_target_dependencies(factory rclcpp std_msgs)


install(TARGETS
  simplePublisher
  simpleSubscriber
  simplePublisherCustom
  simpleSubscriberCustom
  staticBroadcast
  dynamicBroadcast
  frameListener
  timeTraveller
  nodeWithParam
  dynamicReconfiguration
  serviceServer
  serviceClient
  actionServer
  actionClient
  lifecycleNode
  simpleCameraDisplay
  aprilTagDetection
  colourDetection
  navigate
  moveForwardServer
  moveForwardClient
  decorator
  factory
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
