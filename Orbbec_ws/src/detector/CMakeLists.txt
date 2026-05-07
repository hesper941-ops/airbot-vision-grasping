cmake_minimum_required(VERSION 3.8)
project(detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(apple_detector_node src/apple_detector_node.cpp)
ament_target_dependencies(apple_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  geometry_msgs
)
target_link_libraries(apple_detector_node
  ${OpenCV_LIBS}
)

add_executable(duck_detector_node src/duck_detector_node.cpp)
ament_target_dependencies(duck_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  geometry_msgs
)
target_link_libraries(duck_detector_node
  ${OpenCV_LIBS}
)

add_executable(box_detector_node src/box_detector_node.cpp)
ament_target_dependencies(box_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  geometry_msgs
)
target_link_libraries(box_detector_node
  ${OpenCV_LIBS}
)

install(TARGETS
  apple_detector_node
  duck_detector_node
  box_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()