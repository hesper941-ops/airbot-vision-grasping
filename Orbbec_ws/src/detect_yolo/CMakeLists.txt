cmake_minimum_required(VERSION 3.8)
project(detect_yolo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(dnn_node REQUIRED)

add_executable(detect_yolo_node src/detect_yolo_node.cpp)

target_include_directories(detect_yolo_node PRIVATE
  /usr/include
  ${dnn_node_INCLUDE_DIRS}
  ${ai_msgs_INCLUDE_DIRS}
)

target_link_directories(detect_yolo_node PRIVATE /usr/hobot/lib)

ament_target_dependencies(detect_yolo_node
  rclcpp
  sensor_msgs
  cv_bridge
  geometry_msgs
  std_msgs
  ai_msgs
  dnn_node
)

target_link_libraries(detect_yolo_node
  /usr/hobot/lib/libalog.so
  /usr/hobot/lib/libcnn_intf.so
)

install(TARGETS detect_yolo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
