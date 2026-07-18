cmake_minimum_required(VERSION 3.8)
project(emotion_landmark_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  /usr/include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(emotion_landmark_cpp_node
  src/emotion_landmark_cpp_node.cpp
)

ament_target_dependencies(emotion_landmark_cpp_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

target_link_libraries(emotion_landmark_cpp_node
  ${OpenCV_LIBS}
  dnn
)

install(TARGETS emotion_landmark_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
