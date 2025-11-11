cmake_minimum_required(VERSION 3.8)
project(robocup_vision25)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(humanoid_interfaces REQUIRED)


# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS} # cv_bridge include 디렉토리 추가
)

add_executable(robocup_vision25_node src/robocup_vision25.cpp)

target_include_directories(robocup_vision25_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(robocup_vision25_node PUBLIC c_std_99 cxx_std_17)

target_link_libraries(robocup_vision25_node
  ${rclcpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${image_transport_LIBRARIES}
)

ament_target_dependencies(robocup_vision25_node
  rclcpp
  std_msgs
  cv_bridge
  sensor_msgs
  image_transport
  OpenCV
  humanoid_interfaces
)

install(TARGETS
  robocup_vision25_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()


