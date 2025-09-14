cmake_minimum_required(VERSION 3.8)
project(detection_and_mot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(shm_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Open3D)
find_package(Boost QUIET)
find_package(PCL REQUIRED QUIET COMPONENTS common io)
find_package(pcl_msgs REQUIRED)

find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 3 REQUIRED)
endif()

find_package(iceoryx_posh CONFIG REQUIRED)
find_package(fastrtps CONFIG REQUIRED)
find_package(common_msgs REQUIRED)

# list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/onnxruntime/lib64/cmake/onnxruntime")

# find_package(onnxruntime REQUIRED)

add_executable(detection_and_mot_node src/detection_and_mot_node.cpp)

target_include_directories(detection_and_mot_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/onnxruntime/include/>
  $<INSTALL_INTERFACE:include>)
target_compile_features(detection_and_mot_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  detection_and_mot_node
  "rclcpp"
  "shm_msgs"
  "common_msgs"
  "message_filters"
  "cv_bridge"
)

target_link_libraries(detection_and_mot_node
  ${OpenCV_LIBRARIES}
  # ${ONNXRUNTIME_LIBRARIES}
  # onnxruntime::onnxruntime 
  "${CMAKE_CURRENT_SOURCE_DIR}/onnxruntime/lib/libonnxruntime.so"
)

target_include_directories(detection_and_mot_node BEFORE PRIVATE
    ${shm_msgs_DIR}/../../../include
)

install(TARGETS detection_and_mot_node
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY resources/
DESTINATION share/${PROJECT_NAME}/resources)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
