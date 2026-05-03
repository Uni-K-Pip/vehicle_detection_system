cmake_minimum_required(VERSION 3.16)
project(vehicle_detection LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(MPI REQUIRED COMPONENTS C)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation search)
find_package(nlohmann_json REQUIRED)

# cpp-httplib: header-only MIT-licensed HTTP server used by the
# parameter_bridge_node Web GUI. Prefer a sufficiently new system package;
# otherwise fetch a pinned upstream release so the repo stays unvendored.
find_package(httplib 0.27.0 QUIET)
if(NOT httplib_FOUND)
  include(FetchContent)
  set(HTTPLIB_REQUIRE_OPENSSL OFF CACHE BOOL "" FORCE)
  set(HTTPLIB_REQUIRE_ZLIB OFF CACHE BOOL "" FORCE)
  set(HTTPLIB_REQUIRE_BROTLI OFF CACHE BOOL "" FORCE)
  set(HTTPLIB_REQUIRE_ZSTD OFF CACHE BOOL "" FORCE)
  set(HTTPLIB_USE_OPENSSL_IF_AVAILABLE OFF CACHE BOOL "" FORCE)
  set(HTTPLIB_USE_ZLIB_IF_AVAILABLE OFF CACHE BOOL "" FORCE)
  set(HTTPLIB_USE_BROTLI_IF_AVAILABLE OFF CACHE BOOL "" FORCE)
  set(HTTPLIB_USE_ZSTD_IF_AVAILABLE OFF CACHE BOOL "" FORCE)
  fetchcontent_declare(
    cpp_httplib
    GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
    GIT_TAG v0.28.0
    GIT_SHALLOW TRUE
  )
  fetchcontent_makeavailable(cpp_httplib)
endif()

# Pure parameter validation helpers and JSON / HTTP utilities (no PCL).
add_library(vehicle_detection_core
  src/parameter_validation.cpp
  src/detection_json.cpp
  src/detection_result_writer.cpp
  src/http_client.cpp
  src/parameter_json.cpp
  src/pcd_playlist.cpp
)
target_include_directories(vehicle_detection_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(vehicle_detection_core
  PUBLIC nlohmann_json::nlohmann_json
)
ament_target_dependencies(vehicle_detection_core
  PUBLIC rclcpp rcl_interfaces
)
target_compile_features(vehicle_detection_core PUBLIC cxx_std_17)

# PCL-backed point cloud processing pipeline.
add_library(vehicle_detection_pcl
  src/point_cloud_processing.cpp
)
target_include_directories(vehicle_detection_pcl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_compile_features(vehicle_detection_pcl PUBLIC cxx_std_17)
target_link_libraries(vehicle_detection_pcl PUBLIC ${PCL_LIBRARIES})

add_executable(pcd_loader_node src/pcd_loader_node.cpp)
target_include_directories(pcd_loader_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(pcd_loader_node
  vehicle_detection_core
  ${PCL_LIBRARIES}
)
ament_target_dependencies(pcd_loader_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

add_executable(vehicle_detector_node src/vehicle_detector_node.cpp)
target_include_directories(vehicle_detector_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(vehicle_detector_node
  vehicle_detection_core
  vehicle_detection_pcl
  ${PCL_LIBRARIES}
)
ament_target_dependencies(vehicle_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  vision_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
)

add_executable(detection_sender_node src/detection_sender_node.cpp)
target_include_directories(detection_sender_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(detection_sender_node
  vehicle_detection_core
)
ament_target_dependencies(detection_sender_node
  rclcpp
  vision_msgs
)

add_executable(parameter_bridge_node src/parameter_bridge_node.cpp)
target_include_directories(parameter_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(parameter_bridge_node
  vehicle_detection_core
  httplib::httplib
  nlohmann_json::nlohmann_json
)
ament_target_dependencies(parameter_bridge_node
  rclcpp
  rcl_interfaces
  ament_index_cpp
)

install(TARGETS
  vehicle_detection_core
  vehicle_detection_pcl
  EXPORT export_vehicle_detection
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  pcd_loader_node
  vehicle_detector_node
  detection_sender_node
  parameter_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config rviz web
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_parameter_validation
    test/test_parameter_validation.cpp
  )
  target_link_libraries(test_parameter_validation
    vehicle_detection_core
  )
  target_include_directories(test_parameter_validation PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_pcd_playlist
    test/test_pcd_playlist.cpp
  )
  target_link_libraries(test_pcd_playlist
    vehicle_detection_core
  )
  target_include_directories(test_pcd_playlist PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_point_cloud_processing
    test/test_point_cloud_processing.cpp
  )
  target_link_libraries(test_point_cloud_processing
    vehicle_detection_pcl
    ${PCL_LIBRARIES}
  )
  target_include_directories(test_point_cloud_processing PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
  )

  ament_add_gtest(test_detection_json
    test/test_detection_json.cpp
  )
  target_link_libraries(test_detection_json
    vehicle_detection_core
  )
  target_include_directories(test_detection_json PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_detection_result_writer
    test/test_detection_result_writer.cpp
  )
  target_link_libraries(test_detection_result_writer
    vehicle_detection_core
  )
  target_include_directories(test_detection_result_writer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_parameter_json
    test/test_parameter_json.cpp
  )
  target_link_libraries(test_parameter_json
    vehicle_detection_core
    nlohmann_json::nlohmann_json
  )
  target_include_directories(test_parameter_json PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_parameter_json
    rclcpp
  )

  ament_add_pytest_test(test_launch_description
    test/test_launch_description.py
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(vehicle_detection_core vehicle_detection_pcl)
ament_export_dependencies(
  rclcpp
  rcl_interfaces
  nlohmann_json
)
ament_package()
