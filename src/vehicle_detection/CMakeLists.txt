cmake_minimum_required(VERSION 3.16)
project(vehicle_detection LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)

add_library(vehicle_detection_core
  src/parameter_validation.cpp
)
target_include_directories(vehicle_detection_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(vehicle_detection_core PUBLIC cxx_std_17)

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

install(TARGETS
  vehicle_detection_core
  EXPORT export_vehicle_detection
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  pcd_loader_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
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
endif()

ament_export_include_directories(include)
ament_export_libraries(vehicle_detection_core)
ament_package()
