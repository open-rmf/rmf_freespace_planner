cmake_minimum_required(VERSION 3.5.0)

project(rmf_freespace_planner)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmf_traffic REQUIRED)

include(GNUInstallDirs)

add_library(
  ${PROJECT_NAME}
    src/kinodynamic_rrt_star.cpp
    src/posq.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
    PRIVATE
      ${nav_msgs_LIBRARIES}
      ${rclcpp_LIBRARIES}
      rmf_traffic::rmf_traffic
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_executable(test_freespace_planner test_freespace_planner.cpp)

target_link_libraries(
  test_freespace_planner
  PRIVATE
    ${PROJECT_NAME}
    ${nav_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
    rmf_traffic::rmf_traffic
)

target_include_directories(
    test_freespace_planner
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

ament_export_targets(rmf_freespace_planner HAS_LIBRARY_TARGET)

install(
  TARGETS
    rmf_freespace_planner
    test_freespace_planner
  EXPORT rmf_freespace_planner
  RUNTIME DESTINATION lib/rmf_freespace_planner
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/
)
ament_export_include_directories(
  include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
