cmake_minimum_required(VERSION 3.16)
project(autolivery_drone)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(autolivery_core REQUIRED)
find_package(autolivery_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

# Map loading, parameters, planning and conversions shared by every node.
add_library(${PROJECT_NAME}_common STATIC
  src/city_map.cpp
  src/node_main.cpp
  src/params.cpp
  src/route_planner.cpp
)
set_target_properties(${PROJECT_NAME}_common PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME}_common PUBLIC
  autolivery_core::autolivery_core
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  yaml-cpp::yaml-cpp
  ${geometry_msgs_TARGETS}
  ${mavros_msgs_TARGETS}
)

function(autolivery_node name)
  add_executable(${name} src/${name}.cpp)
  target_link_libraries(${name} ${PROJECT_NAME}_common ${ARGN})
  install(TARGETS ${name} DESTINATION lib/${PROJECT_NAME})
endfunction()

autolivery_node(mission_node
  rclcpp_action::rclcpp_action
  ${autolivery_interfaces_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS})
autolivery_node(payload_node
  ${autolivery_interfaces_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS})
autolivery_node(planner_node
  ${autolivery_interfaces_TARGETS}
  ${nav_msgs_TARGETS}
  ${visualization_msgs_TARGETS})
autolivery_node(adsb_traffic_node
  ${autolivery_interfaces_TARGETS})
autolivery_node(fake_fcu_node
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS})

install(DIRECTORY config launch maps rviz DESTINATION share/${PROJECT_NAME})

# The city map loader and planner are reused by autolivery_gazebo (simulated traffic).
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_common EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp autolivery_core geometry_msgs mavros_msgs rclcpp yaml_cpp_vendor yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # ROS 2 C++ style. No per-file copyright headers: the licence is the repository's LICENSE.
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  ament_uncrustify()
  ament_cpplint(FILTERS "-legal/copyright")
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8()
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(test_city_map test/test_city_map.cpp TIMEOUT 300)
  target_link_libraries(test_city_map ${PROJECT_NAME}_common)
  target_compile_definitions(test_city_map PRIVATE
    AUTOLIVERY_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

  ament_add_gtest(test_profile_missions test/test_profile_missions.cpp TIMEOUT 600)
  target_link_libraries(test_profile_missions ${PROJECT_NAME}_common)
  target_compile_definitions(test_profile_missions PRIVATE
    AUTOLIVERY_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_link_libraries(test_conversions ${PROJECT_NAME}_common)

  ament_add_gtest(test_node_main test/test_node_main.cpp TIMEOUT 120)
  target_link_libraries(test_node_main ${PROJECT_NAME}_common)

  # Full ROS graph against the fake flight controller: action in, parcel out.
  add_launch_test(test/test_dry_run_delivery.py TIMEOUT 600)
  add_launch_test(test/test_dry_run_failsafes.py TIMEOUT 600)
  add_launch_test(test/test_dry_run_fixed_wing.py TIMEOUT 900)
endif()

ament_package()
