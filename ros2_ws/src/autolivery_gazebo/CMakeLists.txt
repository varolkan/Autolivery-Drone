cmake_minimum_required(VERSION 3.16)
project(autolivery_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(autolivery_drone REQUIRED)
find_package(autolivery_interfaces REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(sim_traffic_node src/sim_traffic_node.cpp)
target_link_libraries(sim_traffic_node
  autolivery_drone::autolivery_drone_common
  ${autolivery_interfaces_TARGETS}
  gz-msgs::core
  gz-transport::core
  rclcpp::rclcpp
)
add_executable(sim_weather_node src/sim_weather_node.cpp)
target_link_libraries(sim_weather_node
  autolivery_drone::autolivery_drone_common
  ${autolivery_interfaces_TARGETS}
  gz-msgs::core
  gz-transport::core
  rclcpp::rclcpp
)
install(TARGETS sim_traffic_node sim_weather_node DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/generate_world.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # ROS 2 C++ style. No per-file copyright headers: the licence is the repository's LICENSE.md.
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  ament_uncrustify()
  ament_cpplint(FILTERS "-legal/copyright")
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8()
  ament_add_pytest_test(test_world ${CMAKE_CURRENT_SOURCE_DIR}/test/test_world.py
    ENV AUTOLIVERY_GAZEBO_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 120)
endif()

ament_package()
