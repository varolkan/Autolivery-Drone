cmake_minimum_required(VERSION 3.16)
project(autolivery_core VERSION 1.0.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
endif()

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME}
  src/astar.cpp
  src/gripper.cpp
  src/kinematic_vehicle.cpp
  src/mission.cpp
  src/path_follower.cpp
  src/path_smoother.cpp
  src/rrt_star.cpp
  src/vehicle_profile.cpp
  src/velocity_obstacle.cpp
  src/voxel_grid.cpp
  src/world_map.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # ROS 2 C++ style. No per-file copyright headers: the licence is the repository's LICENSE.
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  ament_uncrustify()
  ament_cpplint(FILTERS "-legal/copyright")
  set(core_tests
    test_world_map
    test_planners
    test_path_tools
    test_avoidance
    test_profiles_and_gripper
    test_mission
  )
  foreach(t ${core_tests})
    ament_add_gtest(${t} test/${t}.cpp TIMEOUT 300)
    target_link_libraries(${t} ${PROJECT_NAME})
    target_include_directories(${t} PRIVATE test)
  endforeach()
endif()

ament_package()
