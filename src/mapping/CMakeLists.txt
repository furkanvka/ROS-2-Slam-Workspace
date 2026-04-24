cmake_minimum_required(VERSION 3.8)
project(mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ─── Dependencies ─────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ─── Localization Node ───────────────────────
add_executable(localization_node src/localization_node.cpp)

target_include_directories(localization_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(localization_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(localization_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
)

# ─── Mapping Node ────────────────────────────
add_executable(mapping_node src/mapping_node.cpp)

target_include_directories(mapping_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(mapping_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(mapping_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
)


# ───  Node ────────────────────────path────
add_executable(path_node src/path_node.cpp)

target_include_directories(path_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(path_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(path_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
)


# ─── Explorer Node ────────────────────────────
add_executable(frontier_node src/frontier_node.cpp)

target_include_directories(frontier_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(frontier_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(frontier_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
)

# ─── Install ─────────────────────────────────
install(TARGETS
  localization_node
  mapping_node
  frontier_node
  path_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


# ─── Test ────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()