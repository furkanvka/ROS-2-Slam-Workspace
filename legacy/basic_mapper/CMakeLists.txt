cmake_minimum_required(VERSION 3.8)
project(wallfollower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================
# Dependencies
# =====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(COMMON_DEPS
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

# =====================
# Executables
# =====================

# --- Karar node'u (DFS keşif + BFS planlama + RViz viz) ---
add_executable(maze_decision_node src/maze_decision_node.cpp)
target_include_directories(maze_decision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(maze_decision_node ${COMMON_DEPS})
target_compile_features(maze_decision_node PUBLIC cxx_std_17)

# --- Hareket node'u (PID dönme + ilerleme + duvar hizalama) ---
add_executable(maze_motion_node src/maze_motion_node.cpp)
target_include_directories(maze_motion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(maze_motion_node ${COMMON_DEPS})
target_compile_features(maze_motion_node PUBLIC cxx_std_17)

# --- Hedef köprüsü (RViz clicked_point → /maze_goal grid) ---
add_executable(goal_bridge_node src/goal_bridge_node.cpp)
target_include_directories(goal_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(goal_bridge_node ${COMMON_DEPS})
target_compile_features(goal_bridge_node PUBLIC cxx_std_17)

# =====================
# Install
# =====================
install(TARGETS
  maze_decision_node
  maze_motion_node
  goal_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# =====================
# Testing
# =====================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()