cmake_minimum_required(VERSION 3.22)
project(arm_moveit_config)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(move src/move.cpp)
target_include_directories(move PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  move
  "moveit_ros_planning_interface"
  "rclcpp"
  "trajectory_msgs"
  "moveit_msgs"
  "moveit_core"
)

add_executable(tf_broadcast src/tf_broadcast.cpp)
target_include_directories(tf_broadcast PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tf_broadcast PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  tf_broadcast
  "moveit_ros_planning_interface"
  "rclcpp"
  "trajectory_msgs"
  "tf2_msgs"
  "tf2_ros"
  "tf2"
  "geometry_msgs"
  "moveit_msgs"
  "moveit_core"
)

add_executable(circle_trajectory src/circle_trajectory.cpp)
target_include_directories(circle_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(circle_trajectory PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  circle_trajectory
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2_ros"
  "tf2"
  "tf2_geometry_msgs"
  "geometry_msgs"
  "moveit_msgs"
  "moveit_core"
  "visualization_msgs"
  "std_msgs"
  "std_srvs"
)
target_link_libraries(circle_trajectory Eigen3::Eigen)

add_executable(square_trajectory src/square_trajectory.cpp)
target_include_directories(square_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(square_trajectory PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  square_trajectory
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2_ros"
  "tf2"
  "tf2_geometry_msgs"
  "geometry_msgs"
  "moveit_msgs"
  "moveit_core"
  "visualization_msgs"
  "std_msgs"
  "std_srvs"
)
target_link_libraries(square_trajectory Eigen3::Eigen)

add_executable(text_trajectory src/text_trajectory.cpp)
target_include_directories(text_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(text_trajectory PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  text_trajectory
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
  "moveit_msgs"
  "moveit_core"
  "visualization_msgs"
  "std_msgs"
  "std_srvs"
)

add_executable(cartesian_move src/cartesian_move.cpp)
target_include_directories(cartesian_move PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cartesian_move PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  cartesian_move
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2_ros"
  "tf2"
  "tf2_geometry_msgs"
  "geometry_msgs"
  "moveit_msgs"
  "moveit_core"
  "visualization_msgs"
  "std_msgs"
  "std_srvs"
)
target_link_libraries(cartesian_move Eigen3::Eigen)

install(TARGETS move tf_broadcast circle_trajectory square_trajectory cartesian_move text_trajectory
  DESTINATION lib/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
