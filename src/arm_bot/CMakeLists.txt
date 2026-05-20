cmake_minimum_required(VERSION 3.8)
project(arm_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


install(
  DIRECTORY
    urdf
    launch
    config
    rviz
    meshes
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(arm_bot src/move_program.cpp)
ament_target_dependencies(arm_bot
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  visualization_msgs
  tf2_ros
)

install(TARGETS
  arm_bot
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/slider_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# install(PROGRAMS
#   ${PROJECT_NAME}/fk_arm.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(PROGRAMS
#   ${PROJECT_NAME}/ik_arm.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(PROGRAMS
#   ${PROJECT_NAME}/ik_arm2.py
#   DESTINATION lib/${PROJECT_NAME}
# )

install(PROGRAMS
  ${PROJECT_NAME}/relay_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/drawing_trajectory_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

 install(PROGRAMS
   ${PROJECT_NAME}/moveit_client.py
   DESTINATION lib/${PROJECT_NAME}
 )

install(PROGRAMS
  ${PROJECT_NAME}/ik_verifier.py
  DESTINATION lib/${PROJECT_NAME}
)

# install(PROGRAMS
#   ${PROJECT_NAME}/ik_analytical.py
#   DESTINATION lib/${PROJECT_NAME}
# )

install(PROGRAMS
  ${PROJECT_NAME}/ik_arm_v3.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/fk_arm_v3.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/ik_to_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/cartesian_path.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/send_test_goal.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/compare_runs.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/drawing_ui_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/drawing_executor_node.py
  DESTINATION lib/${PROJECT_NAME}
)


# install(PROGRAMS
#   ${PROJECT_NAME}/move_v2.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(PROGRAMS
#   ${PROJECT_NAME}/move_v3.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Optional: install individual files if you have loose ones (e.g. README)
# install(FILES README.md DESTINATION share/${PROJECT_NAME})

# ────────────────────────────────────────────────
# Testing / linting (good practice – optional but recommended)
# ────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# This must be the last line!
ament_package()
