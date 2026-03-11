cmake_minimum_required(VERSION 3.10)
project(3dof_arm_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED) # Needed for YamlReader class
find_package(tinyxml2 REQUIRED) # Needed for MiniURDFParser class
find_package(pinocchio REQUIRED) # Needed for PinochioSolver class

# -----------------------------------------------------------
# 1. Build the Pure C++ Core Library (Non-ROS related)/ Note: "core" folder contains non-ros related files, "ros" folder contains ros related files.
# -----------------------------------------------------------
add_library(core_kinematics src/core/AnalyticalSolver.cpp src/core/PinocchioSolver.cpp src/core/MiniURDFParser.cpp)
target_include_directories(core_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Needed for classes: YamlReader, MiniURDFParser, PinocchioSolver
target_link_libraries(core_kinematics PUBLIC yaml-cpp tinyxml2::tinyxml2 pinocchio::pinocchio)

# -----------------------------------------------------------
# 2. Build the ROS Node Classes as Libraries (Component Ready)
# -----------------------------------------------------------
add_library(fk_publisher_lib src/ros/FkPublisherNode.cpp)
target_include_directories(fk_publisher_lib PUBLIC include)
target_link_libraries(fk_publisher_lib core_kinematics)
ament_target_dependencies(fk_publisher_lib rclcpp geometry_msgs std_msgs)

add_library(ik_subscriber_lib src/ros/IkSubscriberNode.cpp)
target_include_directories(ik_subscriber_lib PUBLIC include)
target_link_libraries(ik_subscriber_lib core_kinematics)
ament_target_dependencies(ik_subscriber_lib rclcpp geometry_msgs std_msgs)

# -----------------------------------------------------------
# 3. Build the Executables (Entry points)
# -----------------------------------------------------------
add_executable(fk_publisher_node exe/fk_publisher_main.cpp)
target_include_directories(fk_publisher_node PUBLIC include)
target_link_libraries(fk_publisher_node fk_publisher_lib)
ament_target_dependencies(fk_publisher_node rclcpp)

add_executable(ik_subscriber_node exe/ik_subscriber_main.cpp)
target_include_directories(ik_subscriber_node PUBLIC include)
target_link_libraries(ik_subscriber_node ik_subscriber_lib)
ament_target_dependencies(ik_subscriber_node rclcpp)

# -----------------------------------------------------------
# 4. Install Rules
# -----------------------------------------------------------
install(TARGETS 
  core_kinematics
  fk_publisher_lib
  ik_subscriber_lib
  fk_publisher_node
  ik_subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# Install the config folder containing yaml files to the ROS 2 share directory so the launch file can find it.
# This is so yaml filepaths can be passed into the ROS node via the ROS launch files.
install(DIRECTORY include/3dof_arm_kinematics/config/ 
  DESTINATION share/${PROJECT_NAME}/config
)

# -----------------------------------------------------------
# 5. Testing
# -----------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ### Disabling lint tests for now
  # ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(test_AnalyticalSolver test/test_AnalyticalSolver.cpp)
  target_link_libraries(test_AnalyticalSolver core_kinematics)

  ament_add_gtest(test_PinocchioSolver test/test_PinocchioSolver.cpp)
  target_link_libraries(test_PinocchioSolver core_kinematics)

  ament_add_gtest(test_KinematicSolverSimpleGeneralized test/test_KinematicSolverSimpleGeneralized.cpp)
  target_link_libraries(test_KinematicSolverSimpleGeneralized core_kinematics)

  # To avoid hardcoding filepaths within test .cpp files: Use CMake to Inject the absolute path to the test/ directory as a macro named TEST_DIR
  target_compile_definitions(test_PinocchioSolver PRIVATE PKG_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_compile_definitions(test_AnalyticalSolver PRIVATE PKG_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_compile_definitions(test_KinematicSolverSimpleGeneralized PRIVATE PKG_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

endif()

ament_package()