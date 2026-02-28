cmake_minimum_required(VERSION 3.10)
project(planar_arm_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# -----------------------------------------------------------
# 1. Build the Pure C++ Core Library
# -----------------------------------------------------------
add_library(core_kinematics src/core/custom_solver.cpp src/core/kinematics.cpp)
target_include_directories(core_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# -----------------------------------------------------------
# 2. Build the ROS Node Classes as Libraries (Component Ready)
# -----------------------------------------------------------
add_library(fk_publisher_lib src/ros/fk_publisher_node.cpp)
target_include_directories(fk_publisher_lib PUBLIC include)
target_link_libraries(fk_publisher_lib core_kinematics)
ament_target_dependencies(fk_publisher_lib rclcpp geometry_msgs std_msgs)

add_library(ik_subscriber_lib src/ros/ik_subscriber_node.cpp)
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

# -----------------------------------------------------------
# 5. Testing
# -----------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics core_kinematics)
endif()

ament_package()