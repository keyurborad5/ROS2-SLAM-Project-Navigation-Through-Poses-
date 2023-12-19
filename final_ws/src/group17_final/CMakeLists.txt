cmake_minimum_required(VERSION 3.8)
project(group17_final)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mage_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# include folder whish has all the header files
include_directories(include/group17_final)
add_executable(my_robot_node src/my_robot_node.cpp)

ament_target_dependencies(my_robot_node 
                          ament_cmake 
                          rclcpp
                          rclcpp_action
                          rclcpp_components 
                          mage_msgs 
                          geometry_msgs 
                          nav_msgs 
                          nav2_msgs
                          std_msgs 
                          tf2_ros 
                          tf2 
                          tf2_geometry_msgs
                          example_interfaces 
                          ros2_aruco_interfaces)

# install(TARGETS 
# my_robot_node 
# RUNTIME DESTINATION lib/${PROJECT_NAME}
# )



install(TARGETS
my_robot_node
DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include launch config DESTINATION share/${PROJECT_NAME}/
)

ament_package()
