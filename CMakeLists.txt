cmake_minimum_required(VERSION 3.8)
project(car_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Set path for nlohmann_json
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/share/cmake")
find_package(nlohmann_json REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleState.msg"
  "msg/EsfSensor.msg"
  "msg/EsfStatus.msg"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Expose the package's own include/ directory to all targets
include_directories(include)

# Add executables
add_executable(gnss_node src/gnss_node.cpp)
ament_target_dependencies(gnss_node 
  rclcpp
  std_msgs
  sensor_msgs 
  geometry_msgs
  nav_msgs
)

add_executable(comma_node src/comma_node.cpp)
ament_target_dependencies(comma_node 
  rclcpp 
  std_msgs 
  geometry_msgs
  nlohmann_json
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(comma_node ${cpp_typesupport_target})
target_link_libraries(gnss_node ${cpp_typesupport_target})

add_executable(path_follower_node src/path_follower_node.cpp)
ament_target_dependencies(path_follower_node 
  rclcpp 
  std_msgs
  sensor_msgs
  nav_msgs 
  geometry_msgs
)
target_link_libraries(path_follower_node ${cpp_typesupport_target})

# Install executables
install(TARGETS
  gnss_node
  comma_node
  path_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/dashboard_server.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install dashboard HTML
install(FILES
  scripts/dashboard.html
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
