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
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ---------------------------------------------------------------
# OSQP – fetch and build from source with 64-bit integers
# ---------------------------------------------------------------
include(FetchContent)
FetchContent_Declare(
  osqp
  GIT_REPOSITORY https://github.com/osqp/osqp
  GIT_TAG        master
  GIT_SHALLOW    TRUE
)
set(OSQP_USE_LONG     ON  CACHE BOOL "Use 64-bit integers in OSQP" FORCE)
set(OSQP_USE_FLOAT    OFF CACHE BOOL "Use double precision in OSQP" FORCE)
set(OSQP_BUILD_UNITTESTS OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_DEMO_EXE  OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(osqp)
# ---------------------------------------------------------------

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

add_executable(steering_mpc_node src/steering_mpc_node.cpp)
ament_target_dependencies(steering_mpc_node rclcpp std_msgs geometry_msgs)
target_include_directories(steering_mpc_node PRIVATE ${osqp_SOURCE_DIR}/include)
target_link_libraries(steering_mpc_node osqpstatic ${cpp_typesupport_target})

# Install executables
install(TARGETS
  gnss_node
  comma_node
  path_follower_node
  steering_mpc_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/dashboard_server.py
  scripts/drive_recorder.py
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

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
