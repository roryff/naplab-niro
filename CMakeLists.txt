cmake_minimum_required(VERSION 3.8)
project(car_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# GStreamer (for camera_node)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

# Set path for nlohmann_json
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/share/cmake")
find_package(nlohmann_json REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DriveCommand.msg"
  "msg/VehicleState.msg"
  "msg/EsfSensor.msg"
  "msg/EsfStatus.msg"
  "msg/MpcStatus.msg"
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
  tf2
  tf2_ros
  tf2_geometry_msgs
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
ament_target_dependencies(path_follower_node rclcpp std_msgs geometry_msgs nav_msgs)
target_include_directories(path_follower_node PRIVATE ${osqp_SOURCE_DIR}/include)
target_link_libraries(path_follower_node osqpstatic ${cpp_typesupport_target} Eigen3::Eigen)

add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node rclcpp rclcpp_components sensor_msgs foxglove_msgs)
target_include_directories(camera_node PRIVATE ${GST_INCLUDE_DIRS})
target_link_libraries(camera_node ${GST_LIBRARIES})

# Composable component library for camera_node
add_library(camera_node_component SHARED src/camera_node.cpp)
ament_target_dependencies(camera_node_component rclcpp rclcpp_components sensor_msgs foxglove_msgs)
target_include_directories(camera_node_component PRIVATE ${GST_INCLUDE_DIRS})
target_link_libraries(camera_node_component ${GST_LIBRARIES})
rclcpp_components_register_node(camera_node_component
  PLUGIN "CameraNode"
  EXECUTABLE camera_node_component_exe
)

# Foxglove relay — lazy H.265→H.264 transcode for Foxglove Studio preview
add_executable(foxglove_relay_node src/foxglove_relay_node.cpp)
ament_target_dependencies(foxglove_relay_node rclcpp rclcpp_components foxglove_msgs)
target_include_directories(foxglove_relay_node PRIVATE ${GST_INCLUDE_DIRS})
target_link_libraries(foxglove_relay_node ${GST_LIBRARIES})

add_library(foxglove_relay_node_component SHARED src/foxglove_relay_node.cpp)
ament_target_dependencies(foxglove_relay_node_component rclcpp rclcpp_components foxglove_msgs)
target_include_directories(foxglove_relay_node_component PRIVATE ${GST_INCLUDE_DIRS})
target_link_libraries(foxglove_relay_node_component ${GST_LIBRARIES})
rclcpp_components_register_node(foxglove_relay_node_component
  PLUGIN "FoxgloveRelayNode"
  EXECUTABLE foxglove_relay_node_component_exe
)


# Install executables
install(TARGETS
  gnss_node
  comma_node
  path_follower_node
  camera_node
  foxglove_relay_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install component libraries
install(TARGETS camera_node_component foxglove_relay_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python scripts
install(PROGRAMS
  scripts/dashboard_server.py
  # scripts/Kia_niro_rosnode.py
  # scripts/Kia-rosnode_v2.py
  # scripts/nmpc_path_follower_node.py
  # scripts/nmpc_core.py
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

# Install mesh files
install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}
)

# Install bundled paths
install(DIRECTORY
  paths
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
