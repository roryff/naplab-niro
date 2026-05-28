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

add_executable(lateral_mpc_node src/lateral_mpc_node.cpp)
ament_target_dependencies(lateral_mpc_node rclcpp std_msgs geometry_msgs nav_msgs)
target_include_directories(lateral_mpc_node PRIVATE ${osqp_SOURCE_DIR}/include)
target_link_libraries(lateral_mpc_node osqpstatic ${cpp_typesupport_target} Eigen3::Eigen)
add_executable(sysid_node src/sysid_node.cpp)
ament_target_dependencies(sysid_node rclcpp std_msgs geometry_msgs)
target_link_libraries(sysid_node ${cpp_typesupport_target})

add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node rclcpp rclcpp_components sensor_msgs foxglove_msgs)
target_include_directories(camera_node PRIVATE ${GST_INCLUDE_DIRS})
target_link_libraries(camera_node ${GST_LIBRARIES})

# Composable component library (for intra-process use with isaac_ros_h264_encoder)
add_library(camera_node_component SHARED src/camera_node.cpp)
ament_target_dependencies(camera_node_component rclcpp rclcpp_components sensor_msgs foxglove_msgs)
target_include_directories(camera_node_component PRIVATE ${GST_INCLUDE_DIRS})
target_link_libraries(camera_node_component ${GST_LIBRARIES})
rclcpp_components_register_node(camera_node_component
  PLUGIN "CameraNode"
  EXECUTABLE camera_node_component_exe
)


# Install executables
install(TARGETS
  gnss_node
  comma_node
  path_follower_node
  steering_mpc_node
  lateral_mpc_node
  sysid_node
  camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install camera component library
install(TARGETS camera_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python scripts
install(PROGRAMS
  scripts/dashboard_server.py
  scripts/Kia_niro_rosnode.py
  scripts/Kia-rosnode_v2.py
  scripts/nmpc_path_follower_node.py
  scripts/nmpc_core.py
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
