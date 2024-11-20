cmake_minimum_required(VERSION 3.5)
project(maneuver_camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
# endif()
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -g)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
# find_package(pcl REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

# add_library(maneuver_camera_node
#   ./src/maneuver_camera_node.cpp
# )

# target_link_libraries(maneuver_camera_node
#   ${OpenCV_LIBRARIES}
# )

# ament_target_dependencies(maneuver_camera_node cv_bridge rclcpp sensor_msgs pcl_ros pcl_conversions)

add_library(floor_projection_node
  ./src/floor_projection_node.cpp
)

target_link_libraries(floor_projection_node
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(floor_projection_node cv_bridge rclcpp sensor_msgs pcl_ros pcl_conversions)

add_executable(floor_projection
  ./src/floor_projection.cpp
)

target_link_libraries(floor_projection
  floor_projection_node
)

# add_executable(maneuver_camera
#   ./src/maneuver_camera.cpp
# )

# target_link_libraries(maneuver_camera
#   maneuver_camera_node
# )

install(TARGETS floor_projection_node floor_projection
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
