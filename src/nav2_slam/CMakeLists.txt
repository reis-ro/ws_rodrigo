cmake_minimum_required(VERSION 3.5)
project(nav2_slam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(behaviortree_cpp_v3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(behaviortree_cpp_v3 REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Install launch files.
install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}/
)

set(CMAKE_DEBUG_POSTFIX "")

###------------U2----------------------
ament_auto_add_executable(bt_test src/bt_test.cpp)

ament_target_dependencies(bt_test behaviortree_cpp_v3)

# target_link_libraries(bt_test ${BEHAVIOR_TREE_LIBRARY})

install(TARGETS bt_test
  DESTINATION bin/${PROJECT_NAME}/
)

# install(FILES ./src/pisafundo.py DESTINATION scripts/${PROJECT_NAME})

ament_auto_package()
