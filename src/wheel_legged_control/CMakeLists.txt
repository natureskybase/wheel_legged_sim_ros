cmake_minimum_required(VERSION 3.8)
project(wheel_legged_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(./include)
include_directories(./include/joystick)
# include_directories(./include/legged_control)
# include_directories(./include/leg_control)
# include_directories(./include/legged_gait)
# include_directories(./include/mpc_solve)
# include_directories(../../qpOASES/include)
file(GLOB_RECURSE sources "./src/*.cpp")
list(FILTER sources EXCLUDE REGEX ".*/main\\.cpp$")
file(GLOB_RECURSE headfile "./include/*.h")

set(dependencies
  rclcpp
  tf2
  tf2_ros
  nav_msgs
  sensor_msgs
)

set(libs
  Eigen3::Eigen
  qpOASES
)

add_executable(main_test ./src/main.cpp ${sources})

target_link_libraries(main_test ${libs})

ament_target_dependencies(main_test ${dependencies})

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

ament_package()
