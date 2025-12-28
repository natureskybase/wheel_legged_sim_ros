cmake_minimum_required(VERSION 3.8)
project(mujoco_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(MUJOCO_PATH /opt/mujoco)
set(MUJOCO_LIB ${MUJOCO_PATH}/lib/libmujoco.so)

include_directories(${MUJOCO_PATH}/include)
link_directories(${MUJOCO_PATH}/bin)

#simulate编译
include_directories(./simulate)
link_directories(./simulate)
file(GLOB SIMULATE_SRC ./src/main.cpp ./simulate/*.cc ./simulate/*.cpp)
add_executable(simulate ${SIMULATE_SRC})
target_link_libraries(simulate ${MUJOCO_LIB} glut GL GLU glfw)

ament_target_dependencies(simulate
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
)

install(TARGETS
  simulate
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
