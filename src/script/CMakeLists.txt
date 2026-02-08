cmake_minimum_required(VERSION 3.8)
project(script)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)


# Executables and libraries
add_executable(talker function/talker.cpp)
add_executable(ball_chase function/ball_chase.cpp)
add_executable(image_viewer function/image_viewer.cpp)
add_executable(listener function/listener.cpp)
add_executable(hsv_finder function/HSV_finder.cpp)

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

ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(ball_chase
  rclcpp
  sensor_msgs
  cv_bridge
)

# OpenCV is NOT an ament package
target_link_libraries(ball_chase
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  ball_chase
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(hsv_finder
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

target_link_libraries(hsv_finder
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  hsv_finder
  DESTINATION lib/${PROJECT_NAME}
)

# Image Viewer
ament_target_dependencies(image_viewer
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(image_viewer
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  image_viewer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
