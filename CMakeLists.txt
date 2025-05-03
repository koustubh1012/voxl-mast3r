cmake_minimum_required(VERSION 3.8)
project(voxl_mast3r)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include/voxl_mast3r)

# add the executable
add_executable(voxl_mast3r src/capture_image_2.cpp)

# link the libraries
target_link_libraries(voxl_mast3r
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
)
target_include_directories(voxl_mast3r
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(voxl_mast3r
  rclcpp
  px4_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  OpenCV
  cv_bridge
  tf2
)
# install the executable
install(TARGETS
  voxl_mast3r
  DESTINATION lib/${PROJECT_NAME}
)

# install the hpp files
install(DIRECTORY include/
  DESTINATION include/
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
