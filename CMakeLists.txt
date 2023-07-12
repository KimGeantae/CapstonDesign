cmake_minimum_required(VERSION 3.8)
project(zlac_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_library(serial_LIB NAMES serial)


include_directories(include)


add_executable(zlac8015_example src/zlac8015_example.cpp src/zlac8015.cpp)
target_link_libraries(zlac8015_example ${serial_LIB})
# ament_target_dependencies(zlac8015_example rclcpp)


add_library(zlac8015 src/zlac8015.cpp) 
target_link_libraries(zlac8015_example ${serial_LIB})
# ament_target_dependencies(zlac8015 ${serial_LIB} rclcpp)
ament_export_targets(zlac8015 HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/zlac_ros2
  DESTINATION include
)

# install the executable in the lib folder to make it detectable through setup.bash
install(TARGETS 
  zlac8015_example
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  TARGETS zlac8015
  EXPORT zlac8015
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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
