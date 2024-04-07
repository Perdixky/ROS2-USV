cmake_minimum_required(VERSION 3.8)
project(usv_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
)

## COMPILE
add_library(
  usv_hardware
  SHARED
  src/usv_hardware.cpp
)

ament_target_dependencies(
  usv_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface usv_hardware.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/usv_hardware
)

install(
  FILES  description/usv.urdf.xacro
  DESTINATION share/usv_hardware/urdf
)

install(
  FILES  description/usv_core.xacro
  FILES control/control.xacro
  DESTINATION share/usv_hardware
)

install(
  DIRECTORY bringup
  DESTINATION share/usv_hardware
)

install(
  DIRECTORY config
  DESTINATION share/usv_hardware
)

install(TARGETS usv_hardware
  EXPORT export_usv_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES usv_hardware.xml
  DESTINATION share
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

ament_export_targets(export_usv_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()