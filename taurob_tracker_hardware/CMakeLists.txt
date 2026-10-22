cmake_minimum_required(VERSION 3.16)
project(taurob_tracker_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(libtaurob REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  libtaurob
)

add_library(tracker_base_system
  src/tracker_base_system.cpp
)
target_compile_definitions(tracker_base_system PRIVATE TRACKER_BASE_SYSTEM_BUILDING_LIBRARY)
ament_target_dependencies(tracker_base_system ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(tracker_base_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
pluginlib_export_plugin_description_file(hardware_interface tracker_base_system.xml)

add_library(tracker_arm_hardware
  src/tracker_arm_hardware.cpp
)
ament_target_dependencies(tracker_arm_hardware ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(tracker_arm_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
pluginlib_export_plugin_description_file(hardware_interface tracker_arm_hardware.xml)

add_library(tracker_flipper_hardware
  src/tracker_flipper_hardware.cpp
)
ament_target_dependencies(tracker_flipper_hardware ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(tracker_flipper_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
pluginlib_export_plugin_description_file(hardware_interface tracker_flipper_hardware.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include/
  DESTINATION include
)
install(TARGETS tracker_base_system tracker_arm_hardware tracker_flipper_hardware
  EXPORT export_taurob_tracker_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES tracker_base_system.xml tracker_arm_hardware.xml tracker_flipper_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_taurob_tracker_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
