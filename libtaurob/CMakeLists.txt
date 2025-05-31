cmake_minimum_required(VERSION 3.10)
project(libtaurob)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
  
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Convenience variable for dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
    diagnostic_updater
    diagnostic_msgs
    rclcpp
)
ament_export_dependencies(
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

add_library(libtaurob SHARED
  src/Arm.cpp
  src/Arm_frames.cpp
  src/Arm_segment.cpp
  src/Base_frames.cpp
  src/flipper.cpp
  src/flipper_frames.cpp
  src/libtaurob_base.cpp
  src/SocketTCP.cpp
  src/SocketUDP.cpp
  src/SocketUDPBinary.cpp
)

ament_target_dependencies(${PROJECT_NAME}
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    include
)

target_link_libraries(libtaurob
  boost_thread
  boost_system
  boost_date_time
)

#ament_target_dependencies (libtaurob diagnostic_updater)

install(TARGETS libtaurob
  EXPORT export_libtaurob
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_libtaurob HAS_LIBRARY_TARGET)

ament_package()

