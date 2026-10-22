cmake_minimum_required(VERSION 3.16)
project(taurob_watchdog)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(libtaurob REQUIRED)
find_package(Boost REQUIRED)

add_executable(taurob_watchdog_node
  src/taurob_watchdog_node.cpp
)

ament_target_dependencies(taurob_watchdog_node
  rclcpp
  std_msgs
  libtaurob
)

target_link_libraries(taurob_watchdog_node
  libtaurob
  boost_system
)

install(TARGETS taurob_watchdog_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
