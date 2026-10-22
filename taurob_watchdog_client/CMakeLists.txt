cmake_minimum_required(VERSION 3.16)
project(taurob_watchdog_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_executable(taurob_watchdog_client
  src/taurob_watchdog_client.cpp
)

target_link_libraries(taurob_watchdog_client
  pthread
)

install(TARGETS taurob_watchdog_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
