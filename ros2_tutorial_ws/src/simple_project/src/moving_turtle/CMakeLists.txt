cmake_minimum_required(VERSION 3.8)
project(moving_turtle)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
# TODO: find other dependencies

include_directories(
  include
)

# TODO: Add executable moving_turtle.cpp
add_executable(moving_turtle src/moving_turtle.cpp)

# Target linked libraries

## TODO: Add dependencies to the executable
ament_target_dependencies(moving_turtle
  rclcpp
  geometry_msgs
)

## TODO: Install the executable
install(TARGETS
  moving_turtle
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
