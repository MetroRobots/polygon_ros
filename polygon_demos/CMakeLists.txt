cmake_minimum_required(VERSION 3.5)
project(polygon_demos)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(color_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(polygon_msgs REQUIRED)
find_package(polygon_utils REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(demo src/polygon_demo.cpp)
target_include_directories(demo PUBLIC
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(demo
  angles
  geometry_msgs
  polygon_msgs
  polygon_utils
  rclcpp
  color_util
)
install(TARGETS demo
        DESTINATION lib/${PROJECT_NAME}
)
install(FILES config/demo_polygons.rviz
        DESTINATION share/${PROJECT_NAME}/config
)
install(FILES launch/polygons.launch.py
        DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
