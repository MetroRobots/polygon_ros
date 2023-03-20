cmake_minimum_required(VERSION 3.5)
project(polygon_rviz_plugins)

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
find_package(ament_cmake_ros REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Test)
find_package(color_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(polygon_msgs REQUIRED)
find_package(polygon_utils REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)

qt5_wrap_cpp(MOC_FILES
  include/polygon_rviz_plugins/polygon_display.hpp
  include/polygon_rviz_plugins/polygon3d_display.hpp
  include/polygon_rviz_plugins/complex_polygon_display.hpp
  include/polygon_rviz_plugins/polygons_display.hpp
  include/polygon_rviz_plugins/complex_polygons_display.hpp
)

add_library(polygon_rviz_plugins SHARED
  src/polygon_display.cpp
  src/polygon_parts.cpp
  src/polygon3d_display.cpp
  src/complex_polygon_display.cpp
  src/polygons_display.cpp
  src/complex_polygons_display.cpp
  ${MOC_FILES}
)
target_include_directories(polygon_rviz_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(polygon_rviz_plugins
  pluginlib
  polygon_msgs
  polygon_utils
  rviz_common
  geometry_msgs
  std_msgs
  color_util
)

install(DIRECTORY include/
        DESTINATION include
)
install(DIRECTORY icons/classes/
        DESTINATION share/${PROJECT_NAME}/icons/classes
)
install(TARGETS polygon_rviz_plugins
        EXPORT export_polygon_rviz_plugins
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
ament_export_include_directories(include)
ament_export_targets(export_polygon_rviz_plugins)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

ament_package()
