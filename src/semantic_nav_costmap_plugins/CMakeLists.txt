cmake_minimum_required(VERSION 3.8)
project(semantic_nav_costmap_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(semantic_nav_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  nav2_costmap_2d
  nav2_core
  pluginlib
  semantic_nav_interfaces
  tf2_geometry_msgs
  tf2_ros
  geometry_msgs
)

# SHARED, not an executable: pluginlib dlopen()s this library into the running Nav2
# process. The target name must match the <library path="..."> in the plugin XML.
add_library(semantic_layer SHARED
  src/semantic_layer.cpp
)

target_include_directories(semantic_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(semantic_layer ${dependencies})

# Tells pluginlib where to find the class description at runtime. Forgetting this is
# the classic failure: the library builds and installs fine, and Nav2 still reports
# that the plugin "does not exist".
pluginlib_export_plugin_description_file(nav2_costmap_2d semantic_layer_plugin.xml)

install(TARGETS semantic_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES semantic_layer_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(semantic_layer)
ament_export_dependencies(${dependencies})

ament_package()