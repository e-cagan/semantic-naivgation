cmake_minimum_required(VERSION 3.8)
project(semantic_nav_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# This package ships no code - only launch, params and worlds. Without these
# install() rules colcon builds "successfully" but copies nothing into
# install/share/, so get_package_share_directory() finds an empty directory and
# every launch include fails with a confusing "file not found".
install(DIRECTORY
  launch
  params
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
