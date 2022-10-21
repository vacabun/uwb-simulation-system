cmake_minimum_required(VERSION 3.5)
project(uwb_location_3d)

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

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uwb_interfaces REQUIRED)

find_package( PkgConfig REQUIRED)
pkg_check_modules( gsl REQUIRED gsl )

add_executable(uwb_locate
  src/main.cpp 
  src/uwb_locate.cpp
  src/tinyxml2/tinyxml2.cpp
  src/ransac_locator/ransac_locator.cpp
  )

ament_target_dependencies(uwb_locate rclcpp nav_msgs ament_index_cpp uwb_interfaces)
target_link_libraries( uwb_locate ${gsl_LIBRARIES} )

install(TARGETS
  uwb_locate
  DESTINATION lib/${PROJECT_NAME})

ament_package()
