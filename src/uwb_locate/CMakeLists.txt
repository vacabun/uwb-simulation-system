cmake_minimum_required(VERSION 3.5)
project(uwb_locate)

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

find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uwb_interfaces REQUIRED)

find_package( PkgConfig REQUIRED)
pkg_check_modules( gsl REQUIRED gsl )

add_executable(uwb_location
  src/uwb_location_main.cpp 
  src/uwb_location.cpp
  src/tinyxml2/tinyxml2.cpp
  src/ransac_locator/ransac_locator.cpp
  )
  
target_include_directories(uwb_location PUBLIC include)

target_link_libraries(uwb_location ${gsl_LIBRARIES} )

ament_target_dependencies(uwb_location rclcpp nav_msgs ament_index_cpp uwb_interfaces)


install(TARGETS
  uwb_location
  DESTINATION lib/${PROJECT_NAME})

ament_package()
