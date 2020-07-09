
cmake_minimum_required(VERSION 3.5)
project(tiscamera_ros)

# enable C++17 standard (current directory scope)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# configure build types and build c-flags
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")


set(WITH_TONEMAPPING_FLAG OFF)

if(WITH_TONEMAPPING_FLAG)
  message(WARNING "Build with Tonemapping")
  add_definitions( -DTONEMAPPING)
  set(WITH_TONEMAPPING cvl)
else()
  message(WARNING "Build WITHOUT Tonemapping")
endif()


set(CMAKE_CXX_FLAGS "-Wall -Wextra")
# set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ")

add_subdirectory(tiscamera_interface)
add_subdirectory(tests)

find_package(Threads REQUIRED)

find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  imu_vn_100
  mavros
)

# running dependency
catkin_package(
  # INCLUDE_DIRS 
  # LIBRARIES 
  CATKIN_DEPENDS imu_vn_100
  # DEPENDS
)

# message ("catkin_INCLUDE_DIRS = ${catkin_INCLUDE_DIRS}")
# message ("catkin_LIBRARIES = ${catkin_LIBRARIES}")

add_executable(tiscamera_ros main.cpp ros_publisher.cpp utils.cpp)
target_include_directories(tiscamera_ros
  PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(tiscamera_ros
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  tiscamera_interface::tiscamera_interface
  Threads::Threads
  ${WITH_TONEMAPPING}
)
