cmake_minimum_required(VERSION 3.5)
project(tiscamera_test)

find_package(OpenCV 3 REQUIRED)

add_executable(simple-snapimage snapimage.cpp tcamimage.cpp)
target_include_directories(simple-snapimage PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(simple-snapimage tiscamera_interface::tiscamera_interface ${OpenCV_LIBS})