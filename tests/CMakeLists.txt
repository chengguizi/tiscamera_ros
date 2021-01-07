cmake_minimum_required(VERSION 3.5)
project(tiscamera_test)

find_package(OpenCV 4 REQUIRED)

add_executable(snapimage snapimage.cpp)
target_include_directories(snapimage PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(snapimage tiscamera_interface::tiscamera_interface ${OpenCV_LIBS})


add_executable(gst-format-test gst-format-test.cpp)
target_link_libraries(gst-format-test tiscamera_interface::tiscamera_interface)