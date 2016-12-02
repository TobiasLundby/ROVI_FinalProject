cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(ROVI_FinalProject)

find_package(OpenCV REQUIRED)

add_executable(image1 image1.cpp)

target_link_libraries(image1 ${OpenCV_LIBS})
