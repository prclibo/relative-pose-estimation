project(four-point)

cmake_minimum_required(VERSION 2.8)

find_package( OpenCV REQUIRED )
include_directories( include/ )

add_subdirectory(src)
add_subdirectory(test)
