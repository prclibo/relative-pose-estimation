project(four-point)

cmake_minimum_required(VERSION 2.8)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

find_package( OpenCV REQUIRED )
find_package( Cholmod REQUIRED )
find_package( SPQR REQUIRED )

include_directories( include/ )
include_directories( /usr/local/include/eigen3/ )
include_directories( /usr/include/suitesparse/ )

add_subdirectory(src)
add_subdirectory(test)


