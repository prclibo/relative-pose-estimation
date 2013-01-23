cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /home/bo/workspace/four-point/)

find_package( OpenCV REQUIRED )
find_package(GSL REQUIRED)
message(STATUS ${GSL_LIBRARIES})
add_executable( four-point four-point.cpp four-point-helper.cpp )
target_link_libraries( four-point ${OpenCV_LIBS} gsl gslcblas m gsl gslcblas m  )
# target_link_libraries( four-point ${OpenCV_LIBS})
