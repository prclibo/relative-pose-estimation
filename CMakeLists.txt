project(relative-pose-algorithm)

cmake_minimum_required(VERSION 2.8)

add_subdirectory(four-point-numerical)
add_subdirectory(four-point-groebner)
add_subdirectory(five-point-nister)
add_subdirectory(one-point)

add_executable(demo demo.cpp)
target_link_libraries(demo five-point-nister four-point-numerical four-point-groebner ${OpenCV_LIBS})

