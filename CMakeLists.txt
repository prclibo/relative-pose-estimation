project(relative-pose-algorithm)

cmake_minimum_required(VERSION 2.8)

add_subdirectory(four-point-numerical)
add_subdirectory(four-point-groebner)
add_subdirectory(five-point-nister)
add_subdirectory(five-point-groebner)
add_subdirectory(one-point)
add_subdirectory(test)


