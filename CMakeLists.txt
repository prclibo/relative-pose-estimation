cmake_minimum_required(VERSION 2.8)

find_package( OpenCV REQUIRED )
add_executable( perf_test perf_test.cpp four-point.cpp four-point-helper.cpp precomp.cpp modelest.cpp)
add_executable( ransac_test ransac_test.cpp four-point.cpp four-point-helper.cpp precomp.cpp modelest.cpp)
target_link_libraries( perf_test ${OpenCV_LIBS} gsl gslcblas m )
target_link_libraries( ransac_test ${OpenCV_LIBS} gsl gslcblas m )
