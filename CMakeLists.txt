cmake_minimum_required(VERSION 2.8)

find_package( OpenCV REQUIRED )
add_executable( perf_test_sideway 
    perf_test_sideway.cpp four-point.cpp four-point-helper.cpp 
    precomp.cpp modelest.cpp 
    five-point/five-point.cpp) 

add_executable( perf_test_sideway2 
    perf_test_sideway2.cpp four-point.cpp four-point-helper.cpp 
    precomp.cpp modelest.cpp 
    five-point/five-point.cpp) 

add_executable( perf_test_forward
    perf_test_forward.cpp four-point.cpp four-point-helper.cpp 
    precomp.cpp modelest.cpp 
    five-point/five-point.cpp) 

add_executable( perf_test_forward2
    perf_test_forward2.cpp four-point.cpp four-point-helper.cpp 
    precomp.cpp modelest.cpp 
    five-point/five-point.cpp) 

add_executable( perf_test_odonoise
    perf_test_odonoise.cpp four-point.cpp four-point-helper.cpp 
    precomp.cpp modelest.cpp 
    five-point/five-point.cpp) 

add_executable( ransac_test ransac_test.cpp four-point.cpp four-point-helper.cpp precomp.cpp modelest.cpp)


target_link_libraries( perf_test_sideway ${OpenCV_LIBS} gsl gslcblas m )
target_link_libraries( perf_test_sideway2 ${OpenCV_LIBS} gsl gslcblas m )
target_link_libraries( perf_test_forward ${OpenCV_LIBS} gsl gslcblas m )
target_link_libraries( perf_test_forward2 ${OpenCV_LIBS} gsl gslcblas m )
target_link_libraries( perf_test_odonoise ${OpenCV_LIBS} gsl gslcblas m )
target_link_libraries( ransac_test ${OpenCV_LIBS} gsl gslcblas m )
