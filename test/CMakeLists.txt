include_directories(/usr/include/eigen3/)

add_executable( perf_test_sideway perf_test_sideway.cpp ) 

add_executable( perf_test_sideway2 perf_test_sideway2.cpp ) 

add_executable( perf_test_forward perf_test_forward.cpp ) 

add_executable( perf_test_forward2 perf_test_forward2.cpp ) 

add_executable( perf_test_odonoise perf_test_odonoise.cpp ) 

add_executable( perf_test_odonoise2 perf_test_odonoise2.cpp ) 

add_executable( perf_test_vcharge 
                perf_test_vcharge.cpp 
                sparse_graph/SparseGraph.cc sparse_graph/Odometer.cc 
                sparse_graph/tinyxml2/tinyxml2.cpp ) 

add_executable( test_1pt
                test_1pt.cpp) 

target_link_libraries( perf_test_sideway five-point four-point ${OpenCV_LIBS} gsl gslcblas m )
target_link_libraries( perf_test_sideway2 five-point four-point ${OpenCV_LIBS} gsl gslcblas m )
target_link_libraries( perf_test_forward five-point four-point ${OpenCV_LIBS} gsl gslcblas m )
target_link_libraries( perf_test_forward2 five-point four-point ${OpenCV_LIBS} gsl gslcblas m )
target_link_libraries( perf_test_odonoise five-point four-point ${OpenCV_LIBS} gsl gslcblas m )
target_link_libraries( perf_test_odonoise2 five-point four-point ${OpenCV_LIBS} gsl gslcblas m )
target_link_libraries( perf_test_vcharge five-point four-point ${OpenCV_LIBS} gsl gslcblas m boost_filesystem boost_system )

target_link_libraries( test_1pt one-point ${OpenCV_LIBS})
