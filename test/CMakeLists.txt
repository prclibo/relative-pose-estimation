include_directories(/usr/include/eigen3/)

add_executable( perf_test_sideway perf_test_sideway.cpp ) 

add_executable( perf_test_sideway2 perf_test_sideway2.cpp ) 

add_executable( perf_test_forward perf_test_forward.cpp ) 

add_executable( perf_test_forward2 perf_test_forward2.cpp ) 


add_executable( perf_test_odonoise2 perf_test_odonoise2.cpp ) 

add_executable( perf_test_vcharge 
                perf_test_vcharge.cpp 
                sparse_graph/SparseGraph.cc sparse_graph/Odometer.cc 
                sparse_graph/tinyxml2/tinyxml2.cpp ) 

add_executable( perf_test_timing perf_test_timing.cpp ) 

add_executable( test_1pt
                test_1pt.cpp) 

add_executable( test_4pt_groebner
                test_4pt_groebner.cpp )

target_link_libraries( perf_test_sideway five-point four-point-numerical ${OpenCV_LIBS} gsl gslcblas m )
target_link_libraries( perf_test_sideway2 five-point four-point-numerical ${OpenCV_LIBS} gsl gslcblas m )
target_link_libraries( perf_test_forward five-point four-point-numerical ${OpenCV_LIBS} gsl gslcblas m )
target_link_libraries( perf_test_forward2 five-point four-point-numerical ${OpenCV_LIBS} gsl gslcblas m )
target_link_libraries( perf_test_odonoise2 five-point four-point-numerical ${OpenCV_LIBS} gsl gslcblas m )
target_link_libraries( perf_test_vcharge five-point four-point-numerical one-point ${OpenCV_LIBS} gsl gslcblas m boost_filesystem boost_system )
target_link_libraries( perf_test_timing five-point four-point-numerical four-point-groebner
    ${OpenCV_LIBS} 
    ${CHOLMOD_LIBRARIES} 
    ${AMD_LIBRARY} 
    ${COLAMD_LIBRARY} 
    ${SPQR_LIBRARIES}
    lapack
    csparse
    cxsparse  
    cholmod
    blas
    spqr)
 
target_link_libraries( test_1pt one-point ${OpenCV_LIBS})
target_link_libraries( test_4pt_groebner four-point-groebner ${OpenCV_LIBS})
