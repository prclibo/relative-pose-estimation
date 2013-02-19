add_library( four-point-numerical 
    four-point-numerical.cpp 
    precomp.cpp modelest.cpp  )

add_library( five-point 
    five-point.cpp 
    precomp.cpp 
    modelest.cpp )

add_library( one-point
    one-point.cpp
    precomp.cpp
    modelest.cpp )

add_library( four-point-groebner
    four-point-groebner.cpp )

target_link_libraries(four-point-numerical ${OpenCV_LIBS} gsl gslcblas m)

target_link_libraries(four-point-groebner 
    ${OpenCV_LIBS} 
    ${CHOLMOD_LIBRARIES} 
    ${AMD_LIBRARY} 
    ${COLAMD_LIBRARY} 
    ${SPQR_LIBRARIES}
    blas
    lapack)
