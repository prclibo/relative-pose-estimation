add_library( four-point 
    four-point.cpp four-point-helper.cpp 
    precomp.cpp modelest.cpp  )

target_link_libraries(four-point ${OpenCV_LIBS} gsl gslcblas m)

