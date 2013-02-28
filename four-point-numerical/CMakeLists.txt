find_package( OpenCV REQUIRED )

add_library( four-point-numerical 
    four-point-numerical.cpp precomp.cpp modelest.cpp  )

target_link_libraries(four-point-numerical ${OpenCV_LIBS} gsl gslcblas m)
