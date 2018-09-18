find_package( OpenCV 2 REQUIRED )

add_library( one-point
    one-point.cpp precomp.cpp modelest.cpp )

target_link_libraries(one-point
    ${OpenCV_LIBS} )
