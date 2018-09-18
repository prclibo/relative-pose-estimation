find_package( OpenCV 2 REQUIRED )

add_library( five-point-nister
    five-point.cpp precomp.cpp modelest.cpp )

target_link_libraries(five-point-nister 
    ${OpenCV_LIBS} )
