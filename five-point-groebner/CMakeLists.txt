find_package( OpenCV REQUIRED )

include_directories( ../eigen/ )

add_library( five-point-groebner
    five-point.cpp modelest.cpp precomp.cpp )

target_link_libraries(five-point-groebner 
    ${OpenCV_LIBS} )
