
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/four-point-groebner/cmake)

find_package( OpenCV REQUIRED )
find_package( Cholmod REQUIRED )
find_package( SPQR REQUIRED )

include_directories( ../eigen/ )
include_directories( /usr/include/suitesparse/ )

add_library( four-point-groebner
    four-point-groebner.cpp modelest.cpp precomp.cpp )

target_link_libraries(four-point-groebner 
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
