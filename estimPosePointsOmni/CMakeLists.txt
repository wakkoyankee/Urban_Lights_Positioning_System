cmake_minimum_required(VERSION 2.8)
project( TX52 )
find_package(VISP REQUIRED)
include(${VISP_USE_FILE})
find_package( OpenCV REQUIRED )
include_directories(  ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(  ${Boost_INCLUDE_DIRS})
SET(SOURCES
        CPoint.cpp
        CModel.cpp
        COmni.cpp
        CPose.cpp
        CPoseOmni.cpp
        PointTracking.cpp
        )
add_executable( main main.cpp ${SOURCES})
target_link_libraries( main ${OpenCV_LIBS} )
target_link_libraries(main ${Boost_LIBRARIES})
