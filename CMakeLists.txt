cmake_minimum_required(VERSION 3.15)
project(studyslam)

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "DeBug" )
set( CMAKE_CXX_FLAGS "-std=c++11" )

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

############### dependencies ######################

#OpenCV
find_package( OpenCV REQUIRED ) 
include_directories(${OpenCV_INCLUDE_DIRS})


set( THIRD_PARTY_LIBS  ${OpenCV_LIBS})

############### dependencies ######################
include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src )
add_subdirectory( test )