# make sure we find all headers
cmake_minimum_required(VERSION 2.8)

# make sure we find all headers
include_directories(${EdgeBasedTracker_SOURCE_DIR}/)
include_directories(../)

find_package( OpenCV REQUIRED )

# we create a library for line fitting
add_library(LineFitLib SHARED
	fitline.cpp 
	LFLineFitter.cpp 
	LFLineSegment.cpp)

#add_executable(Fitline 
#	fitline.cpp)

#target_link_libraries(Fitline 
#	${OpenCV_LIBS} 
#	${EdgeBasedTracker_SOURCE_DIR}/Fitline/libLineFitLib.a 
#	${OpenCV_LIBS} ${EdgeBasedTracker_SOURCE_DIR}/Image/libImageLib.a)
