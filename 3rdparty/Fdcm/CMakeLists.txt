# make sure we find all headers
cmake_minimum_required(VERSION 2.8)

# make sure we find all headers
include_directories(${EdgeBasedTracker_SOURCE_DIR}/)
include_directories(../)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# we create a library for line fitting
add_library(FdcmLib SHARED
	EIEdgeImage.cpp 
	LMDirectionalIntegralDistanceImage.cpp 
	LMDisplay.cpp 
	LMDistanceImage.cpp 
	LMLineMatcher.cpp 
	LMNonMaximumSuppression.cpp 
	MatchingCostMap.cpp)

#add_executable(Fdcm 
#	EIEdgeImage.cpp 
#	fdcm.cpp 
#	LMDirectionalIntegralDistanceImage.cpp 
#	LMDisplay.cpp 
#	LMDistanceImage.cpp 
#	LMLineMatcher.cpp 
#	LMNonMaximumSuppression.cpp 
#	MatchingCostMap.cpp)

#target_link_libraries(Fdcm 
#	${OpenCV_LIBS} 
#	${EdgeBasedTracker_SOURCE_DIR}/Fdcm/libFdcmLib.a 
#	${EdgeBasedTracker_SOURCE_DIR}/Fitline/libLineFitLib.a 
#	${OpenCV_LIBS} 
#	${EdgeBasedTracker_SOURCE_DIR}/Image/libImageLib.a) 

