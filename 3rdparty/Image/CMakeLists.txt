# make sure we find all headers
#include_directories(${EdgeBasedTracker_SOURCE_DIR}/)

#find_package(OpenCV)

# we create a library for line fitting
add_library(ImageLib 
	DistanceTransform.cpp)
