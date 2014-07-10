# make sure we find all headers
cmake_minimum_required(VERSION 2.8)

# make sure we find all headers
include_directories(${EdgeBasedTracker_SOURCE_DIR}/)
include_directories(../)

# we create a library for line fitting
#add_library(Toon SHARED
#	fitline.cpp 
#	LFLineFitter.cpp 
#	LFLineSegment.cpp)