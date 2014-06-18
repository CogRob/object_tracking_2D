# make sure we find all headers
include_directories(${EdgeBasedTracker_SOURCE_DIR}/)

# we create a library for line fitting
add_library(Toon fitline.cpp LFLineFitter.cpp LFLineSegment.cpp)

