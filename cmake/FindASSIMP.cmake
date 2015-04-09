#
# Find the Assimp includes and library
#
# This module defines
# ASSIMP_INCLUDE_DIR, where to find tiff.h, etc.
# ASSIMP_LIBRARIES, the libraries to link against to use ASSIMP.
# ASSIMP_FOUND, If false, do not try to use ASSIMP.

# also defined, but not for general use are
# ASSIMP_LIBRARY, where to find the ASSIMP library.
# ASSIMP_DEBUG_LIBRARY, where to find the ASSIMP library in debug
# mode.

find_package(PkgConfig REQUIRED)
#find_package(ASSIMP QUIET)
if (NOT ASSIMP_FOUND)
 pkg_check_modules(ASSIMP assimp)
endif()
if (ASSIMP_FOUND)
  if( ${ASSIMP_VERSION} STRGREATER "2.0.0" )
	set(IS_ASSIMP3 1)
	add_definitions(-DIS_ASSIMP3)
	message(STATUS "Found assimp v3")
  else()
	message(STATUS "Found assimp v2")
  endif()
  include_directories(${ASSIMP_INCLUDE_DIRS})
  link_directories(${ASSIMP_LIBRARY_DIRS})
else()
  message(STATUS "could not find assimp (perhaps available thorugh ROS package?), so assimping assimp v2")
  set(ASSIMP_LIBRARIES assimp)
  set(ASSIMP_LIBRARY_DIRS)
  set(ASSIMP_CXX_FLAGS)
  set(ASSIMP_CFLAGS_OTHER)
  set(ASSIMP_LINK_FLAGS)
  set(ASSIMP_INCLUDE_DIRS)
  set(IS_ASSIMP3 0) # most likely not
endif()
