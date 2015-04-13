# Edge Based Tracker
project(EBT)
cmake_minimum_required(VERSION 2.8.7)

#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
#set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# Set the version number for the library
set (EBT_VERSION_MAJOR 1)
set (EBT_VERSION_MINOR 0)
set (EBT_VERSION_PATCH 0)
math (EXPR EBT_VERSION_NUMERIC
    "10000 * ${EBT_VERSION_MAJOR} + 100 * ${EBT_VERSION_MINOR} + ${EBT_VERSION_PATCH}")
set (EBT_VERSION_STRING
    "${EBT_VERSION_MAJOR}.${EBT_VERSION_MINOR}.${EBT_VERSION_PATCH}")

###############################################################################
# Gather information, perform checks, set defaults

# Set the default install path to home
#set (CMAKE_INSTALL_PREFIX ${HOME} CACHE PATH "Install prefix for library")

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(EbtMakeConfigFile)

# Record the root dir for ebt - needed during external builds, e.g., ROS
set(EBT_SOURCE_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "EBT_SOURCE_ROOT_DIR: [${EBT_SOURCE_ROOT_DIR}]")

# Load build type flags and default to Debug mode
#include(EbtBuildTypes)

# Use macros for creating tests/timing scripts
#include(EbtTesting)
#include(EbtPrinting)

# guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR
    "In-source builds not allowed.
        Please make a new directory (named build) and run CMake from there.
        You may need to remove CMakeCache.txt. ")
endif()

###############################################################################
# Set up options


###############################################################################
# Find Libraries
find_package( Boost    1.43 REQUIRED )
find_package( OpenGL   1.10 REQUIRED )
find_package( GLEW     1.10 REQUIRED )
find_package( GLUT    2.8.1 REQUIRED )
find_package( OpenCV    2.4 REQUIRED )
find_package( LAPACK    3.5 REQUIRED )
#find_package( BULLET        REQUIRED )


###############################################################################
# Add components

# Build Image
add_subdirectory(3rdparty/Image)
# Build Fitline
add_subdirectory(3rdparty/Fitline)
# Build Fdcm
add_subdirectory(3rdparty/Fdcm)


################################################################################
# Set up CPack
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "EBT")
set(CPACK_PACKAGE_VENDOR "Henrik Christensen, Georgia Institute of Technology")
set(CPACK_PACKAGE_CONTACT "Henrik Christensen, hic@cc.gatech.edu")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
#set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_VERSION_MAJOR ${EBT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${EBT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${EBT_VERSION_PATCH})
set(CPACK_PACKAGE_INSTALL_DIRECTORY
            "CMake ${CMake_VERSION_MAJOR}.${CMake_VERSION_MINOR}")
set(CPACK_SOURCE_IGNORE_FILES "/build*;/\\\\.;/makestats.sh$")
set(CPACK_SOURCE_PACKAGE_FILE_NAME
            "ebt-${EBT_VERSION_MAJOR}.${EBT_VERSION_MINOR}.${EBT_VERSION_PATCH}")

# Deb-package specific cpack
set(CPACK_DEBIAN_PACKAGE_NAME "libebt-dev")
set(CPACK_DEBIAN_PACKAGE_DEPENDS   "libboost-dev    (>= 1.43),
                                    libglew-dev     (>= 1.10),
                                    libglew-dev     (>= 1.10),
                                    freeglut3-dev   (>= 2.8.1),
                                    libopencv-dev   (>= 2.4),
                                    liblapack3      (>= 3.5)")


################################################################################
# Print configuration variables
message(STATUS "===============================================================")
message(STATUS "================  Configuration Options  ======================")
message(STATUS "Build flags                                                    ")
string(TOUPPER "${CMAKE_BUILD_TYPE}" cmake_build_type_toupper)
if(NOT MSVC AND NOT XCODE_VERSION)
    message(STATUS "  Build type                     : ${CMAKE_BUILD_TYPE}")
    message(STATUS "  C compilation flags            : ${CMAKE_C_FLAGS}"
                                "${CMAKE_C_FLAGS_${cmake_build_type_toupper}}")
    message(STATUS "  C++ compilation flags          : ${CMAKE_CXX_FLAGS}"
                                "${CMAKE_CXX_FLAGS_${cmake_build_type_toupper}}")
endif()

message(STATUS "Packaging flags                                                ")
message(STATUS "  CPack Source Generator         : ${CPACK_SOURCE_GENERATOR}")
message(STATUS "  CPack Generator                : ${CPACK_GENERATOR}")

message(STATUS "EBT flags                                                      ")

message(STATUS "===============================================================")
################################################################################

# Include CPack *after* all flags
#include(CPack)

set(FDCM_LIBS ${FDCM_LIBS}
        ${PROJECT_BINARY_DIR}/3rdparty/Fdcm/libFdcmLib.so)
set(FITLINE_LIBS ${FITLINE_LIBS}
        ${PROJECT_BINARY_DIR}/3rdparty/Fitline/libLineFitLib.so)
set(IMAGE_LIBS ${IMAGE_LIBS}
        ${PROJECT_BINARY_DIR}/3rdparty/Image/libImageLib.so)

include_directories( ${EBT_SOURCE_DIR}/3rdparty)
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${Boost_INCLUDE_DIR} )
include_directories( ${EBT_SOURCE_DIR}/include)
#include_directories( ${EBT_SOURCE_DIR})

set(ADDITIONAL_BOOST_LIBS
        boost_program_options boost_system boost_thread)

include_directories(include)
set(incs
                include/object_tracking_2D/Camera.h
                include/object_tracking_2D/EdgeTracker.h
                include/object_tracking_2D/epnp.h
                include/object_tracking_2D/glm.h
                include/object_tracking_2D/HomoTransform.h
                include/object_tracking_2D/tracker_base.h
                include/object_tracking_2D/tracker_irls.h
                include/object_tracking_2D/tracker_pf.h
                include/object_tracking_2D/tracker_pf_texture.h
                include/object_tracking_2D/tracker_pf_textureless.h
                include/object_tracking_2D/ObjectModel.h
                include/object_tracking_2D/PoseEstimationSURF.h
                include/object_tracking_2D/Timer.h
                include/object_tracking_2D/randn.h
                include/object_tracking_2D/ParticleFilter.h
                include/object_tracking_2D/imageReceiver.h
                include/object_tracking_2D/non_randomness.h)


set(srcs
                src/Camera.cpp
                src/EdgeTracker.cpp
                src/epnp.cpp
                src/glm.cpp
                src/HomoTransform.cpp
                src/objectmodel.cpp
                src/ParticleFilter.cpp
                src/PoseEstimationSURF.cpp)


set(EBT_src     src/main.cpp)

set(ETG_src     src/edge_templates_generator.cpp)

add_executable(EBTDemo ${incs} ${EBT_src} ${srcs})
add_executable(ETGDemo ${incs} ${ETG_src} ${srcs})

set(libs
                ${OPENGL_LIBRARY}
                ${ADDITIONAL_BOOST_LIBS}
                ${Boost_LIBRARIES}
                ${GLEW_LIBRARIES}
                ${GLUT_LIBRARIES}
                ${LAPACK_LIBRARIES}
                ${OpenCV_LIBS}
                ${FDCM_LIBS}
                ${FITLINE_LIBS}
                ${IMAGE_LIBS})


target_link_libraries(ETGDemo
                ${libs})

add_dependencies(ETGDemo
                ImageLib
                LineFitLib
                FdcmLib)

target_link_libraries(EBTDemo
                ${libs})

add_dependencies(EBTDemo
                ImageLib
                LineFitLib
                FdcmLib)

add_library(object_tracking_2d SHARED ${incs} ${srcs})

target_link_libraries(object_tracking_2d ${libs})
