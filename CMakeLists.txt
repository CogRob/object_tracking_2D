# top level script
cmake_minimum_required(VERSION 2.8.10)
project(EdgeBasedTracker)

# Recurse into the "Hello" and "Demo" subdirectories. This does not actually
# cause another cmake executable to run. The same process will walk through
# the project's entire directory structure.

find_package( GLUT          REQUIRED )
find_package( OpenGL        REQUIRED )
find_package( OpenCV 2.4    REQUIRED )
find_package( LAPACK        REQUIRED )
find_package( GLEW          REQUIRED )
find_package( Boost         REQUIRED )
#find_package( BULLET        REQUIRED )

add_subdirectory(3rdparty/Image)
add_subdirectory(3rdparty/Fitline)
add_subdirectory(3rdparty/Fdcm)

set(FDCM_LIBS ${FDCM_LIBS}
	${PROJECT_BINARY_DIR}/3rdparty/Fdcm/libFdcmLib.a)
set(FITLINE_LIBS ${FITLINE_LIBS}
	${PROJECT_BINARY_DIR}/3rdparty/Fitline/libLineFitLib.a)
set(IMAGE_LIBS ${IMAGE_LIBS}
	${PROJECT_BINARY_DIR}/3rdparty/Image/libImageLib.a)

# MESSAGE(  ${EdgeBasedTracker_SOURCE_DIR})

include_directories("${EdgeBasedTracker_SOURCE_DIR}/3rdparty")
include_directories( ${OpenCV_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
include_directories( ${EdgeBasedTracker_SOURCE_DIR}/include)
#include_directories( ${EdgeBasedTracker_SOURCE_DIR})

SET(ADDITIONAL_BOOST_LIBS 
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

set(EBT_src
                src/main.cpp
                src/Camera.cpp
                src/EdgeTracker.cpp
                src/epnp.cpp
                src/glm.cpp
                src/HomoTransform.cpp
                src/objectmodel.cpp
                src/ParticleFilter.cpp
                src/PoseEstimationSURF.cpp)

set(ETG_src
                src/edge_templates_generator.cpp
                src/Camera.cpp
                src/EdgeTracker.cpp
                src/epnp.cpp
                src/glm.cpp
                src/HomoTransform.cpp
                src/objectmodel.cpp
                src/ParticleFilter.cpp
                src/PoseEstimationSURF.cpp)

add_executable(EBT ${incs} ${EBT_src})
add_executable(ETG ${incs} ${ETG_src})

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

target_link_libraries(ETG ${libs})

add_dependencies(ETG ImageLib)
add_dependencies(ETG LineFitLib)
add_dependencies(ETG FdcmLib)

target_link_libraries(EBT ${libs})

add_dependencies(EBT ImageLib)
add_dependencies(EBT LineFitLib)
add_dependencies(EBT FdcmLib)
