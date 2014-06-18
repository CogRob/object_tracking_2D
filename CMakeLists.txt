# top level script
cmake_minimum_required (VERSION 2.8.10)
project (EdgeBasedTracker)

# Recurse into the "Hello" and "Demo" subdirectories. This does not actually
# cause another cmake executable to run. The same process will walk through
# the project's entire directory structure.
add_subdirectory (3rdparty/Image)
add_subdirectory (3rdparty/Fitline) 
add_subdirectory (3rdparty/Fdcm)

set(BULLET_ROOT ${ROOT_DIR}/Downloads/bullet-2.82-r2704) # where to find Bullet

find_package( GLUT )
find_package( OpenGL REQUIRED )
find_package( OpenCV 2.4 REQUIRED )
find_package( LAPACK REQUIRED )
find_package( GLEW REQUIRED)
find_package(Boost REQUIRED)
#FIND_PACKAGE(BULLET REQUIRED)

#FIND_PACKAGE(Eigen REQUIRED)
#include_directories(EIGEN3_INCLUDE_DIR)


MESSAGE(  ${EdgeBasedTracker_SOURCE_DIR})

include_directories(../include)
include_directories("${CMAKE_SOURCE_DIR}/3rdparty")
include_directories( ${OpenCV_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
include_directories( ${EdgeBasedTracker_SOURCE_DIR}/include)
include_directories( ${EdgeBasedTracker_SOURCE_DIR})

include_directories("/usr/lib/")

SET(ADDITIONAL_BOOST_LIBS boost_program_options boost_system boost_thread)

add_executable(EBT src/Camera.cpp src/EdgeTracker.cpp src/epnp.cpp src/glm.cpp src/HomoTransform.cpp src/main.cpp src/objectmodel.cpp src/ParticleFilter.cpp src/PoseEstimationSURF.cpp)

add_executable(edge_templates_generator src/Camera.cpp src/EdgeTracker.cpp src/epnp.cpp src/glm.cpp src/HomoTransform.cpp  src/objectmodel.cpp src/ParticleFilter.cpp src/PoseEstimationSURF.cpp src/edge_templates_generator.cpp)

#target_link_libraries(edge_templates_generator ${OPENGL_LIBRARY} ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OpenCV_LIBS} ${ADDITIONAL_BOOST_LIBS} ${Boost_LIBRARIES} ${EdgeBasedTracker_SOURCE_DIR}/3rdparty/Fitline/libLineFitLib.a  ${LAPACK_LIBRARIES} )

target_link_libraries(edge_templates_generator ${OPENGL_LIBRARY} ${ADDITIONAL_BOOST_LIBS} ${Boost_LIBRARIES}  ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${LAPACK_LIBRARIES} ${OpenCV_LIBS} ${EdgeBasedTracker_SOURCE_DIR}/3rdparty/Fdcm/libFdcmLib.a ${EdgeBasedTracker_SOURCE_DIR}/3rdparty/Fitline/libLineFitLib.a ${OpenCV_LIBS} ${EdgeBasedTracker_SOURCE_DIR}/3rdparty/Image/libImageLib.a)

add_dependencies(edge_templates_generator ImageLib)
add_dependencies(edge_templates_generator LineFitLib)
add_dependencies(edge_templates_generator FdcmLib)



target_link_libraries(EBT ${OPENGL_LIBRARY} ${ADDITIONAL_BOOST_LIBS} ${Boost_LIBRARIES}  ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${LAPACK_LIBRARIES} ${OpenCV_LIBS} ${EdgeBasedTracker_SOURCE_DIR}/3rdparty/Fdcm/libFdcmLib.a ${EdgeBasedTracker_SOURCE_DIR}/3rdparty/Fitline/libLineFitLib.a ${OpenCV_LIBS} ${EdgeBasedTracker_SOURCE_DIR}/3rdparty/Image/libImageLib.a)
#target_link_libraries(Fdcmr ${OPENGL_LIBRARY} ${ADDITIONAL_BOOST_LIBS} ${Boost_LIBRARIES}  ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${LAPACK_LIBRARIES} ${OpenCV_LIBS} ${EdgeBasedTracker_SOURCE_DIR}/3rdparty/Fdcm/libFdcmLib.a ${EdgeBasedTracker_SOURCE_DIR}/3rdparty/Fitline/libLineFitLib.a ${OpenCV_LIBS} ${EdgeBasedTracker_SOURCE_DIR}/3rdparty/Image/libImageLib.a)

add_dependencies(EBT ImageLib)
add_dependencies(EBT LineFitLib)
add_dependencies(EBT FdcmLib)



