cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_viewer)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

### TUTORIALES (fuera de uso) ###

# add_executable (build/cloud_viewer  src/cloud_viewer.cpp)
# target_link_libraries (build/cloud_viewer ${PCL_LIBRARIES})

# add_executable (build/segmentarplano  src/segmentarplano.cpp)
# target_link_libraries (build/segmentarplano ${PCL_LIBRARIES})

# add_executable (build/convexhull  src/convexhull.cpp)
# target_link_libraries (build/convexhull ${PCL_LIBRARIES})

# add_executable (build/concavehull  src/concavehull.cpp)
# target_link_libraries (build/concavehull ${PCL_LIBRARIES})

# add_executable (build/regiongrowing src/regiongrowing.cpp)
# target_link_libraries (build/regiongrowing ${PCL_LIBRARIES})

# add_executable (build/convexhullregiongrowing src/convexhullregiongrowing.cpp)
# target_link_libraries (build/convexhullregiongrowing ${PCL_LIBRARIES})

### EJECUTABLES EN USO ###

add_executable (build/viewer  src/viewer.cpp)
target_link_libraries (build/viewer ${PCL_LIBRARIES})

add_executable (build/viewerrgb  src/viewerrgb.cpp)
target_link_libraries (build/viewerrgb ${PCL_LIBRARIES})

add_executable (build/stablesurface src/stablesurface.cpp)
target_link_libraries (build/stablesurface ${PCL_LIBRARIES})

include_directories(${CMAKE_SOURCE_DIR}/src/Util)
set(SOURCES ${CMAKE_SOURCE_DIR}/src/Util/Util.cpp 
			${CMAKE_SOURCE_DIR}/src/Util/Viewer.cpp 
			${CMAKE_SOURCE_DIR}/src/Util/Polymesh.cpp
			${CMAKE_SOURCE_DIR}/src/Util/Util.h 
			${CMAKE_SOURCE_DIR}/src/Util/Viewer.h 
			${CMAKE_SOURCE_DIR}/src/Util/Polymesh.h)
add_executable(build/stablesurface2 src/stablesurface2.cpp ${SOURCES})
target_link_libraries (build/stablesurface2 ${PCL_LIBRARIES})

add_executable(build/gripperfilter src/gripperfilter.cpp ${SOURCES})
target_link_libraries (build/gripperfilter ${PCL_LIBRARIES})

add_executable(build/stablesurface3 src/stablesurface3.cpp ${SOURCES})
target_link_libraries (build/stablesurface3 ${PCL_LIBRARIES})