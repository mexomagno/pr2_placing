cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_viewer)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cloud_viewer  src/cloud_viewer.cpp)
target_link_libraries (cloud_viewer ${PCL_LIBRARIES})

add_executable (viewer  src/viewer.cpp)
target_link_libraries (viewer ${PCL_LIBRARIES})

#add_executable (simple_objects  src/simple_objects.cpp)
#target_link_libraries (simple_objects ${PCL_LIBRARIES})

add_executable (segmentarplano  src/segmentarplano.cpp)
target_link_libraries (segmentarplano ${PCL_LIBRARIES})

add_executable (convexhull  src/convexhull.cpp)
target_link_libraries (convexhull ${PCL_LIBRARIES})

add_executable (concavehull  src/concavehull.cpp)
target_link_libraries (concavehull ${PCL_LIBRARIES})