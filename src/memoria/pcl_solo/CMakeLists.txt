cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_viewer)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (build/cloud_viewer  src/cloud_viewer.cpp)
target_link_libraries (build/cloud_viewer ${PCL_LIBRARIES})

add_executable (build/viewer  src/viewer.cpp)
target_link_libraries (build/viewer ${PCL_LIBRARIES})

#add_executablebuild/ (simple_objects  src/simple_objects.cpp)
#target_link_libraries (build/simple_objects ${PCL_LIBRARIES})

add_executable (build/segmentarplano  src/segmentarplano.cpp)
target_link_libraries (build/segmentarplano ${PCL_LIBRARIES})

add_executable (build/convexhull  src/convexhull.cpp)
target_link_libraries (build/convexhull ${PCL_LIBRARIES})

add_executable (build/concavehull  src/concavehull.cpp)
target_link_libraries (build/concavehull ${PCL_LIBRARIES})

add_executable (build/stablesurface src/stablesurface.cpp)
target_link_libraries (build/stablesurface ${PCL_LIBRARIES})