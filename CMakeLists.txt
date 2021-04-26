cmake_minimum_required(VERSION 3.16)
project(3dr)

set(CMAKE_CXX_STANDARD 14)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost COMPONENTS thread REQUIRED)
#SET(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
#SET(BUILD_SHARED_LIBS OFF)
#SET(CMAKE_EXE_LINKER_FLAGS "-static")


add_executable(gp3 gp3.cpp)
add_executable(Bspline Bspline.cpp)
add_executable(convex convex.cpp)

target_link_libraries(convex ${PCL_LIBRARIES})
target_link_libraries(convex ${Boost_LIBRARIES})

target_link_libraries (gp3 ${PCL_LIBRARIES})
target_link_libraries(gp3 ${Boost_LIBRARIES})
target_link_libraries (Bspline ${PCL_LIBRARIES})
target_link_libraries(Bspline ${Boost_LIBRARIES})
