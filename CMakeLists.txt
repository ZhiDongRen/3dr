cmake_minimum_required(VERSION 3.16)
project(3dr)

set(CMAKE_CXX_STANDARD 14)
find_package(PCL 1.11 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost COMPONENTS thread REQUIRED)



add_executable(3dr main.cpp)
target_link_libraries (3dr ${PCL_LIBRARIES})
target_link_libraries(3dr ${Boost_LIBRARIES})