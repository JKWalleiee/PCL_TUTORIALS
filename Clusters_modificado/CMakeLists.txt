cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cluster_extraction_modificado)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cluster_extraction_modificado cluster_extraction.cpp)
target_link_libraries (cluster_extraction_modificado ${PCL_LIBRARIES})