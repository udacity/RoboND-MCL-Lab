# set minimum cmake version
cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

# project name and language
project(robotMcl)

#set sources and headers
set(src robotMcl.cpp)

#add header
set(header robotMcl.hpp ${HEADER_FILES})

#create testCases Library
add_library(robotMcl STATIC  ${header} ${src} )
target_include_directories(robotMcl PUBLIC ${HEADER_FILES_INCLUDE_DIRS})



