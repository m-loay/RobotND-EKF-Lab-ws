cmake_minimum_required(VERSION 3.0)
project(testCases)

#set standard of C++11 and build type to debug
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

#set sources and headers
set(src testCases.cpp)
set(header testCases.hpp)

#create testCases Library
add_library(testCases STATIC ${header} ${src})
target_include_directories(testCases PUBLIC  ${CMAKE_INCLUDE_CURRENT_DIR} ${ROBOT_MCL_INCLIDE_DIR})
