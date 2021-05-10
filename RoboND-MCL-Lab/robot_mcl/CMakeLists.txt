# set minimum cmake version
cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

# project name and language
project(robot_mcl)
message("robot_mcl")
message(${CMAKE_SOURCE_DIR})
message(${PROJECT_SOURCE_DIR})
message(${ROBOT_MCL_INCLIDE_DIR})
#set standard of C++11 and build type to debug
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

#set sources and headers
set(src robot_mcl.cpp)
set(header robot_mcl.hpp)

#create robot_mcl Library
add_library(robot_mcl STATIC ${header} ${src})
