cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(Robot)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11) 
# add_executable(Robot solution.cpp robot.cpp)
#Set Paths
set(ROBOT_MCL_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/robot_mcl)
set(PLOT_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/plotter)
set(TEST_CASES_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/testCases)

# # Use simulation  testing mode or Unit testing
# option (USE_UNIT_TESTING "Use Unit testing Mode" ON)

# #set build type to debug and C++ standard to 11
# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_STANDARD 11) 

# #Add robot_mcl sub-directory
# add_subdirectory(${ROBOT_MCL_INCLIDE_DIR})
# list(APPEND EXTRA_LIBS robot_mcl)
# list(APPEND EXTRA_INCLUDES "${ROBOT_MCL_INCLIDE_DIR}")

# #Add Plotter sub-directory
# add_subdirectory(${PLOT_INCLIDE_DIR})
# list(APPEND EXTRA_LIBS plotter)
# list(APPEND EXTRA_INCLUDES "${PLOT_INCLIDE_DIR}")

# #compile the source code and link it with external libraries
# add_executable(robot main.cpp)
# target_include_directories(robot PUBLIC ${EXTRA_INCLUDES})
# target_link_directories(robot PUBLIC ${EXTRA_INCLUDES})
# target_link_libraries (robot PUBLIC ${EXTRA_LIBS})




message("Use unit testing Mode")
enable_testing()

#Find Gtest lib and include the directories
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

#Add Testcases sub-directory
add_subdirectory(${TEST_CASES_INCLIDE_DIR})
list(APPEND EXTRA_LIBS testCases)
list(APPEND EXTRA_INCLUDES "${TEST_CASES_INCLIDE_DIR}")

#Add Plotter sub-directory
add_subdirectory(${PLOT_INCLIDE_DIR})
list(APPEND EXTRA_LIBS plotter)
list(APPEND EXTRA_INCLUDES "${PLOT_INCLIDE_DIR}")

#Add robot_mcl sub-directory
add_subdirectory(${ROBOT_MCL_INCLIDE_DIR})
list(APPEND EXTRA_LIBS robot_mcl)
list(APPEND EXTRA_INCLUDES "${ROBOT_MCL_INCLIDE_DIR}")


#compile the source code and link it with external libraries
add_executable(gtest main_test.cpp)
target_link_libraries (gtest PUBLIC ${GTEST_BOTH_LIBRARIES} ${EXTRA_LIBS})
target_include_directories(gtest PUBLIC ${CMAKE_BINARY_DIR} ${EXTRA_INCLUDES})
#add the testing
add_test(NAME gtest COMMAND gtest)
