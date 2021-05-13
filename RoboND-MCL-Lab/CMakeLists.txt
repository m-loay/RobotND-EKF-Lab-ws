cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(Robot)
set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_STANDARD 11) 

#Set Paths
set(ROBOT_MCL_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/robot_mcl)
set(PLOT_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/plotter)
set(TEST_CASES_INCLIDE_DIR ${CMAKE_SOURCE_DIR}/testCases)
set(PYTHON_INCLIDE_DIR /usr/include/python2.7)
set(NUMBY_INCLIDE_DIR /home/mloay/.local/lib/python2.7/site-packages/numpy/core/include)

include(GNUInstallDirs)
set(PACKAGE_NAME matplotlib_cpp)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/${PACKAGE_NAME}/cmake)


#//Compile with: g++ solution.cpp -o app -std=c++11 -I/usr/include/python2.7 -lpython2.7
find_package (Python3 COMPONENTS Interpreter Development)
unset(NUMPY_VERSION)
unset(NUMPY_INCLUDE_DIR)

if(PYTHONINTERP_FOUND)
  execute_process(COMMAND "${PYTHON_EXECUTABLE}" "-c"
    "import numpy as n; print(n.__version__); print(n.get_include());"
    RESULT_VARIABLE __result
    OUTPUT_VARIABLE __output
    OUTPUT_STRIP_TRAILING_WHITESPACE)

  if(__result MATCHES 0)
    string(REGEX REPLACE ";" "\\\\;" __values ${__output})
    string(REGEX REPLACE "\r?\n" ";"    __values ${__values})
    list(GET __values 0 NUMPY_VERSION)
    list(GET __values 1 NUMPY_INCLUDE_DIR)

    string(REGEX MATCH "^([0-9])+\\.([0-9])+\\.([0-9])+" __ver_check "${NUMPY_VERSION}")
    if(NOT "${__ver_check}" STREQUAL "")
      set(NUMPY_VERSION_MAJOR ${CMAKE_MATCH_1})
      set(NUMPY_VERSION_MINOR ${CMAKE_MATCH_2})
      set(NUMPY_VERSION_PATCH ${CMAKE_MATCH_3})
      math(EXPR NUMPY_VERSION_DECIMAL
        "(${NUMPY_VERSION_MAJOR} * 10000) + (${NUMPY_VERSION_MINOR} * 100) + ${NUMPY_VERSION_PATCH}")
      string(REGEX REPLACE "\\\\" "/"  NUMPY_INCLUDE_DIR ${NUMPY_INCLUDE_DIR})
    else()
     unset(NUMPY_VERSION)
     unset(NUMPY_INCLUDE_DIR)
     message(STATUS "Requested NumPy version and include path, but got instead:\n${__output}\n")
    endif()
  endif()
else()
  message(STATUS "To find NumPy Python interpretator is required to be found.")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NumPy REQUIRED_VARS NUMPY_INCLUDE_DIR NUMPY_VERSION
                                        VERSION_VAR   NUMPY_VERSION)

if(NUMPY_FOUND)
  message(STATUS "NumPy ver. ${NUMPY_VERSION} found (include: ${NUMPY_INCLUDE_DIR})")
endif()


include_directories(${PYTHON_INCLIDE_DIR})
include_directories(${Numpy_DIR})
# Use simulation  testing mode or Unit testing
option (USE_UNIT_TESTING "Use Unit testing Mode" ON)


if(USE_UNIT_TESTING)
    message("Use unit testing Mode")
    enable_testing()

    #Find Gtest lib and include the directories
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})

    #Add Testcases sub-directory
    add_subdirectory(${TEST_CASES_INCLIDE_DIR})
    list(APPEND EXTRA_LIBS testCases)
    list(APPEND EXTRA_INCLUDES "${TEST_CASES_INCLIDE_DIR}")

    #Add robot_mcl sub-directory
    add_subdirectory(${ROBOT_MCL_INCLIDE_DIR})
    list(APPEND EXTRA_LIBS robot_mcl)
    list(APPEND EXTRA_INCLUDES "${ROBOT_MCL_INCLIDE_DIR}")


    #compile the source code and link it with external libraries
    add_executable(gtest main_test.cpp)
    target_link_libraries (gtest PUBLIC ${GTEST_LIBRARIES} pthread ${EXTRA_LIBS})
    target_include_directories(gtest PUBLIC ${CMAKE_BINARY_DIR} ${EXTRA_INCLUDES})

    add_executable(solution solution.cpp)
    target_link_libraries(solution PRIVATE python2.7)
    set_target_properties(solution PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

    #add the testing
    add_test(NAME gtest COMMAND gtest)


else()
    message("Use MCL APP")
    #set build type to debug and C++ standard to 11
    set(CMAKE_BUILD_TYPE Release)
    set(CMAKE_CXX_STANDARD 11) 

    #Add robot_mcl sub-directory
    add_subdirectory(${ROBOT_MCL_INCLIDE_DIR})
    list(APPEND EXTRA_LIBS robot_mcl)
    list(APPEND EXTRA_INCLUDES "${ROBOT_MCL_INCLIDE_DIR}")

    #Add Plotter sub-directory
    add_subdirectory(${PLOT_INCLIDE_DIR})
    list(APPEND EXTRA_LIBS plotter)
    list(APPEND EXTRA_INCLUDES "${PLOT_INCLIDE_DIR}")

    #compile the source code and link it with external libraries
    add_executable(robot main.cpp)
    target_include_directories(robot PUBLIC ${EXTRA_INCLUDES})
    target_link_libraries (robot PUBLIC ${EXTRA_LIBS} ${CMAKE_BINARY_DIR})
    
endif()




