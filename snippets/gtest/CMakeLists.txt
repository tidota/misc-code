# Set minimum required version of CMake
cmake_minimum_required (VERSION 2.6)

# Set the project name
project (GTEST_PRAC)

SET(CMAKE_C_FLAGS_DEF "${DMAKE_C_FLAGS}")
SET(CMAKE_CXX_FLAGS_DEF "${DMAKE_CXX_FLAGS}")
SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS_DEF} -g -pedantic-errors -Wall")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -pedantic-errors -Wall")

# GTest
enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Add exec
add_executable(UNIT-test1 test1.cpp)
target_link_libraries(UNIT-test1 ${GTEST_LIBRARIES} pthread)
add_test(NAME UNIT-test1 COMMAND UNIT-test1)

