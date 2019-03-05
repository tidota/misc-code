# Set minimum required version of CMake
cmake_minimum_required (VERSION 2.6)

# Set the project name
project (CPP_SAMPLES)

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -pedantic-errors -Wall")

# Add source directories
add_subdirectory(cpp11_more)

