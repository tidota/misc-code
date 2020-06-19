# Set minimum required version of CMake
cmake_minimum_required (VERSION 2.6)

# Set the project name
project (CPP_SAMPLES)

SET(CMAKE_C_FLAGS_DEF "${DMAKE_C_FLAGS}")
SET(CMAKE_CXX_FLAGS_DEF "${DMAKE_CXX_FLAGS}")
SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS_DEF} -g -pedantic-errors -Wall")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -pedantic-errors -Wall")

# Add source directories
add_subdirectory(cpp11_more)
add_subdirectory(encoding)
add_subdirectory(multithread)
add_subdirectory(noncanonical)
add_subdirectory(pointers)
add_subdirectory(socket)
add_subdirectory(ssl)
add_subdirectory(io_cpp)
add_subdirectory(xls)
add_subdirectory(misc)
add_subdirectory(pcl)
add_subdirectory(opencv)
add_subdirectory(eigen)
add_subdirectory(protobuf)

