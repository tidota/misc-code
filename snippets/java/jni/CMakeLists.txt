cmake_minimum_required(VERSION 2.6)
project(jniprac)

find_package(JNI REQUIRED)
include_directories(${JNI_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SRC sample.cpp)
add_library(sample SHARED ${SRC})
