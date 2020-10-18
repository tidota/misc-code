cmake_minimum_required(VERSION 2.6)
project(jniprac)

find_package(JNI REQUIRED)
include_directories(${JNI_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_library(database SHARED database.cpp)

add_library(DBWrapper SHARED DBWrapper.cpp)

target_link_libraries(DBWrapper PUBLIC database)
