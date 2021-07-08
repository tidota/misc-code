cmake_minimum_required(VERSION 3.16)
SET(CMAKE_C_FLAGS "")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++03")

project(pointers_prac)

file(GLOB C_SRC_FILES *.c)

foreach (src ${C_SRC_FILES})
  get_filename_component(target ${src} NAME_WE)
  add_executable(${target} ${src})
endforeach ()

file(GLOB CPP_SRC_FILES *.cpp)

foreach (src ${CPP_SRC_FILES})
  get_filename_component(target ${src} NAME_WE)
  add_executable(${target} ${src})
endforeach ()
