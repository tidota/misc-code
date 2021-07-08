cmake_minimum_required(VERSION 3.16)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

project(xlnt_prac)

find_library(xlnt_path NAMES xlnt)
message(STATUS ${xlnt_path})
add_library(xlnt SHARED IMPORTED)
set_target_properties(xlnt PROPERTIES IMPORTED_LOCATION ${xlnt_path})

file(GLOB SRC_FILES *.cpp)

foreach (src ${SRC_FILES})
  get_filename_component(target ${src} NAME_WE)
  add_executable(${target} ${src})
  target_link_libraries(${target} xlnt)
endforeach ()
