cmake_minimum_required(VERSION 3.10)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_subdirectory(proto)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

file(GLOB SRC_FILES *.cpp)
foreach (src ${SRC_FILES})
  get_filename_component(target ${src} NAME_WE)
  add_executable(${target} ${src})
  target_link_libraries(${target} intlist str)
endforeach ()
