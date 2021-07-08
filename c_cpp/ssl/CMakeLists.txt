#install libssl-dev

cmake_minimum_required(VERSION 3.16)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++03")

project(ssl_prac)

file(GLOB SRC_FILES *.cpp)

foreach (src ${SRC_FILES})
  get_filename_component(target ${src} NAME_WE)
  add_executable(${target} ${src})
  target_link_libraries(${target} ssl crypto)
endforeach ()
