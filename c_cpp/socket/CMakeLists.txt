cmake_minimum_required(VERSION 3.16)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")

project(socket_prac)

file(GLOB SRC_FILES *.c)

foreach (src ${SRC_FILES})
  get_filename_component(target ${src} NAME_WE)
  add_executable(${target} ${src})
endforeach ()
