cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

project(octomap_prac)

find_package(octomap REQUIRED)

file(GLOB SRC_FILES *.cpp)

foreach (src ${SRC_FILES})
  get_filename_component(target ${src} NAME_WE)
  add_executable(${target} ${src})
  target_link_libraries(${target} ${OCTOMAP_LIBRARIES})
endforeach ()
