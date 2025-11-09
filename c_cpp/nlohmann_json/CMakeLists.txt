cmake_minimum_required(VERSION 3.8)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS_DEF} -g -Wall")

project(json_prac)

find_package(nlohmann_json REQUIRED)

file(GLOB SRC_FILES *.cpp)

foreach (src ${SRC_FILES})
  get_filename_component(target ${src} NAME_WE)
  add_executable(${target} ${src})
  target_link_libraries(${target} nlohmann_json::nlohmann_json)
endforeach ()
