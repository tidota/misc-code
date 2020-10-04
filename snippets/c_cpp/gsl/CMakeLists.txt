# need to install libgsl-dev

cmake_minimum_required(VERSION 3.11)

project(gls_prac)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

find_package(GSL REQUIRED)

file(GLOB SRC_FILES *.cpp)

foreach (src ${SRC_FILES})
  get_filename_component(target ${src} NAME_WE)
  add_executable(${target} ${src})
  target_link_libraries(${target} gsl gslcblas)
endforeach ()
