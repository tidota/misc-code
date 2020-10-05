# need to install libgsl-dev

cmake_minimum_required(VERSION 3.11)

project(gls_prac)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++11")

find_package(GSL REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

file(GLOB SRC_FILES *.cpp)

foreach (src ${SRC_FILES})
  get_filename_component(target ${src} NAME_WE)
  add_executable(${target} ${src})
  target_link_libraries(${target} gsl gslcblas Eigen3::Eigen)
endforeach ()
