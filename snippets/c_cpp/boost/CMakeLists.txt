cmake_minimum_required(VERSION 3.10)
project (boost_prac)

# https://cmake.org/cmake/help/v3.15/module/FindBoost.html
find_package(Boost REQUIRED COMPONENTS locale regex)
#find_package(Boost 1.56 REQUIRED COMPONENTS date_time filesystem iostreams)
include_directories(${Boost_INCLUDE_DIRS})

file(GLOB SRC_FILES *.cpp)
foreach (src ${SRC_FILES})
  get_filename_component(target ${src} NAME_WE)
  add_executable(${target} ${src})
  target_link_libraries(${target} ${Boost_LIBRARIES})
endforeach ()
