cmake_minimum_required(VERSION 3.11)
# Note: Boost1.66 needs CMake 3.11 (see REPARATION.md)

project (boost_prac)

# If you have another version of boost (compiled from source),
# set the following variables accordingly. (see PREPARATION.md for details)
set( BOOST_ROOT "/usr/local" CACHE PATH "Boost library path" )
set( Boost_NO_BOOST_CMAKE on CACHE BOOL "Do not use CMake for Boost(?)" )
set( Boost_NO_SYSTEM_PATHS on CACHE BOOL "Do not search system for Boost" )

# Then, just run this
# https://cmake.org/cmake/help/v3.15/module/FindBoost.html
find_package(Boost 1.66 REQUIRED COMPONENTS system locale regex thread)
#find_package(Boost 1.56 REQUIRED COMPONENTS date_time filesystem iostreams)
include_directories(${Boost_INCLUDE_DIRS})

file(GLOB SRC_FILES *.cpp)
foreach (src ${SRC_FILES})
  get_filename_component(target ${src} NAME_WE)
  add_executable(${target} ${src})
  target_link_libraries(${target} ${Boost_LIBRARIES} pthread)
endforeach ()

add_subdirectory(asio1)
add_subdirectory(asio2)
