cmake_minimum_required(VERSION 3.10)
project (boost_prac)

# https://cmake.org/cmake/help/v3.15/module/FindBoost.html
find_package(Boost 1.66 REQUIRED COMPONENTS system locale regex)
#find_package(Boost 1.56 REQUIRED COMPONENTS date_time filesystem iostreams)
include_directories(${Boost_INCLUDE_DIRS})
# to install another version of Boost_LIBRARIES
# https://stackoverflow.com/questions/8430332/uninstall-boost-and-install-another-version
# or by apt?
# https://launchpad.net/~mhier/+archive/ubuntu/libboost-latest

file(GLOB SRC_FILES *.cpp)
foreach (src ${SRC_FILES})
  get_filename_component(target ${src} NAME_WE)
  add_executable(${target} ${src})
  target_link_libraries(${target} ${Boost_LIBRARIES})
endforeach ()
