cmake_policy(SET CMP0012 NEW)
find_package(OpenMP REQUIRED)

if (OPENMP_FOUND)
message(STATUS "OPENMP found")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ${OpenMP_CXX_FLAGS}")
endif()

file(GLOB SRC_FILES *.cpp)

foreach (src ${SRC_FILES})
  get_filename_component(target ${src} NAME_WE)
  add_executable(${target} ${src})
endforeach ()

