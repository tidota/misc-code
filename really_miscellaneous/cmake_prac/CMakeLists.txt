cmake_minimum_required(VERSION 3.2)
project(simple_project)


set(BAR src/bar.cpp)
set(FOO src/foo.cpp)
set(MAIN src/main.cpp)


add_executable(main ${MAIN} ${FOO} ${BAR})
target_compile_options(main PUBLIC -ansi -pedantic-errors -Wall)
target_include_directories(main PUBLIC include)

