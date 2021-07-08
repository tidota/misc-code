cmake_minimum_required(VERSION 3.16)
project (eigen_prac)

project(eigen_prac)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable (prac prac.cpp)
target_link_libraries (prac Eigen3::Eigen)
