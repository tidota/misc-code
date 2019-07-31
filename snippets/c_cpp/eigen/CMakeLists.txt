project (eigen_prac)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable (prac prac.cpp)
target_link_libraries (prac Eigen3::Eigen)
