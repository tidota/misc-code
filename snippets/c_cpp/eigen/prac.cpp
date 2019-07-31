#include <iostream>

#include <Eigen/Dense>

using namespace Eigen;

int main()
{
  Matrix3d A = Matrix3d::Identity();
  std::cout << "A = I: " << std::endl << A << std::endl;

  A(1,1) = 5;
  std::cout << "A(1,1) = 5: " << std::endl << A << std::endl;

  return 0;
}