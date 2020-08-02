#include <iostream>

#include <Eigen/Dense>

using namespace Eigen;

int main()
{
  Matrix3d A = Matrix3d::Identity();
  std::cout << "A = I: " << std::endl << A << std::endl;

  A(1,1) = 5;
  std::cout << "A(1,1) = 5: " << std::endl << A << std::endl;


  MatrixXd B = MatrixXd::Zero(4, 4);
  Ref<MatrixXd> subB = B.block(1,1,3,3);
  std::cout << "B: " << std::endl << B << std::endl;
  std::cout << "subB: " << std::endl << subB << std::endl;
  std::cout << "mod on B: B(1,1) = 5" << std::endl;
  B(1,1) = 5;
  std::cout << "mod on subB: subB(2,2) = 9" << std::endl;
  subB(2,2) = 9;
  std::cout << "B: " << std::endl << B << std::endl;
  std::cout << "subB: " << std::endl << subB << std::endl;

  VectorXd v = VectorXd::Zero(3);
  v(0) = 3;
  v(1) = 5;
  v(2) = 7;
  std::cout << "v: " << std::endl << v << std::endl;
  std::cout << "v * 5" << std::endl;
  v = v * 5;
  std::cout << "v: " << std::endl << v << std::endl;

  return 0;
}