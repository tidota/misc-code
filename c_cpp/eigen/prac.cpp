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

  VectorXd v2;
  v2 = v;
  v2(1) = 500;

  std::cout << "v2 = v1 and v2(1) = 500" << std::endl;
  std::cout << "v:  " << std::endl << v << std::endl;
  std::cout << "v2: " << std::endl << v2 << std::endl;

  VectorXd v3(2);
  v3 << 1, 2;
  std::cout << "v3: " << std::endl;
  std::cout << v3 << std::endl;

  VectorXd v4(3);
  v4 << 1, 0, 0;
  MatrixXd v4v4 = v4 * v4.transpose();
  std::cout << "v4: " << std::endl;
  std::cout << v4 << std::endl;
  std::cout << "mat = v4 * v4.transpose()" << std::endl;
  std::cout << v4v4 << std::endl;

  return 0;
}