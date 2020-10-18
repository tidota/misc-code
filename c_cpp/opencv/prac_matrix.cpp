#include <stdio.h>
#include <opencv2/opencv.hpp>
using namespace cv;
int main(int argc, char** argv )
{
  Mat I = Mat::eye(3, 3, CV_64F);
  Mat O = Mat::ones(4, 4, CV_8U);
  Mat Z = Mat::zeros(5, 5, CV_8U);
  Mat R = Mat(4, 5, CV_8U);
  randu(R, Scalar::all(0), Scalar::all(255));
  std::cout << "Identity matrix (in python): " << std::endl << format(I, Formatter::FMT_PYTHON) << std::endl;
  std::cout << "All one matrix (in C): " << std::endl << format(O, Formatter::FMT_C) << std::endl;
  std::cout << "All zero matrix: " << std::endl << Z << std::endl;
  std::cout << "Random matrix: " << std::endl << R << std::endl;

  Point2f p2(3, 1);
  Point3f p3(5, 0, -1);
  std::cout << "Point2f: " << p2 << std::endl;
  std::cout << "Point3f: " << p3 << std::endl;

  return 0;
}