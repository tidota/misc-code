#include <iostream>
#include <opencv2/opencv.hpp>

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

int main()
{
  std::cout << std::endl;
  std::cout << "This program calculates covariance and information matrices" << std::endl;
  std::cout << "for a 3D pose based on the given standard deviations (position" << std::endl;
  std::cout << "and rotation)" << std::endl << std::endl;
  std::cout << "Note: standard dev for rotation is set in degrees for ease of" << std::endl;
  std::cout << "input but the resulted matrices are in radian." << std::endl << std::endl;

  double std_pose;
  std::cout << "standard deviation of pose (in meters): ";
  std::cin >> std_pose;

  double std_rot_deg;
  std::cout << "standard deviation of rotaiton (in degrees): ";
  std::cin >> std_rot_deg;
  double std_rot = std_rot_deg / 180.0 * M_PI;

  std::cout << std::endl;

  cv::Mat cov = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0.0));
  cov.at<float>(0,0) = std_pose * std_pose;
  cov.at<float>(1,1) = std_pose * std_pose;
  cov.at<float>(2,2) = std_pose * std_pose;
  cov.at<float>(3,3) = std_rot * std_rot;
  cov.at<float>(4,4) = std_rot * std_rot;
  cov.at<float>(5,5) = std_rot * std_rot;

  std::cout << "Covariance: " << std::endl << cov << std::endl;

  if (cv::determinant(cov) != 0)
  {
    cv::Mat infoMat = cov.inv();
    std::cout << "Information Matrix: " << std::endl << infoMat << std::endl;
  }
  else
  {
    std::cout << "Determinant of covariance is 0!" << std::endl;
  }

  return 0;
}