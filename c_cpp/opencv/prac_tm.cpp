#include <stdio.h>
#include <opencv2/opencv.hpp>
using namespace cv;
int main(int argc, char** argv )
{
  Mat img = Mat::zeros(10, 10, CV_32F);
  for (int i = 0; i < img.rows; ++i)
  {
    float* p = img.ptr<float>(i);
    for (int j = 0; j < img.cols; ++j)
    {
      if ((i == 5 && 2 <= j && j <= 4) || (j == 3 && 4 <= i && i <= 6))
      {
        p[j] = std::log(0.9);
      }
      else
      {
        p[j] = std::log(0.1);
      }
      std::cout << p[j] << " ";
    }
    std::cout << std::endl;
  }
  Mat tmp = Mat::zeros(3, 3, CV_32F);
  for (int i = 0; i < tmp.rows; ++i)
  {
    float* p = tmp.ptr<float>(i);
    for (int j = 0; j < tmp.cols; ++j)
    {
      if ((i == 1 && 0 <= j && j <= 2) || (j == 1 && 0 <= i && i <= 2))
      {
        p[j] = 1;
      }
      else
      {
        p[j] = 0;
      }
      std::cout << p[j] << " ";
    }
    std::cout << std::endl;
  }

  std::cout << "template type? " << tmp.type() << std::endl;
  std::cout << "image type? " << img.type() << std::endl;

  Mat res;
  matchTemplate(img, tmp, res, TM_CCORR);

  double minVal; double maxVal; Point minLoc; Point maxLoc;
  minMaxLoc( res, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
  std::cout << "max: " << maxVal << std::endl << maxLoc << std::endl;
  std::cout << "min: " << minVal << std::endl << minLoc << std::endl;

  return 0;
}
