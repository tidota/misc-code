#include <stdio.h>
#include <opencv2/opencv.hpp>
using namespace cv;
int main(int argc, char** argv )
{
    if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }
    Mat image;
    image = imread( argv[1], 1 );
    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }

    Mat gray_image;
    cvtColor(image, gray_image, COLOR_RGB2GRAY);

    Mat dark_image = image.clone();
    int channels = dark_image.channels();
    int rows = dark_image.rows;
    int cols = dark_image.cols * channels;

    if (dark_image.isContinuous())
    {
      cols *= rows;
      rows = 1;
    }
    int i, j;
    uchar* p;
    for (i = 0; i < rows; ++i)
    {
      p = dark_image.ptr<uchar>(i);
      for (j = 0; j < cols; ++j)
      {
        p[j] = p[j]/2;
      }
    }

    namedWindow("Original", WINDOW_AUTOSIZE );
    namedWindow("Gray scale", WINDOW_AUTOSIZE );
    namedWindow("Dark", WINDOW_AUTOSIZE );
    imshow("Original", image);
    imshow("Gray scale", gray_image);
    imshow("Dark", dark_image);
    waitKey(0);
    return 0;
}
