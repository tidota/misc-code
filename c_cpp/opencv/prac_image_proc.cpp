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

    std::cout << "Gray scaling..." << std::endl;
    Mat gray_image;
    cvtColor(image, gray_image, COLOR_RGB2GRAY);
    // image.converTo(gray_image, CV_32F);

    std::cout << "Making it darker..." << std::endl;
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

    std::cout << "Sharpening..." << std::endl;
    Mat sharp_image;
    Mat kernel = (Mat_<char>(3,3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
    filter2D(image, sharp_image, image.depth(), kernel);

    std::cout << "Arbitrarily modding..." << std::endl;
    Mat mod_image = image.clone();
    Mat mod_image_select = mod_image(Rect(5, 5, 20, 20));
    mod_image_select = Vec3b(0,0,0);
    line(mod_image, Point2f(0, 0), Point2f(100, 50), Scalar(0, 255, 0), 2, LINE_8);

    std::cout << "Blending two images..." << std::endl;
    Mat blended_image;
    Mat gray_image_temp;
    cvtColor(gray_image, gray_image_temp, COLOR_GRAY2RGB);
    addWeighted(image, 0.3, gray_image_temp, 0.7, 0.0, blended_image);

    namedWindow("Original", WINDOW_AUTOSIZE );
    imshow("Original", image);
    namedWindow("Gray scale", WINDOW_AUTOSIZE );
    imshow("Gray scale", gray_image);
    namedWindow("Dark", WINDOW_AUTOSIZE );
    imshow("Dark", dark_image);
    namedWindow("Sharp", WINDOW_AUTOSIZE );
    imshow("Sharp", sharp_image);
    namedWindow("Mod", WINDOW_AUTOSIZE );
    imshow("Mod", mod_image);
    namedWindow("Blended", WINDOW_AUTOSIZE );
    imshow("Blended", blended_image);
    waitKey(0);
    return 0;
}
