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

    namedWindow("Original", WINDOW_AUTOSIZE );
    namedWindow("Gray scale", WINDOW_AUTOSIZE );
    imshow("Original", image);
    imshow("Gray scale", gray_image);
    waitKey(0);
    return 0;
}
