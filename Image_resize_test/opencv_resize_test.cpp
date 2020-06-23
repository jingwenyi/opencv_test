#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace cv;


int main()
{
	Mat srcImage = imread("src.jpg");
	Mat dstImage;

	resize(srcImage, dstImage, Size(srcImage.cols/4, srcImage.rows/4),INTER_AREA);

	imwrite("dest.jpg", dstImage);

	waitKey();

	return 0;
}




