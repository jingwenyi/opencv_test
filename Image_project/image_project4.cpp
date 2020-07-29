#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <limits.h>
using namespace std;
using namespace cv;


//AB 航线上有27 张图片，对应27 个偏航角
const static float yaw_AB[] ={	92.35f,	91.36f, 87.14f, 91.96f, 85.27f, 94.03f, 90.73f, 95.61f, 95.35f,
							  	94.07f, 99.07f, 88.51f, 89.47f, 90.03f, 90.48f, 87.24f, 88.03f, 91.4f,
							  	93.31f, 91.7f, 	92.13f, 93.33f, 92.82f, 92.64f, 92.29f, 90.83f, 96.42f}; 
//CD 航线上有29 张图片，对应29 个偏航角
const static float yaw_CD[] = {	281.47f,	287.41f,	282.65f,	290.22f,	284.3f,		283.5f,		285.75f,	288.84f,	290.7f,		284.89f,
								284.42f,	288.77f,	280.14f,	285.65f,	285.61f,	287.46f,	286.84f,	286.02f,	276.95f,	285.61f,
								280.0f,		278.9f,		283.64f,	281.82f,	285.08f,	283.47f,	289.31f,	279.25f,	283.06f};



int Overlapping_image_mosaic_algorithm( Mat &image1, Mat &image2, int &x_dis, int &y_dis);
void imrotate(Mat& img, Mat& newIm, double angle);


int main(int argc, char *argv[])
{

	//由于没有AB 和CD 航线的方向数据，这里用所有照片偏航角的平均数据来做航线的方向数据
	float yaw_AB_avg = 0.0f, yaw_CD_avg = 0.0f;

	for(int i=0; i<sizeof(yaw_AB)/4; i++)
	{
		yaw_AB_avg += yaw_AB[i];
	}
	yaw_AB_avg /= sizeof(yaw_AB) / 4;

	for(int i=0; i<sizeof(yaw_CD)/4; i++)
	{
		yaw_CD_avg += yaw_CD[i];
	}
	yaw_CD_avg /= sizeof(yaw_CD) / 4;

	//AB:91.7618, CD:284.542
	cout << "yaw AB avg:" << yaw_AB_avg << ",yaw CD avg:" << yaw_CD_avg << endl;
#if 0
	//申请一张很大的画布，画布的大小可以由航线上照片的数量估计出来，这里是任意的估计的
	Mat mymap( 10000, 10000,CV_8UC3);

	//循环读取每一张图片DSC00622.JPG~DSC00677.JPG
	for(int i=0; i<56; i++)
	{
		
	}
	
	mymap.setTo(100);
	imwrite("mymap.jpg", mymap);
#endif

	
	Mat image01 = imread("/home/wenyi/workspace/test_photo/DSC00622.JPG");
	Mat image02 = imread("/home/wenyi/workspace/test_photo/DSC00623.JPG");
	//先进行图像的旋转
	Mat image01_rotate, image02_rotate;
	imrotate(image01, image01_rotate, yaw_AB_avg - yaw_AB[0]);
	imrotate(image02, image02_rotate, yaw_AB_avg - yaw_AB[1]);

	//imwrite("image01_rotate.jpg", image01_rotate);
	//imwrite("image02_rotate.jpg", image02_rotate);

	Mat image01_gray, image02_gray;
	cvtColor(image01_rotate, image01_gray, CV_RGB2GRAY);
	cvtColor(image02_rotate, image02_gray, CV_RGB2GRAY);

	int x_dis = 0, y_dis = 0;

	Overlapping_image_mosaic_algorithm( image01_gray, image02_gray, x_dis, y_dis);

	imwrite("image01_gray.jpg", image01_gray);
	imwrite("image02_gray.jpg", image02_gray);


	cout << "-----------ok-------------" << endl;

	waitKey();
	return 0;
}


int Overlapping_image_mosaic_algorithm( Mat &image1, Mat &image2, int &x_dis, int &y_dis)

{
	
	return 0;
}



void imrotate(Mat& img, Mat& newIm, double angle)
{
	Point2f pt(img.cols/2, img.rows/2);
	Mat r = getRotationMatrix2D(pt, angle, 1.0);
	warpAffine(img, newIm, r, Size(img.cols, img.rows));
}





