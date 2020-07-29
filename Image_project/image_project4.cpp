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


#define  IMAGE1_NUMBER_OF_SAMPLES   4000
#define  NUMBER_OF_INTERVAL_ROWS    100
#define  IMAGE2_NUMBER_OF_SAMPLES   4100





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

	cout << "x_dis:" << x_dis << ",y_dis" << y_dis << endl;

#if 1
	//把第二幅图像拼接到第一幅上
	
	Mat destImage( image01_rotate.rows + x_dis, image01_rotate.cols + abs(y_dis),CV_8UC3);

	destImage.setTo(0);
	if(y_dis < 0){
		image01_rotate.copyTo(destImage(Rect(abs(y_dis), x_dis, image01_rotate.cols, image01_rotate.rows)));
		image02_rotate.copyTo(destImage(Rect(0,0, image02_rotate.cols, image02_rotate.rows)));
    }else{
		image01_rotate.copyTo(destImage(Rect(0, x_dis, image01_rotate.cols, image01_rotate.rows)));
		image02_rotate.copyTo(destImage(Rect(y_dis,0, image02_rotate.cols, image02_rotate.rows)));
	}
#endif


	imwrite("map.jpg", destImage);

	cout << "-----------ok-------------" << endl;

	waitKey();
	return 0;
}


int Overlapping_image_mosaic_algorithm( Mat &image1, Mat &image2, int &x_dis, int &y_dis)

{
	//快速匹配
	//在image1 中的上半部分 1/2 出，取一个4000*100 的匹配块
	int base[IMAGE1_NUMBER_OF_SAMPLES];
	int start_row = (image1.rows / 4);
	int start_col = (image1.cols / 2) - IMAGE1_NUMBER_OF_SAMPLES/2;

	for(int i=0; i<IMAGE1_NUMBER_OF_SAMPLES; i++)
	{
		base[i] = image1.at<uchar>(start_row - NUMBER_OF_INTERVAL_ROWS, start_col + i) - image1.at<uchar>(start_row, start_col + i);
		image1.at<uchar>(start_row - NUMBER_OF_INTERVAL_ROWS, start_col + i) = 0;
		image1.at<uchar>(start_row, start_col + i) = 0;
	}

	int num = image2.rows  - NUMBER_OF_INTERVAL_ROWS;
	int match_start_col = image2.cols / 2 - IMAGE2_NUMBER_OF_SAMPLES / 2;

	int min_all_err = INT_MAX;
	int min_all_num = 0;
	int min_all_num_dis = 0;

	int min_all_err2 = INT_MAX;
	int min_all_num2 = 0;
	int min_all_num_dis2 = 0;

	int min_all_err3 = INT_MAX;
	int min_all_num3 = 0;
	int min_all_num_dis3 = 0;

	int min_err[num];
	int min_err_dis[num] = {0};

	for(int i=0; i<num; i++)
	{
		min_err[i] = INT_MAX;
	}

	int match_image[IMAGE2_NUMBER_OF_SAMPLES];
	int dis = IMAGE2_NUMBER_OF_SAMPLES - IMAGE1_NUMBER_OF_SAMPLES + 1;

	//在image2 进行匹配，找出最小的3 个值
	for(int n = 0; n < num; n++)
	{
		for(int i=0; i<IMAGE2_NUMBER_OF_SAMPLES; i++)
		{
			match_image[i] = image2.at<uchar>(n, match_start_col + i) -
								 image2.at<uchar>(n + NUMBER_OF_INTERVAL_ROWS, match_start_col + i);
		}

		for(int i=0; i<dis; i++)
		{
			int err = 0;
			for(int j=0; j<IMAGE1_NUMBER_OF_SAMPLES; j++)
			{
				err += pow(match_image[j+i] - base[j], 2);
			}

			if(err < min_err[n])
			{
				min_err[n] = err;
				min_err_dis[n] = i;

				if(min_err[n] < min_all_err)
				{
					min_all_err3 = min_all_err2;
					min_all_num_dis3 = min_all_num_dis2;
					min_all_num3 = min_all_num2;

					min_all_err2 = min_all_err;
					min_all_num_dis2 = min_all_num_dis;
					min_all_num2 = min_all_num;
				
					min_all_err = min_err[n];
					min_all_num_dis = min_err_dis[n];
					min_all_num = n;
				}
				else if(min_err[n] < min_all_err2)
				{
					min_all_err3 = min_all_err2;
					min_all_num_dis3 = min_all_num_dis2;
					min_all_num3 = min_all_num2;

					min_all_err2 = min_err[n];
					min_all_num_dis2 = min_err_dis[n];
					min_all_num2 = n;
				}
				else if(min_err[n] < min_all_err3)
				{
					min_all_err3 = min_err[n];
					min_all_num_dis3 = min_err_dis[n];
					min_all_num3 = n;
				}
			}

		}
	}


	
	cout << "min all err:" << min_all_err << ",min all num:" << min_all_num << ",min all num dis:" << min_all_num_dis << endl;
	cout << "min all err2:" << min_all_err2 << ",min all num:" << min_all_num2 << ",min all num dis:" << min_all_num_dis2 << endl;
	cout << "min all err3:" << min_all_err3 << ",min all num:" << min_all_num3 << ",min all num dis:" << min_all_num_dis3 << endl;


	//块匹配连续性检查
	int err1 = 0, err2 = 0, err3 = 0;
	for(int i=0; i<NUMBER_OF_INTERVAL_ROWS; i++)
	{
		for(int j=0; j<IMAGE1_NUMBER_OF_SAMPLES; j++)
		{
			err1 += pow(image2.at<uchar>(min_all_num + i, match_start_col + min_all_num_dis + j) - 
								image1.at<uchar>(start_row - NUMBER_OF_INTERVAL_ROWS + i, start_col + j), 2);
			err2 += pow(image2.at<uchar>(min_all_num2 + i, match_start_col + min_all_num_dis2 + j) - 
								image1.at<uchar>(start_row - NUMBER_OF_INTERVAL_ROWS + i, start_col + j), 2);
			err3 += pow(image2.at<uchar>(min_all_num3 + i, match_start_col + min_all_num_dis3 + j) - 
								image1.at<uchar>(start_row - NUMBER_OF_INTERVAL_ROWS + i, start_col + j), 2);
		}
	}


	cout << "err1:" << err1 << ",err2:" << err2 << ",err3" << err3 << endl;

	int err0 = 0;
	err0 = err1 < err2 ? err1 : err2;
	err0 = err0 < err3 ? err0 : err3;

	cout << "err0:" << err0 << endl;

	if(err0 == err2)
	{
		min_all_err = min_all_err2;
		min_all_num_dis = min_all_num_dis2;
		min_all_num = min_all_num2;
	}
	else if(err0 == err3)
	{
		min_all_err = min_all_err3;
		min_all_num_dis = min_all_num_dis3;
		min_all_num = min_all_num3;
	}

	cout << "min_all_err:" << min_all_err << ",min_all_num_dis:" << min_all_num_dis << ",min_all_num" << min_all_num << endl;

	
	for(int i=0; i<IMAGE1_NUMBER_OF_SAMPLES; i++)
	{
		image2.at<uchar>(min_all_num, match_start_col + min_all_num_dis + i) = 0;
		image2.at<uchar>(min_all_num+NUMBER_OF_INTERVAL_ROWS, match_start_col + min_all_num_dis + i) = 0;
	}


	//y < 0, 表示向左 移动的像素，y > 0 表示向 右移动的像素
	y_dis = (IMAGE2_NUMBER_OF_SAMPLES - IMAGE1_NUMBER_OF_SAMPLES) / 2 - min_all_num_dis;
	//x > 0, 向上移动的像素
	x_dis = min_all_num - start_row + NUMBER_OF_INTERVAL_ROWS;

	return 0;
}



void imrotate(Mat& img, Mat& newIm, double angle)
{
	Point2f pt(img.cols/2, img.rows/2);
	Mat r = getRotationMatrix2D(pt, angle, 1.0);
	warpAffine(img, newIm, r, Size(img.cols, img.rows));
}





