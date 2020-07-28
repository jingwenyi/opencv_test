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

//Number of interval rows
#define  NUMBER_OF_INTERVAL_ROWS    80
//Number of samples taken from the first image
#define  IMAGE1_NUMBER_OF_SAMPLES   300
#define  IMAGE2_NUMBER_OF_SAMPLES   400


int Overlapping_image_mosaic_algorithm( Mat &image1, Mat &image2, int &x_dis, int &y_dis)
{
		//图像image2 应该拼接到image1 的上方(530*795)
		int base[IMAGE1_NUMBER_OF_SAMPLES];
		int start_row = (image1.rows / 4);
		int start_col = (image1.cols / 2) - IMAGE1_NUMBER_OF_SAMPLES/2;
		
		for(int i=0; i<IMAGE1_NUMBER_OF_SAMPLES; i++)
		{
			base[i] = image1.at<uchar>(start_row - NUMBER_OF_INTERVAL_ROWS, start_col + i) - image1.at<uchar>(start_row, start_col + i);
			image1.at<uchar>(start_row - NUMBER_OF_INTERVAL_ROWS, start_col + i) = 0;
			image1.at<uchar>(start_row, start_col + i) = 0;
		}

		imshow("i1", image1);

		
		int num = image2.rows  - NUMBER_OF_INTERVAL_ROWS;
		int match_start_col = image2.cols / 2 - IMAGE2_NUMBER_OF_SAMPLES / 2;

		int min_all_err = INT_MAX;
		int min_all_num = 0;
		int min_all_num_dis = 0;

		int min_err[num];
		int min_err_dis[num] = {0};

		for(int i=0; i<num; i++)
		{
			min_err[i] = INT_MAX;
		}

		int match_image[IMAGE2_NUMBER_OF_SAMPLES];
		int dis = IMAGE2_NUMBER_OF_SAMPLES - IMAGE1_NUMBER_OF_SAMPLES + 1;

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
					err += pow((match_image[j+i] - base[j]), 2);
				}

				if(err < min_err[n])
				{
					min_err[n] = err;
					min_err_dis[n] = i;

					if(min_err[n] < min_all_err)
					{
						min_all_err = min_err[n];
						min_all_num_dis = min_err_dis[n];
						min_all_num = n;
					}
				}

			}
		}

#if 1
		for(int n = 0; n<num; n++)
		{
			cout << "num:" << n << ",min_err_dis:" << min_err_dis[n] << ",min_err:" << min_err[n] << endl;
		}

		cout << "min all err:" << min_all_err << ",min all num:" << min_all_num << ",min all num dis:" << min_all_num_dis << endl;

		for(int i=0; i<IMAGE1_NUMBER_OF_SAMPLES; i++)
		{
			image2.at<uchar>(min_all_num, match_start_col + min_all_num_dis + i) = 0;
			image2.at<uchar>(min_all_num+NUMBER_OF_INTERVAL_ROWS, match_start_col + min_all_num_dis + i) = 0;
		}

		imshow("i2", image2);
#endif
		//y < 0, 表示向左 移动的像素，y > 0 表示向 右移动的像素
		y_dis = (IMAGE2_NUMBER_OF_SAMPLES - IMAGE1_NUMBER_OF_SAMPLES) / 2 - min_all_num_dis;
		//x > 0, 向上移动了14 个像素点
		x_dis = min_all_num - start_row + NUMBER_OF_INTERVAL_ROWS;

	return 0;
}

void optimize_seam(Mat& img1, Mat& img2, Mat& dst, int x_dis, int y_dis);

int main(int argc, char *argv[])
{
	Mat image01= imread("./test_photo/DSC00032.JPG");
	Mat image02 = imread("./test_photo/DSC00033.JPG");


	Mat image1, image2;
	cvtColor(image01, image1, CV_RGB2GRAY);
	cvtColor(image02, image2, CV_RGB2GRAY);

	int x_dis, y_dis;
	//计算图像拼接的位置
	Overlapping_image_mosaic_algorithm(image1, image2, x_dis, y_dis);

	cout << "x_dis:" << x_dis << ", y_dis:" << y_dis << endl;

	//把第二幅图像拼接到第一幅上
	
	Mat destImage( image01.rows + x_dis, image01.cols + abs(y_dis),CV_8UC3);

	destImage.setTo(0);
	if(y_dis < 0){
		image01.copyTo(destImage(Rect(abs(y_dis), x_dis, image01.cols, image01.rows)));
		image02.copyTo(destImage(Rect(0,0, image02.cols, image02.rows)));
    }else{
		image01.copyTo(destImage(Rect(0, x_dis, image01.cols, image01.rows)));
		image02.copyTo(destImage(Rect(y_dis,0, image02.cols, image02.rows)));
	}

	optimize_seam(image01, image02, destImage, x_dis, y_dis);

	imshow("dest", destImage);
	imwrite("dest.jpg", destImage);

	waitKey();

	return 0;
}

//优化两图的连接处，使得拼接自然
void optimize_seam(Mat& img1, Mat& img2, Mat& dst, int x_dis, int y_dis)
{
	int start = img2.rows;
	//重叠区域的宽度
	double process_width = img2.rows - x_dis;
	double alpha = 1; //img1 中像素的权重

	int i,k;
	for(i=x_dis, k=0; i<img2.rows && k<img1.rows; i++, k++)
	{
		uchar* p = img1.ptr<uchar>(k);
		uchar* t = img2.ptr<uchar>(i);
		uchar* d = dst.ptr<uchar>(i);

		alpha = (process_width - (i-x_dis)) / process_width;
		int j,n;
		for(j=abs(y_dis), n=0; j<img2.cols && n<img1.cols; j++, n++)
		{
			d[j*3] = p[n*3]*alpha + t[j*3]*(1-alpha);
			d[j*3 + 1] = p[n*3 + 1]*alpha + t[j*3 + 1]*(1-alpha);
			d[j*3 + 2] = p[n*3 + 2]*alpha + t[j*3 + 2]*(1-alpha);
		}
	}
}


