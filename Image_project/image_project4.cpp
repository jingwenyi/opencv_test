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
#define  NUMBER_OF_INTERVAL_ROWS    1000
#define  IMAGE2_NUMBER_OF_SAMPLES   4100
#define  IMAGE2_CLIP_WIDTH  		500
#define  INTERFACE_OPTIMIZATION_WIDTH 600





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
void OptimizeSeam(Mat& img1, Mat& img2, Mat& dst, int x_dis, int y_dis);



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

//-------------------622+625---------------
	Mat image01 = imread("/home/wenyi/workspace/test_photo/DSC00622.JPG");
	Mat image02 = imread("/home/wenyi/workspace/test_photo/DSC00625.JPG");
	//先进行图像的旋转
	Mat image01_rotate, image02_rotate;
	imrotate(image01, image01_rotate, yaw_AB_avg - yaw_AB[0]);
	imrotate(image02, image02_rotate, yaw_AB_avg - yaw_AB[3]);

	//imwrite("image01_rotate.jpg", image01_rotate);
	//imwrite("image02_rotate.jpg", image02_rotate);

	Mat image01_gray, image02_gray;
	cvtColor(image01_rotate, image01_gray, CV_RGB2GRAY);
	cvtColor(image02_rotate, image02_gray, CV_RGB2GRAY);

	int x_dis = 0, y_dis = 0;

	Overlapping_image_mosaic_algorithm( image01_gray, image02_gray, x_dis, y_dis);

	//imwrite("image01_gray.jpg", image01_gray);
	//imwrite("image02_gray.jpg", image02_gray);

	cout << "x_dis:" << x_dis << ",y_dis" << y_dis << endl;

	//为目标图片申请空间	
	Mat destImage( image01_rotate.rows + x_dis, image01_rotate.cols + abs(y_dis),CV_8UC3);
	destImage.setTo(0);

	//加权融合
	OptimizeSeam(image01_rotate, image02_rotate, destImage, x_dis, y_dis);

	//imwrite("map.jpg", destImage);

	image01.release();
	image02.release();
	image01_rotate.release();
	image02_rotate.release();
	image01_gray.release();
	image02_gray.release();

	//第三张图片舍弃，角度太大, 5度

//================================628===========================================

	//第四 张图片的融合
	Mat image03 = imread("/home/wenyi/workspace/test_photo/DSC00628.JPG");
	//先进行图像的旋转
	Mat image03_rotate;
	imrotate(image03, image03_rotate, yaw_AB_avg - yaw_AB[6]);
	//imwrite("image03_rotate.jpg", image03_rotate);


	//灰度图转换
	Mat image03_gray, destImage_gray;
	cvtColor(destImage, destImage_gray, CV_RGB2GRAY);
	cvtColor(image03_rotate, image03_gray, CV_RGB2GRAY);

	x_dis = 0;
	y_dis = 0;
	Overlapping_image_mosaic_algorithm( destImage_gray, image03_gray, x_dis, y_dis);

	cout << "image3, x_dis:" << x_dis << ",y_dis:" << y_dis << endl;

	//imwrite("destImage_gray.jpg", destImage_gray);
	//imwrite("image03_gray.jpg", image03_gray);


	//为目标图片申请空间	
	Mat destImage2( destImage.rows + x_dis, destImage.cols + abs(y_dis),CV_8UC3);
	destImage2.setTo(0);

	//加权融合
	OptimizeSeam(destImage, image03_rotate, destImage2, x_dis, y_dis);

	//imwrite("map2.jpg", destImage2);

	destImage.release();
	destImage_gray.release();
	image03.release();
	image03_gray.release();
	image03_rotate.release();


//=========================630==================================

	Mat image04 = imread("/home/wenyi/workspace/test_photo/DSC00630.JPG");
	Mat image04_rotate;
	imrotate(image04, image04_rotate, yaw_AB_avg - yaw_AB[8]);
	//imwrite("image04_rotate.jpg", image04_rotate);

	//灰度图转换
	Mat image04_gray, destImage2_gray;
	cvtColor(destImage2, destImage2_gray, CV_RGB2GRAY);
	cvtColor(image04_rotate, image04_gray, CV_RGB2GRAY);

	x_dis = 0;
	y_dis = 0;
	Overlapping_image_mosaic_algorithm( destImage2_gray, image04_gray, x_dis, y_dis);

	cout << "image4, x_dis:" << x_dis << ",y_dis:" << y_dis << endl;

	
	//imwrite("destImage2_gray.jpg", destImage2_gray);
	//imwrite("image04_gray.jpg", image04_gray);


	//为目标图片申请空间	
	Mat destImage3( destImage2.rows + x_dis, destImage2.cols + abs(y_dis),CV_8UC3);
	destImage3.setTo(0);

	//加权融合
	OptimizeSeam(destImage2, image04_rotate, destImage3, x_dis, y_dis);

	//imwrite("map3.jpg", destImage3);

	destImage2.release();
	destImage2_gray.release();
	image04.release();
	image04_gray.release();
	image04_rotate.release();
	

//===============================632===================
	Mat image05 = imread("/home/wenyi/workspace/test_photo/DSC00632.JPG");
	Mat image05_rotate;
	imrotate(image05, image05_rotate, yaw_AB_avg - yaw_AB[10]);
	//imwrite("image05_rotate.jpg", image05_rotate);

	//灰度图转换
	Mat image05_gray, destImage3_gray;
	cvtColor(destImage3, destImage3_gray, CV_RGB2GRAY);
	cvtColor(image05_rotate, image05_gray, CV_RGB2GRAY);

	x_dis = 0;
	y_dis = 0;
	Overlapping_image_mosaic_algorithm( destImage3_gray, image05_gray, x_dis, y_dis);

	cout << "image4, x_dis:" << x_dis << ",y_dis:" << y_dis << endl;

	
	//imwrite("destImage3_gray.jpg", destImage3_gray);
	//imwrite("image05_gray.jpg", image05_gray);


	//为目标图片申请空间	
	Mat destImage4( destImage3.rows + x_dis, destImage3.cols + abs(y_dis),CV_8UC3);
	destImage4.setTo(0);

	//加权融合
	OptimizeSeam(destImage3, image05_rotate, destImage4, x_dis, y_dis);

	//imwrite("map4.jpg", destImage4);

	destImage3.release();
	destImage3_gray.release();
	image05.release();
	image05_gray.release();
	image05_rotate.release();

//===============================635===================
	Mat image06 = imread("/home/wenyi/workspace/test_photo/DSC00635.JPG");
	Mat image06_rotate;
	imrotate(image06, image06_rotate, yaw_AB_avg - yaw_AB[13]);
	//imwrite("image06_rotate.jpg", image06_rotate);
	
	//灰度图转换
	Mat image06_gray, destImage4_gray;
	cvtColor(destImage4, destImage4_gray, CV_RGB2GRAY);
	cvtColor(image06_rotate, image06_gray, CV_RGB2GRAY);
	
	x_dis = 0;
	y_dis = 0;
	Overlapping_image_mosaic_algorithm( destImage4_gray, image06_gray, x_dis, y_dis);
	
	cout << "image5, x_dis:" << x_dis << ",y_dis:" << y_dis << endl;
	
		
	//imwrite("destImage4_gray.jpg", destImage4_gray);
	//imwrite("image06_gray.jpg", image06_gray);
	
	
	//为目标图片申请空间	
	Mat destImage5( destImage4.rows + x_dis, destImage4.cols + abs(y_dis),CV_8UC3);
	destImage5.setTo(0);
	
	//加权融合
	OptimizeSeam(destImage4, image06_rotate, destImage5, x_dis, y_dis);
	
	//imwrite("map5.jpg", destImage5);
	
	destImage4.release();
	destImage4_gray.release();
	image06.release();
	image06_gray.release();
	image06_rotate.release();

//===============================636===================
	Mat image07 = imread("/home/wenyi/workspace/test_photo/DSC00636.JPG");
	Mat image07_rotate;
	imrotate(image07, image07_rotate, yaw_AB_avg - yaw_AB[14]);
	//imwrite("image07_rotate.jpg", image07_rotate);
		
	//灰度图转换
	Mat image07_gray, destImage5_gray;
	cvtColor(destImage5, destImage5_gray, CV_RGB2GRAY);
	cvtColor(image07_rotate, image07_gray, CV_RGB2GRAY);
		
	x_dis = 0;
	y_dis = 0;
	Overlapping_image_mosaic_algorithm( destImage5_gray, image07_gray, x_dis, y_dis);
		
	cout << "image6, x_dis:" << x_dis << ",y_dis:" << y_dis << endl;
		
			
	//imwrite("destImage5_gray.jpg", destImage5_gray);
	//imwrite("image07_gray.jpg", image07_gray);
		
		
	//为目标图片申请空间	
	Mat destImage6( destImage5.rows + x_dis, destImage5.cols + abs(y_dis),CV_8UC3);
	destImage6.setTo(0);
		
	//加权融合
	OptimizeSeam(destImage5, image07_rotate, destImage6, x_dis, y_dis);
		
	imwrite("map6.jpg", destImage6);
		
	destImage5.release();
	destImage5_gray.release();
	image07.release();
	image07_gray.release();
	image07_rotate.release();


//===============================638===================
	Mat image08 = imread("/home/wenyi/workspace/test_photo/DSC00638.JPG");
	Mat image08_rotate;
	imrotate(image08, image08_rotate, yaw_AB_avg - yaw_AB[16]);
	imwrite("image08_rotate.jpg", image08_rotate);
			
	//灰度图转换
	Mat image08_gray, destImage6_gray;
	cvtColor(destImage6, destImage6_gray, CV_RGB2GRAY);
	cvtColor(image08_rotate, image08_gray, CV_RGB2GRAY);
			
	x_dis = 0;
	y_dis = 0;
	Overlapping_image_mosaic_algorithm( destImage6_gray, image08_gray, x_dis, y_dis);
			
	cout << "image7, x_dis:" << x_dis << ",y_dis:" << y_dis << endl;
			
				
	imwrite("destImage6_gray.jpg", destImage6_gray);
	imwrite("image08_gray.jpg", image08_gray);
			
			
	//为目标图片申请空间	
	Mat destImage7( destImage6.rows + x_dis, destImage6.cols + abs(y_dis),CV_8UC3);
	destImage7.setTo(0);
			
	//加权融合
	OptimizeSeam(destImage6, image08_rotate, destImage7, x_dis, y_dis);
			
	imwrite("map7.jpg", destImage7);
			
	destImage6.release();
	destImage6_gray.release();
	image08.release();
	image08_gray.release();
	image08_rotate.release();



	cout << "-----------ok-------------" << endl;

	waitKey();
	return 0;
}

#define NUM_OF_ERR		3
int Overlapping_image_mosaic_algorithm( Mat &image1, Mat &image2, int &x_dis, int &y_dis)

{
	//快速匹配
	//在image1 中的上半部分 1/2 出，取一个4000*100 的匹配块
	int base[IMAGE1_NUMBER_OF_SAMPLES];
	int start_row = 1000 + NUMBER_OF_INTERVAL_ROWS;
	int start_col = (image1.cols / 2) - IMAGE1_NUMBER_OF_SAMPLES/2;

	for(int i=0; i<IMAGE1_NUMBER_OF_SAMPLES; i++)
	{
		base[i] = image1.at<uchar>(start_row - NUMBER_OF_INTERVAL_ROWS, start_col + i) - image1.at<uchar>(start_row, start_col + i);
		image1.at<uchar>(start_row - NUMBER_OF_INTERVAL_ROWS, start_col + i) = 0;
		image1.at<uchar>(start_row, start_col + i) = 0;
	}

	int num = image2.rows  - NUMBER_OF_INTERVAL_ROWS;
	int match_start_col = image2.cols / 2 - IMAGE2_NUMBER_OF_SAMPLES / 2;


	
	int min_all_err[NUM_OF_ERR];
	int min_all_num[NUM_OF_ERR];
	int min_all_num_dis[NUM_OF_ERR];

	for(int i=0; i<NUM_OF_ERR; i++)
	{
		min_all_err[i] = INT_MAX;
		min_all_num[i] = 0;
		min_all_num_dis[i] = 0;
	}

	
	
	

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

				for(int j=0; j<NUM_OF_ERR; j++)
				{
					if(min_err[n] < min_all_err[j])
					{
						for(int k=NUM_OF_ERR - 1; k>j; k--)
						{
							min_all_err[k] = min_all_err[k-1];
							min_all_num_dis[k] = min_all_num_dis[k-1];
							min_all_num[k] = min_all_num[k-1];
						}


						min_all_err[j] = min_err[n];
						min_all_num_dis[j] = min_err_dis[n];
						min_all_num[j] = n;
						break;
					}
				}
			}

		}
	}


	for(int i=0; i<NUM_OF_ERR; i++)
	{
		cout << "i=" << i << ",err:" << min_all_err[i] << ",num:" << min_all_num[i] << ",dis:" << min_all_num_dis[i] << endl;
	}


	//块匹配连续性检查
	int err[NUM_OF_ERR];
	for(int i=0; i<NUM_OF_ERR; i++)
	{
		err[i] = 0;
	}

	
	for(int i=0; i<NUMBER_OF_INTERVAL_ROWS; i++)
	{
		for(int j=0; j<IMAGE1_NUMBER_OF_SAMPLES; j++)
		{
			for(int k=0; k<NUM_OF_ERR; k++)
			{
				err[k] += pow(image2.at<uchar>(min_all_num[k] + i, match_start_col + min_all_num_dis[k] + j) - 
								image1.at<uchar>(start_row - NUMBER_OF_INTERVAL_ROWS + i, start_col + j), 2);
			}
		}
	}


	for(int i=0; i<NUM_OF_ERR; i++)
	{
		cout << "err" << i << ":" << err[i] << endl;
	}

	int err0 = INT_MAX;
	int err0_i = 0;

	for(int i=0; i<NUM_OF_ERR; i++)
	{
		if(err[i] < err0)
		{
			err0 = err[i];
			err0_i = i;
		}
	}

	cout << "err0:" << err0 << ",i:" << err0_i << endl;



	cout << "min err:" << min_all_err[err0_i] << ",min dis:" << min_all_num_dis[err0_i] << ",min_num:" << min_all_num[err0_i] << endl;

	
	for(int i=0; i<IMAGE1_NUMBER_OF_SAMPLES; i++)
	{
		image2.at<uchar>(min_all_num[err0_i], match_start_col + min_all_num_dis[err0_i] + i) = 0;
		image2.at<uchar>(min_all_num[err0_i]+NUMBER_OF_INTERVAL_ROWS, match_start_col + min_all_num_dis[err0_i] + i) = 0;
	}


	//y < 0, 表示向左 移动的像素，y > 0 表示向 右移动的像素
	y_dis = (IMAGE2_NUMBER_OF_SAMPLES - IMAGE1_NUMBER_OF_SAMPLES) / 2 - min_all_num_dis[err0_i];
	//x > 0, 向上移动的像素
	x_dis = min_all_num[err0_i] - start_row + NUMBER_OF_INTERVAL_ROWS;

	if(x_dis <= 0)
	{
		return -1;
	}

	return 0;
}



void imrotate(Mat& img, Mat& newIm, double angle)
{
	Point2f pt(img.cols/2, img.rows/2);
	Mat r = getRotationMatrix2D(pt, angle, 1.0);
	warpAffine(img, newIm, r, Size(img.cols, img.rows));
}



//优化两图的连接处，使得拼接自然
void OptimizeSeam(Mat& img1, Mat& img2, Mat& dst, int x_dis, int y_dis)
{
	//裁剪掉img2 下边IMAGE2_CLIP_WIDTH  个像素，处理由于旋转导致没有图像的问题
	Mat img2_tmp = img2(Range(0, img2.rows - IMAGE2_CLIP_WIDTH), Range(0, img2.cols));

	if(y_dis < 0){
		img1.copyTo(dst(Rect(abs(y_dis), x_dis, img1.cols, img1.rows)));
		img2_tmp.copyTo(dst(Rect(0,0, img2_tmp.cols, img2_tmp.rows)));
    }else{
		img1.copyTo(dst(Rect(0, x_dis, img1.cols, img1.rows)));
		img2_tmp.copyTo(dst(Rect(y_dis, 0, img2_tmp.cols, img2_tmp.rows)));
	}

#if 1
	//优化接口600 行像素
	int w = INTERFACE_OPTIMIZATION_WIDTH;
	int dst_rows_start = img2_tmp.rows - 1;
	int dst_cols_start = 0;
	int img1_rows_start = img1.rows - (dst.rows - img2_tmp.rows) - 1;
	int img1_cols_start = y_dis < 0 ? abs(y_dis) - 1 : 0;
	int img2_rows_start = img2_tmp.rows - 1;
	int img2_cols_start = y_dis < 0 ? 0 : y_dis - 1;

	float alpha = 1;//img1中像素的权重
	
	for(int i=0; i<w; i++)//rows
	{
		uchar* p = img1.ptr<uchar>(img1_rows_start - i);
		uchar* t = img2_tmp.ptr<uchar>(img2_rows_start - i);
        uchar* d = dst.ptr<uchar>(dst_rows_start - i);

		for(int j=0; j<dst.cols - abs(y_dis); j++)
		{
			//如果遇到img2 中无像素的黑点，则完全拷贝img1 中的数据
			if(t[(img2_cols_start + j) * 3] == 0 &&
				t[(img2_cols_start + j) * 3 + 1] == 0 &&
				t[(img2_cols_start + j) * 3 + 2] == 0)
			{
				alpha = 1.0f;
			}
			else
			{
				alpha = ((float)w - (float)i) / (float)w;
			}


			d[(dst_cols_start + j) * 3] = p[(img1_cols_start + j) * 3] * alpha + t[(img2_cols_start + j) * 3] * (1 - alpha);
			d[(dst_cols_start + j) * 3 + 1] = p[(img1_cols_start + j) * 3 + 1] * alpha + t[(img2_cols_start + j) * 3 + 1] * (1 - alpha);
			d[(dst_cols_start + j) * 3 + 2] = p[(img1_cols_start + j) * 3 + 2] * alpha + t[(img2_cols_start + j) * 3 + 2] * (1 - alpha);
		}		
	}
#endif
}

