#include "./image_algorithm/image_algorithm.h"
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



// AB  航线上对应的27 张图片名称
const static char* image_name[27] = {
	"DSC00622.JPG",
	"DSC00623.JPG",
	"DSC00624.JPG",
	"DSC00625.JPG",
	"DSC00626.JPG",
	"DSC00627.JPG",
	"DSC00628.JPG",
	"DSC00629.JPG",
	"DSC00630.JPG",
	"DSC00631.JPG",
	"DSC00632.JPG",
	"DSC00633.JPG",
	"DSC00634.JPG",
	"DSC00635.JPG",
	"DSC00636.JPG",
	"DSC00637.JPG",
	"DSC00638.JPG",
	"DSC00639.JPG",
	"DSC00640.JPG",
	"DSC00641.JPG",
	"DSC00642.JPG",
	"DSC00643.JPG",
	"DSC00644.JPG",
	"DSC00645.JPG",
	"DSC00646.JPG",
	"DSC00647.JPG",
	"DSC00648.JPG"
};

//CD  航线上对应的29 张
const static char *image_name2[29] = {
	"DSC00649.JPG",
	"DSC00650.JPG",
	"DSC00651.JPG",
	"DSC00652.JPG",
	"DSC00653.JPG",
	"DSC00654.JPG",
	"DSC00655.JPG",
	"DSC00656.JPG",
	"DSC00657.JPG",
	"DSC00658.JPG",
	"DSC00659.JPG",
	"DSC00660.JPG",
	"DSC00661.JPG",
	"DSC00662.JPG",
	"DSC00663.JPG",
	"DSC00664.JPG",
	"DSC00665.JPG",
	"DSC00666.JPG",
	"DSC00667.JPG",
	"DSC00668.JPG",
	"DSC00669.JPG",
	"DSC00670.JPG",
	"DSC00671.JPG",
	"DSC00672.JPG",
	"DSC00673.JPG",
	"DSC00674.JPG",
	"DSC00675.JPG",
	"DSC00676.JPG",
	"DSC00677.JPG"

};
int main(int argc, char **argv)
{
	IMAGE_MOSAIC::Image_algorithm*  image_algorithm = new IMAGE_MOSAIC::Image_algorithm();

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
	//把图片进行缩放
	for(int i=0; i<27; i++)
	{
		string strFile = "/home/wenyi/workspace/test_photo/";
		strFile += string(image_name[i]);

		Mat image = imread(strFile.c_str());
		Mat image_resize;

		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}

		image_algorithm->Image_resize(image, image_resize,  Size(image.cols/8, image.rows/8));

		string strName = "./resize_image/";
		strName += string(image_name[i]);

		imwrite(strName.c_str(), image_resize);
	}

	cout << "image resize ok" << endl;
#endif

#if 0

	//旋转AB 航线上的所有图片
	
	for(int i = 0; i < 27; i++)
	{
		string strFile = "/home/wenyi/workspace/opencv_test/new_project/build/resize_image/";
		strFile += string(image_name[i]);

		Mat image = imread(strFile.c_str());
		Mat image_rotate;

		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}

		image_algorithm->Image_rotate(image, image_rotate,  yaw_AB_avg - yaw_AB[i]);

		string strName = "./rotate_image/";
		strName += string(image_name[i]);

		imwrite(strName.c_str(), image_rotate);
	}

	cout << "image rotate ok" << endl;
#endif

	
#if 1
	//测试两张 图片的拼接坐标查找
	for(int i=0; i<26; i++)
	{
		
		string strFile1 = "/home/wenyi/workspace/opencv_test/new_project/build/rotate_image/";
		string strFile2 = "/home/wenyi/workspace/opencv_test/new_project/build/rotate_image/";
		strFile1 += string(image_name[i]);
		strFile2 += string(image_name[i+1]);

		Mat src_image1 = imread(strFile1.c_str());
		Mat src_image2 = imread(strFile2.c_str());

		if(src_image1.empty())
		{
			cout << "failed to load:" << strFile1 << endl;
			return -1;
		}

		if(src_image2.empty())
		{
			cout << "failed to load:" << strFile2 << endl;
			return -1;
		}

		Point2i point_test;
		image_algorithm->Image_mosaic_algorithm(src_image1, src_image2, IMAGE_MOSAIC::Image_algorithm::UP,point_test);
	
		cout << "point_test x:" << point_test.x << ", y:" << point_test.y << endl;

	}



#endif


	cout << "-----------ok-------------" << endl;

	waitKey();

	return 0;
}



