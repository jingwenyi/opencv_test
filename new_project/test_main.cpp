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




//AB ��������27 ��ͼƬ����Ӧ27 ��ƫ����
const static float yaw_AB[] ={	92.35f,	91.36f, 87.14f, 91.96f, 85.27f, 94.03f, 90.73f, 95.61f, 95.35f,
							  	94.07f, 99.07f, 88.51f, 89.47f, 90.03f, 90.48f, 87.24f, 88.03f, 91.4f,
							  	93.31f, 91.7f, 	92.13f, 93.33f, 92.82f, 92.64f, 92.29f, 90.83f, 96.42f}; 
//CD ��������29 ��ͼƬ����Ӧ29 ��ƫ����
const static float yaw_CD[] = {	281.47f,	287.41f,	282.65f,	290.22f,	284.3f,		283.5f,		285.75f,	288.84f,	290.7f,		284.89f,
								284.42f,	288.77f,	280.14f,	285.65f,	285.61f,	287.46f,	286.84f,	286.02f,	276.95f,	285.61f,
								280.0f,		278.9f,		283.64f,	281.82f,	285.08f,	283.47f,	289.31f,	279.25f,	283.06f};



// AB  �����϶�Ӧ��27 ��ͼƬ����
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

//CD  �����϶�Ӧ��29 ��
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

	//����û��AB ��CD ���ߵķ������ݣ�������������Ƭƫ���ǵ�ƽ�������������ߵķ�������
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

	//Ϊ��ƴ���������ߵ�ͼƬ����������������Ӧ��ƽ��
	//������������߲�ƽ�У�Ҳ�����ǹ̶�����е�ʱ��
	//���ڷ�����Ӱ�쵼��

	yaw_AB_avg = (yaw_AB_avg + (yaw_CD_avg - 180)) / 2;
	yaw_CD_avg = yaw_AB_avg + 180;

	cout << "new yaw AB avg:" << yaw_AB_avg <<", new yaw CD avg:" << yaw_CD_avg << endl;

	
#if 0
	//��ͼƬ��������
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

	// CD ���ߵ�ͼƬ��Ҫ��ת180 �ȣ� ���ܸ�AB ƴ��
	for(int i=0; i<29; i++)
	{
		string strFile = "/home/wenyi/workspace/test_photo/";
		strFile += string(image_name2[i]);

		Mat image = imread(strFile.c_str());
		Mat image_resize;

		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}

		image_algorithm->Image_resize(image, image_resize,  Size(image.cols/8, image.rows/8));

		Mat image_tmp;
		image_algorithm->Image_rotate(image_resize, image_tmp, 180);

		string strName = "./resize_image/";
		strName += string(image_name2[i]);

		imwrite(strName.c_str(), image_tmp);
	}

	cout << "image resize ok" << endl;
#endif



#if 0

	//��תAB �����ϵ�����ͼƬ
	
	for(int i=0; i<27; i++)
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

	//��תCD  �����ϵ�����ͼƬ
	//����CD  �����ϵ�ͼƬ�Ѿ���ת��180  ��
	//����Ƕȵ�ʱ����Ҫ��ȥ180 ��
	for(int i=0; i<29; i++)
	{
		string strFile = "/home/wenyi/workspace/opencv_test/new_project/build/resize_image/";
		strFile += string(image_name2[i]);

		Mat image = imread(strFile.c_str());
		Mat image_rotate;

		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}

		image_algorithm->Image_rotate(image, image_rotate,  yaw_AB_avg - (yaw_CD[i] - 180));

		string strName = "./rotate_image/";
		strName += string(image_name2[i]);

		imwrite(strName.c_str(), image_rotate);
	}

	cout << "image rotate ok" << endl;
#endif


	

#if 1
	//Ϊ��ͼ����һ�Ż���
	//ͼƬ������994 X 663, ������С����Ϊ5000 x 4000
	Mat map_test(5000, 4000,CV_8UC3);
	map_test.setTo(0);

	Point2i last_image_vertex;

	//�������� ͼƬ��ƴ���������
	for(int i=0; i<26; i++)
	{
		
		string strFile1 = "/home/wenyi/workspace/opencv_test/new_project/build/rotate_image/";
		string strFile2 = "/home/wenyi/workspace/opencv_test/new_project/build/rotate_image/";
		strFile1 += string(image_name[i]);
		strFile2 += string(image_name[i+1]);

		Mat src_image1 = imread(strFile1.c_str());

		if(src_image1.empty())
		{
			cout << "failed to load:" << strFile1 << endl;
			return -1;
		}

		Mat src_image2 = imread(strFile2.c_str());

		if(src_image2.empty())
		{
			cout << "failed to load:" << strFile2 << endl;
			return -1;
		}


		
		Mat src_image1_tmp;
		if(i > 0)
		{
			src_image1_tmp = map_test(Range(last_image_vertex.y, last_image_vertex.y + src_image2.rows), Range(last_image_vertex.x, last_image_vertex.x + src_image2.cols));
			if(src_image1_tmp.empty())
			{
				cout << "failed to load:" << strFile2 << endl;
				return -1;
			}
		}
		else
		{
			src_image1_tmp = src_image1;
		}

		

		Point2i point_test;
		image_algorithm->Image_mosaic_algorithm(src_image1, src_image2, IMAGE_MOSAIC::Image_algorithm::UP,point_test);
	
		cout << "point_test x:" << point_test.x << ", y:" << point_test.y << endl;

#if 1
		//������ͼƬ����ƴ��

		Mat dest_image;
		Point2i image1_vertex, image2_vertex;
		image_algorithm->Image_optimize_seam(src_image1_tmp, src_image2, dest_image, point_test,
										IMAGE_MOSAIC::Image_algorithm::UP, image1_vertex, image2_vertex);

		static int num_image = 0;
		stringstream ss1;
		string s1;
		string strName1 = "./dest_image/";
		ss1 << num_image;
		ss1 >> s1;
		num_image++;
		strName1 += s1;
		strName1 += ".jpg";


		cout << "strName1" << strName1 << endl;
		
		imwrite(strName1.c_str(), dest_image);

#if 1
		//��ƴ�ӵ�ͼƬ����������
		//������Ҫ�����ά��ͼƬ�Ķ��������ڻ�����λ��

		if(i == 0)
		{
			last_image_vertex.x = map_test.cols / 2 - src_image1.cols / 2;
			last_image_vertex.y = map_test.rows - src_image1.rows;
			//�ѵ�һ��ͼƬ��������ͼ��
			src_image1.copyTo(map_test(Rect(last_image_vertex.x , last_image_vertex.y, src_image1.cols, src_image1.rows)));
		}

		//ͨ�������ĵ�һ��ͼƬ��dest_image  �е�λ�ã�����dest ��map_test �еĿ�ʼ����
		Point2i dest_image_vertex;
		dest_image_vertex.x = last_image_vertex.x - image1_vertex.x;
		dest_image_vertex.y = last_image_vertex.y - image1_vertex.y;

		//�ü���dest_image ͼƬ�±ߵ�1/6
		Mat tmp_image;
		image_algorithm->Image_cut(dest_image, tmp_image, IMAGE_MOSAIC::Image_algorithm::DOWN, src_image1.rows/6);

		//���� dest_image ��map_test ��
		tmp_image.copyTo(map_test(Rect(dest_image_vertex.x, dest_image_vertex.y, tmp_image.cols, tmp_image.rows)));

		//����ڶ���ͼƬ��map_test �е�����
		last_image_vertex.x = dest_image_vertex.x + image2_vertex.x;
		last_image_vertex.y = dest_image_vertex.y + image2_vertex.y;
#endif


		
#endif

	}



#endif

	cout << "the first fly line is ok" << endl;

#if 1
	//



#endif

	imwrite("map.jpg", map_test);


	cout << "-----------ok-------------" << endl;

	waitKey();

	return 0;
}



