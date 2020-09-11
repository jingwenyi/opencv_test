#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <limits.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "./image_algorithm/image_algorithm.h"

using namespace std;
using namespace cv;



int main(int argc, char **argv)
{
	vector<string>  image_name;
	vector<struct IMAGE_MOSAIC::Location> gps_data;
	vector<struct IMAGE_MOSAIC::Imu_data> imu_data;
	

	string strFile = "/home/wenyi/workspace/DCIM/10000904/image_name.txt";

	ifstream f;
    f.open(strFile.c_str());

	// skip first one lines
    string s0;
    getline(f,s0);

	 while(!f.eof())
    {
		string s;
        getline(f,s);
        if(!s.empty())
        {
        	image_name.push_back(s);
        }
		
	}

	f.close();
	strFile.clear();

#if 0
	for(auto name:image_name)
	{
		cout << name << endl;
	}
#endif
	strFile = "/home/wenyi/workspace/DCIM/10000904/image_gps_imu.txt";

	f.open(strFile.c_str());
	// skip first one lines
    getline(f,s0);

	 while(!f.eof())
    {
		string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double lat;
			ss >> lat;

			double lng;
			ss >> lng;

			double alt;
			ss >> alt;

			double alt_rel;
			ss >> alt_rel;

			double alt_gps;
			ss >> alt_gps;

			double roll;
			ss >> roll;

			double pitch;
			ss >> pitch;

			double yaw;
			ss >> yaw;

			struct IMAGE_MOSAIC::Location gps(alt * 100, lat * 1.0e7, lng * 1.0e7);
			gps_data.push_back(gps);

			struct IMAGE_MOSAIC::Imu_data imu(pitch, roll, yaw);
			imu_data.push_back(imu);

        }
		
	}
	
	f.close();
	strFile.clear();
#if 0
	for(auto gps:gps_data)
	{
		cout << gps.alt << "\t" << gps.lat << "\t" << gps.lng << endl;
	}

	for(auto imu:imu_data)
	{
		cout << imu.pitch << "\t" << imu.roll << "\t" <<imu.yaw << endl;
	}

#endif

	IMAGE_MOSAIC::Image_algorithm*	image_algorithm = new IMAGE_MOSAIC::Image_algorithm();
	float plane_bearing;
	float line_distance;


#if 0
	//ԭͼ̫���ͼ�����ѹ��
	std::string dir = "./resize_image";
	if(access(dir.c_str(), 0) == -1)
	{
		std::cout << dir << " is not existing." << std::endl;
		std::cout << "now make it!" << std::endl;
		int flag = mkdir(dir.c_str(), 0777);
	
		if(flag == 0)
		{
			std::cout << "make successfully" << std::endl;
		}
		else
		{
			std::cout << "mkdir error!" << std::endl;
			return -1;
		}
	}

	for(int i=0; i<image_name.size(); i++)
	{
		//��ȡ��һ��ͼƬ
		strFile.clear();
		strFile = "/home/wenyi/workspace/DCIM/10000904/";
		strFile += image_name[i];


		Mat image = imread(strFile.c_str());

		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}

		Mat image_resize;

		
		image_algorithm->Image_resize(image, image_resize,	Size(image.cols / 2, image.rows / 2));


		strFile.clear();
		strFile = "./resize_image/";
		strFile += string(image_name[i]);

		imwrite(strFile.c_str(), image_resize);

		
	}

#endif



	

	//��gps λ�����꣬��ɻ��ĺ��ߵĺ���
	for(int i=0; i<gps_data.size() - 3; i++)
	{
		
		float bearing1 = image_algorithm->Get_bearing_cd(gps_data[i], gps_data[i + 1]);
		float bearing2 = image_algorithm->Get_bearing_cd(gps_data[i + 2], gps_data[i + 3]);

		cout << "bearing1:" << bearing1 << ",bearing2:" << bearing2 << endl;
		
		if(fabs(bearing2 - bearing1) > 150)
		{
			plane_bearing = image_algorithm->Get_bearing_cd(gps_data[0], gps_data[i + 1]);
			line_distance = image_algorithm->Get_distance(gps_data[0], gps_data[i + 1]);
			break;
		}
	}

	cout << "plane bearing:" << plane_bearing << endl;

	//ͨ����һ�ź͵ڶ���ͼƬ����

	
	//��ȡ��һ��ͼƬ
	strFile = "./resize_image/";
	strFile += image_name[0];
	
	Mat image1 = imread(strFile.c_str());
	
	if(image1.empty())
	{
		cout << "failed to load:" << strFile << endl;
		return -1;
	}

	strFile.clear();

	strFile = "./resize_image/";
	strFile += image_name[1];
	Mat image2 = imread(strFile.c_str());
	
	if(image2.empty())
	{
		cout << "failed to load:" << strFile << endl;
		return -1;
	}

	strFile.clear();

	//Ϊ�˿���ƴ�Ӱ�ͼƬ��С
	float narrow_size = 2.0f;
	Mat image1_resize, image2_resize;
	
	image_algorithm->Image_resize(image1, image1_resize,	Size(image1.cols / narrow_size, image1.rows / narrow_size));
	image_algorithm->Image_resize(image2, image2_resize,	Size(image2.cols / narrow_size, image2.rows / narrow_size));

	//�������ƴ�ӵĴ���λ��
	
	Point2i point_test;
	image_algorithm->Image_fast_mosaic_algorithm(image1_resize, image2_resize,point_test);
	
	point_test.x *= narrow_size;
	point_test.y *= narrow_size;

	int flagx, flagy;

	flagx = point_test.x > 0 ? true:false;
	flagy = point_test.y > 0 ? true:false;

	cout << "point test x:" << point_test.x << ", y:" << point_test.y << endl;

	//����x < 0, �ڶ���ͼƬ����ڵ�һ��ͼƬ����
	//����y < 0, �ڶ���ͼƬ����ڵ�һ��ͼƬ����
	
	//ͨ��ƴ��λ���������ͼ���������ص�ľ���
	float image_center_distance = sqrt(pow(point_test.x, 2)  + pow(point_test.y, 2));
	float gps_center_distance = image_algorithm->Get_distance(gps_data[0], gps_data[1]);

	//���ͼ�ı�����
	float scale;
		
	scale = gps_center_distance / image_center_distance;

	cout << "scale :" << scale << endl;


	Point2i map_size;
	map_size.y = line_distance / scale;
	map_size.y += 3000;

	cout << "map size y:" << map_size.y << endl;
	
	//���������ͼ�ռ�
	Mat map_test(24000, 20000,CV_8UC3);
	map_test.setTo(0);


	vector<Point2i> photo_on_map;

	//��һ��ͼƬ����λ��
	Point2i dest_point; 

	//ƴ�ӵ�һ��ͼƬ
	if(!flagy)
	{
		dest_point.x = 13000;
		dest_point.y = 3000;

		plane_bearing -= 180;
	}
	else
	{
		
	}



	photo_on_map.push_back(dest_point);

	cout << "copy the first image." << endl;
	image1.copyTo(map_test(Rect(dest_point.x , dest_point.y, image1.cols, image1.rows)));

	//ͨ����һ��ͼƬ��λ�ü���map (0,0) ��gps����
	struct IMAGE_MOSAIC::Location map_origin;


	float diff_x = (float)dest_point.x + (float)image1.cols / 2.0f;
	float diff_y = (float)dest_point.y + (float)image1.rows / 2.0f;

	float origin_first_image_distance = sqrt(pow(diff_x, 2) + pow(diff_y, 2)) * scale;

	float tmp_bearing = plane_bearing - atan2(diff_x, diff_y) * (180.0f / M_PI);
	
	map_origin.alt = gps_data[0].alt;
	map_origin.lat = gps_data[0].lat;
	map_origin.lng = gps_data[0].lng;

	image_algorithm->Location_update(map_origin, tmp_bearing, origin_first_image_distance);

	//���ڶ���ͼƬ
	do
	{
		float distance = image_algorithm->Get_distance(map_origin, gps_data[1]) / scale;
		float bearing = image_algorithm->Get_bearing_cd(map_origin, gps_data[1]);

		cout << "distance:" << distance << ",bearing:" << bearing << endl;
		
		// ��ڶ���ͼƬ��ԭ������
		Point2i image_point;
		image_point.x = (int)(distance * sin((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image2.cols / 2);
		image_point.y = (int)(distance * cos((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image2.rows / 2);

#if 0
		//�ں�λ������
		float width_y, width_x;
		int sample1_start_rows, sample1_end_rows, sample1_start_cols, sample1_end_cols;
		int sample2_start_rows, sample2_end_rows, sample2_start_cols, sample2_end_cols;

		width_y = image2.rows - abs(image_point.y - photo_on_map[0].y);
		width_x = image2.cols - abs(image_point.x - photo_on_map[0].x);

		int w_y = 1500;
		int w_x = 2000;
		if(w_y > width_y / 2)
		{
			w_y = width_y / 2 - 10;
		}

		if(w_x > width_x / 2)
		{
			w_x = width_x / 2 - 10;
		}
		
		if(photo_on_map[0].y < image_point.y)
		{
			sample1_start_rows = width_y / 2  - w_y;
			sample1_end_rows = width_y / 2 + w_y;

			sample2_start_rows = abs(image_point.y - photo_on_map[0].y) + width_y / 2  - w_y;
			sample2_end_rows = abs(image_point.y - photo_on_map[0].y) + width_y / 2 + w_y;
		}
		else
		{
			sample1_start_rows = abs(image_point.y - photo_on_map[0].y) + width_y / 2  - w_y;
			sample1_end_rows = abs(image_point.y - photo_on_map[0].y) + width_y / 2 + w_y;

			sample2_start_rows = width_y / 2  - w_y;
			sample2_end_rows = width_y / 2 + w_y;
		}


		if(photo_on_map[0].x < image_point.x)
		{
			sample1_start_cols = width_x / 2 - w_x;
			sample1_end_cols = width_x / 2 + w_x;

			sample2_start_cols = abs(image_point.x - photo_on_map[0].x) + width_x / 2 - w_x;
			sample2_end_cols = abs(image_point.x - photo_on_map[0].x) + width_x / 2 + w_x;
		}
		else
		{
			sample1_start_cols = abs(image_point.x - photo_on_map[0].x) + width_x / 2 - w_x;
			sample1_end_cols = abs(image_point.x - photo_on_map[0].x) + width_x / 2 + w_x;
			
			sample2_start_cols = width_x / 2 - w_x;
			sample2_end_cols = width_x / 2 + w_x;
		}


		Mat sample1_image = image2(cv::Range(sample1_start_rows, sample1_end_rows),
													cv::Range(sample1_start_cols, sample1_end_cols));
		Mat sample2_image = image1(cv::Range(sample2_start_rows, sample2_end_rows),
													cv::Range(sample2_start_cols, sample2_end_cols));

		//��ͼƬ����ģ�´���
		Mat blur_image1, blur_image2;
#if 0
		//��ֵ�˲�
		blur(sample1_image, blur_image1, Size(3, 3));
		blur(sample2_image, blur_image2, Size(3, 3));
#endif

#if 1
		//��˹�˲�
		GaussianBlur(sample1_image, blur_image1, Size(3,3),11,11);
		GaussianBlur(sample2_image, blur_image2, Size(3,3),11,11);
#endif

#if 0
		//��ֵ�˲�
		medianBlur(sample1_image, blur_image1,3);
		medianBlur(sample2_image, blur_image2,3);
#endif

#if 0
		//˫���˲�
		bilateralFilter(sample1_image, blur_image1,15,100,3);
		bilateralFilter(sample2_image, blur_image2,15,100,3);
#endif


		Point2i sample_diff;
		image_algorithm->Image_fast_mosaic_algorithm2(blur_image2, blur_image1, sample_diff);
		
		cout << "------smaple diff x:" << sample_diff.x << ", y:" << sample_diff.y << endl;
		
		image_point.y -= sample_diff.y;
		image_point.x += sample_diff.x;
#endif
		photo_on_map.push_back(image_point);

		cout << "x:" << image_point.x << ", y:" <<image_point.y << endl;

		image2.copyTo(map_test(Rect(image_point.x, image_point.y, image2.cols, image2.rows)));

	}while(0);


#if 1

	Mat image_last = image2.clone();

	for(int i=2; i<image_name.size(); i++)
	{
		//��ȡͼƬ
		string strFile = "./resize_image/";
		strFile += image_name[i];

		Mat image = imread(strFile.c_str());
		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}

		//���ݵ�ͼԭ��gps ���꣬�����ͼƬ������λ��

		float distance = image_algorithm->Get_distance(map_origin, gps_data[i]) / scale;
		float bearing = image_algorithm->Get_bearing_cd(map_origin, gps_data[i]);

		cout << "i:" << i << ",distance:" << distance << ", bearing:" << bearing << endl;

		Point2i image_point;
		image_point.x = (int)(distance * sin((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.cols / 2);
		image_point.y = (int)(distance * cos((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.rows / 2);

		cout << "photo point x: " << image_point.x << ", y:" << image_point.y << endl;


		
#if 0
		//�ں�λ������
		float width_y, width_x;
		int sample1_start_rows, sample1_end_rows, sample1_start_cols, sample1_end_cols;
		int sample2_start_rows, sample2_end_rows, sample2_start_cols, sample2_end_cols;

		width_y = image2.rows - abs(image_point.y - photo_on_map[i - 1].y);
		width_x = image2.cols - abs(image_point.x - photo_on_map[i - 1].x);

		int w_y = 1000;
		int w_x = 2000;
		if(w_y > width_y / 2)
		{
			w_y = width_y / 2 - 10;
		}

		if(w_x > width_x / 2)
		{
			w_x = width_x / 2 - 10;
		}
		
		if(photo_on_map[i - 1].y < image_point.y)
		{
			sample1_start_rows = width_y / 2  - w_y;
			sample1_end_rows = width_y / 2 + w_y;

			sample2_start_rows = abs(image_point.y - photo_on_map[i - 1].y) + width_y / 2  - w_y;
			sample2_end_rows = abs(image_point.y - photo_on_map[i - 1].y) + width_y / 2 + w_y;
		}
		else
		{
			sample1_start_rows = abs(image_point.y - photo_on_map[i - 1].y) + width_y / 2  - w_y;
			sample1_end_rows = abs(image_point.y - photo_on_map[i - 1].y) + width_y / 2 + w_y;

			sample2_start_rows = width_y / 2  - w_y;
			sample2_end_rows = width_y / 2 + w_y;
		}


		if(photo_on_map[i - 1].x < image_point.x)
		{
			sample1_start_cols = width_x / 2 - w_x;
			sample1_end_cols = width_x / 2 + w_x;

			sample2_start_cols = abs(image_point.x - photo_on_map[i - 1].x) + width_x / 2 - w_x;
			sample2_end_cols = abs(image_point.x - photo_on_map[i - 1].x) + width_x / 2 + w_x;
		}
		else
		{
			sample1_start_cols = abs(image_point.x - photo_on_map[i - 1].x) + width_x / 2 - w_x;
			sample1_end_cols = abs(image_point.x - photo_on_map[i - 1].x) + width_x / 2 + w_x;
			
			sample2_start_cols = width_x / 2 - w_x;
			sample2_end_cols = width_x / 2 + w_x;
		}


		Mat sample1_image = image(cv::Range(sample1_start_rows, sample1_end_rows),
													cv::Range(sample1_start_cols, sample1_end_cols));

		
		
		Mat sample2_image = image_last(cv::Range(sample2_start_rows, sample2_end_rows),
													cv::Range(sample2_start_cols, sample2_end_cols));

		//��ͼƬ����ģ�´���
		Mat blur_image1, blur_image2;

#if 0
		//��ֵ�˲�
		blur(sample1_image, blur_image1, Size(3, 3));
		blur(sample2_image, blur_image2, Size(3, 3));
#endif

#if 1
		//��˹�˲�
		GaussianBlur(sample1_image, blur_image1, Size(3,3),11,11);
		GaussianBlur(sample2_image, blur_image2, Size(3,3),11,11);
#endif

#if 0
		//��ֵ�˲�
		medianBlur(sample1_image, blur_image1,3);
		medianBlur(sample2_image, blur_image2,3);
#endif

#if 0
		//˫���˲�
		bilateralFilter(sample1_image, blur_image1,15,100,3);
		bilateralFilter(sample2_image, blur_image2,15,100,3);
#endif

		Point2i sample_diff;
		image_algorithm->Image_fast_mosaic_algorithm2(blur_image2, blur_image1, sample_diff);
		
		cout << "------smaple diff x:" << sample_diff.x << ", y:" << sample_diff.y << endl;
		
		image_point.y -= sample_diff.y;
		image_point.x += sample_diff.x;
#endif


		photo_on_map.push_back(image_point);


		image.copyTo(map_test(Rect(image_point.x, image_point.y, image2.cols, image2.rows)));


		image_last.release();
		image_last = image.clone();
	}
#endif

	cout << "save map, please Wait a few minutes ..." << endl;;

	imwrite("map.jpg", map_test);


	waitKey();
	cout << "I am ok" << endl;
	return 0;
}
