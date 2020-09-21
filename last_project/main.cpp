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

#define X_WIDTH		1500
#define Y_WIDTH		1000

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
	//原图太大对图像进行压缩
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
		//读取第一张图片
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



	

	//用gps 位置坐标，求飞机的航线的航向
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

	//通过第一张和第二张图片计算

	
	//读取第一张图片
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

	//为了快速拼接把图片缩小
	float narrow_size = 2.0f;
	Mat image1_resize, image2_resize;
	
	image_algorithm->Image_resize(image1, image1_resize,	Size(image1.cols / narrow_size, image1.rows / narrow_size));
	image_algorithm->Image_resize(image2, image2_resize,	Size(image2.cols / narrow_size, image2.rows / narrow_size));

	//快速求出拼接的大致位置

	//获取第一张图片的下边 1/2  和第二张图的上边1/2 进行比较
	Mat image1_down, image2_up;
	image1_down = image1_resize(cv::Range(image1_resize.rows / 2, image1_resize.rows),
													cv::Range(0, image1_resize.cols));

	image2_up = image2_resize(cv::Range(0, image2_resize.rows / 2),
													cv::Range(0, image2_resize.cols));

	//对图片进行模仿处理
	Mat image1_blur, image2_blur;
	//双边滤波
	bilateralFilter(image1_down, image1_blur,15,100,3);
	bilateralFilter(image2_up, image2_blur,15,100,3);
	
	Point2i point_test;
	image_algorithm->Image_fast_mosaic_algorithm(image1_blur, image2_blur,point_test);

	point_test.y -= image1_resize.rows / 2;
	
	point_test.x *= narrow_size;
	point_test.y *= narrow_size;


	int flagx, flagy;

	flagx = point_test.x > 0 ? true:false;
	flagy = point_test.y > 0 ? true:false;

	cout << "point test x:" << point_test.x << ", y:" << point_test.y << endl;


	//由于x < 0, 第二张图片相对于第一张图片左移
	//由于y < 0, 第二张图片相对于第一张图片下移
	
	//通过拼接位置求出两个图像中心像素点的距离
	float image_center_distance = sqrt(pow(point_test.x, 2)  + pow(point_test.y, 2));
	float gps_center_distance = image_algorithm->Get_distance(gps_data[0], gps_data[1]);

	//求地图的比例尺
	float scale;
		
	scale = gps_center_distance / image_center_distance;

	cout << "scale :" << scale << endl;


	Point2i map_size;
	map_size.y = line_distance / scale;
	map_size.y += 3000;

	cout << "map size y:" << map_size.y << endl;
	
	//测试申请地图空间
	Mat map_test(16000, 16000,CV_8UC3);
	map_test.setTo(0);


	vector<vector<Point2i> > photo_on_map(11);

	//第一张图片贴的位置
	Point2i dest_point; 

	//拼接第一张图片
	if(!flagy)
	{
		dest_point.x = 11000;
		dest_point.y = 1500;

		plane_bearing -= 180;
	}
	else
	{
		
	}



	photo_on_map[0].push_back(dest_point);

	cout << "copy the first image." << endl;
	image1.copyTo(map_test(Rect(dest_point.x , dest_point.y, image1.cols, image1.rows)));

	//通过第一张图片的位置计算map (0,0) 的gps坐标
	struct IMAGE_MOSAIC::Location map_origin;


	float diff_x = (float)dest_point.x + (float)image1.cols / 2.0f;
	float diff_y = (float)dest_point.y + (float)image1.rows / 2.0f;

	float origin_first_image_distance = sqrt(pow(diff_x, 2) + pow(diff_y, 2)) * scale;

	float tmp_bearing = plane_bearing - atan2(diff_x, diff_y) * (180.0f / M_PI);
	
	map_origin.alt = gps_data[0].alt;
	map_origin.lat = gps_data[0].lat;
	map_origin.lng = gps_data[0].lng;

	image_algorithm->Location_update(map_origin, tmp_bearing, origin_first_image_distance);

	//贴第二张图片
	do
	{
		float distance = image_algorithm->Get_distance(map_origin, gps_data[1]) / scale;
		float bearing = image_algorithm->Get_bearing_cd(map_origin, gps_data[1]);

		cout << "distance:" << distance << ",bearing:" << bearing << endl;
		
		// 求第二张图片的原点坐标
		Point2i image_point;
		image_point.x = (int)(distance * sin((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image2.cols / 2);
		image_point.y = (int)(distance * cos((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image2.rows / 2);

#if 1
		//融合位置修正
		float width_y, width_x;
		int sample1_start_rows, sample1_end_rows, sample1_start_cols, sample1_end_cols;
		int sample2_start_rows, sample2_end_rows, sample2_start_cols, sample2_end_cols;

		width_y = image2.rows - abs(image_point.y - photo_on_map[0][0].y);
		width_x = image2.cols - abs(image_point.x - photo_on_map[0][0].x);

		int w_y = Y_WIDTH;
		int w_x = X_WIDTH;
		if(w_y > width_y / 2)
		{
			w_y = width_y / 2 - 10;
		}

		if(w_x > width_x / 2)
		{
			w_x = width_x / 2 - 10;
		}
		
		if(photo_on_map[0][0].y < image_point.y)
		{
			sample1_start_rows = width_y / 2  - w_y;
			sample1_end_rows = width_y / 2 + w_y;

			sample2_start_rows = abs(image_point.y - photo_on_map[0][0].y) + width_y / 2  - w_y;
			sample2_end_rows = abs(image_point.y - photo_on_map[0][0].y) + width_y / 2 + w_y;
		}
		else
		{
			sample1_start_rows = abs(image_point.y - photo_on_map[0][0].y) + width_y / 2  - w_y;
			sample1_end_rows = abs(image_point.y - photo_on_map[0][0].y) + width_y / 2 + w_y;

			sample2_start_rows = width_y / 2  - w_y;
			sample2_end_rows = width_y / 2 + w_y;
		}


		if(photo_on_map[0][0].x < image_point.x)
		{
			sample1_start_cols = width_x / 2 - w_x;
			sample1_end_cols = width_x / 2 + w_x;

			sample2_start_cols = abs(image_point.x - photo_on_map[0][0].x) + width_x / 2 - w_x;
			sample2_end_cols = abs(image_point.x - photo_on_map[0][0].x) + width_x / 2 + w_x;
		}
		else
		{
			sample1_start_cols = abs(image_point.x - photo_on_map[0][0].x) + width_x / 2 - w_x;
			sample1_end_cols = abs(image_point.x - photo_on_map[0][0].x) + width_x / 2 + w_x;
			
			sample2_start_cols = width_x / 2 - w_x;
			sample2_end_cols = width_x / 2 + w_x;
		}


		Mat sample1_image = image2(cv::Range(sample1_start_rows, sample1_end_rows),
													cv::Range(sample1_start_cols, sample1_end_cols));
		Mat sample2_image = image1(cv::Range(sample2_start_rows, sample2_end_rows),
													cv::Range(sample2_start_cols, sample2_end_cols));

		//为了提高速度，先对图像进行缩小
		Mat sample1_image_narrow, sample2_image_narrow;

		image_algorithm->Image_resize(sample1_image, sample1_image_narrow,	Size(sample1_image.cols / 2, sample1_image.rows / 2));
		image_algorithm->Image_resize(sample2_image, sample2_image_narrow,	Size(sample2_image.cols / 2, sample2_image.rows / 2));

		//对图片进行模仿处理
		Mat blur_image1, blur_image2;

		//双边滤波
		bilateralFilter(sample1_image_narrow, blur_image1,15,100,3);
		bilateralFilter(sample2_image_narrow, blur_image2,15,100,3);

		Point2i sample_diff;
		image_algorithm->Image_fast_mosaic_algorithm(blur_image2, blur_image1, sample_diff);

		
		cout << "------smaple diff x:" << sample_diff.x << ", y:" << sample_diff.y << endl;
		
		image_point.y -= sample_diff.y * 2;
		image_point.x += sample_diff.x * 2;
#endif
		photo_on_map[0].push_back(image_point);

		cout << "x:" << image_point.x << ", y:" <<image_point.y << endl;
		//截掉上边1/4
		Mat dest_image = image2(cv::Range(image2.rows / 4, image2.rows),
															cv::Range(0, image2.cols));
		image_point.y += image2.rows / 4;
		
		dest_image.copyTo(map_test(Rect(image_point.x, image_point.y, dest_image.cols, dest_image.rows)));

	}while(0);





#if 1


	//第一条航线 9 张图片
	Mat image_last = image2.clone();

	for(int i=2; i<11; i++)
	{
		//读取图片
		string strFile = "./resize_image/";
		strFile += image_name[i];

		Mat image = imread(strFile.c_str());
		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}

		//根据地图原点gps 坐标，计算该图片的坐标位置

		float distance = image_algorithm->Get_distance(map_origin, gps_data[i]) / scale;
		float bearing = image_algorithm->Get_bearing_cd(map_origin, gps_data[i]);

		cout << "i:" << i << ",distance:" << distance << ", bearing:" << bearing << endl;

		Point2i image_point;
		image_point.x = (int)(distance * sin((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.cols / 2);
		image_point.y = (int)(distance * cos((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.rows / 2);

		cout << "photo point x: " << image_point.x << ", y:" << image_point.y << endl;


		
#if 1
		//融合位置修正
		float width_y, width_x;
		int sample1_start_rows, sample1_end_rows, sample1_start_cols, sample1_end_cols;
		int sample2_start_rows, sample2_end_rows, sample2_start_cols, sample2_end_cols;

		width_y = image2.rows - abs(image_point.y - photo_on_map[0][i - 1].y);
		width_x = image2.cols - abs(image_point.x - photo_on_map[0][i - 1].x);

		int w_y = Y_WIDTH;
		int w_x = X_WIDTH;
		if(w_y > width_y / 2)
		{
			w_y = width_y / 2 - 10;
		}

		if(w_x > width_x / 2)
		{
			w_x = width_x / 2 - 10;
		}
		
		if(photo_on_map[0][i - 1].y < image_point.y)
		{
			sample1_start_rows = width_y / 2  - w_y;
			sample1_end_rows = width_y / 2 + w_y;

			sample2_start_rows = abs(image_point.y - photo_on_map[0][i - 1].y) + width_y / 2  - w_y;
			sample2_end_rows = abs(image_point.y - photo_on_map[0][i - 1].y) + width_y / 2 + w_y;
		}
		else
		{
			sample1_start_rows = abs(image_point.y - photo_on_map[0][i - 1].y) + width_y / 2  - w_y;
			sample1_end_rows = abs(image_point.y - photo_on_map[0][i - 1].y) + width_y / 2 + w_y;

			sample2_start_rows = width_y / 2  - w_y;
			sample2_end_rows = width_y / 2 + w_y;
		}


		if(photo_on_map[0][i - 1].x < image_point.x)
		{
			sample1_start_cols = width_x / 2 - w_x;
			sample1_end_cols = width_x / 2 + w_x;

			sample2_start_cols = abs(image_point.x - photo_on_map[0][i - 1].x) + width_x / 2 - w_x;
			sample2_end_cols = abs(image_point.x - photo_on_map[0][i - 1].x) + width_x / 2 + w_x;
		}
		else
		{
			sample1_start_cols = abs(image_point.x - photo_on_map[0][i - 1].x) + width_x / 2 - w_x;
			sample1_end_cols = abs(image_point.x - photo_on_map[0][i - 1].x) + width_x / 2 + w_x;
			
			sample2_start_cols = width_x / 2 - w_x;
			sample2_end_cols = width_x / 2 + w_x;
		}


		Mat sample1_image = image(cv::Range(sample1_start_rows, sample1_end_rows),
													cv::Range(sample1_start_cols, sample1_end_cols));

		
		
		Mat sample2_image = image_last(cv::Range(sample2_start_rows, sample2_end_rows),
													cv::Range(sample2_start_cols, sample2_end_cols));

		//为了提高速度，先对图像进行缩小
		Mat sample1_image_narrow, sample2_image_narrow;

		image_algorithm->Image_resize(sample1_image, sample1_image_narrow,	Size(sample1_image.cols / 2, sample1_image.rows / 2));
		image_algorithm->Image_resize(sample2_image, sample2_image_narrow,	Size(sample2_image.cols / 2, sample2_image.rows / 2));
		

		//对图片进行模仿处理
		Mat blur_image1, blur_image2;

		//双边滤波
		bilateralFilter(sample1_image_narrow, blur_image1,15,100,3);
		bilateralFilter(sample2_image_narrow, blur_image2,15,100,3);

		Point2i sample_diff;
		image_algorithm->Image_fast_mosaic_algorithm(blur_image2, blur_image1, sample_diff);

		
		
		cout << "------smaple diff x:" << sample_diff.x << ", y:" << sample_diff.y << endl;
		
		image_point.y -= sample_diff.y * 2;
		image_point.x += sample_diff.x * 2;
#endif


		photo_on_map[0].push_back(image_point);


		//截掉上边1/4
		Mat dest_image = image(cv::Range(image.rows / 4, image.rows),
															cv::Range(0, image.cols));
		image_point.y += image.rows / 4;


		dest_image.copyTo(map_test(Rect(image_point.x, image_point.y, dest_image.cols, dest_image.rows)));


		image_last.release();
		image_last = image.clone();
	}
#endif


	cout << "first line is ok--------------------" << endl;
	


#if 1
	int number = 0;
	int _last_line_number = 0;
	int _last_line = 1;
	int _current_line = 2;

	int _last_line_number_size = photo_on_map[_last_line - 1].size();

	//第二条8张图片
	for(int i=14; i<22; i++)
	{
		//读取图片
		string strFile = "./resize_image/";
		strFile += image_name[i];

		Mat image = imread(strFile.c_str());
		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}

		//根据地图原点gps 坐标，计算该图片的坐标位置

		float distance = image_algorithm->Get_distance(map_origin, gps_data[i]) / scale;
		float bearing = image_algorithm->Get_bearing_cd(map_origin, gps_data[i]);

		cout << "i:" << i << ",distance:" << distance << ", bearing:" << bearing << endl;

		Point2i image_point;
		image_point.x = (int)(distance * sin((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.cols / 2);
		image_point.y = (int)(distance * cos((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.rows / 2);

		cout << "photo point x: " << image_point.x << ", y:" << image_point.y << endl;

#if 1
		//融合位置修正
		float width_y, width_x;
		int sample1_start_rows, sample1_end_rows, sample1_start_cols, sample1_end_cols;
		int sample2_start_rows, sample2_end_rows, sample2_start_cols, sample2_end_cols;
		
		Point2i last_photo_coordinate;
		
		if(number == 0)
		{
			last_photo_coordinate.x = photo_on_map[_last_line - 1][_last_line_number_size - 1].x;
			last_photo_coordinate.y = photo_on_map[_last_line - 1][_last_line_number_size - 1].y;
		}
		else
		{
			last_photo_coordinate.x = photo_on_map[_current_line - 1][number - 1].x;
			last_photo_coordinate.y = photo_on_map[_current_line - 1][number - 1].y;
		}
				
		width_y = image.rows - abs(image_point.y - last_photo_coordinate.y);
		width_x = image.cols - abs(image_point.x - last_photo_coordinate.x);
				
		int w_y = Y_WIDTH;
		int w_x = X_WIDTH;
		if(w_y > width_y / 2)
		{
			w_y = width_y / 2 - 10;
		}
				
		if(w_x > width_x / 2)
		{
			w_x = width_x / 2 - 10;
		}
						
		if(last_photo_coordinate.y < image_point.y)
		{
			sample1_start_rows = width_y / 2  - w_y;
			sample1_end_rows = width_y / 2 + w_y;
				
			sample2_start_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2  - w_y;
			sample2_end_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2 + w_y;
		}
		else
		{
			sample1_start_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2  - w_y;
			sample1_end_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2 + w_y;
				
			sample2_start_rows = width_y / 2  - w_y;
			sample2_end_rows = width_y / 2 + w_y;
		}
				
				
		if(last_photo_coordinate.x < image_point.x)
		{
			sample1_start_cols = width_x / 2 - w_x;
			sample1_end_cols = width_x / 2 + w_x;
				
			sample2_start_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 - w_x;
			sample2_end_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 + w_x;
		}
		else
		{
			sample1_start_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 - w_x;
			sample1_end_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 + w_x;
							
			sample2_start_cols = width_x / 2 - w_x;
			sample2_end_cols = width_x / 2 + w_x;
		}
				
				
				
		Mat sample1_image = image(cv::Range(sample1_start_rows, sample1_end_rows),
												cv::Range(sample1_start_cols, sample1_end_cols));
				
						
						
		Mat sample2_image = image_last(cv::Range(sample2_start_rows, sample2_end_rows),
												cv::Range(sample2_start_cols, sample2_end_cols));

		//为了提高速度，先对图像进行缩小
		Mat sample1_image_narrow, sample2_image_narrow;

		image_algorithm->Image_resize(sample1_image, sample1_image_narrow,	Size(sample1_image.cols / 2, sample1_image.rows / 2));
		image_algorithm->Image_resize(sample2_image, sample2_image_narrow,	Size(sample2_image.cols / 2, sample2_image.rows / 2));
				
		//对图片进行模糊处理
		Mat blur_image1, blur_image2;
				
		//双边滤波
		bilateralFilter(sample1_image_narrow, blur_image1,15,100,3);
		bilateralFilter(sample2_image_narrow, blur_image2,15,100,3);
				
		Point2i sample_diff;
		
		image_algorithm->Image_fast_mosaic_algorithm(blur_image2, blur_image1, sample_diff);

		
						
		cout << "------smaple diff x:" << sample_diff.x << ", y:" << sample_diff.y << endl;
						
		image_point.y -= sample_diff.y * 2;
		image_point.x += sample_diff.x * 2;

#if 1
		//左右融合位置修正
				
		if(number != 0)
		{
			int right_photo = _last_line_number_size - photo_on_map[_current_line - 1].size();
			Point2i right_photo_coordinate;
			right_photo_coordinate.x = photo_on_map[_last_line - 1][right_photo - 1].x;
			right_photo_coordinate.y = photo_on_map[_last_line - 1][right_photo - 1].y;


			width_y = image.rows - abs(image_point.y - right_photo_coordinate.y);
			width_x = image.cols - abs(image_point.x - right_photo_coordinate.x);


			w_y = Y_WIDTH;
			w_x = X_WIDTH;
			if(w_y > width_y / 2)
			{
				w_y = width_y / 2 - 10;
			}
				
			if(w_x > width_x / 2)
			{
				w_x = width_x / 2 - 10;
			}

			if(right_photo_coordinate.y < image_point.y)
			{
				sample1_start_rows = width_y / 2  - w_y;
				sample1_end_rows = width_y / 2 + w_y;
				
				sample2_start_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2  - w_y;
				sample2_end_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2 + w_y;
			}
			else
			{
				sample1_start_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2  - w_y;
				sample1_end_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2 + w_y;
				
				sample2_start_rows = width_y / 2  - w_y;
				sample2_end_rows = width_y / 2 + w_y;
			}
				
				
			if(right_photo_coordinate.x < image_point.x)
			{
				sample1_start_cols = width_x / 2 - w_x;
				sample1_end_cols = width_x / 2 + w_x;
				
				sample2_start_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 - w_x;
				sample2_end_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 + w_x;
			}
			else
			{
				sample1_start_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 - w_x;
				sample1_end_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 + w_x;
							
				sample2_start_cols = width_x / 2 - w_x;
				sample2_end_cols = width_x / 2 + w_x;
			}


			Mat sample1_image_right = image(cv::Range(sample1_start_rows, sample1_end_rows),
												cv::Range(sample1_start_cols, sample1_end_cols));

			strFile.clear();
			strFile = "./resize_image/";
			strFile += image_name[_last_line_number + right_photo - 1];

			Mat right_image = imread(strFile.c_str());
			
			if(right_image.empty())
			{
				cout << "failed to load:" << strFile << endl;
				return -1;
			}

			Mat sample2_image_right = right_image(cv::Range(sample2_start_rows, sample2_end_rows),
												cv::Range(sample2_start_cols, sample2_end_cols));

			//为了提高速度，先对图像进行缩小
			Mat sample1_image_right_narrow, sample2_image_right_narrow;

			image_algorithm->Image_resize(sample1_image_right, sample1_image_right_narrow,	Size(sample1_image_right.cols / 2, sample1_image_right.rows / 2));
			image_algorithm->Image_resize(sample2_image_right, sample2_image_right_narrow,	Size(sample2_image_right.cols / 2, sample2_image_right.rows / 2));

			
			//对图片进行模仿处理
			Mat blur_image1_right, blur_image2_right;

			//双边滤波
			bilateralFilter(sample1_image_right_narrow, blur_image1_right,15,100,3);
			bilateralFilter(sample2_image_right_narrow, blur_image2_right,15,100,3);

			Point2i sample_diff_right;
		
			image_algorithm->Image_fast_mosaic_algorithm(blur_image2_right, blur_image1_right, sample_diff_right);

						
			cout << "++++right+++++++++++++++smaple diff x:" << sample_diff_right.x << ", y:" << sample_diff_right.y << endl;
						
			image_point.y -= sample_diff_right.y * 2;
			image_point.x += sample_diff_right.x * 2;
		}
		
#endif

#endif


		photo_on_map[_current_line - 1].push_back(image_point);
		number++;


		//截掉下边的1/4, 截掉右边的1/4
		int cut_size_up = image.rows / 4;
		int cut_size_right = image.cols / 4;

		if(number == 0)
		{
			cut_size_up = 0;
		}
		

		Mat dest_image = image(cv::Range(0, image.rows - cut_size_up),
															cv::Range(0, image.cols - cut_size_right));
		dest_image.copyTo(map_test(Rect(image_point.x, image_point.y, dest_image.cols, dest_image.rows)));


		image_last.release();
		image_last = image.clone();
	}

#endif

	cout << "the 2 line is ok----------------" << endl;


#if 1

	number = 0;
	_last_line_number = 14;
	_last_line++;
	_current_line++;
	_last_line_number_size = photo_on_map[_last_line - 1].size();
	//第三条航线8 张图片
	for(int i=25; i<33; i++)
	{
		//读取图片
		string strFile = "./resize_image/";
		strFile += image_name[i];

		Mat image = imread(strFile.c_str());
		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}

		//根据地图原点gps 坐标，计算该图片的坐标位置

		float distance = image_algorithm->Get_distance(map_origin, gps_data[i]) / scale;
		float bearing = image_algorithm->Get_bearing_cd(map_origin, gps_data[i]);

		cout << "i:" << i << ",distance:" << distance << ", bearing:" << bearing << endl;

		Point2i image_point;
		image_point.x = (int)(distance * sin((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.cols / 2);
		image_point.y = (int)(distance * cos((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.rows / 2);

		cout << "photo point x: " << image_point.x << ", y:" << image_point.y << endl;
#if 1
		//融合位置修正
		float width_y, width_x;
		int sample1_start_rows, sample1_end_rows, sample1_start_cols, sample1_end_cols;
		int sample2_start_rows, sample2_end_rows, sample2_start_cols, sample2_end_cols;
				
		Point2i last_photo_coordinate;
				
		if(number == 0)
		{
			last_photo_coordinate.x = photo_on_map[_last_line - 1][_last_line_number_size - 1].x;
			last_photo_coordinate.y = photo_on_map[_last_line - 1][_last_line_number_size - 1].y;
		}
		else
		{
			last_photo_coordinate.x = photo_on_map[_current_line - 1][number - 1].x;
			last_photo_coordinate.y = photo_on_map[_current_line - 1][number - 1].y;
		}
						
		width_y = image.rows - abs(image_point.y - last_photo_coordinate.y);
		width_x = image.cols - abs(image_point.x - last_photo_coordinate.x);
						
		int w_y = Y_WIDTH;
		int w_x = X_WIDTH;
		if(w_y > width_y / 2)
		{
			w_y = width_y / 2 - 10;
		}
						
		if(w_x > width_x / 2)
		{
			w_x = width_x / 2 - 10;
		}
								
		if(last_photo_coordinate.y < image_point.y)
		{
			sample1_start_rows = width_y / 2  - w_y;
			sample1_end_rows = width_y / 2 + w_y;
						
			sample2_start_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2  - w_y;
			sample2_end_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2 + w_y;
		}
		else
		{
			sample1_start_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2  - w_y;
			sample1_end_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2 + w_y;
						
			sample2_start_rows = width_y / 2  - w_y;
			sample2_end_rows = width_y / 2 + w_y;
		}
						
						
		if(last_photo_coordinate.x < image_point.x)
		{
			sample1_start_cols = width_x / 2 - w_x;
			sample1_end_cols = width_x / 2 + w_x;
						
			sample2_start_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 - w_x;
			sample2_end_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 + w_x;
		}
		else
		{
			sample1_start_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 - w_x;
			sample1_end_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 + w_x;
									
			sample2_start_cols = width_x / 2 - w_x;
			sample2_end_cols = width_x / 2 + w_x;
		}
						
		Mat sample1_image = image(cv::Range(sample1_start_rows, sample1_end_rows),
												cv::Range(sample1_start_cols, sample1_end_cols));
						
								
								
		Mat sample2_image = image_last(cv::Range(sample2_start_rows, sample2_end_rows),
												cv::Range(sample2_start_cols, sample2_end_cols));
		//为了提高速度，先对图像进行缩小
		Mat sample1_image_narrow, sample2_image_narrow;

		image_algorithm->Image_resize(sample1_image, sample1_image_narrow,	Size(sample1_image.cols / 2, sample1_image.rows / 2));
		image_algorithm->Image_resize(sample2_image, sample2_image_narrow,	Size(sample2_image.cols / 2, sample2_image.rows / 2));
						
		//对图片进行模糊处理
		Mat blur_image1, blur_image2;
						
		//双边滤波
		bilateralFilter(sample1_image_narrow, blur_image1,15,100,3);
		bilateralFilter(sample2_image_narrow, blur_image2,15,100,3);
						
		Point2i sample_diff;
				
		image_algorithm->Image_fast_mosaic_algorithm(blur_image2, blur_image1, sample_diff);
		
		
								
		cout << "------smaple diff x:" << sample_diff.x << ", y:" << sample_diff.y << endl;
								
		image_point.y -= sample_diff.y * 2;
		image_point.x += sample_diff.x * 2;
		
#if 1
		//左右融合位置修正
						
		if(number != 0)
		{
			int right_photo = _last_line_number_size - photo_on_map[_current_line - 1].size();
			Point2i right_photo_coordinate;
			right_photo_coordinate.x = photo_on_map[_last_line - 1][right_photo - 1].x;
			right_photo_coordinate.y = photo_on_map[_last_line - 1][right_photo - 1].y;
		
		
			width_y = image2.rows - abs(image_point.y - right_photo_coordinate.y);
			width_x = image2.cols - abs(image_point.x - right_photo_coordinate.x);
		
		
			w_y = Y_WIDTH;
			w_x = X_WIDTH;
			if(w_y > width_y / 2)
			{
				w_y = width_y / 2 - 10;
			}
						
			if(w_x > width_x / 2)
			{
				w_x = width_x / 2 - 10;
			}
		
			if(right_photo_coordinate.y < image_point.y)
			{
				sample1_start_rows = width_y / 2  - w_y;
				sample1_end_rows = width_y / 2 + w_y;
						
				sample2_start_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2  - w_y;
				sample2_end_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2 + w_y;
			}
			else
			{
				sample1_start_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2  - w_y;
				sample1_end_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2 + w_y;
						
				sample2_start_rows = width_y / 2  - w_y;
				sample2_end_rows = width_y / 2 + w_y;
			}
						
						
			if(right_photo_coordinate.x < image_point.x)
			{
				sample1_start_cols = width_x / 2 - w_x;
				sample1_end_cols = width_x / 2 + w_x;
						
				sample2_start_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 - w_x;
				sample2_end_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 + w_x;
			}
			else
			{
				sample1_start_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 - w_x;
				sample1_end_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 + w_x;
									
				sample2_start_cols = width_x / 2 - w_x;
				sample2_end_cols = width_x / 2 + w_x;
			}
		
		
			Mat sample1_image_right = image(cv::Range(sample1_start_rows, sample1_end_rows),
												cv::Range(sample1_start_cols, sample1_end_cols));
		
			strFile.clear();
			strFile = "./resize_image/";
			strFile += image_name[_last_line_number  + right_photo - 1];
		
			Mat right_image = imread(strFile.c_str());
					
			if(right_image.empty())
			{
				cout << "failed to load:" << strFile << endl;
				return -1;
			}
		
			Mat sample2_image_right = right_image(cv::Range(sample2_start_rows, sample2_end_rows),
														cv::Range(sample2_start_cols, sample2_end_cols));
			
			//为了提高速度，先对图像进行缩小
			Mat sample1_image_right_narrow, sample2_image_right_narrow;

			image_algorithm->Image_resize(sample1_image_right, sample1_image_right_narrow,	Size(sample1_image_right.cols / 2, sample1_image_right.rows / 2));
			image_algorithm->Image_resize(sample2_image_right, sample2_image_right_narrow,	Size(sample2_image_right.cols / 2, sample2_image_right.rows / 2));

			
			//对图片进行模仿处理
			Mat blur_image1_right, blur_image2_right;
		
			//双边滤波
			bilateralFilter(sample1_image_right_narrow, blur_image1_right,15,100,3);
			bilateralFilter(sample2_image_right_narrow, blur_image2_right,15,100,3);
		
			Point2i sample_diff_right;
				
			image_algorithm->Image_fast_mosaic_algorithm(blur_image2_right, blur_image1_right, sample_diff_right);
								
			cout << "++++right+++++++++++++++smaple diff x:" << sample_diff_right.x << ", y:" << sample_diff_right.y << endl;
								
			image_point.y -= sample_diff_right.y * 2;
			image_point.x += sample_diff_right.x * 2;
		}
				
#endif
		
#endif


		photo_on_map[_current_line - 1].push_back(image_point);
		number++;

		//截掉上 边的1/4, 截掉右边的1/4
		int cut_size_up = image.rows / 4;
		int cut_size_right = image.cols / 4;

		if(number == 0)
		{
			cut_size_up = 0;
		}


		image_point.y += image.rows / 4;


	

		Mat dest_image = image(cv::Range(cut_size_up, image.rows),
															cv::Range(0, image.cols - cut_size_right));


		dest_image.copyTo(map_test(Rect(image_point.x, image_point.y, dest_image.cols, dest_image.rows)));



		image_last.release();
		image_last = image.clone();
	}



#endif

	cout << "the 3 line is ok ------------"  <<endl;

#if 1
	number = 0;
	_last_line_number = 25;
	_last_line++;
	_current_line++;
	_last_line_number_size = photo_on_map[_last_line - 1].size();

	//第四 条航线8 张图片
	for(int i=36; i<44; i++)
	{
		//读取图片
		string strFile = "./resize_image/";
		strFile += image_name[i];
	
		Mat image = imread(strFile.c_str());
		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}
	
		//根据地图原点gps 坐标，计算该图片的坐标位置
	
		float distance = image_algorithm->Get_distance(map_origin, gps_data[i]) / scale;
		float bearing = image_algorithm->Get_bearing_cd(map_origin, gps_data[i]);
	
		cout << "i:" << i << ",distance:" << distance << ", bearing:" << bearing << endl;
	
		Point2i image_point;
		image_point.x = (int)(distance * sin((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.cols / 2);
		image_point.y = (int)(distance * cos((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.rows / 2);
	
		cout << "photo point x: " << image_point.x << ", y:" << image_point.y << endl;

#if 1
		//融合位置修正
		float width_y, width_x;
		int sample1_start_rows, sample1_end_rows, sample1_start_cols, sample1_end_cols;
		int sample2_start_rows, sample2_end_rows, sample2_start_cols, sample2_end_cols;
				
		Point2i last_photo_coordinate;
				
		if(number == 0)
		{
			last_photo_coordinate.x = photo_on_map[_last_line - 1][_last_line_number_size - 1].x;
			last_photo_coordinate.y = photo_on_map[_last_line - 1][_last_line_number_size - 1].y;
		}
		else
		{
			last_photo_coordinate.x = photo_on_map[_current_line - 1][number - 1].x;
			last_photo_coordinate.y = photo_on_map[_current_line - 1][number - 1].y;
		}
						
		width_y = image.rows - abs(image_point.y - last_photo_coordinate.y);
		width_x = image.cols - abs(image_point.x - last_photo_coordinate.x);
						
		int w_y = Y_WIDTH;
		int w_x = X_WIDTH;
		if(w_y > width_y / 2)
		{
			w_y = width_y / 2 - 10;
		}
						
		if(w_x > width_x / 2)
		{
			w_x = width_x / 2 - 10;
		}
								
		if(last_photo_coordinate.y < image_point.y)
		{
			sample1_start_rows = width_y / 2  - w_y;
			sample1_end_rows = width_y / 2 + w_y;
						
			sample2_start_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2  - w_y;
			sample2_end_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2 + w_y;
		}
		else
		{
			sample1_start_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2  - w_y;
			sample1_end_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2 + w_y;
						
			sample2_start_rows = width_y / 2  - w_y;
			sample2_end_rows = width_y / 2 + w_y;
		}
						
						
		if(last_photo_coordinate.x < image_point.x)
		{
			sample1_start_cols = width_x / 2 - w_x;
			sample1_end_cols = width_x / 2 + w_x;
						
			sample2_start_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 - w_x;
			sample2_end_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 + w_x;
		}
		else
		{
			sample1_start_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 - w_x;
			sample1_end_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 + w_x;
									
			sample2_start_cols = width_x / 2 - w_x;
			sample2_end_cols = width_x / 2 + w_x;
		}
						
						
						
		Mat sample1_image = image(cv::Range(sample1_start_rows, sample1_end_rows),
												cv::Range(sample1_start_cols, sample1_end_cols));
						
								
								
		Mat sample2_image = image_last(cv::Range(sample2_start_rows, sample2_end_rows),
												cv::Range(sample2_start_cols, sample2_end_cols));

		//为了提高速度，先对图像进行缩小
		Mat sample1_image_narrow, sample2_image_narrow;

		image_algorithm->Image_resize(sample1_image, sample1_image_narrow,	Size(sample1_image.cols / 2, sample1_image.rows / 2));
		image_algorithm->Image_resize(sample2_image, sample2_image_narrow,	Size(sample2_image.cols / 2, sample2_image.rows / 2));
						
		//对图片进行模糊处理
		Mat blur_image1, blur_image2;
						
		//双边滤波
		bilateralFilter(sample1_image_narrow, blur_image1,15,100,3);
		bilateralFilter(sample2_image_narrow, blur_image2,15,100,3);
						
		Point2i sample_diff;
				
		image_algorithm->Image_fast_mosaic_algorithm(blur_image2, blur_image1, sample_diff);
		
								
		cout << "------smaple diff x:" << sample_diff.x << ", y:" << sample_diff.y << endl;
								
		image_point.y -= sample_diff.y * 2;
		image_point.x += sample_diff.x * 2;
		
#if 1
		//左右融合位置修正
						
		if(number != 0)
		{
			int right_photo = _last_line_number_size - photo_on_map[_current_line - 1].size();
			Point2i right_photo_coordinate;
			right_photo_coordinate.x = photo_on_map[_last_line - 1][right_photo - 1].x;
			right_photo_coordinate.y = photo_on_map[_last_line - 1][right_photo - 1].y;
		
		
			width_y = image.rows - abs(image_point.y - right_photo_coordinate.y);
			width_x = image.cols - abs(image_point.x - right_photo_coordinate.x);
		
		
			w_y = Y_WIDTH;
			w_x = X_WIDTH;
			if(w_y > width_y / 2)
			{
				w_y = width_y / 2 - 10;
			}
						
			if(w_x > width_x / 2)
			{
				w_x = width_x / 2 - 10;
			}
		
			if(right_photo_coordinate.y < image_point.y)
			{
				sample1_start_rows = width_y / 2  - w_y;
				sample1_end_rows = width_y / 2 + w_y;
						
				sample2_start_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2  - w_y;
				sample2_end_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2 + w_y;
			}
			else
			{
				sample1_start_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2  - w_y;
				sample1_end_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2 + w_y;
						
				sample2_start_rows = width_y / 2  - w_y;
				sample2_end_rows = width_y / 2 + w_y;
			}
						
						
			if(right_photo_coordinate.x < image_point.x)
			{
				sample1_start_cols = width_x / 2 - w_x;
				sample1_end_cols = width_x / 2 + w_x;
						
				sample2_start_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 - w_x;
				sample2_end_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 + w_x;
			}
			else
			{
				sample1_start_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 - w_x;
				sample1_end_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 + w_x;
									
				sample2_start_cols = width_x / 2 - w_x;
				sample2_end_cols = width_x / 2 + w_x;
			}
		
		
			Mat sample1_image_right = image(cv::Range(sample1_start_rows, sample1_end_rows),
												cv::Range(sample1_start_cols, sample1_end_cols));
		
			strFile.clear();
			strFile = "./resize_image/";
			strFile += image_name[_last_line_number + right_photo - 1];
		
			Mat right_image = imread(strFile.c_str());
					
			if(right_image.empty())
			{
				cout << "failed to load:" << strFile << endl;
				return -1;
			}
		
			Mat sample2_image_right = right_image(cv::Range(sample2_start_rows, sample2_end_rows),
												cv::Range(sample2_start_cols, sample2_end_cols));

			//为了提高速度，先对图像进行缩小
			Mat sample1_image_right_narrow, sample2_image_right_narrow;

			image_algorithm->Image_resize(sample1_image_right, sample1_image_right_narrow,	Size(sample1_image_right.cols / 2, sample1_image_right.rows / 2));
			image_algorithm->Image_resize(sample2_image_right, sample2_image_right_narrow,	Size(sample2_image_right.cols / 2, sample2_image_right.rows / 2));
			
			//对图片进行模仿处理
			Mat blur_image1_right, blur_image2_right;
		
			//双边滤波
			bilateralFilter(sample1_image_right_narrow, blur_image1_right,15,100,3);
			bilateralFilter(sample2_image_right_narrow, blur_image2_right,15,100,3);
		
			Point2i sample_diff_right;
				
			image_algorithm->Image_fast_mosaic_algorithm(blur_image2_right, blur_image1_right, sample_diff_right);
								
			cout << "++++right+++++++++++++++smaple diff x:" << sample_diff_right.x << ", y:" << sample_diff_right.y << endl;
								
			image_point.y -= sample_diff_right.y * 2;
			image_point.x += sample_diff_right.x * 2;
		}
				
#endif
		
#endif

	
		photo_on_map[_current_line - 1].push_back(image_point);
		number++;
	
	
		//截掉下边的1/4, 截掉右边的1/4
		int cut_size_up = image.rows / 4;
		int cut_size_right = image.cols / 4;

		if(number == 0)
		{
			cut_size_up = 0;
		}
		

		Mat dest_image = image(cv::Range(0, image.rows - cut_size_up),
															cv::Range(0, image.cols - cut_size_right));
		dest_image.copyTo(map_test(Rect(image_point.x, image_point.y, dest_image.cols, dest_image.rows)));
	
	
		image_last.release();
		image_last = image.clone();
	}
	
	
	
#endif


	cout << "the 4 line is ok ------------"  <<endl;





#if 1
	number = 0;
	_last_line_number = 36;
	_last_line++;
	_current_line++;
	_last_line_number_size = photo_on_map[_last_line - 1].size();

	//第五 条航线8 张图片
	for(int i=47; i<55; i++)
	{
		//读取图片
		string strFile = "./resize_image/";
		strFile += image_name[i];
		
		Mat image = imread(strFile.c_str());
		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}
		
		//根据地图原点gps 坐标，计算该图片的坐标位置
		
		float distance = image_algorithm->Get_distance(map_origin, gps_data[i]) / scale;
		float bearing = image_algorithm->Get_bearing_cd(map_origin, gps_data[i]);
		
		cout << "i:" << i << ",distance:" << distance << ", bearing:" << bearing << endl;
		
		Point2i image_point;
		image_point.x = (int)(distance * sin((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.cols / 2);
		image_point.y = (int)(distance * cos((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.rows / 2);
		
		cout << "photo point x: " << image_point.x << ", y:" << image_point.y << endl;

#if 1
		//融合位置修正
		float width_y, width_x;
		int sample1_start_rows, sample1_end_rows, sample1_start_cols, sample1_end_cols;
		int sample2_start_rows, sample2_end_rows, sample2_start_cols, sample2_end_cols;
						
		Point2i last_photo_coordinate;
						
		if(number == 0)
		{
			last_photo_coordinate.x = photo_on_map[_last_line - 1][_last_line_number_size - 1].x;
			last_photo_coordinate.y = photo_on_map[_last_line - 1][_last_line_number_size - 1].y;
		}
		else
		{
			last_photo_coordinate.x = photo_on_map[_current_line - 1][number - 1].x;
			last_photo_coordinate.y = photo_on_map[_current_line - 1][number - 1].y;
		}
								
		width_y = image.rows - abs(image_point.y - last_photo_coordinate.y);
		width_x = image.cols - abs(image_point.x - last_photo_coordinate.x);
								
		int w_y = Y_WIDTH;
		int w_x = X_WIDTH;
		if(w_y > width_y / 2)
		{
			w_y = width_y / 2 - 10;
		}
								
		if(w_x > width_x / 2)
		{
			w_x = width_x / 2 - 10;
		}
										
		if(last_photo_coordinate.y < image_point.y)
		{
			sample1_start_rows = width_y / 2  - w_y;
			sample1_end_rows = width_y / 2 + w_y;
								
			sample2_start_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2  - w_y;
			sample2_end_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2 + w_y;
		}
		else
		{
			sample1_start_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2  - w_y;
			sample1_end_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2 + w_y;
								
			sample2_start_rows = width_y / 2  - w_y;
			sample2_end_rows = width_y / 2 + w_y;
		}
								
								
		if(last_photo_coordinate.x < image_point.x)
		{
			sample1_start_cols = width_x / 2 - w_x;
			sample1_end_cols = width_x / 2 + w_x;
								
			sample2_start_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 - w_x;
			sample2_end_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 + w_x;
		}
		else
		{
			sample1_start_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 - w_x;
			sample1_end_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 + w_x;
											
			sample2_start_cols = width_x / 2 - w_x;
			sample2_end_cols = width_x / 2 + w_x;
		}
								
		Mat sample1_image = image(cv::Range(sample1_start_rows, sample1_end_rows),
												cv::Range(sample1_start_cols, sample1_end_cols));
								
										
										
		Mat sample2_image = image_last(cv::Range(sample2_start_rows, sample2_end_rows),
												cv::Range(sample2_start_cols, sample2_end_cols));

		//为了提高速度，先对图像进行缩小
		Mat sample1_image_narrow, sample2_image_narrow;

		image_algorithm->Image_resize(sample1_image, sample1_image_narrow,	Size(sample1_image.cols / 2, sample1_image.rows / 2));
		image_algorithm->Image_resize(sample2_image, sample2_image_narrow,	Size(sample2_image.cols / 2, sample2_image.rows / 2));
								
		//对图片进行模糊处理
		Mat blur_image1, blur_image2;
								
		//双边滤波
		bilateralFilter(sample1_image_narrow, blur_image1,15,100,3);
		bilateralFilter(sample2_image_narrow, blur_image2,15,100,3);
								
		Point2i sample_diff;
						
		image_algorithm->Image_fast_mosaic_algorithm(blur_image2, blur_image1, sample_diff);
										
		cout << "------smaple diff x:" << sample_diff.x << ", y:" << sample_diff.y << endl;
										
		image_point.y -= sample_diff.y * 2;
		image_point.x += sample_diff.x * 2;
				
#if 1
		//左右融合位置修正
								
		if(number != 0)
		{
			int right_photo = _last_line_number_size - photo_on_map[_current_line - 1].size();
			Point2i right_photo_coordinate;
			right_photo_coordinate.x = photo_on_map[_last_line - 1][right_photo - 1].x;
			right_photo_coordinate.y = photo_on_map[_last_line - 1][right_photo - 1].y;
				
				
			width_y = image2.rows - abs(image_point.y - right_photo_coordinate.y);
			width_x = image2.cols - abs(image_point.x - right_photo_coordinate.x);
				
				
			w_y = Y_WIDTH;
			w_x = X_WIDTH;
			if(w_y > width_y / 2)
			{
				w_y = width_y / 2 - 10;
			}
								
			if(w_x > width_x / 2)
			{
				w_x = width_x / 2 - 10;
			}
				
			if(right_photo_coordinate.y < image_point.y)
			{
				sample1_start_rows = width_y / 2  - w_y;
				sample1_end_rows = width_y / 2 + w_y;
								
				sample2_start_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2  - w_y;
				sample2_end_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2 + w_y;
			}
			else
			{
				sample1_start_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2  - w_y;
				sample1_end_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2 + w_y;
								
				sample2_start_rows = width_y / 2  - w_y;
				sample2_end_rows = width_y / 2 + w_y;
			}
								
								
			if(right_photo_coordinate.x < image_point.x)
			{
				sample1_start_cols = width_x / 2 - w_x;
				sample1_end_cols = width_x / 2 + w_x;
								
				sample2_start_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 - w_x;
				sample2_end_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 + w_x;
			}
			else
			{
				sample1_start_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 - w_x;
				sample1_end_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 + w_x;
											
				sample2_start_cols = width_x / 2 - w_x;
				sample2_end_cols = width_x / 2 + w_x;
			}
				
				
			Mat sample1_image_right = image(cv::Range(sample1_start_rows, sample1_end_rows),
												cv::Range(sample1_start_cols, sample1_end_cols));
				
			strFile.clear();
			strFile = "./resize_image/";
			strFile += image_name[_last_line_number  + right_photo - 1];
				
			Mat right_image = imread(strFile.c_str());
							
			if(right_image.empty())
			{
				cout << "failed to load:" << strFile << endl;
				return -1;
			}
				
			Mat sample2_image_right = right_image(cv::Range(sample2_start_rows, sample2_end_rows),
														cv::Range(sample2_start_cols, sample2_end_cols));

			//为了提高速度，先对图像进行缩小
			Mat sample1_image_right_narrow, sample2_image_right_narrow;

			image_algorithm->Image_resize(sample1_image_right, sample1_image_right_narrow,	Size(sample1_image_right.cols / 2, sample1_image_right.rows / 2));
			image_algorithm->Image_resize(sample2_image_right, sample2_image_right_narrow,	Size(sample2_image_right.cols / 2, sample2_image_right.rows / 2));
			//对图片进行模仿处理
			Mat blur_image1_right, blur_image2_right;
				
			//双边滤波
			bilateralFilter(sample1_image_right_narrow, blur_image1_right,15,100,3);
			bilateralFilter(sample2_image_right_narrow, blur_image2_right,15,100,3);
				
			Point2i sample_diff_right;
						
			image_algorithm->Image_fast_mosaic_algorithm(blur_image2_right, blur_image1_right, sample_diff_right);
										
			cout << "++++right+++++++++++++++smaple diff x:" << sample_diff_right.x << ", y:" << sample_diff_right.y << endl;
										
			image_point.y -= sample_diff_right.y * 2;
			image_point.x += sample_diff_right.x * 2;
		}
						
#endif
				
#endif

		
		photo_on_map[_current_line - 1].push_back(image_point);
		number++;

		//截掉上 边的1/4, 截掉右边的1/4
		int cut_size_up = image.rows / 4;
		int cut_size_right = image.cols / 4;

		if(number == 0)
		{
			cut_size_up = 0;
		}


		image_point.y += image.rows / 4;


	

		Mat dest_image = image(cv::Range(cut_size_up, image.rows),
															cv::Range(0, image.cols - cut_size_right));


		dest_image.copyTo(map_test(Rect(image_point.x, image_point.y, dest_image.cols, dest_image.rows)));



		image_last.release();
		image_last = image.clone();
	}
		
		
		
#endif


	cout << "the 5 line is ok--------------" << endl;




#if 1
	number = 0;
	_last_line_number = 47;
	_last_line++;
	_current_line++;
	_last_line_number_size = photo_on_map[_last_line - 1].size();

	//第六 条航线8张图片
	for(int i=58; i<66; i++)
	{
		//读取图片
		string strFile = "./resize_image/";
		strFile += image_name[i];
			
		Mat image = imread(strFile.c_str());
		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}
			
		//根据地图原点gps 坐标，计算该图片的坐标位置
			
		float distance = image_algorithm->Get_distance(map_origin, gps_data[i]) / scale;
		float bearing = image_algorithm->Get_bearing_cd(map_origin, gps_data[i]);
			
		cout << "i:" << i << ",distance:" << distance << ", bearing:" << bearing << endl;
			
		Point2i image_point;
		image_point.x = (int)(distance * sin((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.cols / 2);
		image_point.y = (int)(distance * cos((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.rows / 2);
			
		cout << "photo point x: " << image_point.x << ", y:" << image_point.y << endl;

#if 1
		//融合位置修正
		float width_y, width_x;
		int sample1_start_rows, sample1_end_rows, sample1_start_cols, sample1_end_cols;
		int sample2_start_rows, sample2_end_rows, sample2_start_cols, sample2_end_cols;
						
		Point2i last_photo_coordinate;
						
		if(number == 0)
		{
			last_photo_coordinate.x = photo_on_map[_last_line - 1][_last_line_number_size - 1].x;
			last_photo_coordinate.y = photo_on_map[_last_line - 1][_last_line_number_size - 1].y;
		}
		else
		{
			last_photo_coordinate.x = photo_on_map[_current_line - 1][number - 1].x;
			last_photo_coordinate.y = photo_on_map[_current_line - 1][number - 1].y;
		}
								
		width_y = image.rows - abs(image_point.y - last_photo_coordinate.y);
		width_x = image.cols - abs(image_point.x - last_photo_coordinate.x);
								
		int w_y = Y_WIDTH;
		int w_x = X_WIDTH;
		if(w_y > width_y / 2)
		{
			w_y = width_y / 2 - 10;
		}
								
		if(w_x > width_x / 2)
		{
			w_x = width_x / 2 - 10;
		}
										
		if(last_photo_coordinate.y < image_point.y)
		{
			sample1_start_rows = width_y / 2  - w_y;
			sample1_end_rows = width_y / 2 + w_y;
								
			sample2_start_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2  - w_y;
			sample2_end_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2 + w_y;
		}
		else
		{
			sample1_start_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2  - w_y;
			sample1_end_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2 + w_y;
								
			sample2_start_rows = width_y / 2  - w_y;
			sample2_end_rows = width_y / 2 + w_y;
		}
								
								
		if(last_photo_coordinate.x < image_point.x)
		{
			sample1_start_cols = width_x / 2 - w_x;
			sample1_end_cols = width_x / 2 + w_x;
								
			sample2_start_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 - w_x;
			sample2_end_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 + w_x;
		}
		else
		{
			sample1_start_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 - w_x;
			sample1_end_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 + w_x;
											
			sample2_start_cols = width_x / 2 - w_x;
			sample2_end_cols = width_x / 2 + w_x;
		}
								
								
								
		Mat sample1_image = image(cv::Range(sample1_start_rows, sample1_end_rows),
												cv::Range(sample1_start_cols, sample1_end_cols));
								
										
										
		Mat sample2_image = image_last(cv::Range(sample2_start_rows, sample2_end_rows),
												cv::Range(sample2_start_cols, sample2_end_cols));

		//为了提高速度，先对图像进行缩小
		Mat sample1_image_narrow, sample2_image_narrow;

		image_algorithm->Image_resize(sample1_image, sample1_image_narrow,	Size(sample1_image.cols / 2, sample1_image.rows / 2));
		image_algorithm->Image_resize(sample2_image, sample2_image_narrow,	Size(sample2_image.cols / 2, sample2_image.rows / 2));
								
		//对图片进行模糊处理
		Mat blur_image1, blur_image2;
								
		//双边滤波
		bilateralFilter(sample1_image_narrow, blur_image1,15,100,3);
		bilateralFilter(sample2_image_narrow, blur_image2,15,100,3);
								
		Point2i sample_diff;
						
		image_algorithm->Image_fast_mosaic_algorithm(blur_image2, blur_image1, sample_diff);
										
		cout << "------smaple diff x:" << sample_diff.x << ", y:" << sample_diff.y << endl;
										
		image_point.y -= sample_diff.y * 2;
		image_point.x += sample_diff.x * 2;
				
#if 1
		//左右融合位置修正
								
		if(number != 0)
		{
			int right_photo = _last_line_number_size - photo_on_map[_current_line - 1].size();
			Point2i right_photo_coordinate;
			right_photo_coordinate.x = photo_on_map[_last_line - 1][right_photo - 1].x;
			right_photo_coordinate.y = photo_on_map[_last_line - 1][right_photo - 1].y;
				
				
			width_y = image.rows - abs(image_point.y - right_photo_coordinate.y);
			width_x = image.cols - abs(image_point.x - right_photo_coordinate.x);
				
				
			w_y = Y_WIDTH;
			w_x = X_WIDTH;
			if(w_y > width_y / 2)
			{
				w_y = width_y / 2 - 10;
			}
								
			if(w_x > width_x / 2)
			{
				w_x = width_x / 2 - 10;
			}
				
			if(right_photo_coordinate.y < image_point.y)
			{
				sample1_start_rows = width_y / 2  - w_y;
				sample1_end_rows = width_y / 2 + w_y;
								
				sample2_start_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2  - w_y;
				sample2_end_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2 + w_y;
			}
			else
			{
				sample1_start_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2  - w_y;
				sample1_end_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2 + w_y;
								
				sample2_start_rows = width_y / 2  - w_y;
				sample2_end_rows = width_y / 2 + w_y;
			}
								
								
			if(right_photo_coordinate.x < image_point.x)
			{
				sample1_start_cols = width_x / 2 - w_x;
				sample1_end_cols = width_x / 2 + w_x;
								
				sample2_start_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 - w_x;
				sample2_end_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 + w_x;
			}
			else
			{
				sample1_start_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 - w_x;
				sample1_end_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 + w_x;
											
				sample2_start_cols = width_x / 2 - w_x;
				sample2_end_cols = width_x / 2 + w_x;
			}
				
				
			Mat sample1_image_right = image(cv::Range(sample1_start_rows, sample1_end_rows),
												cv::Range(sample1_start_cols, sample1_end_cols));
				
			strFile.clear();
			strFile = "./resize_image/";
			strFile += image_name[_last_line_number + right_photo - 1];
				
			Mat right_image = imread(strFile.c_str());
							
			if(right_image.empty())
			{
				cout << "failed to load:" << strFile << endl;
				return -1;
			}
				
			Mat sample2_image_right = right_image(cv::Range(sample2_start_rows, sample2_end_rows),
												cv::Range(sample2_start_cols, sample2_end_cols));

			//为了提高速度，先对图像进行缩小
			Mat sample1_image_right_narrow, sample2_image_right_narrow;

			image_algorithm->Image_resize(sample1_image_right, sample1_image_right_narrow,	Size(sample1_image_right.cols / 2, sample1_image_right.rows / 2));
			image_algorithm->Image_resize(sample2_image_right, sample2_image_right_narrow,	Size(sample2_image_right.cols / 2, sample2_image_right.rows / 2));
			
			//对图片进行模仿处理
			Mat blur_image1_right, blur_image2_right;
				
			//双边滤波
			bilateralFilter(sample1_image_right_narrow, blur_image1_right,15,100,3);
			bilateralFilter(sample2_image_right_narrow, blur_image2_right,15,100,3);
				
			Point2i sample_diff_right;
						
			image_algorithm->Image_fast_mosaic_algorithm(blur_image2_right, blur_image1_right, sample_diff_right);
										
			cout << "++++right+++++++++++++++smaple diff x:" << sample_diff_right.x << ", y:" << sample_diff_right.y << endl;
										
			image_point.y -= sample_diff_right.y * 2;
			image_point.x += sample_diff_right.x * 2;
		}
						
#endif
				
#endif

			
		photo_on_map[_current_line - 1].push_back(image_point);
		number++;
	
	
		//截掉下边的1/4, 截掉右边的1/4
		int cut_size_up = image.rows / 4;
		int cut_size_right = image.cols / 4;

		if(number == 0)
		{
			cut_size_up = 0;
		}
		

		Mat dest_image = image(cv::Range(0, image.rows - cut_size_up),
															cv::Range(0, image.cols - cut_size_right));
		dest_image.copyTo(map_test(Rect(image_point.x, image_point.y, dest_image.cols, dest_image.rows)));
			
			
		image_last.release();
		image_last = image.clone();
	}
#endif

	cout << "the 6 line is ok -------------------" <<endl;


#if 1
	number = 0;
	_last_line_number = 58;
	_last_line++;
	_current_line++;
	_last_line_number_size = photo_on_map[_last_line - 1].size();

	//第七 条航线8张图片
	for(int i=68; i<76; i++)
	{
		//读取图片
		string strFile = "./resize_image/";
		strFile += image_name[i];
				
		Mat image = imread(strFile.c_str());
		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}
				
		//根据地图原点gps 坐标，计算该图片的坐标位置
				
		float distance = image_algorithm->Get_distance(map_origin, gps_data[i]) / scale;
		float bearing = image_algorithm->Get_bearing_cd(map_origin, gps_data[i]);
				
		cout << "i:" << i << ",distance:" << distance << ", bearing:" << bearing << endl;
				
		Point2i image_point;
		image_point.x = (int)(distance * sin((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.cols / 2);
		image_point.y = (int)(distance * cos((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.rows / 2);
				
		cout << "photo point x: " << image_point.x << ", y:" << image_point.y << endl;


#if 1
		//融合位置修正
		float width_y, width_x;
		int sample1_start_rows, sample1_end_rows, sample1_start_cols, sample1_end_cols;
		int sample2_start_rows, sample2_end_rows, sample2_start_cols, sample2_end_cols;
								
		Point2i last_photo_coordinate;
								
		if(number == 0)
		{
			last_photo_coordinate.x = photo_on_map[_last_line - 1][_last_line_number_size - 1].x;
			last_photo_coordinate.y = photo_on_map[_last_line - 1][_last_line_number_size - 1].y;
		}
		else
		{
			last_photo_coordinate.x = photo_on_map[_current_line - 1][number - 1].x;
			last_photo_coordinate.y = photo_on_map[_current_line - 1][number - 1].y;
		}
										
		width_y = image.rows - abs(image_point.y - last_photo_coordinate.y);
		width_x = image.cols - abs(image_point.x - last_photo_coordinate.x);
										
		int w_y = Y_WIDTH;
		int w_x = X_WIDTH;
		if(w_y > width_y / 2)
		{
			w_y = width_y / 2 - 10;
		}
										
		if(w_x > width_x / 2)
		{
			w_x = width_x / 2 - 10;
		}
												
		if(last_photo_coordinate.y < image_point.y)
		{
			sample1_start_rows = width_y / 2  - w_y;
			sample1_end_rows = width_y / 2 + w_y;
										
			sample2_start_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2  - w_y;
			sample2_end_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2 + w_y;
		}
		else
		{
			sample1_start_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2  - w_y;
			sample1_end_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2 + w_y;
										
			sample2_start_rows = width_y / 2  - w_y;
			sample2_end_rows = width_y / 2 + w_y;
		}
										
										
		if(last_photo_coordinate.x < image_point.x)
		{
			sample1_start_cols = width_x / 2 - w_x;
			sample1_end_cols = width_x / 2 + w_x;
										
			sample2_start_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 - w_x;
			sample2_end_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 + w_x;
		}
		else
		{
			sample1_start_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 - w_x;
			sample1_end_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 + w_x;
													
			sample2_start_cols = width_x / 2 - w_x;
			sample2_end_cols = width_x / 2 + w_x;
		}
										
		Mat sample1_image = image(cv::Range(sample1_start_rows, sample1_end_rows),
												cv::Range(sample1_start_cols, sample1_end_cols));
										
												
												
		Mat sample2_image = image_last(cv::Range(sample2_start_rows, sample2_end_rows),
												cv::Range(sample2_start_cols, sample2_end_cols));

		//为了提高速度，先对图像进行缩小
		Mat sample1_image_narrow, sample2_image_narrow;

		image_algorithm->Image_resize(sample1_image, sample1_image_narrow,	Size(sample1_image.cols / 2, sample1_image.rows / 2));
		image_algorithm->Image_resize(sample2_image, sample2_image_narrow,	Size(sample2_image.cols / 2, sample2_image.rows / 2));
										
		//对图片进行模糊处理
		Mat blur_image1, blur_image2;
										
		//双边滤波
		bilateralFilter(sample1_image_narrow, blur_image1,15,100,3);
		bilateralFilter(sample2_image_narrow, blur_image2,15,100,3);
										
		Point2i sample_diff;
								
		image_algorithm->Image_fast_mosaic_algorithm(blur_image2, blur_image1, sample_diff);
												
		cout << "------smaple diff x:" << sample_diff.x << ", y:" << sample_diff.y << endl;
												
		image_point.y -= sample_diff.y * 2;
		image_point.x += sample_diff.x * 2;
						
#if 1
		//左右融合位置修正
										
		if(number != 0)
		{
			int right_photo = _last_line_number_size - photo_on_map[_current_line - 1].size();
			Point2i right_photo_coordinate;
			right_photo_coordinate.x = photo_on_map[_last_line - 1][right_photo - 1].x;
			right_photo_coordinate.y = photo_on_map[_last_line - 1][right_photo - 1].y;
						
						
			width_y = image2.rows - abs(image_point.y - right_photo_coordinate.y);
			width_x = image2.cols - abs(image_point.x - right_photo_coordinate.x);
						
						
			w_y = Y_WIDTH;
			w_x = X_WIDTH;
			if(w_y > width_y / 2)
			{
				w_y = width_y / 2 - 10;
			}
										
			if(w_x > width_x / 2)
			{
				w_x = width_x / 2 - 10;
			}
						
			if(right_photo_coordinate.y < image_point.y)
			{
				sample1_start_rows = width_y / 2  - w_y;
				sample1_end_rows = width_y / 2 + w_y;
										
				sample2_start_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2  - w_y;
				sample2_end_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2 + w_y;
			}
			else
			{
				sample1_start_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2  - w_y;
				sample1_end_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2 + w_y;
										
				sample2_start_rows = width_y / 2  - w_y;
				sample2_end_rows = width_y / 2 + w_y;
			}
										
										
			if(right_photo_coordinate.x < image_point.x)
			{
				sample1_start_cols = width_x / 2 - w_x;
				sample1_end_cols = width_x / 2 + w_x;
										
				sample2_start_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 - w_x;
				sample2_end_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 + w_x;
			}
			else
			{
				sample1_start_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 - w_x;
				sample1_end_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 + w_x;
													
				sample2_start_cols = width_x / 2 - w_x;
				sample2_end_cols = width_x / 2 + w_x;
			}
						
						
			Mat sample1_image_right = image(cv::Range(sample1_start_rows, sample1_end_rows),
												cv::Range(sample1_start_cols, sample1_end_cols));
						
			strFile.clear();
			strFile = "./resize_image/";
			strFile += image_name[_last_line_number  + right_photo - 1];
						
			Mat right_image = imread(strFile.c_str());
									
			if(right_image.empty())
			{
				cout << "failed to load:" << strFile << endl;
				return -1;
			}
						
			Mat sample2_image_right = right_image(cv::Range(sample2_start_rows, sample2_end_rows),
														cv::Range(sample2_start_cols, sample2_end_cols));
			//为了提高速度，先对图像进行缩小
			Mat sample1_image_right_narrow, sample2_image_right_narrow;

			image_algorithm->Image_resize(sample1_image_right, sample1_image_right_narrow,	Size(sample1_image_right.cols / 2, sample1_image_right.rows / 2));
			image_algorithm->Image_resize(sample2_image_right, sample2_image_right_narrow,	Size(sample2_image_right.cols / 2, sample2_image_right.rows / 2));
			
			//对图片进行模仿处理
			Mat blur_image1_right, blur_image2_right;
						
			//双边滤波
			bilateralFilter(sample1_image_right_narrow, blur_image1_right,15,100,3);
			bilateralFilter(sample2_image_right_narrow, blur_image2_right,15,100,3);
						
			Point2i sample_diff_right;
								
			image_algorithm->Image_fast_mosaic_algorithm(blur_image2_right, blur_image1_right, sample_diff_right);
												
			cout << "++++right+++++++++++++++smaple diff x:" << sample_diff_right.x << ", y:" << sample_diff_right.y << endl;
												
			image_point.y -= sample_diff_right.y * 2;
			image_point.x += sample_diff_right.x * 2;
		}
								
#endif
						
#endif

				
				
		photo_on_map[_current_line - 1].push_back(image_point);
		number++;

		//截掉上 边的1/4, 截掉右边的1/4
		int cut_size_up = image.rows / 4;
		int cut_size_right = image.cols / 4;

		if(number == 0)
		{
			cut_size_up = 0;
		}


		image_point.y += image.rows / 4;


	

		Mat dest_image = image(cv::Range(cut_size_up, image.rows),
													cv::Range(0, image.cols - cut_size_right));


		dest_image.copyTo(map_test(Rect(image_point.x, image_point.y, dest_image.cols, dest_image.rows)));
				
				
		image_last.release();
		image_last = image.clone();
	}
#endif

	cout << "the 7 line is ok---------------" << endl;


#if 1
	number = 0;
	_last_line_number = 68;
	_last_line++;
	_current_line++;
	_last_line_number_size = photo_on_map[_last_line - 1].size();

	//第八 条航线8 张图片
	for(int i=80; i<88; i++)
	{
		//读取图片
		string strFile = "./resize_image/";
		strFile += image_name[i];
					
		Mat image = imread(strFile.c_str());
		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}
					
		//根据地图原点gps 坐标，计算该图片的坐标位置
					
		float distance = image_algorithm->Get_distance(map_origin, gps_data[i]) / scale;
		float bearing = image_algorithm->Get_bearing_cd(map_origin, gps_data[i]);
					
		cout << "i:" << i << ",distance:" << distance << ", bearing:" << bearing << endl;
					
		Point2i image_point;
		image_point.x = (int)(distance * sin((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.cols / 2);
		image_point.y = (int)(distance * cos((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.rows / 2);
					
		cout << "photo point x: " << image_point.x << ", y:" << image_point.y << endl;

#if 1
		//融合位置修正
		float width_y, width_x;
		int sample1_start_rows, sample1_end_rows, sample1_start_cols, sample1_end_cols;
		int sample2_start_rows, sample2_end_rows, sample2_start_cols, sample2_end_cols;
								
		Point2i last_photo_coordinate;
								
		if(number == 0)
		{
			last_photo_coordinate.x = photo_on_map[_last_line - 1][_last_line_number_size - 1].x;
			last_photo_coordinate.y = photo_on_map[_last_line - 1][_last_line_number_size - 1].y;
		}
		else
		{
			last_photo_coordinate.x = photo_on_map[_current_line - 1][number - 1].x;
			last_photo_coordinate.y = photo_on_map[_current_line - 1][number - 1].y;
		}
										
		width_y = image.rows - abs(image_point.y - last_photo_coordinate.y);
		width_x = image.cols - abs(image_point.x - last_photo_coordinate.x);
										
		int w_y = Y_WIDTH;
		int w_x = X_WIDTH;
		if(w_y > width_y / 2)
		{
			w_y = width_y / 2 - 10;
		}
										
		if(w_x > width_x / 2)
		{
			w_x = width_x / 2 - 10;
		}
												
		if(last_photo_coordinate.y < image_point.y)
		{
			sample1_start_rows = width_y / 2  - w_y;
			sample1_end_rows = width_y / 2 + w_y;
										
			sample2_start_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2  - w_y;
			sample2_end_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2 + w_y;
		}
		else
		{
			sample1_start_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2  - w_y;
			sample1_end_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2 + w_y;
										
			sample2_start_rows = width_y / 2  - w_y;
			sample2_end_rows = width_y / 2 + w_y;
		}
										
										
		if(last_photo_coordinate.x < image_point.x)
		{
			sample1_start_cols = width_x / 2 - w_x;
			sample1_end_cols = width_x / 2 + w_x;
										
			sample2_start_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 - w_x;
			sample2_end_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 + w_x;
		}
		else
		{
			sample1_start_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 - w_x;
			sample1_end_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 + w_x;
													
			sample2_start_cols = width_x / 2 - w_x;
			sample2_end_cols = width_x / 2 + w_x;
		}
										
										
										
		Mat sample1_image = image(cv::Range(sample1_start_rows, sample1_end_rows),
												cv::Range(sample1_start_cols, sample1_end_cols));
										
												
												
		Mat sample2_image = image_last(cv::Range(sample2_start_rows, sample2_end_rows),
												cv::Range(sample2_start_cols, sample2_end_cols));

		//为了提高速度，先对图像进行缩小
		Mat sample1_image_narrow, sample2_image_narrow;

		image_algorithm->Image_resize(sample1_image, sample1_image_narrow,	Size(sample1_image.cols / 2, sample1_image.rows / 2));
		image_algorithm->Image_resize(sample2_image, sample2_image_narrow,	Size(sample2_image.cols / 2, sample2_image.rows / 2));
										
		//对图片进行模糊处理
		Mat blur_image1, blur_image2;
										
		//双边滤波
		bilateralFilter(sample1_image_narrow, blur_image1,15,100,3);
		bilateralFilter(sample2_image_narrow, blur_image2,15,100,3);
										
		Point2i sample_diff;
								
		image_algorithm->Image_fast_mosaic_algorithm(blur_image2, blur_image1, sample_diff);
												
		cout << "------smaple diff x:" << sample_diff.x << ", y:" << sample_diff.y << endl;
												
		image_point.y -= sample_diff.y * 2;
		image_point.x += sample_diff.x * 2;
						
#if 1
		//左右融合位置修正
										
		if(number != 0)
		{
			int right_photo = _last_line_number_size - photo_on_map[_current_line - 1].size();
			Point2i right_photo_coordinate;
			right_photo_coordinate.x = photo_on_map[_last_line - 1][right_photo - 1].x;
			right_photo_coordinate.y = photo_on_map[_last_line - 1][right_photo - 1].y;
						
						
			width_y = image.rows - abs(image_point.y - right_photo_coordinate.y);
			width_x = image.cols - abs(image_point.x - right_photo_coordinate.x);
						
						
			w_y = Y_WIDTH;
			w_x = X_WIDTH;
			if(w_y > width_y / 2)
			{
				w_y = width_y / 2 - 10;
			}
										
			if(w_x > width_x / 2)
			{
				w_x = width_x / 2 - 10;
			}
						
			if(right_photo_coordinate.y < image_point.y)
			{
				sample1_start_rows = width_y / 2  - w_y;
				sample1_end_rows = width_y / 2 + w_y;
										
				sample2_start_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2  - w_y;
				sample2_end_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2 + w_y;
			}
			else
			{
				sample1_start_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2  - w_y;
				sample1_end_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2 + w_y;
										
				sample2_start_rows = width_y / 2  - w_y;
				sample2_end_rows = width_y / 2 + w_y;
			}
										
										
			if(right_photo_coordinate.x < image_point.x)
			{
				sample1_start_cols = width_x / 2 - w_x;
				sample1_end_cols = width_x / 2 + w_x;
										
				sample2_start_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 - w_x;
				sample2_end_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 + w_x;
			}
			else
			{
				sample1_start_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 - w_x;
				sample1_end_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 + w_x;
													
				sample2_start_cols = width_x / 2 - w_x;
				sample2_end_cols = width_x / 2 + w_x;
			}
						
						
			Mat sample1_image_right = image(cv::Range(sample1_start_rows, sample1_end_rows),
												cv::Range(sample1_start_cols, sample1_end_cols));
						
			strFile.clear();
			strFile = "./resize_image/";
			strFile += image_name[_last_line_number + right_photo - 1];
						
			Mat right_image = imread(strFile.c_str());
									
			if(right_image.empty())
			{
				cout << "failed to load:" << strFile << endl;
				return -1;
			}
						
			Mat sample2_image_right = right_image(cv::Range(sample2_start_rows, sample2_end_rows),
												cv::Range(sample2_start_cols, sample2_end_cols));
			//为了提高速度，先对图像进行缩小
			Mat sample1_image_right_narrow, sample2_image_right_narrow;

			image_algorithm->Image_resize(sample1_image_right, sample1_image_right_narrow,	Size(sample1_image_right.cols / 2, sample1_image_right.rows / 2));
			image_algorithm->Image_resize(sample2_image_right, sample2_image_right_narrow,	Size(sample2_image_right.cols / 2, sample2_image_right.rows / 2));
			
			//对图片进行模仿处理
			Mat blur_image1_right, blur_image2_right;
						
			//双边滤波
			bilateralFilter(sample1_image_right_narrow, blur_image1_right,15,100,3);
			bilateralFilter(sample2_image_right_narrow, blur_image2_right,15,100,3);
						
			Point2i sample_diff_right;
								
			image_algorithm->Image_fast_mosaic_algorithm(blur_image2_right, blur_image1_right, sample_diff_right);
												
			cout << "++++right+++++++++++++++smaple diff x:" << sample_diff_right.x << ", y:" << sample_diff_right.y << endl;
												
			image_point.y -= sample_diff_right.y * 2;
			image_point.x += sample_diff_right.x * 2;
		}
								
#endif
						
#endif

					
					
		photo_on_map[_current_line - 1].push_back(image_point);
		number++;
	
	
		//截掉下边的1/4, 截掉右边的1/4
		int cut_size_up = image.rows / 4;
		int cut_size_right = image.cols / 4;

		if(number == 0)
		{
			cut_size_up = 0;
		}
		

		Mat dest_image = image(cv::Range(0, image.rows - cut_size_up),
															cv::Range(0, image.cols - cut_size_right));
		dest_image.copyTo(map_test(Rect(image_point.x, image_point.y, dest_image.cols, dest_image.rows)));
					
					
		image_last.release();
		image_last = image.clone();
	}
#endif

	cout << "the 8 line is ok--------------------" << endl;



#if 1
	
	number = 0;
	_last_line_number = 80;
	_last_line++;
	_current_line++;
	_last_line_number_size = photo_on_map[_last_line - 1].size();

	//第九 条航线8 张图片
	for(int i=90; i<98; i++)
	{
		//读取图片
		string strFile = "./resize_image/";
		strFile += image_name[i];
						
		Mat image = imread(strFile.c_str());
		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}
						
		//根据地图原点gps 坐标，计算该图片的坐标位置
						
		float distance = image_algorithm->Get_distance(map_origin, gps_data[i]) / scale;
		float bearing = image_algorithm->Get_bearing_cd(map_origin, gps_data[i]);
						
		cout << "i:" << i << ",distance:" << distance << ", bearing:" << bearing << endl;
						
		Point2i image_point;
		image_point.x = (int)(distance * sin((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.cols / 2);
		image_point.y = (int)(distance * cos((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.rows / 2);
						
		cout << "photo point x: " << image_point.x << ", y:" << image_point.y << endl;


#if 1
		//融合位置修正
		float width_y, width_x;
		int sample1_start_rows, sample1_end_rows, sample1_start_cols, sample1_end_cols;
		int sample2_start_rows, sample2_end_rows, sample2_start_cols, sample2_end_cols;
										
		Point2i last_photo_coordinate;
										
		if(number == 0)
		{
			last_photo_coordinate.x = photo_on_map[_last_line - 1][_last_line_number_size - 1].x;
			last_photo_coordinate.y = photo_on_map[_last_line - 1][_last_line_number_size - 1].y;
		}
		else
		{
			last_photo_coordinate.x = photo_on_map[_current_line - 1][number - 1].x;
			last_photo_coordinate.y = photo_on_map[_current_line - 1][number - 1].y;
		}
												
		width_y = image.rows - abs(image_point.y - last_photo_coordinate.y);
		width_x = image.cols - abs(image_point.x - last_photo_coordinate.x);
												
		int w_y = Y_WIDTH;
		int w_x = X_WIDTH;
		if(w_y > width_y / 2)
		{
			w_y = width_y / 2 - 10;
		}
												
		if(w_x > width_x / 2)
		{
			w_x = width_x / 2 - 10;
		}
														
		if(last_photo_coordinate.y < image_point.y)
		{
			sample1_start_rows = width_y / 2  - w_y;
			sample1_end_rows = width_y / 2 + w_y;
												
			sample2_start_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2  - w_y;
			sample2_end_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2 + w_y;
		}
		else
		{
			sample1_start_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2  - w_y;
			sample1_end_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2 + w_y;
												
			sample2_start_rows = width_y / 2  - w_y;
			sample2_end_rows = width_y / 2 + w_y;
		}
												
												
		if(last_photo_coordinate.x < image_point.x)
		{
			sample1_start_cols = width_x / 2 - w_x;
			sample1_end_cols = width_x / 2 + w_x;
												
			sample2_start_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 - w_x;
			sample2_end_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 + w_x;
		}
		else
		{
			sample1_start_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 - w_x;
			sample1_end_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 + w_x;
															
			sample2_start_cols = width_x / 2 - w_x;
			sample2_end_cols = width_x / 2 + w_x;
		}
												
		Mat sample1_image = image(cv::Range(sample1_start_rows, sample1_end_rows),
												cv::Range(sample1_start_cols, sample1_end_cols));
												
														
														
		Mat sample2_image = image_last(cv::Range(sample2_start_rows, sample2_end_rows),
												cv::Range(sample2_start_cols, sample2_end_cols));

		//为了提高速度，先对图像进行缩小
		Mat sample1_image_narrow, sample2_image_narrow;

		image_algorithm->Image_resize(sample1_image, sample1_image_narrow,	Size(sample1_image.cols / 2, sample1_image.rows / 2));
		image_algorithm->Image_resize(sample2_image, sample2_image_narrow,	Size(sample2_image.cols / 2, sample2_image.rows / 2));
												
		//对图片进行模糊处理
		Mat blur_image1, blur_image2;
												
		//双边滤波
		bilateralFilter(sample1_image_narrow, blur_image1,15,100,3);
		bilateralFilter(sample2_image_narrow, blur_image2,15,100,3);
												
		Point2i sample_diff;
										
		image_algorithm->Image_fast_mosaic_algorithm(blur_image2, blur_image1, sample_diff);
														
		cout << "------smaple diff x:" << sample_diff.x << ", y:" << sample_diff.y << endl;
														
		image_point.y -= sample_diff.y * 2;
		image_point.x += sample_diff.x * 2;
								
#if 1
		//左右融合位置修正
												
		if(number != 0)
		{
			int right_photo = _last_line_number_size - photo_on_map[_current_line - 1].size();
			Point2i right_photo_coordinate;
			right_photo_coordinate.x = photo_on_map[_last_line - 1][right_photo - 1].x;
			right_photo_coordinate.y = photo_on_map[_last_line - 1][right_photo - 1].y;
								
								
			width_y = image2.rows - abs(image_point.y - right_photo_coordinate.y);
			width_x = image2.cols - abs(image_point.x - right_photo_coordinate.x);
								
								
			w_y = Y_WIDTH;
			w_x = X_WIDTH;
			if(w_y > width_y / 2)
			{
				w_y = width_y / 2 - 10;
			}
												
			if(w_x > width_x / 2)
			{
				w_x = width_x / 2 - 10;
			}
								
			if(right_photo_coordinate.y < image_point.y)
			{
				sample1_start_rows = width_y / 2  - w_y;
				sample1_end_rows = width_y / 2 + w_y;
												
				sample2_start_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2  - w_y;
				sample2_end_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2 + w_y;
			}
			else
			{
				sample1_start_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2  - w_y;
				sample1_end_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2 + w_y;
												
				sample2_start_rows = width_y / 2  - w_y;
				sample2_end_rows = width_y / 2 + w_y;
			}
												
												
			if(right_photo_coordinate.x < image_point.x)
			{
				sample1_start_cols = width_x / 2 - w_x;
				sample1_end_cols = width_x / 2 + w_x;
												
				sample2_start_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 - w_x;
				sample2_end_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 + w_x;
			}
			else
			{
				sample1_start_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 - w_x;
				sample1_end_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 + w_x;
															
				sample2_start_cols = width_x / 2 - w_x;
				sample2_end_cols = width_x / 2 + w_x;
			}
								
								
			Mat sample1_image_right = image(cv::Range(sample1_start_rows, sample1_end_rows),
												cv::Range(sample1_start_cols, sample1_end_cols));
								
			strFile.clear();
			strFile = "./resize_image/";
			strFile += image_name[_last_line_number  + right_photo - 1];
								
			Mat right_image = imread(strFile.c_str());
											
			if(right_image.empty())
			{
				cout << "failed to load:" << strFile << endl;
				return -1;
			}
								
			Mat sample2_image_right = right_image(cv::Range(sample2_start_rows, sample2_end_rows),
														cv::Range(sample2_start_cols, sample2_end_cols));
			//为了提高速度，先对图像进行缩小
			Mat sample1_image_right_narrow, sample2_image_right_narrow;

			image_algorithm->Image_resize(sample1_image_right, sample1_image_right_narrow,	Size(sample1_image_right.cols / 2, sample1_image_right.rows / 2));
			image_algorithm->Image_resize(sample2_image_right, sample2_image_right_narrow,	Size(sample2_image_right.cols / 2, sample2_image_right.rows / 2));

			
			
			//对图片进行模仿处理
			Mat blur_image1_right, blur_image2_right;
								
			//双边滤波
			bilateralFilter(sample1_image_right_narrow, blur_image1_right,15,100,3);
			bilateralFilter(sample2_image_right_narrow, blur_image2_right,15,100,3);
								
			Point2i sample_diff_right;
										
			image_algorithm->Image_fast_mosaic_algorithm(blur_image2_right, blur_image1_right, sample_diff_right);
														
			cout << "++++right+++++++++++++++smaple diff x:" << sample_diff_right.x << ", y:" << sample_diff_right.y << endl;
														
			image_point.y -= sample_diff_right.y * 2;
			image_point.x += sample_diff_right.x * 2;
		}
										
#endif
								
#endif

						
		photo_on_map[_current_line - 1].push_back(image_point);
		number++;

		//截掉上 边的1/4, 截掉右边的1/4
		int cut_size_up = image.rows / 4;
		int cut_size_right = image.cols / 4;

		if(number == 0)
		{
			cut_size_up = 0;
		}


		image_point.y += image.rows / 4;


	

		Mat dest_image = image(cv::Range(cut_size_up, image.rows),
													cv::Range(0, image.cols - cut_size_right));


		dest_image.copyTo(map_test(Rect(image_point.x, image_point.y, dest_image.cols, dest_image.rows)));
						
						
		image_last.release();
		image_last = image.clone();
	}
#endif

	cout << "the 9 line is ok---------------------" << endl;




#if 1
	number = 0;
	_last_line_number = 90;
	_last_line++;
	_current_line++;
	_last_line_number_size = photo_on_map[_last_line - 1].size();
	//第十条航线8张图片
	for(int i=102; i<110; i++)
	{
		//读取图片
		string strFile = "./resize_image/";
		strFile += image_name[i];
							
		Mat image = imread(strFile.c_str());
		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}
							
		//根据地图原点gps 坐标，计算该图片的坐标位置
							
		float distance = image_algorithm->Get_distance(map_origin, gps_data[i]) / scale;
		float bearing = image_algorithm->Get_bearing_cd(map_origin, gps_data[i]);
							
		cout << "i:" << i << ",distance:" << distance << ", bearing:" << bearing << endl;
							
		Point2i image_point;
		image_point.x = (int)(distance * sin((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.cols / 2);
		image_point.y = (int)(distance * cos((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.rows / 2);
							
		cout << "photo point x: " << image_point.x << ", y:" << image_point.y << endl;


#if 1
		//融合位置修正
		float width_y, width_x;
		int sample1_start_rows, sample1_end_rows, sample1_start_cols, sample1_end_cols;
		int sample2_start_rows, sample2_end_rows, sample2_start_cols, sample2_end_cols;
										
		Point2i last_photo_coordinate;
										
		if(number == 0)
		{
			last_photo_coordinate.x = photo_on_map[_last_line - 1][_last_line_number_size - 1].x;
			last_photo_coordinate.y = photo_on_map[_last_line - 1][_last_line_number_size - 1].y;
		}
		else
		{
			last_photo_coordinate.x = photo_on_map[_current_line - 1][number - 1].x;
			last_photo_coordinate.y = photo_on_map[_current_line - 1][number - 1].y;
		}
												
		width_y = image.rows - abs(image_point.y - last_photo_coordinate.y);
		width_x = image.cols - abs(image_point.x - last_photo_coordinate.x);
												
		int w_y = Y_WIDTH;
		int w_x = X_WIDTH;
		if(w_y > width_y / 2)
		{
			w_y = width_y / 2 - 10;
		}
												
		if(w_x > width_x / 2)
		{
			w_x = width_x / 2 - 10;
		}
														
		if(last_photo_coordinate.y < image_point.y)
		{
			sample1_start_rows = width_y / 2  - w_y;
			sample1_end_rows = width_y / 2 + w_y;
												
			sample2_start_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2  - w_y;
			sample2_end_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2 + w_y;
		}
		else
		{
			sample1_start_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2  - w_y;
			sample1_end_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2 + w_y;
												
			sample2_start_rows = width_y / 2  - w_y;
			sample2_end_rows = width_y / 2 + w_y;
		}
												
												
		if(last_photo_coordinate.x < image_point.x)
		{
			sample1_start_cols = width_x / 2 - w_x;
			sample1_end_cols = width_x / 2 + w_x;
												
			sample2_start_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 - w_x;
			sample2_end_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 + w_x;
		}
		else
		{
			sample1_start_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 - w_x;
			sample1_end_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 + w_x;
															
			sample2_start_cols = width_x / 2 - w_x;
			sample2_end_cols = width_x / 2 + w_x;
		}
												
												
												
		Mat sample1_image = image(cv::Range(sample1_start_rows, sample1_end_rows),
												cv::Range(sample1_start_cols, sample1_end_cols));
												
														
														
		Mat sample2_image = image_last(cv::Range(sample2_start_rows, sample2_end_rows),
												cv::Range(sample2_start_cols, sample2_end_cols));

		//为了提高速度，先对图像进行缩小
		Mat sample1_image_narrow, sample2_image_narrow;

		image_algorithm->Image_resize(sample1_image, sample1_image_narrow,	Size(sample1_image.cols / 2, sample1_image.rows / 2));
		image_algorithm->Image_resize(sample2_image, sample2_image_narrow,	Size(sample2_image.cols / 2, sample2_image.rows / 2));
												
		//对图片进行模糊处理
		Mat blur_image1, blur_image2;
												
		//双边滤波
		bilateralFilter(sample1_image_narrow, blur_image1,15,100,3);
		bilateralFilter(sample2_image_narrow, blur_image2,15,100,3);
												
		Point2i sample_diff;
										
		image_algorithm->Image_fast_mosaic_algorithm(blur_image2, blur_image1, sample_diff);
														
		cout << "------smaple diff x:" << sample_diff.x << ", y:" << sample_diff.y << endl;
														
		image_point.y -= sample_diff.y * 2;
		image_point.x += sample_diff.x * 2;
								
#if 1
		//左右融合位置修正
												
		if(number != 0)
		{
			int right_photo = _last_line_number_size - photo_on_map[_current_line - 1].size();
			Point2i right_photo_coordinate;
			right_photo_coordinate.x = photo_on_map[_last_line - 1][right_photo - 1].x;
			right_photo_coordinate.y = photo_on_map[_last_line - 1][right_photo - 1].y;
								
								
			width_y = image.rows - abs(image_point.y - right_photo_coordinate.y);
			width_x = image.cols - abs(image_point.x - right_photo_coordinate.x);
								
								
			w_y = Y_WIDTH;
			w_x = X_WIDTH;
			if(w_y > width_y / 2)
			{
				w_y = width_y / 2 - 10;
			}
												
			if(w_x > width_x / 2)
			{
				w_x = width_x / 2 - 10;
			}
								
			if(right_photo_coordinate.y < image_point.y)
			{
				sample1_start_rows = width_y / 2  - w_y;
				sample1_end_rows = width_y / 2 + w_y;
												
				sample2_start_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2  - w_y;
				sample2_end_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2 + w_y;
			}
			else
			{
				sample1_start_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2  - w_y;
				sample1_end_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2 + w_y;
												
				sample2_start_rows = width_y / 2  - w_y;
				sample2_end_rows = width_y / 2 + w_y;
			}
												
												
			if(right_photo_coordinate.x < image_point.x)
			{
				sample1_start_cols = width_x / 2 - w_x;
				sample1_end_cols = width_x / 2 + w_x;
												
				sample2_start_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 - w_x;
				sample2_end_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 + w_x;
			}
			else
			{
				sample1_start_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 - w_x;
				sample1_end_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 + w_x;
															
				sample2_start_cols = width_x / 2 - w_x;
				sample2_end_cols = width_x / 2 + w_x;
			}
								
								
			Mat sample1_image_right = image(cv::Range(sample1_start_rows, sample1_end_rows),
												cv::Range(sample1_start_cols, sample1_end_cols));
								
			strFile.clear();
			strFile = "./resize_image/";
			strFile += image_name[_last_line_number + right_photo - 1];
								
			Mat right_image = imread(strFile.c_str());
											
			if(right_image.empty())
			{
				cout << "failed to load:" << strFile << endl;
				return -1;
			}
								
			Mat sample2_image_right = right_image(cv::Range(sample2_start_rows, sample2_end_rows),
												cv::Range(sample2_start_cols, sample2_end_cols));

			//为了提高速度，先对图像进行缩小
			Mat sample1_image_right_narrow, sample2_image_right_narrow;

			image_algorithm->Image_resize(sample1_image_right, sample1_image_right_narrow,	Size(sample1_image_right.cols / 2, sample1_image_right.rows / 2));
			image_algorithm->Image_resize(sample2_image_right, sample2_image_right_narrow,	Size(sample2_image_right.cols / 2, sample2_image_right.rows / 2));

			
			//对图片进行模仿处理
			Mat blur_image1_right, blur_image2_right;
								
			//双边滤波
			bilateralFilter(sample1_image_right_narrow, blur_image1_right,15,100,3);
			bilateralFilter(sample2_image_right_narrow, blur_image2_right,15,100,3);
								
			Point2i sample_diff_right;
										
			image_algorithm->Image_fast_mosaic_algorithm(blur_image2_right, blur_image1_right, sample_diff_right);
														
			cout << "++++right+++++++++++++++smaple diff x:" << sample_diff_right.x << ", y:" << sample_diff_right.y << endl;
														
			image_point.y -= sample_diff_right.y * 2;
			image_point.x += sample_diff_right.x * 2;
		}
										
#endif
								
#endif

							
		photo_on_map[_current_line - 1].push_back(image_point);
		number++;
	
	
		//截掉下边的1/4, 截掉右边的1/4
		int cut_size_up = image.rows / 4;
		int cut_size_right = image.cols / 4;

		if(number == 0)
		{
			cut_size_up = 0;
		}
		

		Mat dest_image = image(cv::Range(0, image.rows - cut_size_up),
															cv::Range(0, image.cols - cut_size_right));
		dest_image.copyTo(map_test(Rect(image_point.x, image_point.y, dest_image.cols, dest_image.rows)));
							
							
		image_last.release();
		image_last = image.clone();
	}
#endif

	cout << "the 10 line is ok-------------------------" << endl;


#if 1
	number = 0;
	_last_line_number = 102;
	_last_line++;
	_current_line++;
	_last_line_number_size = photo_on_map[_last_line - 1].size();
	//第十 一条航线8张图片
	for(int i=112; i<120; i++)
	{
		//读取图片
		string strFile = "./resize_image/";
		strFile += image_name[i];
								
		Mat image = imread(strFile.c_str());
		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}
								
		//根据地图原点gps 坐标，计算该图片的坐标位置
								
		float distance = image_algorithm->Get_distance(map_origin, gps_data[i]) / scale;
		float bearing = image_algorithm->Get_bearing_cd(map_origin, gps_data[i]);
								
		cout << "i:" << i << ",distance:" << distance << ", bearing:" << bearing << endl;
								
		Point2i image_point;
		image_point.x = (int)(distance * sin((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.cols / 2);
		image_point.y = (int)(distance * cos((plane_bearing + 180 - bearing) * (M_PI / 180.0f)) - (float)image.rows / 2);
								
		cout << "photo point x: " << image_point.x << ", y:" << image_point.y << endl;

#if 1
		//融合位置修正
		float width_y, width_x;
		int sample1_start_rows, sample1_end_rows, sample1_start_cols, sample1_end_cols;
		int sample2_start_rows, sample2_end_rows, sample2_start_cols, sample2_end_cols;
												
		Point2i last_photo_coordinate;
												
		if(number == 0)
		{
			last_photo_coordinate.x = photo_on_map[_last_line - 1][_last_line_number_size - 1].x;
			last_photo_coordinate.y = photo_on_map[_last_line - 1][_last_line_number_size - 1].y;
		}
		else
		{
			last_photo_coordinate.x = photo_on_map[_current_line - 1][number - 1].x;
			last_photo_coordinate.y = photo_on_map[_current_line - 1][number - 1].y;
		}
														
		width_y = image.rows - abs(image_point.y - last_photo_coordinate.y);
		width_x = image.cols - abs(image_point.x - last_photo_coordinate.x);
														
		int w_y = Y_WIDTH;
		int w_x = X_WIDTH;
		if(w_y > width_y / 2)
		{
			w_y = width_y / 2 - 10;
		}
														
		if(w_x > width_x / 2)
		{
			w_x = width_x / 2 - 10;
		}
																
		if(last_photo_coordinate.y < image_point.y)
		{
			sample1_start_rows = width_y / 2  - w_y;
			sample1_end_rows = width_y / 2 + w_y;
														
			sample2_start_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2  - w_y;
			sample2_end_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2 + w_y;
		}
		else
		{
			sample1_start_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2  - w_y;
			sample1_end_rows = abs(image_point.y - last_photo_coordinate.y) + width_y / 2 + w_y;
														
			sample2_start_rows = width_y / 2  - w_y;
			sample2_end_rows = width_y / 2 + w_y;
		}
														
														
		if(last_photo_coordinate.x < image_point.x)
		{
			sample1_start_cols = width_x / 2 - w_x;
			sample1_end_cols = width_x / 2 + w_x;
														
			sample2_start_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 - w_x;
			sample2_end_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 + w_x;
		}
		else
		{
			sample1_start_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 - w_x;
			sample1_end_cols = abs(image_point.x - last_photo_coordinate.x) + width_x / 2 + w_x;
																	
			sample2_start_cols = width_x / 2 - w_x;
			sample2_end_cols = width_x / 2 + w_x;
		}
														
		Mat sample1_image = image(cv::Range(sample1_start_rows, sample1_end_rows),
												cv::Range(sample1_start_cols, sample1_end_cols));
														
																
																
		Mat sample2_image = image_last(cv::Range(sample2_start_rows, sample2_end_rows),
												cv::Range(sample2_start_cols, sample2_end_cols));

		//为了提高速度，先对图像进行缩小
		Mat sample1_image_narrow, sample2_image_narrow;

		image_algorithm->Image_resize(sample1_image, sample1_image_narrow,	Size(sample1_image.cols / 2, sample1_image.rows / 2));
		image_algorithm->Image_resize(sample2_image, sample2_image_narrow,	Size(sample2_image.cols / 2, sample2_image.rows / 2));
														
		//对图片进行模糊处理
		Mat blur_image1, blur_image2;
														
		//双边滤波
		bilateralFilter(sample1_image_narrow, blur_image1,15,100,3);
		bilateralFilter(sample2_image_narrow, blur_image2,15,100,3);
														
		Point2i sample_diff;
												
		image_algorithm->Image_fast_mosaic_algorithm(blur_image2, blur_image1, sample_diff);
																
		cout << "------smaple diff x:" << sample_diff.x << ", y:" << sample_diff.y << endl;
																
		image_point.y -= sample_diff.y * 2;
		image_point.x += sample_diff.x * 2;
										
#if 1
		//左右融合位置修正
														
		if(number != 0)
		{
			int right_photo = _last_line_number_size - photo_on_map[_current_line - 1].size();
			Point2i right_photo_coordinate;
			right_photo_coordinate.x = photo_on_map[_last_line - 1][right_photo - 1].x;
			right_photo_coordinate.y = photo_on_map[_last_line - 1][right_photo - 1].y;
										
										
			width_y = image2.rows - abs(image_point.y - right_photo_coordinate.y);
			width_x = image2.cols - abs(image_point.x - right_photo_coordinate.x);
										
										
			w_y = Y_WIDTH;
			w_x = X_WIDTH;
			if(w_y > width_y / 2)
			{
				w_y = width_y / 2 - 10;
			}
														
			if(w_x > width_x / 2)
			{
				w_x = width_x / 2 - 10;
			}
										
			if(right_photo_coordinate.y < image_point.y)
			{
				sample1_start_rows = width_y / 2  - w_y;
				sample1_end_rows = width_y / 2 + w_y;
														
				sample2_start_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2  - w_y;
				sample2_end_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2 + w_y;
			}
			else
			{
				sample1_start_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2  - w_y;
				sample1_end_rows = abs(image_point.y - right_photo_coordinate.y) + width_y / 2 + w_y;
														
				sample2_start_rows = width_y / 2  - w_y;
				sample2_end_rows = width_y / 2 + w_y;
			}
														
														
			if(right_photo_coordinate.x < image_point.x)
			{
				sample1_start_cols = width_x / 2 - w_x;
				sample1_end_cols = width_x / 2 + w_x;
														
				sample2_start_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 - w_x;
				sample2_end_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 + w_x;
			}
			else
			{
				sample1_start_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 - w_x;
				sample1_end_cols = abs(image_point.x - right_photo_coordinate.x) + width_x / 2 + w_x;
																	
				sample2_start_cols = width_x / 2 - w_x;
				sample2_end_cols = width_x / 2 + w_x;
			}
										
										
			Mat sample1_image_right = image(cv::Range(sample1_start_rows, sample1_end_rows),
												cv::Range(sample1_start_cols, sample1_end_cols));
										
			strFile.clear();
			strFile = "./resize_image/";
			strFile += image_name[_last_line_number  + right_photo - 1];
										
			Mat right_image = imread(strFile.c_str());
													
			if(right_image.empty())
			{
				cout << "failed to load:" << strFile << endl;
				return -1;
			}
										
			Mat sample2_image_right = right_image(cv::Range(sample2_start_rows, sample2_end_rows),
														cv::Range(sample2_start_cols, sample2_end_cols));
			//为了提高速度，先对图像进行缩小
			Mat sample1_image_right_narrow, sample2_image_right_narrow;

			image_algorithm->Image_resize(sample1_image_right, sample1_image_right_narrow,	Size(sample1_image_right.cols / 2, sample1_image_right.rows / 2));
			image_algorithm->Image_resize(sample2_image_right, sample2_image_right_narrow,	Size(sample2_image_right.cols / 2, sample2_image_right.rows / 2));

			
			//对图片进行模仿处理
			Mat blur_image1_right, blur_image2_right;
										
			//双边滤波
			bilateralFilter(sample1_image_right_narrow, blur_image1_right,15,100,3);
			bilateralFilter(sample2_image_right_narrow, blur_image2_right,15,100,3);
										
			Point2i sample_diff_right;
												
			image_algorithm->Image_fast_mosaic_algorithm(blur_image2_right, blur_image1_right, sample_diff_right);
																
			cout << "++++right+++++++++++++++smaple diff x:" << sample_diff_right.x << ", y:" << sample_diff_right.y << endl;
																
			image_point.y -= sample_diff_right.y * 2;
			image_point.x += sample_diff_right.x * 2;
		}
												
#endif
										
#endif

								
		photo_on_map[_current_line - 1].push_back(image_point);
		number++;

		//截掉上 边的1/4, 截掉右边的1/4
		int cut_size_up = image.rows / 4;
		int cut_size_right = image.cols / 4;

		if(number == 0)
		{
			cut_size_up = 0;
		}


		image_point.y += image.rows / 4;


	

		Mat dest_image = image(cv::Range(cut_size_up, image.rows),
													cv::Range(0, image.cols - cut_size_right));


		dest_image.copyTo(map_test(Rect(image_point.x, image_point.y, dest_image.cols, dest_image.rows)));
						
								
								
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
