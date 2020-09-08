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

	//用gps 位置坐标，求飞机的航线的航向
	for(int i=0; i<gps_data.size() - 3; i++)
	{
		
		float bearing1 = image_algorithm->Get_bearing_cd(gps_data[i], gps_data[i + 1]);
		float bearing2 = image_algorithm->Get_bearing_cd(gps_data[i + 2], gps_data[i + 3]);

		cout << "bearing1:" << bearing1 << ",bearing2:" << bearing2 << endl;
		
		if(fabs(bearing2 - bearing1) > 150)
		{
			plane_bearing = image_algorithm->Get_bearing_cd(gps_data[0], gps_data[i + 1]);
			break;
		}
	}

	cout << "plane bearing:" << plane_bearing << endl;

	//通过第一张和第二张图片计算

	
	//读取第一张图片
	strFile = "/home/wenyi/workspace/DCIM/10000904/";
	strFile += image_name[0];
	
	Mat image1 = imread(strFile.c_str());
	
	if(image1.empty())
	{
		cout << "failed to load:" << strFile << endl;
		return -1;
	}

	strFile.clear();

	strFile = "/home/wenyi/workspace/DCIM/10000904/";
	strFile += image_name[1];
	Mat image2 = imread(strFile.c_str());
	
	if(image2.empty())
	{
		cout << "failed to load:" << strFile << endl;
		return -1;
	}

	strFile.clear();

	//为了快速拼接把图片缩小
	float narrow_size = 4.0f;
	Mat image1_resize, image2_resize;
	
	image_algorithm->Image_resize(image1, image1_resize,	Size(image1.cols / narrow_size, image1.rows / narrow_size));
	image_algorithm->Image_resize(image2, image2_resize,	Size(image2.cols / narrow_size, image2.rows / narrow_size));

	//快速求出拼接的大致位置
	
	Point2i point_test;
	image_algorithm->Image_fast_mosaic_algorithm(image1_resize, image2_resize,point_test);

	cout << "point test11 x:" << point_test.x << ", y:" << point_test.y << endl;
	
	point_test.x *= narrow_size;
	point_test.y *= narrow_size;

	cout << "point test x:" << point_test.x << ", y:" << point_test.y << endl;

	//在两张图片的公共块中取一个小块，进行进行对比


#if 0
	//测试申请地图空间
	Mat map_test(20000, 20000,CV_8UC3);
	map_test.setTo(0);

	imwrite("map.jpg", map_test);
#endif
	cout << "I am ok" << endl;
	return 0;
}
