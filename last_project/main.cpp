#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <limits.h>
#include "./image_algorithm/image_algorithm.h"

using namespace std;
using namespace cv;



int main(int argc, char **argv)
{

#if 0
	//测试读取所有的图像名称image_name.txt
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
        	cout << s << endl;;
        }
		
	}
#endif

#if 0

	//测试读取gps 和imu 数据
	string strFile = "/home/wenyi/workspace/DCIM/10000904/image_gps_imu.txt";

	ifstream f;
    f.open(strFile.c_str());

	// skip first one lines
	string s0;
	getline(f,s0);

	cout << "lat\t" << "lng\t" << "alt\t" << "roll\t" << "pitch\t" << "yaw" << endl;
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
		
		cout << setprecision(15) << lat << "\t" <<lng << "\t" << alt << "\t" << roll << "\t" << pitch << "\t" << yaw << endl;

        }
		
	}
#endif

	vector<string>  image_name;

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

#if 0
	for(int i=0; i<image_name.size(); i++)
	{
		cout << image_name[i] << endl;
	}
#endif

#if 0
	for(auto name:image_name)
	{
		cout << name << endl;
	}

#endif

	for(auto it = image_name.begin(); it != image_name.end(); ++it)
	{
		cout << *it << endl;
	}

	cout << "I am ok" << endl;
	return 0;
}
