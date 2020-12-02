#include <iostream>
#include <fstream>
#include <math.h>
#include <limits.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>



using namespace std;
using namespace cv;

struct Location {
	Location(int _alt=0, int _lat=0, int _lng=0):
		alt(_alt), lat(_lat), lng(_lng)
	{
	}
	
    int alt:24;                                     ///Altitude in centimeters (meters * 100) see LOCATION_ALT_MAX_M
    int lat;                                        /// Latitude * 10**7
    int lng;                                        ///Longitude * 10**7
};

struct Imu_data {
	Imu_data(float _pitch=0, float _roll=0, float _yaw=0):
		pitch(_pitch), roll(_roll), yaw(_yaw)
	{
	}

	float pitch;
	float roll;
	float yaw;
};


void Image_rotate(Mat& src_image,  Mat& dest_image, double angle)
{
	Point2f pt(src_image.cols/2, src_image.rows/2);
	Mat r = getRotationMatrix2D(pt, angle, 1.0);
	warpAffine(src_image, dest_image, r, Size(src_image.cols, src_image.rows));
}




int main(int argc, char *argv[])
{

	vector<string>  image_name;
	vector<Location> gps_data;
	vector<Imu_data> imu_data;


	string strFile = "../plane_image/image_name.txt";

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


	for(auto name:image_name)
	{
		cout << name << endl;
	}


	strFile = "../plane_image/gps.txt";

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

			struct Location gps(alt * 100, lat * 1.0e7, lng * 1.0e7);
			gps_data.push_back(gps);
        }
	}
	
	f.close();
	strFile.clear();

	for(auto gps:gps_data)
	{
		cout << gps.alt << "\t" << gps.lat << "\t" << gps.lng << endl;
	}


	strFile = "../plane_image/imu.txt";

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

			double roll;
			ss >> roll;

			double pitch;
			ss >> pitch;

			double yaw;
			ss >> yaw;

			struct Imu_data imu(pitch, roll, yaw);
			imu_data.push_back(imu);
        }
	}
	
	f.close();
	strFile.clear();


	for(auto imu:imu_data)
	{
		cout << imu.pitch << "\t" << imu.roll << "\t" <<imu.yaw << endl;
	}




	std::string dir = "./resize_image";
	if(access(dir.c_str(), 0) == -1)
	{
		cout << dir << " is not existing." << endl;
		cout << "now make it!" << endl;
		int flag = mkdir(dir.c_str(), 0777);
	
		if(flag == 0)
		{
			cout << "make successfully" << endl;
		}
		else
		{
			cout << "mkdir error!" << endl;
			return -1;
		}
	}


	float way_line_angle = 92.0f;
	Mat map;

	for(size_t num=0; num < image_name.size(); num++)
	{
		strFile.clear();
		strFile = "../plane_image/";
		strFile += image_name[num];

		cout << image_name[num] << endl;


		Mat image = imread(strFile.c_str());

		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}

		resize(image, image, Size(image.cols / 8, image.rows / 8),INTER_AREA);

		Image_rotate(image, image, way_line_angle - imu_data[num].yaw);

		strFile.clear();
		strFile = "./resize_image/";
		strFile += image_name[num];

		imwrite(strFile.c_str(), image);
	}

	

	waitKey();
	cout << "I am ok" << endl;

	return 0;
}

