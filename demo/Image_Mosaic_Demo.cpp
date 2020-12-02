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


void Image_fast_mosaic_algorithm(cv::Mat &src_image1, cv::Mat &src_image2, cv::Point2i &distance)

{
	//需要先转换成灰度图像
	cv::Mat image1_gray, image2_gray;
	cv::cvtColor(src_image1, image1_gray, CV_RGB2GRAY);
	cv::cvtColor(src_image2, image2_gray, CV_RGB2GRAY);

	cv::Point2i image_size(image1_gray.cols, image1_gray.rows);
	cv::Point2i image1_sample_size;

	//图像1 和图像2  在x 方向上可能移动的最大距离 diff_x
	int diff_x;
	diff_x = image_size.x / 4;
	image1_sample_size.x = image_size.x / 4;
	image1_sample_size.y = image_size.y / 15;
	
	cv::Point2i image2_sample_size(image1_sample_size.x + diff_x, image1_sample_size.y);

	int start_row[6] = {image1_gray.rows / 4,
						image1_gray.rows / 4,
						image1_gray.rows / 4 + image1_sample_size.y + (image_size.y / 10), 
						image1_gray.rows / 4 + image1_sample_size.y + (image_size.y / 10),
						image1_gray.rows / 4 + 2 * (image1_sample_size.y + (image_size.y / 10)),
						image1_gray.rows / 4 + 2 * (image1_sample_size.y + (image_size.y / 10))};

	int start_col[6] = {image1_gray.cols / 3 - image1_sample_size.x / 2,
						image1_gray.cols * 2 / 3 - image1_sample_size.x / 2,
						image1_gray.cols / 4 - image1_sample_size.x / 2,
						image1_gray.cols * 3 / 4 - image1_sample_size.x / 2,
						image1_gray.cols / 3 - image1_sample_size.x / 2,
						image1_gray.cols * 2 / 3 - image1_sample_size.x / 2};


	int min_err[6];
	int min_err_idex[6];
	int min_err_dis[6];

	for(int i=0; i<6; i++)
	{
		min_err[i] = INT_MAX;
		min_err_idex[i] = 0;
		min_err_dis[i] = 0;
	}

	//分别查找4 组中最小二乘的位置
	for(int i=0; i<6; i++)
	{
		//计算图像 1  的匹配模板
		int base[image1_sample_size.x];

		for(int k=0; k<image1_sample_size.x; k++)
		{
			base[k] = image1_gray.at<uchar>(start_row[i], start_col[i] + k) - image1_gray.at<uchar>(start_row[i] + image1_sample_size.y, start_col[i] + k);
		}

		//找出图像2  的最佳匹配
		int num = image2_gray.rows  - image1_sample_size.y;
		int rows_min_err[num];
		int rows_min_err_dis[num];

		for(int n=0; n<num; n++)
		{
			rows_min_err[n] = INT_MAX;
			rows_min_err_dis[n] = 0;
		}

		int match_image[image2_sample_size.x];

		for(int n = 0; n< num; n++)
		{
			for(int j=0; j<image2_sample_size.x; j++)
			{
				match_image[j] = image2_gray.at<uchar>(n, start_col[i] - diff_x / 2 + j) -
								 image2_gray.at<uchar>(n + image2_sample_size.y, start_col[i] - diff_x / 2 + j);
			}

			//求每一行和第一张图像的最小二乘的最佳位置和值
			for(int d=0; d<diff_x; d++)
			{
				int err = 0;
				for(int p=0; p<image1_sample_size.x; p++)
				{
					err += std::pow(match_image[p + d] - base[p], 2);
				}

				
				if(err < rows_min_err[n])
				{
					rows_min_err[n] = err;
					rows_min_err_dis[n] = d;

					if(rows_min_err[n] < min_err[i])
					{
						min_err[i] = rows_min_err[n];
						min_err_dis[i] = rows_min_err_dis[n];
						min_err_idex[i] = n;
					}
				}
			}
		}
	}

	
	int v_dis_x[6];
	int v_dis_y[6];
	int x_average = 0;
	int y_average = 0;

	for(int i=0; i<6; i++)
	{
		v_dis_y[i] = min_err_idex[i] - start_row[i];
		v_dis_x[i] = diff_x / 2 - min_err_dis[i];

		x_average += v_dis_x[i];
		y_average += v_dis_y[i];
	}

	x_average /= 6;
	y_average /= 6;

	//求方差

	int v[6];
	int v_min = INT_MAX;
	int v_min2 = INT_MAX;
	int v_min3 = INT_MAX;
	int v_min_n;
	int v_min2_n;
	int v_min3_n;
	
	for(int i=0; i<6; i++)
	{
		v_dis_x[i] = pow(v_dis_x[i] - x_average, 2);
		v_dis_y[i] = pow(v_dis_y[i] - y_average, 2);

		v[i] = v_dis_x[i] + v_dis_y[i];
		//求出最小值
		if(v[i] < v_min)
		{
			v_min3 = v_min2;
			v_min3_n = v_min2_n;

			v_min2 = v_min;
			v_min2_n = v_min_n;

			
			v_min = v[i];
			v_min_n = i;
		}
		else if(v[i] < v_min2)
		{
			v_min3 = v_min2;
			v_min3_n = v_min2_n;

			v_min2 = v[i];
			v_min2_n = i;
		}
		else if(v[i] < v_min3)
		{
			v_min3 = v[i];
			v_min3_n = i;
		}

	}


	bool is_ok[6] = {false};
	is_ok[v_min_n] = true;
	is_ok[v_min2_n] = true;
	is_ok[v_min3_n] = true;
	
	

	//块匹配连续性检查
	int err[6];
	int err_min = INT_MAX;
	int err_min_num;

	for(int i=0; i<6; i++)
	{
		if(!is_ok[i]) continue;
	
		err[i] = 0;

		for(int j=0; j<image1_sample_size.y; j++)
		{
			for(int k=0; k<image1_sample_size.x; k++)
			{
				err[i] += pow(	image2_gray.at<uchar>(min_err_idex[i] + j, start_col[i] - diff_x / 2 + min_err_dis[i] + k) - 
							 	image1_gray.at<uchar>(start_row[i] + j, start_col[i] + k), 2);
			}
		}

		if(err[i] < err_min)
		{
			err_min = err[i];
			err_min_num = i;
		}
	}


	//计算图像之间的拼接位置

	//y > 0, 表示第二张图片相对于第一张图片上移
	//y < 0, 表示第二张图片相对于第一张图片下移
	distance.y = min_err_idex[err_min_num] - start_row[err_min_num];
	
	//x > 0, 表示第二张图片相对于第一张图片右移
	//x < 0 ,表示第二张图片相对于第一张图片左移
	distance.x = diff_x / 2 - min_err_dis[err_min_num];

}


#define DEG_TO_RAD      (M_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / M_PI)

// scaling factor from 1e-7 degrees to meters at equater
// == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
#define LOCATION_SCALING_FACTOR 0.011131884502145034f
// inverse of LOCATION_SCALING_FACTOR
#define LOCATION_SCALING_FACTOR_INV 89.83204953368922f


bool Is_zero(float a)
{
	return std::fabs(a) < 1.0e-6f ? true : false;
}





float Constrain_float(float amt, float low, float high) 
{
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}


float Longitude_scale(const struct Location &loc)
{
    float scale = std::cos(loc.lat * 1.0e-7f * DEG_TO_RAD);
    return Constrain_float(scale, 0.01f, 1.0f);
}

/*
 *  extrapolate latitude/longitude given distances north and east
 */
void Location_offset(struct Location &loc, float ofs_north, float ofs_east)
{
    if (!Is_zero(ofs_north) || !Is_zero(ofs_east)) {
        int dlat = ofs_north * LOCATION_SCALING_FACTOR_INV;
        int dlng = (ofs_east * LOCATION_SCALING_FACTOR_INV) / Longitude_scale(loc);
        loc.lat += dlat;
        loc.lng += dlng;
    }
}



/*
 *  extrapolate latitude/longitude given bearing and distance
 * Note that this function is accurate to about 1mm at a distance of 
 * 100m. This function has the advantage that it works in relative
 * positions, so it keeps the accuracy even when dealing with small
 * distances and floating point numbers
 */
void Location_update(struct Location &loc, float bearing, float distance)
{
    float ofs_north = std::cos(bearing * DEG_TO_RAD)*distance;
    float ofs_east  = std::sin(bearing * DEG_TO_RAD)*distance;
    Location_offset(loc, ofs_north, ofs_east);
}



// return bearing in centi-degrees between two locations
float Get_bearing_cd(const struct Location &loc1, const struct Location &loc2)
{
    int off_x = loc2.lng - loc1.lng;
    int off_y = (loc2.lat - loc1.lat) / Longitude_scale(loc2);
    int bearing = 9000 + std::atan2(-off_y, off_x) * 5729.57795f;
    if (bearing < 0) bearing += 36000;
    return (float)bearing / 100.0f;
}






// return distance in meters between two locations
float Get_distance(const struct Location &loc1, const struct Location &loc2)
{
    float dlat              = (float)(loc2.lat - loc1.lat);
    float dlong             = ((float)(loc2.lng - loc1.lng)) * Longitude_scale(loc2);
    return std::sqrt(std::pow(dlat, 2)  + std::pow(dlong, 2)) * LOCATION_SCALING_FACTOR;
}



#define NARROW_SCALE    8


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


	//航线方向
	float way_line_angle = 92.0f;
	//地图画布
	Mat map;
	//地图画布每个像素点对应的比例尺
	float scale;


	//每张图片在地图上画布上的位置
	vector<Point2i> photo_on_map;

	//画布原点对应的gps 坐标
	struct Location map_origin;

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

		resize(image, image, Size(image.cols / NARROW_SCALE, image.rows / NARROW_SCALE),INTER_AREA);

		Image_rotate(image, image, way_line_angle - imu_data[num].yaw);

		strFile.clear();
		strFile = "./resize_image/";
		strFile += image_name[num];

		imwrite(strFile.c_str(), image);

		if(num == 1)
		{
			
			strFile.clear();
			strFile = "./resize_image/";
			strFile += image_name[num - 1];
			Mat image_last = imread(strFile.c_str());

			if(image_last.empty())
			{
				cout << "failed to load:" << strFile << endl;
				return -1;
			}

			Mat image_down, image_last_up;
			image_down = image(Range(image.rows / 2, image.rows),
													Range(0, image.cols));

			image_last_up = image_last(Range(0, image_last.rows / 2),
													Range(0, image_last.cols));


			Mat image_blur, image_last_blur;
			bilateralFilter(image_down, image_blur,15,100,3);
			bilateralFilter(image_last_up, image_last_blur,15,100,3);

			
			Point2i point_test;
			Image_fast_mosaic_algorithm(image_last_blur, image_blur,point_test);
			point_test.y += image.rows / 2;

			cout << "point test x:" << point_test.x << ", y:" << point_test.y << endl;


			//通过拼接位置求出两个图像中心像素点的距离
			float image_center_distance = sqrt(pow(point_test.x, 2)  + pow(point_test.y, 2));
			float gps_center_distance = Get_distance(gps_data[0], gps_data[1]);

			scale = gps_center_distance / image_center_distance;

			cout << "scale :" << scale << endl;

			//这里应该根据航区的大小申请画布的大小
			map.create(12000, 4000, CV_8UC3);
			map.setTo(0);


			//第一张图片贴的位置
			Point2i dest_point; 

			dest_point.x = map.cols / 2 - image_last.cols / 2;
			dest_point.y = map.rows -  2 * image_last.rows;


			photo_on_map.push_back(dest_point);

			image_last.copyTo(map(Rect(dest_point.x , dest_point.y, image_last.cols, image_last.rows)));

			//通过第一张图片的位置计算map (0,0) 的gps坐标
			
			float diff_x = (float)dest_point.x + (float)image_last.cols / 2.0f;
			float diff_y = (float)dest_point.y + (float)image_last.rows / 2.0f;

			float origin_first_image_distance = sqrt(pow(diff_x, 2) + pow(diff_y, 2)) * scale;

			float tmp_bearing = way_line_angle - atan2(diff_x, diff_y) * (180.0f / M_PI);
	
			map_origin.alt = gps_data[0].alt;
			map_origin.lat = gps_data[0].lat;
			map_origin.lng = gps_data[0].lng;

			Location_update(map_origin, tmp_bearing, origin_first_image_distance);
		}

		if(num >= 1){
			float distance = Get_distance(map_origin, gps_data[num]) / scale;
			float bearing = Get_bearing_cd(map_origin, gps_data[num]);


			Point2i image_point;
			image_point.x = (int)(distance * sin((way_line_angle + 180 - bearing) * (M_PI / 180.0f)) - (float)image.cols / 2);
			image_point.y = (int)(distance * cos((way_line_angle + 180 - bearing) * (M_PI / 180.0f)) - (float)image.rows / 2);

			image.copyTo(map(Rect(image_point.x , image_point.y, image.cols, image.rows)));
		}
		
	}



	imwrite("map.jpg", map);

	waitKey();
	cout << "I am ok" << endl;

	return 0;
}

