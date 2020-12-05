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
	//��Ҫ��ת���ɻҶ�ͼ��
	cv::Mat image1_gray, image2_gray;
	cv::cvtColor(src_image1, image1_gray, CV_RGB2GRAY);
	cv::cvtColor(src_image2, image2_gray, CV_RGB2GRAY);

	cv::Point2i image_size(image1_gray.cols, image1_gray.rows);
	cv::Point2i image1_sample_size;

	//ͼ��1 ��ͼ��2  ��x �����Ͽ����ƶ��������� diff_x
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

	//�ֱ����4 ������С���˵�λ��
	for(int i=0; i<6; i++)
	{
		//����ͼ�� 1  ��ƥ��ģ��
		int base[image1_sample_size.x];

		for(int k=0; k<image1_sample_size.x; k++)
		{
			base[k] = image1_gray.at<uchar>(start_row[i], start_col[i] + k) - image1_gray.at<uchar>(start_row[i] + image1_sample_size.y, start_col[i] + k);
		}

		//�ҳ�ͼ��2  �����ƥ��
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

			//��ÿһ�к͵�һ��ͼ�����С���˵����λ�ú�ֵ
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

	//�󷽲�

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
		//�����Сֵ
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
	
	

	//��ƥ�������Լ��
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


	//����ͼ��֮���ƴ��λ��

	//y > 0, ��ʾ�ڶ���ͼƬ����ڵ�һ��ͼƬ����
	//y < 0, ��ʾ�ڶ���ͼƬ����ڵ�һ��ͼƬ����
	distance.y = min_err_idex[err_min_num] - start_row[err_min_num];
	
	//x > 0, ��ʾ�ڶ���ͼƬ����ڵ�һ��ͼƬ����
	//x < 0 ,��ʾ�ڶ���ͼƬ����ڵ�һ��ͼƬ����
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
float Get_bearing(const struct Location &loc1, const struct Location &loc2)
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




double wrap_360(const double angle)
{
    double res = fmod(angle, 360.0);
    if (res < 0) {
        res += 360.0;
    }
    return res;
}



#define NARROW_SCALE    8
#define X_WIDTH		900
#define Y_WIDTH		600



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


	strFile = "../plane_image/gps_imu.txt";

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

			struct Location gps(alt * 100, lat * 1.0e7, lng * 1.0e7);
			gps_data.push_back(gps);

			struct Imu_data imu(pitch, roll, yaw);
			imu_data.push_back(imu);

        }
		
	}

	for(auto gps:gps_data)
	{
		cout << gps.alt << "\t" << gps.lat << "\t" << gps.lng << endl;
	}

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


	//���߷���
	float way_line_angle = 189.0f;
	//��ͼ����
	Mat map;
	//��ͼ����ÿ�����ص��Ӧ�ı�����
	float scale;


	//ÿ��ͼƬ�ڵ�ͼ�ϻ����ϵ�λ��
	vector<Point2i> photo_on_map;

	//����ԭ���Ӧ��gps ����
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

			Mat image_up, image_last_down;
			image_up = image(Range(0, image.rows / 2),
													Range(0, image.cols));

			image_last_down = image_last(Range(image_last.rows / 2, image_last.rows),
													Range(0, image_last.cols));


			Mat image_blur, image_last_blur;
			bilateralFilter(image_up, image_blur,15,100,3);
			bilateralFilter(image_last_down, image_last_blur,15,100,3);

			
			Point2i point_test;
			Image_fast_mosaic_algorithm(image_last_blur, image_blur,point_test);
			point_test.y -= image.rows / 2;

			cout << "point test x:" << point_test.x << ", y:" << point_test.y << endl;


			//ͨ��ƴ��λ���������ͼ���������ص�ľ���
			float image_center_distance = sqrt(pow(point_test.x, 2)  + pow(point_test.y, 2));
			float gps_center_distance = Get_distance(gps_data[0], gps_data[1]);

			scale = gps_center_distance / image_center_distance;

			cout << "scale :" << scale << endl;

			//����Ӧ�ø��ݺ����Ĵ�С���뻭���Ĵ�С
			//(y,x)  (row, col)
			map.create(5000, 4000, CV_8UC3);
			map.setTo(0);


			//��һ��ͼƬ����λ��
			Point2i dest_point; 

			dest_point.x = 2000;
			dest_point.y = 1000;


			photo_on_map.push_back(dest_point);

			image_last.copyTo(map(Rect(dest_point.x , dest_point.y, image_last.cols, image_last.rows)));

			//ͨ����һ��ͼƬ��λ�ü���map (0,0) ��gps����
			
			float diff_x = (float)dest_point.x + (float)image_last.cols / 2.0f;
			float diff_y = (float)dest_point.y + (float)image_last.rows / 2.0f;

			float origin_first_image_distance = sqrt(pow(diff_x, 2) + pow(diff_y, 2)) * scale;

			float tmp_bearing = way_line_angle - 180.0f - atan2(diff_x, diff_y) * (180.0f / M_PI);
	
			map_origin.alt = gps_data[0].alt;
			map_origin.lat = gps_data[0].lat;
			map_origin.lng = gps_data[0].lng;

			Location_update(map_origin, tmp_bearing, origin_first_image_distance);

			
		}

		if(num >= 1)
		{
			float distance = Get_distance(map_origin, gps_data[num]) / scale;
			float bearing = Get_bearing(map_origin, gps_data[num]);


			Point2i image_point;
			image_point.x = (int)(distance * sin((way_line_angle  - bearing) * (M_PI / 180.0f)) - (float)image.cols / 2);
			image_point.y = (int)(distance * cos((way_line_angle  - bearing) * (M_PI / 180.0f)) - (float)image.rows / 2);


			//����pos ����˼·:  ��ǰ��������ӽ���gps ����
			//ȷ��������ƴ��ͼƬ������ƴ��ͼƬ
			//Ϊ�˼�����������ͼƬƴ��ʧ�ܵĿ����ԣ�
			// ���б�Ҫ�������ص�ľ���, �ж��ص����Ƿ����50 %

			int dst_min1 = INT_MAX;
			int dst_min2 = INT_MAX;
			int min1_num = -1;
			int min2_num = -1;


			int right = -1;
			int down = -1;
			int up = -1;

			for(int i=0; i<num; i++)
			{
				float dst = Get_distance(gps_data[i], gps_data[num]);

				if(dst < dst_min1)
				{
					dst_min2 = dst_min1;
					min2_num = min1_num;

					dst_min1 = dst;
					min1_num = i;
				}
				else if(dst < dst_min2)
				{
					dst_min2 = dst;
					min2_num = i;
				}
			}

			float err_bearing;
			float min_bearing = Get_bearing(gps_data[min1_num], gps_data[num]);
			err_bearing  = fabs(min_bearing - way_line_angle);
			//cout<< "min1 err:" << err_bearing << endl;
			if(err_bearing < 5 || err_bearing > 355)
			{
				up = min1_num;
			}
			else if(err_bearing > 175 && err_bearing < 185)
			{
				down = min1_num;
			}
			else if(err_bearing > 60 && err_bearing < 120)
			{
				//�������ͨ���������ж����ң���������ͼƬ�����ұ�
				right = min1_num;
			}

			if(min2_num != -1)
			{
				min_bearing = Get_bearing(gps_data[min2_num], gps_data[num]);
			
				err_bearing  = fabs(min_bearing - way_line_angle);

				//cout<< "min2 err:" << err_bearing << endl;

				if(down == -1 && up == -1)
				{
					if(err_bearing < 5 || err_bearing > 355)
					{
						up = min2_num;
					}
					else if(err_bearing > 175 && err_bearing < 185)
					{
						down = min2_num;
					}
				}
				else if(right == -1)
				{
					if(err_bearing > 60 && err_bearing < 120)
					{
						right = min2_num;
					}
				}

			}


			if(down >= 0)
			{
				cout << "down  is:" << image_name[down] << endl;
			}

			if(up >= 0)
			{
				cout << "up is:" << image_name[up] << endl;
			}

			if(right >= 0)
			{
				cout << "right is:" << image_name[right] << endl;
			}

			if(up >= 0){
				//�ں�λ������
				float width_y, width_x;
				int sample1_start_rows, sample1_end_rows, sample1_start_cols, sample1_end_cols;
				int sample2_start_rows, sample2_end_rows, sample2_start_cols, sample2_end_cols;

				width_y = image.rows - abs(image_point.y - photo_on_map[up].y);
				width_x = image.cols - abs(image_point.x - photo_on_map[up].x);

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

				if(photo_on_map[up].y < image_point.y)
				{
					sample1_start_rows = width_y / 2  - w_y;
					sample1_end_rows = width_y / 2 + w_y;

					sample2_start_rows = abs(image_point.y - photo_on_map[up].y) + width_y / 2  - w_y;
					sample2_end_rows = abs(image_point.y - photo_on_map[up].y) + width_y / 2 + w_y;
				}
				else
				{
					sample1_start_rows = abs(image_point.y - photo_on_map[up].y) + width_y / 2  - w_y;
					sample1_end_rows = abs(image_point.y - photo_on_map[up].y) + width_y / 2 + w_y;

					sample2_start_rows = width_y / 2  - w_y;
					sample2_end_rows = width_y / 2 + w_y;
				}


				
				if(photo_on_map[up].x < image_point.x)
				{
					sample1_start_cols = width_x / 2 - w_x;
					sample1_end_cols = width_x / 2 + w_x;
				
					sample2_start_cols = abs(image_point.x - photo_on_map[up].x) + width_x / 2 - w_x;
					sample2_end_cols = abs(image_point.x - photo_on_map[up].x) + width_x / 2 + w_x;
				}
				else
				{
					sample1_start_cols = abs(image_point.x - photo_on_map[up].x) + width_x / 2 - w_x;
					sample1_end_cols = abs(image_point.x - photo_on_map[up].x) + width_x / 2 + w_x;
							
					sample2_start_cols = width_x / 2 - w_x;
					sample2_end_cols = width_x / 2 + w_x;
				}




				Mat sample1_image = image(cv::Range(sample1_start_rows, sample1_end_rows),
													cv::Range(sample1_start_cols, sample1_end_cols));


				strFile.clear();
				strFile = "./resize_image/";
				strFile += image_name[up];
				Mat image_up = imread(strFile.c_str());

				if(image_up.empty())
				{
					cout << "failed to load:" << strFile << endl;
					return -1;
				}
		
				Mat sample2_image = image_up(cv::Range(sample2_start_rows, sample2_end_rows),
													cv::Range(sample2_start_cols, sample2_end_cols));

				
				//��ͼƬ����ģ������
				Mat blur_image1, blur_image2;

				//˫���˲�
				bilateralFilter(sample1_image, blur_image1,15,100,3);
				bilateralFilter(sample2_image, blur_image2,15,100,3);

				Point2i sample_diff;
				Image_fast_mosaic_algorithm(blur_image2, blur_image1, sample_diff);
		
				image_point.y -= sample_diff.y;
				image_point.x += sample_diff.x;

			}


			if(down >= 0)
			{
				//�ں�λ������
				float width_y, width_x;
				int sample1_start_rows, sample1_end_rows, sample1_start_cols, sample1_end_cols;
				int sample2_start_rows, sample2_end_rows, sample2_start_cols, sample2_end_cols;

				width_y = image.rows - abs(image_point.y - photo_on_map[down].y);
				width_x = image.cols - abs(image_point.x - photo_on_map[down].x);

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

				if(photo_on_map[down].y < image_point.y)
				{
					sample1_start_rows = width_y / 2  - w_y;
					sample1_end_rows = width_y / 2 + w_y;

					sample2_start_rows = abs(image_point.y - photo_on_map[down].y) + width_y / 2  - w_y;
					sample2_end_rows = abs(image_point.y - photo_on_map[down].y) + width_y / 2 + w_y;
				}
				else
				{
					sample1_start_rows = abs(image_point.y - photo_on_map[down].y) + width_y / 2  - w_y;
					sample1_end_rows = abs(image_point.y - photo_on_map[down].y) + width_y / 2 + w_y;

					sample2_start_rows = width_y / 2  - w_y;
					sample2_end_rows = width_y / 2 + w_y;
				}


				
				if(photo_on_map[down].x < image_point.x)
				{
					sample1_start_cols = width_x / 2 - w_x;
					sample1_end_cols = width_x / 2 + w_x;
				
					sample2_start_cols = abs(image_point.x - photo_on_map[down].x) + width_x / 2 - w_x;
					sample2_end_cols = abs(image_point.x - photo_on_map[down].x) + width_x / 2 + w_x;
				}
				else
				{
					sample1_start_cols = abs(image_point.x - photo_on_map[down].x) + width_x / 2 - w_x;
					sample1_end_cols = abs(image_point.x - photo_on_map[down].x) + width_x / 2 + w_x;
							
					sample2_start_cols = width_x / 2 - w_x;
					sample2_end_cols = width_x / 2 + w_x;
				}




				Mat sample1_image = image(cv::Range(sample1_start_rows, sample1_end_rows),
													cv::Range(sample1_start_cols, sample1_end_cols));


				strFile.clear();
				strFile = "./resize_image/";
				strFile += image_name[down];
				Mat image_up = imread(strFile.c_str());

				if(image_up.empty())
				{
					cout << "failed to load:" << strFile << endl;
					return -1;
				}
		
				Mat sample2_image = image_up(cv::Range(sample2_start_rows, sample2_end_rows),
													cv::Range(sample2_start_cols, sample2_end_cols));

				
				//��ͼƬ����ģ������
				Mat blur_image1, blur_image2;

				//˫���˲�
				bilateralFilter(sample1_image, blur_image1,15,100,3);
				bilateralFilter(sample2_image, blur_image2,15,100,3);

				Point2i sample_diff;
				Image_fast_mosaic_algorithm(blur_image2, blur_image1, sample_diff);
		
				image_point.y -= sample_diff.y;
				image_point.x += sample_diff.x;
			}


			if(right >= 0)
			{
				//�ں�λ������
				float width_y, width_x;
				int sample1_start_rows, sample1_end_rows, sample1_start_cols, sample1_end_cols;
				int sample2_start_rows, sample2_end_rows, sample2_start_cols, sample2_end_cols;

				width_y = image.rows - abs(image_point.y - photo_on_map[right].y);
				width_x = image.cols - abs(image_point.x - photo_on_map[right].x);

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

				if(photo_on_map[right].y < image_point.y)
				{
					sample1_start_rows = width_y / 2  - w_y;
					sample1_end_rows = width_y / 2 + w_y;

					sample2_start_rows = abs(image_point.y - photo_on_map[right].y) + width_y / 2  - w_y;
					sample2_end_rows = abs(image_point.y - photo_on_map[right].y) + width_y / 2 + w_y;
				}
				else
				{
					sample1_start_rows = abs(image_point.y - photo_on_map[right].y) + width_y / 2  - w_y;
					sample1_end_rows = abs(image_point.y - photo_on_map[right].y) + width_y / 2 + w_y;

					sample2_start_rows = width_y / 2  - w_y;
					sample2_end_rows = width_y / 2 + w_y;
				}


				
				if(photo_on_map[right].x < image_point.x)
				{
					sample1_start_cols = width_x / 2 - w_x;
					sample1_end_cols = width_x / 2 + w_x;
				
					sample2_start_cols = abs(image_point.x - photo_on_map[right].x) + width_x / 2 - w_x;
					sample2_end_cols = abs(image_point.x - photo_on_map[right].x) + width_x / 2 + w_x;
				}
				else
				{
					sample1_start_cols = abs(image_point.x - photo_on_map[right].x) + width_x / 2 - w_x;
					sample1_end_cols = abs(image_point.x - photo_on_map[right].x) + width_x / 2 + w_x;
							
					sample2_start_cols = width_x / 2 - w_x;
					sample2_end_cols = width_x / 2 + w_x;
				}




				Mat sample1_image = image(cv::Range(sample1_start_rows, sample1_end_rows),
													cv::Range(sample1_start_cols, sample1_end_cols));


				strFile.clear();
				strFile = "./resize_image/";
				strFile += image_name[right];
				Mat image_up = imread(strFile.c_str());

				if(image_up.empty())
				{
					cout << "failed to load:" << strFile << endl;
					return -1;
				}
		
				Mat sample2_image = image_up(cv::Range(sample2_start_rows, sample2_end_rows),
													cv::Range(sample2_start_cols, sample2_end_cols));

				
				//��ͼƬ����ģ������
				Mat blur_image1, blur_image2;

				//˫���˲�
				bilateralFilter(sample1_image, blur_image1,15,100,3);
				bilateralFilter(sample2_image, blur_image2,15,100,3);

				Point2i sample_diff;
				Image_fast_mosaic_algorithm(blur_image2, blur_image1, sample_diff);
		
				image_point.y -= sample_diff.y;
				image_point.x += sample_diff.x;
			}

			photo_on_map.push_back(image_point);

			if(up >= 0 && right == -1)
			{
				//�ص��ϱ�1/4
				Mat dest_image = image(cv::Range(image.rows / 4, image.rows),
															cv::Range(0, image.cols));
				image_point.y += image.rows / 4;


				//ͼ���Ȩ�ں�
				int src_start_row = 0;
				int src_start_col = 0;
				int map_start_row = image_point.y + src_start_row;
				int map_start_col = image_point.x + src_start_col;
				float alpha = 1.0f;//map	�����ص�Ȩ��
				int w = 100; //�ӿ�  ֻ�ں�100 �����ص�
				for(int j=0; j<w; j++)
				{
					for(int k=0; k<dest_image.cols; k++)
					{
						Scalar color1 = map.at<Vec3b>(map_start_row + j, map_start_col + k);
						Scalar color2 = dest_image.at<Vec3b>(src_start_row + j, src_start_col + k);

						alpha = (float)(w-j) / (float)w;
						if(color1(0) == 0 && color1(1) == 0 && color1(2) == 0)
						{
							alpha = 0;
						}

						if(color2(0) == 0 && color2(1) == 0 && color2(2) == 0)
						{
							alpha = 1;
						}
		
						Scalar color3;
						color3(0) = color1(0) * alpha + color2(0) * (1 - alpha);
						color3(1) = color1(1) * alpha + color2(1) * (1 - alpha);
						color3(2) = color1(2) * alpha + color2(2) * (1 - alpha);
		
						dest_image.at<Vec3b>(src_start_row + j, src_start_col + k) = Vec3b(color3(0), color3(1), color3(2));
					}
					
				}


				dest_image.copyTo(map(Rect(image_point.x, image_point.y, dest_image.cols, dest_image.rows)));
			}
			else if(down == -1 && right >= 0)
			{
				// �ص��ұߵ�1/4
				int cut_size_right = image.cols / 4;
				
		
				Mat dest_image = image(cv::Range(0, image.rows),
												cv::Range(0, image.cols - cut_size_right));



				//���Ҽ�Ȩ�ں��ں�100 �����ص�
				int _w = 100;
				float _alpha = 1.0f;//map	�����ص�Ȩ��
				int src_start_rows = 0;
				int src_start_cols = dest_image.cols - _w;
				int map_start_rows = image_point.y + src_start_rows;
				int map_start_cols = image_point.x + src_start_cols;
		
				for(int j=0; j<dest_image.rows; j++)
				{
					for(int k=0; k<_w; k++)
					{
						Scalar color1 = map.at<Vec3b>(map_start_rows + j, map_start_cols + k);
						Scalar color2 = dest_image.at<Vec3b>(src_start_rows + j, src_start_cols + k);

						_alpha = (float)(k) / (float)_w;
						if(color1(0) == 0 && color1(1) == 0 && color1(2) == 0)
						{
							_alpha = 0;
						}

						if(color2(0) == 0 && color2(1) == 0 && color2(2) == 0)
						{
							_alpha = 1;
						}
				
						Scalar color3;
						color3(0) = color1(0) * _alpha + color2(0) * (1 - _alpha);
						color3(1) = color1(1) * _alpha + color2(1) * (1 - _alpha);
						color3(2) = color1(2) * _alpha + color2(2) * (1 - _alpha);
				
						dest_image.at<Vec3b>(src_start_rows + j, src_start_cols + k) = Vec3b(color3(0), color3(1), color3(2));
					}
				}
				
				dest_image.copyTo(map(Rect(image_point.x, image_point.y, dest_image.cols, dest_image.rows)));
			}
			else if(down >=0 && right >=0)
			{
				
				//�ص��±ߵ�1/4
				int cut_size_up = image.rows / 4;
				int cut_size_right = image.cols / 4;
				
				
				Mat dest_image = image(cv::Range(0, image.rows - cut_size_up),
																			cv::Range(0, image.cols - cut_size_right));

				//�Ƚ������¼�Ȩ�ں�
				int w = 100; //�ӿ�  ֻ�ں�100 �����ص�
				int src_start_row = dest_image.rows - w;
				int src_start_col = 0;
				int map_start_row = image_point.y + src_start_row;
				int map_start_col = image_point.x + src_start_col;
				float alpha = 1.0f;//map	�����ص�Ȩ��

				
				
				
				for(int j=0; j<w; j++)
				{
					for(int k=0; k<image.cols; k++)
					{
						Scalar color1 = map.at<Vec3b>(map_start_row + j, map_start_col + k);
						Scalar color2 = dest_image.at<Vec3b>(src_start_row + j, src_start_col + k);

						alpha = (float)(j) / (float)w;

						if(color1(0) == 0 && color1(1) == 0 && color1(2) == 0)
						{
							alpha = 0;
						}

						if(color2(0) == 0 && color2(1) == 0 && color2(2) == 0)
						{
							alpha = 1;
						}
		
						Scalar color3;
						color3(0) = color1(0) * alpha + color2(0) * (1 - alpha);
						color3(1) = color1(1) * alpha + color2(1) * (1 - alpha);
						color3(2) = color1(2) * alpha + color2(2) * (1 - alpha);
		
						dest_image.at<Vec3b>(src_start_row + j, src_start_col + k) = Vec3b(color3(0), color3(1), color3(2));
					}
				}

				//�ٽ������Ҽ�Ȩ�ں�
				//���Ҽ�Ȩ�ں��ں�20 �����ص�
				int _w = 20;
				float _alpha = 1.0f;//map	�����ص�Ȩ��
				int src_start_rows = 0;
				int src_start_cols = dest_image.cols - _w;
				int map_start_rows = image_point.y + src_start_rows;
				int map_start_cols = image_point.x + src_start_cols;
		
				for(int j=0; j<dest_image.rows; j++)
				{
					for(int k=0; k<_w; k++)
					{
						Scalar color1 = map.at<Vec3b>(map_start_rows + j, map_start_cols + k);
						Scalar color2 = dest_image.at<Vec3b>(src_start_rows + j, src_start_cols + k);

						_alpha = (float)(k) / (float)_w;

						if(color1(0) == 0 && color1(1) == 0 && color1(2) == 0)
						{
							_alpha = 0;
						}

						if(color2(0) == 0 && color2(1) == 0 && color2(2) == 0)
						{
							_alpha = 1;
						}
				
						Scalar color3;
						color3(0) = color1(0) * _alpha + color2(0) * (1 - _alpha);
						color3(1) = color1(1) * _alpha + color2(1) * (1 - _alpha);
						color3(2) = color1(2) * _alpha + color2(2) * (1 - _alpha);
				
						dest_image.at<Vec3b>(src_start_rows + j, src_start_cols + k) = Vec3b(color3(0), color3(1), color3(2));
					}
				}

				dest_image.copyTo(map(Rect(image_point.x , image_point.y, dest_image.cols, dest_image.rows)));

			}
			else
			{
				image.copyTo(map(Rect(image_point.x , image_point.y, image.cols, image.rows)));
			}
		}


	}



	imwrite("map.jpg", map);

	waitKey();
	cout << "I am ok" << endl;

	return 0;
}

