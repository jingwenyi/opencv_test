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

		}
	}

	

	waitKey();
	cout << "I am ok" << endl;

	return 0;
}

