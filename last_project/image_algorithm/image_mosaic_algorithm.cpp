#include "image_algorithm.h"
#include <complex>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>



namespace IMAGE_MOSAIC
{

#define  DUBUG

#ifdef DUBUG
static int num_image = 0;
#endif

#define DEG_TO_RAD      (M_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / M_PI)

// scaling factor from 1e-7 degrees to meters at equater
// == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
#define LOCATION_SCALING_FACTOR 0.011131884502145034f
// inverse of LOCATION_SCALING_FACTOR
#define LOCATION_SCALING_FACTOR_INV 89.83204953368922f


bool Image_algorithm::Is_zero(float a)
{
	return std::fabs(a) < 1.0e-6f ? true : false;
}


/*
 *  extrapolate latitude/longitude given distances north and east
 */
void Image_algorithm::Location_offset(struct Location &loc, float ofs_north, float ofs_east)
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
void Image_algorithm::Location_update(struct Location &loc, float bearing, float distance)
{
    float ofs_north = std::cos(bearing * DEG_TO_RAD)*distance;
    float ofs_east  = std::sin(bearing * DEG_TO_RAD)*distance;
    Location_offset(loc, ofs_north, ofs_east);
}



// return bearing in centi-degrees between two locations
float Image_algorithm::Get_bearing_cd(const struct Location &loc1, const struct Location &loc2)
{
    int off_x = loc2.lng - loc1.lng;
    int off_y = (loc2.lat - loc1.lat) / Longitude_scale(loc2);
    int bearing = 9000 + std::atan2(-off_y, off_x) * 5729.57795f;
    if (bearing < 0) bearing += 36000;
    return (float)bearing / 100.0f;
}



float Image_algorithm::Constrain_float(float amt, float low, float high) 
{
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}


float Image_algorithm::Longitude_scale(const struct Location &loc)
{
    float scale = std::cos(loc.lat * 1.0e-7f * DEG_TO_RAD);
    return Constrain_float(scale, 0.01f, 1.0f);
}


// return distance in meters between two locations
float Image_algorithm::Get_distance(const struct Location &loc1, const struct Location &loc2)
{
    float dlat              = (float)(loc2.lat - loc1.lat);
    float dlong             = ((float)(loc2.lng - loc1.lng)) * Longitude_scale(loc2);
    return std::sqrt(std::pow(dlat, 2)  + std::pow(dlong, 2)) * LOCATION_SCALING_FACTOR;
}


void Image_algorithm::Image_rotate(cv::Mat& src_image,  cv::Mat& dest_image, double angle)
{
	cv::Point2f pt(src_image.cols/2, src_image.rows/2);
	cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);
	cv::warpAffine(src_image, dest_image, r, cv::Size(src_image.cols, src_image.rows));
}


void Image_algorithm::Image_resize(cv::Mat& src_image, cv::Mat& dest_image, cv::Size dsize)
{
	cv::resize(src_image, dest_image, dsize,cv::INTER_AREA);
}


void Image_algorithm::Image_fast_mosaic_algorithm2(cv::Mat &src_image1, cv::Mat &src_image2, cv::Point2i &distance)
{
	//需要先转换成灰度图像
	cv::Mat image1_gray, image2_gray;
	cv::cvtColor(src_image1, image1_gray, CV_RGB2GRAY);
	cv::cvtColor(src_image2, image2_gray, CV_RGB2GRAY);
	
	cv::Point2i image_size(image1_gray.cols, image1_gray.rows);
	cv::Point2i image1_sample_size;
	
	//图像1 和图像2  在x 方向上可能移动的最大距离 diff_x
	int diff_x;
	diff_x = image_size.x / 20;
	image1_sample_size.x = image_size.x / 4;
	image1_sample_size.y = image_size.y / 6;
		
	cv::Point2i image2_sample_size(image1_sample_size.x + diff_x, image1_sample_size.y);
	
	int start_row[4] = {image1_gray.rows / 6,
						image1_gray.rows / 6 +	image1_sample_size.y + 10, 
						image1_gray.rows / 6 + 2 * (image1_sample_size.y + 10),
						image1_gray.rows / 6 + 3 * (image1_sample_size.y + 10)};
	
	int start_col[4] = {image1_gray.cols / 2 - image1_sample_size.x / 2,
						diff_x / 2,
						image1_gray.cols - diff_x / 2 - image1_sample_size.x,
						image1_gray.cols / 2 - image1_sample_size.x / 2};
	
#ifdef DUBUG
	std::cout << "image_mosaic_algorithm image1 cols:" << image1_gray.cols << ", rows:" << image1_gray.rows << std::endl;
	std::cout << "image1_sample_size x:" << image1_sample_size.x << ", y:" << image1_sample_size.y << std::endl;
	std::cout << "diff_x:" << diff_x << std::endl;
	std::cout << "image2_sample_size x:" << image2_sample_size.x << ", y:" << image2_sample_size.y << std::endl;
	std::cout << "start row, 1:" << start_row[0] << ", 2:" << start_row[1] << ", 3:" << start_row[2]  << ", 4:" << start_row[3]<< std::endl;
	std::cout << "start col, 1:" << start_col[0] << ", 2:" << start_col[1] << ", 3:" << start_col[2] << ", 4" << start_row[3] << std::endl;
#endif
	
	int min_err[4];
	int min_err_idex[4];
	int min_err_dis[4];
	
	for(int i=0; i<4; i++)
	{
		min_err[i] = INT_MAX;
		min_err_idex[i] = 0;
		min_err_dis[i] = 0;
	}
	
	//分别查找4 组中最小二乘的位置
	for(int i=0; i<4; i++)
	{
		//计算图像 1  的匹配模板
		int base[image1_sample_size.x];
	
		for(int k=0; k<image1_sample_size.x; k++)
		{
			base[k] = image1_gray.at<uchar>(start_row[i], start_col[i] + k) - image1_gray.at<uchar>(start_row[i] + image1_sample_size.y, start_col[i] + k);
		}
	
		//找出图像2  的最佳匹配
		int num = image2_gray.rows	- image1_sample_size.y;
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
	
	//块匹配连续性检查
	int err[4];
	int err_min = INT_MAX;
	int err_min_num;
	for(int i=0; i<4; i++)
	{
		if(std::abs(min_err_idex[i] - start_row[i]) > 100) continue;
		if(std::abs(diff_x / 2 - min_err_dis[i]) > 100) continue;
	
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

	if(err_min == INT_MAX)
	{
		return;
	}
	
	//计算图像之间的拼接位置
	
	//y > 0, 表示第二张图片相对于第一张图片上移
	//y < 0, 表示第二张图片相对于第一张图片下移
	distance.y = min_err_idex[err_min_num] - start_row[err_min_num];
		
	//x > 0, 表示第二张图片相对于第一张图片右移
	//x < 0 ,表示第二张图片相对于第一张图片左移
	distance.x = diff_x / 2 - min_err_dis[err_min_num];
	
#ifdef DUBUG
	std::cout <<"err min num:" << err_min_num << ",err min:" << err_min << std::endl;
	
	for(int i=0; i<4; i++)
	{
		std::cout << i <<",min err:" << min_err[i] << ",min err dis:" << min_err_dis[i] << ",min err idex:" << min_err_idex[i] << std::endl;
	
		for(int j=0; j<image1_sample_size.x; j++)
		{
			image1_gray.at<uchar>(start_row[i], start_col[i] + j) = 255;
			image1_gray.at<uchar>(start_row[i] + image1_sample_size.y, start_col[i] + j) = 255;
			
			image2_gray.at<uchar>(min_err_idex[i], start_col[i] - diff_x / 2 + min_err_dis[i] + j) = 255;
			image2_gray.at<uchar>(min_err_idex[i] + image2_sample_size.y, start_col[i] - diff_x / 2 + min_err_dis[i] + j) = 255;
		}
	}
	
	std::string dir = "./gray_image";
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
			return;
		}
	}
	
	std::stringstream ss1, ss2;
	std::string s1, s2;
	std::string strName1 = "./gray_image/";
	ss1 << num_image;
	ss1 >> s1;
	num_image++;
	strName1 += s1;
	strName1 += ".jpg";
	
	
	std::string strName2 = "./gray_image/";
	ss2 << num_image;
	ss2 >> s2;
	num_image++;
	strName2 += s2;
	strName2 += ".jpg";
	
	std::cout << "strName1" << strName1 << std::endl;
	std::cout << "strName2" << strName2 << std::endl;
	
	cv::imwrite(strName1.c_str(), image1_gray);
	cv::imwrite(strName2.c_str(), image2_gray);
#endif
}



void Image_algorithm::Image_fast_mosaic_algorithm(cv::Mat &src_image1, cv::Mat &src_image2, cv::Point2i &distance)
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

#ifdef DUBUG
	std::cout << "image_mosaic_algorithm image1 cols:" << image1_gray.cols << ", rows:" << image1_gray.rows << std::endl;
	std::cout << "image1_sample_size x:" << image1_sample_size.x << ", y:" << image1_sample_size.y << std::endl;
	std::cout << "diff_x:" << diff_x << std::endl;
	std::cout << "image2_sample_size x:" << image2_sample_size.x << ", y:" << image2_sample_size.y << std::endl;
	std::cout << "start row, 1:" << start_row[0] << ", 2:" << start_row[1] << ", 3:" << start_row[2]  << ", 4:" << start_row[3]<< std::endl;
	std::cout << "start col, 1:" << start_col[0] << ", 2:" << start_col[1] << ", 3:" << start_col[2] << ", 4" << start_row[3] << std::endl;
#endif

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

	std::cout << "==========================" << std::endl;
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

		std::cout << "x:"<< i << "," << v_dis_x[i] << ",y:" << v_dis_y[i] << std::endl;
	}

	std::cout << "v:" << v_min_n << ",v2:" << v_min2_n << ", v3:" << v_min3_n << std::endl;

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

#ifdef DUBUG
	std::cout <<"err min num:" << err_min_num << std::endl;

	for(int i=0; i<6; i++)
	{
		std::cout << i <<",min err:" << min_err[i] << ",min err dis:" << min_err_dis[i] << ",min err idex:" << min_err_idex[i] << std::endl;

		for(int j=0; j<image1_sample_size.x; j++)
		{
			image1_gray.at<uchar>(start_row[i], start_col[i] + j) = 255;
			image1_gray.at<uchar>(start_row[i] + image1_sample_size.y, start_col[i] + j) = 255;
		
			image2_gray.at<uchar>(min_err_idex[i], start_col[i] - diff_x / 2 + min_err_dis[i] + j) = 255;
			image2_gray.at<uchar>(min_err_idex[i] + image2_sample_size.y, start_col[i] - diff_x / 2 + min_err_dis[i] + j) = 255;
		}
	}

	std::string dir = "./gray_image";
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
			return;
		}
	}

	std::stringstream ss1, ss2;
	std::string s1, s2;
	std::string strName1 = "./gray_image/";
	ss1 << num_image;
	ss1 >> s1;
	num_image++;
	strName1 += s1;
	strName1 += ".jpg";


	std::string strName2 = "./gray_image/";
	ss2 << num_image;
	ss2 >> s2;
	num_image++;
	strName2 += s2;
	strName2 += ".jpg";

	std::cout << "strName1" << strName1 << std::endl;
	std::cout << "strName2" << strName2 << std::endl;

	cv::imwrite(strName1.c_str(), image1_gray);
	cv::imwrite(strName2.c_str(), image2_gray);
#endif
}

} //namespace IMAGE_MOSAIC

