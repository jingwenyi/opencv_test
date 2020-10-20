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




const int PATCH_SIZE = 31;
const int HALF_PATCH_SIZE = 15;
const int EDGE_THRESHOLD = 19;


static int bit_pattern_31_[256*4] =
{
	8,-3, 9,5/*mean (0), correlation (0)*/,
	4,2, 7,-12/*mean (1.12461e-05), correlation (0.0437584)*/,
	-11,9, -8,2/*mean (3.37382e-05), correlation (0.0617409)*/,
	7,-12, 12,-13/*mean (5.62303e-05), correlation (0.0636977)*/,
	2,-13, 2,12/*mean (0.000134953), correlation (0.085099)*/,
	1,-7, 1,6/*mean (0.000528565), correlation (0.0857175)*/,
	-2,-10, -2,-4/*mean (0.0188821), correlation (0.0985774)*/,
	-13,-13, -11,-8/*mean (0.0363135), correlation (0.0899616)*/,
	-13,-3, -12,-9/*mean (0.121806), correlation (0.099849)*/,
	10,4, 11,9/*mean (0.122065), correlation (0.093285)*/,
	-13,-8, -8,-9/*mean (0.162787), correlation (0.0942748)*/,
	-11,7, -9,12/*mean (0.21561), correlation (0.0974438)*/,
	7,7, 12,6/*mean (0.160583), correlation (0.130064)*/,
	-4,-5, -3,0/*mean (0.228171), correlation (0.132998)*/,
	-13,2, -12,-3/*mean (0.00997526), correlation (0.145926)*/,
	-9,0, -7,5/*mean (0.198234), correlation (0.143636)*/,
	12,-6, 12,-1/*mean (0.0676226), correlation (0.16689)*/,
	-3,6, -2,12/*mean (0.166847), correlation (0.171682)*/,
	-6,-13, -4,-8/*mean (0.101215), correlation (0.179716)*/,
	11,-13, 12,-8/*mean (0.200641), correlation (0.192279)*/,
	4,7, 5,1/*mean (0.205106), correlation (0.186848)*/,
	5,-3, 10,-3/*mean (0.234908), correlation (0.192319)*/,
	3,-7, 6,12/*mean (0.0709964), correlation (0.210872)*/,
	-8,-7, -6,-2/*mean (0.0939834), correlation (0.212589)*/,
	-2,11, -1,-10/*mean (0.127778), correlation (0.20866)*/,
	-13,12, -8,10/*mean (0.14783), correlation (0.206356)*/,
	-7,3, -5,-3/*mean (0.182141), correlation (0.198942)*/,
	-4,2, -3,7/*mean (0.188237), correlation (0.21384)*/,
	-10,-12, -6,11/*mean (0.14865), correlation (0.23571)*/,
	5,-12, 6,-7/*mean (0.222312), correlation (0.23324)*/,
	5,-6, 7,-1/*mean (0.229082), correlation (0.23389)*/,
	1,0, 4,-5/*mean (0.241577), correlation (0.215286)*/,
	9,11, 11,-13/*mean (0.00338507), correlation (0.251373)*/,
	4,7, 4,12/*mean (0.131005), correlation (0.257622)*/,
	2,-1, 4,4/*mean (0.152755), correlation (0.255205)*/,
	-4,-12, -2,7/*mean (0.182771), correlation (0.244867)*/,
	-8,-5, -7,-10/*mean (0.186898), correlation (0.23901)*/,
	4,11, 9,12/*mean (0.226226), correlation (0.258255)*/,
	0,-8, 1,-13/*mean (0.0897886), correlation (0.274827)*/,
	-13,-2, -8,2/*mean (0.148774), correlation (0.28065)*/,
	-3,-2, -2,3/*mean (0.153048), correlation (0.283063)*/,
	-6,9, -4,-9/*mean (0.169523), correlation (0.278248)*/,
	8,12, 10,7/*mean (0.225337), correlation (0.282851)*/,
	0,9, 1,3/*mean (0.226687), correlation (0.278734)*/,
	7,-5, 11,-10/*mean (0.00693882), correlation (0.305161)*/,
	-13,-6, -11,0/*mean (0.0227283), correlation (0.300181)*/,
	10,7, 12,1/*mean (0.125517), correlation (0.31089)*/,
	-6,-3, -6,12/*mean (0.131748), correlation (0.312779)*/,
	10,-9, 12,-4/*mean (0.144827), correlation (0.292797)*/,
	-13,8, -8,-12/*mean (0.149202), correlation (0.308918)*/,
	-13,0, -8,-4/*mean (0.160909), correlation (0.310013)*/,
	3,3, 7,8/*mean (0.177755), correlation (0.309394)*/,
	5,7, 10,-7/*mean (0.212337), correlation (0.310315)*/,
	-1,7, 1,-12/*mean (0.214429), correlation (0.311933)*/,
	3,-10, 5,6/*mean (0.235807), correlation (0.313104)*/,
	2,-4, 3,-10/*mean (0.00494827), correlation (0.344948)*/,
	-13,0, -13,5/*mean (0.0549145), correlation (0.344675)*/,
	-13,-7, -12,12/*mean (0.103385), correlation (0.342715)*/,
	-13,3, -11,8/*mean (0.134222), correlation (0.322922)*/,
	-7,12, -4,7/*mean (0.153284), correlation (0.337061)*/,
	6,-10, 12,8/*mean (0.154881), correlation (0.329257)*/,
	-9,-1, -7,-6/*mean (0.200967), correlation (0.33312)*/,
	-2,-5, 0,12/*mean (0.201518), correlation (0.340635)*/,
	-12,5, -7,5/*mean (0.207805), correlation (0.335631)*/,
	3,-10, 8,-13/*mean (0.224438), correlation (0.34504)*/,
	-7,-7, -4,5/*mean (0.239361), correlation (0.338053)*/,
	-3,-2, -1,-7/*mean (0.240744), correlation (0.344322)*/,
	2,9, 5,-11/*mean (0.242949), correlation (0.34145)*/,
	-11,-13, -5,-13/*mean (0.244028), correlation (0.336861)*/,
	-1,6, 0,-1/*mean (0.247571), correlation (0.343684)*/,
	5,-3, 5,2/*mean (0.000697256), correlation (0.357265)*/,
	-4,-13, -4,12/*mean (0.00213675), correlation (0.373827)*/,
	-9,-6, -9,6/*mean (0.0126856), correlation (0.373938)*/,
	-12,-10, -8,-4/*mean (0.0152497), correlation (0.364237)*/,
	10,2, 12,-3/*mean (0.0299933), correlation (0.345292)*/,
	7,12, 12,12/*mean (0.0307242), correlation (0.366299)*/,
	-7,-13, -6,5/*mean (0.0534975), correlation (0.368357)*/,
	-4,9, -3,4/*mean (0.099865), correlation (0.372276)*/,
	7,-1, 12,2/*mean (0.117083), correlation (0.364529)*/,
	-7,6, -5,1/*mean (0.126125), correlation (0.369606)*/,
	-13,11, -12,5/*mean (0.130364), correlation (0.358502)*/,
	-3,7, -2,-6/*mean (0.131691), correlation (0.375531)*/,
	7,-8, 12,-7/*mean (0.160166), correlation (0.379508)*/,
	-13,-7, -11,-12/*mean (0.167848), correlation (0.353343)*/,
	1,-3, 12,12/*mean (0.183378), correlation (0.371916)*/,
	2,-6, 3,0/*mean (0.228711), correlation (0.371761)*/,
	-4,3, -2,-13/*mean (0.247211), correlation (0.364063)*/,
	-1,-13, 1,9/*mean (0.249325), correlation (0.378139)*/,
	7,1, 8,-6/*mean (0.000652272), correlation (0.411682)*/,
	1,-1, 3,12/*mean (0.00248538), correlation (0.392988)*/,
	9,1, 12,6/*mean (0.0206815), correlation (0.386106)*/,
	-1,-9, -1,3/*mean (0.0364485), correlation (0.410752)*/,
	-13,-13, -10,5/*mean (0.0376068), correlation (0.398374)*/,
	7,7, 10,12/*mean (0.0424202), correlation (0.405663)*/,
	12,-5, 12,9/*mean (0.0942645), correlation (0.410422)*/,
	6,3, 7,11/*mean (0.1074), correlation (0.413224)*/,
	5,-13, 6,10/*mean (0.109256), correlation (0.408646)*/,
	2,-12, 2,3/*mean (0.131691), correlation (0.416076)*/,
	3,8, 4,-6/*mean (0.165081), correlation (0.417569)*/,
	2,6, 12,-13/*mean (0.171874), correlation (0.408471)*/,
	9,-12, 10,3/*mean (0.175146), correlation (0.41296)*/,
	-8,4, -7,9/*mean (0.183682), correlation (0.402956)*/,
	-11,12, -4,-6/*mean (0.184672), correlation (0.416125)*/,
	1,12, 2,-8/*mean (0.191487), correlation (0.386696)*/,
	6,-9, 7,-4/*mean (0.192668), correlation (0.394771)*/,
	2,3, 3,-2/*mean (0.200157), correlation (0.408303)*/,
	6,3, 11,0/*mean (0.204588), correlation (0.411762)*/,
	3,-3, 8,-8/*mean (0.205904), correlation (0.416294)*/,
	7,8, 9,3/*mean (0.213237), correlation (0.409306)*/,
	-11,-5, -6,-4/*mean (0.243444), correlation (0.395069)*/,
	-10,11, -5,10/*mean (0.247672), correlation (0.413392)*/,
	-5,-8, -3,12/*mean (0.24774), correlation (0.411416)*/,
	-10,5, -9,0/*mean (0.00213675), correlation (0.454003)*/,
	8,-1, 12,-6/*mean (0.0293635), correlation (0.455368)*/,
	4,-6, 6,-11/*mean (0.0404971), correlation (0.457393)*/,
	-10,12, -8,7/*mean (0.0481107), correlation (0.448364)*/,
	4,-2, 6,7/*mean (0.050641), correlation (0.455019)*/,
	-2,0, -2,12/*mean (0.0525978), correlation (0.44338)*/,
	-5,-8, -5,2/*mean (0.0629667), correlation (0.457096)*/,
	7,-6, 10,12/*mean (0.0653846), correlation (0.445623)*/,
	-9,-13, -8,-8/*mean (0.0858749), correlation (0.449789)*/,
	-5,-13, -5,-2/*mean (0.122402), correlation (0.450201)*/,
	8,-8, 9,-13/*mean (0.125416), correlation (0.453224)*/,
	-9,-11, -9,0/*mean (0.130128), correlation (0.458724)*/,
	1,-8, 1,-2/*mean (0.132467), correlation (0.440133)*/,
	7,-4, 9,1/*mean (0.132692), correlation (0.454)*/,
	-2,1, -1,-4/*mean (0.135695), correlation (0.455739)*/,
	11,-6, 12,-11/*mean (0.142904), correlation (0.446114)*/,
	-12,-9, -6,4/*mean (0.146165), correlation (0.451473)*/,
	3,7, 7,12/*mean (0.147627), correlation (0.456643)*/,
	5,5, 10,8/*mean (0.152901), correlation (0.455036)*/,
	0,-4, 2,8/*mean (0.167083), correlation (0.459315)*/,
	-9,12, -5,-13/*mean (0.173234), correlation (0.454706)*/,
	0,7, 2,12/*mean (0.18312), correlation (0.433855)*/,
	-1,2, 1,7/*mean (0.185504), correlation (0.443838)*/,
	5,11, 7,-9/*mean (0.185706), correlation (0.451123)*/,
	3,5, 6,-8/*mean (0.188968), correlation (0.455808)*/,
	-13,-4, -8,9/*mean (0.191667), correlation (0.459128)*/,
	-5,9, -3,-3/*mean (0.193196), correlation (0.458364)*/,
	-4,-7, -3,-12/*mean (0.196536), correlation (0.455782)*/,
	6,5, 8,0/*mean (0.1972), correlation (0.450481)*/,
	-7,6, -6,12/*mean (0.199438), correlation (0.458156)*/,
	-13,6, -5,-2/*mean (0.211224), correlation (0.449548)*/,
	1,-10, 3,10/*mean (0.211718), correlation (0.440606)*/,
	4,1, 8,-4/*mean (0.213034), correlation (0.443177)*/,
	-2,-2, 2,-13/*mean (0.234334), correlation (0.455304)*/,
	2,-12, 12,12/*mean (0.235684), correlation (0.443436)*/,
	-2,-13, 0,-6/*mean (0.237674), correlation (0.452525)*/,
	4,1, 9,3/*mean (0.23962), correlation (0.444824)*/,
	-6,-10, -3,-5/*mean (0.248459), correlation (0.439621)*/,
	-3,-13, -1,1/*mean (0.249505), correlation (0.456666)*/,
	7,5, 12,-11/*mean (0.00119208), correlation (0.495466)*/,
	4,-2, 5,-7/*mean (0.00372245), correlation (0.484214)*/,
	-13,9, -9,-5/*mean (0.00741116), correlation (0.499854)*/,
	7,1, 8,6/*mean (0.0208952), correlation (0.499773)*/,
	7,-8, 7,6/*mean (0.0220085), correlation (0.501609)*/,
	-7,-4, -7,1/*mean (0.0233806), correlation (0.496568)*/,
	-8,11, -7,-8/*mean (0.0236505), correlation (0.489719)*/,
	-13,6, -12,-8/*mean (0.0268781), correlation (0.503487)*/,
	2,4, 3,9/*mean (0.0323324), correlation (0.501938)*/,
	10,-5, 12,3/*mean (0.0399235), correlation (0.494029)*/,
	-6,-5, -6,7/*mean (0.0420153), correlation (0.486579)*/,
	8,-3, 9,-8/*mean (0.0548021), correlation (0.484237)*/,
	2,-12, 2,8/*mean (0.0616622), correlation (0.496642)*/,
	-11,-2, -10,3/*mean (0.0627755), correlation (0.498563)*/,
	-12,-13, -7,-9/*mean (0.0829622), correlation (0.495491)*/,
	-11,0, -10,-5/*mean (0.0843342), correlation (0.487146)*/,
	5,-3, 11,8/*mean (0.0929937), correlation (0.502315)*/,
	-2,-13, -1,12/*mean (0.113327), correlation (0.48941)*/,
	-1,-8, 0,9/*mean (0.132119), correlation (0.467268)*/,
	-13,-11, -12,-5/*mean (0.136269), correlation (0.498771)*/,
	-10,-2, -10,11/*mean (0.142173), correlation (0.498714)*/,
	-3,9, -2,-13/*mean (0.144141), correlation (0.491973)*/,
	2,-3, 3,2/*mean (0.14892), correlation (0.500782)*/,
	-9,-13, -4,0/*mean (0.150371), correlation (0.498211)*/,
	-4,6, -3,-10/*mean (0.152159), correlation (0.495547)*/,
	-4,12, -2,-7/*mean (0.156152), correlation (0.496925)*/,
	-6,-11, -4,9/*mean (0.15749), correlation (0.499222)*/,
	6,-3, 6,11/*mean (0.159211), correlation (0.503821)*/,
	-13,11, -5,5/*mean (0.162427), correlation (0.501907)*/,
	11,11, 12,6/*mean (0.16652), correlation (0.497632)*/,
	7,-5, 12,-2/*mean (0.169141), correlation (0.484474)*/,
	-1,12, 0,7/*mean (0.169456), correlation (0.495339)*/,
	-4,-8, -3,-2/*mean (0.171457), correlation (0.487251)*/,
	-7,1, -6,7/*mean (0.175), correlation (0.500024)*/,
	-13,-12, -8,-13/*mean (0.175866), correlation (0.497523)*/,
	-7,-2, -6,-8/*mean (0.178273), correlation (0.501854)*/,
	-8,5, -6,-9/*mean (0.181107), correlation (0.494888)*/,
	-5,-1, -4,5/*mean (0.190227), correlation (0.482557)*/,
	-13,7, -8,10/*mean (0.196739), correlation (0.496503)*/,
	1,5, 5,-13/*mean (0.19973), correlation (0.499759)*/,
	1,0, 10,-13/*mean (0.204465), correlation (0.49873)*/,
	9,12, 10,-1/*mean (0.209334), correlation (0.49063)*/,
	5,-8, 10,-9/*mean (0.211134), correlation (0.503011)*/,
	-1,11, 1,-13/*mean (0.212), correlation (0.499414)*/,
	-9,-3, -6,2/*mean (0.212168), correlation (0.480739)*/,
	-1,-10, 1,12/*mean (0.212731), correlation (0.502523)*/,
	-13,1, -8,-10/*mean (0.21327), correlation (0.489786)*/,
	8,-11, 10,-6/*mean (0.214159), correlation (0.488246)*/,
	2,-13, 3,-6/*mean (0.216993), correlation (0.50287)*/,
	7,-13, 12,-9/*mean (0.223639), correlation (0.470502)*/,
	-10,-10, -5,-7/*mean (0.224089), correlation (0.500852)*/,
	-10,-8, -8,-13/*mean (0.228666), correlation (0.502629)*/,
	4,-6, 8,5/*mean (0.22906), correlation (0.498305)*/,
	3,12, 8,-13/*mean (0.233378), correlation (0.503825)*/,
	-4,2, -3,-3/*mean (0.234323), correlation (0.476692)*/,
	5,-13, 10,-12/*mean (0.236392), correlation (0.475462)*/,
	4,-13, 5,-1/*mean (0.236842), correlation (0.504132)*/,
	-9,9, -4,3/*mean (0.236977), correlation (0.497739)*/,
	0,3, 3,-9/*mean (0.24314), correlation (0.499398)*/,
	-12,1, -6,1/*mean (0.243297), correlation (0.489447)*/,
	3,2, 4,-8/*mean (0.00155196), correlation (0.553496)*/,
	-10,-10, -10,9/*mean (0.00239541), correlation (0.54297)*/,
	8,-13, 12,12/*mean (0.0034413), correlation (0.544361)*/,
	-8,-12, -6,-5/*mean (0.003565), correlation (0.551225)*/,
	2,2, 3,7/*mean (0.00835583), correlation (0.55285)*/,
	10,6, 11,-8/*mean (0.00885065), correlation (0.540913)*/,
	6,8, 8,-12/*mean (0.0101552), correlation (0.551085)*/,
	-7,10, -6,5/*mean (0.0102227), correlation (0.533635)*/,
	-3,-9, -3,9/*mean (0.0110211), correlation (0.543121)*/,
	-1,-13, -1,5/*mean (0.0113473), correlation (0.550173)*/,
	-3,-7, -3,4/*mean (0.0140913), correlation (0.554774)*/,
	-8,-2, -8,3/*mean (0.017049), correlation (0.55461)*/,
	4,2, 12,12/*mean (0.01778), correlation (0.546921)*/,
	2,-5, 3,11/*mean (0.0224022), correlation (0.549667)*/,
	6,-9, 11,-13/*mean (0.029161), correlation (0.546295)*/,
	3,-1, 7,12/*mean (0.0303081), correlation (0.548599)*/,
	11,-1, 12,4/*mean (0.0355151), correlation (0.523943)*/,
	-3,0, -3,6/*mean (0.0417904), correlation (0.543395)*/,
	4,-11, 4,12/*mean (0.0487292), correlation (0.542818)*/,
	2,-4, 2,1/*mean (0.0575124), correlation (0.554888)*/,
	-10,-6, -8,1/*mean (0.0594242), correlation (0.544026)*/,
	-13,7, -11,1/*mean (0.0597391), correlation (0.550524)*/,
	-13,12, -11,-13/*mean (0.0608974), correlation (0.55383)*/,
	6,0, 11,-13/*mean (0.065126), correlation (0.552006)*/,
	0,-1, 1,4/*mean (0.074224), correlation (0.546372)*/,
	-13,3, -9,-2/*mean (0.0808592), correlation (0.554875)*/,
	-9,8, -6,-3/*mean (0.0883378), correlation (0.551178)*/,
	-13,-6, -8,-2/*mean (0.0901035), correlation (0.548446)*/,
	5,-9, 8,10/*mean (0.0949843), correlation (0.554694)*/,
	2,7, 3,-9/*mean (0.0994152), correlation (0.550979)*/,
	-1,-6, -1,-1/*mean (0.10045), correlation (0.552714)*/,
	9,5, 11,-2/*mean (0.100686), correlation (0.552594)*/,
	11,-3, 12,-8/*mean (0.101091), correlation (0.532394)*/,
	3,0, 3,5/*mean (0.101147), correlation (0.525576)*/,
	-1,4, 0,10/*mean (0.105263), correlation (0.531498)*/,
	3,-6, 4,5/*mean (0.110785), correlation (0.540491)*/,
	-13,0, -10,5/*mean (0.112798), correlation (0.536582)*/,
	5,8, 12,11/*mean (0.114181), correlation (0.555793)*/,
	8,9, 9,-6/*mean (0.117431), correlation (0.553763)*/,
	7,-4, 8,-12/*mean (0.118522), correlation (0.553452)*/,
	-10,4, -10,9/*mean (0.12094), correlation (0.554785)*/,
	7,3, 12,4/*mean (0.122582), correlation (0.555825)*/,
	9,-7, 10,-2/*mean (0.124978), correlation (0.549846)*/,
	7,0, 12,-2/*mean (0.127002), correlation (0.537452)*/,
	-1,-6, 0,-11/*mean (0.127148), correlation (0.547401)*/
};


static float IC_Angle(const cv::Mat& image, cv::Point2f pt,  const std::vector<int> & u_max)
{
    int m_01 = 0, m_10 = 0;

    const uchar* center = &image.at<uchar> (cvRound(pt.y), cvRound(pt.x));

    // Treat the center line differently, v=0
    for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
        m_10 += u * center[u];

    // Go line by line in the circuI853lar patch
    int step = (int)image.step1();
    for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
    {
        // Proceed over the two lines
        int v_sum = 0;
        int d = u_max[v];
        for (int u = -d; u <= d; ++u)
        {
            int val_plus = center[u + v*step], val_minus = center[u - v*step];
            v_sum += (val_plus - val_minus);
            m_10 += u * (val_plus + val_minus);
        }
        m_01 += v * v_sum;
    }

    return cv::fastAtan2((float)m_01, (float)m_10);
}



const float factorPI = (float)(M_PI/180.f);
static void computeOrbDescriptor(const cv::KeyPoint& kpt,
                                 const cv::Mat& img, const cv::Point* pattern,
                                 uchar* desc)
{
    float angle = (float)kpt.angle*factorPI;
    float a = (float)std::cos(angle), b = (float)std::sin(angle);

    const uchar* center = &img.at<uchar>(cvRound(kpt.pt.y), cvRound(kpt.pt.x));
    const int step = (int)img.step;

    #define GET_VALUE(idx) \
        center[static_cast<uchar>(cvRound(pattern[idx].x*b + pattern[idx].y*a)*step + \
               cvRound(pattern[idx].x*a - pattern[idx].y*b))]


    for (int i = 0; i < 32; ++i, pattern += 16)
    {
        int t0, t1, val;
        t0 = GET_VALUE(0); t1 = GET_VALUE(1);
        val = t0 < t1;
        t0 = GET_VALUE(2); t1 = GET_VALUE(3);
        val |= (t0 < t1) << 1;
        t0 = GET_VALUE(4); t1 = GET_VALUE(5);
        val |= (t0 < t1) << 2;
        t0 = GET_VALUE(6); t1 = GET_VALUE(7);
        val |= (t0 < t1) << 3;
        t0 = GET_VALUE(8); t1 = GET_VALUE(9);
        val |= (t0 < t1) << 4;
        t0 = GET_VALUE(10); t1 = GET_VALUE(11);
        val |= (t0 < t1) << 5;
        t0 = GET_VALUE(12); t1 = GET_VALUE(13);
        val |= (t0 < t1) << 6;
        t0 = GET_VALUE(14); t1 = GET_VALUE(15);
        val |= (t0 < t1) << 7;

        desc[i] = (uchar)val;
    }

    #undef GET_VALUE
}


static void computeOrientation(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const std::vector<int>& umax)
{
    for (std::vector<cv::KeyPoint>::iterator keypoint = keypoints.begin(),
         keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
    {
        keypoint->angle = IC_Angle(image, keypoint->pt, umax);
    }
}


static void computeDescriptors(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
                               const std::vector<cv::Point>& pattern)
{
    descriptors = cv::Mat::zeros((int)keypoints.size(), 32, CV_8UC1);

    for (size_t i = 0; i < keypoints.size(); i++)
        computeOrbDescriptor(keypoints[i], image, &pattern[0], descriptors.ptr((int)i));
}

void ExtractorNode::DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4)
{
    const int halfX = ceil(static_cast<float>(UR.x-UL.x)/2);
    const int halfY = ceil(static_cast<float>(BR.y-UL.y)/2);

    //Define boundaries of childs
    n1.UL = UL;
    n1.UR = cv::Point2i(UL.x+halfX,UL.y);
    n1.BL = cv::Point2i(UL.x,UL.y+halfY);
    n1.BR = cv::Point2i(UL.x+halfX,UL.y+halfY);
    n1.vKeys.reserve(vKeys.size());

    n2.UL = n1.UR;
    n2.UR = UR;
    n2.BL = n1.BR;
    n2.BR = cv::Point2i(UR.x,UL.y+halfY);
    n2.vKeys.reserve(vKeys.size());

    n3.UL = n1.BL;
    n3.UR = n1.BR;
    n3.BL = BL;
    n3.BR = cv::Point2i(n1.BR.x,BL.y);
    n3.vKeys.reserve(vKeys.size());

    n4.UL = n3.UR;
    n4.UR = n2.BR;
    n4.BL = n3.BR;
    n4.BR = BR;
    n4.vKeys.reserve(vKeys.size());

    //Associate points to childs
    for(size_t i=0;i<vKeys.size();i++)
    {
        const cv::KeyPoint &kp = vKeys[i];
        if(kp.pt.x<n1.UR.x)
        {
            if(kp.pt.y<n1.BR.y)
                n1.vKeys.push_back(kp);
            else
                n3.vKeys.push_back(kp);
        }
        else if(kp.pt.y<n1.BR.y)
            n2.vKeys.push_back(kp);
        else
            n4.vKeys.push_back(kp);
    }

    if(n1.vKeys.size()==1)
        n1.bNoMore = true;
    if(n2.vKeys.size()==1)
        n2.bNoMore = true;
    if(n3.vKeys.size()==1)
        n3.bNoMore = true;
    if(n4.vKeys.size()==1)
        n4.bNoMore = true;

}


Image_feature_points_extraction::Image_feature_points_extraction()
{
	nfeatures = 20000;
	scaleFactor = 2.0f;
	nlevels = 8;
	iniThFAST = 20;
    minThFAST = 7;

	matchWindowsSize = 100;


	CheckOrientation = true;
	CheckVariance = true;
	
	mvScaleFactor.resize(nlevels);
	mvScaleFactor[0]=1.0f;
    for(int i=1; i<nlevels; i++)
    {
        mvScaleFactor[i]=mvScaleFactor[i-1]*scaleFactor;
    }


	 mvInvScaleFactor.resize(nlevels);
    for(int i=0; i<nlevels; i++)
    {
        mvInvScaleFactor[i]=1.0f/mvScaleFactor[i];
    }

	mvImagePyramid.resize(nlevels);

	mnFeaturesPerLevel.resize(nlevels);

	float factor = 1.0f / scaleFactor;
    float nDesiredFeaturesPerScale = nfeatures*(1 - factor)/(1 - (float)std::pow((double)factor, (double)nlevels));

	int sumFeatures = 0;
    for( int level = 0; level < nlevels-1; level++ )
    {
        mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
        sumFeatures += mnFeaturesPerLevel[level];
        nDesiredFeaturesPerScale *= factor;
    }
    mnFeaturesPerLevel[nlevels-1] = std::max(nfeatures - sumFeatures, 0);


	const int npoints = 512;
    const cv::Point* pattern0 = (const cv::Point*)bit_pattern_31_;
    std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));


	//This is for orientation
    // pre-compute the end of a row in a circular patch
    umax.resize(HALF_PATCH_SIZE + 1);

    int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * std::sqrt(2.f) / 2 + 1);
    int vmin = cvCeil(HALF_PATCH_SIZE * std::sqrt(2.f) / 2);
    const double hp2 = HALF_PATCH_SIZE*HALF_PATCH_SIZE;
    for (v = 0; v <= vmax; ++v)
        umax[v] = cvRound(std::sqrt(hp2 - v * v));

    // Make sure we are symmetric
    for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
    {
        while (umax[v0] == umax[v0 + 1])
            ++v0;
        umax[v] = v0;
        ++v0;
    }
}


void Image_feature_points_extraction::Image_extract_feature_point(cv::InputArray _image, std::vector<cv::KeyPoint>& _keypoints, cv::OutputArray _descriptors)
{
	if(_image.empty())
    	return;
	cv::Mat image = _image.getMat();
	assert(image.type() == CV_8UC1 );

	//计算图像金字塔
	ComputePyramid(image);

	std::vector < std::vector<cv::KeyPoint> > allKeypoints;

	ComputeKeyPointsOctTree(allKeypoints);

	cv::Mat descriptors;

	int nkeypoints = 0;
    for (int level = 0; level < nlevels; ++level)
        nkeypoints += (int)allKeypoints[level].size();
    if( nkeypoints == 0 )
        _descriptors.release();
    else
    {
        _descriptors.create(nkeypoints, 32, CV_8U);
        descriptors = _descriptors.getMat();
    }

	_keypoints.clear();
    _keypoints.reserve(nkeypoints);

	int offset = 0;
    for (int level = 0; level < nlevels; ++level)
    {
        std::vector<cv::KeyPoint>& keypoints = allKeypoints[level];
        int nkeypointsLevel = (int)keypoints.size();

        if(nkeypointsLevel==0)
            continue;

        // preprocess the resized image
        cv::Mat workingMat = mvImagePyramid[level].clone();
        cv::GaussianBlur(workingMat, workingMat, cv::Size(7, 7), 2, 2, cv::BORDER_REFLECT_101);

        // Compute the descriptors
        cv::Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
        computeDescriptors(workingMat, keypoints, desc, pattern);

        offset += nkeypointsLevel;

        // Scale keypoint coordinates
        if (level != 0)
        {
            float scale = mvScaleFactor[level]; //getScale(level, firstLevel, scaleFactor);
            for (std::vector<cv::KeyPoint>::iterator keypoint = keypoints.begin(),
                 keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
                keypoint->pt *= scale;
        }
        // And add the keypoints to the output
        _keypoints.insert(_keypoints.end(), keypoints.begin(), keypoints.end());
    }
}


void Image_feature_points_extraction::ComputePyramid(cv::Mat image)
{
    for (int level = 0; level < nlevels; ++level)
    {
        float scale = mvInvScaleFactor[level];
        cv::Size sz(cvRound((float)image.cols*scale), cvRound((float)image.rows*scale));
        cv::Size wholeSize(sz.width + EDGE_THRESHOLD*2, sz.height + EDGE_THRESHOLD*2);
        cv::Mat temp(wholeSize, image.type()), masktemp;
        mvImagePyramid[level] = temp(cv::Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

        // Compute the resized image
        if( level != 0 )
        {
            cv::resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, cv::INTER_LINEAR);

            cv::copyMakeBorder(mvImagePyramid[level], temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                           cv::BORDER_REFLECT_101 + cv::BORDER_ISOLATED);            
        }
        else
        {
            cv::copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                           cv::BORDER_REFLECT_101);            
        }
    }

}


void Image_feature_points_extraction::ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints)
{
	allKeypoints.resize(nlevels);

	const float W = 30;

	for(int level = 0; level < nlevels; ++level)
	{
		const int minBorderX = EDGE_THRESHOLD-3;
		const int minBorderY = minBorderX;
		const int maxBorderX = mvImagePyramid[level].cols-EDGE_THRESHOLD+3;
		const int maxBorderY = mvImagePyramid[level].rows-EDGE_THRESHOLD+3;

		std::vector<cv::KeyPoint> vToDistributeKeys;
        vToDistributeKeys.reserve(nfeatures*10);

		const float width = (maxBorderX-minBorderX);
        const float height = (maxBorderY-minBorderY);

		const int nCols = width/W;
        const int nRows = height/W;
        const int wCell = std::ceil(width/nCols);
        const int hCell = std::ceil(height/nRows);


		for(int i=0; i<nRows; i++)
        {
			const float iniY =minBorderY+i*hCell;
            float maxY = iniY+hCell+6;

            if(iniY>=maxBorderY-3)
                continue;
            if(maxY>maxBorderY)
                maxY = maxBorderY;

			for(int j=0; j<nCols; j++)
            {
				const float iniX =minBorderX+j*wCell;
                float maxX = iniX+wCell+6;
                if(iniX>=maxBorderX-6)
                    continue;
                if(maxX>maxBorderX)
                    maxX = maxBorderX;

				std::vector<cv::KeyPoint> vKeysCell;

				cv::FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                     vKeysCell,iniThFAST,true);

				if(vKeysCell.empty())
                {
                    cv::FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                         vKeysCell,minThFAST,true);
                }

				if(!vKeysCell.empty())
                {
                    for(std::vector<cv::KeyPoint>::iterator vit=vKeysCell.begin(); vit!=vKeysCell.end();vit++)
                    {
                        (*vit).pt.x+=j*wCell;
                        (*vit).pt.y+=i*hCell;
                        vToDistributeKeys.push_back(*vit);
                    }
                }
			}
		}

		std::vector<cv::KeyPoint> & keypoints = allKeypoints[level];
        keypoints.reserve(nfeatures);

		keypoints = DistributeOctTree(vToDistributeKeys, minBorderX, maxBorderX,
                                      minBorderY, maxBorderY,mnFeaturesPerLevel[level], level);

		const int scaledPatchSize = PATCH_SIZE*mvScaleFactor[level];

		// Add border to coordinates and scale information
        const int nkps = keypoints.size();
        for(int i=0; i<nkps ; i++)
        {
            keypoints[i].pt.x+=minBorderX;
            keypoints[i].pt.y+=minBorderY;
            keypoints[i].octave=level;
            keypoints[i].size = scaledPatchSize;
        }
	}

	// compute orientations
    for (int level = 0; level < nlevels; ++level)
        computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
}

std::vector<cv::KeyPoint> Image_feature_points_extraction::DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                       const int &maxX, const int &minY, const int &maxY, const int &N, const int &level)
{
	// Compute how many initial nodes   
    const int nIni = std::round(static_cast<float>(maxX-minX)/(maxY-minY));

    const float hX = static_cast<float>(maxX-minX)/nIni;

	std::list<ExtractorNode> lNodes;

	std::vector<ExtractorNode*> vpIniNodes;
    vpIniNodes.resize(nIni);

	
	for(int i=0; i<nIni; i++)
	{
		ExtractorNode ni;
		ni.UL = cv::Point2i(hX*static_cast<float>(i),0);
		ni.UR = cv::Point2i(hX*static_cast<float>(i+1),0);
		ni.BL = cv::Point2i(ni.UL.x,maxY-minY);
		ni.BR = cv::Point2i(ni.UR.x,maxY-minY);
		ni.vKeys.reserve(vToDistributeKeys.size());
	
		lNodes.push_back(ni);
		vpIniNodes[i] = &lNodes.back();
	}

	
	//Associate points to childs
	for(size_t i=0;i<vToDistributeKeys.size();i++)
	{
		const cv::KeyPoint &kp = vToDistributeKeys[i];
		vpIniNodes[kp.pt.x/hX]->vKeys.push_back(kp);
	}

	std::list<ExtractorNode>::iterator lit = lNodes.begin();

	while(lit!=lNodes.end())
    {
        if(lit->vKeys.size()==1)
        {
            lit->bNoMore=true;
            lit++;
        }
        else if(lit->vKeys.empty())
            lit = lNodes.erase(lit);
        else
            lit++;
    }


	bool bFinish = false;

    int iteration = 0;

    std::vector<std::pair<int,ExtractorNode*> > vSizeAndPointerToNode;
    vSizeAndPointerToNode.reserve(lNodes.size()*4);

	while(!bFinish)
    {
		iteration++;

        int prevSize = lNodes.size();

        lit = lNodes.begin();

        int nToExpand = 0;

        vSizeAndPointerToNode.clear();

		while(lit!=lNodes.end())
		{
			if(lit->bNoMore)
            {
                // If node only contains one point do not subdivide and continue
                lit++;
                continue;
            }
			else
            {
                // If more than one point, subdivide
                ExtractorNode n1,n2,n3,n4;
                lit->DivideNode(n1,n2,n3,n4);

                // Add childs if they contain points
                if(n1.vKeys.size()>0)
                {
                    lNodes.push_front(n1);                    
                    if(n1.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(std::make_pair(n1.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n2.vKeys.size()>0)
                {
                    lNodes.push_front(n2);
                    if(n2.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(std::make_pair(n2.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n3.vKeys.size()>0)
                {
                    lNodes.push_front(n3);
                    if(n3.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(std::make_pair(n3.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n4.vKeys.size()>0)
                {
                    lNodes.push_front(n4);
                    if(n4.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(std::make_pair(n4.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }

                lit=lNodes.erase(lit);
                continue;
            }
		}

		// Finish if there are more nodes than required features
        // or all nodes contain just one point
        if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
        {
            bFinish = true;
        }
		else if(((int)lNodes.size()+nToExpand*3)>N)
        {

            while(!bFinish)
            {

                prevSize = lNodes.size();

                std::vector<std::pair<int,ExtractorNode*> > vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
                vSizeAndPointerToNode.clear();

                std::sort(vPrevSizeAndPointerToNode.begin(),vPrevSizeAndPointerToNode.end());
                for(int j=vPrevSizeAndPointerToNode.size()-1;j>=0;j--)
                {
                    ExtractorNode n1,n2,n3,n4;
                    vPrevSizeAndPointerToNode[j].second->DivideNode(n1,n2,n3,n4);

                    // Add childs if they contain points
                    if(n1.vKeys.size()>0)
                    {
                        lNodes.push_front(n1);
                        if(n1.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(std::make_pair(n1.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n2.vKeys.size()>0)
                    {
                        lNodes.push_front(n2);
                        if(n2.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(std::make_pair(n2.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n3.vKeys.size()>0)
                    {
                        lNodes.push_front(n3);
                        if(n3.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(std::make_pair(n3.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n4.vKeys.size()>0)
                    {
                        lNodes.push_front(n4);
                        if(n4.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(std::make_pair(n4.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

                    if((int)lNodes.size()>=N)
                        break;
                }

                if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
                    bFinish = true;

            }
        }
	}

	// Retain the best point in each node
    std::vector<cv::KeyPoint> vResultKeys;
    vResultKeys.reserve(nfeatures);
    for(std::list<ExtractorNode>::iterator lit=lNodes.begin(); lit!=lNodes.end(); lit++)
    {
        std::vector<cv::KeyPoint> &vNodeKeys = lit->vKeys;
        cv::KeyPoint* pKP = &vNodeKeys[0];
        float maxResponse = pKP->response;

        for(size_t k=1;k<vNodeKeys.size();k++)
        {
            if(vNodeKeys[k].response>maxResponse)
            {
                pKP = &vNodeKeys[k];
                maxResponse = vNodeKeys[k].response;
            }
        }

        vResultKeys.push_back(*pKP);
    }

    return vResultKeys;
}


#define TH_HIGH  		150
#define TH_LOW   		50
#define HISTO_LENGTH  	30


// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int Image_feature_points_extraction::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}


int Image_feature_points_extraction::Feature_points_match(std::vector<cv::KeyPoint>& image1_keypoints, cv::Mat& image1_descriptors,
								std::vector<cv::KeyPoint>& image2_keypoints, cv::Mat& image2_descriptors,std::vector<int> &vnMatches12)
{

	int nmatches=0;
    vnMatches12 = std::vector<int>(image1_keypoints.size(),-1);
	std::vector<int> vnMatches21(image2_keypoints.size(),-1);

	std::vector<int> vMatchedDistance(image2_keypoints.size(),INT_MAX);

	std::vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;
    
	for(size_t i1=0, iend1 = image1_keypoints.size(); i1<iend1; i1++)
	{
		//获取该关键点获取的金字塔层
		int level1 = image1_keypoints[i1].octave;
		
		//获取k1 的描述子
		cv::Mat d1 = image1_descriptors.row(i1);

		int bestDist = INT_MAX;
		int bestDist2 = INT_MAX;
		int bestIdx2 = -1;

		for(size_t i2=0, iend2 = image2_keypoints.size(); i2<iend2; i2++)
		{
			//获取该关键点获取的金字塔层
			int level2 = image2_keypoints[i2].octave;

			if(level2 != level1)
				continue;
			
			//获取k2 的描述子
			cv::Mat d2 = image2_descriptors.row(i2);
			int dist = DescriptorDistance(d1, d2);

			if(vMatchedDistance[i2] <= dist)
				continue;

			if(dist<bestDist)
			{
				bestDist2 = bestDist;
				bestDist = dist;
				bestIdx2 = i2;
			}
			else if(dist<bestDist2)
			{
				bestDist2 = dist;
			}
		}


		//std::cout<< bestDist << std::endl;

		if(bestDist<=TH_LOW)
		{
			//最小距离要小于次小距离 0.9 倍
			if(bestDist < (float)bestDist2 * 0.9)
			{
				//如果已经匹配
				if(vnMatches21[bestIdx2] >= 0)
				{
					//移除匹配
					vnMatches12[vnMatches21[bestIdx2]] = -1;
					nmatches--;
				}

				vnMatches12[i1] = bestIdx2;
				vnMatches21[bestIdx2] = i1;
				vMatchedDistance[bestIdx2] = bestDist;
				nmatches++;

				if(CheckOrientation)
                {
                    float rot =  image1_keypoints[i1].angle - image2_keypoints[bestIdx2].angle;
                    if(rot<0.0)
                        rot+=360.0f;
                    int bin = round(rot*factor);
                    if(bin==HISTO_LENGTH)
                        bin=0;
                    assert(bin>=0 && bin<HISTO_LENGTH);
                    rotHist[bin].push_back(i1);
                }
			}
		}
	}
	

	
	if(CheckOrientation)
	{
		int ind1=-1;
		int ind2=-1;
		int ind3=-1;
	
		ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);
	
		for(int i=0; i<HISTO_LENGTH; i++)
		{
			if(i==ind1 || i==ind2 || i==ind3)
				continue;
			for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
			{
				int idx1 = rotHist[i][j];
				if(vnMatches12[idx1]>=0)
				{
					vnMatches12[idx1]=-1;
					nmatches--;
				}
			}
		}
	}

	//遍历所有匹配成功的特征点，使用最小二乘法
	if(CheckVariance){
		int x_average = 0;
		int y_average = 0;
	
		for(int i=0; i<vnMatches12.size(); i++)
		{
			if(vnMatches12[i] != -1)
			{
				x_average += image1_keypoints[i].pt.x - image2_keypoints[vnMatches12[i]].pt.x;
				y_average += image1_keypoints[i].pt.y - image2_keypoints[vnMatches12[i]].pt.y;
			}
		}

		x_average /= nmatches;
		y_average /= nmatches;

		int variance_min_num = nmatches / 2;
		int variance_min_dis[variance_min_num];
		int variance_min_err[variance_min_num];

		for(int i=0; i<variance_min_num; i++)
		{
			variance_min_dis[i] = -1;
			variance_min_err[i] = INT_MAX;
		}

		for(int i=0; i<vnMatches12.size(); i++)
		{
			if(vnMatches12[i] != -1)
			{
				int x_d = image1_keypoints[i].pt.x - image2_keypoints[vnMatches12[i]].pt.x;
				int y_d = image1_keypoints[i].pt.y - image2_keypoints[vnMatches12[i]].pt.y;
				int v_x = pow(x_d - x_average, 2);
				int v_y = pow(y_d - y_average, 2);

				int v_x_y = v_x + v_y;
				
				for(int j=0; j<variance_min_num; j++)
				{
					if(v_x_y < variance_min_err[j])
					{
						for(int k=variance_min_num-1; k>j; k--)
						{
							variance_min_err[k] = variance_min_err[k-1];
						}
						variance_min_err[j] = v_x_y;
						variance_min_dis[j] = i;
						break;
					}
				}
			}
		}

		for(int i=0; i<vnMatches12.size(); i++)
		{
			if(vnMatches12[i] != -1)
			{
				bool have = false;
				for(int j=0; j<variance_min_num; j++)
				{
					if(variance_min_dis[j] == i)
					{
						have = true;
					}
				}

				if(!have)
				{
					vnMatches12[i] = -1;
					nmatches--;
				}
			}
		}
	}

	return nmatches;
}


int Image_feature_points_extraction::Feature_points_match_windows(cv::Mat& image1, std::vector<cv::KeyPoint>& image1_keypoints, cv::Mat& image1_descriptors,
								cv::Mat& image2, std::vector<cv::KeyPoint>& image2_keypoints, cv::Mat& image2_descriptors,std::vector<int> &vnMatches12)
{
	//第一步:  把image1 分成matchWindowsSize  个窗口
	int windows_size_cols, windows_size_rows;
	int windows_cols_num, windows_rows_num;

	float y_div_x = (float)image1.rows / (float)image1.cols;
	float y = std::sqrt(matchWindowsSize *  y_div_x);
	windows_rows_num = (int)y;
	windows_cols_num  = (int)(y / y_div_x);

	std::cout << "windows num rows:" << windows_rows_num << ", cols:" << windows_cols_num << std::endl;

	windows_size_cols = image1.cols / windows_cols_num;
	windows_size_rows = image1.rows / windows_rows_num;

	std::cout << "windows size rows:" << windows_size_rows << ", cols:" << windows_size_cols << std::endl;

	

	std::vector<cv::KeyPoint> windows_feature_points[windows_cols_num][windows_rows_num];
	std::vector<cv::Mat> windows_feature_descriptors[windows_cols_num][windows_rows_num];


	for(int i=0; i<image1_keypoints.size(); i++)
	{
		int cols = image1_keypoints[i].pt.x / windows_size_cols;
		int rows = image1_keypoints[i].pt.y / windows_size_rows;

		windows_feature_points[cols][rows].push_back(image1_keypoints[i]);
		windows_feature_descriptors[cols][rows].push_back(image1_descriptors.row(i));
	}

#if 0
	for(int i=0; i<windows_cols_num; i++)
	{
		for(int j=0; j<windows_rows_num; j++)
		{
			std::cout << "i:" << i << ", j:" << j <<", num:" << windows_feature_points[i][j].size() << std::endl;
		}
	}
#endif


	//第二步:  从每个窗口中找出最佳匹配特征点，跟其他特征点的汉明距离最远

	//第三步:  从最佳特征点去与image2 中的特征点进行匹配，取出3 个最优匹配位置

	//第四步: 在3 个最优匹配位置获取matchWindowsSize 大小的所有特征点，进行匹配，寻找最佳匹配位置
	//通过最佳匹配位置，和周围特征点的匹配，可以求出图片的旋转和位置偏移
	//第五步: 把image1 所有的窗口执行二、三、四步，通过最小二乘法求出最优解

	std::vector<cv::KeyPoint> all_best_keypoint;
	for(int i=0; i<windows_cols_num; i++)
	{
		for(int j=0; j<windows_rows_num; j++)
		{
			//求出每个窗口中最佳的特征点
			int descriptors_err_max = 0;
			int max_point;
			int windows_size = windows_feature_descriptors[i][j].size();
			for(int k=0; k<windows_size; k++)
			{
				int dist = 0;
				cv::Mat k_d = windows_feature_descriptors[i][j][k];
				for(int p=0; p < windows_size; p++)
				{
					cv::Mat p_d = windows_feature_descriptors[i][j][p];
					dist += DescriptorDistance(k_d, p_d);
				}

				if(dist > descriptors_err_max)
				{
					descriptors_err_max = dist;
					max_point = k;
				}
			}

			cv::KeyPoint &best_keypoint = windows_feature_points[i][j][max_point];

			all_best_keypoint.push_back(best_keypoint);

			std::cout << "max_point:" << max_point << "descriptors err max:" << descriptors_err_max <<std::endl;
		}
	}

	cv::drawKeypoints(image1, all_best_keypoint, image1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);
	cv::imwrite("all_best_keypoint.jpg",image1);
	
}



void Image_feature_points_extraction::drawKeyPointsMatch(cv::Mat& image1, std::vector<cv::KeyPoint>& image1_keypoints,
					cv::Mat& image2, std::vector<cv::KeyPoint>& image2_keypoints, std::vector<int> &vnMatches12, cv::Mat &image_match)
{
	//为两个图片申请空间
	int rows = image1.rows > image2.rows ? image1.rows : image2.rows;
	image_match.create(rows, image1.cols + image2.cols, CV_8UC1);
	image_match.setTo(0);

	if(image1_keypoints.size() > 0)
		cv::drawKeypoints(image1, image1_keypoints, image1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);

	if(image2_keypoints.size() > 0)
		cv::drawKeypoints(image2, image2_keypoints, image2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);

	//拷贝图像
	image1.copyTo(image_match(cv::Rect(0, 0, image1.cols, image1.rows)));
	image2.copyTo(image_match(cv::Rect(image1.cols, 0, image2.cols, image2.rows)));

	//匹配ok  的点进行画线
	for(int i=0; i<vnMatches12.size(); i++)
	{
		if(vnMatches12[i] != -1)
		{
			//获取image 关键点
			cv::Point2i image1_k, image2_k;

			image1_k.x = image1_keypoints[i].pt.x;
			image1_k.y = image1_keypoints[i].pt.y;
			
			image2_k.x = image2_keypoints[vnMatches12[i]].pt.x + image1.cols;
			image2_k.y = image2_keypoints[vnMatches12[i]].pt.y;

			cv::line(image_match, image1_k, image2_k, cv::Scalar(255, 255, 255), 2);
		}
	}
	
}


void Image_feature_points_extraction::ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}


} //namespace IMAGE_MOSAIC

