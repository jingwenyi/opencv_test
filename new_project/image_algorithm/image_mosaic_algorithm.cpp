#include "image_algorithm.h"

namespace IMAGE_MOSAIC
{

#define  DUBUG

#define  NUMBER_OF_INTERVAL_ROWS    40
#define  DISLOCATION_DISTANCE		100


void Image_algorithm::Image_rotate(cv::Mat& src_image,  cv::Mat& dest_image, double angle)
{
	cv::Point2f pt(src_image.cols/2, src_image.rows/2);
	cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);
	cv::warpAffine(src_image, dest_image, r, cv::Size(src_image.cols, src_image.rows));
}




//算法思路:   用3 组块匹配，然后在3  组最佳的里面进行块连续性匹配
int Image_algorithm::Image_mosaic_algorithm(cv::Mat &src_image1, cv::Mat &src_image2, int head, cv::Point2i &distance)
{
	cv::Mat image1_gray, image2_gray;
	cv::cvtColor(src_image1, image1_gray, CV_RGB2GRAY);
	cv::cvtColor(src_image2, image2_gray, CV_RGB2GRAY);

	if(head == UP)
	{
		return Image_mosaic_up_algorithm(image1_gray, image2_gray, distance);
	}
	else if(head == DOWN)
	{
		return Image_mosaic_down_algorithm(image1_gray, image2_gray, distance);
	}
	else if(head == LEFT)
	{
		return Image_mosaic_left_algorithm(image1_gray, image2_gray, distance);
	}
	else if(head == RIGHT)
	{
		return Image_mosaic_right_algorithm(image1_gray, image2_gray, distance);
	}

	return ERR;
}


int Image_algorithm::Image_mosaic_up_algorithm(cv::Mat &src_image1, cv::Mat &src_image2, cv::Point2i &distance)
{
	cv::Point2i image1_sample_size(src_image1.cols / 2, NUMBER_OF_INTERVAL_ROWS);
	
	//图像1 和图像2  在x 方向上可能移动的最大距离 diff_x
	int diff_x = DISLOCATION_DISTANCE;
	cv::Point2i image2_sample_size(image1_sample_size.x + diff_x, image1_sample_size.y);

	int start_row[3] = {src_image1.rows / 4,
						src_image1.rows / 4 + 2 * NUMBER_OF_INTERVAL_ROWS, 
						src_image1.rows / 4 + 4 * NUMBER_OF_INTERVAL_ROWS};

	//保证 图像2  的采样在1/8   到 7/8  之间
	int start_col[3] = {	src_image1.cols / 2 - image1_sample_size.x / 2,
							src_image1.cols / 8 + diff_x / 2,
							src_image1.cols *7 / 8 - diff_x / 2 - image1_sample_size.x};

#ifdef DUBUG
		std::cout << "image_mosaic_algorithm image1 cols:" << src_image1.cols << ", rows:" << src_image1.rows << std::endl;
		std::cout << "image1_sample_size x:" << image1_sample_size.x << ", y:" << image1_sample_size.y << std::endl;
		std::cout << "diff_x:" << diff_x << std::endl;
		std::cout << "image2_sample_size x:" << image2_sample_size.x << ", y:" << image2_sample_size.y << std::endl;
		std::cout << "start row, 1:" << start_row[0] << ", 2:" << start_row[1] << ", 3:" << start_row[2] << std::endl;
		std::cout << "start col, 1:" << start_col[0] << ", 2:" << start_col[1] << ", 3:" << start_col[2] << std::endl;
#endif

	int min_err[3];
	int min_err_idex[3];
	int min_err_dis[3];

	for(int i=0; i<3; i++)
	{
		min_err[i] = INT_MAX;
		min_err_idex[i] = 0;
		min_err_dis[i] = 0;
	}

	//分别查找3 组中最小二乘的位置
	for(int i=0; i<3; i++)
	{
		//计算图像 1  的匹配模板
		int base[image1_sample_size.x];

		for(int k=0; k<image1_sample_size.x; k++)
		{
			base[k] = src_image1.at<uchar>(start_row[i], start_col[i] + k) - src_image1.at<uchar>(start_row[i] + image1_sample_size.y, start_col[i] + k);
#ifdef DUBUG
			src_image1.at<uchar>(start_row[i], start_col[i] + k) = 0;
			src_image1.at<uchar>(start_row[i] + image1_sample_size.y, start_col[i] + k) = 0;
#endif
		}

		//找出图像2  的最佳匹配
		int num = src_image2.rows  - start_row[i] - image1_sample_size.y;
		int rows_min_err[num];
		int rows_min_err_dis[num];

		for(int n=0; n<num; n++)
		{
			rows_min_err[n] = INT_MAX;
			rows_min_err_dis[n] = 0;
		}

		int match_image[image2_sample_size.x];

		for(int n=start_row[i]; n<num; n++)
		{
			for(int j=0; j<image2_sample_size.x; j++)
			{
				match_image[j] = src_image2.at<uchar>(n, start_col[i] - diff_x / 2 + j) -
								 src_image2.at<uchar>(n + image2_sample_size.y, start_col[i] - diff_x / 2 + j);
			}

			//求每一行和第一张图像的最小二乘的最佳位置和值
			for(int d=0; d<diff_x; d++)
			{
				int err = 0;
				for(int p=0; p<image1_sample_size.x; p++)
				{
					err += std::pow(match_image[p + d] - base[p], 2);
				}

				
				if(err < rows_min_err[n - start_row[i]])
				{
					rows_min_err[n - start_row[i]] = err;
					rows_min_err_dis[n - start_row[i]] = d;

					if(rows_min_err[n - start_row[i]] < min_err[i])
					{
						min_err[i] = rows_min_err[n - start_row[i]];
						min_err_dis[i] = rows_min_err_dis[n - start_row[i]];
						min_err_idex[i] = n;
					}
				}
			}
		}


		std::cout << "=================" << i << "=============" << std::endl;
			
	}

#ifdef DUBUG
	for(int i=0; i<3; i++)
	{
		std::cout << i <<",min err:" << min_err[i] << ",min err dis:" << min_err_dis[i] << ",min err idex:" << min_err_idex[i] << std::endl;
	}
#endif




#ifdef DUBUG
	static int num_image = 0;
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

	cv::imwrite(strName1.c_str(), src_image1);
	cv::imwrite(strName2.c_str(), src_image2);
#endif

	return OK;
}

int Image_algorithm::Image_mosaic_down_algorithm(cv::Mat &src_image1, cv::Mat &src_image2, cv::Point2i &distance)
{
	return OK;
}

int Image_algorithm::Image_mosaic_left_algorithm(cv::Mat &src_image1, cv::Mat &src_image2, cv::Point2i &distance)
{
	return OK;
}

int Image_algorithm::Image_mosaic_right_algorithm(cv::Mat &src_image1, cv::Mat &src_image2, cv::Point2i &distance)
{
	return OK;
}


int Image_algorithm::Optimize_seam(cv::Mat& src_image1, cv::Mat& src_image2, cv::Mat& dest_image, cv::Point2i distace, cv::Point2i &left_top, cv::Point2i &right_bottom)
{

	return OK;
}


} //namespace IMAGE_MOSAIC
