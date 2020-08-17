#include "image_algorithm.h"

namespace IMAGE_MOSAIC
{

#define  DUBUG

#ifdef DUBUG
static int num_image = 0;
#endif

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

void Image_algorithm::Image_cut(cv::Mat& src_image, cv::Mat& dest_image, enum Image_mosaic_head head, int cut_size)
{
	if(head == UP)
	{
		dest_image = src_image(cv::Range(cut_size, src_image.rows), cv::Range(0, src_image.cols));
	}
	else if(head == DOWN)
	{
		dest_image = src_image(cv::Range(0, src_image.rows - cut_size), cv::Range(0, src_image.cols));
	}
	else if(head == LEFT)
	{
		dest_image = src_image(cv::Range(0, src_image.rows), cv::Range(cut_size, src_image.cols));
	}
	else if(head == RIGHT)
	{
		dest_image = src_image(cv::Range(0, src_image.rows), cv::Range(0, src_image.cols - cut_size));
	}
}


void Image_algorithm::Get_sample_size_up_down(cv::Point2i image_size, cv::Point2i &sample_size, int &dis)
{
	dis = image_size.x / 2;
	if(image_size.x > 4000)
	{
		sample_size.x = 1000;
	}
	else
	{
		sample_size.x = image_size.x / 4;
	}

	if(image_size.y > 4000)
	{
		sample_size.y = 500;
	}
	else
	{
		sample_size.y = image_size.y / 10;
	}

	if(sample_size.y < 20)
	{
		sample_size.y = 20;
	}

}

void Image_algorithm::Get_sample_size_left_right(cv::Point2i image_size, cv::Point2i &sample_size, int &dis)
{
	if(image_size.x > 4000)
	{
		sample_size.x = 500;
	}
	else
	{
		sample_size.x = image_size.x / 10;
	}

	if(sample_size.x < 20)
	{
		sample_size.x = 20;
	}

	dis = image_size.y / 3;
	if(image_size.y > 4000)
	{
		sample_size.y = 1000;
	}
	else
	{
		sample_size.y = image_size.y / 4;
	}
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
	cv::Point2i image_size(src_image1.cols, src_image1.rows);
	cv::Point2i image1_sample_size;

	//图像1 和图像2  在x 方向上可能移动的最大距离 diff_x
	int diff_x;
	Get_sample_size_up_down(image_size, image1_sample_size, diff_x);
	
	
	cv::Point2i image2_sample_size(image1_sample_size.x + diff_x, image1_sample_size.y);

	int start_row[3] = {src_image1.rows / 4,
						src_image1.rows / 4 +  image1_sample_size.y + 10, 
						src_image1.rows / 4 + 2 * (image1_sample_size.y + 10)};

	int start_col[3] = {	src_image1.cols / 2 - image1_sample_size.x / 2,
							diff_x / 2,
							src_image1.cols - diff_x / 2 - image1_sample_size.x};

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

		for(int n = start_row[i]; n< num + start_row[i]; n++)
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
	}

	//块匹配连续性检查
	int err[3];
	int err_min = INT_MAX;
	int err_min_num;
	for(int i=0; i<3; i++)
	{
		err[i] = 0;

		for(int j=0; j<image1_sample_size.y; j++)
		{
			for(int k=0; k<image1_sample_size.x; k++)
			{
				err[i] += pow(	src_image2.at<uchar>(min_err_idex[i] + j, start_col[i] - diff_x / 2 + min_err_dis[i] + k) - 
							 	src_image1.at<uchar>(start_row[i] + j, start_col[i] + k), 2);
			}
		}

		if(err[i] < err_min)
		{
			err_min = err[i];
			err_min_num = i;
		}
	}

	//计算图像之间的拼接位置

	//y 始终大于0
	distance.y = min_err_idex[err_min_num] - start_row[err_min_num];
	
	//x < 0, 表示向左 移动的像素，x > 0 表示向 右移动的像素
	distance.x = diff_x / 2 - min_err_dis[err_min_num];

#ifdef DUBUG
	std::cout <<"err min num:" << err_min_num << ",err min:" << err_min << std::endl;

	for(int i=0; i<3; i++)
	{
		std::cout << i <<",min err:" << min_err[i] << ",min err dis:" << min_err_dis[i] << ",min err idex:" << min_err_idex[i] << std::endl;

		for(int j=0; j<image1_sample_size.x; j++)
		{
			src_image1.at<uchar>(start_row[i], start_col[i] + j) = 255;
			src_image1.at<uchar>(start_row[i] + image1_sample_size.y, start_col[i] + j) = 255;
		
			src_image2.at<uchar>(min_err_idex[i], start_col[i] - diff_x / 2 + min_err_dis[i] + j) = 255;
			src_image2.at<uchar>(min_err_idex[i] + image2_sample_size.y, start_col[i] - diff_x / 2 + min_err_dis[i] + j) = 255;
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

	cv::imwrite(strName1.c_str(), src_image1);
	cv::imwrite(strName2.c_str(), src_image2);
#endif

	return OK;
}

int Image_algorithm::Image_mosaic_down_algorithm(cv::Mat &src_image1, cv::Mat &src_image2, cv::Point2i &distance)
{
#ifdef DUBUG
	std::cout << "please use Image_mosaic_up_algorithm!!!" << std::endl;
#endif
	return ERR;
}

int Image_algorithm::Image_mosaic_left_algorithm(cv::Mat &src_image1, cv::Mat &src_image2, cv::Point2i &distance)
{
#ifdef DUBUG
		std::cout << "please use Image_mosaic_right_algorithm!!!" << std::endl;
#endif
	return ERR;
}

int Image_algorithm::Image_mosaic_right_algorithm(cv::Mat &src_image1, cv::Mat &src_image2, cv::Point2i &distance)
{
	cv::Point2i image_size(src_image1.cols, src_image1.rows);
	cv::Point2i image1_sample_size;

	//图像1 和图像2  在y  方向上最大的距离可能是diff_y
	int diff_y;
	Get_sample_size_left_right(image_size, image1_sample_size, diff_y);

	cv::Point2i image2_sample_size(image1_sample_size.x, image1_sample_size.y + diff_y);

	int start_row[3] = {src_image1.rows / 2 - image1_sample_size.y / 2,
						diff_y / 2,
						src_image1.rows - diff_y / 2 - image1_sample_size.y};


	int start_col[3]= {	src_image1.cols * 3 / 4, 
						src_image1.cols * 3 / 4 - image1_sample_size.x - 10,
						src_image1.cols * 3 / 4 - 2 * (image1_sample_size.x + 10)};

#ifdef DUBUG
	std::cout << "image_mosaic_algorithm image1 cols:" << src_image1.cols << ", rows:" << src_image1.rows << std::endl;
	std::cout << "image1_sample_size x:" << image1_sample_size.x << ", y:" << image1_sample_size.y << std::endl;
	std::cout << "diff_y:" << diff_y << std::endl;
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

	//分别查找3  组中最小二乘法位置
	for(int i=0; i<3; i++)
	{
		//计算图像 1  的匹配模板
		int base[image1_sample_size.y];

		for(int k=0; k<image1_sample_size.y; k++)
		{
			base[k] = src_image1.at<uchar>(start_row[i] + k, start_col[i]) - src_image1.at<uchar>(start_row[i] + k, start_col[i] - image1_sample_size.x);
		}
		
		//找出图像2  的最佳匹配
		int num = src_image2.cols  - start_col[i] - image1_sample_size.x;
		int rows_min_err[num];
		int rows_min_err_dis[num];

		for(int n=0; n<num; n++)
		{
			rows_min_err[n] = INT_MAX;
			rows_min_err_dis[n] = 0;
		}

		int match_image[image2_sample_size.y];

		for(int n=start_col[i]; n>start_col[i]-num; n--)
		{
			for(int j=0; j<image2_sample_size.y; j++)
			{
				match_image[j] = src_image2.at<uchar>(start_row[i] - diff_y / 2 + j, n) -
								 src_image2.at<uchar>(start_row[i] - diff_y / 2 + j, n - image2_sample_size.x);
			}

			//求每一行和第一张图像的最小二乘的最佳位置和值
			for(int d=0; d<diff_y; d++)
			{
				int err = 0;
				for(int p=0; p<image1_sample_size.y; p++)
				{
					err += std::pow(match_image[p + d] - base[p], 2);
				}

				
				if(err < rows_min_err[start_col[i] - n])
				{
					rows_min_err[start_col[i] - n] = err;
					rows_min_err_dis[start_col[i] - n] = d;

					if(rows_min_err[start_col[i] - n] < min_err[i])
					{
						min_err[i] = rows_min_err[start_col[i] - n];
						min_err_dis[i] = rows_min_err_dis[start_col[i] - n];
						min_err_idex[i] = n;
					}
				}
			}
		}
	}

	//块匹配连续性检查
	int err[3];
	int err_min = INT_MAX;
	int err_min_num;


	for(int i=0; i<3; i++)
	{
		err[i] = 0;

		for(int j=0; j<image1_sample_size.x; j++)
		{
			for(int k=0; k<image1_sample_size.y; k++)
			{
				err[i] += pow(	src_image2.at<uchar>(start_row[i]- diff_y / 2 + min_err_dis[i] + k, min_err_idex[i] + j) - 
							 	src_image1.at<uchar>(start_row[i] + k, start_col[i] + j), 2);
			}
		}

		if(err[i] < err_min)
		{
			err_min = err[i];
			err_min_num = i;
		}
	}

	//计算图像之间的拼接位置

	// x 始终大于0， 第二张图像右移 
	distance.x = start_col[err_min_num] - min_err_idex[err_min_num];

	// y < 0  表示相对左 图像，右边图像上移
	// y > 0  表示相对左图像， 右边图像下移
	distance.y = diff_y /2 - min_err_dis[err_min_num];

#ifdef DUBUG
	std::cout <<"err min num:" << err_min_num << ",err min:" << err_min << std::endl;
	
	for(int i=0; i<3; i++)
	{
		std::cout << i <<",min err:" << min_err[i] << ",min err dis:" << min_err_dis[i] << ",min err idex:" << min_err_idex[i] << std::endl;
	
		for(int j=0; j<image1_sample_size.y; j++)
		{
			src_image1.at<uchar>(start_row[i] + j, start_col[i]) = 255;
			src_image1.at<uchar>(start_row[i] + j, start_col[i] - image1_sample_size.x) = 255;

			
			src_image2.at<uchar>(start_row[i]- diff_y / 2 + min_err_dis[i] + j, min_err_idex[i]) = 255;
			src_image2.at<uchar>(start_row[i]- diff_y / 2 + min_err_dis[i] + j, min_err_idex[i] - image2_sample_size.x) = 255;
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
	
	cv::imwrite(strName1.c_str(), src_image1);
	cv::imwrite(strName2.c_str(), src_image2);
#endif

	

	return OK;
}


int Image_algorithm::Image_optimize_seam(cv::Mat& src_image1, cv::Mat& src_image2, cv::Mat& dest_image, cv::Point2i distance,
															enum Image_mosaic_head head, cv::Point2i &image1_vertex, cv::Point2i &image2_vertex)
{
	

	if(head == UP)
	{
		//为目标图片申请空间	
		dest_image.create(src_image1.rows + std::abs(distance.y), src_image1.cols + std::abs(distance.x), CV_8UC3);
		dest_image.setTo(0);

		//裁剪掉 src_image2  的下方的 1/6  ，目的是去掉旋转带来的黑边
		cv::Mat image1,image2;

		int cut_size = src_image2.rows/6;
		int image1_cut_size = src_image1.rows - (dest_image.rows - (src_image2.rows - cut_size));
		Image_cut(src_image1, image1, UP, image1_cut_size);
		Image_cut(src_image2, image2, DOWN, cut_size);

		// y  > 0 , 恒成立
		// x > 0, 表示image2  相对于image1 右移
		// x < 0, 表示image2  相对于image1 左移

		if(distance.x < 0)
		{
			image1.copyTo(dest_image(cv::Rect(std::abs(distance.x), image2.rows, image1.cols, image1.rows)));
			image2.copyTo(dest_image(cv::Rect(0, 0, image2.cols, image2.rows)));

			image1_vertex.x = abs(distance.x);
			image1_vertex.y = distance.y;

			image2_vertex.x = 0;
			image2_vertex.y = 0;
		}
		else
		{
			image1.copyTo(dest_image(cv::Rect(0, image2.rows, image1.cols, image1.rows)));
			image2.copyTo(dest_image(cv::Rect(distance.x, 0, image2.cols, image2.rows)));

			image1_vertex.x = 0;
			image1_vertex.y = distance.y;

			image2_vertex.x = distance.x;
			image2_vertex.y = 0;
		}

		//接口优化
		int w = (src_image1.rows + src_image2.rows - dest_image.rows) / 2;
		int image1_start_row = image1_cut_size - w;
		int image2_start_row = image2.rows - w;
		int dest_start_row = image2_start_row;
		int dest_start_col = std::abs(distance.x);
		int optimize_size = dest_image.cols - 2 * std::abs(distance.x);
		int image1_start_col;
		int image2_start_col;

		float alpha = 1.0f;//src_image2  中像素的权重

		if(distance.x < 0)
		{
			image1_start_col = 0;
			image2_start_col = std::abs(distance.x);
		}
		else
		{
			image1_start_col = std::abs(distance.x);
			image2_start_col = 0;
		}

		for(int i=0; i<w; i++)
		{
			alpha = (float)(w - i) / (float)w;
			for(int j=0; j<optimize_size; j++)
			{
				cv::Scalar color1 = src_image1.at<cv::Vec3b>(image1_start_row + i, image1_start_col + j);
				cv::Scalar color2 = src_image2.at<cv::Vec3b>(image2_start_row + i, image2_start_col + j);
				cv::Scalar color3;
				color3(0) = color1(0) * (1 - alpha) + color2(0) * alpha;
				color3(1) = color1(1) * (1 - alpha) + color2(1) * alpha;
				color3(2) = color1(2) * (1 - alpha) + color2(2) * alpha;
				dest_image.at<cv::Vec3b>(dest_start_row + i, dest_start_col + j) = cv::Vec3b(color3(0), color3(1), color3(2));
			}
		}
	}
	else if(head == DOWN)
	{
#ifdef DUBUG
		std::cout << "please use UP!!!" << std::endl;
#endif
		return ERR;
	}
	else if(head == LEFT)
	{
#ifdef DUBUG
		std::cout << "please use RIGHT!!!" << std::endl;
#endif
		return ERR;
	}
	else if(head == RIGHT)
	{
		//为目标图片申请空间	
		dest_image.create(src_image1.rows + std::abs(distance.y), src_image1.cols + std::abs(distance.x), CV_8UC3);
		dest_image.setTo(0);

		//裁剪掉 src_image2  的左边 的 1/6  ，目的是去掉旋转带来的黑边
		cv::Mat image1,image2;

		int cut_size = src_image2.cols/6;
		int image1_cut_size = src_image1.cols - (dest_image.cols - (src_image2.cols - cut_size));
		Image_cut(src_image1, image1, RIGHT, image1_cut_size);
		Image_cut(src_image2, image2, LEFT, cut_size);

		// x 始终大于0， 第二张图像右移 
		// y < 0  表示相对左 图像，右边图像上移
		// y > 0  表示相对左图像， 右边图像下移

		if(distance.y < 0)
		{
			
			image1.copyTo(dest_image(cv::Rect(0, std::abs(distance.y), image1.cols, image1.rows)));
			image2.copyTo(dest_image(cv::Rect(std::abs(distance.x) + cut_size, 0, image2.cols, image2.rows)));

			image1_vertex.x = 0;
			image1_vertex.y = std::abs(distance.y);

			image2_vertex.x = std::abs(distance.x);
			image2_vertex.y = 0;
		}
		else
		{
			image1.copyTo(dest_image(cv::Rect(0, 0, image1.cols, image1.rows)));
			image2.copyTo(dest_image(cv::Rect(std::abs(distance.x) + cut_size, distance.y, image2.cols, image2.rows)));

			image1_vertex.x = 0;
			image1_vertex.y = 0;

			image2_vertex.x = std::abs(distance.x);
			image2_vertex.y = distance.y;
		}

		//拼图接口优化
		int w = (src_image1.cols + src_image2.cols - dest_image.cols) / 2;
		int image1_start_row;
		int image2_start_row;
		int dest_start_row;
		int optimize_size = dest_image.rows - 2 * std::abs(distance.y);
		int image1_start_col = image1.cols;
		int image2_start_col = cut_size;
		int dest_start_col = image1_start_col;

		if(distance.y < 0)
		{
			image1_start_row = 0;
			
			image2_start_row = std::abs(distance.y);
		}
		else
		{
			image1_start_row = distance.y;
			image2_start_row = 0;
		}

		dest_start_row = image1_start_row;

		float alpha = 1.0f;//src_image1  中像素的权重

		for(int i=0; i<w; i++)
		{
			alpha = (float)(w - i) / (float)w;
			for(int j=0; j<optimize_size; j++)
			{
				cv::Scalar color1 = src_image1.at<cv::Vec3b>(image1_start_row + j, image1_start_col + i);
				cv::Scalar color2 = src_image2.at<cv::Vec3b>(image2_start_row + j, image2_start_col + i);
				cv::Scalar color3;
				color3(0) = color1(0) * alpha + color2(0) * (1 - alpha);
				color3(1) = color1(1) * alpha + color2(1) * (1 - alpha);
				color3(2) = color1(2) * alpha + color2(2) * (1 - alpha);
				dest_image.at<cv::Vec3b>(dest_start_row + j, dest_start_col + i) = cv::Vec3b(color3(0), color3(1), color3(2));
			}
		}
	}

	return OK;
}


} //namespace IMAGE_MOSAIC
