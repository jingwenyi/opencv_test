#include "image_algorithm.h"

namespace IMAGE_MOSAIC
{

#define  DUBUG

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
		sample_size.y = 300;
	}
	else
	{
		sample_size.y = image_size.y / 15;
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
		sample_size.x = 300;
	}
	else
	{
		sample_size.x = image_size.x / 15;
	}

	if(sample_size.x < 20)
	{
		sample_size.x = 20;
	}

	dis = image_size.y / 2;
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
			src_image1.at<uchar>(start_row[i], start_col[i] + j) = 0;
			src_image1.at<uchar>(start_row[i] + image1_sample_size.y, start_col[i] + j) = 0;
		
			src_image2.at<uchar>(min_err_idex[i], start_col[i] - diff_x / 2 + min_err_dis[i] + j) = 0;
			src_image2.at<uchar>(min_err_idex[i] + image2_sample_size.y, start_col[i] - diff_x / 2 + min_err_dis[i] + j) = 0;
		}
	}

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


int Image_algorithm::Image_optimize_seam(cv::Mat& src_image1, cv::Mat& src_image2, cv::Mat& dest_image, cv::Point2i distance,
															enum Image_mosaic_head head, cv::Point2i &left_top, cv::Point2i &right_bottom)
{
	//为目标图片申请空间	
	dest_image.create(src_image1.rows + std::abs(distance.y), src_image1.cols + std::abs(distance.x), CV_8UC3);
	dest_image.setTo(0);

	//裁剪掉 src_image2  的下方的 1/6  ，目的是去掉旋转带来的黑边
	cv::Mat image1,image2;

	if(head == UP)
	{
		int cut_size = src_image2.rows/6;
		int image1_cut_size = src_image1.rows - (dest_image.rows - (src_image2.rows - cut_size));
		Image_cut(src_image1, image1, UP, image1_cut_size);
		Image_cut(src_image2, image2, DOWN, cut_size);

		// y  > 0 , 恒成立
		// x > 0, 表示image2  相对于image1 右移
		// x < 0, 表示image2  相对于image1 左移

		if(distance.x < 0)
		{
			image1.copyTo(dest_image(cv::Rect(std::abs(distance.x), image2.rows -1, image1.cols, image1.rows)));
			image2.copyTo(dest_image(cv::Rect(0, 0, image2.cols, image2.rows)));
		}
		else
		{
			image1.copyTo(dest_image(cv::Rect(0, image2.rows -1, image1.cols, image1.rows)));
			image2.copyTo(dest_image(cv::Rect(distance.x, 0, image2.cols, image2.rows)));
		}
		
	}
	else if(head == DOWN)
	{
		
	}
	else if(head == LEFT)
	{
		
	}
	else if(head == RIGHT)
	{
		
	}
	
	
	

#ifdef DUBUG
	std::cout << "dest image cols:" << dest_image.cols << ", rows:" << dest_image.rows << std::endl;
	std::cout << "image1     cols:" << image1.cols << ", rows:" << image1.rows << std::endl;
	std::cout << "image2     cols:" << image2.cols << ", rows:" << image2.rows << std::endl;
#endif


#if 0
	if(head == UP)
	{
		// y  > 0 , 恒成立
		// x > 0, 表示image2  相对于image1 右移
		// x < 0, 表示image2  相对于image1 左移	
		if(distance.x < 0){
			left_top.x = std::abs(distance.x);
			left_top.y = distance.y;

			src_image1.copyTo(dest_image(cv::Rect(left_top.x, left_top.y, src_image1.cols, src_image1.rows)));
			image2.copyTo(dest_image(cv::Rect(0,0, image2.cols, image2.rows)));
    	}else{
    		left_top.x = 0;
			left_top.y = distance.y;

			src_image1.copyTo(dest_image(cv::Rect(left_top.x, left_top.y, src_image1.cols, src_image1.rows)));
			image2.copyTo(dest_image(cv::Rect(distance.x, 0, image2.cols, image2.rows)));
		}

		right_bottom.x = left_top.x + src_image1.cols;
		right_bottom.y = left_top.y + src_image1.rows;
#if 0
		//接口优化

		int w = 100;
		int dst_rows_start = image2.rows - 1;
		int dst_cols_start = 0;
		int img1_rows_start = src_image1.rows - (dest_image.rows - image2.rows) - 1;
		int img1_cols_start = distance.x < 0 ? std::abs(distance.x) : 0;
		int img2_rows_start = image2.rows - 1;
		int img2_cols_start = distance.x < 0 ? 0 : distance.x;

		float alpha = 1;//img1中像素的权重
	
		for(int i=0; i<w; i++)//rows
		{
			uchar* p = src_image1.ptr<uchar>(img1_rows_start - i);
			uchar* t = image2.ptr<uchar>(img2_rows_start - i);
        	uchar* d = dest_image.ptr<uchar>(dst_rows_start - i);

			for(int j=0; j<dest_image.cols - std::abs(distance.x); j++)
			{
				//如果遇到img2 中无像素的黑点，则完全拷贝img1 中的数据
				if(t[(img2_cols_start + j) * 3] == 0 &&
					t[(img2_cols_start + j) * 3 + 1] == 0 &&
					t[(img2_cols_start + j) * 3 + 2] == 0)
				{
					alpha = 1.0f;
				}
				else
				{
					alpha = ((float)w - (float)i) / (float)w;
				}


				d[(dst_cols_start + j) * 3] = p[(img1_cols_start + j) * 3] * alpha + t[(img2_cols_start + j) * 3] * (1 - alpha);
				d[(dst_cols_start + j) * 3 + 1] = p[(img1_cols_start + j) * 3 + 1] * alpha + t[(img2_cols_start + j) * 3 + 1] * (1 - alpha);
				d[(dst_cols_start + j) * 3 + 2] = p[(img1_cols_start + j) * 3 + 2] * alpha + t[(img2_cols_start + j) * 3 + 2] * (1 - alpha);
			}		
		}
#endif
	}
	else if(head == DOWN)
	{
		
	}
	else if(head == LEFT)
	{

	}
	else if(head == RIGHT)
	{
		
	}

#endif
	
	return OK;
}


} //namespace IMAGE_MOSAIC
