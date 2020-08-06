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




//算法思路:   用3 组块匹配，然后在3  组最佳的里面进行块连续性匹配
int Image_algorithm::Image_mosaic_algorithm(cv::Mat &src_image1, cv::Mat &src_image2, int head, cv::Point2i &distance)
{
	if(head == UP)
	{
		return Image_mosaic_up_algorithm(src_image1, src_image2, distance);
	}
	else if(head == DOWN)
	{
		return Image_mosaic_down_algorithm(src_image1, src_image2, distance);
	}
	else if(head == LEFT)
	{
		return Image_mosaic_left_algorithm(src_image1, src_image2, distance);
	}
	else if(head == RIGHT)
	{
		return Image_mosaic_right_algorithm(src_image1, src_image2, distance);
	}

	return ERR;
}


int Image_algorithm::Image_mosaic_up_algorithm(cv::Mat &src_image1, cv::Mat &src_image2, cv::Point2i &distance)
{
	cv::Point2i image1_sample_size(src_image1.cols / 2, src_image1.rows / 20);

#ifdef DUBUG
		std::cout << "image_mosaic_algorithm" << image1_sample_size.x << "," << image1_sample_size.y <<std::endl;
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
