#include "image_algorithm.h"

namespace IMAGE_MOSAIC
{
void Image_algorithm::Image_rotate(cv::Mat& src_image,  cv::Mat& dest_image, double angle)
{
	cv::Point2f pt(src_image.cols/2, src_image.rows/2);
	cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);
	cv::warpAffine(src_image, dest_image, r, cv::Size(src_image.cols, src_image.rows));
}

int Image_algorithm::Image_mosaic_algorithm(cv::Mat &src_image1, cv::Mat &src_image2, int head, cv::Point2f &distance)
{
	return OK;
}

int Image_algorithm::Optimize_seam(cv::Mat& src_image1, cv::Mat& src_image2, cv::Mat& dest_image, cv::Point2f distace, cv::Point2f &left_top, cv::Point2f &right_bottom)
{

	return OK;
}


} //namespace IMAGE_MOSAIC
