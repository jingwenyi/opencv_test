#ifndef IMAGE_ALGORITHM_H
#define IMAGE_ALGORITHM_H

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <limits.h>


namespace IMAGE_MOSAIC
{
class Point
{
public:
	Point(int x1 = 0, int y1 = 0):x(x1), y(y1){}
public:
	int x,y;
};

class image_algorithm
{
public:

	enum Image_mosaic_head{
        UP=0,
        DOWN=1,
        LEFT=2,
        RIGHT=3
    };

	enum{
		OK = 0,
		ERR = 1
	};

	/*
	**   图像的旋转
	**   src_image :   需要旋转的图片
	**   dest_image:  旋转后的目标图片
	**   angle:  图像内容需要旋转的角度
	*/
	void Image_rotate(cv::Mat& src_image, cv::Mat& dest_image, double angle);

	/*
	**  计算两张图片连接处的位置
	**  src_image1:  需要拼接的第一张图片
	**  src_image2:  需要拼接的第二张图片
	**  head:   两张图片拼接的方向,  见Image_mosaic_head
	**  distance :  distance.x 表示 两幅图像的列向距离，y 表示两幅图像的行向距离
	**  成功返回OK,  失败返回 ERR
	*/
	int Image_mosaic_algorithm(cv::Mat &src_image1, cv::Mat &src_image2, int head, Point &distance);

	/*
	**	图像拼接函数
	**    src_image1:  需要拼接的第一张图片
	**	src_image2:  需要拼接的第二张图片
	**   dest_image:   拼接完成输出的图片
	**   distace:  两张图片连接处的位置
	**   left_top  right_bottom:  第一张图片拼接后在 dest_image 中的位置
	**  成功返回 OK,  失败返回 ERR
	*/
	int Optimize_seam(cv::Mat& src_image1, cv::Mat& src_image2, cv::Mat& dest_image, Point distace, Point &left_top, Point &right_bottom);
	

private:

};

}//namespace IMAGE_MOSAIC

#endif //IMAGE_ALGORITHM_H