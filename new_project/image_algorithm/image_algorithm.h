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
class Image_algorithm
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
	**   ͼ�����ת
	**   src_image :   ��Ҫ��ת��ͼƬ
	**   dest_image:  ��ת���Ŀ��ͼƬ
	**   angle:  ͼ��������Ҫ��ת�ĽǶ�
	*/
	void Image_rotate(cv::Mat& src_image, cv::Mat& dest_image, double angle);

	/*
	**  ��������ͼƬ���Ӵ���λ��
	**  src_image1:  ��Ҫƴ�ӵĵ�һ��ͼƬ
	**  src_image2:  ��Ҫƴ�ӵĵڶ���ͼƬ
	**  head:   ����ͼƬƴ�ӵķ���,  ��Image_mosaic_head
	**  distance :  distance.x ��ʾ ����ͼ���������룬y ��ʾ����ͼ����������
	**  �ɹ�����OK,  ʧ�ܷ��� ERR
	*/
	int Image_mosaic_algorithm(cv::Mat &src_image1, cv::Mat &src_image2, int head, cv::Point2f &distance);

	/*
	**	ͼ��ƴ�Ӻ���
	**    src_image1:  ��Ҫƴ�ӵĵ�һ��ͼƬ
	**	src_image2:  ��Ҫƴ�ӵĵڶ���ͼƬ
	**   dest_image:   ƴ����������ͼƬ
	**   distace:  ����ͼƬ���Ӵ���λ��
	**   left_top  right_bottom:  ��һ��ͼƬƴ�Ӻ��� dest_image �е�λ��
	**  �ɹ����� OK,  ʧ�ܷ��� ERR
	*/
	int Optimize_seam(cv::Mat& src_image1, cv::Mat& src_image2, cv::Mat& dest_image, cv::Point2f distace, cv::Point2f &left_top, cv::Point2f &right_bottom);


};

}//namespace IMAGE_MOSAIC

#endif //IMAGE_ALGORITHM_H