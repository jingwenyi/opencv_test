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
	int Image_mosaic_algorithm(cv::Mat &src_image1, cv::Mat &src_image2, int head, cv::Point2i &distance);

	int Image_mosaic_up_algorithm(cv::Mat &src_image1, cv::Mat &src_image2, cv::Point2i &distance);

	int Image_mosaic_down_algorithm(cv::Mat &src_image1, cv::Mat &src_image2, cv::Point2i &distance);

	int Image_mosaic_left_algorithm(cv::Mat &src_image1, cv::Mat &src_image2, cv::Point2i &distance);

	int Image_mosaic_right_algorithm(cv::Mat &src_image1, cv::Mat &src_image2, cv::Point2i &distance);

	/*
	**	ͼ��ƴ�Ӻ���
	**    src_image1:  ��Ҫƴ�ӵĵ�һ��ͼƬ
	**	src_image2:  ��Ҫƴ�ӵĵڶ���ͼƬ
	**   dest_image:   ƴ����������ͼƬ
	**   distace:  ����ͼƬ���Ӵ���λ��
	**   left_top  right_bottom:  ��һ��ͼƬƴ�Ӻ��� dest_image �е�λ��
	**  �ɹ����� OK,  ʧ�ܷ��� ERR
	*/
	int Image_optimize_seam(cv::Mat& src_image1, cv::Mat& src_image2, cv::Mat& dest_image, cv::Point2i distance,
											enum Image_mosaic_head head, cv::Point2i &image1_vertex, cv::Point2i &image2_vertex);

	void Get_sample_size_up_down(cv::Point2i image_size, cv::Point2i &sample_size, int &dis); 

	void Get_sample_size_left_right(cv::Point2i image_size, cv::Point2i &sample_size, int &dis); 

	void Image_resize(cv::Mat& src_image, cv::Mat& dest_image, cv::Size dsize);
	void Image_cut(cv::Mat& src_image, cv::Mat& dest_image, enum Image_mosaic_head head, int cut_size);

	//���� ͼƬ��roll  �� pitch  ��ͼ�����͸�ӱ任
	void Image_perspective(cv::Mat& src_image, cv::Mat& dest_image, float roll, float pitch);

	// ���ټ���Ŀ��ͼƬ�ڵ�ͼ�е�λ��
	void Fast_calc_dest_point(cv::Mat& src_image1, cv::Point2i distance,
										enum Image_mosaic_head head, cv::Point2i &image1_vertex, cv::Point2i &image2_vertex);

	//�Ƚ������غϵ�ͼƬ, ���ҵľ��룬�������캽��ƴ��ʱ
	void Image_fast_mosaic_algorithm(cv::Mat &src_image1, cv::Mat &src_image2, cv::Point2i &distance);
	void Image_fast_mosaic_algorithm2(cv::Mat &src_image1, cv::Mat &src_image2, cv::Point2i &distance);

	void Image_fast_mosaic_algorithm3(cv::Mat &src_image1, cv::Mat &src_image2, cv::Point2i &distance);

	
	void Image_fast_mosaic_algorithm4(cv::Mat &src_image1, cv::Mat &src_image2, cv::Point2i &distance);

	float Get_distance(const struct Location &loc1, const struct Location &loc2);

	float Longitude_scale(const struct Location &loc);

	float Constrain_float(float amt, float low, float high);

	float Get_bearing_cd(const struct Location &loc1, const struct Location &loc2);

	void Location_update(struct Location &loc, float bearing, float distance);

	void Location_offset(struct Location &loc, float ofs_north, float ofs_east);

	bool Is_zero(float a);

	void Location_update_baseon_pitch_roll(struct Location & loc, const struct Location base, const struct Imu_data imu);
};

}//namespace IMAGE_MOSAIC

#endif //IMAGE_ALGORITHM_H
