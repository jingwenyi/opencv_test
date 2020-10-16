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

public:  //for function
	void Image_rotate(cv::Mat& src_image, cv::Mat& dest_image, double angle);

	void Image_resize(cv::Mat& src_image, cv::Mat& dest_image, cv::Size dsize);

	float Get_distance(const struct Location &loc1, const struct Location &loc2);

	float Longitude_scale(const struct Location &loc);

	float Constrain_float(float amt, float low, float high);

	float Get_bearing_cd(const struct Location &loc1, const struct Location &loc2);

	void Location_update(struct Location &loc, float bearing, float distance);

	void Location_offset(struct Location &loc, float ofs_north, float ofs_east);

	bool Is_zero(float a);
	
	void Image_fast_mosaic_algorithm(cv::Mat &src_image1, cv::Mat &src_image2, cv::Point2i &distance);

};

class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};


class Image_feature_points_extraction
{
public:
	Image_feature_points_extraction();
	void Image_extract_feature_point(cv::InputArray _image, std::vector<cv::KeyPoint>& _keypoints, cv::OutputArray _descriptors);
	void ComputePyramid(cv::Mat image);
	void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
	std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                       const int &maxX, const int &minY, const int &maxY, const int &N, const int &level);

	int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

	int Feature_points_match(std::vector<cv::KeyPoint>& image1_keypoints, cv::Mat& image1_descriptors,
					std::vector<cv::KeyPoint>& image2_keypoints, cv::Mat& image2_descriptors,std::vector<int> &vnMatches12);

	void drawKeyPointsMatch(cv::Mat image1, std::vector<cv::KeyPoint>& image1_keypoints,
					cv::Mat image2, std::vector<cv::KeyPoint>& image2_keypoints, std::vector<int> &vnMatches12, cv::Mat &image_match);

private:
	int nfeatures;
	double scaleFactor;
	int nlevels;
	int iniThFAST;
    int minThFAST;
	std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;

	std::vector<cv::Mat> mvImagePyramid;

	std::vector<int> mnFeaturesPerLevel;

	std::vector<int> umax;

	std::vector<cv::Point> pattern;
};


}//namespace IMAGE_MOSAIC

#endif //IMAGE_ALGORITHM_H

