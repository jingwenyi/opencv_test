#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;


typedef std::set<std::pair<int,int> > MatchesSet;

typedef struct
{
    Point2f left_top;
    Point2f left_bottom;
    Point2f right_top;
    Point2f right_bottom;
}four_corners_t;

four_corners_t corners;

void CalcCorners(const Mat& H, const Mat& src)
{
    double v2[] = { 0, 0, 1 };//左上角
    double v1[3];//变换后的坐标值
    Mat V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    Mat V1 = Mat(3, 1, CV_64FC1, v1);  //列向量

    V1 = H * V2;
    //左上角(0,0,1)
    cout << "V2: " << V2 << endl;
    cout << "V1: " << V1 << endl;
    corners.left_top.x = v1[0] / v1[2];
    corners.left_top.y = v1[1] / v1[2];

    //左下角(0,src.rows,1)
    v2[0] = 0;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;
    corners.left_bottom.x = v1[0] / v1[2];
    corners.left_bottom.y = v1[1] / v1[2];

    //右上角(src.cols,0,1)
    v2[0] = src.cols;
    v2[1] = 0;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;
    corners.right_top.x = v1[0] / v1[2];
    corners.right_top.y = v1[1] / v1[2];

    //右下角(src.cols,src.rows,1)
    v2[0] = src.cols;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;
    corners.right_bottom.x = v1[0] / v1[2];
    corners.right_bottom.y = v1[1] / v1[2];

}




int main(int argc, char *argv[])
{
	Mat image01 = imread("./test_photo/DSC00032.JPG");
	Mat image02 = imread("./test_photo/DSC00033.JPG");

	Mat image1, image2;
	cvtColor(image01, image1, CV_RGB2GRAY);
	cvtColor(image02, image2, CV_RGB2GRAY);

	std::vector<KeyPoint> keyPoints1, keyPoints2;
	Ptr<SURF> detector = SURF::create(2000);
	detector->detect(image1, keyPoints1);
	detector->detect(image2, keyPoints2);

	cout << "keypints1:" << keyPoints1.size() << endl;
	cout << "keypints2:" << keyPoints2.size() << endl;

#if 0
	drawKeypoints( image1, keyPoints1, image1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

	drawKeypoints( image2, keyPoints2, image2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

	imshow("k1", image1);
	imshow("k2", image2);

#endif

	Ptr<SURF> extractor = SURF::create();
	Mat descriptors1, descriptors2;
	extractor->compute(image1, keyPoints1, descriptors1);
	extractor->compute(image2, keyPoints2, descriptors2);

#if 0
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
	std::vector<DMatch> matches;
	matcher->match(descriptors1, descriptors2, matches);

	Mat imgMatches;
	drawMatches(image1, keyPoints1, image2, keyPoints2, matches, imgMatches);
	imshow("m", imgMatches);

#endif
	

	FlannBasedMatcher matcher;
	std::vector<std::vector<DMatch>> matchePoints;
	std::vector<DMatch> GoodMatchePoints;

	MatchesSet matches;
	
	std::vector<Mat> train_desc(1, descriptors1);
	matcher.add(train_desc);
	matcher.train();
	matcher.knnMatch(descriptors2, matchePoints, 2);


	for(int i=0; i<matchePoints.size(); i++)
	{
		if(matchePoints[i][0].distance < 0.4 * matchePoints[i][1].distance)
		{
			GoodMatchePoints.push_back(matchePoints[i][0]);
			matches.insert(make_pair(matchePoints[i][0].queryIdx, matchePoints[i][0].trainIdx));
		}
	}
	
	cout <<"n1-> matches:" << GoodMatchePoints.size() << endl;
	
	FlannBasedMatcher matcher2;
	matchePoints.clear();
	std::vector<Mat> train_desc2(1, descriptors2);
	matcher2.add(train_desc2);
	matcher2.train();
	matcher2.knnMatch(descriptors1, matchePoints, 2);
	
	for(int i=0; i<matchePoints.size(); i++)
	{
		if(matchePoints[i][0].distance < 0.4 * matchePoints[i][1].distance)
		{
			if (matches.find(make_pair(matchePoints[i][0].trainIdx, matchePoints[i][0].queryIdx)) == matches.end())
        		{
            			GoodMatchePoints.push_back(DMatch(matchePoints[i][0].trainIdx, matchePoints[i][0].queryIdx, matchePoints[i][0].distance));
        		}
		}
	}
	
	cout << "1->2 & 2->1 matches:" << GoodMatchePoints.size() << endl;

#if 0
	Mat first_match;
	drawMatches(image02, keyPoints2, image01, keyPoints1, GoodMatchePoints, first_match);
	imshow("first_match", first_match);

#endif

	std::vector<Point2f> imagePoints1, imagePoints2;
	for(int i=0; i<GoodMatchePoints.size(); i++)
	{
		imagePoints2.push_back(keyPoints2[GoodMatchePoints[i].queryIdx].pt);
        	imagePoints1.push_back(keyPoints1[GoodMatchePoints[i].trainIdx].pt);
	}

	Mat homo = findHomography(imagePoints1, imagePoints2, CV_RANSAC);
	cout << "H:\n" << homo << endl << endl;

	CalcCorners(homo, image01);

	cout << "left_top:" << corners.left_top << endl;
   	cout << "left_bottom:" << corners.left_bottom << endl;
    	cout << "right_top:" << corners.right_top << endl;
    	cout << "right_bottom:" << corners.right_bottom << endl;
	
	Mat imageTransform;
	warpPerspective(image01, imageTransform, homo, Size(MAX(corners.right_top.x, corners.right_bottom.x), image02.rows));
    	
#if 0
	imwrite("trans1.jpg", imageTransform);
	imshow("tr", imageTransform);
	

#endif

	int dst_width = imageTransform.cols;
	int dst_height = image02.rows;
	Mat dst(dst_height, dst_width, CV_8UC3);
	dst.setTo(0);
	imageTransform.copyTo(dst(Rect(0, 0, imageTransform.cols, imageTransform.rows)));
	image02.copyTo(dst(Rect(0 , 0, image02.cols, image02.rows)));
	imwrite("dst.jpg", dst);
	imshow("dest", dst);
	
	cout << "-----------ok----------------" << endl;
	waitKey();


	return 0;

}
