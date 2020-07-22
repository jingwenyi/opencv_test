#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>


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

void Rotate(const Mat &srcImage, Mat &destImage)
{
	for(int i=0; i<srcImage.cols; i++)
	{
		for(int j=0; j<srcImage.rows; j++)
		{
			Scalar color = srcImage.at<Vec3b>(srcImage.rows - j -1, i);
			destImage.at<Vec3b>(i,j) = Vec3b(color(0), color(1), color(2));
		}
	}
}


int main_test(int argc, char *argv[])
{
	Mat image01= imread("./test_photo/DSC00032.JPG");
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

#if 1
	Ptr<DescriptorMatcher> matcher11 = DescriptorMatcher::create("BruteForce");
	std::vector<DMatch> matches11;
	matcher11->match(descriptors1, descriptors2, matches11);

	Mat imgMatches;
	drawMatches(image1, keyPoints1, image2, keyPoints2, matches11, imgMatches);
	imwrite("keypoint.jpg", imgMatches);
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
#if 0	
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
#endif

#if 1
	Mat img_matches;
	drawMatches(image1, keyPoints1, image2, keyPoints2, GoodMatchePoints,
				img_matches, Scalar::all(-1), Scalar::all(-1), std::vector<char>(), 
				DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	imwrite("good_matches.jpg", img_matches);
	imshow("good_matches", img_matches);

#endif
	

	//The edge of the image is counted by matching feature points
	std::vector<Point2f> imagePoints1, imagePoints2;
	// 1.Find the largest edge
	
	for(int i=0; i<GoodMatchePoints.size(); i++)
	{
		imagePoints1.push_back(keyPoints1[GoodMatchePoints[i].trainIdx].pt);
		imagePoints2.push_back(keyPoints2[GoodMatchePoints[i].queryIdx].pt);
		cout << "image1:x=" << imagePoints1[i].x << ",y=" << imagePoints1[i].y;
		cout << "image2:x=" << imagePoints2[i].x << ",y=" << imagePoints2[i].y << endl;
		
	}
	
	
	
	
	

	

	
	cout << "-----------ok----------------" << endl;
	waitKey();


	return 0;

}


//orb test
int main(int argc, char *argv[])
{
	Mat image01= imread("./test_photo/DSC00032.JPG");
	Mat image02 = imread("./test_photo/DSC00033.JPG");

	//�Ҷ�ͼת��  
	Mat image1, image2;
	cvtColor(image01, image1, CV_RGB2GRAY);
	cvtColor(image02, image2, CV_RGB2GRAY);



	Ptr<ORB> orb = ORB::create(2000);
	std::vector<KeyPoint> keypoints1, keypoints2;
	orb->detect(image1, keypoints1);
	orb->detect(image2, keypoints2);

	Mat descriptors1, descriptors2;
	orb->compute(image1, keypoints1, descriptors1);
	orb->compute(image2, keypoints1, descriptors2);
#if 0
	flann::Index flannIndex(descriptors1, flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);

    vector<DMatch> GoodMatchePoints;

    Mat macthIndex(descriptors2.rows, 2, CV_32SC1), matchDistance(descriptors2.rows, 2, CV_32FC1);
    flannIndex.knnSearch(descriptors2, macthIndex, matchDistance, 2, flann::SearchParams());

    // Lowe's algorithm,��ȡ����ƥ���
    for (int i = 0; i < matchDistance.rows; i++)
    {
        if (matchDistance.at<float>(i, 0) < 0.5 * matchDistance.at<float>(i, 1))
        {
            DMatch dmatches(i, macthIndex.at<int>(i, 0), matchDistance.at<float>(i, 0));
            GoodMatchePoints.push_back(dmatches);
        }
    }

	cout << "good matches:" << GoodMatchePoints.size() << endl;

    Mat first_match;
    drawMatches(image02, keypoints2, image01, keypoints1, GoodMatchePoints, first_match, Scalar::all(-1), Scalar::all(-1), std::vector<char>(), 
				DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	imwrite("first_match.jpg", first_match);
    imshow("first_match ", first_match);

    vector<Point2f> imagePoints1, imagePoints2;

    for (int i = 0; i<GoodMatchePoints.size(); i++)
    {
        imagePoints2.push_back(keypoints2[GoodMatchePoints[i].queryIdx].pt);
        imagePoints1.push_back(keypoints1[GoodMatchePoints[i].trainIdx].pt);
    }
#else
	FlannBasedMatcher matcher;
	std::vector<DMatch> matches;

	if(descriptors1.type() != CV_32F)
	{
		descriptors1.convertTo(descriptors1, CV_32F);
		descriptors2.convertTo(descriptors2, CV_32F);
	}

	matcher.match(descriptors1, descriptors2, matches);

	double min_dist = min_element(matches.begin(), matches.end(),
					[](const DMatch& d1, const DMatch& d2)->double
	{
		return d1.distance < d2.distance;
	})->distance;

	cout << "min distance" << min_dist << endl;

	vector<DMatch> good_matches;
	for(int i = 0; i < descriptors1.rows; i++)
	{
		if(matches[i].distance < max<double>(min_dist * 2, 60.0))
		{
			good_matches.push_back(matches[i]);
		}
	}

	cout << "good matches size:" << good_matches.size() << endl;

	Mat img_matches;
	drawMatches(image01, keypoints1, image02, keypoints2,
			good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
			std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	imwrite("good_matches.jpg", img_matches);
	imshow("good matches", img_matches);
	
#endif

	cout << "------------ok-----------" << endl;
	waitKey();


	return 0;
}
