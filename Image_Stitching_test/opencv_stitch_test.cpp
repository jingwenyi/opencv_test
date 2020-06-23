#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/stitching/stitcher.hpp>


using namespace std;

using namespace cv;


bool try_use_gpu = false;

vector<Mat> imgs;


string result_name = "dst1.jpg";


int main(int argc, char *argv[])
{
	Mat img1 = imread("34.jpg");
	Mat img2 = imread("35.jpg");
	
	imshow("P1", img1);
	imshow("P2", img2);

	if(img1.empty() || img2.empty())
	{
		cout << "can't read image" <<endl;
		return -1;
	}

	imgs.push_back(img1);
	imgs.push_back(img2);

	Stitcher stitcher = Stitcher::createDefault(try_use_gpu);

	Mat pano;
	Stitcher::Status status = stitcher.stitch(imgs, pano);

	if(status != Stitcher::OK)
	{
		cout <<"can't stitch images, error code =" <<int( status) << endl;
		return -1;
	}

	imwrite(result_name, pano);
	Mat pano2 = pano.clone();
	imshow("pano",pano);
	if(waitKey() == 27)  return 0;

	cout << "---------ok------------"<<endl;

}
