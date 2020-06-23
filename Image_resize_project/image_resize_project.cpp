#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace std;
using namespace cv;



int main()
{
	string strFile = "./Image_src/src.txt";
	ifstream f;
	f.open(strFile.c_str());
	
	//skip first one lines
	string s0;
	getline(f, s0);
	cout << s0 << endl;
	
	while(!f.eof())
	{
		string s;
		getline(f, s);
		if(!s.empty())
		{
			string imageName = "./Image_src/" + s;
			cout << imageName <<endl; 
			Mat srcImage  = imread(imageName.c_str());
			
			Mat dstImage;
			resize(srcImage, dstImage, Size(srcImage.cols/10, srcImage.rows/10), INTER_AREA);
			string destName = "./Image_dest/" + s;
			imwrite(destName, dstImage);
		}
		
	}

	waitKey();
	return 0;
}



