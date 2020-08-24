#include "./image_algorithm/image_algorithm.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <limits.h>
using namespace std;
using namespace cv;


const static struct IMAGE_MOSAIC::Location gps_base(1350.896 * 100, 41.31896004908 * 1.0e7, 114.72281532399 * 1.0e7);



//AB 航线上有27 张图片，对应27 个偏航角
const static float yaw_AB[] ={	92.35f,	91.36f, 87.14f, 91.96f, 85.27f, 94.03f, 90.73f, 95.61f, 95.35f,
							  	94.07f, 99.07f, 88.51f, 89.47f, 90.03f, 90.48f, 87.24f, 88.03f, 91.4f,
							  	93.31f, 91.7f, 	92.13f, 93.33f, 92.82f, 92.64f, 92.29f, 90.83f, 96.42f}; 
const static float roll_AB[] = {-0.27f,	-1.1f,	0.55f,	-2.09f,	2.88f,	-1.6f,	-3.24f,	-3.71f,	-5.25f,
								-3.4f,	-7.18f,	-6.89f,	-1.32f,	-1.24f,	-1.38f,	-3.81f,	-0.92f,	-1.13f,
								-3.89f,	-3.93f,	-2.58f,	-2.9f,	-4.16f,	-3.93f,	-4.68f,	-3.94f,	-3.88f};

const static float pitch_AB[] = {-1.71f,	1.24f,		-0.16f,		-1.61f,		-3.86f,		0.36f,		1.53f,		3.25f,		1.62f,
								 1.86f,		2.53f,		3.8f,		5.19f,		4.96f,		3.19f,		3.37f,		4.49f,		4.3f,
								 2.91f,		4.11f,		4.55f,		4.63f,		4.85f,		6.11f,		5.13f,		7.19f,		6.77f};

const static struct IMAGE_MOSAIC::Location  gps_location_AB[] = {
	IMAGE_MOSAIC::Location(1661.284 	* 	100,	41.3240930316	*	1.0e7,	114.716242598	*	1.0e7),
	IMAGE_MOSAIC::Location(1662.41 		* 	100,	41.3241013967	*	1.0e7,	114.716795434	*	1.0e7),
	IMAGE_MOSAIC::Location(1663.425 	* 	100,	41.3241076288	*	1.0e7,	114.717313666	*	1.0e7),
	IMAGE_MOSAIC::Location(1662.302		*	100,	41.3241124796	*	1.0e7,	114.717815475	*	1.0e7),
	IMAGE_MOSAIC::Location(1659.247		*	100,	41.3241136619	*	1.0e7,	114.718382046	*	1.0e7),
	IMAGE_MOSAIC::Location(1656.947		*	100,	41.3241072933	*	1.0e7,	114.718908279	*	1.0e7),
	IMAGE_MOSAIC::Location(1656.033		*	100,	41.3240971905	*	1.0e7,	114.71945056	*	1.0e7),
	IMAGE_MOSAIC::Location(1656.396		*	100,	41.3240911417	*	1.0e7,	114.719994238	*	1.0e7),
	IMAGE_MOSAIC::Location(1656.839		*	100,	41.3240843298	*	1.0e7,	114.72053507	*	1.0e7),
	IMAGE_MOSAIC::Location(1656.918		*	100,	41.3240789156	*	1.0e7,	114.721073507	*	1.0e7),
	IMAGE_MOSAIC::Location(1656.02		*	100,	41.3240653305	*	1.0e7,	114.721612697	*	1.0e7),
	IMAGE_MOSAIC::Location(1654.595		*	100,	41.3240572545	*	1.0e7,	114.722163027	*	1.0e7),
	IMAGE_MOSAIC::Location(1655.074		*	100,	41.3240675804	*	1.0e7,	114.722724531	*	1.0e7),
	IMAGE_MOSAIC::Location(1656.678		*	100,	41.3240774818	*	1.0e7,	114.723282714	*	1.0e7),
	IMAGE_MOSAIC::Location(1657.645		*	100,	41.3240820722	*	1.0e7,	114.723829769	*	1.0e7),
	IMAGE_MOSAIC::Location(1657.499		*	100,	41.3240815304	*	1.0e7,	114.724376005	*	1.0e7),
	IMAGE_MOSAIC::Location(1657.581		*	100,	41.3240872362	*	1.0e7,	114.72492152	*	1.0e7),
	IMAGE_MOSAIC::Location(1657.993		*	100,	41.3240913488	*	1.0e7,	114.725460657	*	1.0e7),
	IMAGE_MOSAIC::Location(1657.894		*	100,	41.3240873672	*	1.0e7,	114.725992663	*	1.0e7),
	IMAGE_MOSAIC::Location(1657.128		*	100,	41.324081934	*	1.0e7,	114.726525264	*	1.0e7),
	IMAGE_MOSAIC::Location(1657.06		*	100,	41.324079872	*	1.0e7,	114.727059529	*	1.0e7),
	IMAGE_MOSAIC::Location(1656.932		*	100,	41.3240776387	*	1.0e7,	114.727594152	*	1.0e7),
	IMAGE_MOSAIC::Location(1656.703		*	100,	41.3240739204	*	1.0e7,	114.72813138	*	1.0e7),
	IMAGE_MOSAIC::Location(1656.611		*	100,	41.3240709119	*	1.0e7,	114.72867214	*	1.0e7),
	IMAGE_MOSAIC::Location(1656.386		*	100,	41.3240677552	*	1.0e7,	114.729215299	*	1.0e7),
	IMAGE_MOSAIC::Location(1656.181		*	100,	41.3240671039	*	1.0e7,	114.729760218	*	1.0e7),
	IMAGE_MOSAIC::Location(1656.495		*	100,	41.3240669368	*	1.0e7,	114.730300246	*	1.0e7)
};


//CD 航线上有29 张图片，对应29 个偏航角
const static float yaw_CD[] = {	281.47f,	287.41f,	282.65f,	290.22f,	284.3f,		283.5f,		285.75f,	288.84f,	290.7f,		284.89f,
								284.42f,	288.77f,	280.14f,	285.65f,	285.61f,	287.46f,	286.84f,	286.02f,	276.95f,	285.61f,
								280.0f,		278.9f,		283.64f,	281.82f,	285.08f,	283.47f,	289.31f,	279.25f,	283.06f};

const static float roll_CD[] = {-1.77f,	-5.83f,	-2.5f,	-6.23f,	-3.25f,	-2.29f,	-0.09f,	-1.16f,	-2.01f,	-6.98f,
								-0.28f,	-5.35f,	-1.73f,	-3.52f,	-1.54f,	-2.96f,	-4.36f,	-1.75f,	-2.54f,	-1.83f,
								-1.21f,	-1.53f,	-2.88f,	-3.28f,	-5.39f,	-1.38f,	-8.57f,	2.89f,	-4.76f};

const static float pitch_CD[] = {3.32f,	5.58f,	5.78f,	4.55f,	5.34f,	4.85f,	3.15f,	4.64f,	1.78f,	5.06f,
								 3.95f,	2.89f,	4.44f,	5.68f,	5.66f,	5.06f,	4.84f,	4.01f,	1.34f,	1.76f,
								 1.29f,	2.7f,	-0.13f,	3.27f,	4.38f,	0.86f,	2.37f,	1.31f,	3.69f};

const static struct IMAGE_MOSAIC::Location  gps_location_CD[] = {
		IMAGE_MOSAIC::Location(1658.676	* 	100,	41.3232205156	*	1.0e7,	114.730368742	*	1.0e7),
		IMAGE_MOSAIC::Location(1657.946	* 	100,	41.323229685	*	1.0e7,	114.729830524	*	1.0e7),
		IMAGE_MOSAIC::Location(1659.802	* 	100,	41.3232266136	*	1.0e7,	114.729326347	*	1.0e7),
		IMAGE_MOSAIC::Location(1657.899	* 	100,	41.323229404	*	1.0e7,	114.728825886	*	1.0e7),
		IMAGE_MOSAIC::Location(1659.131	* 	100,	41.3232303406	*	1.0e7,	114.728313658	*	1.0e7),
		IMAGE_MOSAIC::Location(1660.181	* 	100,	41.3232176903	*	1.0e7,	114.727801056	*	1.0e7),
		IMAGE_MOSAIC::Location(1659.754	* 	100,	41.3232106502	*	1.0e7,	114.727276914	*	1.0e7),
		IMAGE_MOSAIC::Location(1660.669	* 	100,	41.3232178605	*	1.0e7,	114.726770072	*	1.0e7),
		IMAGE_MOSAIC::Location(1660.223	* 	100,	41.3232219399	*	1.0e7,	114.72624652	*	1.0e7),
		IMAGE_MOSAIC::Location(1659.152	* 	100,	41.3232394106	*	1.0e7,	114.725715129	*	1.0e7),
		IMAGE_MOSAIC::Location(1660.504	* 	100,	41.3232331906	*	1.0e7,	114.725210417	*	1.0e7),
		IMAGE_MOSAIC::Location(1659.513	* 	100,	41.3232378156	*	1.0e7,	114.724692513	*	1.0e7),
		IMAGE_MOSAIC::Location(1658.733	* 	100,	41.3232268093	*	1.0e7,	114.724191541	*	1.0e7),
		IMAGE_MOSAIC::Location(1656.709	* 	100,	41.3232130393	*	1.0e7,	114.723671704	*	1.0e7),
		IMAGE_MOSAIC::Location(1657.609	* 	100,	41.323218376	*	1.0e7,	114.723158064	*	1.0e7),
		IMAGE_MOSAIC::Location(1657.931	* 	100,	41.3232201063	*	1.0e7,	114.72265273	*	1.0e7),
		IMAGE_MOSAIC::Location(1657.629	* 	100,	41.3232268274	*	1.0e7,	114.72212558	*	1.0e7),
		IMAGE_MOSAIC::Location(1660.35	* 	100,	41.3232271078	*	1.0e7,	114.721619691	*	1.0e7),
		IMAGE_MOSAIC::Location(1661.705	* 	100,	41.3232268733	*	1.0e7,	114.721083064	*	1.0e7),
		IMAGE_MOSAIC::Location(1663.221	* 	100,	41.3232196562	*	1.0e7,	114.720574945	*	1.0e7),
		IMAGE_MOSAIC::Location(1660.39	* 	100,	41.3232234596	*	1.0e7,	114.720036358	*	1.0e7),
		IMAGE_MOSAIC::Location(1659.79	* 	100,	41.3232180549	*	1.0e7,	114.719522996	*	1.0e7),
		IMAGE_MOSAIC::Location(1659.557	* 	100,	41.3232221387	*	1.0e7,	114.719014262	*	1.0e7),
		IMAGE_MOSAIC::Location(1656.874	* 	100,	41.3232304099	*	1.0e7,	114.7184905 	*	1.0e7),
		IMAGE_MOSAIC::Location(1657.925	* 	100,	41.3232227082	*	1.0e7,	114.717979467	*	1.0e7),
		IMAGE_MOSAIC::Location(1659.013	* 	100,	41.3232208952	*	1.0e7,	114.717454322	*	1.0e7),
		IMAGE_MOSAIC::Location(1661.212	* 	100,	41.3232307214	*	1.0e7,	114.716942786	*	1.0e7),
		IMAGE_MOSAIC::Location(1659.172	* 	100,	41.3232125977	*	1.0e7,	114.716432361	*	1.0e7),
		IMAGE_MOSAIC::Location(1657.509	* 	100,	41.3232146144	*	1.0e7,	114.715924978	*	1.0e7)
};


// AB  航线上对应的27 张图片名称
const static char* image_name[27] = {
	"DSC00622.JPG",
	"DSC00623.JPG",
	"DSC00624.JPG",
	"DSC00625.JPG",
	"DSC00626.JPG",
	"DSC00627.JPG",
	"DSC00628.JPG",
	"DSC00629.JPG",
	"DSC00630.JPG",
	"DSC00631.JPG",
	"DSC00632.JPG",
	"DSC00633.JPG",
	"DSC00634.JPG",
	"DSC00635.JPG",
	"DSC00636.JPG",
	"DSC00637.JPG",
	"DSC00638.JPG",
	"DSC00639.JPG",
	"DSC00640.JPG",
	"DSC00641.JPG",
	"DSC00642.JPG",
	"DSC00643.JPG",
	"DSC00644.JPG",
	"DSC00645.JPG",
	"DSC00646.JPG",
	"DSC00647.JPG",
	"DSC00648.JPG"
};

//CD  航线上对应的29 张
const static char *image_name2[29] = {
	"DSC00649.JPG",
	"DSC00650.JPG",
	"DSC00651.JPG",
	"DSC00652.JPG",
	"DSC00653.JPG",
	"DSC00654.JPG",
	"DSC00655.JPG",
	"DSC00656.JPG",
	"DSC00657.JPG",
	"DSC00658.JPG",
	"DSC00659.JPG",
	"DSC00660.JPG",
	"DSC00661.JPG",
	"DSC00662.JPG",
	"DSC00663.JPG",
	"DSC00664.JPG",
	"DSC00665.JPG",
	"DSC00666.JPG",
	"DSC00667.JPG",
	"DSC00668.JPG",
	"DSC00669.JPG",
	"DSC00670.JPG",
	"DSC00671.JPG",
	"DSC00672.JPG",
	"DSC00673.JPG",
	"DSC00674.JPG",
	"DSC00675.JPG",
	"DSC00676.JPG",
	"DSC00677.JPG"

};

#if 0
int main(int argc, char **argv)
{
	IMAGE_MOSAIC::Image_algorithm*  image_algorithm = new IMAGE_MOSAIC::Image_algorithm();

	//由于没有AB 和CD 航线的方向数据，这里用所有照片偏航角的平均数据来做航线的方向数据
	float yaw_AB_avg = 0.0f, yaw_CD_avg = 0.0f;

	for(int i=0; i<sizeof(yaw_AB)/4; i++)
	{
		yaw_AB_avg += yaw_AB[i];
	}
	yaw_AB_avg /= sizeof(yaw_AB) / 4;

	for(int i=0; i<sizeof(yaw_CD)/4; i++)
	{
		yaw_CD_avg += yaw_CD[i];
	}
	yaw_CD_avg /= sizeof(yaw_CD) / 4;

	//AB:91.7618, CD:284.542
	cout << "yaw AB avg:" << yaw_AB_avg << ",yaw CD avg:" << yaw_CD_avg << endl;

	//为了拼接两条航线的图片，理论上两条航线应该平行
	//这里的两条航线不平行，也可能是固定翼飞行的时候
	//由于风力的影响导致

	yaw_AB_avg = (yaw_AB_avg + (yaw_CD_avg - 180)) / 2;
	yaw_CD_avg = yaw_AB_avg + 180;

	cout << "new yaw AB avg:" << yaw_AB_avg <<", new yaw CD avg:" << yaw_CD_avg << endl;

	
#if 1
	//把图片进行缩放
	//由于roll pitch  的影响，导致在拼接两条航线是，会出现错位现象
	//这里需要用roll  和 pitch 的值对图像进行拉伸
	for(int i=0; i<27; i++)
	{
		string strFile = "/home/wenyi/workspace/test_photo/";
		strFile += string(image_name[i]);

		Mat image = imread(strFile.c_str());
		Mat image_resize;

		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}

		image_algorithm->Image_resize(image, image_resize,  Size(image.cols/8, image.rows/8));

		//用roll  和 pitch 对image_resize  进行拉伸处理

		Mat tmp_image;
		image_algorithm->Image_perspective(image_resize, tmp_image, roll_AB[i], pitch_AB[i]);

		string strName = "./resize_image/";
		strName += string(image_name[i]);

		imwrite(strName.c_str(), tmp_image);
	}


	// CD 航线的图片需要旋转180 度， 才能跟AB 拼接
	for(int i=0; i<29; i++)
	{
		string strFile = "/home/wenyi/workspace/test_photo/";
		strFile += string(image_name2[i]);

		Mat image = imread(strFile.c_str());
		Mat image_resize;

		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}

		image_algorithm->Image_resize(image, image_resize,  Size(image.cols/8, image.rows/8));

		
		//用roll  和 pitch 对image_resize  进行拉伸处理
		
		Mat tmp_image1;
		image_algorithm->Image_perspective(image_resize, tmp_image1, roll_CD[i], pitch_CD[i]);

		Mat image_tmp;
		image_algorithm->Image_rotate(tmp_image1, image_tmp, 180);

		string strName = "./resize_image/";
		strName += string(image_name2[i]);

		imwrite(strName.c_str(), image_tmp);
	}

	cout << "image resize ok" << endl;
#endif



#if 1

	//旋转AB 航线上的所有图片
	
	for(int i=0; i<27; i++)
	{
		string strFile = "/home/wenyi/workspace/opencv_test/new_project/build/resize_image/";
		strFile += string(image_name[i]);

		Mat image = imread(strFile.c_str());
		Mat image_rotate;

		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}

		image_algorithm->Image_rotate(image, image_rotate,  yaw_AB_avg - yaw_AB[i]);

		string strName = "./rotate_image/";
		strName += string(image_name[i]);

		imwrite(strName.c_str(), image_rotate);
	}

	//旋转CD  航线上的所有图片
	//由于CD  航线上的图片已经旋转了180  度
	//计算角度的时候需要减去180 度
	for(int i=0; i<29; i++)
	{
		string strFile = "/home/wenyi/workspace/opencv_test/new_project/build/resize_image/";
		strFile += string(image_name2[i]);

		Mat image = imread(strFile.c_str());
		Mat image_rotate;

		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}

		image_algorithm->Image_rotate(image, image_rotate,  yaw_AB_avg - (yaw_CD[i] - 180));

		string strName = "./rotate_image/";
		strName += string(image_name2[i]);

		imwrite(strName.c_str(), image_rotate);
	}

	cout << "image rotate ok" << endl;
#endif


	

#if 1
	//为地图申请一张画布
	//图片现在是994 X 663, 画布大小设置为7000 x 5000
	Mat map_test(7000, 5000,CV_8UC3);
	map_test.setTo(0);

	Point2i last_image_vertex;
	int num_image = 0;

	//测试两张 图片的拼接坐标查找
	for(int i=0; i<26; i++)
	{
		
		string strFile1 = "/home/wenyi/workspace/opencv_test/new_project/build/rotate_image/";
		string strFile2 = "/home/wenyi/workspace/opencv_test/new_project/build/rotate_image/";
		strFile1 += string(image_name[i]);
		strFile2 += string(image_name[i+1]);

		Mat src_image1 = imread(strFile1.c_str());

		if(src_image1.empty())
		{
			cout << "failed to load:" << strFile1 << endl;
			return -1;
		}

		Mat src_image2 = imread(strFile2.c_str());

		if(src_image2.empty())
		{
			cout << "failed to load:" << strFile2 << endl;
			return -1;
		}


		
		Mat src_image1_tmp;
		if(i > 0)
		{
			src_image1_tmp = map_test(Range(last_image_vertex.y, last_image_vertex.y + src_image2.rows), Range(last_image_vertex.x, last_image_vertex.x + src_image2.cols));
			if(src_image1_tmp.empty())
			{
				cout << "failed to load:" << strFile2 << endl;
				return -1;
			}
		}
		else
		{
			src_image1_tmp = src_image1;
		}

		

		Point2i point_test;
		image_algorithm->Image_mosaic_algorithm(src_image1, src_image2, IMAGE_MOSAIC::Image_algorithm::UP,point_test);
	
		cout << "point_test x:" << point_test.x << ", y:" << point_test.y << endl;

#if 1
		//对两张图片进行拼接

		Mat dest_image;
		Point2i image1_vertex, image2_vertex;
		image_algorithm->Image_optimize_seam(src_image1_tmp, src_image2, dest_image, point_test,
										IMAGE_MOSAIC::Image_algorithm::UP, image1_vertex, image2_vertex);

		stringstream ss1;
		string s1;
		string strName1 = "./dest_image/";
		ss1 << num_image;
		ss1 >> s1;
		num_image++;
		strName1 += s1;
		strName1 += ".jpg";


		cout << "strName1" << strName1 << endl;
		
		imwrite(strName1.c_str(), dest_image);

#if 1
		//把拼接的图片贴到画布上
		//这里需要计算和维护图片的顶点坐标在画布上位置

		if(i == 0)
		{
			last_image_vertex.x = map_test.cols / 3 - src_image1.cols / 2;
			last_image_vertex.y = map_test.rows - 3 * src_image1.rows;
			//把第一张图片拷贝到地图上
			src_image1.copyTo(map_test(Rect(last_image_vertex.x , last_image_vertex.y, src_image1.cols, src_image1.rows)));
		}

		//通过传出的第一张图片在dest_image  中的位置，计算dest 在map_test 中的开始坐标
		Point2i dest_image_vertex;
		dest_image_vertex.x = last_image_vertex.x - image1_vertex.x;
		dest_image_vertex.y = last_image_vertex.y - image1_vertex.y;

		//裁减掉dest_image 图片下边的1/6
		Mat tmp_image;
		image_algorithm->Image_cut(dest_image, tmp_image, IMAGE_MOSAIC::Image_algorithm::DOWN, src_image1.rows/6);

		//拷贝 dest_image 到map_test 中
		tmp_image.copyTo(map_test(Rect(dest_image_vertex.x, dest_image_vertex.y, tmp_image.cols, tmp_image.rows)));

		//计算第二张图片在map_test 中的坐标
		last_image_vertex.x = dest_image_vertex.x + image2_vertex.x;
		last_image_vertex.y = dest_image_vertex.y + image2_vertex.y;
#endif


		
#endif

	}

	imwrite("map.jpg", map_test);
#endif

	//先对map_test 克隆一个，为了第二行的匹配
	Mat map_test2 = map_test.clone();
	

	cout << "--------------the first fly line is ok------------" << endl;
#if 0
	map_test.setTo(0);
#else
	//拼接第一条航线的最后一张和第二条航线的第一张图片
	//并并计算第二条航线第一张图片的位置
	//这里第二条航线在第一条航线的右边，所有是 左右拼接
	do{
		string strFile1 = "/home/wenyi/workspace/opencv_test/new_project/build/rotate_image/";
		strFile1 += string(image_name[26]);

		Mat src_image1 = imread(strFile1.c_str());

		if(src_image1.empty())
		{
			cout << "failed to load:" << strFile1 << endl;
			return -1;
		}


		string strFile2 = "/home/wenyi/workspace/opencv_test/new_project/build/rotate_image/";
		strFile2 += string(image_name2[0]);

		Mat src_image2 = imread(strFile2.c_str());

		if(src_image2.empty())
		{
			cout << "failed to load:" << strFile2 << endl;
			return -1;
		}

		
		Mat src_image1_tmp;
		
		src_image1_tmp = map_test(Range(last_image_vertex.y, last_image_vertex.y + src_image2.rows), Range(last_image_vertex.x, last_image_vertex.x + src_image2.cols));
		if(src_image1_tmp.empty())
		{
			cout << "failed to load:" << strFile2 << endl;
			return -1;
		}

		Point2i point_test;
		image_algorithm->Image_mosaic_algorithm(src_image1, src_image2, IMAGE_MOSAIC::Image_algorithm::RIGHT,point_test);
	
		cout << "point_test x:" << point_test.x << ", y:" << point_test.y << endl;


		//对两张图片进行拼接

		Mat dest_image;
		Point2i image1_vertex, image2_vertex;
		image_algorithm->Image_optimize_seam(src_image1_tmp, src_image2, dest_image, point_test,
										IMAGE_MOSAIC::Image_algorithm::RIGHT, image1_vertex, image2_vertex);

		stringstream ss1;
		string s1;
		string strName1 = "./dest_image/";
		ss1 << num_image;
		ss1 >> s1;
		num_image++;
		strName1 += s1;
		strName1 += ".jpg";


		cout << "strName1" << strName1 << endl;
		
		imwrite(strName1.c_str(), dest_image);


		//通过传出的第一张图片在dest_image  中的位置，计算dest 在map_test 中的开始坐标
		Point2i dest_image_vertex;
		dest_image_vertex.x = last_image_vertex.x - image1_vertex.x;
		dest_image_vertex.y = last_image_vertex.y - image1_vertex.y;

		
		//计算第二张图片在map_test 中的坐标
		last_image_vertex.x = dest_image_vertex.x + image2_vertex.x;
		last_image_vertex.y = dest_image_vertex.y + image2_vertex.y;

#if 1
		//裁剪掉dest image 图片下边的1/4, 左边的1/3
		Mat tmp_image;
		tmp_image = dest_image(Range(0, dest_image.rows - dest_image.rows/4), Range(dest_image.cols / 3, dest_image.cols));

		dest_image_vertex.x += dest_image.cols / 3;

		//拷贝 dest_image 到map_test 中
		tmp_image.copyTo(map_test(Rect(dest_image_vertex.x, dest_image_vertex.y, tmp_image.cols, tmp_image.rows)));
#endif
		
	}while(0);

#endif




#if 1
	//开始拼接第二条航线
	//第二条航线经过旋转后，在下方进行拼接
	for(int i=0; i<28; i++)
	{
		string strFile1 = "/home/wenyi/workspace/opencv_test/new_project/build/rotate_image/";
		string strFile2 = "/home/wenyi/workspace/opencv_test/new_project/build/rotate_image/";
		strFile1 += string(image_name2[i]);
		strFile2 += string(image_name2[i+1]);

		Mat src_image1 = imread(strFile1.c_str());

		if(src_image1.empty())
		{
			cout << "failed to load:" << strFile1 << endl;
			return -1;
		}

		Mat src_image2 = imread(strFile2.c_str());

		if(src_image2.empty())
		{
			cout << "failed to load:" << strFile2 << endl;
			return -1;
		}

		Mat src_image1_tmp;
		
		src_image1_tmp = map_test(Range(last_image_vertex.y, last_image_vertex.y + src_image2.rows), Range(last_image_vertex.x, last_image_vertex.x + src_image2.cols));
		if(src_image1_tmp.empty())
		{
			cout << "failed to load:" << strFile2 << endl;
			return -1;
		}
		

		//先进行上下匹配
		Point2i point_test;
		image_algorithm->Image_mosaic_algorithm(src_image1, src_image2, IMAGE_MOSAIC::Image_algorithm::DOWN,point_test);
	
		cout << "point_test x:" << point_test.x << ", y:" << point_test.y << endl;

#if 1
		//在进行左右匹配
		Point2i image1_vertex1, image2_vertex1;
		Point2i dest_image_vertex2;
		//快速获取改幅图片在dest 中的位置
		image_algorithm->Fast_calc_dest_point( src_image1, point_test, IMAGE_MOSAIC::Image_algorithm::DOWN,
													image1_vertex1, image2_vertex1);

		dest_image_vertex2.x = last_image_vertex.x - image1_vertex1.x + image2_vertex1.x;
		dest_image_vertex2.y = last_image_vertex.y - image1_vertex1.y + image2_vertex1.y;

		//获取该张图片

		Mat left_image = map_test2(Range(dest_image_vertex2.y, dest_image_vertex2.y + src_image2.rows),
									Range(dest_image_vertex2.x, dest_image_vertex2.x + src_image2.cols));

		Point2i point_test2;
		image_algorithm->Image_fast_mosaic_algorithm(left_image, src_image2, point_test2);

		
		cout << "+++++++++point_test2 x:" << point_test2.x << ", y:" << point_test2.y << "++++++++++++++" << endl;


		if(point_test2.y > -20 && point_test2.y < 20)
		{
			if(point_test2.x > -30  && point_test2.x < 30)
			{
				point_test.y = point_test.y + point_test2.y;
			}

			//if(point_test2.x > -20  && point_test2.x < 20)
			//{
			//	point_test.x = point_test.x - point_test2.x;
			//}
		}
#endif


		//对两张图片进行拼接

		Mat dest_image;
		Point2i image1_vertex, image2_vertex;
		image_algorithm->Image_optimize_seam(src_image1_tmp, src_image2, dest_image, point_test,
										IMAGE_MOSAIC::Image_algorithm::DOWN, image1_vertex, image2_vertex);

		stringstream ss1;
		string s1;
		string strName1 = "./dest_image/";
		ss1 << num_image;
		ss1 >> s1;
		num_image++;
		strName1 += s1;
		strName1 += ".jpg";


		cout << "strName1" << strName1 << endl;
		
		imwrite(strName1.c_str(), dest_image);

#if 1
		//拼接第二条航线的图片

		//通过传出的第一张图片在dest_image  中的位置，计算dest 在map_test 中的开始坐标
		Point2i dest_image_vertex;
		dest_image_vertex.x = last_image_vertex.x - image1_vertex.x;
		dest_image_vertex.y = last_image_vertex.y - image1_vertex.y;

		//计算第二张图片在map_test 中的坐标
		last_image_vertex.x = dest_image_vertex.x + image2_vertex.x;
		last_image_vertex.y = dest_image_vertex.y + image2_vertex.y;
#if 0
		//裁减掉dest_image 图片下边的1/6
		Mat tmp_image;
		image_algorithm->Image_cut(dest_image, tmp_image, IMAGE_MOSAIC::Image_algorithm::DOWN, src_image1.rows/6);
#else
		//裁剪掉dest image 图片下边的1/6, 左边的1/4
		Mat tmp_image;
		tmp_image = dest_image(Range(0, dest_image.rows - dest_image.rows / 6), Range(dest_image.cols / 4, dest_image.cols));

		dest_image_vertex.x += dest_image.cols / 4;


		
		//接口优化
		int w = tmp_image.cols / 4;
		//image1  为 map
		int image1_start_row = dest_image_vertex.y;
		int image1_start_col = dest_image_vertex.x;
		
		//image2 为tmp_image
		int image2_start_row  = 0;
		int image2_start_col = 0;
		
		//tmp_image 为dest
		int dest_start_row = 0;
		int dest_start_col = 0;

		float alpha = 1.0f;//map  中像素的权重

		for(int j=0; j<tmp_image.rows; j++)
		{
			for(int k=0; k<w; k++)
			{
				alpha = (float)(w - k) / (float)w;
				Scalar color1 = map_test.at<Vec3b>(image1_start_row + j, image1_start_col + k);
				Scalar color2 = tmp_image.at<Vec3b>(image2_start_row + j, image2_start_col + k);

				Scalar color3;
				color3(0) = color1(0) * alpha + color2(0) * (1 - alpha);
				color3(1) = color1(1) * alpha + color2(1) * (1 - alpha);
				color3(2) = color1(2) * alpha + color2(2) * (1 - alpha);

				tmp_image.at<Vec3b>(dest_start_row + j, dest_start_col + k) = Vec3b(color3(0), color3(1), color3(2));
			}
		}
		
#endif

		//拷贝 tmp_image 到map_test 中
		tmp_image.copyTo(map_test(Rect(dest_image_vertex.x, dest_image_vertex.y, tmp_image.cols, tmp_image.rows)));

#endif

		
	}

	

#endif

	imwrite("map1.jpg", map_test);
	


	cout << "-----------ok-------------" << endl;

	waitKey();

	return 0;
}


#endif


#if 0
//left 方向的拼接测试
int main(int argc, char **argv)
{

	IMAGE_MOSAIC::Image_algorithm*  image_algorithm = new IMAGE_MOSAIC::Image_algorithm();

	string strFile1 = "/home/wenyi/workspace/opencv_test/new_project/build/rotate_image/";
	strFile1 += string(image_name[26]);

	Mat src_image1 = imread(strFile1.c_str());

	if(src_image1.empty())
	{
		cout << "failed to load:" << strFile1 << endl;
		return -1;
	}


	string strFile2 = "/home/wenyi/workspace/opencv_test/new_project/build/rotate_image/";
	strFile2 += string(image_name2[0]);

	Mat src_image2 = imread(strFile2.c_str());

	if(src_image2.empty())
	{
		cout << "failed to load:" << strFile2 << endl;
		return -1;
	}



	Point2i point_test;
	image_algorithm->Image_mosaic_algorithm(src_image2, src_image1, IMAGE_MOSAIC::Image_algorithm::LEFT,point_test);
	
	cout << "point_test x:" << point_test.x << ", y:" << point_test.y << endl;

	//对两张图片进行拼接

	Mat dest_image;
	Point2i image1_vertex, image2_vertex;
	image_algorithm->Image_optimize_seam(src_image2, src_image1, dest_image, point_test,
										IMAGE_MOSAIC::Image_algorithm::LEFT, image2_vertex, image1_vertex);

	stringstream ss1;
	string s1;
	string strName1 = "./dest_image/";
	ss1 << 0;
	ss1 >> s1;
	strName1 += s1;
	strName1 += ".jpg";


	cout << "strName1" << strName1 << endl;
		
	imwrite(strName1.c_str(), dest_image);
	
	waitKey();

	return 0;
}

#endif



#if 1
// gps 坐标位置拼图测试
int main(int argc, char **argv)
{
	IMAGE_MOSAIC::Image_algorithm*  image_algorithm = new IMAGE_MOSAIC::Image_algorithm();

#if 1
	
	//把图片进行缩放
	//由于roll pitch  的影响，导致在拼接两条航线是，会出现错位现象
	//这里需要用roll  和 pitch 的值对图像进行拉伸
	for(int i=0; i<27; i++)
	{
		string strFile = "/home/wenyi/workspace/test_photo/";
		strFile += string(image_name[i]);

		Mat image = imread(strFile.c_str());
		Mat image_resize;

		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}

		image_algorithm->Image_resize(image, image_resize,  Size(image.cols/8, image.rows/8));

		//用roll  和 pitch 对image_resize  进行拉伸处理

		Mat tmp_image;
		image_algorithm->Image_perspective(image_resize, tmp_image, roll_AB[i], pitch_AB[i]);

		string strName = "./resize_image/";
		strName += string(image_name[i]);

		imwrite(strName.c_str(), tmp_image);
	}

	
	//旋转AB 航线上的所有图片
	//为了测试的方便，这里旋转的目标角度为90
		
	for(int i=0; i<27; i++)
	{
		string strFile = "/home/wenyi/workspace/opencv_test/new_project/build/resize_image/";
		strFile += string(image_name[i]);
	
		Mat image = imread(strFile.c_str());
		Mat image_rotate;
	
		if(image.empty())
		{
			cout << "failed to load:" << strFile << endl;
			return -1;
		}
	
		image_algorithm->Image_rotate(image, image_rotate,	90 - yaw_AB[i]);
	
		string strName = "./rotate_image/";
		strName += string(image_name[i]);
	
		imwrite(strName.c_str(), image_rotate);
	}

#endif

	cout << "deal with image ok" << endl;

	float scale;
#if 0
	for(int i=0; i<27; i++)
	{
		cout << "gps alt:" << gps_location_AB[i].alt << ", lat:" << gps_location_AB[i].lat << ", lng:" << gps_location_AB[i].lng << endl;
	}
#endif
	// 1、查找两个图片拼接位置

	//do{
		string strFile1 = "/home/wenyi/workspace/opencv_test/new_project/build/rotate_image/";
		string strFile2 = "/home/wenyi/workspace/opencv_test/new_project/build/rotate_image/";
		strFile1 += string(image_name[0]);
		strFile2 += string(image_name[1]);

		Mat src_image1 = imread(strFile1.c_str());

		if(src_image1.empty())
		{
			cout << "failed to load:" << strFile1 << endl;
			return -1;
		}

		Mat src_image2 = imread(strFile2.c_str());

		if(src_image2.empty())
		{
			cout << "failed to load:" << strFile2 << endl;
			return -1;
		}

		
		Point2i point_test;
		image_algorithm->Image_mosaic_algorithm(src_image1, src_image2, IMAGE_MOSAIC::Image_algorithm::UP,point_test);
			
		cout << "point_test x:" << point_test.x << ", y:" << point_test.y << endl;


		//通过拼接位置求出两张求出两个图像中心像素点的距离
		float image_center_distance = sqrt(pow(point_test.x, 2)  + pow(point_test.y, 2));

		cout << " image center distance:" << image_center_distance << endl;

		//计算第一张和第二张图片的距离和像素的比例尺
#if 0
		float gps_center_distance = image_algorithm->Get_distance(gps_location_AB[0], gps_location_AB[1]);
#else
		struct IMAGE_MOSAIC::Location image_location1(gps_location_AB[0].alt, gps_location_AB[0].lat, gps_location_AB[0].lng);
		struct IMAGE_MOSAIC::Imu_data imu_data1(pitch_AB[0], roll_AB[0], 90);
		image_algorithm->Location_update_baseon_pitch_roll(image_location1, gps_base, imu_data1);

		struct IMAGE_MOSAIC::Location image_location2(gps_location_AB[1].alt, gps_location_AB[1].lat, gps_location_AB[1].lng);
		struct IMAGE_MOSAIC::Imu_data imu_data2(pitch_AB[1], roll_AB[1], 90);
		image_algorithm->Location_update_baseon_pitch_roll(image_location2, gps_base, imu_data2);

		float gps_center_distance = image_algorithm->Get_distance(image_location1, image_location2);
#endif

		cout << "gps center distance:" << gps_center_distance << endl;

		
		scale = gps_center_distance / image_center_distance;
		
	//}while(0);

	cout << "scale:" << scale << endl;



	//为地图申请一张画布， y 轴的正 方向为90 度
	//根据航线的长度，基本可以确定画布的大小范围
	//图片现在是994 X 663, 画布大小设置为7000 x 5000
	Mat map_test(7000, 5000,CV_8UC3);
	map_test.setTo(0);

	//把第一张图片贴到画布的底部

	Point2i dest_point; 
	dest_point.x = map_test.cols / 3 - src_image1.cols / 2;
	dest_point.y = map_test.rows - 3 * src_image1.rows;
	
	//把第一张图片拷贝到地图上
	src_image1.copyTo(map_test(Rect(dest_point.x , dest_point.y, src_image1.cols, src_image1.rows)));

	//根据第一张图片中心的gps 位置，计算(0, 0) 坐标对应的gps 位置
	struct IMAGE_MOSAIC::Location map_origin;

	float diff_x = (float)dest_point.x + (float)src_image1.cols / 2.0f;
	float diff_y = (float)dest_point.y + (float)src_image1.rows / 2.0f;

	float origin_first_image_distance = sqrt(pow(diff_x, 2) + pow(diff_y, 2)) * scale;

	float angle = atan2(diff_x, diff_y) * (180.0f / M_PI);

	cout << "angle:" << angle << ", origin distance:" << origin_first_image_distance << endl;

	//图片 (0, 0) 坐标在第一张图片的中心的方位需要在90 度
	float bearing0 = angle + 90;

	cout << "bearing:" << bearing0 << endl;

	map_origin.alt = image_location1.alt;
	map_origin.lat = image_location1.lat;
	map_origin.lng = image_location1.lng;

	image_algorithm->Location_update(map_origin, bearing0, origin_first_image_distance);

	//根据第二张图片的gps 坐标，计算第二张图片的在地图中的位置
	for(int i=1; i<27; i++)
	{
		string strFile = "/home/wenyi/workspace/opencv_test/new_project/build/rotate_image/";
		
		strFile += string(image_name[i]);
		

		Mat src_image = imread(strFile.c_str());

		if(src_image1.empty())
		{
			cout << "failed to load:" << strFile1 << endl;
			return -1;
		}
#if 0
		float distance = image_algorithm->Get_distance(map_origin, gps_location_AB[i]) / scale;
		float bearing = image_algorithm->Get_bearing_cd(map_origin, gps_location_AB[i]);
#else
		
		struct IMAGE_MOSAIC::Location image_location(gps_location_AB[i].alt, gps_location_AB[i].lat, gps_location_AB[i].lng);
		struct IMAGE_MOSAIC::Imu_data imu_data(pitch_AB[i], roll_AB[i], 90);
		image_algorithm->Location_update_baseon_pitch_roll(image_location, gps_base, imu_data);
		float distance = image_algorithm->Get_distance(map_origin, image_location) / scale;
		float bearing = image_algorithm->Get_bearing_cd(map_origin, image_location);

#endif

		cout << "bearing:" << bearing << ", distance:" << distance << endl;

		// 求第二张图片的原点坐标
		Point2i image_point;
		image_point.x = (int)(distance * sin((bearing - 270) * (M_PI / 180.0f)) - (float)src_image.cols / 2);
		image_point.y = (int)(distance * cos((bearing - 270) * (M_PI / 180.0f)) - (float)src_image.rows / 2);

		//把第二张图片拼接到地图上
#if 1
		//src_image.copyTo(map_test(Rect(image_point.x, image_point.y, src_image.cols, src_image.rows)));
		Mat dest_image;
		image_algorithm->Image_cut(src_image, dest_image, IMAGE_MOSAIC::Image_algorithm::DOWN, src_image.rows / 6);
		dest_image.copyTo(map_test(Rect(image_point.x, image_point.y, dest_image.cols, dest_image.rows)));
#else
		//去掉下边的1/6, 加权融合
		int w = 100;
		Mat dest_image;
		image_algorithm->Image_cut(src_image, dest_image, IMAGE_MOSAIC::Image_algorithm::DOWN, src_image.rows / 6 + w);
		int src_start_row = dest_image.rows;
		int map_start_row = image_point.y + src_start_row;
		int map_start_col = image_point.x;
		float alpha = 1.0f;//src_image  中像素的权重
		for(int j=0; j<w; j++)
		{
			alpha = (float)(w-j) / (float)w;
			for(int k=0; k<src_image.cols; k++)
			{
				Scalar color1 = map_test.at<Vec3b>(map_start_row + j, map_start_col + k);
				Scalar color2 = src_image.at<Vec3b>(src_start_row + j, k);

				Scalar color3;
				color3(0) = color1(0) * (1 - alpha) + color2(0) * alpha;
				color3(1) = color1(1) * (1 - alpha) + color2(1) * alpha;
				color3(2) = color1(2) * (1 - alpha) + color2(2) * alpha;

				map_test.at<Vec3b>(map_start_row + j, map_start_col + k) = Vec3b(color3(0), color3(1), color3(2));
			}
		}

		dest_image.copyTo(map_test(Rect(image_point.x, image_point.y, dest_image.cols, dest_image.rows)));
#endif
	
	}
	
	imwrite("map.jpg", map_test);

	cout << "--------------I am ok------------" << endl;

	waitKey();
	return 0;
}

#endif




