#include "cstdlib" // #include "stdlib.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "ros/ros.h"
#include "cmath"  // #include "math.h"
#include "opencv2/highgui.hpp"

using namespace cv;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "expr3_node");
	ros::NodeHandle n;
	
	Mat Src = imread("/home/handleandwheel/lena512color.tiff");
	Mat Grey8U = Src.clone();
	cvtColor(Src, Grey8U, COLOR_BGR2GRAY);
	Mat Grey16S(Src.rows, Src.cols, CV_16SC1);
	Grey8U.convertTo(Grey16S, CV_16SC1);
	imshow("Src", Src);
	imshow("Grey", Grey8U);
	
	// sobel
	Mat SobelX(3, 3, CV_16SC1, Scalar(0));
	Mat SobelY(3, 3, CV_16SC1, Scalar(0));
	SobelX.at<int16_t>(0, 0) = -1;
	SobelX.at<int16_t>(0, 1) = -2;
	SobelX.at<int16_t>(0, 2) = -1;
	SobelX.at<int16_t>(1, 0) = 0;
	SobelX.at<int16_t>(1, 1) = 0;
	SobelX.at<int16_t>(1, 2) = 0;
	SobelX.at<int16_t>(2, 0) = 1;
	SobelX.at<int16_t>(2, 1) = 2;
	SobelX.at<int16_t>(2, 2) = 1;
	SobelY.at<int16_t>(0, 0) = -1;
	SobelY.at<int16_t>(0, 1) = 0;
	SobelY.at<int16_t>(0, 2) = 1;
	SobelY.at<int16_t>(1, 0) = -2;
	SobelY.at<int16_t>(1, 1) = 0;
	SobelY.at<int16_t>(1, 2) = 2;
	SobelY.at<int16_t>(2, 0) = -1;
	SobelY.at<int16_t>(2, 1) = 0;
	SobelY.at<int16_t>(2, 2) = 1;
	Mat DstSobelX(Src.rows, Src.cols, CV_16SC1);
	Mat DstSobelY(Src.rows, Src.cols, CV_16SC1);
	filter2D(Grey16S, DstSobelX, CV_16SC1, SobelX, Point(1, 1));
	filter2D(Grey16S, DstSobelY, CV_16SC1, SobelY, Point(1, 1));
	Mat DstSobel16S(Src.rows, Src.cols, CV_16SC1);
	addWeighted(DstSobelX, 0.5, DstSobelY, 0.5, 0, DstSobel16S);
	Mat DstSobel8U(Src.rows, Src.cols, CV_8UC1);
	convertScaleAbs(DstSobel16S, DstSobel16S);
	DstSobel16S.convertTo(DstSobel8U, CV_8UC1);
	imshow("Sobel", DstSobel8U);
	
	// roberts
	Mat RobertsX(2, 2, CV_16SC1, Scalar(0));
	Mat RobertsY(2, 2, CV_16SC1, Scalar(0));
	RobertsX.at<int16_t>(0, 0) = -1;
	RobertsX.at<int16_t>(0, 1) = 0;
	RobertsX.at<int16_t>(1, 0) = 0;
	RobertsX.at<int16_t>(1, 1) = 1;
	RobertsY.at<int16_t>(0, 0) = 0;
	RobertsY.at<int16_t>(0, 1) = -1;
	RobertsY.at<int16_t>(1, 0) = 1;
	RobertsY.at<int16_t>(1, 1) = 0;
	Mat DstRobertsX(Src.rows, Src.cols, CV_16SC1);
	Mat DstRobertsY(Src.rows, Src.cols, CV_16SC1);
	filter2D(Grey16S, DstRobertsX, CV_16SC1, RobertsX, Point(0, 0));
	filter2D(Grey16S, DstRobertsY, CV_16SC1, RobertsY, Point(0, 0));
	Mat DstRoberts16S(Src.rows, Src.cols, CV_16SC1);
	addWeighted(DstRobertsX, 0.5, DstRobertsY, 0.5, 0, DstRoberts16S);
	Mat DstRoberts8U(Src.rows, Src.cols, CV_8UC1);
	convertScaleAbs(DstRoberts16S, DstRoberts16S);
	DstRoberts16S.convertTo(DstRoberts8U, CV_8UC1);
	imshow("Roberts", DstRoberts8U);
	
	// Prewitt
	Mat PrewittX(3, 3, CV_16SC1, Scalar(0));
	Mat PrewittY(3, 3, CV_16SC1, Scalar(0));
	PrewittX.at<int16_t>(0, 0) = -1;
	PrewittX.at<int16_t>(0, 1) = -1;
	PrewittX.at<int16_t>(0, 2) = -1;
	PrewittX.at<int16_t>(1, 0) = 0;
	PrewittX.at<int16_t>(1, 1) = 0;
	PrewittX.at<int16_t>(1, 2) = 0;
	PrewittX.at<int16_t>(2, 0) = 1;
	PrewittX.at<int16_t>(2, 1) = 1;
	PrewittX.at<int16_t>(2, 2) = 1;
	PrewittY.at<int16_t>(0, 0) = -1;
	PrewittY.at<int16_t>(0, 1) = 0;
	PrewittY.at<int16_t>(0, 2) = 1;
	PrewittY.at<int16_t>(1, 0) = -1;
	PrewittY.at<int16_t>(1, 1) = 0;
	PrewittY.at<int16_t>(1, 2) = 1;
	PrewittY.at<int16_t>(2, 0) = -1;
	PrewittY.at<int16_t>(2, 1) = 0;
	PrewittY.at<int16_t>(2, 2) = 1;
	Mat DstPrewittX(Src.rows, Src.cols, CV_16SC1);
	Mat DstPrewittY(Src.rows, Src.cols, CV_16SC1);
	filter2D(Grey16S, DstPrewittX, CV_16SC1, PrewittX, Point(1, 1));
	filter2D(Grey16S, DstPrewittY, CV_16SC1, PrewittY, Point(1, 1));
	Mat DstPrewitt16S(Src.rows, Src.cols, CV_16SC1);
	addWeighted(DstPrewittX, 0.5, DstPrewittY, 0.5, 0, DstPrewitt16S);
	Mat DstPrewitt8U(Src.rows, Src.cols, CV_8UC1);
	convertScaleAbs(DstPrewitt16S, DstPrewitt16S);
	DstPrewitt16S.convertTo(DstPrewitt8U, CV_8UC1);
	imshow("Prewitt", DstPrewitt8U);
	
	// LoG
	Mat LoG(5, 5, CV_16SC1, Scalar(0));
	LoG.at<int16_t>(0, 0) = 0;
	LoG.at<int16_t>(0, 1) = 0;
	LoG.at<int16_t>(0, 2) = -1;
	LoG.at<int16_t>(0, 3) = 0;
	LoG.at<int16_t>(0, 4) = 0;
	LoG.at<int16_t>(1, 0) = 0;
	LoG.at<int16_t>(1, 1) = -1;
	LoG.at<int16_t>(1, 2) = -2;
	LoG.at<int16_t>(1, 3) = -1;
	LoG.at<int16_t>(1, 4) = 0;
	LoG.at<int16_t>(2, 0) = -1;
	LoG.at<int16_t>(2, 1) = -2;
	LoG.at<int16_t>(2, 2) = 16;
	LoG.at<int16_t>(2, 3) = -2;
	LoG.at<int16_t>(2, 4) = -1;
	LoG.at<int16_t>(3, 0) = 0;
	LoG.at<int16_t>(3, 1) = -1;
	LoG.at<int16_t>(3, 2) = -2;
	LoG.at<int16_t>(3, 3) = -1;
	LoG.at<int16_t>(3, 4) = 0;
	LoG.at<int16_t>(4, 0) = 0;
	LoG.at<int16_t>(4, 1) = 0;
	LoG.at<int16_t>(4, 2) = -1;
	LoG.at<int16_t>(4, 3) = 0;
	LoG.at<int16_t>(4, 4) = 0;
	Mat DstLoG16S(Src.rows, Src.cols, CV_16SC1);
	filter2D(Grey16S, DstLoG16S, CV_16SC1, LoG, Point(2, 2));
	Mat DstLoG8U(Src.rows, Src.cols, CV_8UC1);
	convertScaleAbs(DstLoG16S, DstLoG16S);
	DstLoG16S.convertTo(DstLoG8U, CV_8UC1);
	imshow("LoG", DstLoG8U);
	
	// canny
	Mat DstCanny(Src.rows, Src.cols, CV_8UC1);
	Canny(Grey8U, DstCanny, 200, 50);
	imshow("Canny", DstCanny);
	
	waitKey(0);
}