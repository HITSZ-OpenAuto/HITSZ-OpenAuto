//
// Created by handleandwheel on 2021/10/17.
//

#ifndef WEBOTS_WS_EXPR2_HPP
#define WEBOTS_WS_EXPR2_HPP

#include "cstdlib" // #include "stdlib.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "ros/ros.h"
#include "cmath"  // #include "math.h"
#include "opencv2/highgui.hpp"

using namespace std;
using namespace cv;

#define USE_COMPUTER_CAMERA 1

class Expr2Node
{
public:
	Expr2Node(ros::NodeHandle &);
private:
	void Gaussian(const Mat &, Mat &, double);
	void Dilate(const Mat &, Mat &, const Mat &, const Vec2i &);
	void Erode(const Mat &, Mat &, const Mat &, const Vec2i &);
	VideoCapture capture;
};

Expr2Node::Expr2Node(ros::NodeHandle &nh)
{
	capture.open(0);
	
	if(!capture.isOpened()) ROS_ERROR("Failed to open camera. Replug the camera.");
	else
	{
		Mat frame;
		
		while(ros::ok())
		{
			capture.read(frame);
			
			if(frame.empty()) break;
#if USE_COMPUTER_CAMERA
			Mat frIn = frame.clone();
#else
			Mat frIn = frame()cv::Rect(0, 0, frame.cols/2, frame.rows);
#endif
			// gaussian
			Mat frOutG(frame.rows, frame.cols, CV_8UC3, Scalar(0));
			Gaussian(frame, frOutG, 5);
			// dilate
			Mat frOutSrc(frame.rows, frame.cols, CV_8UC1, Scalar(0));
			Mat frOutThd(frame.rows, frame.cols, CV_8UC1, Scalar(0));
			Mat frOutDDst(frame.rows, frame.cols, CV_8UC1, Scalar(0));
			Mat tem(9, 9, CV_8UC1, Scalar(0));
			cvtColor(frame, frOutSrc, COLOR_BGR2GRAY);
			threshold(frOutSrc, frOutThd, 127, 255, THRESH_BINARY);
			for(int i = 0; i < 9; i++)
			{
				for(int j = 0; j < 9; j++)
				{
					tem.at<uchar>(i, j) = 0;
				}
			}
			Vec2i ori(3, 3);
			Dilate(frOutThd, frOutDDst, tem, ori);
			// erode
			Mat frOutEDst(frame.rows, frame.cols, CV_8UC1, Scalar(0));
			cvtColor(frame, frOutSrc, COLOR_BGR2GRAY);
			Erode(frOutThd, frOutEDst, tem, ori);
			//Mat frOutEDst = frOutETmp(Rect(0, 0, (int)(frOutETmp.cols/2), frOutETmp.rows));
			//resize(frOutEDst, frOutEDst, frOutETmp.size(), 0, 0);
			// show imgs
			imshow("gaussian", frOutG);
			imshow("origin", frame);
			imshow("grey_src", frOutSrc);
			imshow("grey_thd", frOutThd);
			imshow("dilate_dst", frOutDDst);
			imshow("erode_dst", frOutEDst);
			waitKey(10);
		}
	}
}

void Expr2Node::Gaussian(const Mat &input, Mat &output, double sigma)
{
	if(output.rows != input.rows || output.cols != input.cols || output.channels() != input.channels()) return;
	// generate gaussian kernel
	int kernel_size = 9;
	double gaussian_kernel[kernel_size][kernel_size];
	double kernel_sum = 0;
	for(int i = 0; i < kernel_size; i++)
	{
		for(int j = 0; j < kernel_size; j++)
		{
			gaussian_kernel[i][j] = \
			exp(-(pow(((kernel_size-1)/2-i), 2) + pow(((kernel_size-1)/2-j), 2))/(2*pow(sigma, 2)));
			kernel_sum += gaussian_kernel[i][j];
		}
	}
	for(int i = 0; i < kernel_size; i++)
	{
		for(int j = 0; j < kernel_size; j++)
		{
			gaussian_kernel[i][j] /= kernel_sum;
		}
	}
	// relation(convolution)
	int chnl = input.channels();
	int rw = input.rows;
	int cl = input.cols;
	for(int c = 0; c < chnl; c++)
	{
		for(int i = 0; i < rw; i++)
		{
			for(int j = 0; j < cl; j++)
			{
				double tmp_val = 0.0;
				for(int m = -(kernel_size-1)/2; m <= (kernel_size-1)/2; m++)
				{
					for(int n = -(kernel_size-1)/2; n <= (kernel_size-1)/2; n++)
					{
						if(i+m < 0 || i+m >= rw || j+n < 0 || j+n >= cl)
							tmp_val += 255 * gaussian_kernel[m+(kernel_size-1)/2][n+(kernel_size-1)/2];
						else
							tmp_val += input.at<Vec3b>(i+m,j+n)[c] \
							        * gaussian_kernel[m+(kernel_size-1)/2][n+(kernel_size-1)/2];
					}
				}
				output.at<Vec3b>(i, j)[c] = (int)tmp_val;
			}
		}
	}
}

void Expr2Node::Dilate(const Mat &src, Mat &dst, const Mat &tem, const Vec2i &ori)
{
	// check
	if(src.channels() != 1 || dst.channels() != 1) return;
	if(src.rows != dst.rows || src.cols != dst.cols) return;
	if(ori[0] > tem.rows-1 || ori[1] > tem.cols-1) return;
	if(tem.rows%2 == 0 || tem.cols%2 == 0) return;
	// rotate structuring element
	Mat tem_use(tem.rows, tem.cols, CV_8UC1, Scalar(0));
	Vec2i ori_use(tem_use.rows-ori[0]-1, tem_use.cols-ori[1]-1);
	for(int i = 0; i < tem_use.rows; i++)
	{
		for(int j = 0; j < tem_use.cols; j++)
		{
			tem_use.at<uchar>(i, j) = tem.at<uchar>(tem_use.rows-i-1, tem_use.cols-j-1);
		}
	}
	int r_min = -ori_use[0];
	int r_max = tem_use.rows - ori_use[0]-1;
	int c_min = -ori_use[1];
	int c_max = tem_use.cols - ori_use[1]-1;
	// dilate
	int rw = src.rows;
	int cl = src.cols;
	for(int i = 0; i < rw; i++)
	{
		for(int j = 0; j < cl; j++)
		{
			bool hit = false;
			for(int m = r_min; !hit && m < r_max; m++)
			{
				for(int n = c_min; !hit && n < c_max; n++)
				{
					if(i+m<0 || i+m>=src.rows || j+n<0 || j+n>=src.cols){;}
					else if(
						tem_use.at<uchar>(ori_use[0]+m, ori_use[1]+n) == src.at<uchar>(i+m, j+n) &&
						tem_use.at<uchar>(ori_use[0]+m, ori_use[1]+n) == 0)
					{
						hit = true;
						break;
					}
				}
			}
			if(hit) dst.at<uchar>(i, j) = 0;
			else dst.at<uchar>(i, j) = 255;
		}
	}
}

void Expr2Node::Erode(const Mat &src, Mat &dst, const Mat &tem, const Vec2i &ori)
{
	// check
	if(src.channels() != 1 || dst.channels() != 1) return;
	if(src.rows != dst.rows || src.cols != dst.cols) return;
	if(ori[0] > tem.rows-1 || ori[1] > tem.cols-1) return;
	if(tem.rows%2 == 0 || tem.cols%2 == 0) return;
	// erode
	int r_min = -ori[0];
	int r_max = tem.rows - ori[0]-1;
	int c_min = -ori[1];
	int c_max = tem.cols - ori[1]-1;
	int rw = src.rows;
	int cl = src.cols;
	for(int i = 0; i < rw; i++)
	{
		for(int j = 0; j < cl; j++)
		{
			bool fit = true;
			for(int m = r_min; fit && m < r_max; m++)
			{
				for(int n = c_min; fit && n < c_max; n++)
				{
					if(i+m<0 || i+m>=src.rows || j+n<0 || j+n>=src.cols)
					{
						fit = false;
						break;
					}
					else if(
							tem.at<uchar>(ori[0]+m, ori[1]+n) != src.at<uchar>(i+m, j+n) &&
							tem.at<uchar>(ori[0]+m, ori[1]+n) == 0)
					{
						fit = false;
						break;
					}
				}
			}
			if(fit) dst.at<uchar>(i, j) = 0;
			else dst.at<uchar>(i, j) = 255;
		}
	}
	//dst = dst(Rect(0, 0, (int)(dst.cols/2), dst.rows));
	//resize(dst, dst, src.size(), 0, 0);
}

#endif //WEBOTS_WS_EXPR2_HPP