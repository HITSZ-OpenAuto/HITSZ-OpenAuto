//
// Created by handleandwheel on 2021/10/24.
//

#ifndef WEBOTS_WS_EXPR3_HPP
#define WEBOTS_WS_EXPR3_HPP

#include "cstdlib" // #include "stdlib.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "ros/ros.h"
#include "cmath"  // #include "math.h"
#include "opencv2/highgui.hpp"

using namespace std;
using namespace cv;

#define CANNY_RIGHT 1
#define CANNY_UP_RIGHT 2
#define CANNY_UP 3
#define CANNY_UP_LEFT 4
#define CANNY_LEFT 5
#define CANNY_DOWN_LEFT 6
#define CANNY_DOWN 7
#define CANNY_DOWN_RIGHT 8

#define USE_COMPUTER_CAMERA 1

class Expr3Node
{
public:
	Expr3Node(ros::NodeHandle &);
private:
	void EdgeDetector(const Mat &input, Mat &output, double sigma, double high_th, double low_th);
	void HoughLine(const Mat &input, Mat &output, int thresh);
	void Gaussian(const Mat &input, Mat &output, double sigma);
	void Gradient(const Mat &input, Mat &gradient, Mat &phase);
	void Suppression(const Mat &gradient, const Mat &phase, Mat &grad_sup);
	void DoubleThresh(const Mat &grad_sup, Mat &grad_h, Mat &grad_l, double high_th, double low_th);
	void EdgeConncetion(Mat &grad_h, Mat &grad_l);
	void DrawLine(Mat &input, double theta, double r);
	void SobelDirection(const Mat &input, Mat &ori);
	void DrawCircle(Mat &input, int orirw, int oricl, int radius);
	void HoughCircle(const Mat &input, Mat &output, double max_r, double min_r);
	VideoCapture capture;
};

Expr3Node::Expr3Node(ros::NodeHandle &nh)
{
	/*capture.open(!USE_COMPUTER_CAMERA);
	
	if(!capture.isOpened()) ROS_ERROR("Failed to open camera. Replug the camera.");
	else
	{
		Mat frame;*/
	Mat frame = imread("/home/handleandwheel/lena512color.tiff");
		while (ros::ok())
		{
			//capture.read(frame);
			
			if (frame.empty()) break;
#if USE_COMPUTER_CAMERA
			Mat frIn = frame.clone();
#else
			Mat frIn = frame()cv::Rect(0, 0, frame.cols/2, frame.rows);
#endif
			// src frame
			imshow("origin", frame);
			// canny edge detection
			Mat edge(frame.rows, frame.cols, CV_8UC1, Scalar(0));
			Mat grey(frame.rows, frame.cols, CV_8UC1, Scalar(0));
			cvtColor(frame, grey, COLOR_BGR2GRAY);
			EdgeDetector(grey, edge, 0.8, 20, 10);
			imshow("canny edge", edge);
			// hough line
			Mat line = frame.clone();
			HoughLine(frame, line, 115);
			imshow("line", line);
			//hough circle
			Mat circle = frame.clone();
			HoughCircle(frame, circle, 200, 0);
			imshow("circle", circle);
			waitKey(0);
		}
	
}

void Expr3Node::Gaussian(const Mat &input, Mat &output, double sigma)
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
						if(chnl == 3)
							tmp_val += input.at<Vec3b>(i+m,j+n)[c] \
										* gaussian_kernel[m+(kernel_size-1)/2][n+(kernel_size-1)/2];
						else
							tmp_val += input.at<uchar>(i+m,j+n) \
										* gaussian_kernel[m+(kernel_size-1)/2][n+(kernel_size-1)/2];
					}
				}
				if(chnl == 3)
					output.at<Vec3b>(i, j)[c] = (int)tmp_val;
				else
					output.at<uchar>(i, j) = (int)tmp_val;
			}
		}
	}
}

void Expr3Node::Gradient(const Mat &input, Mat &gradient, Mat &phase)
{
	int rw = input.rows;
	int cl = input.cols;
	for(int i = 1; i < rw-1; i++)
	{
		for(int j = 1; j < cl-1; j++)
		{
			double gx, gy;
			gx = 0.5 * (-input.at<uchar>(i, j) - input.at<uchar>(i + 1, j) + \
                        input.at<uchar>(i, j + 1) + input.at<uchar>(i + 1, j + 1));
			gy = 0.5 * (input.at<uchar>(i, j) - input.at<uchar>(i + 1, j) + \
                        input.at<uchar>(i, j + 1) - input.at<uchar>(i + 1, j + 1));
			gradient.at<float>(i,j) = sqrt(gx*gx + gy*gy);
			double gradient_phase = atan2(gy, gx);
			if(-2.3561 <= gradient_phase < -1.9635) phase.at<uchar>(i,j) = CANNY_DOWN_LEFT;
			else if(-1.9635 <= gradient_phase < -1.1779) phase.at<uchar>(i,j) = CANNY_DOWN;
			else if(-1.1779 <= gradient_phase < -0.3925) phase.at<uchar>(i,j) = CANNY_DOWN_RIGHT;
			else if(-0.3925 <= gradient_phase < 0.3925) phase.at<uchar>(i,j) = CANNY_RIGHT;
			else if(0.3925 <= gradient_phase < 1.1779) phase.at<uchar>(i,j) = CANNY_UP_RIGHT;
			else if(1.1779 <= gradient_phase < 1.9635) phase.at<uchar>(i,j) = CANNY_UP;
			else if(1.9635 <= gradient_phase < 2.3561) phase.at<uchar>(i,j) = CANNY_UP_LEFT;
			else phase.at<uchar>(i,j) = CANNY_LEFT;
			//phase.at<uchar>(i,j) = (int)atan2(gy, gx);
		}
	}
}

void Expr3Node::Suppression(const Mat &gradient, const Mat &phase, Mat &grad_sup)
{
	int rw = gradient.rows;
	int cl = gradient.cols;
	for(int i = 1; i < rw-1; i++)
	{
		for(int j = 1; j < cl-1; j++)
		{
			switch(phase.at<uchar>(i,j))
			{
				case CANNY_RIGHT:
					if (gradient.at<float>(i, j + 1) <= gradient.at<float>(i, j))
						grad_sup.at<float>(i, j) = gradient.at<float>(i, j);
					else
						grad_sup.at<float>(i, j) = 0;
					break;
				case CANNY_UP_RIGHT:
					if (gradient.at<float>(i - 1, j + 1) <= gradient.at<float>(i, j))
						grad_sup.at<float>(i, j) = gradient.at<float>(i, j);
					else
						grad_sup.at<float>(i, j) = 0;
					break;
				case CANNY_UP:
					if (gradient.at<float>(i - 1, j) <= gradient.at<float>(i, j))
						grad_sup.at<float>(i, j) = gradient.at<float>(i, j);
					else
						grad_sup.at<float>(i, j) = 0;
					break;
				case CANNY_UP_LEFT:
					if (gradient.at<float>(i - 1, j - 1) <= gradient.at<float>(i, j))
						grad_sup.at<float>(i, j) = gradient.at<float>(i, j);
					else
						grad_sup.at<float>(i, j) = 0;
					break;
				case CANNY_LEFT:
					if (gradient.at<float>(i, j - 1) <= gradient.at<float>(i, j))
						grad_sup.at<float>(i, j) = gradient.at<float>(i, j);
					else
						grad_sup.at<float>(i, j) = 0;
					break;
				case CANNY_DOWN_LEFT:
					if (gradient.at<float>(i + 1, j - 1) <= gradient.at<float>(i, j))
						grad_sup.at<float>(i, j) = gradient.at<float>(i, j);
					else
						grad_sup.at<float>(i, j) = 0;
					break;
				case CANNY_DOWN:
					if (gradient.at<float>(i + 1, j) <= gradient.at<float>(i, j))
						grad_sup.at<float>(i, j) = gradient.at<float>(i, j);
					else
						grad_sup.at<float>(i, j) = 0;
					break;
				case CANNY_DOWN_RIGHT:
					if (gradient.at<float>(i + 1, j + 1) <= gradient.at<float>(i, j))
						grad_sup.at<float>(i, j) = gradient.at<float>(i, j);
					else
						grad_sup.at<float>(i, j) = 0;
					break;
			}
		}
	}
}

void Expr3Node::DoubleThresh(const Mat &grad_sup, Mat &grad_h, Mat &grad_l, double high_th, double low_th)
{
	int rw = grad_sup.rows;
	int cl = grad_sup.cols;
	for(int i = 1; i < rw - 1; i++)
	{
		for(int j = 1; j < cl - 1; j++)
		{
			if(grad_sup.at<float>(i,j) >= high_th)
			{
				grad_h.at<uchar>(i,j) = 255;
				grad_l.at<uchar>(i,j) = 0;
			}
			else if(grad_sup.at<float>(i,j) >= low_th)
			{
				grad_h.at<uchar>(i,j) = 0;
				grad_l.at<uchar>(i,j) = 255;
			}
			else
			{
				grad_h.at<uchar>(i,j) = 0;
				grad_l.at<uchar>(i,j) = 0;
			}
		}
	}
}

void Expr3Node::EdgeConncetion(Mat &grad_h, Mat &grad_l)
{
	int rw = grad_h.rows;
	int cl = grad_h.cols;
	for(int i = 1; i < rw-1; i++)
	{
		for(int j = 1; j < cl-1; j++)
		{
			if(grad_l.at<uchar>(i,j))
			{
				if(grad_h.at<uchar>(i-1,j-1) || grad_h.at<uchar>(i-1, j) || grad_h.at<uchar>(i-1,j+1) ||
						grad_h.at<uchar>(i,j-1) || grad_h.at<uchar>(i,j+1) ||
						grad_h.at<uchar>(i+1,j-1) || grad_h.at<uchar>(i+1,j) || grad_h.at<uchar>(i+1,j+1))
				{
					grad_l.at<uchar>(i,j) = 0;
					grad_h.at<uchar>(i,j) = 255;
					EdgeConncetion(grad_h, grad_l);
				}
				else grad_l.at<uchar>(i,j) = 0;
			}
		}
	}
}

void Expr3Node::EdgeDetector(const Mat &input, Mat &output, double sigma, double high_th, double low_th)
{
	Gaussian(input, output, sigma);
	int rw = input.rows;
	int cl = input.cols;
	
	// calculate gradient amplitude and size
	Mat gradient(rw, cl, CV_32FC1, Scalar(0));
	Mat phase(rw, cl, CV_8UC1, Scalar(0));
	Gradient(output, gradient, phase);
	//imshow("gradient", gradient);
	
	// non-maxima suppression
	Mat grad_sup(gradient.rows, gradient.cols, CV_32FC1, Scalar(0));
	Suppression(gradient, phase, grad_sup);
	//imshow("nms", grad_sup);
	
	// double threshold
	Mat grad_h(grad_sup.rows, grad_sup.cols, CV_8UC1, Scalar(0));
	Mat grad_l(grad_sup.rows, grad_sup.cols, CV_8UC1, Scalar(0));
	DoubleThresh(grad_sup, grad_h, grad_l, high_th, low_th);
	//imshow("high", grad_h);
	//imshow("low", grad_l);
	
	// edge connection
	EdgeConncetion(grad_h, grad_l);
	//imshow("cnct", grad_h);
	
	// output
	output = grad_h.clone();
}

void Expr3Node::DrawLine(Mat &input, double theta, double r)
{
	int rw = input.rows;
	int cl = input.cols;
	if(fabs(fabs(theta) - M_PI_2) <= 0.1)
	{
		for (int j = 0; j < cl; j++)
		{
			int i = (int)((r/sin(theta) - j*cos(theta)/sin(theta)));
			if(i>=0 && i<rw)
			{
				input.at<Vec3b>(i, j)[0] = 255;
				input.at<Vec3b>(i, j)[1] = 0;
				input.at<Vec3b>(i, j)[2] = 0;
			}
		}
	}
	else
	{
		for(int i = 0; i < rw; i++)
		{
			int j = (int)((r/cos(theta)) - i*tan(theta));
			if(j>=0 && j<cl)
			{
				input.at<Vec3b>(i, j)[0] = 255;
				input.at<Vec3b>(i, j)[1] = 0;
				input.at<Vec3b>(i, j)[2] = 0;
			}
		}
	}
}

void Expr3Node::HoughLine(const Mat &input, Mat &output, int thresh)
{
	Mat grey(input.rows, input.cols, CV_8UC1, Scalar(0));
	Mat edge(grey.rows, grey.cols, CV_8UC1, Scalar(0));
	cvtColor(input, grey, COLOR_BGR2GRAY);
	EdgeDetector(grey, edge, 0.8, 20, 10);
	
	double r_max = sqrt(2) * max(edge.cols, edge.rows);
	double theta_max = M_PI;
	int slice = 400;  // num of slices in the param space
	int rw = edge.rows;
	int cl = edge.cols;
	
	// calculate the param space
	Mat param = Mat::zeros(slice, slice, CV_32SC1);
	int max_val = 0;
	for(int i = 0; i < rw; i++)
	{
		for(int j = 0; j < cl; j++)
		{
			if(edge.at<uchar>(i,j))
			{
				for(int m = 0; m < slice; m++)
				{
					double amp = j * cos(2*theta_max*m/slice-theta_max) + i * sin(2*theta_max*m/slice-theta_max) + 1e-2;  // i is y, j is x
					amp = amp*slice/(2*r_max)+(slice/2);
					if(amp >= slice-1) amp = slice-1;
					if(amp <= 0) amp = 0;
					param.at<int>(floor(amp), m) += 1;
					if(param.at<int>(floor(amp), m) >= max_val) max_val = param.at<int>(floor(amp), m);
				}
			}
		}
	}
	
	// normalize param space
	Mat param_8(slice, slice, CV_8UC1, Scalar(0));
	for(int i = 0; i < slice; i++)
	{
		for(int j = 0; j < slice; j++)
		{
			param_8.at<uchar>(i,j) = (int)((255*param.at<int>(i,j))/max_val);
		}
	}
	// imshow("param_8", param_8);
	// get back to spacial space
	for(int i = 0; i < slice; i++)
	{
		for(int j = 0; j < slice; j++)
		{
			if(param_8.at<uchar>(i,j) >= thresh)
			{
				double theta = j*2*theta_max/slice-theta_max;
				double r = (i-slice/2)*2*r_max/slice;
				DrawLine(output, theta, r);
			}
		}
	}
}

void Expr3Node::SobelDirection(const Mat &input, Mat &ori)
{
	int rw = input.rows;
	int cl = input.cols;
	for(int i = 1; i < rw-1; i++)
	{
		for(int j = 1; j < cl-1; j++)
		{
			double gx = input.at<uchar>(i-1,j+1) + 2 * input.at<uchar>(i, j+1) + input.at<uchar>(i+1, j+1)
			        -(input.at<uchar>(i-1,j-1) + 2 * input.at<uchar>(i, j-1) + input.at<uchar>(i+1, j-1));
			double gy = -(input.at<uchar>(i-1,j-1) + 2 * input.at<uchar>(i-1, j) + input.at<uchar>(i-1, j+1))
					+(input.at<uchar>(i+1,j-1) + 2 * input.at<uchar>(i+1, j) + input.at<uchar>(i+1, j+1));
			double dir = fabs(atan2(gy, gx));
			if(dir < 0) dir += (M_PI);
			ori.at<float>(i,j) = dir;
		}
	}
}

void Expr3Node::DrawCircle(Mat &input, int orirw, int oricl, int radius)
{
	int rw = input.rows;
	int cl = input.cols;
	for(double i = 0; i < 4*M_PI*radius; i++)
	{
		double theta = (double)i / (2*radius);
		if ((int) (orirw + radius * cos(theta)) >= 0 && (int) (orirw + radius * cos(theta)) < rw &&
			(int) (oricl + radius * sin(theta)) >= 0 && (int) (oricl + radius * sin(theta)) < cl)
		{
			input.at<Vec3b>((int) (orirw + radius * cos(theta)), (int) (oricl + radius * sin(theta)))[0] = 255;
			input.at<Vec3b>((int) (orirw + radius * cos(theta)), (int) (oricl + radius * sin(theta)))[1] = 0;
			input.at<Vec3b>((int) (orirw + radius * cos(theta)), (int) (oricl + radius * sin(theta)))[2] = 0;
		}
	}
}

void Expr3Node::HoughCircle(const Mat &input, Mat &output, double max_r, double min_r=0)
{
	Mat edge(input.rows, input.cols, CV_8UC1, Scalar(0));
	Mat sobel_ori(input.rows, input.cols, CV_32FC1, Scalar(0));
	Mat grey(input.rows, input.cols, CV_8UC1, Scalar(0));
	
	cvtColor(input, grey, COLOR_BGR2GRAY);
	EdgeDetector(grey, edge, 0.6, 20, 10);
	SobelDirection(grey, sobel_ori);
	//imshow("edge", edge);
	
	// apply center lines
	Mat center = Mat::zeros(input.rows, input.cols, CV_8UC1);
	int rw = input.rows;
	int cl = input.cols;
	for(int i = 0; i < rw; i++)
	{
		for (int j = 0; j < cl; j++)
		{
			if (edge.at<uchar>(i, j))
			{
				center.at<uchar>(i,j) += 1;
				if (fabs(sobel_ori.at<float>(i, j) - M_PI_2) <= M_PI_4)
				{
					for (int n = 1; n < rw-1; n++)
					{
						if (i + n < rw &&
							j + n / sin(sobel_ori.at<float>(i, j)) * cos(sobel_ori.at<float>(i, j)) < cl &&
							j + n / sin(sobel_ori.at<float>(i, j)) * cos(sobel_ori.at<float>(i, j)) >= 0)
							center.at<uchar>(i + n,
											 j + (int)(n / sin(sobel_ori.at<float>(i, j)) *
												 cos(sobel_ori.at<float>(i, j)))) += 1;
						if (i - n >= 0 &&
							j - n / sin(sobel_ori.at<float>(i, j)) * cos(sobel_ori.at<float>(i, j)) < cl &&
							j - n / sin(sobel_ori.at<float>(i, j)) * cos(sobel_ori.at<float>(i, j)) >= 0)
							center.at<uchar>(i - n,
											 j - (int) (n / sin(sobel_ori.at<float>(i, j)) *
														cos(sobel_ori.at<float>(i, j)))) += 1;
					}
				}
				else
				{
					for (int m = 1; m < cl-1; m++)
					{
						if (j + m < cl &&
							i + m / cos(sobel_ori.at<float>(i, j)) * sin(sobel_ori.at<float>(i, j)) < rw &&
							i + m / cos(sobel_ori.at<float>(i, j)) * sin(sobel_ori.at<float>(i, j)) >= 0)
							center.at<uchar>(
									i + (int) (m / cos(sobel_ori.at<float>(i, j)) * sin(sobel_ori.at<float>(i, j))),
									j + m) += 1;
						if (j - m >= 0 &&
							i - m / cos(sobel_ori.at<float>(i, j)) * sin(sobel_ori.at<float>(i, j)) < rw &&
							i - m / cos(sobel_ori.at<float>(i, j)) * sin(sobel_ori.at<float>(i, j)) >= 0)
							center.at<uchar>(
									i - (int) (m / cos(sobel_ori.at<float>(i, j)) * sin(sobel_ori.at<float>(i, j))),
									j - m) += 1;
					}
				}
			}
		}
	}
	
	// find center
	int max_val = 0;
	int max_i;
	int max_j;
	for(int i = 1; i < rw-1; i++)
	{
		for (int j = 1; j < cl - 1; j++)
		{
			if (max_val < center.at<uchar>(i, j))
			{
				max_val = center.at<uchar>(i, j);
				max_i = i;
				max_j = j;
			}
		}
	}
	
	// normalize counts
	for(int i = 1; i < rw-1; i++)
	{
		for (int j = 1; j < cl-1; j++)
		{
			center.at<uchar>(i,j) = (int)(center.at<uchar>(i,j)*(255.0/max_val));
		}
	}
	//imshow("center", center);
	
	// calculate distance(radius space)
	Mat radius_space = Mat::zeros(1, (int)(max_r-min_r), CV_32SC1);
	for(int i = 1; i < rw-1; i++)
	{
		for(int j = 1; j < cl-1; j++)
		{
			if(edge.at<uchar>(i,j))
			{
				int distance = (int) (sqrt((max_i-i)*(max_i-i) + (max_j-j)*(max_j-j)));
				if(distance < max_r && distance > min_r) radius_space.at<int>(0, (int)(distance-min_r)) += 1;
			}
		}
	}
	
	// find qualified radius
	int radius_sum = 0;
	int radius;
	for(int i = 0; i <(int)(max_r-min_r); i++)
	{
		if(radius_sum <= radius_space.at<int>(0, i))
		{
			radius_sum = radius_space.at<int>(0, i);
			radius = i+min_r;
		}
	}
	DrawCircle(output, max_i, max_j, radius);
}

#endif //WEBOTS_WS_EXPR3_HPP
