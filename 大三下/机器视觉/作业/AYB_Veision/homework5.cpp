#include <stdlib.h>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;

void AvgFilter(Mat img, Mat dst, int kernel_size)
{
    int kernel_len = kernel_size * kernel_size;
    double* kernel = new double[kernel_len];
    for(int i=0; i<kernel_len; i++)
        kernel[i] = 1.0 / kernel_len;

    for(int i=kernel_size / 2; i < dst.rows - kernel_size / 2; i++)
    {
        for(int j=kernel_size / 2; j < dst.cols - kernel_size / 2; j++)
        {
            double pixel = 0;
            for(int m=-kernel_size / 2; m<=kernel_size / 2; m++)
            {
                for(int n=-kernel_size / 2; n<=kernel_size / 2; n++){
                    pixel += img.at<uchar>(i+m, j+n) * kernel[kernel_size * kernel_size / 2 + m * kernel_size + n];
                }
            }
            dst.at<uchar>(i, j) = (int)pixel;
        }
    }
}

// 分离均值滤波
void AvgDisFilter(Mat img, Mat dst, int kernel_size)
{
    double* kernel_h = new double[kernel_size];
    double* kernel_v = new double[kernel_size];
    for(int i=0; i<kernel_size; i++)
    {
        kernel_h[i] = 1.0 / kernel_size;
        kernel_v[i] = 1.0 / kernel_size;
    }

    for(int i=kernel_size / 2; i < dst.rows - kernel_size / 2; i++)
    {
        for(int j=kernel_size / 2; j < dst.cols - kernel_size / 2; j++)
        {
            double pixel = 0;
            for(int m=-kernel_size / 2; m<=kernel_size / 2; m++)
            {
                pixel += img.at<uchar>(i+m, j) * kernel_h[kernel_size / 2 + m];
            }
            dst.at<uchar>(i, j) = (int)pixel;
        }
    }

    for(int i=kernel_size / 2; i < dst.rows - kernel_size / 2; i++)
    {
        for(int j=kernel_size / 2; j < dst.cols - kernel_size / 2; j++)
        {
            double pixel = 0;
            for(int n=-kernel_size / 2; n<=kernel_size / 2; n++)
            {
                pixel += dst.at<uchar>(i, j+n) * kernel_v[kernel_size / 2 + n];
            }
            dst.at<uchar>(i, j) = (int)pixel;
        }
    }
}

void testCase_Fliter3(const Mat &img,bool show_diff = true){
    Mat img_avg_3(img.rows, img.cols, CV_8UC1, Scalar(0));
    Mat img_avg_3s(img.rows, img.cols, CV_8UC1, Scalar(0));
    AvgFilter(img, img_avg, 3);
    AvgDisFilter(img, img_avg_s, 3);
    imshow("img", img);
    imshow("Kernel3 Avg", img_avg_3);
    imshow("Kernel3 Avg_Sep", img_avg_3s);
    waitKey(0);
    if(show_diff){
        Mat dis(img.rows, img.cols, CV_8UC1, Scalar(0));
        dis = img_avg_3 - img_avg_3s;
        imshow("difference", dis);
        waitKey(0);
    }
}

void testCase_Fliter5_TimeCount(const Mat &img,bool show_diff = true){
    clock_t startTime, endTime;
    startTime = clock();
    Mat img_avg_5(img.rows, img.cols, CV_8UC1, Scalar(0));
    Filter(img, img_avg_5, 5);
    endTime = clock();
    cout << "Avg Fliter Run Time(with Kernel size = 5): " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
    startTime = clock();
    Mat img_avg_5s(img.rows, img.cols, CV_8UC1, Scalar(0));
    AvgDisFilter(img, img_avg_5s, 5);
    endTime = clock();
    cout << "Avg Fliter Run Time(with Kernel size = 5): " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
    imshow("Kernel5 Avg", img_avg_5);
    imshow("Kernel5 Avg_Sep", img_avg_5s);
    if(show_diff){
        Mat dis(img.rows, img.cols, CV_8UC1, Scalar(0));
        dis = img_avg_5 - img_avg_5s;
        imshow("difference", dis);
        waitKey(0);
    }
    waitKey(0);
}
int main() {
    Mat img = imread("./test.txt");
    cvtColor(img,img,COLOR_BGR2GRAY);
    testCase_Fliter3(img,true);
    testCase_Fliter5_TimeCount(img,true);

    return 0;
}
