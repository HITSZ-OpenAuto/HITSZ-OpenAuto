#include <iostream>
#include <string>
#include <vector>
#include <time.h>
#include "opencv2/opencv.hpp"

#define IMGNUM 15
#define LEVEL 8

using namespace cv;
using namespace std;

void getPyramidLevel(Mat& in, Mat& out, int ksize, float sigma, int level);
void imncc(vector<Mat>& ins, Mat& model, vector<Mat>& nccscores);
void markmatch(Mat& img, Mat& score);

int main(){
    vector<Mat> inImages, grayImages, bluredImages, nccScores;
    Mat model, bluredModel;
    clock_t startclock, endclock;

    cvtColor(imread("../img/pattern.bmp"), model, COLOR_RGB2GRAY);
    for(int i=1;i<=IMGNUM;++i){
        Mat rawImg, grayImg;
        rawImg = imread("../img/img"+to_string(i)+".bmp");
        cvtColor(rawImg, grayImg, COLOR_RGB2GRAY);
        inImages.push_back(rawImg);
        grayImages.push_back(grayImg);
    }

    cout << "图像读取完成，开始计时。\n";
    startclock = clock();
    getPyramidLevel(model, bluredModel, 5, 3, LEVEL);
    for(Mat& img :grayImages){
        Mat blured;
        getPyramidLevel(img, blured, 5, 3, LEVEL);
        bluredImages.push_back(blured);
    }

    imncc(bluredImages, bluredModel, nccScores);

    for(int i=0;i<IMGNUM;++i){
        markmatch(inImages[i], nccScores[i]);
    }
    endclock = clock();
    cout << "图像匹配、标记完成，计时结束。\n";
    cout << "共用时" << (endclock-startclock)/1e6 << "s，平均用时" << (endclock-startclock)/1e3/IMGNUM << "ms。\n";

    for(int i=1;i<=IMGNUM;++i){
        imwrite("../result/out_"+to_string(i)+".jpg", inImages[i]);
    }

    return 0;
}

void getPyramidLevel(Mat& in, Mat& out, int ksize, float sigma, int level){
    Mat tmpImg = in;
    for(int i=1;i<level;++i){
        Mat bluredImg;
        GaussianBlur(tmpImg, bluredImg, Size(ksize, ksize), sigma);
        resize(bluredImg, tmpImg, Size(bluredImg.cols/2, bluredImg.rows/2));
    }
    out = tmpImg;
}

void imncc(vector<Mat>& ins, Mat& model, vector<Mat>& nccscores){
    Mat normalin, normalmodel;
    model.convertTo(normalmodel, CV_32FC1, 1./255);
    int w = model.cols;
    int h = model.rows;
    Mat modelmeanmat, modelstddevmat;
    meanStdDev(normalmodel, modelmeanmat, modelstddevmat);
    float modelmean = modelmeanmat.at<double>(0,0);
    float modelstddev = modelstddevmat.at<double>(0,0);
    Mat modeldev = (normalmodel-modelmean)/(w*h*modelstddev);
    nccscores.empty();
    for(Mat& in :ins){
        int tx = in.cols - w;
        int ty = in.rows - h;
        Mat nccscore = nccscore.zeros(Size(tx, ty), CV_32FC1);
        Rect mask(0,0,w,h);
        int* xp = &(mask.x);
        int* yp = &(mask.y);
        in.convertTo(normalin, CV_32FC1, 1./255);
        for(*xp=0;*xp<tx;++*xp){
            for(*yp=0;*yp<ty;++*yp){
                Mat subImg = normalin(mask);
                Mat meannummat, stddevnummat;
                meanStdDev(subImg, meannummat, stddevnummat);
                float meannum = meannummat.at<double>(0,0);
                float stddevnum = stddevnummat.at<double>(0,0);
                nccscore.at<float>(*yp,*xp) = sum(modeldev.mul(subImg-meannum))[0]/stddevnum;
            }
        }
        nccscores.push_back(nccscore);
    }
}

void markmatch(Mat& img, Mat& score){
    double maxval;
    int maxpos[2];
    int p2 = pow(2,LEVEL-1);
    int w = img.cols-p2*score.cols;
    int h = img.rows-p2*score.rows;
    minMaxIdx(score, nullptr, &maxval, nullptr, maxpos);
    rectangle(img, Rect(maxpos[1]*2, maxpos[0]*2, w, h), Scalar(66, 66, 233));
    putText(img, "ncc score: "+to_string(maxval), Point(maxpos[1]*2+3, maxpos[0]*2+20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
    putText(img, "x="+to_string(maxpos[1]*2)+", y="+to_string(maxpos[0]*2), Point(maxpos[1]*2+3, maxpos[0]*2+44), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
}
