#include <iostream>
#include <string>
#include <vector>
#include <time.h>
#include "opencv2/opencv.hpp"

#define IMGNUM 15
#define LEVEL 8

using namespace cv;
using namespace std;

void getPyramidLevel(Mat& in, vector<Mat>& out, int ksize, float sigma, int level);
void imncc(vector<Mat>& ins, Mat& model, vector<Mat>& nccscores);
void markmatch(Mat &in, double nccscore, pair<int, int> loc, Mat& model);
void imncc_single(Mat& in, Mat& model, Mat& nccscore);




void match(vector<Mat>& imgpyramid, vector<Mat>& modelpyramid, Rect& roi, double &nccScore, int level){
    if(level == 0) return;
    Rect imgROI = roi;
    imgROI.width += modelpyramid[level-1].cols;
    imgROI.height += modelpyramid[level-1].rows;
    int imW = imgpyramid[level-1].cols;
    int imH = imgpyramid[level-1].rows;
    Mat roiImg = imgpyramid[level-1](imgROI);
    Mat nccscore;
    imncc_single(roiImg, modelpyramid[level-1], nccscore);
    double maxval;
    int maxpos[2];
    int p2 = pow(2,LEVEL-1);
    minMaxIdx(nccscore, nullptr, &maxval, nullptr, maxpos);
    roi.x += maxpos[1];
    roi.y += maxpos[0];
    roi.x *= 2;
    roi.y *= 2;
    roi.width = 2;
    roi.height = 2;
    nccScore = maxval;
    match(imgpyramid, modelpyramid, roi, nccScore, level-1);
}


int main(){
    vector<Mat> inImages, grayImages, bluredImages;
    Mat model, bluredModel;
    clock_t startclock, endclock;

    cvtColor(imread("image/pattern.bmp"), model, COLOR_RGB2GRAY);
    for(int i=0;i<IMGNUM;++i){
        Mat rawImg, grayImg;
        rawImg = imread("image/"+to_string(i)+".bmp");
        cvtColor(rawImg, grayImg, COLOR_RGB2GRAY);
        inImages.push_back(rawImg);
        grayImages.push_back(grayImg);
    }

    cout << "图像读取完成，开始计时。\n";
    startclock = clock();

    vector<Mat> modelPyramid;
    getPyramidLevel(model, modelPyramid, 5, 3, LEVEL);
    vector<double> nccScores;
    vector<pair<int,int> > rois;
    for (Mat& img :grayImages) {
        vector<Mat> imgPyramid;
        getPyramidLevel(img, imgPyramid, 5, 3, LEVEL);
        Rect roi(Point(0,0),
                 Point(imgPyramid[LEVEL-1].cols-modelPyramid[LEVEL-1].cols-1,
                       imgPyramid[LEVEL-1].rows-modelPyramid[LEVEL-1].rows-1));
        double nccscore;
        match(imgPyramid,
              modelPyramid,
              roi,
              nccscore,
              LEVEL );
        roi.x /=2;
        roi.y /=2;
        nccScores.push_back(nccscore);
        rois.push_back(make_pair(roi.x, roi.y));
    }

    for(int i=0;i<IMGNUM;++i){
        markmatch(inImages[i], nccScores[i],rois[i], model);
    }
    endclock = clock();
    cout << "图像匹配、标记完成，计时结束。\n";
    cout << "共用时" << (endclock-startclock)/1e6 << "s，平均用时" << (endclock-startclock)/1e3/IMGNUM << "ms。\n";

    for(int i=0;i<IMGNUM;++i){
        imwrite("result/out_"+to_string(i)+".jpg", inImages[i]);
    }
    return 0;
}

void getPyramidLevel(Mat& in, vector<Mat>& out, int ksize, float sigma, int level){
    Mat tmpImg = in;
    Mat bluredImg;
    GaussianBlur(tmpImg, tmpImg, Size(ksize, ksize), sigma);
    out.push_back(tmpImg);
    for(int i=1;i<level;++i){
        resize(tmpImg, tmpImg, Size(tmpImg.cols/2, tmpImg.rows/2));
        out.push_back(tmpImg);
    }
//    out = tmpImg;
}

void imncc_single(Mat& in, Mat& model, Mat& nccscore){
    Mat normalin, normalmodel;
    model.convertTo(normalmodel, CV_32FC1, 1./255);
    int w = model.cols;
    int h = model.rows;
    Mat modelmeanmat, modelstddevmat;
    meanStdDev(normalmodel, modelmeanmat, modelstddevmat);
    float modelmean = modelmeanmat.at<double>(0,0);
    float modelstddev = modelstddevmat.at<double>(0,0);
    Mat modeldev = (normalmodel-modelmean)/(w*h*modelstddev);

    int tx = in.cols - w;
    int ty = in.rows - h;
    nccscore = nccscore.zeros(Size(tx, ty), CV_32FC1);
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
}

void markmatch(Mat &img, double maxval, pair<int, int> loc, Mat& model){

    int w = model.cols;
    int h = model.rows;

    rectangle(img, Rect(loc.first, loc.second, w, h), Scalar(66, 66, 233));
    putText(img, "ncc score: "+to_string(maxval), Point(loc.first+3, loc.second+20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
    putText(img, "x="+to_string(loc.first)+", y="+to_string(loc.second), Point(loc.first+3, loc.second+44), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
}
