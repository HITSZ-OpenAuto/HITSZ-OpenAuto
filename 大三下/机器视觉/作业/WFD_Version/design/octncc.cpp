#include <iostream>
#include <string>
#include <vector>
#include <time.h>
#include "opencv2/opencv.hpp"

#define IMGNUM 15

using namespace cv;
using namespace std;

class NCCModel{
public:
    NCCModel(Mat& modelimg, int level, int ksize=3, float sigma=2.f);
    void match(Mat& img, Point3d& result, int level);
    void markresult(Mat& canvas, Point3d& result);

private:
    void search(vector<Mat>& imgPyramid, Point3d& result, int level);

    vector<Mat> modelPyramid;
    vector<Mat> modelCalcor;
    int levelnum;
    int gaussKsize;
    float gaussSigma;
    float passthres=0.2;
};

int main(){
    Mat modelimg;
    clock_t startclock, endclock;

    cout << "构建模板...";
    startclock = clock();
    modelimg = imread("../img/pattern.bmp");
    NCCModel model(modelimg, 8);
    endclock = clock();
    cout << "完成。用时 " << (endclock-startclock)/1e3 << " ms\n";

    for(int i=1;i<=IMGNUM;++i){
        Mat rawimg = imread("../img/img"+to_string(i)+".bmp");
        Point3d result;
        cout << "处理第" << i << "张图片...";
        startclock = clock();
        model.match(rawimg, result, 8);
        endclock = clock();
        cout << "完成。用时 " << (endclock-startclock)/1e3 << " ms\n";
        model.markresult(rawimg, result);
        imwrite("../result/out_"+to_string(i)+".png", rawimg);
    }

    return 0;
}

NCCModel::NCCModel(Mat& modelimg, int level, int ksize, float sigma){
    Mat tmpimg;
    int i;

    gaussKsize = ksize;
    gaussSigma = sigma;

    cvtColor(modelimg, tmpimg, COLOR_BGR2GRAY);

    for (i=0; i<level && tmpimg.cols>0 && tmpimg.rows>0; ++i){
        GaussianBlur(tmpimg, tmpimg, Size(ksize, ksize), sigma);
        modelPyramid.push_back(tmpimg);
        resize(tmpimg, tmpimg, Size(tmpimg.cols/2, tmpimg.rows/2));
    }
    levelnum = i;

    for (auto& model : modelPyramid){
        Mat modelnorm;
        Mat modelmeanmat, modelstddevmat;
        Mat modeldev;
        model.convertTo(modelnorm, CV_32FC1, 1./255);
        meanStdDev(modelnorm, modelmeanmat, modelstddevmat);

        modeldev = (modelnorm-modelmeanmat.at<double>(0,0)) / (model.cols*model.rows*modelstddevmat.at<double>(0,0));
        modelCalcor.push_back(modeldev);
    }
}

void NCCModel::match(Mat& img, Point3d& result, int level){
    vector<Mat> imgPyramid;
    Mat tmpimg;

    cvtColor(img, tmpimg, COLOR_BGR2GRAY);

    for (int i=0; i<level; ++i){
        GaussianBlur(tmpimg, tmpimg, Size(gaussKsize, gaussKsize), gaussSigma);
        imgPyramid.push_back(tmpimg);
        resize(tmpimg, tmpimg, Size(tmpimg.cols/2, tmpimg.rows/2));
    }

    result.z = 0.0;
    search(imgPyramid, result, level);
}

void NCCModel::search(vector<Mat>& imgPyramid, Point3d& result, int level){
    if (level <= levelnum){
        Mat imagelev;
        Mat modellev = modelCalcor[level-1];
        imgPyramid[level-1].convertTo(imagelev, CV_32FC1, 1./255);

        int tx = imagelev.cols-modellev.cols, ty = imagelev.rows-modellev.rows;
        Mat nccscore = nccscore.zeros(Size(tx, ty), CV_32FC1);

        Rect mask(0, 0, modellev.cols, modellev.rows);
        int xst, yst, xee, yee;

        if (result.z > passthres){
            result.x = result.x*2;
            result.y = result.y*2;

            xst = (int)(result.x-1);
            yst = (int)(result.y-1);
            xee = (int)(result.x+2);
            yee = (int)(result.y+2);

            if (xst < 0) mask.x = 0;
            if (yst < 0) mask.y = 0;
            if (xee >= tx) xee = tx-1;
            if (yee >= ty) yee = ty-1;
        }
        else{
            xee = tx-1;
            yee = ty-1;
        }

        for (mask.x=xst; mask.x<=xee; ++mask.x){
            for (mask.y=yst; mask.y<=yee; ++mask.y){
                Mat subimg = imagelev(mask);
                Mat meanmat, stddevmat;
                meanStdDev(subimg, meanmat, stddevmat);
                nccscore.at<float>(mask.y, mask.x) = sum(modellev.mul(subimg-meanmat.at<double>(0,0)))[0]/stddevmat.at<double>(0,0);
            }
        }

        double maxval;
        int maxpos[2];
        minMaxIdx(nccscore, nullptr, &maxval, nullptr, maxpos);

        if (maxpos[1] == 0 || maxpos[1] == nccscore.cols-1){
            result.x = maxpos[1];
        }
        else{
            float dx1 = maxval - nccscore.at<float>(maxpos[0], maxpos[1]-1);
            float dx2 = maxval - nccscore.at<float>(maxpos[0], maxpos[1]+1);
            result.x = maxpos[1] - 0.5 + dx1/(dx1+dx2);
        }
        if (maxpos[0] == 0 || maxpos[0] == nccscore.rows-1){
            result.y = maxpos[0];
        }
        else{
            float dy1 = maxval - nccscore.at<float>(maxpos[0]-1, maxpos[1]);
            float dy2 = maxval - nccscore.at<float>(maxpos[0]+1, maxpos[1]);
            result.y = maxpos[0] - 0.5 + dy1/(dy1+dy2);
        }
        result.z = maxval;
    }

    if (level>1){
        search(imgPyramid, result, level-1);
    }
}

void NCCModel::markresult(Mat& canvas, Point3d& result){
    rectangle(canvas, Rect((int)result.x, (int)result.y, modelPyramid[0].cols, modelPyramid[0].rows), Scalar(66, 66, 233));
    putText(canvas, "ncc score: "+to_string(result.z), Point(result.x+3, result.y+20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
    putText(canvas, "x="+to_string(result.x)+", y="+to_string(result.y), Point(result.x+3, result.y+44), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
}
