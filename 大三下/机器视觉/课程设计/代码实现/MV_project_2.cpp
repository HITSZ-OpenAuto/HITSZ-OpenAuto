// ***************************************	//
// 机器视觉-课题3
// 黄继凡
// 2023/5/28
// ***************************************	//
// 项目说明
// 四层金字塔加速NCC
// NCC模板旋转匹配

// ***************************************	//
// 程序说明:
// 在高层图像中，只进行少量的旋转匹配
// 例如每隔10°或者20°进行一次，找到一个大致的匹配角度范围
// 然后在低层图像中进行更细致的旋转匹配，例如每隔1°或者2°进行一次，找到一个更精确的匹配角度
// 这样可以减少旋转次数，同时保证匹配精度
//
// 在每一层图像中，只在上一层中找到的匹配点附近进行局部搜索，而不需要进行全局搜索
// 这样可以减少搜索范围和计算量，提高匹配速度
// 同时，由于高层图像的分辨率较低，可以容忍一些旋转或者形变的变换，增加匹配的鲁棒性
//
// 在每一层图像中，只保留最佳匹配的结果，而不需要保留所有的结果
// 这样可以减少内存占用和数据传输，提高匹配效率
// 同时，由于最佳匹配的结果往往是最稳定和最可靠的，可以避免一些错误或者冗余的结果

#include <iostream>

// 包含opencv库，用于图像处理
#include <opencv2/opencv.hpp>
// 包含chrono库，用于计时
#include <chrono>
#include <map>

using namespace cv;
using namespace std;

class Pyramid_Rotated_NCC {
   public:
    // 构造函数
    // 参数：
    // 原图像，模板图像，旋转角度范围，旋转角度步长，金字塔层数，金字塔缩放因子
    // 默认旋转角度范围为正负5度，旋转角度步长为0.5度
    // 默认金字塔层数为4，缩放因子为0.5
    Pyramid_Rotated_NCC(Mat srcImg,
                        Mat templateImg,
                        double angleMax = 5.0,
                        double angleMin = -5.0,
                        double unit_angle = 0.5,
                        int layer = 4,
                        double scale_factor = 0.5);
    ~Pyramid_Rotated_NCC();
    // 获取最佳匹配位置及角度
    void getBestMatchPosAndAngle();
    // 设置金字塔层数
    void setPyramidLayer(int layer);
    // 设置金字塔缩放因子
    void setPyramidScaleFactor(double scale_factor);
    // 设置旋转角度范围及步长
    void setAngleRange(double angleMax, double angleMin, double unit_angle);
    // 修改待匹配图像
    void setSrcImg(Mat srcImg);
    // 修改模板图像
    void setTemplateImg(Mat templateImg);

    // 在原始图像上绘制匹配结果
    void drawResult();

    // 设置debug模式
    void setDebugMode(bool debugMode);

   private:
    // 计算图像均值
    Mat getMean(Mat src);
    // 计算图像标准差
    Mat getStdDev(Mat src);
    // 计算图像归一化互相关系数
    Mat getNcc(Mat src1, Mat src2);
    // 计算图像归一化互相关系数矩阵
    Mat getNccMatrix(Mat srcImg, Mat templateImg);
    // 计算局部搜索的图像归一化互相关系数矩阵
    Mat getLocalSearchNccMatrix(Mat srcImg,
                                Mat templateImg,
                                Point bestMatchPos);
    // 获取最佳匹配位置
    Point getBestMatchPos(Mat nccMatrix);
    // 进行金字塔缩放加速
    void pyramidScale(Mat srcImg, Mat templateImg);

    // 启动计时器
    void Start_Timer();
    // 停止计时器
    void Stop_Timer();

    Mat srcImg;       // 待匹配图像
    Mat templateImg;  // 模板图像

    double angleMax;  // 旋转角度范围
    double angleMin;
    double unit_angle;  // 旋转角度步长

    int layer;                 // 金字塔层数
    double scale_factor;       // 金字塔缩放因子
    double current_ncc_value;  // 当前NCC值

    // 防止图像缩放次数过多，导致图像失真
    // 设定一个阈值与图像大小进行比较
    // 如果图像缩放后的大小小于阈值，则不再进行缩放
    int scale_threshold = 50;
    // NCC匹配分数阈值
    // 判定是否下一层可以进行局部搜索
    double localsearch_threshold = 0.8;
    // 判定是否需要继续进行旋转匹配
    double anglematch_threshold = 0.9;
    // 局部搜索范围，以最佳匹配位置为中心，向四周扩展
    int local_search_range = 20;
    bool isLocalSearch = false;  // 是否进行局部搜索

    // 由于缩放可能会进行多次，因此需要记录缩放后的图像
    vector<Mat> scaled_srcImg;       // 缩放后的待匹配图像
    vector<Mat> scaled_templateImg;  // 缩放后的模板图像
    Mat rotateTemplateImg;           // 旋转后的模板图像
    Point center;                    // 旋转中心

    double bestScore;    // 最佳匹配分数
    Point bestMatchPos;  // 最佳匹配位置
    double bestAngle;    // 最佳匹配角度

    bool debugMode;  // debug模式

    // 计时器，用于计算程序运行时间
    chrono::steady_clock::time_point start_time;
    chrono::steady_clock::time_point stop_time;
    long long run_time = 0;
    // 时间键值对，用于存储各个步骤运行时间
    map<string, long long> time_map;
};

Pyramid_Rotated_NCC::Pyramid_Rotated_NCC(Mat srcImg,
                                         Mat templateImg,
                                         double angleMax,
                                         double angleMin,
                                         double unit_angle,
                                         int layer,
                                         double scale_factor) {
    // 设置待匹配图像
    this->srcImg = srcImg;
    // 设置模板图像
    this->templateImg = templateImg;

    // 设置旋转角度范围及步长
    this->angleMax = angleMax;
    this->angleMin = angleMin;
    this->unit_angle = unit_angle;

    // 设置金字塔缩放因子
    this->scale_factor = scale_factor;
    // 设置金字塔层数
    this->layer = layer;

    // 初始值
    this->center = Point(templateImg.cols / 2.0f, templateImg.rows / 2.0f);
    this->bestScore = 0.0;
    this->bestMatchPos = Point(-1, -1);
    this->bestAngle = 0.0;
}

Pyramid_Rotated_NCC::~Pyramid_Rotated_NCC() {}

void Pyramid_Rotated_NCC::setPyramidLayer(int layer) {
    this->layer = layer;
}

void Pyramid_Rotated_NCC::setPyramidScaleFactor(double scale_factor) {
    this->scale_factor = scale_factor;
}

void Pyramid_Rotated_NCC::setAngleRange(double angleMax,
                                        double angleMin,
                                        double unit_angle) {
    this->angleMax = angleMax;
    this->angleMin = angleMin;
    this->unit_angle = unit_angle;
}

void Pyramid_Rotated_NCC::setSrcImg(Mat srcImg) {
    this->srcImg = srcImg;
}

void Pyramid_Rotated_NCC::setTemplateImg(Mat templateImg) {
    this->templateImg = templateImg;
}

void Pyramid_Rotated_NCC::setDebugMode(bool debugMode) {
    this->debugMode = debugMode;
}

void Pyramid_Rotated_NCC::Start_Timer() {
    // 记录开始时间
    start_time = chrono::steady_clock::now();
}

void Pyramid_Rotated_NCC::Stop_Timer() {
    // 记录结束时间
    stop_time = chrono::steady_clock::now();
    // 计算运行时间
    run_time =
        chrono::duration_cast<chrono::milliseconds>(stop_time - start_time)
            .count();
    // 输出运行时间，以毫秒为单位
    if (debugMode == true) {
        // cout << "Run time: " << run_time << " ms" << endl;
    }
}

Mat Pyramid_Rotated_NCC::getMean(Mat src) {
    Mat mean;
    blur(src, mean, Size(3, 3));
    return mean;
}

Mat Pyramid_Rotated_NCC::getStdDev(Mat src) {
    Mat mean;
    Mat stddev;
    meanStdDev(src, mean, stddev);
    return stddev;
}

Mat Pyramid_Rotated_NCC::getNcc(Mat src1, Mat src2) {
    Mat mean1;
    Mat mean2;
    Mat stddev1;
    Mat stddev2;

    meanStdDev(src1, mean1, stddev1);
    meanStdDev(src2, mean2, stddev2);

    Mat ncc;
    ncc.create(src1.size(), CV_32FC1);
    for (int i = 0; i < src1.rows; i++) {
        for (int j = 0; j < src1.cols; j++) {
            ncc.at<float>(i, j) =
                (src1.at<uchar>(i, j) - mean1.at<double>(0)) *
                (src2.at<uchar>(i, j) - mean2.at<double>(0)) /
                (stddev1.at<double>(0) * stddev2.at<double>(0)) /
                (src2.rows * src2.cols);
        }
    }

    return ncc;
}

Mat Pyramid_Rotated_NCC::getNccMatrix(Mat srcImg, Mat templateImg) {
    int width = srcImg.cols - templateImg.cols + 1;
    int height = srcImg.rows - templateImg.rows + 1;

    Mat nccMatrix(height, width, CV_32FC1);

    // 计时开始
    Start_Timer();
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            Rect rect(j, i, templateImg.cols, templateImg.rows);
            Mat roiImg(srcImg(rect));
            nccMatrix.at<float>(i, j) = sum(getNcc(roiImg, templateImg))[0];
        }
    }

    if (debugMode == true) {
        cout << "getNCCMatrix:" << endl;
    }
    // 计时结束
    Stop_Timer();
    // 记录对应耗时
    time_map["getNccMatrix"] = run_time;

    return nccMatrix;
}

Mat Pyramid_Rotated_NCC::getLocalSearchNccMatrix(Mat srcImg,
                                                 Mat templateImg,
                                                 Point bestMatchPos) {
    int width = srcImg.cols - templateImg.cols + 1;
    int height = srcImg.rows - templateImg.rows + 1;
    
    Mat nccMatrix(height, width, CV_32FC1);
    // 初始化
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++){
            nccMatrix.at<float>(i, j) = 0.0;
        }
    }

    // 计时开始
    Start_Timer();
    // 局部搜索范围
    int x_min = bestMatchPos.x - local_search_range;
    int x_max = bestMatchPos.x + local_search_range;
    int y_min = bestMatchPos.y - local_search_range;
    int y_max = bestMatchPos.y + local_search_range;
    // 防止越界
    if (x_min < 0) {
        x_min = 0;
    }
    if (x_max > width) {
        x_max = width;
    }
    if (y_min < 0) {
        y_min = 0;
    }
    if (y_max > height) {
        y_max = height;
    }
    for (int i = y_min; i < y_max; i++) {
        for (int j = x_min; j < x_max; j++) {
            Rect rect(j, i, templateImg.cols, templateImg.rows);
            Mat roiImg(srcImg(rect));
            nccMatrix.at<float>(i, j) = sum(getNcc(roiImg, templateImg))[0];
        }
    }

    if (debugMode == true) {
        cout << "getLocalSearchNCCMatrix:" << endl;
    }
    // 计时结束
    Stop_Timer();
    // 记录对应耗时
    time_map["getLocalSearchNccMatrix"] = run_time;

    return nccMatrix;
}

Point Pyramid_Rotated_NCC::getBestMatchPos(Mat nccMatrix) {
    // 开始计时
    Start_Timer();

    Point bestMatchPos(-1, -1);
    double maxVal = 0.0;
    minMaxLoc(nccMatrix, NULL, &maxVal, NULL, &bestMatchPos);

    // 停止计时
    Stop_Timer();
    // 记录对应耗时
    time_map["getNccMatrix"] = run_time;

    return bestMatchPos;
}

void Pyramid_Rotated_NCC::pyramidScale(Mat srcImg, Mat templateImg) {
    // 根据指定的金字塔层数，进行缩放
    for (int i = 0; i < layer; i++) {
        // 缩放后的图像
        Mat tmp_scaled_srcImg;
        Mat tmp_scaled_templateImg;
        if (i == 0) {
            tmp_scaled_srcImg = srcImg.clone();
            tmp_scaled_templateImg = templateImg.clone();
        } else {
            resize(scaled_srcImg[i - 1], tmp_scaled_srcImg, Size(),
                   scale_factor, scale_factor);
            resize(scaled_templateImg[i - 1], tmp_scaled_templateImg, Size(),
                   scale_factor, scale_factor);
        }

        // 防止图像缩放次数过多，导致图像失真
        if (tmp_scaled_srcImg.cols < scale_threshold ||
            tmp_scaled_srcImg.rows < scale_threshold) {
            break;
        }

        // 将缩放后的图像存储到vector中
        // vector中越靠近后面的图像越小
        scaled_srcImg.push_back(tmp_scaled_srcImg);
        scaled_templateImg.push_back(tmp_scaled_templateImg);
    }
    // 输出缩放后的图像数量
    if (debugMode == true) {
        cout << "scaled_srcImg.size():" << scaled_srcImg.size() << endl;
        cout << "scaled_templateImg.size():" << scaled_templateImg.size()
             << endl;
        cout << "actual layer of pyramid:" << scaled_srcImg.size() << endl;
    }
}

void Pyramid_Rotated_NCC::getBestMatchPosAndAngle() {
    // 进行金字塔缩放
    pyramidScale(srcImg, templateImg);

    // 从最上层开始进行匹配
    // 根据每层匹配的最大NCC数值，判定下一层是否可以进行局部搜索
    for (int i = scaled_srcImg.size() - 1; i >= 0; i--) {
        // 旋转模板图像，进行匹配
        // 若当前匹配结果最大NCC数值大于局部搜索阈值，则进行局部搜索
        // 否则，下一层进行全局搜索
        // 局部搜索范围为最佳匹配位置附近的10*10的区域
        // 若当前匹配结果最大NCC数值大于旋转匹配阈值，则停止旋转匹配
        // 否则，继续旋转匹配

        // 若不是第一次匹配，调整对应的匹配位置
        if (i != scaled_srcImg.size() - 1) {
            bestMatchPos.x /= scale_factor;
            bestMatchPos.y /= scale_factor;
        }

        if(debugMode == true)
        {
            cout << endl;
            cout << "current layer:" << i + 1 << endl;
            cout << "bestMatchPos:" << bestMatchPos << endl;
        }

        // 当前最佳匹配分数
        double scoreTemp = 0.0;
        bestScore = 0.0;
        for (double angleTemp = angleMin; angleTemp <= angleMax;
             angleTemp += unit_angle) {
            // 旋转模板图像
            Mat rotateTemplateImg(scaled_templateImg[i].size(), CV_8UC3);

            Mat rotMat = getRotationMatrix2D(center, angleTemp, 1.0f);
            warpAffine(scaled_templateImg[i], rotateTemplateImg, rotMat,
                       scaled_templateImg[i].size());
            Mat nccMatrix;  // NCC矩阵
            Point matchPos;  // 当前最优匹配位置
            // 全局搜索
            if (isLocalSearch == false) {
                nccMatrix = getNccMatrix(scaled_srcImg[i], rotateTemplateImg);
                matchPos = getBestMatchPos(nccMatrix);
            }
            // 局部搜索
            else {
                nccMatrix = getLocalSearchNccMatrix(
                    scaled_srcImg[i], rotateTemplateImg, bestMatchPos);
                matchPos = getBestMatchPos(nccMatrix);
            }
            // 记录最佳匹配结果
            scoreTemp = nccMatrix.at<float>(matchPos.y, matchPos.x);
            if (scoreTemp > bestScore) {
                bestScore = scoreTemp;
                bestMatchPos = matchPos;
                bestAngle = angleTemp;
                if (debugMode == true) {
                    cout << endl;
                    cout << "bestScore:" << bestScore << endl;
                    cout << "bestMatchPos:" << bestMatchPos << endl;
                    cout << "bestAngle:" << bestAngle << endl;
                    // 显示当前匹配的图像
                    imshow("current_match", scaled_srcImg[i](Rect(
                                                 bestMatchPos.x,
                                                 bestMatchPos.y,
                                                 scaled_templateImg[i].cols,
                                                 scaled_templateImg[i].rows)));
                    waitKey(30);
                }
            }
            // 若当前匹配结果最大NCC数值大于旋转匹配阈值，则停止旋转匹配
            if (scoreTemp > anglematch_threshold) {
                break;
            }
        }
        // 若当前匹配结果最大NCC数值大于局部搜索阈值，则下一次进行局部搜索
        if (bestScore > localsearch_threshold) {
            isLocalSearch = true;
        } else {
            // 否则，如果当前已经是全局搜索
            // 且匹配结果最大NCC数值小于局部搜索阈值
            // 则下一层进行全局搜索
            if (isLocalSearch == false) {
                break;
            } else {
                // 否则，当前由局部搜索转为全局搜索
                // 重新进行全局搜索
                isLocalSearch = false;
                i++;
            }
        }
    }

    // 记录对应耗时
    time_map["getBestMatchPosAndAngle"] = run_time;
}

void Pyramid_Rotated_NCC::drawResult() {
    // 绘制匹配结果
    Rect rect(bestMatchPos.x, bestMatchPos.y, templateImg.cols,
              templateImg.rows);
    // 克隆原始图像，转换为彩色图像
    Mat srcImgColor = srcImg.clone();
    cvtColor(srcImgColor, srcImgColor, COLOR_GRAY2BGR);
    // 绘制匹配结果
    rectangle(srcImgColor, rect, Scalar(0, 0, 255), 2, 8, 0);
    // 绘制匹配结果的位置和角度
    putText(srcImgColor,
            "bestMatchPos:" + to_string(bestMatchPos.x) + "," +
                to_string(bestMatchPos.y),
            Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
    putText(srcImgColor, "bestAngle:" + to_string(bestAngle), Point(10, 60),
            FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
    // 绘制匹配结果的分数
    putText(srcImgColor,
            "bestScore:" + to_string(bestScore) + "/" + to_string(1.0),
            Point(10, 90), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);

    // 输出各个步骤的运行时间
    if (debugMode == true) {
        cout << endl;
        cout << "time_map:" << endl;
        for (auto iter = time_map.begin(); iter != time_map.end(); iter++) {
            cout << iter->first << ":" << iter->second << " ms" << endl;
        }
    }

    if (debugMode == true) {
        // 输出匹配结果的位置和角度
        cout << "drawResult:" << endl;
        cout << "bestMatchPos:" << bestMatchPos << endl;
        cout << "bestAngle:" << bestAngle << endl;
    }

    imshow("result", srcImgColor);
}

int main() {
    Mat srcImage = imread(
        "E:\\作业\\大三下\\机器视觉\\课程设计\\课程设计\\课题2\\IMAGEB1.bmp");
    Mat templateImage = imread(
        "E:\\作业\\大三下\\机器视觉\\课程设计\\课程设计\\课题2\\pattern.bmp");
    cvtColor(srcImage, srcImage, COLOR_BGR2GRAY);
    cvtColor(templateImage, templateImage, COLOR_BGR2GRAY);

    imshow("origin_image", srcImage);
    imshow("template_image", templateImage);
    waitKey(30);
    
    Pyramid_Rotated_NCC matcher(srcImage, templateImage, 5.0, -5.0, 0.5, 4,
                                0.5);
    matcher.setDebugMode(true);
    matcher.getBestMatchPosAndAngle();
    matcher.drawResult();

    waitKey(0);

    return 0;
}