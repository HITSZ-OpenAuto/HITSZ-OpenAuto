// ***************************************	//
// 机器视觉-课题3
// 黄继凡
// 2023/5/12

// ***************************************	//
// 项目说明
// 复杂环境中亚像素椭圆中心点计算
// 可以先采用blob分析确定初始椭圆中心
// 再从中心向外发射射线，按一维边缘点计算椭圆边界亚像素边缘点
// 再用椭圆拟合计算亚像素椭圆中心点

// ***************************************	//
// 项目流程
// 使用 blob 分析算法确定初始椭圆中心
// 从中心向外发射射线，计算椭圆边界上的一维边缘点
// 可以使用 Canny 边缘检测算法来实现，具体步骤如下：
// a. 对图像进行高斯滤波，去除噪声。
// b. 计算图像的梯度，得到图像的边缘强度和方向。
// c. 使用非极大值抑制算法，将边缘强度最大的像素点作为边缘点。
// d. 使用双阈值算法将边缘点分为强边缘点和弱边缘点。
// e. 使用连接算法将强边缘点连接成边缘线段。
// f. 在边缘线段上计算亚像素边缘点。
// 使用椭圆拟合算法计算亚像素椭圆中心。可以使用 OpenCV 中的 cv::fitEllipse()
// 函数来实现。

#include <iostream>

// 包含opencv库，用于图像处理
#include <opencv2/opencv.hpp>
// 包含chrono库，用于计时
#include <chrono>
#include <map>

using namespace cv;
using namespace std;

// 鼠标交互，用于debug时获取鼠标点击位置
void onMouse(int event, int x, int y, int flags, void* param) {
    if (event == EVENT_FLAG_LBUTTON) {
        cout << "x: " << x << " y: " << y << endl;
    }
}

// 图像处理类，保存本次任务中需要用到的相关存储变量，以及相关函数
class Image_processor {
   public:
    // 构造函数，填入待处理图像路径
    Image_processor(string image_path);
    // 析构函数
    ~Image_processor(){};

    // 设置debug模式
    void Set_Debug(bool debug_mode) { debug = debug_mode; }
    // 预处理图像，去除噪声并增强图像对比度
    void Preprocess_Image();
    // 使用opencv自带的canny边缘检测算法，计算边缘上的一维边缘点
    void OpenCV_Canny_Edge_Detection();
    // 使用opencv自带的椭圆拟合算法，计算亚像素椭圆中心点
    void OpenCV_Fit_Ellipse();

    // TODO 边缘检测，计算边缘上的一维边缘点
    void Edge_Detection();
    // TODO 亚像素边缘检测，计算边缘上的亚像素边缘点
    void Subpixel_Edge_Detection();
    // TODO 椭圆拟合，计算亚像素椭圆中心点
    void Fit_Ellipse();

    // 显示椭圆拟合信息
    void Show_Information();
    // 开始计时
    void Start_Timer();
    // 结束计时，以毫秒为单位输出程序运行时间
    void Stop_Timer();
    // 根据程序各个步骤运行耗时，进行图像分析显示
    void Analyze_Time();

   private:
    Mat image;         // 存储待处理的图像
    Mat gray_image;    // 存储灰度图像
    Mat canvas_image;  // 绘制图像，用于显示图像处理过程

    Mat information_image;  // 存储显示测量结果的图像
    Mat Performance_Chart;  // 存储程序运行时间分析图像

    // 存储计算出的所有椭圆中心点
    vector<Point2f> ellipse_centers;

    // 计时器，用于计算程序运行时间
    chrono::steady_clock::time_point start_time;
    chrono::steady_clock::time_point stop_time;
    double run_time = 0;
    // 时间键值对，用于存储各个步骤运行时间
    map<string, double> time_map;

    bool debug = false;  // 是否开启debug模式
};

Image_processor::Image_processor(string image_path) {
    // [DEBUG]
    if (debug) {
        cout << endl;
        cout << "[DEBUG] Initializing image processor..." << endl;
    }

    // 读取图像
    image = imread(image_path);
}

void Image_processor::Preprocess_Image() {
    // 开始计时
    Start_Timer();

    // [DEBUG]
    if (debug) {
        cout << endl;
        cout << "[DEBUG] Preprocessing image..." << endl;
    }

    // 预处理图像
    // 将图像转换为灰度图像
    cvtColor(image, gray_image, COLOR_BGR2GRAY);
    // 对灰度图像进行高斯滤波
    GaussianBlur(gray_image, gray_image, Size(3, 3), 0, 0);
    // 对灰度图像进行二值化
    threshold(gray_image, gray_image, 0, 255, THRESH_BINARY | THRESH_OTSU);
    // 对灰度图像进行闭运算
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(gray_image, gray_image, MORPH_CLOSE, element);

    // 结束计时
    Stop_Timer();
    // 记录运行时间
    time_map["Preprocess_Image"] = run_time;

    imshow("Preprocessed Image", gray_image);
}

void Image_processor::OpenCV_Canny_Edge_Detection() {
    // 开始计时
    Start_Timer();

    // [DEBUG]
    if (debug) {
        cout << endl;
        cout << "[DEBUG] Detecting edges..." << endl;
    }

    // 边缘检测
    // 使用Canny算法进行边缘检测
    Canny(gray_image, gray_image, 50, 150, 3);

    // 结束计时
    Stop_Timer();
    // 记录运行时间
    time_map["Edge_Detection"] = run_time;

    imshow("Edge Detection", gray_image);
}

void Image_processor::OpenCV_Fit_Ellipse() {
    // 开始计时
    Start_Timer();

    // [DEBUG]
    if (debug) {
        cout << endl;
        cout << "[DEBUG] Fitting ellipses..." << endl;
    }

    // 克隆一份原图像
    canvas_image = image.clone();

    // 椭圆拟合
    // 使用opencv自带的椭圆拟合算法，计算亚像素椭圆中心点
    vector<vector<Point>> contours;
    findContours(gray_image, contours, RETR_LIST, CHAIN_APPROX_NONE);
    for (size_t i = 0; i < contours.size(); i++) {
        // 椭圆拟合
        RotatedRect ellipse_fit = fitEllipse(contours[i]);
        // 保存椭圆中心点
        ellipse_centers.push_back(ellipse_fit.center);
        // [DEBUG]
        if (debug) {
            // 在information_image上绘制椭圆
            ellipse(canvas_image, ellipse_fit, Scalar(0, 0, 255), 2);
            // 在information_image上绘制椭圆中心点
            circle(canvas_image, ellipse_fit.center, 2, Scalar(0, 255, 0), 2);
        }
    }

    // 结束计时
    Stop_Timer();
    // 记录运行时间
    time_map["Fit_Ellipse"] = run_time;

    // [DEBUG]
    if (debug) {
        imshow("Ellipse_Fitting", canvas_image);
    }
}

void Image_processor::Edge_Detection() {
    // 开始计时
    Start_Timer();

    // [DEBUG]
    if (debug) {
        cout << endl;
        cout << "[DEBUG] Detecting edges..." << endl;
    }

    // 边缘检测
    // 计算边缘上的一维边缘点
    // TODO

    // 结束计时
    Stop_Timer();
    // 记录运行时间
    time_map["Edge_Detection"] = run_time;

    imshow("Edge Detection", gray_image);
}

void Image_processor::Subpixel_Edge_Detection() {
    // 开始计时
    Start_Timer();

    // [DEBUG]
    if (debug) {
        cout << endl;
        cout << "[DEBUG] Detecting subpixel edges..." << endl;
    }

    // 子像素边缘检测
    // 计算边缘上的亚像素边缘点
    // TODO

    // 结束计时
    Stop_Timer();
    // 记录运行时间
    time_map["Subpixel_Edge_Detection"] = run_time;

    imshow("Subpixel Edge Detection", gray_image);
}

void Image_processor::Fit_Ellipse() {
    // 开始计时
    Start_Timer();

    // [DEBUG]
    if (debug) {
        cout << endl;
        cout << "[DEBUG] Fitting ellipses..." << endl;
    }

    // 椭圆拟合
    // 计算亚像素椭圆中心点
    // TODO

    // 结束计时
    Stop_Timer();
    // 记录运行时间
    time_map["Fit_Ellipse"] = run_time;

    // [DEBUG]
    if (debug) {
        imshow("Ellipse_Fitting", information_image);
    }
}

void Image_processor::Show_Information() {
    // 在information_image上显示信息
    // 椭圆拟合的个数和前十个椭圆中心点的坐标
    information_image = Mat::zeros(400, 400, CV_8UC3);
    // 显示椭圆拟合的个数
    putText(information_image,
            "Number of ellipses: " + to_string(ellipse_centers.size()),
            Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
    // 显示前十个椭圆中心点的坐标
    for (int i = 0; i < 10; i++) {
        // 如果椭圆拟合的个数不足十个，则跳出循环
        if (i >= ellipse_centers.size()) {
            break;
        }
        // 显示椭圆中心点的坐标
        putText(information_image,
                "Ellipse " + to_string(i + 1) + ": (" +
                    to_string(ellipse_centers[i].x) + ", " +
                    to_string(ellipse_centers[i].y) + ")",
                Point(10, 60 + 30 * i), FONT_HERSHEY_SIMPLEX, 0.5,
                Scalar(0, 255, 0), 1);
    }
    imshow("Information", information_image);
}

void Image_processor::Start_Timer() {
    // 记录开始时间
    start_time = chrono::high_resolution_clock::now();
}

void Image_processor::Stop_Timer() {
    // 记录结束时间
    stop_time = chrono::high_resolution_clock::now();

    // 计算程序运行时间，单位为ns，之后转换为ms
    auto duration =
        chrono::duration_cast<chrono::nanoseconds>(stop_time - start_time);
    run_time = static_cast<double>(duration.count()) / 1e6;
    // 输出程序运行时间
    cout << fixed << setprecision(2) << "duration: " << run_time << "ms"
        << endl;
}

void Image_processor::Analyze_Time() {
    // 在Performance_chart上以条状图绘制运行时间
    // 并在图像上显示各个函数的运行时间，单位为毫秒

    // 根据时间对的长度，确定performance_chart的大小
    Mat performance_chart = Mat::zeros(time_map.size() * 60 + 60, 400, CV_8UC3);
    
    int i = 0;
    for (auto iter = time_map.begin(); iter != time_map.end(); iter++) {
        // 为了保证条状图不会遮挡文字，令条状图的绘制起点和文字错开
        rectangle(performance_chart, Point(0, 50 + i * 60),
                  Point(iter->second / 10, 80 + i * 60), Scalar(0, 255, 0), -1);
        putText(performance_chart, iter->first + ": " + to_string(iter->second),
                Point(0, 40 + i * 60), FONT_HERSHEY_SIMPLEX, 0.5,
                Scalar(0, 0, 255), 1);
        i++;
    }
    imshow("performance_chart", performance_chart);
}

int main() {
    // 创建图像处理器对象
    Image_processor processor(
        "E:"
        "\\作业\\大三下\\机器视觉\\课程设计\\课程设计\\课题3\\椭圆检测\\calib_"
        "01.png");
    // 开启debug
    processor.Set_Debug(true);
    // 图像预处理
    processor.Preprocess_Image();
    // 边缘检测
    processor.OpenCV_Canny_Edge_Detection();
    // 椭圆拟合
    processor.OpenCV_Fit_Ellipse();
    // 显示信息
    processor.Show_Information();

    // 分析运行时间
    processor.Analyze_Time();

    waitKey(0);

    return 0;
}