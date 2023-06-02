// ***************************************	//
// 机器视觉-课题6
// 黄继凡 何景淞
// 2023/5/12

// ***************************************	//
// 项目说明
// 机器视觉尺寸测量项目
// 向程序输入图片，测量对应的物体宽度和间隔宽度
// 1D测量，亚像素精度，并考虑计算时间

// ***************************************	//
// 处理流程：
// 读取图片，进行图片的预处理
// 通过轮廓检测找出目标物体，并计算对应最小外接矩形
// 计算目标物体的长度信息和彼此间隔

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
    ~Image_processor() {};

    // 设置debug模式
    void Set_Debug(bool debug_mode) { debug = debug_mode; }
    // 预处理图像，去除噪声并增强图像对比度
    void Preprocess_Image();
    // 检测图像中的轮廓点集
    void Detect_Contours();
    // 检测每个轮廓的最小外接矩形
    void Detect_Rectangles();
    // 测量每个矩形的长度和宽度
    void Measure_Width_And_Length();
    // 测量条状物体之间的间隔宽度
    void Measure_Spacing();
    // 在图像上显示测量结果
    void Show_Result();

    // 开始计时
    void Start_Timer();
    // 结束计时，以毫秒为单位输出程序运行时间
    void Stop_Timer();
    // 根据程序各个步骤运行耗时，进行图像分析显示
    void Analyze_Time();

private:
    Mat image;       // 存储待处理的图像
    Mat gray_image;  // 存储灰度图像

    Mat information_image;  // 存储显示测量结果的图像
    Mat Performance_Chart;  // 存储程序运行时间分析图像

    vector<Vec4i> hierarchy;         // 存储轮廓的层级信息
    vector<vector<Point>> contours;  // 存储检测到的轮廓点集
    vector<RotatedRect> rectangles;  // 存储每个轮廓的最小外接矩形
    vector<double> object_widths;    // 存储每个矩形的宽度
    vector<double> object_lengths;   // 存储每个矩形的长度

    // 向量，存储物体宽度和间隔宽度，以及物体低端到顶端的长度
    vector<double> object_spacings;

    // 计时器，用于计算程序运行时间
    chrono::steady_clock::time_point start_time;
    chrono::steady_clock::time_point stop_time;
    long long run_time = 0;
    // 时间键值对，用于存储各个步骤运行时间
    map<string, long long> time_map;

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
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(gray_image, gray_image, MORPH_CLOSE, element);

    // 结束计时
    Stop_Timer();
    // 记录运行时间
    time_map["Preprocess_Image"] = run_time;
}

void Image_processor::Detect_Contours() {
    // 开始计时
    Start_Timer();

    // [DEBUG]
    if (debug) {
        cout << endl;
        cout << "[DEBUG] Detecting contours..." << endl;
    }

    // 轮廓检测
    findContours(gray_image, contours, hierarchy, RETR_TREE,
        CHAIN_APPROX_SIMPLE, Point(0, 0));
    // 由于给定测试图像存在边缘较小的黑色方块和黑色边框，不将其作为目标物体
    // 根据面积大小，将面积小于100、大于100,000的轮廓过滤掉
    for (int i = 0; i < contours.size(); i++) {
        // 输出当前轮廓的面积
        if (debug) {
            cout << "Area of contour " << i << ": " << contourArea(contours[i])
                << endl;
        }
        if (contourArea(contours[i]) < 100 ||
            contourArea(contours[i]) > 100000) {
            contours.erase(contours.begin() + i);
            i--;
        }
    }
    // 输出轮廓的数量信息
    if (debug) {
        cout << "Number of contours detected: " << contours.size() << endl;
    }

    // 结束计时
    Stop_Timer();
    // 记录运行时间
    time_map["Detect_Contours"] = run_time;
}

void Image_processor::Detect_Rectangles() {
    // 开始计时
    Start_Timer();

    // [DEBUG]
    if (debug) {
        cout << endl;
        cout << "[DEBUG] Detecting rectangles..." << endl;
    }

    // 检测矩形
    for (int i = 0; i < contours.size(); i++) {
        // 检测轮廓的最小外接矩形
        // rect 包含矩形的中心点坐标、长度和宽度、旋转角度
        // rect.size.width 为矩形的长度
        // rect.size.height 为矩形的宽度
        // rect.center 为矩形的中心点坐标
        // rect.angle 为矩形的旋转角度
        // rect.points(vertices) 为矩形的四个顶点坐标
        // vertices[0] 为矩形的左上角顶点坐标
        // vertices[1] 为矩形的右上角顶点坐标
        // vertices[2] 为矩形的右下角顶点坐标
        // vertices[3] 为矩形的左下角顶点坐标
        RotatedRect rect = minAreaRect(contours[i]);
        // 输出当前矩形的长度和宽度
        if (debug) {
            cout << "Width of rectangle " << i << ": " << rect.size.width
                << endl;
            cout << "Length of rectangle " << i << ": " << rect.size.height
                << endl;
        }
        // 将矩形加入到矩形集合中
        rectangles.push_back(rect);
    }
    // 按照矩形中心点的x坐标对矩形进行排序
    sort(rectangles.begin(), rectangles.end(),
         [](const RotatedRect& a, const RotatedRect& b) {
             return a.center.x < b.center.x;
         });
    // 输出矩形的数量信息
    if (debug) {
        cout << "Number of rectangles detected: " << rectangles.size() << endl;
    }

    // 结束计时
    Stop_Timer();
    // 记录运行时间
    time_map["Detect_Rectangles"] = run_time;
}

void Image_processor::Measure_Width_And_Length() {
    // 开始计时
    Start_Timer();

    // [DEBUG]
    if (debug) {
        cout << endl;
        cout << "[DEBUG] Measuring width and length..." << endl;
    }

    // 计算每个矩形的长度和宽度
    for (int i = 0; i < rectangles.size(); i++) {
        // 输出当前矩形的长度和宽度
        if (debug) {
            cout << "Width of rectangle " << i << ": "
                << rectangles[i].size.height << endl;
            cout << "Length of rectangle " << i << ": "
                << rectangles[i].size.width << endl;
        }
        object_widths.push_back(rectangles[i].size.height);
        object_lengths.push_back(rectangles[i].size.width);
    }

    // 结束计时
    Stop_Timer();
    // 记录运行时间
    time_map["Measure_Width_And_Length"] = run_time;
}

void Image_processor::Measure_Spacing() {
    // 开始计时
    Start_Timer();

    // [DEBUG]
    if (debug) {
        cout << endl;
        cout << "[DEBUG] Measuring spacing..." << endl;
    }

    // 计算条状物体之间的间隔宽度，并存储在object_spacing中
    // 此处的间隔宽度为相邻的两个条状目标的右侧边缘到下一个条状目标的左侧边缘的距离
    // 考虑到矩形彼此之间不一定完全平行，可以根据第二个矩形的左侧边缘拟合出一条直线
    // 使用第一个矩形右上方顶点到直线的距离作为间隔宽度
    for (int i = 0; i < rectangles.size() - 1; i++) {
        Point2f vertices[4];
        Point2f vertices_next[4];
        rectangles[i].points(vertices);
        rectangles[i + 1].points(vertices_next);
        // 计算第二个矩形的左侧边缘拟合出的直线
        // 直线方程为y = kx + b
        double k = (vertices_next[0].y - vertices_next[3].y) /
            (vertices_next[0].x - vertices_next[3].x);
        double b = vertices_next[0].y - k * vertices_next[0].x;
        // 计算第一个矩形右上方顶点到直线的距离
        double distance = abs(k * vertices[1].x - vertices[1].y + b) /
            sqrt(k * k + 1);
        // 输出当前矩形的间隔宽度
        if (debug) {
            cout << "Spacing of rectangle " << i << ": " << distance << endl;
        }
        object_spacings.push_back(distance);
    }
    

    // 结束计时
    Stop_Timer();
    // 记录运行时间
    time_map["Measure_Spacing"] = run_time;
}

void Image_processor::Show_Result() {
    // 开始计时
    Start_Timer();

    // [DEBUG]
    if (debug) {
        cout << "[DEBUG] Showing result..." << endl;
    }

    // 显示原始图像
    imshow("origin_image", image);

    // [DEBUG]
    if (debug) {
        // 开启鼠标回调函数
        setMouseCallback("origin_image", onMouse, 0);
    }

    // 在处理后的图像上显示测量结果
    Mat result_image = gray_image.clone();
    // 转为彩色图像
    cvtColor(result_image, result_image, COLOR_GRAY2BGR);

    for (int i = 0; i < rectangles.size(); i++) {
        // 绘制矩形
        Point2f vertices[4];
        rectangles[i].points(vertices);
        for (int j = 0; j < 4; j++) {
            line(result_image, vertices[j], vertices[(j + 1) % 4],
                Scalar(0, 255, 0), 2);
        }
        // 绘制矩形中心点
        circle(result_image, rectangles[i].center, 2, Scalar(0, 255, 255), 2);
        // 在矩形左上角外侧绘制矩形编号
        putText(result_image, to_string(i), Point(vertices[1].x - 10,
            vertices[1].y - 10),
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1);
    }
    for (int i = 0; i < rectangles.size(); i++) {
        // 绘制矩形长度和宽度
        putText(result_image, to_string(object_widths[i]),
            Point(rectangles[i].center.x, rectangles[i].center.y - 20),
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
        putText(result_image, to_string(object_lengths[i]),
            Point(rectangles[i].center.x - 10, rectangles[i].center.y),
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
    }
    // 显示图像
    imshow("result", result_image);

    // 在information_image上以文字形式显示测量结果
    // 其中目标物体的宽度和间隔取平均值，长度分别显示
    Mat information_image = Mat::zeros(800, 400, CV_8UC3);
    double mean_width = 0;
    double mean_spacing = 0;
    for (int i = 0; i < object_widths.size(); i++) {
        mean_width += object_widths[i];
    }
    mean_width /= object_widths.size();
    for (int i = 0; i < object_spacings.size(); i++) {
        mean_spacing += object_spacings[i];
    }
    mean_spacing /= (object_lengths.size()-1);
    // 显示平均值
    putText(information_image, "Width: " + to_string(mean_width), Point(0, 20),
        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
    putText(information_image, "Spacing: " + to_string(mean_spacing),
        Point(0, 60), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
    // 显示长度
    for (int i = 0; i < object_lengths.size(); i++) {
        putText(information_image,
            "Length " + to_string(i) + ": " + to_string(object_lengths[i]),
            Point(0, 100 + i * 40), FONT_HERSHEY_SIMPLEX, 0.5,
            Scalar(0, 0, 255), 1);
    }

    imshow("information", information_image);

    // 结束计时
    Stop_Timer();

    // 结束计时
    Stop_Timer();
    // 记录运行时间
    time_map["Show_Result"] = run_time;
}

void Image_processor::Start_Timer() {
    // 记录开始时间
    start_time = chrono::steady_clock::now();
}

void Image_processor::Stop_Timer() {
    // 记录结束时间
    stop_time = chrono::steady_clock::now();
    // 计算运行时间
    run_time =
        chrono::duration_cast<chrono::milliseconds>(stop_time - start_time)
        .count();
    // 输出运行时间，以毫秒为单位
    cout << "Run time: " << run_time << " ms" << endl;
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
        "E:\\作业\\大三下\\机器视觉\\课程设计\\课程设计\\课题6\\noised_0.5."
        "bmp");
    // 图像预处理
    processor.Preprocess_Image();
    // 开启debug
    processor.Set_Debug(true);
    // 轮廓检测
    processor.Detect_Contours();
    // 检测矩形
    processor.Detect_Rectangles();
    // 测量矩形的长度和宽度
    processor.Measure_Width_And_Length();
    // 测量矩形之间的间隔
    processor.Measure_Spacing();
    // 显示测量结果
    processor.Show_Result();
    // 分析运行时间
    processor.Analyze_Time();

    waitKey(0);

    return 0;
}