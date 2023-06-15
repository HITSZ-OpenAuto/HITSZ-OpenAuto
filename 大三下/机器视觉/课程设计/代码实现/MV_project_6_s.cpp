// ***************************************	//
// 机器视觉-课题6
// 黄继凡 何景淞
// 2023/5/12

// ***************************************	//
// 项目说明
// 机器视觉尺寸测量项目
// 向程序输入图片，测量对应的物体宽度和间隔宽度
// 1D测量，亚像素精度，并考虑计算时间

#include <iostream>

// 包含opencv库，用于图像处理
#include <opencv2/opencv.hpp>
// 包含chrono库，用于计时
#include <chrono>
#include <map>

using namespace cv;
using namespace std;

// 鼠标交互，用于debug时获取鼠标点击位置
void onMouse(int event, int x, int y, int flags, void* param);

// 工具类，卡尺
// 用于识别并测量两点范围内的线段距离，亚像素精度
// 用户点击原始图像要进行测量的两点
// 程序会在原始图像上绘制两点之间的连线
// 并在information_image上以文字形式显示线段内的相关距离信息
class Caliper {
   public:
    // 构造函数
    // 参数：原始图像路径
    Caliper(string image_path);
    ~Caliper();                  // 析构函数
    void Init();                 // 初始化卡尺
    void Set_Debug(bool debug);  // 设置是否显示调试信息
    void Upgrade_Image();        // 更新图像显示
    void OneD_Measure();         // 一维测量
    void Loop();                 // 循环处理

    // 开始计时
    void Start_Timer();
    // 结束计时，以毫秒为单位输出程序运行时间
    void Stop_Timer();
    // 根据程序各个步骤运行耗时，进行图像分析显示
    void Analyze_Time();

    friend void onMouse(int event, int x, int y, int flags, void* userdata);

   private:
    Mat original_image;     // 原始图像
    Mat canvas_image;       // 用于显示测量操作的图像
    Mat information_image;  // 用于显示测量结果的图像

    Point2f points[2];    // 两点的坐标
    double distance;      // 两点之间的距离
    Vec4f selected_line;  // 两点之间的连线

    // 计时器，用于计算程序运行时间
    chrono::steady_clock::time_point start_time;
    chrono::steady_clock::time_point stop_time;
    double run_time = 0;
    // 时间键值对，用于存储各个步骤运行时间
    map<string, double> time_map;

    bool debug = false;  // 用于判断是否显示调试信息
};

Caliper::Caliper(string image_path) {
    this->original_image = imread(image_path);
    distance = 0;
}

Caliper::~Caliper() {}

void Caliper::Init() {
    // 显示原始图像
    if (debug == true)
        imshow("original", original_image);

    // 创建用于显示测量操作的图像
    canvas_image = original_image.clone();
    // 显示cavas_image
    imshow("canvas", canvas_image);

    // 初始化information_image
    information_image = Mat::zeros(800, 400, CV_8UC3);
    // 显示information_image
    imshow("information", information_image);

    // 设置鼠标回调函数
    setMouseCallback("canvas", onMouse, this);

    waitKey(30);
}

void Caliper::Set_Debug(bool debug) {
    this->debug = debug;
}

void Caliper::Upgrade_Image() {
    // 创建用于显示测量操作的图像
    canvas_image = original_image.clone();
    // 绘制两个点
    circle(canvas_image, points[0], 5, Scalar(0, 0, 255), -1);
    circle(canvas_image, points[1], 5, Scalar(0, 255, 0), -1);
    // 在canvas_image上绘制两点之间的连线
    line(canvas_image, points[0], points[1], Scalar(0, 0, 255), 2);
    // 显示cavas_image
    imshow("canvas", canvas_image);

    // 初始化information_image
    information_image = Mat::zeros(800, 400, CV_8UC3);
    // 将information_image上面一部分的背景设置为白色
    rectangle(information_image, Point(0, 0), Point(400, 140),
        Scalar(255, 255, 255), -1);
    // 操作提示，按下空格键开始测量，按下ESC键退出程序
    putText(information_image, "1D measurement tool", Point(70, 30),
            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 1);
    putText(information_image, "Press SPACE to measure", Point(0, 50),
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
    putText(information_image, "Press ESC to exit", Point(0, 70),
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
    // 在information_image上显示两点的坐标和彼此之间的距离
    putText(information_image,
            "point1: (" + to_string(points[0].x) + ", " +
                to_string(points[0].y) + ")",
            Point(0, 90), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
    putText(information_image,
            "point2: (" + to_string(points[1].x) + ", " +
                to_string(points[1].y) + ")",
            Point(0, 110), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);

    distance = sqrt(pow(points[0].x - points[1].x, 2) +
                    pow(points[0].y - points[1].y, 2));
    putText(information_image, "distance: " + to_string(distance),
            Point(0, 130), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
    // 显示information_image
    imshow("information", information_image);
}

// 1D测量
// 读取输入的图像和直线，将图像转换为灰度图
// 在直线上沿着垂直方向进行等间隔采样，得到一组采样点的灰度值
// 对采样点的灰度值进行平滑处理，例如使用高斯滤波或中值滤波，以减少噪声的影响
// 在平滑后的灰度值中寻找边缘点，即灰度值发生较大变化的点
// 可以使用一阶或二阶差分，或者使用Sobel算子等边缘检测方法
// 对边缘点进行亚像素精度的插值，以提高测量的准确性。可以使用线性插值，或者使用三次样条插值等更高阶的方法
// 计算两个边缘点之间的距离，即为测量结果
// 输出测量结果，并显示在图像上
void Caliper::OneD_Measure() {
    // 定时开始
    Start_Timer();

    // 根据输入的原始图像，创建对应的灰度图
    Mat gray_image;
    cvtColor(original_image, gray_image, COLOR_BGR2GRAY);
    // 滤波处理
    GaussianBlur(gray_image, gray_image, Size(3, 3), 0, 0);
    // 二值化，这一步比较离谱
    // threshold(gray_image, gray_image, 0, 255, THRESH_BINARY | THRESH_OTSU);
    // 闭合运算
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(gray_image, gray_image, MORPH_CLOSE, element);
    // 开运算
    morphologyEx(gray_image, gray_image, MORPH_OPEN, element);

    // 停止计时
    Stop_Timer();
    // 记录对应步骤的运行时间
    time_map["Preprocess_Image"] = run_time;

    if (debug == true)
        imshow("gray", gray_image);

    // 定时开始
    Start_Timer();

    // 在直线方向上，沿着垂直方向等间隔采样
    // 采样点个数，采样点个数与直线长度成正比
    double length = sqrt(pow(points[1].x - points[0].x, 2) +
                         pow(points[1].y - points[0].y, 2));
    int n = static_cast<int>(length / 2);
    vector<double> x(n);  // x坐标
    vector<double> y(n);  // y坐标

    // 直线方程
    // y = kx + b
    // k = (y2 - y1) / (x2 - x1)
    // b = y1 - k * x1
    double k = (points[1].y - points[0].y) / (points[1].x - points[0].x);
    double b = points[0].y - k * points[0].x;

    // 采样点的坐标
    for (int i = 0; i < n; i++) {
        // 计算采样点的坐标
        double x_i = points[0].x + i * (points[1].x - points[0].x) / n;
        double y_i = k * x_i + b;

        x[i] = x_i;
        y[i] = y_i;
        // 在图像上绘制采样点
        circle(canvas_image, Point2f(x_i, y_i), 2, Scalar(255, 0, 0), -1);
    }

    // 停止计时
    Stop_Timer();
    // 记录对应步骤的运行时间
    time_map["Calculate_Sample_Points"] = run_time;

    // 定时开始
    Start_Timer();

    // 对采样点的灰度值进行平滑处理
    // 沿着垂直采样线的方向，进一步采样，计算灰度值的平均值
    vector<double> gray(n);  // 灰度值
    cout << "n:" << n << endl;
    for (int i = 0; i < n; i++) {
        // 垂直方向采样点的灰度值列表
        vector<double> gray_vertical_sample;

        // 垂直直线方向，以采样点为中心，左右各取5个点，计算灰度值的平均值
        // 计算垂直方向的点的坐标
        int vertical_sample_num = 11;
        vector<double> x_vertical_sample(vertical_sample_num);
        vector<double> y_vertical_sample(vertical_sample_num);
        for (int j = 0; j < vertical_sample_num; j++) {
            // 使用前述计算的斜率，计算垂直于采样线方向的补充采样点
            // 斜率越接近0，垂直方向的采样点越接近水平方向
            if (k < -1 || k > 1) {
                x_vertical_sample[j] =
                    x[i] + (j - (vertical_sample_num - 1) / 2);
                y_vertical_sample[j] =
                    (x_vertical_sample[j] - x[i]) * (-1 / k) + y[i];
            } else {
                y_vertical_sample[j] =
                    y[i] + (j - (vertical_sample_num - 1) / 2);
                x_vertical_sample[j] =
                    (y_vertical_sample[j] - y[i]) / (-1 / k) + x[i];
            }
            // 边界检查
            if (x_vertical_sample[j] < 0 ||
                x_vertical_sample[j] >= gray_image.cols ||
                y_vertical_sample[j] < 0 ||
                y_vertical_sample[j] >= gray_image.rows) {
                continue;
            }
            // 在图像上绘制垂直方向采样点
            circle(canvas_image,
                   Point2f(x_vertical_sample[j], y_vertical_sample[j]), 0,
                   Scalar(0, 255, 0), -1);

            // 计算垂直方向采样点的灰度值
            gray_vertical_sample.push_back(gray_image.at<uchar>(
                y_vertical_sample[j], x_vertical_sample[j]));
        }
        // 计算均值作为灰度值
        for (int j = 0; j < gray_vertical_sample.size(); j++) {
            gray[i] += gray_vertical_sample[j];
        }
        gray[i] /= vertical_sample_num;
    }

    // 对灰度值进行平滑处理
    // 采用中值滤波
    vector<double> gray_smooth(n);
    for (int i = 0; i < n; i++) {
        // 边界检查
        if (i < 2 || i > n - 3) {
            gray_smooth[i] = gray[i];
            continue;
        }
        // 中值滤波
        vector<double> gray_sample;
        for (int j = i - 2; j <= i + 2; j++) {
            gray_sample.push_back(gray[j]);
        }
        sort(gray_sample.begin(), gray_sample.end());
        gray_smooth[i] = gray_sample[2];
    }

    // 停止计时
    Stop_Timer();
    // 记录对应步骤的运行时间
    time_map["Smooth_Gray"] = run_time;

    // 定时开始
    Start_Timer();

    // 寻找边缘点，即灰度值发生跳变的点
    // 采用二阶差分的方法
    vector<double> gray_diff(n);
    for (int i = 0; i < n; i++) {
        // 边界检查
        if (i == 0) {
            gray_diff[i] = gray_smooth[i + 1] - gray_smooth[i];
            continue;
        }
        if (i == n - 1) {
            gray_diff[i] = gray_smooth[i] - gray_smooth[i - 1];
            continue;
        }
        gray_diff[i] =
            gray_smooth[i + 1] - 2 * gray_smooth[i] + gray_smooth[i - 1];

        // [DEBUG]
        cout << "gray_diff[" << i << "]:" << gray_diff[i] << endl;
    }

    // 寻找边缘点
    vector<Point2f> edge_points;
    vector<int> index_edge_points;
    for (int i = 0; i < n; i++) {
        // 边界检查
        if (i == 0 || i == n - 1) {
            continue;
        }
        // 判断是否为边缘点
        // 大于规定阈值，且与前一个点的符号不同
        if (abs(gray_diff[i]) < 30) {
            continue;
        }
        // 若前一个点为边缘点，则跳过
        if (index_edge_points.size() > 0 && i - index_edge_points.back() < 5) {
            continue;
        }
        if (gray_diff[i] * gray_diff[i - 1] < 0 ||
            gray_diff[i] * gray_diff[i + 1] < 0) {
            // 计算边缘点的坐标
            double x_edge = x[i];
            double y_edge = k * x_edge + b;
            edge_points.push_back(Point2f(x_edge, y_edge));
            index_edge_points.push_back(i);

            // [DEBUG]
            cout << "index of edge point:" << i << endl;
        }
    }

    // 定时结束
    Stop_Timer();
    // 记录对应步骤的运行时间
    time_map["Find_Edge"] = run_time;

    // 对边缘点进行亚像素精确化
    // 定时开始
    Start_Timer();

    // 若边缘点个数小于2，则无法进行亚像素精确化
    if (edge_points.size() < 2) {
        cout << "edge points number is less than 2" << endl;
        return;
    } else {
        // 亚像素精确化
        cornerSubPix(
            gray_image, edge_points, Size(5, 5), Size(-1, -1),
            TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 40, 0.001));
    }

    // 定时结束
    Stop_Timer();
    // 记录对应步骤的运行时间
    time_map["Subpixel"] = run_time;

    // 边缘点可能不止两个，输出边缘点的个数
    // 在information_image上显示边缘点的个数
    // 并在canvas_image上显示边缘点，标注边缘点的坐标和彼此之间的距离
    putText(information_image,
            "edge points number: " + to_string(edge_points.size()),
            Point2f(0, 220), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255),
            1);
    for (int i = 0; i < edge_points.size(); i++) {
        // 在canvas_image上显示边缘点
        circle(canvas_image, edge_points[i], 5, Scalar(0, 255, 255), -1);
        // 并显示边缘点的编号以及彼此之间的距离
        if (i == 0) {
            putText(canvas_image, "1",
                    Point2f(edge_points[i].x - 10, edge_points[i].y - 10),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 128, 255), 2);
        } else {
            putText(canvas_image, to_string(i + 1),
                    Point2f(edge_points[i].x - 10, edge_points[i].y - 10),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 128, 255), 2);
            // 计算边缘点之间的距离
            double distance =
                sqrt(pow(edge_points[i].x - edge_points[i - 1].x, 2) +
                     pow(edge_points[i].y - edge_points[i - 1].y, 2));
            string distance_str;
            ostringstream ss;
            ss << std::fixed << std::setprecision(2) << distance;
            distance_str = ss.str();
            // 为了防止距离文字重叠，对距离文字的位置进行微调
            // 单数编号的边缘点，距离文字在边缘点的上方
            // 双数编号的边缘点，距离文字在边缘点的下方
            if (i % 2 == 0) {
                putText(
                    canvas_image, distance_str,
                    Point2f((edge_points[i].x + edge_points[i - 1].x) / 2,
                            (edge_points[i].y + edge_points[i - 1].y) / 2 - 30),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
            } else {
                putText(
                    canvas_image, distance_str,
                    Point2f((edge_points[i].x + edge_points[i - 1].x) / 2,
                            (edge_points[i].y + edge_points[i - 1].y) / 2 + 30),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
            }
            // 同时在图像左上角绘制边缘点之间的距离
            // 将背景设置为黑色
            rectangle(canvas_image, Point(0, 25 * (i-1)), Point(250, 25 * i),
                      Scalar(0, 0, 0), -1);
            putText(canvas_image,
                    "distance " + to_string(i) + "-" + to_string(i + 1) + ": " +
                        distance_str,
                    Point2f(0, i * 25 -3), FONT_HERSHEY_SIMPLEX, 0.5,
                    Scalar(0, 128, 255));
        }

        // 在information_image上显示边缘点的坐标
        putText(information_image,
                "edge point " + to_string(i) + ": (" +
                    to_string(edge_points[i].x) + ", " +
                    to_string(edge_points[i].y) + ")",
                Point2f(0, 240 + i * 20), FONT_HERSHEY_SIMPLEX, 0.5,
                Scalar(255, 255, 255), 1);
    }

    imshow("canvas", canvas_image);
    imshow("information", information_image);
}

void Caliper::Loop() {
    // 更新显示图像
    Upgrade_Image();
    // 根据用户按下键盘的不同按键，进行不同的操作
    int key = waitKey(10);
    switch (key) {
        // 按下空格键，开始测量
        case 32:
            // [DEBUG]
            if (debug == true) {
                cout << "space" << endl;
            }
            // 在information_image上显示开始进行1D测量
            putText(information_image, "1D measurement result", Point2f(80, 160),
                    FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);
            // 提示按下任意键可以重新选择测量区域
            putText(information_image, "press any key to select again",
                    Point2f(20, 190), FONT_HERSHEY_SIMPLEX, 0.7,
                    Scalar(255, 255, 255), 2, 1);
            imshow("information", information_image);

            // 开始测量
            OneD_Measure();

            // 显示耗时分析
            Analyze_Time();

            // 堵塞，等待用户输入
            waitKey(0);
            break;

        // 按下ESC键，退出程序
        case 27:
            // [DEBUG]
            if (debug == true) {
                cout << "ESC" << endl;
            }
            destroyAllWindows();
            // 退出程序
            exit(0);
            break;
        default:
            break;
    }
}

void Caliper::Start_Timer() {
    // start_time = chrono::steady_clock::now();
    start_time = chrono::high_resolution_clock::now();
}

void Caliper::Stop_Timer() {
    // stop_time = chrono::steady_clock::now();
    stop_time = chrono::high_resolution_clock::now();

    // 计算程序运行时间，单位为ns
    auto duration =
        chrono::duration_cast<chrono::nanoseconds>(stop_time - start_time);
    run_time = static_cast<double>(duration.count()) / 1e6;
    // 输出程序运行时间
    cout << fixed << setprecision(2) << "duration: " << run_time << "ms"
         << endl;
}

void Caliper::Analyze_Time() {
    // 在Performance_chart上以条状图绘制运行时间
    // 并在图像上显示各个函数的运行时间，单位为毫秒

    // 根据时间对的长度，确定performance_chart的大小
    // 设置performance_chart的背景为白色
    Mat performance_chart = Mat::zeros(time_map.size() * 60 + 60, 400, CV_8UC3);
    rectangle(performance_chart, Point(0, 0),
        Point(performance_chart.cols, performance_chart.rows),
        Scalar(255, 255, 255), -1);

    // 由于各个步骤的运行时间差异较大，为了保证条状图的可视化效果
    // 需要对时间进行归一化处理，使得时间最长的步骤的展示长度为380
    // 创建一个数组，用于存储归一化后的时间
    vector<double> time_normalized(time_map.size());
    // 找到时间最长的步骤
    double max_time = 0;
    for (auto iter = time_map.begin(); iter != time_map.end(); iter++) {
        if (iter->second > max_time) {
            max_time = iter->second;
        }
    }
    // 归一化
    int i = 0;
    for (auto iter = time_map.begin(); iter != time_map.end(); iter++) {
        time_normalized[i++] = iter->second / max_time * 380;
    }

    // 在performance_chart上绘制条状图
    i = 0;
    for (auto iter = time_map.begin(); iter != time_map.end(); iter++) {
        // 为了保证条状图不会遮挡文字，令条状图的绘制起点和文字错开
        rectangle(performance_chart, Point(0, 50 + i * 60),
                  Point(time_normalized[i], 80 + i * 60), Scalar(0, 0, 255),
                  -1);
        putText(performance_chart, iter->first + ": " + to_string(iter->second),
                Point(0, 40 + i * 60), FONT_HERSHEY_SIMPLEX, 0.5,
                Scalar(0, 0, 0), 1);
        i++;
    }
    imshow("performance_chart", performance_chart);
}

// 鼠标回调函数，用于获取用户点击的两个点
void onMouse(int event, int x, int y, int flags, void* userdata) {
    // 鼠标左键按下，记录第一个点
    if (event == EVENT_FLAG_LBUTTON) {
        Caliper* myclass = static_cast<Caliper*>(userdata);
        if (myclass) {
            myclass->points[0] = Point2f(x, y);
        }
    }
    // 鼠标右键按下，记录第二个点
    else if (event == EVENT_FLAG_RBUTTON) {
        Caliper* myclass = static_cast<Caliper*>(userdata);
        if (myclass) {
            myclass->points[1] = Point2f(x, y);
        }
    }
}

int main() {
    // 创建卡尺对象
    Caliper caliper(
        "E:\\作业\\大三下\\机器视觉\\课程设计\\课程设计\\课题6\\noised_0.3."
        "bmp");
    // 初始化卡尺
    caliper.Init();
    // 设置debug
    caliper.Set_Debug(false);
    // 循环处理
    while (true) {
        caliper.Loop();
    }

    return 0;
}