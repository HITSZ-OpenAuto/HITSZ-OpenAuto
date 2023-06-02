// ***************************************	//
// 机器视觉-课题4
// 黄继凡
// 2023/5/12

// ***************************************	//
// 项目说明
// 模板匹配定位火花塞位置
// 并测量火花塞触发器的间隔宽度

// ***************************************	//
// 程序流程
// 输入图片路径后，程序会自动进行图像处理
// 使用NCC模板匹配器进行模板匹配，找到火花塞位置
// 以火花塞位置坐标为起点，向下寻找电极，计算间隙

#include <iostream>

// 包含opencv库，用于图像处理
#include <opencv2/opencv.hpp>
// 包含chrono库，用于计时
#include <chrono>
#include <map>

using namespace cv;
using namespace std;

// 鼠标回调函数，用于获取用户点击的两个点
void onMouse(int event, int x, int y, int flags, void* userdata);

// NCC模板匹配器
class NCC_Matcher {
   public:
    // 构造函数，填入模板图像
    NCC_Matcher() {}
    // 析构函数
    ~NCC_Matcher() {}

    // 设置debug模式
    void Set_Debug(bool debug_mode) { debug = debug_mode; }
    // 设置模板图像
    void Set_Template(Mat input_template_image) {
        input_template_image.copyTo(template_image);
    }
    // 设置待匹配图像
    void Set_Image(Mat image) { image.copyTo(image_to_match); }
    // 设置匹配阈值
    void Set_Threshold(double threshold_value) { threshold = threshold_value; }

    // 模板匹配
    void NCC_Match_Template();
    // 返回匹配结果
    Mat Get_Result() { return result; }
    // 返回匹配位置
    Point2f Get_Match_Location() { return match_location; }
    // 返回匹配值
    int Get_NCC_Value() { return NCC_value; }

   private:
    Mat template_image;  // 存储模板图像
    Mat image_to_match;  // 存储待匹配图像

    double threshold = 0.9;  // 匹配阈值

    Mat result;              // 存储匹配结果
    Point2f match_location;  // 存储匹配位置
    int NCC_value;           // 存储匹配值

    bool debug = false;  // 是否开启debug模式
};

void NCC_Matcher::NCC_Match_Template() {
    // [DEBUG]
    if (debug) {
        cout << endl;
        cout << "[DEBUG] Matching template..." << endl;
    }

    // 进行模板匹配
    matchTemplate(image_to_match, template_image, result, TM_CCORR_NORMED);
    // 归一化匹配结果
    normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());

    // 保存匹配结果
    double minVal, maxVal;
    Point minLoc, maxLoc;
    minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
    // 由于需要获得的是模板中心坐标，因此需要对匹配位置进行修正
    match_location = Point2f(maxLoc.x + template_image.cols / 2,
                             maxLoc.y + template_image.rows / 2);

    NCC_value = maxVal;

    // [DEBUG]
    if (debug) {
        cout << "[DEBUG] Matching finished!" << endl;
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
    // 设置是否令用户自行指定模板
    void Set_Manual(bool manual_mode) { manual = manual_mode; }
    // 手动设置模板区域，生成模板图像
    void Generate_Template();
    // 读取模板图像
    void Read_Template(string template_path) {
        template_image = imread(template_path);
    }
    // 进行模板匹配，确定火花塞位置
    void Match_Template();
    // 以火花塞位置为起点，寻找电极，计算间隙
    void Cacularate_Gap();

    // 开始计时
    void Start_Timer();
    // 结束计时，以毫秒为单位输出程序运行时间
    void Stop_Timer();
    // 根据程序各个步骤运行耗时，进行图像分析显示
    void Analyze_Time();
    // 存储两个点，用于手动指定模板区域
    Point2f points[2] = {Point2f(0, 0), Point2f(0, 0)};

   private:
    Mat image;           // 存储待处理的图像
    Mat gray_image;      // 存储灰度图像
    Mat template_image;  // 存储模板图像
    Mat canvas;          // 存储绘制图像

    Mat information_image;  // 存储显示测量结果的图像
    Mat Performance_Chart;  // 存储程序运行时间分析图

    NCC_Matcher matcher;  // 模板匹配器

    // 计时器，用于计算程序运行时间
    chrono::high_resolution_clock::time_point start_time;
    chrono::high_resolution_clock::time_point stop_time;
    double run_time = 0;
    // 时间键值对，用于存储各个步骤运行时间
    map<string, double> time_map;

    bool debug = false;   // 是否开启debug模式
    bool manual = false;  // 是否令用户自行指定模板
};

Image_processor::Image_processor(string image_path) {
    // [DEBUG]
    if (debug) {
        cout << endl;
        cout << "[DEBUG] Initializing image processor..." << endl;
    }

    // 读取图像
    image = imread(image_path);
    // 显示图像
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

    // [DEBUG]
    if (debug) {
        cout << "[DEBUG] Preprocessing finished!" << endl;
        // 显示原始图像
        imshow("Original_Image", image);
        // 显示预处理后的图像
        imshow("Preprocessed_Image", gray_image);
    }
}

void Image_processor::Generate_Template() {
    // 创建原始图像的克隆，用于显示模板区域
    Mat image_clone = image.clone();
    // 显示图像
    imshow("select_template", image_clone);
    // 设置回调函数，用于手动指定模板区域
    setMouseCallback("select_template", onMouse, this);

    // 等待用户指定模板区域
    while (1) {
        image_clone = image.clone();
        // 画出两点
        circle(image_clone, points[0], 2, Scalar(0, 0, 255), 2, 8, 0);
        circle(image_clone, points[1], 2, Scalar(0, 0, 255), 2, 8, 0);
        // 画出矩形
        rectangle(image_clone, points[0], points[1], Scalar(0, 0, 255), 2, 8,
                  0);
        // 显示图像
        imshow("select_template", image_clone);
        // 等待按键
        int key = waitKey(10);
        // 按下ESC键，退出循环
        if (key == 27) {
            // 保存模板图像
            template_image = image(Rect(points[0], points[1]));
            imwrite("template.jpg", template_image);
            break;
        }
    }

    // [DEBUG]
    if (debug) {
        cout << endl;
        cout << "[DEBUG] Generating template..." << endl;
    }
}

void Image_processor::Match_Template() {
    if (manual == true) {
        // [DEBUG]
        if (debug) {
            cout << endl;
            cout << "[DEBUG] Please select the template in the image..."
                 << endl;
        }

        // 生成模板
        Generate_Template();
    } else {
        // [DEBUG]
        if (debug) {
            cout << endl;
            cout << "[DEBUG] Loading template..." << endl;
        }
    }

    // [DEBUG]
    if (debug) {
        // 显示模板图像
        imshow("Template_Image", template_image);
    }

    // 开始计时
    Start_Timer();

    // [DEBUG]
    if (debug) {
        cout << endl;
        cout << "[DEBUG] Matching template..." << endl;
    }
    matcher.Set_Image(image);
    matcher.Set_Template(template_image);
    matcher.Set_Debug(debug);
    matcher.NCC_Match_Template();

    // 结束计时
    Stop_Timer();
    // 记录运行时间
    time_map["Match_Template"] = run_time;

    // 显示匹配结果
    cout << "Matching result: " << matcher.Get_NCC_Value() << endl;
    // 显示匹配结果
    canvas = image.clone();
    cout << "Matching location: " << matcher.Get_Match_Location() << endl;
    circle(canvas, matcher.Get_Match_Location(), 2, Scalar(0, 0, 255), 2, 8, 0);
    imshow("Matched_Image", canvas);
}

void Image_processor::Cacularate_Gap() {
    // 开始计时
    Start_Timer();

    // [DEBUG]
    if (debug) {
        cout << endl;
        cout << "[DEBUG] Cacularating gap..." << endl;
    }

    // 通过对二值化处理后的灰度图，计算缝隙宽度
    Point2f match_location = matcher.Get_Match_Location();
    // 由匹配点的位置开始，从上向下扫描
    // 记录第一次遇到白色像素的位置
    // 和在遇到白色像素之前遇到的第一个黑色像素的位置
    int top = 0, bottom = 0;
    for (int i = match_location.y; i < gray_image.rows; i++) {
        // 遇到白色像素
        if (gray_image.at<uchar>(i, match_location.x) == 255) {
            // 记录第一次遇到白色像素的位置
            if (top == 0) {
                top = i;
            }
        }
        // 遇到黑色像素
        else {
            // 记录在遇到白色像素之前遇到的第一个黑色像素的位置
            if (top != 0) {
                bottom = i;
                break;
            }
        }
    }
    // 计算缝隙宽度
    int gap = bottom - top;

    // 显示缝隙宽度，并在图像上画出缝隙，用于检查
    cout << "Gap: " << gap << endl;

    // 结束计时
    Stop_Timer();
    // 记录运行时间
    time_map["Cacularate_Gap"] = run_time;

    // 根据模板图像的信息，画出模板区域
    // 其中match_location为模板的中心位置
    Point2f temp_points[2];
    temp_points[0] = Point(match_location.x - template_image.cols / 2,
                           match_location.y - template_image.rows / 2);
    temp_points[1] = Point(match_location.x + template_image.cols / 2,
                           match_location.y + template_image.rows / 2);
    rectangle(canvas, temp_points[0], temp_points[1], Scalar(255, 0, 0), 2, 8,
              0);
    // 画出缝隙
    line(canvas, Point(match_location.x, top), Point(match_location.x, bottom),
         Scalar(0, 255, 255), 2, 8, 0);
    // 显示缝隙宽度
    putText(canvas, to_string(gap), Point(match_location.x, top - 10),
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8, false);
    imshow("Gap_Image", canvas);
}

void Image_processor::Start_Timer() {
    start_time = chrono::high_resolution_clock::now();
}

void Image_processor::Stop_Timer() {
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

void onMouse(int event, int x, int y, int flags, void* userdata) {
    // 鼠标左键按下，记录第一个点
    if (event == EVENT_FLAG_LBUTTON) {
        Image_processor* myclass = static_cast<Image_processor*>(userdata);
        if (myclass) {
            myclass->points[0] = Point2f(x, y);
        }
    }
    // 鼠标右键按下，记录第二个点
    else if (event == EVENT_FLAG_RBUTTON) {
        Image_processor* myclass = static_cast<Image_processor*>(userdata);
        if (myclass) {
            myclass->points[1] = Point2f(x, y);
        }
    }
}

int main() {
    // 创建图像处理器对象
    Image_processor processor(
        "E:\\作业\\大三下\\机器视觉\\课程设计\\课程设计\\课题4\\spark_plug_01."
        "png");
    // 开启debug
    processor.Set_Debug(true);

    // 设置手动模式
    // processor.Set_Manual(true);
    processor.Set_Manual(false);
    // 读取模板
    processor.Read_Template(
        "E:\\作业\\大三下\\机器视觉\\课程设计\\课程设计\\课题4\\template.jpg");

    // 图像预处理
    processor.Preprocess_Image();
    // 模板匹配
    processor.Match_Template();
    // 计算缝隙宽度
    processor.Cacularate_Gap();
    // 分析运行时间
    processor.Analyze_Time();

    waitKey(0);

    return 0;
}