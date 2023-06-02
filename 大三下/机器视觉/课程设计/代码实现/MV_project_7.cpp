// ***************************************	//
// 机器视觉-课题7
// 黄继凡
// 2023/5/12

// ***************************************	//
// 项目说明
// 图像状态分类判断
// A状态不做处理
// B状态找到圆柱末端，设置对应ROI
// 进行裁剪和保存
// ROI的长宽作为参数输入

// ***************************************	//

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

// 图像状态枚举类型
// C表明暂时未知状态
enum Image_Status { A, B, C };

// 图像处理类，保存本次任务中需要用到的相关存储变量，以及相关函数
class Image_processor {
   public:
    // 构造函数，填入待处理图像路径
    Image_processor(string image_path);
    // 析构函数
    ~Image_processor(){};

    // 设置debug模式
    void Set_Debug(bool debug_mode) { debug = debug_mode; }
    // 设置图像压缩比例
    void Set_Image_Scale(double scale) { image_scale = scale; }
    // 用户输入ROI长度、宽度
    void Set_ROI_Length_And_Width() {
        cout << "Please input ROI length: ";
        cin >> ROI_length;
        cout << "Please input ROI width: ";
        cin >> ROI_width;
    }
    // 输入图像保存路径
    void Set_Save_Path(string path) { save_path = path; }
    // 预处理图像，去除噪声并增强图像对比度
    void Preprocess_Image();
    // 检测图像中的轮廓点集
    void Detect_Contours();
    // 判断图像状态
    void Classify_Image();
    // 获得图像状态
    Image_Status Get_Image_Status() { return image_status; }
    // 找出圆柱末端中心点坐标
    void Find_Cylinder_End();
    // 生成ROI框，用于裁剪图像
    void Generate_ROI();
    // 保存图像
    void Save_Image();

    // 开始计时
    void Start_Timer();
    // 结束计时，以毫秒为单位输出程序运行时间
    void Stop_Timer();
    // 根据程序各个步骤运行耗时，进行图像分析显示
    void Analyze_Time();

   private:
    Mat image;       // 存储待处理的图像
    Mat gray_image;  // 存储灰度图像
    Mat ROI_image;   // 存储裁剪后的图像

    double ROI_width = 0;    // ROI宽度
    double ROI_length = 0;   // ROI长度
    double image_scale = 1;  // 图像压缩比例

    Mat information_image;  // 存储显示测量结果的图像
    Mat Performance_Chart;  // 存储程序运行时间分析图像

    vector<Vec4i> hierarchy;            // 存储轮廓的层级信息
    vector<vector<Point>> contours;     // 存储检测到的轮廓点集
    vector<double> contour_areas;       // 存储轮廓面积
    vector<double> contour_perimeters;  // 存储轮廓周长
    double max_area = 0;                // 最大轮廓面积
    double max_perimeter = 0;           // 最大轮廓周长

    // 面积和周长的比值，用于判断图像状态
    array<double, 2> area_perimeter_ratio = {85, 95};
    Image_Status image_status = C;  // 图像状态
    // 用于存储圆柱末端中心点坐标
    Point cylinder_end_center;
    // 用于存储图像保存路径
    string save_path;

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
    // 压缩图像
    resize(image, image, Size(), image_scale, image_scale);
    // 显示图像
    imshow("Original Image", image);
    // 将图像转换为灰度图像
    cvtColor(image, gray_image, COLOR_BGR2GRAY);
    // 对灰度图像进行高斯滤波
    GaussianBlur(gray_image, gray_image, Size(3, 3), 0, 0);
    // 对灰度图像进行直方图均衡化
    equalizeHist(gray_image, gray_image);
    // 显示均衡化后的图像
    imshow("Equalized Image", gray_image);
    // 对灰度图像进行二值化
    threshold(gray_image, gray_image, 0, 255, THRESH_BINARY | THRESH_OTSU);
    // 对灰度图像进行闭运算
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(gray_image, gray_image, MORPH_CLOSE, element);
    // 膨胀图像
    dilate(gray_image, gray_image, element);

    imshow("Preprocessed Image", gray_image);

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
    // 保存轮廓面积
    for (int i = 0; i < contours.size(); i++) {
        contour_areas.push_back(contourArea(contours[i]));
        max_area = max(max_area, contourArea(contours[i]));
    }
    // 保存轮廓周长
    for (int i = 0; i < contours.size(); i++) {
        contour_perimeters.push_back(arcLength(contours[i], true));
        max_perimeter = max(max_perimeter, arcLength(contours[i], true));
    }
    // 输出轮廓信息
    if (debug) {
        // 输出轮廓数目
        cout << "Number of contours detected: " << contours.size() << endl;
        // 输出轮廓面积
        for (int i = 0; i < contours.size(); i++) {
            cout << "Area of contour " << i << ": " << contourArea(contours[i])
                 << endl;
        }
        // 输出轮廓周长
        for (int i = 0; i < contours.size(); i++) {
            cout << "Perimeter of contour " << i << ": "
                 << arcLength(contours[i], true) << endl;
        }
    }

    // 结束计时
    Stop_Timer();
    // 记录运行时间
    time_map["Detect_Contours"] = run_time;
}

void Image_processor::Classify_Image() {
    // 开始计时
    Start_Timer();

    // [DEBUG]
    if (debug) {
        cout << endl;
        cout << "[DEBUG] Classifying image..." << endl;
    }

    // 判断图像状态
    double ratio = max_area / max_perimeter;
    if (ratio < area_perimeter_ratio[0] || ratio > area_perimeter_ratio[1]) {
        // 图像状态为A
        image_status = A;
    } else {
        // 图像状态为B
        image_status = B;
    }

    // 输出图像状态
    if (debug) {
        cout << "Image status: " << image_status << endl;
    }

    // 结束计时
    Stop_Timer();
    // 记录运行时间
    time_map["Classify_Image"] = run_time;
}

void Image_processor::Find_Cylinder_End() {
    // 开始计时
    Start_Timer();

    // [DEBUG]
    if (debug) {
        cout << endl;
        cout << "[DEBUG] Finding cylinder end..." << endl;
    }

    // 计算图形的一阶矩
    Moments m = moments(gray_image, true);
    // 计算图形的重心
    cylinder_end_center = Point(m.m10 / m.m00, m.m01 / m.m00);
    // 经过对比，可以认为圆柱末端与图形中心非常接近
    // 可以以图形中心为搜索起点
    // 找到周围从上至下的第一个白色像素点y值最大的列
    // 该列的y值即为圆柱末端的y值
    // 对应列的第一个白色像素点的x值即为圆柱末端的x值
    int y = cylinder_end_center.y;
    int x = cylinder_end_center.x;
    // 从左至右搜索,两侧分别搜索50列
    for (int i = 1; i <= 50; i++) {
        // 搜索左侧
        if (x - i >= 0) {
            // 搜索从上至下的第一个白色像素点
            for (int j = 0; j < gray_image.rows; j++) {
                if (gray_image.at<uchar>(j, x - i) == 255) {
                    // 比较是否比原先的y值大
                    if (j > y) {
                        // 更新y值
                        y = j;
                        // 更新圆柱末端坐标
                        cylinder_end_center.x = x - i;
                        cylinder_end_center.y = j;
                    }
                    break;
                }
            }
        }
        // 搜索右侧
        if (x + i < gray_image.cols) {
            // 搜索从上至下的第一个白色像素点
            for (int j = 0; j < gray_image.rows; j++) {
                if (gray_image.at<uchar>(j, x + i) == 255) {
                    // 比较是否比原先的y值大
                    if (j > y) {
                        // 更新y值
                        y = j;
                        // 更新圆柱末端坐标
                        cylinder_end_center.x = x + i;
                        cylinder_end_center.y = j;
                    }
                    break;
                }
            }
        }
    }

    // 输出圆柱末端坐标
    if (debug) {
        cout << "Cylinder end: " << cylinder_end_center << endl;
    }

    // 结束计时
    Stop_Timer();
    // 记录运行时间
    time_map["Find_Cylinder_End"] = run_time;
}

void Image_processor::Generate_ROI() {
    // 开始计时
    Start_Timer();

    // [DEBUG]
    if (debug) {
        cout << endl;
        cout << "[DEBUG] Generating ROI..." << endl;
    }

    // 生成ROI
    // ROI为圆柱末端坐标向上偏移ROI_offset个像素的矩形
    // ROI的宽度为ROI_width
    // ROI的高度为ROI_height
    int ROI_x = cylinder_end_center.x - ROI_width / 2;
    int ROI_y = cylinder_end_center.y - ROI_length / 2;
    Rect ROI = Rect(ROI_x, ROI_y, ROI_width, ROI_length);
    // 从原始图像中提取ROI
    ROI_image = image(ROI);
    // 显示ROI区域图像
    imshow("ROI", ROI_image);

    // 在原始图像上绘制ROI和ROI中心
    circle(image, cylinder_end_center, 2, Scalar(0, 255, 255), 2);
    rectangle(image, ROI, Scalar(0, 0, 255), 2);

    // 显示绘制ROI后的图像
    imshow("Image", image);

    // 输出ROI信息
    if (debug) {
        cout << "ROI: " << ROI << endl;
    }

    // 结束计时
    Stop_Timer();
    // 记录运行时间
    time_map["Generate_ROI"] = run_time;
}

void Image_processor::Save_Image() {
    // 开始计时
    Start_Timer();

    // [DEBUG]
    if (debug) {
        cout << endl;
        cout << "[DEBUG] Saving image..." << endl;
    }

    // 保存图像
    imwrite(save_path + "processed_image.jpg", image);

    // 结束计时
    Stop_Timer();
    // 记录运行时间
    time_map["Save_Image"] = run_time;
}

void Image_processor::Start_Timer() {
    // 记录开始时间
    start_time = chrono::high_resolution_clock::now();
}

void Image_processor::Stop_Timer() {
    // 记录结束时间
    stop_time = chrono::high_resolution_clock::now();
    // 计算运行时间
    auto duration =
        chrono::duration_cast<chrono::nanoseconds>(stop_time - start_time)
            .count();
    run_time = duration / 1e6;
    // 输出运行时间，以毫秒为单位
    cout << fixed << setprecision(2) << "Run time: " << run_time << " ms"
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
        "\\作业\\大三下\\机器视觉\\课程设计\\课程设计\\课题7\\课题7图像\\"
        "2023-02-07 19-59-16-772.bmp");
    // 设置保存图像的文件夹
    processor.Set_Save_Path(
        "E:"
        "\\作业\\大三下\\机器视觉\\课程设计\\课程设计\\课题7\\课题7图像\\proces"
        "sed\\");
    // 开启debug
    processor.Set_Debug(true);
    // 设置压缩比例
    processor.Set_Image_Scale(0.2);
    // 图像预处理
    processor.Preprocess_Image();
    // 设置ROI的长宽
    processor.Set_ROI_Length_And_Width();
    // 轮廓检测
    processor.Detect_Contours();
    // 分类图像
    processor.Classify_Image();

    // 若图像为A状态
    if (processor.Get_Image_Status() == B) {
        // 检测圆柱末端
        processor.Find_Cylinder_End();
        // 生成ROI
        processor.Generate_ROI();
        // 保存图像
        processor.Save_Image();
    } else {
        cout << "Image status is A status" << endl;
    }

    // 分析运行时间
    processor.Analyze_Time();

    waitKey(0);

    return 0;
}