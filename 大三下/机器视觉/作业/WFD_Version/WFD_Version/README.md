机器视觉部分作业可能需要使用Halcon软件。大家可以按需百度安装。

如果你的Halcon许可丢失或者过时，可以每月从这里下载：[lovelyyoshino/Halcon_licenses](https://github.com/lovelyyoshino/Halcon_licenses)

其中Halcon导出的文件我将仅保留程序源文件(.c,.cpp)，生成的Visual Studio项目文件，由于各位的Halcon库安装位置不同，我将不再保存，请各位自行生成并配置。

一些借助Opencv写的算法程序，我也仅保留程序源文件和CMakeLists.txt文件，记得去CMakeLists.txt里改Opencv依赖路径。

注意程序内的图片访问路径，以免找不到图片报错。

课程设计是有多个选题的。我们组当时抽到了“使用8层金字塔加速NCC，以实现检测”这个题目。如果你抽到了其他的题目，或许我们无法提供帮助。