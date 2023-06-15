
#ifndef DBImageProcess_h
#define DBImageProcess_h

#include <iostream>
#include <stdio.h>

#include "cxcore.h"		//OpenCV头文件
#include "highgui.h"
#include "cv.h"    



#define IMAGEWIDTH	    640
#define IMAGEHEIGHT		480

#define LINELENGTH		18  //十字定标线长度
#define NCCTHRESHOLD8	0.5
#define LEVEL3POINT		12
#define RELEVEL3POINT	4*LEVEL3POINT

#define MUTILEVEL3POINT	60
#define MUTISELECT		MUTILEVEL3POINT/2

// = 后续添加的额外注释形式将以此为准 = //

class CHTimer
{
public:
	CHTimer();
	~CHTimer();
	void StartTime();
	void EndTime();
	double GetTime();
protected:
    
private:
	LONGLONG  m_lStartTime;
	LONGLONG  m_lEndTime;
	LONGLONG  m_lPersecond;
	double	  m_fTime;
};

class CDBImageProcess    //图像处理类
{
private:
	struct SelectPoint 
	{
		double score;   //匹配度NCC
		int    width;   //坐标值
		int    height;
	};
public:   //data
	SelectPoint m_MaxNccPoint;				// = 用于存储最大NCC值的点 = //
	SelectPoint m_SelectNcc[MUTISELECT];	// = 用于存储最大NCC值的点 = //
	int m_nMutiNumber;
	IplImage* m_pTemplate;
private:
	double m_Locoftarget[2];				// = 用于存储最大NCC值的点 = //
	double m_Ncc;							// 归一化相关系数
	int m_PyramidKey;						// = 好像没有用到 = //
	int m_nTemplateSize;	//1/8 1/4  1/2 模板图像像素点数
	int m_nTemplateSize2;	// 1/16 模板图像像素点数
	int	m_nTemplateSize4;
	int m_nTemplateSize8;
	int m_nTemplateSize16;
	// ======================= //
	// 扩展到8层NCC模板匹配
	int m_nTemplateSize32;
	int m_nTemplateSize64;
	int m_nTemplateSize128;
	int m_nTemplateSize256;
	// ======================= //
	
	//原模板图像的参数
	double m_nSquareSumOfTemplate;
	unsigned int m_nSumGrayTemplate;
	double 	m_SumSquareTemplate;//模板灰度和的平方
	double 	m_NSquareSumTemplate;//模板灰度的平方和乘以N
	//1/2模板图像参数
	unsigned int m_nSumGrayTemplate2;
	double 	m_SumSquareTemplate2;//模板灰度和的平方
	double 	m_NSquareSumTemplate2;//模板灰度的平方和乘以N
	//1/4模板图像参数
	unsigned int m_nSumGrayTemplate4;
	double 	m_SumSquareTemplate4;//模板灰度和的平方
	double 	m_NSquareSumTemplate4;//模板灰度的平方和乘以N
	// *********************** //
	// 1/8模板图像参数
	unsigned int m_nSumGrayTemplate8;
	double 	m_SumSquareTemplate8;//模板灰度和的平方
	double 	m_NSquareSumTemplate8;//模板灰度的平方和乘以N
	// 1/16模板图像参数
	unsigned int m_nSumGrayTemplate16;
	double 	m_SumSquareTemplate16;//模板灰度和的平方
	double 	m_NSquareSumTemplate16;//模板灰度的平方和乘以N
	// ======================= //
	// 扩展到8层NCC模板匹配
	// 1/32模板图像参数
	unsigned int m_nSumGrayTemplate32;
	double 	m_SumSquareTemplate32;//模板灰度和的平方
	double 	m_NSquareSumTemplate32;//模板灰度的平方和乘以N
	// 1/64模板图像参数
	unsigned int m_nSumGrayTemplate64;
	double 	m_SumSquareTemplate64;//模板灰度和的平方
	double 	m_NSquareSumTemplate64;//模板灰度的平方和乘以N
	// 1/128模板图像参数
	unsigned int m_nSumGrayTemplate128;
	double 	m_SumSquareTemplate128;//模板灰度和的平方
	double 	m_NSquareSumTemplate128;//模板灰度的平方和乘以N
	// 1/256模板图像参数
	unsigned int m_nSumGrayTemplate256;
	double 	m_SumSquareTemplate256;//模板灰度和的平方
	double 	m_NSquareSumTemplate256;//模板灰度的平方和乘以N
	// ======================= //

	CvMat* pMatImage2;		//目标图像的1/2细分		
	CvMat* pMatImage4;		//目标图像的1/4细分	
	CvMat* pMatFlag4;
	CvMat* pMatImage8;
	CvMat* pMatFlag8;
	CvMat* pMatImage16;
	CvMat* pMatFlag16;
	// ======================= //
	// 扩展到8层NCC模板匹配
	CvMat* pMatImage32;
	CvMat* pMatFlag32;
	CvMat* pMatImage64;
	CvMat* pMatFlag64;
	CvMat* pMatImage128;
	CvMat* pMatFlag128;
	CvMat* pMatImage256;
	// CvMat* pMatFlag256;
	// ======================= //

	CvMat* pMatTemplate;   //模板原图像矩阵
	CvMat* pMatTemplate2;   //模板图像的1/2细分
	CvMat* pMatTemplate4;
	CvMat* pMatTemplate8; 
	CvMat* pMatTemplate16;
	// ======================= //
	// 扩展到8层NCC模板匹配
	CvMat* pMatTemplate32;
	CvMat* pMatTemplate64;
	CvMat* pMatTemplate128;
	CvMat* pMatTemplate256;
	// ======================= //

public:   //function
	CDBImageProcess();
	~CDBImageProcess();
	inline int SquareFunc(unsigned char x){ return (int)x*x;}
	inline int SquareFunc(int x){ return x*x;}       //求整数平方的内联函数
	inline unsigned int SquareFunc(unsigned int x){ return x*x;}
	inline double SquareFunc(double x){ return x*x;};//求double型数平方的内联函数
	inline double GetNcc(){ return m_Ncc;};
	
	//!分配内存空间。在使用类对象时，需先调用这个成员函数
	bool AllocateMemory( int nImageWidth,      //目标图像的宽
						 int nImageHeight,     //目标图像高
						 int nTemplateWidth,   //模板图像宽
						 int nTemplateHeight); //模板图像高
	// = 金字塔降采样 = //
    void PyramidDown(CvMat* src,CvMat* dst,IplImage* pImg);
	// = 加载模板 = //
	bool LoadTemplate(char* szFile);
	bool SaveTemplate(int nWidth,int nHeight,int top,int left,unsigned char* pbImgData);
	//!模板预处理
	void PreProcessTemplate( IplImage* pTemplate); //模板图像指针

	void SetMatchROI();	//设置匹配感兴趣区域 (未完成!!!) // = ok = //
	
	// = 单一模板匹配 = //
	bool SingleTemplateMatch( char* ImageData, 
						      int ImageWidth,
						      int ImageHeight,
							  int widthstep,
							  int ImagenChannels);
	// = 多模板匹配 = // 
	// = 实际没有用到，可以参照TemplateMatchDlg.cpp文件中对于不同方法的模板匹配调用部分的代码 = //
	bool MutiTemplateMatch(   char * ImageData,
							  int ImageWidth,
							  int ImageHeight,
							  int widthstep,
							  int ImagenChannels,
							  int nMatchNumber);
	// = 绘制匹配结果 = //
	void DrawPattern(IplImage* pTarget,IplImage* pTemplate,SelectPoint MaxNccPoint);
	// = 绘制多模板匹配结果 = //
	void DrawMutiPattern(IplImage* pTarget,IplImage* pTemplate);
	// = 获取匹配位置 = //
	double* GetTargetLocation();

private:
	bool m_bImageMemoryReady;	// = 图像内存是否已经分配 = // // = 但是在DBImageProcess.cpp中没有使用 = //
	// = 预分配内存 = //
	CvMat* PreAllocMemory(int nSrcWidth,int nSrcHeight);

};

#endif