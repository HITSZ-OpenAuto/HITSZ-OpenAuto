#include "StdAfx.h"
#include "cxcore.h"		//OpenCV头文件
#include "highgui.h"
#include "cv.h"  
#include "CMatrix.h"

#include "DBImageProcess.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


CDBImageProcess::CDBImageProcess()
{
	m_Ncc=0.0;
	m_nMutiNumber=0;
	m_MaxNccPoint.height=0;
	m_MaxNccPoint.width=0;
	m_MaxNccPoint.score=0.0;

	m_nTemplateSize=0;
	m_nTemplateSize2=0;
	m_nTemplateSize4=0;
	m_nTemplateSize8=0;
	m_nTemplateSize16=0;
	// ======================= //
	// 扩展到8层NCC模板匹配
	m_nTemplateSize32=0;
	m_nTemplateSize64=0;
	m_nTemplateSize128=0;
	m_nTemplateSize256=0;
	// ======================= //

	m_nSumGrayTemplate=0;
	m_nSquareSumOfTemplate=0;
	m_SumSquareTemplate=0;
	m_NSquareSumTemplate=0;

	m_nSumGrayTemplate2=0;
	m_SumSquareTemplate2=0;
	m_NSquareSumTemplate2=0;

	m_nSumGrayTemplate4=0;
	m_SumSquareTemplate4=0;
	m_NSquareSumTemplate4=0;
	// ======================= //
	// 扩展到8层NCC模板匹配
	m_nSumGrayTemplate8=0;
	m_SumSquareTemplate8=0;
	m_NSquareSumTemplate8=0;

	m_nSumGrayTemplate16=0;
	m_SumSquareTemplate16=0;
	m_NSquareSumTemplate16=0;

	m_nSumGrayTemplate32=0;
	m_SumSquareTemplate32=0;
	m_NSquareSumTemplate32=0;

	m_nSumGrayTemplate64=0;
	m_SumSquareTemplate64=0;
	m_NSquareSumTemplate64=0;

	m_nSumGrayTemplate128=0;
	m_SumSquareTemplate128=0;
	m_NSquareSumTemplate128=0;

	m_nSumGrayTemplate256=0;
	m_SumSquareTemplate256=0;
	m_NSquareSumTemplate256=0;
	// ======================= //

	m_Locoftarget[0] = 0;
	m_Locoftarget[1] = 0;

	pMatImage2=NULL;
	pMatImage4=NULL;
	pMatImage8=NULL;
	pMatImage16=NULL;
	pMatTemplate=NULL;
	pMatTemplate2=NULL;
	pMatTemplate4=NULL;
	pMatTemplate8=NULL;
	pMatTemplate16=NULL;
	pMatFlag4=NULL;
	pMatFlag8=NULL;
	// ======================= //
	// 扩展到8层NCC模板匹配
	pMatImage32=NULL;
	pMatImage64=NULL;
	pMatImage128=NULL;
	pMatImage256=NULL;

	pMatTemplate32=NULL;
	pMatTemplate64=NULL;
	pMatTemplate128=NULL;
	pMatTemplate256=NULL;

	pMatFlag16=NULL;
	pMatFlag32=NULL;
	pMatFlag64=NULL;
	pMatFlag128=NULL;
	// ======================= //
	
	m_pTemplate=NULL;
	//for(int i=0;i<9;i++)
	//	m_fNccArray[i]=0.0;
}

CDBImageProcess::~CDBImageProcess()
{
	//Release matrix memory
	cvReleaseImage(&m_pTemplate);
	cvReleaseMat(&pMatImage2);
	cvReleaseMat(&pMatImage4);
	cvReleaseMat(&pMatImage8);
	cvReleaseMat(&pMatImage16);
	// ======================= //
	// 扩展到8层NCC模板匹配
	cvReleaseMat(&pMatImage32);
	cvReleaseMat(&pMatImage64);
	cvReleaseMat(&pMatImage128);
	cvReleaseMat(&pMatImage256);
	// ======================= //

	cvReleaseMat(&pMatTemplate);
	cvReleaseMat(&pMatTemplate2);
	cvReleaseMat(&pMatTemplate4);
	cvReleaseMat(&pMatTemplate8);	
	cvReleaseMat(&pMatTemplate16);
	cvReleaseMat(&pMatFlag4);
	cvReleaseMat(&pMatFlag8);
	// ======================= //
	// 扩展到8层NCC模板匹配
	cvReleaseMat(&pMatTemplate32);
	cvReleaseMat(&pMatTemplate64);
	cvReleaseMat(&pMatTemplate128);
	cvReleaseMat(&pMatTemplate256);
	cvReleaseMat(&pMatFlag16);
	cvReleaseMat(&pMatFlag32);
	cvReleaseMat(&pMatFlag64);
	cvReleaseMat(&pMatFlag128);
	// ======================= //



}

//****************************************************************************
CvMat* CDBImageProcess::PreAllocMemory(int nSrcWidth,int nSrcHeight)
{
	int width=0,height=0,temp=0;
	CvMat * pMat=NULL;
	temp=nSrcWidth/2;//如果源图像宽（高）为奇数，且除以二后仍为奇数时
	if ((nSrcWidth-2*(nSrcWidth/2))&&(temp-2*(temp/2)))
		width=(nSrcWidth+1)/2;
	else 
		width=nSrcWidth/2;
	temp=nSrcHeight/2;
	if ((nSrcHeight-2*(nSrcHeight/2))&&(temp-2*(temp/2)))
		height=(nSrcHeight+1)/2;
	else
		height=nSrcHeight/2;
	pMat=cvCreateMat(height,width,CV_8UC1);

	return pMat;
}


//********************************************************
bool CDBImageProcess::AllocateMemory(	int nImageWidth,  //目标图像的宽
										int nImageHeight,
										int nTemplateWidth,//模板图像的宽
										int nTemplateHeight)
{
	int width=0,height=0,temp=0,h=0,w=0;
	if(pMatTemplate!=NULL)
	{
		cvReleaseMat(&pMatTemplate);
		pMatTemplate=NULL;
	}
	pMatTemplate=cvCreateMat(nTemplateHeight,nTemplateWidth,CV_8UC1);
	//*********************
	if(pMatTemplate2!=NULL)
	{
		cvReleaseMat(&pMatTemplate2);
		pMatTemplate2=NULL;
	}
	pMatTemplate2=PreAllocMemory(nTemplateWidth,nTemplateHeight);
    //*********************
	width=pMatTemplate2->width;
	height=pMatTemplate2->height;

	if(pMatTemplate4!=NULL)
	{
		cvReleaseMat(&pMatTemplate4);
		pMatTemplate4=NULL;
	}
	pMatTemplate4=PreAllocMemory(width,height);
	//*********************
	width=pMatTemplate4->width;
	height=pMatTemplate4->height;

	if(pMatTemplate8!=NULL)
	{
		cvReleaseMat(&pMatTemplate8);
		pMatTemplate8=NULL;
	}
	pMatTemplate8=PreAllocMemory(width,height);
	//*********************
	width=pMatTemplate8->width;
	height=pMatTemplate8->height;

	if(pMatTemplate16!=NULL)
	{
		cvReleaseMat(&pMatTemplate16);
		pMatTemplate16=NULL;
	}
	pMatTemplate16=PreAllocMemory(width,height);

	// ======================= //
	// 扩展到8层NCC模板匹配
	width=pMatTemplate16->width;
	height=pMatTemplate16->height;

	if(pMatTemplate32!=NULL)
	{
		cvReleaseMat(&pMatTemplate32);
		pMatTemplate32=NULL;
	}
	pMatTemplate32=PreAllocMemory(width,height);
	//*********************
	width=pMatTemplate32->width;
	height=pMatTemplate32->height;

	if(pMatTemplate64!=NULL)
	{
		cvReleaseMat(&pMatTemplate64);
		pMatTemplate64=NULL;
	}
	pMatTemplate64=PreAllocMemory(width,height);
	//*********************
	width=pMatTemplate64->width;
	height=pMatTemplate64->height;

	if(pMatTemplate128!=NULL)
	{
		cvReleaseMat(&pMatTemplate128);
		pMatTemplate128=NULL;
	}
	pMatTemplate128=PreAllocMemory(width,height);
	//*********************
	width=pMatTemplate128->width;
	height=pMatTemplate128->height;

	if(pMatTemplate256!=NULL)
	{
		cvReleaseMat(&pMatTemplate256);
		pMatTemplate256=NULL;
	}
	pMatTemplate256=PreAllocMemory(width,height);
	// ======================= //

	if ((pMatTemplate==NULL)||(pMatTemplate2==NULL)||(pMatTemplate4==NULL)||(pMatTemplate8==NULL)||(pMatTemplate16==NULL))
		return FALSE;//分配内存失败!!!

	// ======================= //
	// 扩展到8层NCC模板匹配
	if ((pMatTemplate32==NULL)||(pMatTemplate64==NULL)||(pMatTemplate128==NULL)||(pMatTemplate256==NULL))
		return FALSE;//分配内存失败!!!
	// ======================= //
	
	//*********************
	if (pMatImage2!=NULL)
	{
		cvReleaseMat(&pMatImage2);
		pMatImage2=NULL;
	}
	pMatImage2=PreAllocMemory(nImageWidth,nImageHeight);
	//*********************
	width=pMatImage2->width;
	height=pMatImage2->height;
	if (pMatImage4!=NULL)
	{
		cvReleaseMat(&pMatImage4);
		pMatImage4=NULL;
	}
	pMatImage4=PreAllocMemory(width,height);
	if (pMatFlag4!=NULL)
	{
		cvReleaseMat(&pMatFlag4);
		pMatFlag4=NULL;
	}
	pMatFlag4=PreAllocMemory(width,height);
	//*********************
	width=pMatImage4->width;
	height=pMatImage4->height;
	if (pMatImage8!=NULL)
	{
		cvReleaseMat(&pMatImage8);
		pMatImage8=NULL;
	}
	pMatImage8=PreAllocMemory(width,height);
	if (pMatFlag8!=NULL)
	{
		cvReleaseMat(&pMatFlag8);
		pMatFlag8=NULL;
	}
	pMatFlag8=PreAllocMemory(width,height);
	//*********************
	width=pMatImage8->width;
	height=pMatImage8->height;
	if (pMatImage16!=NULL)
	{
		cvReleaseMat(&pMatImage16);
		pMatImage16=NULL;
	}
	pMatImage16=PreAllocMemory(width,height);
	// ======================= //
	// 扩展到8层NCC模板匹配
	if (pMatFlag16!=NULL)
	{
		cvReleaseMat(&pMatFlag16);
		pMatFlag16=NULL;
	}
	pMatFlag16=PreAllocMemory(width,height);
	// ======================= //

	// ======================= //
	// 扩展到8层NCC模板匹配
	width=pMatImage16->width;
	height=pMatImage16->height;
	if (pMatImage32!=NULL)
	{
		cvReleaseMat(&pMatImage32);
		pMatImage32=NULL;
	}
	pMatImage32=PreAllocMemory(width,height);
	if (pMatFlag32!=NULL)
	{
		cvReleaseMat(&pMatFlag32);
		pMatFlag32=NULL;
	}
	pMatFlag32=PreAllocMemory(width,height);
	//*********************
	width=pMatImage32->width;
	height=pMatImage32->height;
	if (pMatImage64!=NULL)
	{
		cvReleaseMat(&pMatImage64);
		pMatImage64=NULL;
	}
	pMatImage64=PreAllocMemory(width,height);
	if (pMatFlag64!=NULL)
	{
		cvReleaseMat(&pMatFlag64);
		pMatFlag64=NULL;
	}
	pMatFlag64=PreAllocMemory(width,height);
	//*********************
	width=pMatImage64->width;
	height=pMatImage64->height;
	if (pMatImage128!=NULL)
	{
		cvReleaseMat(&pMatImage128);
		pMatImage128=NULL;
	}
	pMatImage128=PreAllocMemory(width,height);
	if (pMatFlag128!=NULL)
	{
		cvReleaseMat(&pMatFlag128);
		pMatFlag128=NULL;
	}
	pMatFlag128=PreAllocMemory(width,height);
	//*********************
	width=pMatImage128->width;
	height=pMatImage128->height;
	if (pMatImage256!=NULL)
	{
		cvReleaseMat(&pMatImage256);
		pMatImage256=NULL;
	}
	pMatImage256=PreAllocMemory(width,height);
	// ======================= //

	//*********************
	if ((pMatImage2==NULL)||(pMatImage4==NULL)||(pMatImage8==NULL)||(pMatImage16==NULL))
		return FALSE;//分配内存失败!!!
	if ((pMatFlag8==NULL)||(pMatFlag4==NULL))
		return FALSE;//分配内存失败!!!

	// ======================= //
	// 扩展到8层NCC模板匹配
	if ((pMatImage32==NULL)||(pMatImage64==NULL)||(pMatImage128==NULL)||(pMatImage256==NULL))
		return FALSE;//分配内存失败!!!
	if ((pMatFlag32==NULL)||(pMatFlag64==NULL)||(pMatFlag128==NULL))
		return FALSE;//分配内存失败!!!
	// ======================= //

	//*********************
	for (h=0;h<pMatFlag4->height;h++)//初始化标志矩阵
	{
		for (w=0;w<pMatFlag4->width;w++)
		{
			(pMatFlag4->data.ptr+h*pMatFlag4->step)[w]=0;
		}
	}
	for (h=0;h<pMatFlag8->height;h++)//初始化标志矩阵
	{
		for (w=0;w<pMatFlag8->width;w++)
		{
			(pMatFlag8->data.ptr+h*pMatFlag8->step)[w]=0;
		}
	}

	// ======================= //
	// 扩展到8层NCC模板匹配
	for (h=0;h<pMatFlag16->height;h++)//初始化标志矩阵
	{
		for (w=0;w<pMatFlag16->width;w++)
		{
			(pMatFlag16->data.ptr+h*pMatFlag16->step)[w]=0;
		}
	}
	for (h=0;h<pMatFlag32->height;h++)//初始化标志矩阵
	{
		for (w=0;w<pMatFlag32->width;w++)
		{
			(pMatFlag32->data.ptr+h*pMatFlag32->step)[w]=0;
		}
	}
	for (h=0;h<pMatFlag64->height;h++)//初始化标志矩阵
	{
		for (w=0;w<pMatFlag64->width;w++)
		{
			(pMatFlag64->data.ptr+h*pMatFlag64->step)[w]=0;
		}
	}
	for (h=0;h<pMatFlag128->height;h++)//初始化标志矩阵
	{
		for (w=0;w<pMatFlag128->width;w++)
		{
			(pMatFlag128->data.ptr+h*pMatFlag128->step)[w]=0;
		}
	}
	// ======================= //

	return TRUE;
}

//**********************************************************
bool CDBImageProcess::LoadTemplate(char* szFile)
{
	if (m_pTemplate!=NULL)
	{
		cvReleaseImage(&m_pTemplate);
		m_pTemplate = NULL;
	}
	
	m_pTemplate=cvLoadImage(szFile);
	
	if (NULL==m_pTemplate)
	{
		return false;
	}
	
	bool rtn = AllocateMemory(IMAGEWIDTH,IMAGEHEIGHT,m_pTemplate->width,m_pTemplate->height);
	if ( !rtn)
	{
		cvReleaseImage(&m_pTemplate);
		m_pTemplate = NULL;
		return false;
	}
	PreProcessTemplate(m_pTemplate);
	return true;
}

//**************************************************************
bool CDBImageProcess::SaveTemplate(int nWidth,int nHeight,int top,int left,unsigned char* pbImgData)
{
	int i=0,j=0,h=0,w=0;
	if (m_pTemplate!=NULL)
	{
		cvReleaseImage(&m_pTemplate);
		m_pTemplate = NULL;
	}
	m_pTemplate=cvCreateImage(cvSize(nWidth,nHeight),IPL_DEPTH_8U,1);
	if(m_pTemplate==NULL)
		return FALSE;
	for (j=0,h=top;j<nHeight;j++,h++)
	{
		for (i=0,w=left;i<nWidth;i++,w++)
		{
			((uchar*)(m_pTemplate->imageData+j*m_pTemplate->widthStep))[i]=((uchar*)(pbImgData+IMAGEWIDTH*h))[w];
		}
	}
	bool rtn = AllocateMemory(IMAGEWIDTH,IMAGEHEIGHT,m_pTemplate->width,m_pTemplate->height);
	if ( !rtn)
	{
		cvReleaseImage(&m_pTemplate);
		m_pTemplate = NULL;
		return false;
	}
	//cvSaveImage("model.bmp",m_pTemplate);
	PreProcessTemplate(m_pTemplate);
	return TRUE;
}

//**********************************************************
void CDBImageProcess::PreProcessTemplate(IplImage* pTemplate)											
{
	CvMat* pMatTemp=NULL;
	int h=0,w=0,n=0,m=0;
	int nWidth=0,nHeight=0,nWidthStep=0;
	int nTemplateWidth=0,nTemplateHeight=0,nTemplatenChannels=1;
	unsigned int SumGrayTemplate=0;
	char* ucpData=NULL;
	unsigned char nGrayValue=0;
	
	//将模板图像数据复制到pMatTemplate矩阵中,并计算相关参数
	nTemplateWidth=pTemplate->width;
	nTemplateHeight=pTemplate->height;
	pMatTemp=cvCreateMat(nTemplateHeight,nTemplateWidth,CV_8UC1);
	nTemplatenChannels=pTemplate->nChannels;//模板通道数
	m_nTemplateSize=pTemplate->height*pTemplate->width; //求得原模板图像的总像素点数
	ucpData=pTemplate->imageData;
	nWidthStep=pTemplate->widthStep;
	//将原始模板图像复制到pMatTemp矩阵中
	for (h=0;h<nTemplateHeight;h++)
	{
		for (w=0;w<nTemplateWidth;w++)
		{
			(pMatTemp->data.ptr+h*pMatTemp->step)[w]=((uchar*)(ucpData + nWidthStep*h))[nTemplatenChannels*w];
		}
	}
	cvSmooth(pMatTemp,pMatTemplate,CV_GAUSSIAN,3,3,0,0);//滤波
	//计算原模板参数
	m_SumSquareTemplate=0;
	m_NSquareSumTemplate=0;
	for (h=0;h<nTemplateHeight;h++)  
	{
		for (w=0;w<nTemplateWidth;w++)
		{
			nGrayValue=(pMatTemplate->data.ptr+h*pMatTemplate->step)[w];
			SumGrayTemplate+=nGrayValue;
			m_NSquareSumTemplate+=(unsigned int)nGrayValue*nGrayValue;
		}
	}
	m_nSumGrayTemplate=SumGrayTemplate;
	m_nSquareSumOfTemplate=m_NSquareSumTemplate;
	m_NSquareSumTemplate=m_nTemplateSize*m_NSquareSumTemplate;
	m_SumSquareTemplate=(double)SumGrayTemplate*SumGrayTemplate;
	PyramidDown(pMatTemplate,pMatTemplate2,NULL); //1/2
	PyramidDown(pMatTemplate2,pMatTemplate4,NULL);//1/4
	PyramidDown(pMatTemplate4,pMatTemplate8,NULL);//1/8
	PyramidDown(pMatTemplate8,pMatTemplate16,NULL);//1/16
	// ======================= //
	// 扩展到8层NCC模板匹配
	PyramidDown(pMatTemplate16,pMatTemplate32,NULL);//1/32
	PyramidDown(pMatTemplate32,pMatTemplate64,NULL);//1/64
	PyramidDown(pMatTemplate64,pMatTemplate128,NULL);//1/128
	PyramidDown(pMatTemplate128,pMatTemplate256,NULL);//1/256
	// ======================= //

	//计算1/2模板参数
    nWidth=pMatTemplate2->width;
	nHeight=pMatTemplate2->height;
	m_SumSquareTemplate2=0;
	m_NSquareSumTemplate2=0;
	SumGrayTemplate=0;
	m_nTemplateSize2=nWidth*nHeight;
	for (h=0;h<nHeight;h++)  
	{
		for (w=0;w<nWidth;w++)
		{
			nGrayValue=(pMatTemplate2->data.ptr+h*pMatTemplate2->step)[w];
			SumGrayTemplate+=nGrayValue;
			m_NSquareSumTemplate2+=(unsigned int)nGrayValue*nGrayValue;
		}
	}
	m_nSumGrayTemplate2=SumGrayTemplate;
	m_NSquareSumTemplate2=m_nTemplateSize2*m_NSquareSumTemplate2;
	m_SumSquareTemplate2=(double)SumGrayTemplate*SumGrayTemplate;
	//计算1/4模板参数
    nWidth=pMatTemplate4->width;
	nHeight=pMatTemplate4->height;
	m_SumSquareTemplate4=0;
	m_NSquareSumTemplate4=0;
	SumGrayTemplate=0;
	m_nTemplateSize4=nWidth*nHeight;
	for (h=0;h<nHeight;h++)  
	{
		for (w=0;w<nWidth;w++)
		{
			nGrayValue=(pMatTemplate4->data.ptr+h*pMatTemplate4->step)[w];
			SumGrayTemplate+=nGrayValue;
			m_NSquareSumTemplate4+=(unsigned int)nGrayValue*nGrayValue;
		}
	}
	m_nSumGrayTemplate4=SumGrayTemplate;
	m_NSquareSumTemplate4=m_nTemplateSize4*m_NSquareSumTemplate4;
	m_SumSquareTemplate4=(double)SumGrayTemplate*SumGrayTemplate;
	// 计算1/8模板参数
	nWidth=pMatTemplate8->width;
	nHeight=pMatTemplate8->height;
	m_SumSquareTemplate8=0;
	m_NSquareSumTemplate8=0;
	SumGrayTemplate=0;
	m_nTemplateSize8=nWidth*nHeight;
	for (h=0;h<nHeight;h++)  
	{
		for (w=0;w<nWidth;w++)
		{
			nGrayValue=(pMatTemplate8->data.ptr+h*pMatTemplate8->step)[w];
			SumGrayTemplate+=nGrayValue;
			m_NSquareSumTemplate8+=(unsigned int)nGrayValue*nGrayValue;
		}
	}
	m_nSumGrayTemplate8=SumGrayTemplate;
	m_NSquareSumTemplate8=m_nTemplateSize8*m_NSquareSumTemplate8;
	m_SumSquareTemplate8=(double)SumGrayTemplate*SumGrayTemplate;
	// ======================= //
	// 扩展到8层NCC模板匹配
	// 计算1/16模板参数
	nWidth=pMatTemplate16->width;
	nHeight=pMatTemplate16->height;
	m_SumSquareTemplate16=0;
	m_NSquareSumTemplate16=0;
	SumGrayTemplate=0;
	m_nTemplateSize16=nWidth*nHeight;
	for (h=0;h<nHeight;h++)  
	{
		for (w=0;w<nWidth;w++)
		{
			nGrayValue=(pMatTemplate16->data.ptr+h*pMatTemplate16->step)[w];
			SumGrayTemplate+=nGrayValue;
			m_NSquareSumTemplate16+=(unsigned int)nGrayValue*nGrayValue;
		}
	}
	m_nSumGrayTemplate16=SumGrayTemplate;
	m_NSquareSumTemplate16=m_nTemplateSize16*m_NSquareSumTemplate16;
	m_SumSquareTemplate16=(double)SumGrayTemplate*SumGrayTemplate;
	// 计算1/32模板参数
	nWidth=pMatTemplate32->width;
	nHeight=pMatTemplate32->height;
	m_SumSquareTemplate32=0;
	m_NSquareSumTemplate32=0;
	SumGrayTemplate=0;
	m_nTemplateSize32=nWidth*nHeight;
	for (h=0;h<nHeight;h++)  
	{
		for (w=0;w<nWidth;w++)
		{
			nGrayValue=(pMatTemplate32->data.ptr+h*pMatTemplate32->step)[w];
			SumGrayTemplate+=nGrayValue;
			m_NSquareSumTemplate32+=(unsigned int)nGrayValue*nGrayValue;
		}
	}
	m_nSumGrayTemplate32=SumGrayTemplate;
	m_NSquareSumTemplate32=m_nTemplateSize32*m_NSquareSumTemplate32;
	m_SumSquareTemplate32=(double)SumGrayTemplate*SumGrayTemplate;
	// 计算1/64模板参数
	nWidth=pMatTemplate64->width;
	nHeight=pMatTemplate64->height;
	m_SumSquareTemplate64=0;
	m_NSquareSumTemplate64=0;
	SumGrayTemplate=0;
	m_nTemplateSize64=nWidth*nHeight;
	for (h=0;h<nHeight;h++)  
	{
		for (w=0;w<nWidth;w++)
		{
			nGrayValue=(pMatTemplate64->data.ptr+h*pMatTemplate64->step)[w];
			SumGrayTemplate+=nGrayValue;
			m_NSquareSumTemplate64+=(unsigned int)nGrayValue*nGrayValue;
		}
	}
	m_nSumGrayTemplate64=SumGrayTemplate;
	m_NSquareSumTemplate64=m_nTemplateSize64*m_NSquareSumTemplate64;
	m_SumSquareTemplate64=(double)SumGrayTemplate*SumGrayTemplate;
	// 计算1/128模板参数
	nWidth=pMatTemplate128->width;
	nHeight=pMatTemplate128->height;
	m_SumSquareTemplate128=0;
	m_NSquareSumTemplate128=0;
	SumGrayTemplate=0;
	m_nTemplateSize128=nWidth*nHeight;
	for (h=0;h<nHeight;h++)  
	{
		for (w=0;w<nWidth;w++)
		{
			nGrayValue=(pMatTemplate128->data.ptr+h*pMatTemplate128->step)[w];
			SumGrayTemplate+=nGrayValue;
			m_NSquareSumTemplate128+=(unsigned int)nGrayValue*nGrayValue;
		}
	}
	m_nSumGrayTemplate128=SumGrayTemplate;
	m_NSquareSumTemplate128=m_nTemplateSize128*m_NSquareSumTemplate128;
	m_SumSquareTemplate128=(double)SumGrayTemplate*SumGrayTemplate;
	// 计算1/256模板参数
	nWidth=pMatTemplate256->width;
	nHeight=pMatTemplate256->height;
	m_SumSquareTemplate256=0;
	m_NSquareSumTemplate256=0;
	SumGrayTemplate=0;
	m_nTemplateSize256=nWidth*nHeight;
	for (h=0;h<nHeight;h++)  
	{
		for (w=0;w<nWidth;w++)
		{
			nGrayValue=(pMatTemplate256->data.ptr+h*pMatTemplate256->step)[w];
			SumGrayTemplate+=nGrayValue;
			m_NSquareSumTemplate256+=(unsigned int)nGrayValue*nGrayValue;
		}
	}
	m_nSumGrayTemplate256=SumGrayTemplate;
	m_NSquareSumTemplate256=m_nTemplateSize256*m_NSquareSumTemplate256;
	m_SumSquareTemplate256=(double)SumGrayTemplate*SumGrayTemplate;
	// ======================= //

	// 释放内存
	cvReleaseMat(&pMatTemp);
}

//************************************************************
//!图像金字塔细分
//!1/2细分:边缘部分为原始图像灰度值;其他隔行隔列取点(偶数点),
//!该点的灰度值为其邻域(3×3)的平均值。
void CDBImageProcess::PyramidDown(CvMat* src,CvMat* dst,IplImage* pImg)
{
	int h=0,w=0,n=0,m=0;
	int nWidth=0,nHeight=0,temp=0,nSrcStep=0,nDstStep=0;
	unsigned char* pSrcData=NULL;
	unsigned char* pDstData=NULL;

	nSrcStep=src->step;
	nDstStep=dst->step;
	pSrcData=src->data.ptr;
	pDstData=dst->data.ptr;
	temp=src->width/2;//如果源图像宽（高）为奇数，且除以二后仍为奇数时
	if ((src->width-2*(src->width/2))&&(temp-2*(temp/2)))
		nWidth=(src->width+1)/2;
	else 
		nWidth=src->width/2;
	temp=src->height/2;
	if ((src->height-2*(src->height/2))&&(temp-2*(temp/2)))
		nHeight=(src->height+1)/2;
	else
		nHeight=src->height/2;
	temp=src->width/2;
	if ((src->width-2*(src->width/2))&&(temp-2*(temp/2)))
	{   //如果源图像宽（高）为奇数，且除以2后仍为奇数时
		//最后一列为原始灰度值
		w=src->width-1;
		m=dst->width-1;
		for (h=0,n=0;n<nHeight;h+=2,n++)
			(pDstData+n*nDstStep)[m]=(pSrcData+h*nSrcStep)[w];
	}
	temp=src->height/2;
	if ((src->height-2*(src->height/2))&&(temp-2*(temp/2)))
	{	//同上
		//最后一行为原始灰度值
		h=src->height-1;
		n=dst->height-1;
		for (w=0,m=0;m<nWidth;w+=2,m++)
			(pDstData+n*nDstStep)[m]=(pSrcData+h*nSrcStep)[w];
	}
	nWidth=src->width-1;
	nHeight=src->height-1;
	//其他区域的像素值为其2×2邻域的平均值
    for (h=0,n=0;h<nHeight;h+=2,n++)
    {
		for (w=0,m=0;w<nWidth;w+=2,m++)
		{
			(pDstData+n*nDstStep)[m]= ( (pSrcData +(h)*nSrcStep)[w]+
										(pSrcData +(h)*nSrcStep)[w+1]+
										(pSrcData +(h+1)*nSrcStep)[w]+
										(pSrcData +(h+1)*nSrcStep)[w+1]+2)>>2;//加2，是为了四舍五入
		}
    }
}

//Compute the location of target in the image
double* CDBImageProcess::GetTargetLocation()
{
	m_Locoftarget[0] = m_Locoftarget[0]+m_pTemplate->height/2.0f-0.5f;
	m_Locoftarget[1] = m_Locoftarget[1]+m_pTemplate->width/2.0f-0.5f;
	
	double temp = m_Locoftarget[0];
	m_Locoftarget[0] = m_Locoftarget[1];
	m_Locoftarget[1] = temp;
	
	return m_Locoftarget;
}

//***********************************************************
//!单目标匹配
//!寻找匹配度最大的目标点
// ========================================================== //
//						[原始函数解释]
// 包含了一个单模板匹配函数
// 该函数可以对一幅输入图像和一个给定的模板进行匹配
// 匹配过程中，函数计算每个像素与模板之间的相似度（归一化相关系数 NCC）
// 然后在输入图像上寻找与模板相似度最高的位置
//
// 函数首先将输入图像进行预处理，包括将其缩小为原始尺寸的四分之一和八分之一，然后在八分之一大小的图像上运行匹配算法
// 匹配算法首先将模板和输入图像分别存储在二维数组中
// 然后使用滑动窗口算法，在输入图像上按步长（2个像素）扫描图像
// 计算当前窗口与模板之间的 NCC 值
// 
// 如果当前窗口的 NCC 值高于之前的值，则将其插入到一个大小为 7 的结构体数组中
// 该数组存储了与模板相似度最高的 7 个位置
// 如果该数组已满，则当新的位置的 NCC 值高于数组中最小的 NCC 值时，将替换数组中最小的元素
// 
// 函数还使用了一些计时器，以帮助记录运行时间
// 
// 函数的输入参数包括输入图像的数据指针、图像的宽度和高度、图像的步长和通道数，以及模板的宽度和高度
// 函数的输出参数包括匹配度最高的位置和匹配度值
// 函数返回一个布尔值，指示是否成功执行了匹配算法 
// ========================================================== //
//						[修改说明]
// 增加了金字塔高层图像的相关变量，以便于在高层图像上进行匹配
// 添加了部分控制变量，以防止金字塔层数过高影响匹配结果
// 代码存在大量的冗余，需要进一步优化
// 部分结构可以封装为函数，可考虑修改相关变量为数组等，便于封装和批量处理
bool CDBImageProcess::SingleTemplateMatch( char* ImageData, 
										  int ImageWidth,
										  int ImageHeight,
										  int widthstep,
										  int ImagenChannels)
{
	// = LEVEL3POINT 参照头文件宏定义，为12 = //
	// = RELEVEL3POINT 参照头文件宏定义，为12*4=48 = //
	// = 主打一个遥相呼应 = //
	SelectPoint SelectNcc[LEVEL3POINT];			// = 12个备选最大NCC值的点 = //
	SelectPoint ReSelectNcc[RELEVEL3POINT];		// = 48个备选最大NCC值的点 = //

	SelectPoint MaxNccPoint,TempPoint;		// = 最大NCC值的点，临时点 = //
	double fNccArray[7][7];					// = 7*7的NCC值数组 = // // = 但是好像没有用到 = //
	double m_fNccArray[9];					// = 9个NCC值的数组 = //
	// = 初始化NCC数值、平方和 = //
	double NCC=0.0,fNsumIM=0.0,fNSquareSumImage=0.0,fSumSquareImage=0.0,temp=0.0;
	int nWidth=0,nHeight=0,nTemplateWidth=0,nTemplateHeight=0;
	int NumCounter=0,nTemp1=0,nTemp2=0;		// = 计数器，临时变量 = //
	int nWidthStep=0,nIstep=0,nTstep=0;		// = 步长 = //
	int h=0,w=0,n=0,m=0,l=0,k=0,i=0,j=0;	// = 循环变量 = //
	unsigned int nSumIM=0,nSquareSumImage=0,nGray=0,nSumGrayImage=0;	// = 复灰度和，复平方和，灰度值，灰度和 = //
	unsigned int nSubSquareSum=0,nSubSquareSumThreshold=0;				// = 平方和阈值 = //
	unsigned char* pIData=NULL;		// = 输入图像数据指针 = //
	unsigned char* pTData=NULL;		// = 模板图像数据指针 = //

	CHTimer timer;					// = 计时器 = //

	if((m_pTemplate==NULL)||(ImageData==NULL))
		return FALSE;//未加载模板，退出函数
    
/*	timer.StartTime();*/
	// = 初始化NCC数值 = //
    for (i=0;i<7;i++)
    {
		for (j=0;j<7;j++)
		{
			fNccArray[i][j]=0.0;
		}
    }
	pTData=pMatImage2->data.ptr;
	nWidthStep=pMatImage2->width;
	pIData=(unsigned char*)ImageData;
	nWidth=pMatImage2->width;
	nHeight=pMatImage2->height;
	for (h=0,n=0;n<nHeight;h+=2,n++)
	{
		nIstep=h*widthstep;		// = 输入图像的步长 = //
		nTstep=n*nWidthStep;	// = 模板图像的步长 = //
		// = 用于正确进行数组赋值 = //
		for (w=0,m=0;m<nWidth;w+=2,m++)
			(pTData+nTstep)[m]= (pIData +nIstep)[ImagenChannels*w];
	}
	// = 图像金字塔降采样 = //
	PyramidDown(pMatImage2,pMatImage4,NULL);
	PyramidDown(pMatImage4,pMatImage8,NULL);
	// ======================= //
	// 扩展到8层NCC模板匹配
	PyramidDown(pMatImage8,pMatImage16,NULL);
	PyramidDown(pMatImage16,pMatImage32,NULL);
	PyramidDown(pMatImage32,pMatImage64,NULL);
	PyramidDown(pMatImage64,pMatImage128,NULL);
	PyramidDown(pMatImage128,pMatImage256,NULL);
	// ======================= //

// 	timer.EndTime();
// 	cout<<"预处理用时："<<timer.GetTime()<<"ms"<<endl;
// 	timer.StartTime();

	for (k=0;k<LEVEL3POINT;k++)//复位结构数组
		SelectNcc[k].score=0.0;
	
	// = 添加额外控制变量 = //
	// = 模板尺寸阈值，当模板尺寸小于阈值时，不再进行这一层的模板匹配 = //
	int nTemplateHeightThreshold=1;
	int nTemplateWidthThreshold=1;
	// = NCC阈值，当上一层给出的NCC备选点数值太小时，重新进行这一层的全局模板匹配 = //
	int nNccThreshold=0.1;
	// = 布尔值，确定下一层是否进行全局模板匹配 = //
	bool bGlobalMatch=true;

	// = 从金字塔的最底层开始匹配 = //
	// = 金字塔第一层 = //
	nTemplateHeight=pMatTemplate256->height;		// = 模板图像的高度 = //
	nTemplateWidth=pMatTemplate256->width;			// = 模板图像的宽度 = //

	// = 若模板尺寸大于阈值，且需要进行全局匹配，则进行全局模板匹配 = //
	if (nTemplateHeight >= nTemplateHeightThreshold&&
		nTemplateWidth >= nTemplateWidthThreshold&&
		bGlobalMatch)
	{
		// = 输入图像尺寸减去模板图像尺寸，用于确定NCC搜索范围 = //
		nWidth=pMatImage256->width-nTemplateWidth;		// = 输入图像的宽度-模板图像的宽度 = //
		nHeight=pMatImage256->height-nTemplateHeight;	// = 输入图像的高度-模板图像的高度 = //
		
		pIData=pMatImage256->data.ptr;					// = 输入图像数据指针 = //
		nIstep=pMatImage256->step;						// = 输入图像的步长 = //
		pTData=pMatTemplate256->data.ptr;				// = 模板图像数据指针 = //
		nTstep=pMatTemplate256->step;					// = 模板图像的步长 = //
		NumCounter=0;									// = 计数器 = //
		nSubSquareSumThreshold=0xFFFFFFFF;				// = 子平方和阈值 = //

		for (h=0;h<nHeight;h++)
		{
			for (w=0;w<nWidth;w++)
			{
				nSubSquareSum=0;
				i=1;
				for (n=0;n<nTemplateHeight;n++)
				{
					nTemp1=(h+n)*nIstep+w;		// = 图像的行地址 = //
					nTemp2=n*nTstep;			// = 模板的行地址 = //
					// = 子平方和 = //
					for (m=0;m<nTemplateWidth;m++)
						nSubSquareSum+=SquareFunc(pIData[nTemp1+m]-pTData[m+nTemp2]);
					// = 若子平方和大于阈值，则终止循环 = //
					if(nSubSquareSum>nSubSquareSumThreshold)
					{
						i=0;
						break;//终止条件 
					}
				}
				if(i)
				{	//插值排序 // = 从大到小排序 = //
					NCC=256.0-(double)nSubSquareSum/SquareFunc(nTemplateWidth*nTemplateHeight);
					if ((NCC>SelectNcc[LEVEL3POINT-1].score)&&(NumCounter>=LEVEL3POINT))
					{//如果当前NCC大于数组的最末一个且数组已填满
						for (l=LEVEL3POINT-2;l>0;l--)
						{
							// = 若当前NCC小于数组中的某个值，则插入到该值之后 = //
							if (NCC<SelectNcc[l].score)
							{
								TempPoint=SelectNcc[l+1];
								SelectNcc[l+1].score=NCC;
								SelectNcc[l+1].height=h;
								SelectNcc[l+1].width=w;
								for (k=LEVEL3POINT-2;k>(l+1);k--)//后移,从最后一个开始移动
									SelectNcc[k+1]=SelectNcc[k];				
								if ((l+2)<LEVEL3POINT)
									SelectNcc[l+2]=TempPoint;
								break;//跳出循环
							}
							// = 若当前NCC大于数组中的最大值，则插入到数组的最前面 = //
							if (NCC>SelectNcc[0].score)
							{
								TempPoint=SelectNcc[0];
								SelectNcc[0].score=NCC;
								SelectNcc[0].height=h;
								SelectNcc[0].width=w;
								for (k=LEVEL3POINT-2;k>0;k--)//后移,从最后一个开始移动
									SelectNcc[k+1]=SelectNcc[k];
								SelectNcc[1]=TempPoint;
							}
						}
						// = 更新阈值 = //
						nSubSquareSumThreshold=(unsigned int)(SquareFunc(nTemplateWidth*nTemplateHeight)*(256.0-SelectNcc[LEVEL3POINT-1].score));
					}
					else if(NumCounter<LEVEL3POINT)//数组未填满
					{
						if (NumCounter==0)//第一次向数组填入数据
						{
							SelectNcc[0].score=NCC;
							SelectNcc[0].height=h;
							SelectNcc[0].width=w;
						}
						else
						{
							for (l=0;l<NumCounter;l++)
							{	
								if (NCC>SelectNcc[l].score)//如果当前匹配度有比数组中大的
								{
									TempPoint=SelectNcc[l];
									SelectNcc[l].score=NCC;
									SelectNcc[l].height=h;
									SelectNcc[l].width=w;
									for (k=NumCounter-1;k>l;k--)//后移,从最后一个开始移动
										SelectNcc[k+1]=SelectNcc[k];
									SelectNcc[l+1]=TempPoint;
									l=NumCounter; //结束循环,不能用break
								}
								if (l==NumCounter-1)//当前数组中最小的一个
								{
									SelectNcc[l+1].score=NCC;
									SelectNcc[l+1].height=h;
									SelectNcc[l+1].width=w;
								}
							}
						}
						NumCounter++;
					}
					else ;//排序结束
				}
			}
		}

		// = 检查NCC数值是否全部小于阈值 = //
		for (k=0;k<LEVEL3POINT;k++)
		{
			// = 若有一个NCC大于阈值，中断循环，下一层金字塔不需要全局匹配 = //
			if (SelectNcc[k].score>0)
			{
				bGlobalMatch=false;
				break;
			}
		}

		// = 若全部NCC小于阈值，则进行全局匹配 = //
		// = 重置NCC数组 = //
		if (bGlobalMatch==true)
		{
			for (k=0;k<LEVEL3POINT;k++)//复位结构数组
				SelectNcc[k].score=0.0;
		}
	}
	else 
		bGlobalMatch=true;

	// = 金字塔第二层 = //
	nTemplateHeight=pMatTemplate128->height;
	nTemplateWidth=pMatTemplate128->width;

	if (nTemplateHeight >= nTemplateHeightThreshold&&
		nTemplateWidth >= nTemplateWidthThreshold&&
		bGlobalMatch)
	{
		nWidth=pMatImage128->width-nTemplateWidth; 
		nHeight=pMatImage128->height-nTemplateHeight;
		
		pIData=pMatImage128->data.ptr;			
		nIstep=pMatImage128->step;			
		pTData=pMatTemplate128->data.ptr;		
		nTstep=pMatTemplate128->step;		
		NumCounter=0;		
		nSubSquareSumThreshold=0xFFFFFFFF;

		for (h=0;h<nHeight;h++)
		{
			for (w=0;w<nWidth;w++)
			{
				nSubSquareSum=0;
				i=1;
				for (n=0;n<nTemplateHeight;n++)
				{
					nTemp1=(h+n)*nIstep+w;		// = 图像的行地址 = //
					nTemp2=n*nTstep;			// = 模板的行地址 = //
					// = 子平方和 = //
					for (m=0;m<nTemplateWidth;m++)
						nSubSquareSum+=SquareFunc(pIData[nTemp1+m]-pTData[m+nTemp2]);
					// = 若子平方和大于阈值，则终止循环 = //
					if(nSubSquareSum>nSubSquareSumThreshold)
					{
						i=0;
						break;//终止条件 
					}
				}
				if(i)
				{	//插值排序 // = 从大到小排序 = //
					NCC=256.0-(double)nSubSquareSum/SquareFunc(nTemplateWidth*nTemplateHeight);
					if ((NCC > SelectNcc[LEVEL3POINT - 1].score) && (NumCounter >= LEVEL3POINT))
					{//如果当前NCC大于数组的最末一个且数组已填满
						for (l = LEVEL3POINT - 2; l > 0; l--)
						{
							// = 若当前NCC小于数组中的某个值，则插入到该值之后 = //
							if (NCC < SelectNcc[l].score)
							{
								TempPoint = SelectNcc[l + 1];
								SelectNcc[l + 1].score = NCC;
								SelectNcc[l + 1].height = h;
								SelectNcc[l + 1].width = w;
								for (k = LEVEL3POINT - 2; k > (l + 1); k--)//后移,从最后一个开始移动
									SelectNcc[k + 1] = SelectNcc[k];
								if ((l + 2) < LEVEL3POINT)
									SelectNcc[l + 2] = TempPoint;
								break;//跳出循环
							}
							// = 若当前NCC大于数组中的最大值，则插入到数组的最前面 = //
							if (NCC > SelectNcc[0].score)
							{
								TempPoint = SelectNcc[0];
								SelectNcc[0].score = NCC;
								SelectNcc[0].height = h;
								SelectNcc[0].width = w;
								for (k = LEVEL3POINT - 2; k > 0; k--)//后移,从最后一个开始移动
									SelectNcc[k + 1] = SelectNcc[k];
								SelectNcc[1] = TempPoint;
							}
						}
						// = 更新阈值 = //
						nSubSquareSumThreshold = (unsigned int)(SquareFunc(nTemplateWidth * nTemplateHeight) * (256.0 - SelectNcc[LEVEL3POINT - 1].score));
					}
					else if (NumCounter < LEVEL3POINT)//数组未填满
					{
						if (NumCounter == 0)//第一次向数组填入数据
						{
							SelectNcc[0].score = NCC;
							SelectNcc[0].height = h;
							SelectNcc[0].width = w;
						}
						else
						{
							for (l = 0; l < NumCounter; l++)
							{
								if (NCC > SelectNcc[l].score)//如果当前匹配度有比数组中大的
								{
									TempPoint = SelectNcc[l];
									SelectNcc[l].score = NCC;
									SelectNcc[l].height = h;
									SelectNcc[l].width = w;
									for (k = NumCounter - 1; k > l; k--)//后移,从最后一个开始移动
										SelectNcc[k + 1] = SelectNcc[k];
									SelectNcc[l + 1] = TempPoint;
									l = NumCounter; //结束循环,不能用break
								}
								if (l == NumCounter - 1)//当前数组中最小的一个
								{
									SelectNcc[l + 1].score = NCC;
									SelectNcc[l + 1].height = h;
									SelectNcc[l + 1].width = w;
								}
							}
						}
						NumCounter++;
					}
					else;//排序结束
				}
			}
		}

		// = 检查NCC数值是否全部小于阈值 = //
		for (k=0;k<LEVEL3POINT;k++)
		{
			// = 若有一个NCC大于阈值，中断循环，下一层金字塔不需要全局匹配 = //
			if (SelectNcc[k].score>0)
			{
				bGlobalMatch=false;
				break;
			}
		}

		// = 若全部NCC小于阈值，则进行全局匹配 = //
		// = 重置NCC数组 = //
		if (bGlobalMatch==true)
		{
			for (k=0;k<LEVEL3POINT;k++)//复位结构数组
				SelectNcc[k].score=0.0;
		}
	}
	else if(nTemplateHeight >= nTemplateHeightThreshold&&
		nTemplateWidth >= nTemplateWidthThreshold)
	{
		nWidth=pMatTemplate128->width;
		nHeight=pMatTemplate128->height;

		pIData=pMatImage128->data.ptr;
		nIstep=pMatImage128->step;
		pTData=pMatTemplate128->data.ptr;
		nTstep=pMatTemplate128->step;
		TempPoint.score=-1;
		NumCounter=0;

		// = 将1/256图像的坐标反映到1/128图像上 = //
		for (k=0;k<LEVEL3POINT;k++)
		{
			h=2*SelectNcc[k].height;
			w=2*SelectNcc[k].width;
			for (i=0;i<2;i++)
			{
				for (j=0;j<2;j++)
				{
					// ReSelectNcc数组保存的是1/128图像的坐标 //
					if(((w+j+nWidth)<=pMatImage128->width)&&((h+i+nHeight)<=pMatImage128->height))
					{
						ReSelectNcc[NumCounter].score=0.0;
						ReSelectNcc[NumCounter].width=w+j;
						ReSelectNcc[NumCounter].height=h+i;
						NumCounter++;
					}
				}
			}
		}
		// = 对每个点计算NCC，求出NCC最大的点 = //
		for (k=0;k<NumCounter;k++)
		{
			h=ReSelectNcc[k].height;
			w=ReSelectNcc[k].width;
			nSumIM=0;
			nSquareSumImage=0;
			nSumGrayImage=0;
			for (n=0;n<nHeight;n++)
			{
				nTemp1=(h+n)*nIstep+w;
				nTemp2=n*nTstep;
				for (m=0;m<nWidth;m++)
				{
					nGray=pIData[nTemp1+m];
					nSumGrayImage+=nGray;
					nSquareSumImage+=nGray*nGray;
					nSumIM+=nGray*(pTData[m+nTemp2]);
				}
			}
			fNsumIM=(double)m_nTemplateSize4*nSumIM;
			fNSquareSumImage=(double)m_nTemplateSize4*nSquareSumImage;
			fSumSquareImage=(double)nSumGrayImage*nSumGrayImage;
			temp=(double)(fNSquareSumImage-fSumSquareImage)*(m_NSquareSumTemplate4-m_SumSquareTemplate4);
			if(temp<0) temp=-temp;
			if(temp==0)
				NCC=0.0;
			else
				NCC=SquareFunc(fNsumIM-(double)nSumGrayImage*m_nSumGrayTemplate4)/temp;
			if(NCC>TempPoint.score)
			{	
				TempPoint.score=NCC;
				TempPoint.width=w;
				TempPoint.height=h;
			}
		}
		
		// = 若最大NCC值小于阈值，则重置NCC备选点 = //
		if (NCC<nNccThreshold)
			bGlobalMatch=true;

		// = 重置NCC数组 = //
		if (bGlobalMatch==true)
		{
			for (k=0;k<LEVEL3POINT;k++)//复位结构数组
				SelectNcc[k].score=0.0;
		}
		else
		{
			// = 更新SelectPoint数组 = //
			for (k=0;k<LEVEL3POINT;k++)
			{
				SelectNcc[k].height=2*SelectNcc[k].height;
				SelectNcc[k].width=2*SelectNcc[k].width;
			}
		}
	}
	else
		bGlobalMatch=true;

	// = 金字塔第三层 = //
	nTemplateHeight=pMatTemplate64->height;
	nTemplateWidth=pMatTemplate64->width;

	if (nTemplateHeight >= nTemplateHeightThreshold&&
		nTemplateWidth >= nTemplateWidthThreshold&&
		bGlobalMatch)
	{
		nWidth=pMatImage64->width-nTemplateWidth; 
		nHeight=pMatImage64->height-nTemplateHeight;
		
		pIData=pMatImage64->data.ptr;			
		nIstep=pMatImage64->step;			
		pTData=pMatTemplate64->data.ptr;		
		nTstep=pMatTemplate64->step;		
		NumCounter=0;		
		nSubSquareSumThreshold=0xFFFFFFFF;

		for (h=0;h<nHeight;h++)
		{
			for (w=0;w<nWidth;w++)
			{
				nSubSquareSum=0;
				i=1;
				for (n=0;n<nTemplateHeight;n++)
				{
					nTemp1=(h+n)*nIstep+w;		// = 图像的行地址 = //
					nTemp2=n*nTstep;			// = 模板的行地址 = //
					// = 子平方和 = //
					for (m=0;m<nTemplateWidth;m++)
						nSubSquareSum+=SquareFunc(pIData[nTemp1+m]-pTData[m+nTemp2]);
					// = 若子平方和大于阈值，则终止循环 = //
					if(nSubSquareSum>nSubSquareSumThreshold)
					{
						i=0;
						break;//终止条件 
					}
				}
				if(i)
				{	//插值排序 // = 从大到小排序 = //
					NCC=256.0-(double)nSubSquareSum/SquareFunc(nTemplateWidth*nTemplateHeight);
					if ((NCC > SelectNcc[LEVEL3POINT - 1].score) && (NumCounter >= LEVEL3POINT))
					{//如果当前NCC大于数组的最末一个且数组已填满
						for (l = LEVEL3POINT - 2; l > 0; l--)
						{
							// = 若当前NCC小于数组中的某个值，则插入到该值之后 = //
							if (NCC < SelectNcc[l].score)
							{
								TempPoint = SelectNcc[l + 1];
								SelectNcc[l + 1].score = NCC;
								SelectNcc[l + 1].height = h;
								SelectNcc[l + 1].width = w;
								for (k = LEVEL3POINT - 2; k > (l + 1); k--)//后移,从最后一个开始移动
									SelectNcc[k + 1] = SelectNcc[k];
								if ((l + 2) < LEVEL3POINT)
									SelectNcc[l + 2] = TempPoint;
								break;//跳出循环
							}
							// = 若当前NCC大于数组中的最大值，则插入到数组的最前面 = //
							if (NCC > SelectNcc[0].score)
							{
								TempPoint = SelectNcc[0];
								SelectNcc[0].score = NCC;
								SelectNcc[0].height = h;
								SelectNcc[0].width = w;
								for (k = LEVEL3POINT - 2; k > 0; k--)//后移,从最后一个开始移动
									SelectNcc[k + 1] = SelectNcc[k];
								SelectNcc[1] = TempPoint;
							}
						}
						// = 更新阈值 = //
						nSubSquareSumThreshold = (unsigned int)(SquareFunc(nTemplateWidth * nTemplateHeight) * (256.0 - SelectNcc[LEVEL3POINT - 1].score));
					}
					else if (NumCounter < LEVEL3POINT)//数组未填满
					{
						if (NumCounter == 0)//第一次向数组填入数据
						{
							SelectNcc[0].score = NCC;
							SelectNcc[0].height = h;
							SelectNcc[0].width = w;
						}
						else
						{
							for (l = 0; l < NumCounter; l++)
							{
								if (NCC > SelectNcc[l].score)//如果当前匹配度有比数组中大的
								{
									TempPoint = SelectNcc[l];
									SelectNcc[l].score = NCC;
									SelectNcc[l].height = h;
									SelectNcc[l].width = w;
									for (k = NumCounter - 1; k > l; k--)//后移,从最后一个开始移动
										SelectNcc[k + 1] = SelectNcc[k];
									SelectNcc[l + 1] = TempPoint;
									l = NumCounter; //结束循环,不能用break
								}
								if (l == NumCounter - 1)//当前数组中最小的一个
								{
									SelectNcc[l + 1].score = NCC;
									SelectNcc[l + 1].height = h;
									SelectNcc[l + 1].width = w;
								}
							}
						}
						NumCounter++;
					}
					else;//排序结束
				}
			}
		}
	
		// = 检查NCC数值是否全部小于阈值 = //
		for (k=0;k<LEVEL3POINT;k++)
		{
			// = 若有一个NCC大于阈值，中断循环，下一层金字塔不需要全局匹配 = //
			if (SelectNcc[k].score>0)
			{
				bGlobalMatch=false;
				break;
			}
		}

		// = 若全部NCC小于阈值，则进行全局匹配 = //
		// = 重置NCC数组 = //
		if (bGlobalMatch==true)
		{
			for (k=0;k<LEVEL3POINT;k++)//复位结构数组
				SelectNcc[k].score=0.0;
		}
	}
	else if(nTemplateHeight >= nTemplateHeightThreshold&&
		nTemplateWidth >= nTemplateWidthThreshold)	
	{
		nWidth=pMatTemplate64->width;
		nHeight=pMatTemplate64->height;

		pIData=pMatImage64->data.ptr;
		nIstep=pMatImage64->step;
		pTData=pMatTemplate64->data.ptr;
		nTstep=pMatTemplate64->step;
		TempPoint.score=-1;
		NumCounter=0;

		// = 将1/128图像的坐标反映到1/64图像上 = //
		for (k=0;k<LEVEL3POINT;k++)
		{
			h=2*SelectNcc[k].height;
			w=2*SelectNcc[k].width;
			for (i=0;i<2;i++)
			{
				for (j=0;j<2;j++)
				{
					// ReSelectNcc数组保存的是1/64图像的坐标 //
					if(((w+j+nWidth)<=pMatImage64->width)&&((h+i+nHeight)<=pMatImage64->height))
					{
						ReSelectNcc[NumCounter].score=0.0;
						ReSelectNcc[NumCounter].width=w+j;
						ReSelectNcc[NumCounter].height=h+i;
						NumCounter++;
					}
				}
			}
		}
		// = 对每个点计算NCC，求出NCC最大的点 = //
		for (k=0;k<NumCounter;k++)
		{
			h=ReSelectNcc[k].height;
			w=ReSelectNcc[k].width;
			nSumIM=0;
			nSquareSumImage=0;
			nSumGrayImage=0;
			for (n=0;n<nHeight;n++)
			{
				nTemp1=(h+n)*nIstep+w;
				nTemp2=n*nTstep;
				for (m=0;m<nWidth;m++)
				{
					nGray=pIData[nTemp1+m];
					nSumGrayImage+=nGray;
					nSquareSumImage+=nGray*nGray;
					nSumIM+=nGray*(pTData[m+nTemp2]);
				}
			}
			fNsumIM=(double)m_nTemplateSize4*nSumIM;
			fNSquareSumImage=(double)m_nTemplateSize4*nSquareSumImage;
			fSumSquareImage=(double)nSumGrayImage*nSumGrayImage;
			temp=(double)(fNSquareSumImage-fSumSquareImage)*(m_NSquareSumTemplate4-m_SumSquareTemplate4);
			if(temp<0) temp=-temp;
			if(temp==0)
				NCC=0.0;
			else
				NCC=SquareFunc(fNsumIM-(double)nSumGrayImage*m_nSumGrayTemplate4)/temp;
			if(NCC>TempPoint.score)
			{	
				TempPoint.score=NCC;
				TempPoint.width=w;
				TempPoint.height=h;
			}
		}
	
		// = 若最大NCC值小于阈值，则重置NCC备选点 = //
		if (NCC<nNccThreshold)
			bGlobalMatch=true;

		// = 重置NCC数组 = //
		if (bGlobalMatch==true)
		{
			for (k=0;k<LEVEL3POINT;k++)//复位结构数组
				SelectNcc[k].score=0.0;
		}
		else
		{
			// = 更新SelectPoint数组 = //
			for (k=0;k<LEVEL3POINT;k++)
			{
				SelectNcc[k].height=2*SelectNcc[k].height;
				SelectNcc[k].width=2*SelectNcc[k].width;
			}
		}
	}
	else
		bGlobalMatch=true;

	// = 金字塔第四层 = //
	nTemplateHeight=pMatTemplate32->height;
	nTemplateWidth=pMatTemplate32->width;

	if (nTemplateHeight >= nTemplateHeightThreshold&&
		nTemplateWidth >= nTemplateWidthThreshold&&
		bGlobalMatch)
	{
		nWidth=pMatImage32->width-nTemplateWidth; 
		nHeight=pMatImage32->height-nTemplateHeight;
		
		pIData=pMatImage32->data.ptr;			
		nIstep=pMatImage32->step;			
		pTData=pMatTemplate32->data.ptr;		
		nTstep=pMatTemplate32->step;		
		NumCounter=0;		
		nSubSquareSumThreshold=0xFFFFFFFF;

		for (h=0;h<nHeight;h++)
		{
			for (w=0;w<nWidth;w++)
			{
				nSubSquareSum=0;
				i=1;
				for (n=0;n<nTemplateHeight;n++)
				{
					nTemp1=(h+n)*nIstep+w;		// = 图像的行地址 = //
					nTemp2=n*nTstep;			// = 模板的行地址 = //
					// = 子平方和 = //
					for (m=0;m<nTemplateWidth;m++)
						nSubSquareSum+=SquareFunc(pIData[nTemp1+m]-pTData[m+nTemp2]);
					// = 若子平方和大于阈值，则终止循环 = //
					if(nSubSquareSum>nSubSquareSumThreshold)
					{
						i=0;
						break;//终止条件 
					}
				}
				if(i)
				{	//插值排序 // = 从大到小排序 = //
					NCC=256.0-(double)nSubSquareSum/SquareFunc(nTemplateWidth*nTemplateHeight);
					if ((NCC > SelectNcc[LEVEL3POINT - 1].score) && (NumCounter >= LEVEL3POINT))
					{//如果当前NCC大于数组的最末一个且数组已填满
						for (l = LEVEL3POINT - 2; l > 0; l--)
						{
							// = 若当前NCC小于数组中的某个值，则插入到该值之后 = //
							if (NCC < SelectNcc[l].score)
							{
								TempPoint = SelectNcc[l + 1];
								SelectNcc[l + 1].score = NCC;
								SelectNcc[l + 1].height = h;
								SelectNcc[l + 1].width = w;
								for (k = LEVEL3POINT - 2; k > (l + 1); k--)//后移,从最后一个开始移动
									SelectNcc[k + 1] = SelectNcc[k];
								if ((l + 2) < LEVEL3POINT)
									SelectNcc[l + 2] = TempPoint;
								break;//跳出循环
							}
							// = 若当前NCC大于数组中的最大值，则插入到数组的最前面 = //
							if (NCC > SelectNcc[0].score)
							{
								TempPoint = SelectNcc[0];
								SelectNcc[0].score = NCC;
								SelectNcc[0].height = h;
								SelectNcc[0].width = w;
								for (k = LEVEL3POINT - 2; k > 0; k--)//后移,从最后一个开始移动
									SelectNcc[k + 1] = SelectNcc[k];
								SelectNcc[1] = TempPoint;
							}
						}
						// = 更新阈值 = //
						nSubSquareSumThreshold = (unsigned int)(SquareFunc(nTemplateWidth * nTemplateHeight) * (256.0 - SelectNcc[LEVEL3POINT - 1].score));
					}
					else if (NumCounter < LEVEL3POINT)//数组未填满
					{
						if (NumCounter == 0)//第一次向数组填入数据
						{
							SelectNcc[0].score = NCC;
							SelectNcc[0].height = h;
							SelectNcc[0].width = w;
						}
						else
						{
							for (l = 0; l < NumCounter; l++)
							{
								if (NCC > SelectNcc[l].score)//如果当前匹配度有比数组中大的
								{
									TempPoint = SelectNcc[l];
									SelectNcc[l].score = NCC;
									SelectNcc[l].height = h;
									SelectNcc[l].width = w;
									for (k = NumCounter - 1; k > l; k--)//后移,从最后一个开始移动
										SelectNcc[k + 1] = SelectNcc[k];
									SelectNcc[l + 1] = TempPoint;
									l = NumCounter; //结束循环,不能用break
								}
								if (l == NumCounter - 1)//当前数组中最小的一个
								{
									SelectNcc[l + 1].score = NCC;
									SelectNcc[l + 1].height = h;
									SelectNcc[l + 1].width = w;
								}
							}
						}
						NumCounter++;
					}
					else;//排序结束
				}
			}
		}
	
		// = 检查NCC数值是否全部小于阈值 = //
		for (k=0;k<LEVEL3POINT;k++)
		{
			// = 若有一个NCC大于阈值，中断循环，下一层金字塔不需要全局匹配 = //
			if (SelectNcc[k].score>0)
			{
				bGlobalMatch=false;
				break;
			}
		}

		// = 若全部NCC小于阈值，则进行全局匹配 = //
		// = 重置NCC数组 = //
		if (bGlobalMatch==true)
		{
			for (k=0;k<LEVEL3POINT;k++)//复位结构数组
				SelectNcc[k].score=0.0;
		}
	}
	else if(nTemplateHeight >= nTemplateHeightThreshold&&
		nTemplateWidth >= nTemplateWidthThreshold)
	{
		nWidth=pMatTemplate32->width;
		nHeight=pMatTemplate32->height;

		pIData=pMatImage32->data.ptr;
		nIstep=pMatImage32->step;
		pTData=pMatTemplate32->data.ptr;
		nTstep=pMatTemplate32->step;
		TempPoint.score=-1;
		NumCounter=0;

		// = 将1/64图像的坐标反映到1/32图像上 = //
		for (k=0;k<LEVEL3POINT;k++)
		{
			h=2*SelectNcc[k].height;
			w=2*SelectNcc[k].width;
			for (i=0;i<2;i++)
			{
				for (j=0;j<2;j++)
				{
					// ReSelectNcc数组保存的是1/32图像的坐标 //
					if(((w+j+nWidth)<=pMatImage32->width)&&((h+i+nHeight)<=pMatImage32->height))
					{
						ReSelectNcc[NumCounter].score=0.0;
						ReSelectNcc[NumCounter].width=w+j;
						ReSelectNcc[NumCounter].height=h+i;
						NumCounter++;
					}
				}
			}
		}
		// = 对每个点计算NCC，求出NCC最大的点 = //
		for (k=0;k<NumCounter;k++)
		{
			h=ReSelectNcc[k].height;
			w=ReSelectNcc[k].width;
			nSumIM=0;
			nSquareSumImage=0;
			nSumGrayImage=0;
			for (n=0;n<nHeight;n++)
			{
				nTemp1=(h+n)*nIstep+w;
				nTemp2=n*nTstep;
				for (m=0;m<nWidth;m++)
				{
					nGray=pIData[nTemp1+m];
					nSumGrayImage+=nGray;
					nSquareSumImage+=nGray*nGray;
					nSumIM+=nGray*(pTData[m+nTemp2]);
				}
			}
			fNsumIM=(double)m_nTemplateSize4*nSumIM;
			fNSquareSumImage=(double)m_nTemplateSize4*nSquareSumImage;
			fSumSquareImage=(double)nSumGrayImage*nSumGrayImage;
			temp=(double)(fNSquareSumImage-fSumSquareImage)*(m_NSquareSumTemplate4-m_SumSquareTemplate4);
			if(temp<0) temp=-temp;
			if(temp==0)
				NCC=0.0;
			else
				NCC=SquareFunc(fNsumIM-(double)nSumGrayImage*m_nSumGrayTemplate4)/temp;
			if(NCC>TempPoint.score)
			{	
				TempPoint.score=NCC;
				TempPoint.width=w;
				TempPoint.height=h;
			}
		}
	
		// = 若最大NCC值小于阈值，则重置NCC备选点 = //
		if (NCC<nNccThreshold)
			bGlobalMatch=true;

		// = 重置NCC数组 = //
		if (bGlobalMatch==true)
		{
			for (k=0;k<LEVEL3POINT;k++)//复位结构数组
				SelectNcc[k].score=0.0;
		}
		else
		{
			// = 更新SelectPoint数组 = //
			for (k=0;k<LEVEL3POINT;k++)
			{
				SelectNcc[k].height=2*SelectNcc[k].height;
				SelectNcc[k].width=2*SelectNcc[k].width;
			}
		}
	}
	else
		bGlobalMatch=true;

	// = 金字塔第五层 = //
	nTemplateHeight=pMatTemplate16->height;
	nTemplateWidth=pMatTemplate16->width;

	if (nTemplateHeight >= nTemplateHeightThreshold&&
		nTemplateWidth >= nTemplateWidthThreshold&&
		bGlobalMatch)
	{
		nWidth=pMatImage16->width-nTemplateWidth; 
		nHeight=pMatImage16->height-nTemplateHeight;
		
		pIData=pMatImage16->data.ptr;			
		nIstep=pMatImage16->step;			
		pTData=pMatTemplate16->data.ptr;		
		nTstep=pMatTemplate16->step;		
		NumCounter=0;		
		nSubSquareSumThreshold=0xFFFFFFFF;

		for (h=0;h<nHeight;h++)
		{
			for (w=0;w<nWidth;w++)
			{
				nSubSquareSum=0;
				i=1;
				for (n=0;n<nTemplateHeight;n++)
				{
					nTemp1=(h+n)*nIstep+w;		// = 图像的行地址 = //
					nTemp2=n*nTstep;			// = 模板的行地址 = //
					// = 子平方和 = //
					for (m=0;m<nTemplateWidth;m++)
						nSubSquareSum+=SquareFunc(pIData[nTemp1+m]-pTData[m+nTemp2]);
					// = 若子平方和大于阈值，则终止循环 = //
					if(nSubSquareSum>nSubSquareSumThreshold)
					{
						i=0;
						break;//终止条件 
					}
				}
				if(i)
				{	//插值排序 // = 从大到小排序 = //
					NCC=256.0-(double)nSubSquareSum/SquareFunc(nTemplateWidth*nTemplateHeight);
					if ((NCC > SelectNcc[LEVEL3POINT - 1].score) && (NumCounter >= LEVEL3POINT))
					{//如果当前NCC大于数组的最末一个且数组已填满
						for (l = LEVEL3POINT - 2; l > 0; l--)
						{
							// = 若当前NCC小于数组中的某个值，则插入到该值之后 = //
							if (NCC < SelectNcc[l].score)
							{
								TempPoint = SelectNcc[l + 1];
								SelectNcc[l + 1].score = NCC;
								SelectNcc[l + 1].height = h;
								SelectNcc[l + 1].width = w;
								for (k = LEVEL3POINT - 2; k > (l + 1); k--)//后移,从最后一个开始移动
									SelectNcc[k + 1] = SelectNcc[k];
								if ((l + 2) < LEVEL3POINT)
									SelectNcc[l + 2] = TempPoint;
								break;//跳出循环
							}
							// = 若当前NCC大于数组中的最大值，则插入到数组的最前面 = //
							if (NCC > SelectNcc[0].score)
							{
								TempPoint = SelectNcc[0];
								SelectNcc[0].score = NCC;
								SelectNcc[0].height = h;
								SelectNcc[0].width = w;
								for (k = LEVEL3POINT - 2; k > 0; k--)//后移,从最后一个开始移动
									SelectNcc[k + 1] = SelectNcc[k];
								SelectNcc[1] = TempPoint;
							}
						}
						// = 更新阈值 = //
						nSubSquareSumThreshold = (unsigned int)(SquareFunc(nTemplateWidth * nTemplateHeight) * (256.0 - SelectNcc[LEVEL3POINT - 1].score));
					}
					else if (NumCounter < LEVEL3POINT)//数组未填满
					{
						if (NumCounter == 0)//第一次向数组填入数据
						{
							SelectNcc[0].score = NCC;
							SelectNcc[0].height = h;
							SelectNcc[0].width = w;
						}
						else
						{
							for (l = 0; l < NumCounter; l++)
							{
								if (NCC > SelectNcc[l].score)//如果当前匹配度有比数组中大的
								{
									TempPoint = SelectNcc[l];
									SelectNcc[l].score = NCC;
									SelectNcc[l].height = h;
									SelectNcc[l].width = w;
									for (k = NumCounter - 1; k > l; k--)//后移,从最后一个开始移动
										SelectNcc[k + 1] = SelectNcc[k];
									SelectNcc[l + 1] = TempPoint;
									l = NumCounter; //结束循环,不能用break
								}
								if (l == NumCounter - 1)//当前数组中最小的一个
								{
									SelectNcc[l + 1].score = NCC;
									SelectNcc[l + 1].height = h;
									SelectNcc[l + 1].width = w;
								}
							}
						}
						NumCounter++;
					}
					else;//排序结束
				}
			}
		}
	
		// = 检查NCC数值是否全部小于阈值 = //
		for (k=0;k<LEVEL3POINT;k++)
		{
			// = 若有一个NCC大于阈值，中断循环，下一层金字塔不需要全局匹配 = //
			if (SelectNcc[k].score>0)
			{
				bGlobalMatch=false;
				break;
			}
		}

		// = 若全部NCC小于阈值，则进行全局匹配 = //
		// = 重置NCC数组 = //
		if (bGlobalMatch==true)
		{
			for (k=0;k<LEVEL3POINT;k++)//复位结构数组
				SelectNcc[k].score=0.0;
		}
	}
	else if(nTemplateHeight >= nTemplateHeightThreshold&&
		nTemplateWidth >= nTemplateWidthThreshold)
	{
		nWidth=pMatTemplate16->width;
		nHeight=pMatTemplate16->height;

		pIData=pMatImage16->data.ptr;
		nIstep=pMatImage16->step;
		pTData=pMatTemplate16->data.ptr;
		nTstep=pMatTemplate16->step;
		TempPoint.score=-1;
		NumCounter=0;
		
		// = 将1/32图像的坐标反映到1/16图像上 = //
		for (k=0;k<LEVEL3POINT;k++)
		{
			h=2*SelectNcc[k].height;
			w=2*SelectNcc[k].width;
			for (i=0;i<2;i++)
			{
				for (j=0;j<2;j++)
				{
					// ReSelectNcc数组保存的是1/16图像的坐标 //
					if(((w+j+nWidth)<=pMatImage16->width)&&((h+i+nHeight)<=pMatImage16->height))
					{
						ReSelectNcc[NumCounter].score=0.0;
						ReSelectNcc[NumCounter].width=w+j;
						ReSelectNcc[NumCounter].height=h+i;
						NumCounter++;
					}
				}
			}
		}
		// = 对每个点计算NCC，求出NCC最大的点 = //
		for (k=0;k<NumCounter;k++)
		{
			h=ReSelectNcc[k].height;
			w=ReSelectNcc[k].width;
			nSumIM=0;
			nSquareSumImage=0;
			nSumGrayImage=0;
			for (n=0;n<nHeight;n++)
			{
				nTemp1=(h+n)*nIstep+w;
				nTemp2=n*nTstep;
				for (m=0;m<nWidth;m++)
				{
					nGray=pIData[nTemp1+m];
					nSumGrayImage+=nGray;
					nSquareSumImage+=nGray*nGray;
					nSumIM+=nGray*(pTData[m+nTemp2]);
				}
			}
			fNsumIM=(double)m_nTemplateSize4*nSumIM;
			fNSquareSumImage=(double)m_nTemplateSize4*nSquareSumImage;
			fSumSquareImage=(double)nSumGrayImage*nSumGrayImage;
			temp=(double)(fNSquareSumImage-fSumSquareImage)*(m_NSquareSumTemplate4-m_SumSquareTemplate4);
			if(temp<0) temp=-temp;
			if(temp==0)
				NCC=0.0;
			else
				NCC=SquareFunc(fNsumIM-(double)nSumGrayImage*m_nSumGrayTemplate4)/temp;
			if(NCC>TempPoint.score)
			{	
				TempPoint.score=NCC;
				TempPoint.width=w;
				TempPoint.height=h;
			}
		}

		// = 若最大NCC值小于阈值，则重置NCC备选点 = //
		if (NCC<nNccThreshold)
			bGlobalMatch=true;

		// = 重置NCC数组 = //
		if (bGlobalMatch==true)
		{
			for (k=0;k<LEVEL3POINT;k++)//复位结构数组
				SelectNcc[k].score=0.0;
		}
		else
		{
			// = 更新SelectPoint数组 = //
			for (k=0;k<LEVEL3POINT;k++)
			{
				SelectNcc[k].height=2*SelectNcc[k].height;
				SelectNcc[k].width=2*SelectNcc[k].width;
			}
		}
	}
	else
		bGlobalMatch=true;

	// = 金字塔第六层 = //
	// ======================= //
	// 扩展到8层NCC模板匹配
	nTemplateHeight=pMatTemplate8->height;
	nTemplateWidth=pMatTemplate8->width;

	if (nTemplateHeight >= nTemplateHeightThreshold&&
		nTemplateWidth >= nTemplateWidthThreshold&&
		bGlobalMatch)
	{
		nWidth=pMatImage8->width-nTemplateWidth; 
		nHeight=pMatImage8->height-nTemplateHeight;
		
		pIData=pMatImage8->data.ptr;			
		nIstep=pMatImage8->step;			
		pTData=pMatTemplate8->data.ptr;		
		nTstep=pMatTemplate8->step;		
		NumCounter=0;		
		nSubSquareSumThreshold=0xFFFFFFFF;

		for (h=0;h<nHeight;h++)
		{
			for (w=0;w<nWidth;w++)
			{
				nSubSquareSum=0;
				i=1;
				for (n=0;n<nTemplateHeight;n++)
				{
					nTemp1=(h+n)*nIstep+w;		// = 图像的行地址 = //
					nTemp2=n*nTstep;			// = 模板的行地址 = //
					// = 子平方和 = //
					for (m=0;m<nTemplateWidth;m++)
						nSubSquareSum+=SquareFunc(pIData[nTemp1+m]-pTData[m+nTemp2]);
					// = 若子平方和大于阈值，则终止循环 = //
					if(nSubSquareSum>nSubSquareSumThreshold)
					{
						i=0;
						break;//终止条件 
					}
				}
				if(i)
				{	//插值排序 // = 从大到小排序 = //
					NCC=256.0-(double)nSubSquareSum/SquareFunc(nTemplateWidth*nTemplateHeight);
					if ((NCC > SelectNcc[LEVEL3POINT - 1].score) && (NumCounter >= LEVEL3POINT))
					{//如果当前NCC大于数组的最末一个且数组已填满
						for (l = LEVEL3POINT - 2; l > 0; l--)
						{
							// = 若当前NCC小于数组中的某个值，则插入到该值之后 = //
							if (NCC < SelectNcc[l].score)
							{
								TempPoint = SelectNcc[l + 1];
								SelectNcc[l + 1].score = NCC;
								SelectNcc[l + 1].height = h;
								SelectNcc[l + 1].width = w;
								for (k = LEVEL3POINT - 2; k > (l + 1); k--)//后移,从最后一个开始移动
									SelectNcc[k + 1] = SelectNcc[k];
								if ((l + 2) < LEVEL3POINT)
									SelectNcc[l + 2] = TempPoint;
								break;//跳出循环
							}
							// = 若当前NCC大于数组中的最大值，则插入到数组的最前面 = //
							if (NCC > SelectNcc[0].score)
							{
								TempPoint = SelectNcc[0];
								SelectNcc[0].score = NCC;
								SelectNcc[0].height = h;
								SelectNcc[0].width = w;
								for (k = LEVEL3POINT - 2; k > 0; k--)//后移,从最后一个开始移动
									SelectNcc[k + 1] = SelectNcc[k];
								SelectNcc[1] = TempPoint;
							}
						}
						// = 更新阈值 = //
						nSubSquareSumThreshold = (unsigned int)(SquareFunc(nTemplateWidth * nTemplateHeight) * (256.0 - SelectNcc[LEVEL3POINT - 1].score));
					}
					else if (NumCounter < LEVEL3POINT)//数组未填满
					{
						if (NumCounter == 0)//第一次向数组填入数据
						{
							SelectNcc[0].score = NCC;
							SelectNcc[0].height = h;
							SelectNcc[0].width = w;
						}
						else
						{
							for (l = 0; l < NumCounter; l++)
							{
								if (NCC > SelectNcc[l].score)//如果当前匹配度有比数组中大的
								{
									TempPoint = SelectNcc[l];
									SelectNcc[l].score = NCC;
									SelectNcc[l].height = h;
									SelectNcc[l].width = w;
									for (k = NumCounter - 1; k > l; k--)//后移,从最后一个开始移动
										SelectNcc[k + 1] = SelectNcc[k];
									SelectNcc[l + 1] = TempPoint;
									l = NumCounter; //结束循环,不能用break
								}
								if (l == NumCounter - 1)//当前数组中最小的一个
								{
									SelectNcc[l + 1].score = NCC;
									SelectNcc[l + 1].height = h;
									SelectNcc[l + 1].width = w;
								}
							}
						}
						NumCounter++;
					}
					else;//排序结束
				}
			}
		}
	
		// = 检查NCC数值是否全部小于阈值 = //
		for (k=0;k<LEVEL3POINT;k++)
		{
			// = 若有一个NCC大于阈值，中断循环，下一层金字塔不需要全局匹配 = //
			if (SelectNcc[k].score>0)
			{
				bGlobalMatch=false;
				break;
			}
		}

		// = 若全部NCC小于阈值，则进行全局匹配 = //
		// = 重置NCC数组 = //
		if (bGlobalMatch==true)
		{
			for (k=0;k<LEVEL3POINT;k++)//复位结构数组
				SelectNcc[k].score=0.0;
		}
	}
	else if(nTemplateHeight >= nTemplateHeightThreshold&&
		nTemplateWidth >= nTemplateWidthThreshold)	
	{
		nWidth=pMatTemplate8->width;
		nHeight=pMatTemplate8->height;

		pIData=pMatImage8->data.ptr;
		nIstep=pMatImage8->step;
		pTData=pMatTemplate8->data.ptr;
		nTstep=pMatTemplate8->step;
		TempPoint.score=-1;
		NumCounter=0;

		// = 将1/16图像的坐标反映到1/8图像上 = //
		for (k=0;k<LEVEL3POINT;k++)
		{
			h=2*SelectNcc[k].height;
			w=2*SelectNcc[k].width;
			for (i=0;i<2;i++)
			{
				for (j=0;j<2;j++)
				{
					// ReSelectNcc数组保存的是1/64图像的坐标 //
					if(((w+j+nWidth)<=pMatImage64->width)&&((h+i+nHeight)<=pMatImage64->height))
					{
						ReSelectNcc[NumCounter].score=0.0;
						ReSelectNcc[NumCounter].width=w+j;
						ReSelectNcc[NumCounter].height=h+i;
						NumCounter++;
					}
				}
			}
		}
		// = 对每个点计算NCC，求出NCC最大的点 = //
		for (k=0;k<NumCounter;k++)
		{
			h=ReSelectNcc[k].height;
			w=ReSelectNcc[k].width;
			nSumIM=0;
			nSquareSumImage=0;
			nSumGrayImage=0;
			for (n=0;n<nHeight;n++)
			{
				nTemp1=(h+n)*nIstep+w;
				nTemp2=n*nTstep;
				for (m=0;m<nWidth;m++)
				{
					nGray=pIData[nTemp1+m];
					nSumGrayImage+=nGray;
					nSquareSumImage+=nGray*nGray;
					nSumIM+=nGray*(pTData[m+nTemp2]);
				}
			}
			fNsumIM=(double)m_nTemplateSize4*nSumIM;
			fNSquareSumImage=(double)m_nTemplateSize4*nSquareSumImage;
			fSumSquareImage=(double)nSumGrayImage*nSumGrayImage;
			temp=(double)(fNSquareSumImage-fSumSquareImage)*(m_NSquareSumTemplate4-m_SumSquareTemplate4);
			if(temp<0) temp=-temp;
			if(temp==0)
				NCC=0.0;
			else
				NCC=SquareFunc(fNsumIM-(double)nSumGrayImage*m_nSumGrayTemplate4)/temp;
			if(NCC>TempPoint.score)
			{	
				TempPoint.score=NCC;
				TempPoint.width=w;
				TempPoint.height=h;
			}
		}
	
		// = 若最大NCC值小于阈值，则重置NCC备选点 = //
		if (NCC<nNccThreshold)
			bGlobalMatch=true;

		// = 重置NCC数组 = //
		if (bGlobalMatch==true)
		{
			for (k=0;k<LEVEL3POINT;k++)//复位结构数组
				SelectNcc[k].score=0.0;
		}
		else
		{
			// = 更新SelectPoint数组 = //
			for (k=0;k<LEVEL3POINT;k++)
			{
				SelectNcc[k].height=2*SelectNcc[k].height;
				SelectNcc[k].width=2*SelectNcc[k].width;
			}
		}
	}
	else
		bGlobalMatch=true;


	// // ====================== //
	// nWidth=pMatTemplate8->width;
	// nHeight=pMatTemplate8->height;
	// pIData=pMatImage8->data.ptr;
	// nIstep=pMatImage8->step;
	// pTData=pMatTemplate8->data.ptr;
	// nTstep=pMatTemplate8->step;
	// // ======================= //
	// // 扩展到8层NCC模板匹配
	// TempPoint.score=-1;
	// // ======================= //
	// NumCounter=0;
	// // ======================= //
	// // 扩展到8层NCC模板匹配
	// // nSubSquareSumThreshold=0xFFFFFFFF;
	// // ======================= //
	// // = 将1/16图像的坐标反映到1/8图像上 = //
	// for (k=0;k<LEVEL3POINT;k++)
	// {
	// 	h=2*ReSelectNcc[k].height;
	// 	w=2*ReSelectNcc[k].width;
	// 	for (i=0;i<2;i++)
	// 	{
	// 		for (j=0;j<2;j++)
	// 		{
	// 			if(((w+j+nWidth)<=pMatImage4->width)&&((h+i+nHeight)<=pMatImage4->height))
	// 			{
	// 				ReSelectNcc[NumCounter].score=0.0;
	// 				ReSelectNcc[NumCounter].width=w+j;
	// 				ReSelectNcc[NumCounter].height=h+i;
	// 				NumCounter++;
	// 			}
	// 		}
	// 	}
	// }
	// // ======================= //
	// // 扩展到8层NCC模板匹配
	// // // = 遍历图像 = //
	// // for (h=0;h<nHeight;h++)
	// // {
	// // 	for (w=0;w<nWidth;w++)
	// // 	{
	// // 		nSubSquareSum=0;
	// // 		i=1;
	// // 		for (n=0;n<nTemplateHeight;n++)
	// // 		{
	// // 			nTemp1=(h+n)*nIstep+w;		// = 图像的行地址 = //
	// // 			nTemp2=n*nTstep;			// = 模板的行地址 = //
	// // 			// = 子平方和 = //
	// // 			for (m=0;m<nTemplateWidth;m++)
	// // 				nSubSquareSum+=SquareFunc(pIData[nTemp1+m]-pTData[m+nTemp2]);
	// // 			// = 若子平方和大于阈值，则终止循环 = //
	// // 			if(nSubSquareSum>nSubSquareSumThreshold)
	// // 			{
	// // 				i=0;
	// // 				break;//终止条件 
	// // 			}
	// // 		}
	// // 		if(i)
	// // 		{	//插值排序 // = 从大到小排序 = //
	// // 			NCC=256.0-(double)nSubSquareSum/SquareFunc(nTemplateWidth*nTemplateHeight);
	// // 			if ((NCC>SelectNcc[LEVEL3POINT-1].score)&&(NumCounter>=LEVEL3POINT))
	// // 			{//如果当前NCC大于数组的最末一个且数组已填满
	// // 				for (l=LEVEL3POINT-2;l>0;l--)
	// // 				{
	// // 					// = 若当前NCC小于数组中的某个值，则插入到该值之后 = //
	// // 					if (NCC<SelectNcc[l].score)
	// // 					{
	// // 						TempPoint=SelectNcc[l+1];
	// // 						SelectNcc[l+1].score=NCC;
	// // 						SelectNcc[l+1].height=h;
	// // 						SelectNcc[l+1].width=w;
	// // 						for (k=LEVEL3POINT-2;k>(l+1);k--)//后移,从最后一个开始移动
	// // 							SelectNcc[k+1]=SelectNcc[k];				
	// // 						if ((l+2)<LEVEL3POINT)
	// // 							SelectNcc[l+2]=TempPoint;
	// // 						break;//跳出循环
	// // 					}
	// // 					// = 若当前NCC大于数组中的最大值，则插入到数组的最前面 = //
	// // 					if (NCC>SelectNcc[0].score)
	// // 					{
	// // 						TempPoint=SelectNcc[0];
	// // 						SelectNcc[0].score=NCC;
	// // 						SelectNcc[0].height=h;
	// // 						SelectNcc[0].width=w;
	// // 						for (k=LEVEL3POINT-2;k>0;k--)//后移,从最后一个开始移动
	// // 							SelectNcc[k+1]=SelectNcc[k];
	// // 						SelectNcc[1]=TempPoint;
	// // 					}
	// // 				}
	// // 				// = 更新阈值 = //
	// // 				nSubSquareSumThreshold=(unsigned int)(SquareFunc(nTemplateWidth*nTemplateHeight)*(256.0-SelectNcc[LEVEL3POINT-1].score));
	// // 			}
	// // 			else if(NumCounter<LEVEL3POINT)//数组未填满
	// // 			{
	// // 				if (NumCounter==0)//第一次向数组填入数据
	// // 				{
	// // 					SelectNcc[0].score=NCC;
	// // 					SelectNcc[0].height=h;
	// // 					SelectNcc[0].width=w;
	// // 				}
	// // 				else
	// // 				{
	// // 					for (l=0;l<NumCounter;l++)
	// // 					{	
	// // 						if (NCC>SelectNcc[l].score)//如果当前匹配度有比数组中大的
	// // 						{
	// // 							TempPoint=SelectNcc[l];
	// // 							SelectNcc[l].score=NCC;
	// // 							SelectNcc[l].height=h;
	// // 							SelectNcc[l].width=w;
	// // 							for (k=NumCounter-1;k>l;k--)//后移,从最后一个开始移动
	// // 								SelectNcc[k+1]=SelectNcc[k];
	// // 							SelectNcc[l+1]=TempPoint;
	// // 							l=NumCounter; //结束循环,不能用break
	// // 						}
	// // 						if (l==NumCounter-1)//当前数组中最小的一个
	// // 						{
	// // 							SelectNcc[l+1].score=NCC;
	// // 							SelectNcc[l+1].height=h;
	// // 							SelectNcc[l+1].width=w;
	// // 						}
	// // 					}
	// // 				}
	// // 				NumCounter++;
	// // 			}
	// // 			else ;//排序结束
	// // 		}
	// // 	}
	// // }
	// // ======================= //
	// // = 对每个点计算NCC，求出NCC最大的点 = //
	// for (k=0;k<NumCounter;k++)
	// {
	// 	h=ReSelectNcc[k].height;
	// 	w=ReSelectNcc[k].width;
	// 	nSumIM=0;
	// 	nSquareSumImage=0;
	// 	nSumGrayImage=0;
	// 	for (n=0;n<nHeight;n++)
	// 	{
	// 		nTemp1=(h+n)*nIstep+w;
	// 		nTemp2=n*nTstep;
	// 		for (m=0;m<nWidth;m++)
	// 		{
	// 			nGray=pIData[nTemp1+m];
	// 			nSumGrayImage+=nGray;
	// 			nSquareSumImage+=nGray*nGray;
	// 			nSumIM+=nGray*(pTData[m+nTemp2]);
	// 		}
	// 	}
	// 	fNsumIM=(double)m_nTemplateSize4*nSumIM;
	// 	fNSquareSumImage=(double)m_nTemplateSize4*nSquareSumImage;
	// 	fSumSquareImage=(double)nSumGrayImage*nSumGrayImage;
	// 	temp=(double)(fNSquareSumImage-fSumSquareImage)*(m_NSquareSumTemplate4-m_SumSquareTemplate4);
	// 	if(temp<0) temp=-temp;
	// 	if(temp==0)
	// 		NCC=0.0;
	// 	else
	// 		NCC=SquareFunc(fNsumIM-(double)nSumGrayImage*m_nSumGrayTemplate4)/temp;
	// 	if(NCC>TempPoint.score)
	// 	{	
	// 		TempPoint.score=NCC;
	// 		TempPoint.width=w;
	// 		TempPoint.height=h;
	// 	}
	// }
	
// 	timer.EndTime();
// 	cout<<"1/8匹配用时："<<timer.GetTime()<<"ms"<<endl;
// 	timer.StartTime();
//1/4	
	nWidth=pMatTemplate4->width;
	nHeight=pMatTemplate4->height;

	pIData=pMatImage4->data.ptr;
	nIstep=pMatImage4->step;
	pTData=pMatTemplate4->data.ptr;
	nTstep=pMatTemplate4->step;
	TempPoint.score=-1;
	NumCounter=0;
	for (k=0;k<LEVEL3POINT;k++)//将1/8图像的坐标反映(映射)到1/4(高一级分辨率)图像下。
	{
		h=2*SelectNcc[k].height;
		w=2*SelectNcc[k].width;
		for (i=0;i<2;i++)
		{
			for (j=0;j<2;j++)
			{
				if(((w+j+nWidth)<=pMatImage4->width)&&((h+i+nHeight)<=pMatImage4->height))
				{
					ReSelectNcc[NumCounter].score=0.0;
					ReSelectNcc[NumCounter].width=w+j;
					ReSelectNcc[NumCounter].height=h+i;
					NumCounter++;
				}
			}
		}
	}
	//对每个点计算NCC,求出NCC最大的点
	for (k=0;k<NumCounter;k++)
	{
		h=ReSelectNcc[k].height;
		w=ReSelectNcc[k].width;
		nSumIM=0;
		nSquareSumImage=0;
		nSumGrayImage=0;
		for (n=0;n<nHeight;n++)
		{
			nTemp1=(h+n)*nIstep+w;
			nTemp2=n*nTstep;
			for (m=0;m<nWidth;m++)
			{
				nGray=pIData[nTemp1+m];
				nSumGrayImage+=nGray;
				nSquareSumImage+=nGray*nGray;
				nSumIM+=nGray*(pTData[m+nTemp2]);
			}
		}
		fNsumIM=(double)m_nTemplateSize4*nSumIM;
		fNSquareSumImage=(double)m_nTemplateSize4*nSquareSumImage;
		fSumSquareImage=(double)nSumGrayImage*nSumGrayImage;
		temp=(double)(fNSquareSumImage-fSumSquareImage)*(m_NSquareSumTemplate4-m_SumSquareTemplate4);
		if(temp<0) temp=-temp;
		if(temp==0)
			NCC=0.0;
		else
			NCC=SquareFunc(fNsumIM-(double)nSumGrayImage*m_nSumGrayTemplate4)/temp;
		if(NCC>TempPoint.score)
		{	
			TempPoint.score=NCC;
			TempPoint.width=w;
			TempPoint.height=h;
		}
	}
// 	timer.EndTime();
// 	cout<<"1/4匹配用时："<<timer.GetTime()<<"ms"<<endl;
// 	timer.StartTime();
//1/2
	nWidth=pMatTemplate2->width;
	nHeight=pMatTemplate2->height;
	pIData=pMatImage2->data.ptr;
	nIstep=pMatImage2->step;
	pTData=pMatTemplate2->data.ptr;
	nTstep=pMatTemplate2->step;
	MaxNccPoint.score=-1;
	for (k=-1;k<=2;k++)//4×4邻域
	{
		for (l=-1;l<=2;l++)
		{
			h=2*TempPoint.height+k;
			w=2*TempPoint.width+l;
			if((h>=0)&&(w>=0)&&((h+nHeight)<=pMatImage2->height)&&((w+nWidth)<=pMatImage2->width))
			{
				nSumIM=0;
				nSquareSumImage=0;
				nSumGrayImage=0;
				for (n=0;n<nHeight;n++)
				{
					nTemp1=(h+n)*nIstep+w;
					nTemp2=n*nTstep;
					for (m=0;m<nWidth;m++)
					{
						nGray=pIData[nTemp1+m];
						nSumGrayImage+=nGray;
						nSquareSumImage+=nGray*nGray;
						nSumIM+=nGray*(pTData[m+nTemp2]);
					}
				}
				fNsumIM=(double)m_nTemplateSize2*nSumIM;
				fNSquareSumImage=(double)m_nTemplateSize2*nSquareSumImage;
				fSumSquareImage=(double)nSumGrayImage*nSumGrayImage;
				temp=(double)(fNSquareSumImage-fSumSquareImage)*(m_NSquareSumTemplate2-m_SumSquareTemplate2);
				if(temp<0) temp=-temp;
				if(temp==0)
					NCC=0.0;
				else
					NCC=SquareFunc(fNsumIM-(double)nSumGrayImage*m_nSumGrayTemplate2)/temp;
				if(NCC>MaxNccPoint.score)
				{	
					MaxNccPoint.score=NCC;	//此处的坐标不是中心点的坐标，
					MaxNccPoint.width=w;	//而是存像左上角在图像中坐标
					MaxNccPoint.height=h;
				}
			}
		}
	}
// 	timer.EndTime();
// 	cout<<"1/2匹配用时："<<timer.GetTime()<<"ms"<<endl;
// 	timer.StartTime();
//1/1

	// = 对原始图像进行匹配 = //
	TempPoint=MaxNccPoint;
	nWidth=pMatTemplate->width;
	nHeight=pMatTemplate->height;
	pIData=(unsigned char*)ImageData;
	nIstep=widthstep;
	pTData=pMatTemplate->data.ptr;
	nTstep=pMatTemplate->step;
	MaxNccPoint.score=-1;

	int nKeyFlag=1,cx=0,cy=0,nOffsetx=0,nOffsety=0;
	while(nKeyFlag)
	{
		i=0;
		for (k=-1;k<=1;k++)
		{
			for (l=-1;l<=1;l++)
			{
				h=2*TempPoint.height+k+nOffsety;
				w=2*TempPoint.width+l+nOffsetx;
				if((h>=0)&&(w>=0)&&((h+nHeight)<=ImageHeight)&&((w+nWidth)<=ImageWidth))
				{
					nSumIM=0;
					nSquareSumImage=0;
					nSumGrayImage=0;
					for (n=0;n<nHeight;n++)
					{
						nTemp1=(h+n)*nIstep;
						nTemp2=n*nTstep;
						for (m=0;m<nWidth;m++)
						{
							nGray=pIData[nTemp1+ImagenChannels*(w+m)];
							nSumGrayImage+=nGray;
							nSquareSumImage+=nGray*nGray;
							nSumIM+=nGray*(pTData[m+nTemp2]);
						}
					}
					fNsumIM=(double)m_nTemplateSize*nSumIM;
					fNSquareSumImage=(double)m_nTemplateSize*nSquareSumImage;
					fSumSquareImage=(double)nSumGrayImage*nSumGrayImage;
					temp=(double)(fNSquareSumImage-fSumSquareImage)*(m_NSquareSumTemplate-m_SumSquareTemplate);
					if(temp<0) temp=-temp;
					if(temp==0)
						NCC=0.0;
					else
						NCC=SquareFunc(fNsumIM-(double)nSumGrayImage*m_nSumGrayTemplate)/temp;
					if(NCC>MaxNccPoint.score)
					{	
						MaxNccPoint.score=NCC;	//此处的坐标不是中心点的坐标，
						MaxNccPoint.width=w;	//而是存像左上角在图像中坐标
						MaxNccPoint.height=h;
						cx=l;
						cy=k;
					}
					m_fNccArray[i]=NCC;
					i++;
				}
			}
		}
	/*	cout<<"cx:"<<cx<<"  cy:"<<cy<<endl;*/
		if ((cx==0)&&(cy==0))
			nKeyFlag=0;
		else
		{
			nOffsetx+=cx;
			nOffsety+=cy;
			cx=0;
			cy=0;
		}
	/*	cout<<"nOffset:"<<nOffsetx<<"  nOffsety:"<<nOffsety<<endl;*/
	}
	// = 确定最大NCC点 = //
	m_MaxNccPoint=MaxNccPoint;

	//subpixel accuracy	
	// = 亚像素精度计算 = //
	int dementionOfNCC=0;				// = NCC矩阵的维数 = //
	int x1,y1;							// = NCC矩阵的行列号 = //
	Matrix Location_Matrix(9,6);		// = NCC矩阵的位置矩阵 = //
	Matrix NCC_Matrix(m_fNccArray,9);	// = NCC矩阵 = //
	
	for (i=0;i<3;i++){
		for (j=0;j<3;j++)
		{
			x1=MaxNccPoint.height+(i-1);
			y1=MaxNccPoint.width+(j-1);
			
			Location_Matrix[3*i+j][0]=x1*x1;
			Location_Matrix[3*i+j][1]=y1*y1;
			Location_Matrix[3*i+j][2]=x1;
			Location_Matrix[3*i+j][3]=y1;
			Location_Matrix[3*i+j][4]=x1*y1;
			Location_Matrix[3*i+j][5]=1;
		}
	}

	// = 模板匹配模型参数 = //
	Matrix paramat_model=((Location_Matrix.T()*Location_Matrix).Inv())*(Location_Matrix.T())*NCC_Matrix;
	
	// = 模板匹配点的亚像素坐标 = //
	m_Locoftarget[0] =-(2*paramat_model[1][0]*paramat_model[2][0]-paramat_model[3][0]*paramat_model[4][0])		
		               /(4*paramat_model[0][0]*paramat_model[1][0]-paramat_model[4][0]*paramat_model[4][0]);
	m_Locoftarget[1] =-(2*paramat_model[0][0]*paramat_model[3][0]-paramat_model[2][0]*paramat_model[4][0])
		               /(4*paramat_model[0][0]*paramat_model[1][0]-paramat_model[4][0]*paramat_model[4][0]);

	// = 模板匹配成功 = //
	return TRUE;
}

//*********************************************************
//!多目标模板匹配
//!寻找目标图像中的多个目标
// ========================================================= //
//
bool CDBImageProcess::MutiTemplateMatch( char * ImageData,  //目标图像指针
										int ImageWidth,    //目标图像宽
										int ImageHeight,   //目标图像高
									 	int widthstep,     //目标图像行字节数,图像行大小
										int ImagenChannels,//目标图像通道数
										int nMatchNumber)  //在目标图像中要寻找的目标数
{	
	SelectPoint SelectNcc[MUTILEVEL3POINT];
	SelectPoint ReSelectNcc[MUTILEVEL3POINT];
	SelectPoint MaxNccPoint;	
	SelectPoint TempPoint;
	int nWidth=0,nHeight=0,nTemplateWidth=0,nTemplateHeight=0;
	int NumCounter=0,nTemp1=0,nTemp2=0;
	int nSumGrayImage=0;

	double NCC=0.0;
	double fNsumIM=0.0,fNSquareSumImage=0.0,fSumSquareImage=0.0,temp=0.0;
	unsigned int nSumIM=0,nSquareSumImage=0,nGray=0;
	unsigned int nSubSquareSum=0,nSubSquareSumThreshold;
	int nIstep=0,nTstep=0;
	unsigned char* pIData=NULL;
	unsigned char* pTData=NULL;
	int TemplateSize=0;
	unsigned char nGrayValue=0;
	int h=0,w=0,n=0,m=0,l=0,k=0,i=0,j=0;
	int nWidthStep=0;
	int nLevel=8;	// 金字塔层数，但是为什么写到了这个地方？
	unsigned int SumGrayTemplate=0;	

	if((m_pTemplate==NULL)||(ImageData==NULL))
		return FALSE;//未加载模板，退出函数

	pTData=pMatImage2->data.ptr;
	nWidthStep=pMatImage2->width;
	pIData=(unsigned char*)ImageData;
	nWidth=pMatImage2->width;
	nHeight=pMatImage2->height;

	for (h=0,n=0;n<nHeight;h+=2,n++)
	{
		nIstep=h*widthstep;
		nTstep=n*nWidthStep;
		for (w=0,m=0;m<nWidth;w+=2,m++)	(pTData+nTstep)[m]= (pIData +nIstep)[ImagenChannels*w];
	}
	PyramidDown(pMatImage2,pMatImage4,NULL);
	switch (nLevel)
	{
	case 4:
		nTemplateWidth=pMatTemplate4->width;
		nTemplateHeight=pMatTemplate4->height;
		nWidth=pMatImage4->width-nTemplateWidth;
		nHeight=pMatImage4->height-nTemplateHeight;
		TemplateSize=SquareFunc(nTemplateWidth*nTemplateHeight);
		pIData=pMatImage4->data.ptr;
		nIstep=pMatImage4->step;
		pTData=pMatTemplate4->data.ptr;
		nTstep=pMatTemplate4->step;
		NumCounter=0;
		nSubSquareSumThreshold=0xFFFFFFFF;
		for (k=0;k<MUTILEVEL3POINT;k++)
			SelectNcc[k].score=0.0; //Set Zero
		for (h=0;h<pMatFlag4->height;h++)//初始化标志矩阵
		{
			for (w=0;w<pMatFlag4->width;w++) 
				(pMatFlag4->data.ptr+h*pMatFlag4->step)[w]=0;
		}
		for (h=0;h<nHeight;h++)
		{
			for (w=0;w<nWidth;w++)
			{
				nSubSquareSum=0;
				for (n=0;n<nTemplateHeight;n++)
				{
					nTemp1=(h+n)*nIstep+w;
					nTemp2=n*nTstep;
					for (m=0;m<nTemplateWidth;m++)
						nSubSquareSum+=SquareFunc(pIData[nTemp1+m]-pTData[m+nTemp2]);
					if(nSubSquareSum>nSubSquareSumThreshold) break;//终止条件
				}
				NCC=256.0-(double)nSubSquareSum/TemplateSize;//ImageSize为面积的平方
				//插值排序
				//如果当前NCC大于数组的最末一个且数组已填满
				if ((NCC>SelectNcc[MUTILEVEL3POINT-1].score)&&(NumCounter>=MUTILEVEL3POINT))
				{	
					nSubSquareSumThreshold=(unsigned int)(TemplateSize*(256.0-SelectNcc[MUTILEVEL3POINT-1].score));
					for (l=MUTILEVEL3POINT-2;l>0;l--)
					{
						if (NCC<SelectNcc[l].score)
						{
							TempPoint=SelectNcc[l+1];
							SelectNcc[l+1].score=NCC;
							SelectNcc[l+1].height=h;
							SelectNcc[l+1].width=w;
							for (k=MUTILEVEL3POINT-2;k>(l+1);k--)//后移,从最后一个开始移动
								SelectNcc[k+1]=SelectNcc[k];				
							if ((l+2)<MUTILEVEL3POINT)
								SelectNcc[l+2]=TempPoint;
							break;//跳出循环
						}
						if (NCC>SelectNcc[0].score)
						{
							TempPoint=SelectNcc[0];
							SelectNcc[0].score=NCC;
							SelectNcc[0].height=h;
							SelectNcc[0].width=w;
							for (k=MUTILEVEL3POINT-2;k>0;k--)//后移,从最后一个开始移动
								SelectNcc[k+1]=SelectNcc[k];
							SelectNcc[1]=TempPoint;
						}
					}
				}
				else if(NumCounter<MUTILEVEL3POINT)//数组未填满
				{
					if (NumCounter==0)//第一次向数组填入数据
					{
						SelectNcc[0].score=NCC;
						SelectNcc[0].height=h;
						SelectNcc[0].width=w;
					}
					else
					{
						for (l=0;l<NumCounter;l++)
						{	
							if (NCC>SelectNcc[l].score)//如果当前匹配度有比数组中大的
							{
								TempPoint=SelectNcc[l];
								SelectNcc[l].score=NCC;
								SelectNcc[l].height=h;
								SelectNcc[l].width=w;
								for (k=NumCounter-1;k>l;k--)//后移,从最后一个开始移动
									SelectNcc[k+1]=SelectNcc[k];
								SelectNcc[l+1]=TempPoint;
								l=NumCounter; //结束循环,不能用break
							}
							if (l==NumCounter-1)//当前数组中最小的一个
							{
								SelectNcc[l+1].score=NCC;
								SelectNcc[l+1].height=h;
								SelectNcc[l+1].width=w;
							}
						}
					}
					NumCounter++;
				}
				else ;
				//排序结束
			}
		}

		pIData=pMatFlag4->data.ptr;
		nIstep=pMatFlag4->step;
		for (k=0;k<MUTILEVEL3POINT;k++)
			(pIData+SelectNcc[k].height*nIstep)[SelectNcc[k].width]=k;
	
		for (k=0;k<MUTILEVEL3POINT;k++)
		{
			if (SelectNcc[k].score>0)
			{
				TempPoint=SelectNcc[k];
				nWidth=SelectNcc[k].width;
				nHeight=SelectNcc[k].height;
				nTemplateWidth=pMatTemplate4->height;
				nTemplateHeight=pMatTemplate4->width;
				for (h=(nHeight-nTemplateHeight/2);h<(nHeight+nTemplateHeight/2);h++)
				{
					for (w=(nWidth-nTemplateWidth/2);w<(nWidth+nTemplateWidth/2);w++)
					{
						if((h>=0)&&(w>=0)&&(h<pMatFlag4->height)&&(w<pMatFlag4->width))
						{
							if((pIData+h*nIstep)[w]!=0)
								SelectNcc[(pIData+h*nIstep)[w]].score=0;
						}
					}
				}
				ReSelectNcc[k]=TempPoint;
			}	
		}
		break;
	case 8:
		PyramidDown(pMatImage4,pMatImage8,NULL);
		nTemplateWidth=pMatTemplate8->width;
		nTemplateHeight=pMatTemplate8->height;
		nWidth=pMatImage8->width-nTemplateWidth;
		nHeight=pMatImage8->height-nTemplateHeight;
		TemplateSize=SquareFunc(nTemplateWidth*nTemplateHeight);
		pIData=pMatImage8->data.ptr;
		nIstep=pMatImage8->step;
		pTData=pMatTemplate8->data.ptr;
		nTstep=pMatTemplate8->step;
		NumCounter=0;
		nSubSquareSumThreshold=0xFFFFFFFF;
		for (k=0;k<MUTILEVEL3POINT;k++)
			SelectNcc[k].score=0.0; //Set Zero
		for (h=0;h<pMatFlag8->height;h++)//初始化标志矩阵
		{
			for (w=0;w<pMatFlag8->width;w++)
			{
				(pMatFlag8->data.ptr+h*pMatFlag8->step)[w]=0;
			}
		}
		for (h=0;h<nHeight;h++)
		{
			for (w=0;w<nWidth;w++)
			{
				nSubSquareSum=0;
				for (n=0;n<nTemplateHeight;n++)
				{
					nTemp1=(h+n)*nIstep+w;
					nTemp2=n*nTstep;
					for (m=0;m<nTemplateWidth;m++)
						nSubSquareSum+=SquareFunc(pIData[nTemp1+m]-pTData[m+nTemp2]);
					if(nSubSquareSum>nSubSquareSumThreshold) break;//终止条件
				}
				NCC=256.0-(double)nSubSquareSum/TemplateSize;//ImageSize为面积的平方
				//插值排序
				//如果当前NCC大于数组的最末一个且数组已填满
				if ((NCC>SelectNcc[MUTILEVEL3POINT-1].score)&&(NumCounter>=MUTILEVEL3POINT))
				{	
					nSubSquareSumThreshold=(unsigned int)(TemplateSize*(256.0-SelectNcc[MUTILEVEL3POINT-1].score));
					for (l=MUTILEVEL3POINT-2;l>0;l--)
					{
						if (NCC<SelectNcc[l].score)
						{
							TempPoint=SelectNcc[l+1];
							SelectNcc[l+1].score=NCC;
							SelectNcc[l+1].height=h;
							SelectNcc[l+1].width=w;
							for (k=MUTILEVEL3POINT-2;k>(l+1);k--)//后移,从最后一个开始移动
								SelectNcc[k+1]=SelectNcc[k];				
							if ((l+2)<MUTILEVEL3POINT)
								SelectNcc[l+2]=TempPoint;
							break;//跳出循环
						}
						if (NCC>SelectNcc[0].score)
						{
							TempPoint=SelectNcc[0];
							SelectNcc[0].score=NCC;
							SelectNcc[0].height=h;
							SelectNcc[0].width=w;
							for (k=MUTILEVEL3POINT-2;k>0;k--)//后移,从最后一个开始移动
								SelectNcc[k+1]=SelectNcc[k];
							SelectNcc[1]=TempPoint;
						}
					}
				}
				else if(NumCounter<MUTILEVEL3POINT)//数组未填满
				{
					if (NumCounter==0)//第一次向数组填入数据
					{
						SelectNcc[0].score=NCC;
						SelectNcc[0].height=h;
						SelectNcc[0].width=w;
					}
					else
					{
						for (l=0;l<NumCounter;l++)
						{	
							if (NCC>SelectNcc[l].score)//如果当前匹配度有比数组中大的
							{
								TempPoint=SelectNcc[l];
								SelectNcc[l].score=NCC;
								SelectNcc[l].height=h;
								SelectNcc[l].width=w;
								for (k=NumCounter-1;k>l;k--)//后移,从最后一个开始移动
									SelectNcc[k+1]=SelectNcc[k];
								SelectNcc[l+1]=TempPoint;
								l=NumCounter; //结束循环,不能用break
							}
							if (l==NumCounter-1)//当前数组中最小的一个
							{
								SelectNcc[l+1].score=NCC;
								SelectNcc[l+1].height=h;
								SelectNcc[l+1].width=w;
							}
						}
					}
					NumCounter++;
				}
				else ;
				//排序结束
			}
		}

		pIData=pMatFlag8->data.ptr;
		nIstep=pMatFlag8->step;
		for (k=0;k<MUTILEVEL3POINT;k++)
		{
			(pIData+SelectNcc[k].height*nIstep)[SelectNcc[k].width]=k;
		}
		for (k=0;k<MUTILEVEL3POINT;k++)
		{
			if (SelectNcc[k].score>0)
			{
				TempPoint=SelectNcc[k];
				nWidth=SelectNcc[k].width;
				nHeight=SelectNcc[k].height;
				nTemplateWidth=pMatTemplate8->height;
				nTemplateHeight=pMatTemplate8->width;
				for (h=(nHeight-nTemplateHeight/2);h<(nHeight+nTemplateHeight/2);h++)
				{
					for (w=(nWidth-nTemplateWidth/2);w<(nWidth+nTemplateWidth/2);w++)
					{
						if((h>=0)&&(w>=0)&&(h<pMatFlag8->height)&&(w<pMatFlag8->width))
						{
							if((pIData+h*nIstep)[w]!=0)
								SelectNcc[(pIData+h*nIstep)[w]].score=0;
						}
					}
				}
				SelectNcc[k]=TempPoint;
			}	
		}

	//1/4>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
		nWidth=pMatTemplate4->width;
		nHeight=pMatTemplate4->height;
		TemplateSize=SquareFunc(pMatTemplate4->width*pMatTemplate4->height);
		pIData=pMatImage4->data.ptr;
		nIstep=pMatImage4->step;
		pTData=pMatTemplate4->data.ptr;
		nTstep=pMatTemplate4->step;
		NumCounter=0;
		for (k=0;k<MUTILEVEL3POINT;k++)
		{
			if(SelectNcc[k].score>0)
			{	
				MaxNccPoint.score=256.0;
				for (j=-2;j<=3;j++)//4×4邻域
				{
					for (i=-2;i<=3;i++)
					{
						h=2*SelectNcc[k].height+j;
						w=2*SelectNcc[k].width+i;
						if((h>=0)&&(w>=0)&&((h+nHeight)<=pMatImage2->height)&&((w+nWidth)<=pMatImage2->width))
						{
							nSubSquareSum=0;
							for (n=0;n<nHeight;n++)
							{
								nTemp1=(h+n)*nIstep+w;
								nTemp2=n*nTstep;
								for (m=0;m<nWidth;m++)
									nSubSquareSum+=SquareFunc(pIData[nTemp1+m]-pTData[m+nTemp2]);
							}
							NCC=(double)nSubSquareSum/TemplateSize;//ImageSize为面积的平方
							if(NCC<MaxNccPoint.score)
							{	
								MaxNccPoint.score=NCC;
								MaxNccPoint.width=w;
								MaxNccPoint.height=h;
							}
						}
					}
				}
				ReSelectNcc[NumCounter]=MaxNccPoint;
				NumCounter++;
			}
		}
		break;
	default:
		break;
	}

//1/2>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	nWidth=pMatTemplate2->width;
	nHeight=pMatTemplate2->height;
	TemplateSize=SquareFunc(pMatTemplate2->width*pMatTemplate2->height);
	pIData=pMatImage2->data.ptr;
	nIstep=pMatImage2->step;
	pTData=pMatTemplate2->data.ptr;
	nTstep=pMatTemplate2->step;
	for (k=0;k<NumCounter;k++)
	{
		MaxNccPoint.score=256.0;
		for (j=-1;j<=2;j++)//4×4邻域
		{
			for (i=-1;i<=2;i++)
			{
				h=2*ReSelectNcc[k].height+j;
				w=2*ReSelectNcc[k].width+i;
				if((h>=0)&&(w>=0)&&((h+nHeight)<=pMatImage2->height)&&((w+nWidth)<=pMatImage2->width))
				{
					nSubSquareSum=0;
					for (n=0;n<nHeight;n++)
					{
						nTemp1=(h+n)*nIstep+w;
						nTemp2=n*nTstep;
						for (m=0;m<nWidth;m++)
							nSubSquareSum+=SquareFunc(pIData[nTemp1+m]-pTData[m+nTemp2]);
					}
					NCC=(double)nSubSquareSum/TemplateSize;//ImageSize为面积的平方
					if(NCC<MaxNccPoint.score)
					{	
						MaxNccPoint.score=NCC;
						MaxNccPoint.width=w;
						MaxNccPoint.height=h;
					}
				}
			}
		}
		SelectNcc[k]=MaxNccPoint;
	}
//1/1>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	nWidth=pMatTemplate->width;
	nHeight=pMatTemplate->height;
	TemplateSize=nWidth*nHeight;
	pIData=(unsigned char*)ImageData;
	nIstep=widthstep;
	pTData=pMatTemplate->data.ptr;
	nTstep=pMatTemplate->step;
	//重新排序
	l=0;
	for (k=l;k<NumCounter-1;k++)
	{
		for (i=l+1;i<NumCounter;i++)
		{
			if (SelectNcc[k].score<SelectNcc[i].score);
			else
			{
				TempPoint=SelectNcc[k];
				SelectNcc[k]=SelectNcc[i];
				SelectNcc[i]=TempPoint;
			}
		}
		l++;
	}
	if(NumCounter>nMatchNumber)//如果选出的点数大于设定的点数。取设定的点数。
		NumCounter=nMatchNumber;
	for (k=0;k<NumCounter;k++)
	{
		MaxNccPoint.score=0.0;
		for (j=-1;j<=1;j++)
		{
			for (i=-1;i<=1;i++)
			{
				h=2*SelectNcc[k].height+j;
				w=2*SelectNcc[k].width+i;
				if((h>=0)&&(w>=0)&&((h+nHeight)<=ImageHeight)&&((w+nWidth)<=ImageWidth))
				{
					nSumIM=0;
					nSquareSumImage=0;
					nSumGrayImage=0;
					for (n=0;n<nHeight;n++)
					{
						nTemp1=(h+n)*nIstep;
						nTemp2=n*nTstep;
						for (m=0;m<nWidth;m++)
						{
							nGray=pIData[nTemp1+ImagenChannels*(w+m)];
							nSumGrayImage+=nGray;
							nSquareSumImage+=nGray*nGray;
							nSumIM+=nGray*(pTData[m+nTemp2]);
						}
					}
					fNsumIM=(double)m_nTemplateSize*nSumIM;
					fNSquareSumImage=(double)m_nTemplateSize*nSquareSumImage;
					fSumSquareImage=(double)nSumGrayImage*nSumGrayImage;
					temp=(double)(fNSquareSumImage-fSumSquareImage)*(m_NSquareSumTemplate-m_SumSquareTemplate);
					if(temp<0) temp=-temp;
					if(temp==0)
						NCC=0.0;
					else
						NCC=SquareFunc(fNsumIM-(double)nSumGrayImage*m_nSumGrayTemplate)/temp;
					if(NCC>MaxNccPoint.score)
					{	
						MaxNccPoint.score=NCC;	//此处的坐标不是中心点的坐标，
						MaxNccPoint.width=w;	//而是存像左上角在图像中坐标
						MaxNccPoint.height=h;
					}
				}
			}
		}
		m_SelectNcc[k]=MaxNccPoint;
	}
	m_nMutiNumber=NumCounter;
	m_MaxNccPoint=m_SelectNcc[0];



	return TRUE;
}

// = 绘制模板匹配的结果 = //
void CDBImageProcess::DrawPattern(IplImage* pTarget,IplImage* pTemplate,SelectPoint MaxNccPoint)
{
	if(m_pTemplate!=NULL)
	{
		if(MaxNccPoint.score>=NCCTHRESHOLD8)//如果大于阈值，画出找到的图像
		{
			cvRectangle( pTarget,
				cvPoint(MaxNccPoint.width,MaxNccPoint.height),
				cvPoint((MaxNccPoint.width+pTemplate->width),(MaxNccPoint.height+pTemplate->height)),
				CV_RGB(102,255,78),1,8,0); //(102,255,78)
			cvLine( pTarget,
				cvPoint(MaxNccPoint.width+pTemplate->width/2,MaxNccPoint.height+pTemplate->height/2-LINELENGTH),
				cvPoint(MaxNccPoint.width+pTemplate->width/2,(LINELENGTH+MaxNccPoint.height+pTemplate->height/2)),
				CV_RGB(102,255,78),1,8,0);
			cvLine( pTarget,
				cvPoint(MaxNccPoint.width+pTemplate->width/2-LINELENGTH,MaxNccPoint.height+pTemplate->height/2),
				cvPoint(LINELENGTH+MaxNccPoint.width+pTemplate->width/2,(MaxNccPoint.height+pTemplate->height/2)),
				CV_RGB(102,255,78),1,8,0);
		}
		else
		{
			cvRectangle( pTarget,
				cvPoint(MaxNccPoint.width,MaxNccPoint.height),
				cvPoint((MaxNccPoint.width+pTemplate->width),(MaxNccPoint.height+pTemplate->height)),
				CV_RGB(255,30,78),1,8,0);
			cvLine( pTarget,
				cvPoint(MaxNccPoint.width+pTemplate->width/2,MaxNccPoint.height+pTemplate->height/2-LINELENGTH),
				cvPoint(MaxNccPoint.width+pTemplate->width/2,(LINELENGTH+MaxNccPoint.height+pTemplate->height/2)),
				CV_RGB(255,30,78),1,8,0);
			cvLine( pTarget,
				cvPoint(MaxNccPoint.width+pTemplate->width/2-LINELENGTH,MaxNccPoint.height+pTemplate->height/2),
				cvPoint(LINELENGTH+MaxNccPoint.width+pTemplate->width/2,(MaxNccPoint.height+pTemplate->height/2)),
				CV_RGB(255,30,78),1,8,0);
		}
	}
}

// = 绘制多模板匹配的结果 = //
void CDBImageProcess::DrawMutiPattern(IplImage* pTarget,IplImage* pTemplate)
{
	SelectPoint MaxNccPoint;
	if(m_pTemplate!=NULL)
	{
		for (int k=0;k<m_nMutiNumber;k++)
		{
			MaxNccPoint=m_SelectNcc[k];
			if(MaxNccPoint.score>=NCCTHRESHOLD8)//如果大于阈值，画出找到的图像
			{
				cvRectangle( pTarget,
					cvPoint(MaxNccPoint.width,MaxNccPoint.height),
					cvPoint((MaxNccPoint.width+pTemplate->width),(MaxNccPoint.height+pTemplate->height)),
					CV_RGB(102,255,78),1,8,0);
				cvLine( pTarget,
					cvPoint(MaxNccPoint.width+pTemplate->width/2,MaxNccPoint.height+pTemplate->height/2-LINELENGTH),
					cvPoint(MaxNccPoint.width+pTemplate->width/2,(LINELENGTH+MaxNccPoint.height+pTemplate->height/2)),
					CV_RGB(102,255,78),1,8,0);
				cvLine( pTarget,
					cvPoint(MaxNccPoint.width+pTemplate->width/2-LINELENGTH,MaxNccPoint.height+pTemplate->height/2),
					cvPoint(LINELENGTH+MaxNccPoint.width+pTemplate->width/2,(MaxNccPoint.height+pTemplate->height/2)),
					CV_RGB(102,255,78),1,8,0);
			}
			else
			{
				cvRectangle( pTarget,
					cvPoint(MaxNccPoint.width,MaxNccPoint.height),
					cvPoint((MaxNccPoint.width+pTemplate->width),(MaxNccPoint.height+pTemplate->height)),
					CV_RGB(255,30,78),1,8,0);
				cvLine( pTarget,
					cvPoint(MaxNccPoint.width+pTemplate->width/2,MaxNccPoint.height+pTemplate->height/2-LINELENGTH),
					cvPoint(MaxNccPoint.width+pTemplate->width/2,(LINELENGTH+MaxNccPoint.height+pTemplate->height/2)),
					CV_RGB(255,30,78),1,8,0);
				cvLine( pTarget,
					cvPoint(MaxNccPoint.width+pTemplate->width/2-LINELENGTH,MaxNccPoint.height+pTemplate->height/2),
					cvPoint(LINELENGTH+MaxNccPoint.width+pTemplate->width/2,(MaxNccPoint.height+pTemplate->height/2)),
					CV_RGB(255,30,78),1,8,0);
			}
		}
	}
}

//***************************************************
CHTimer::CHTimer()
{
	m_lStartTime=0;
	m_lEndTime=0;
	m_lPersecond=0;
	m_fTime=0.0;
	QueryPerformanceFrequency((LARGE_INTEGER *)&m_lPersecond);//询问系统一秒钟的频率
}

//***************************************************
CHTimer::~CHTimer()
{
	
}

//***************************************************
//启动计时
void CHTimer::StartTime()
{
	QueryPerformanceCounter((LARGE_INTEGER *)&m_lStartTime);
}

//***************************************************
//停止计时
void CHTimer::EndTime()
{
	QueryPerformanceCounter((LARGE_INTEGER *)&m_lEndTime);
}

//***************************************************
//获得计时时间(ms),以毫秒为单位
double CHTimer::GetTime()
{
	m_fTime=(double)(m_lEndTime-m_lStartTime)/m_lPersecond;
	m_fTime=m_fTime*1000;
	return m_fTime;
}
