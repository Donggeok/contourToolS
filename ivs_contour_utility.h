#ifndef IVS_CONTOUR_UTILITY_H
#define IVS_CONTOUR_UTILITY_H

#include <opencv.hpp>
#include <ocl\ocl.hpp>
#include "ivs_parameter.h"

#define MAX_NUM_PYRAMID 6

/* 梯度、积分图、二值图*/  //每张待测图片不一样，需要在处理进程中重新计算
struct ContourUtility {
	cv::Mat srcFiltered;								// 原图
	cv::Mat gradx[MAX_NUM_PYRAMID];						// 待测图各层x方向梯度图
	cv::Mat grady[MAX_NUM_PYRAMID];						// 待测图各层y方向梯度图
	cv::Mat searchRegion[MAX_NUM_PYRAMID];				// 降采样图片金字塔
	cv::Mat imageSearchInteg[MAX_NUM_PYRAMID];			// 积分图金字塔，积分图只需要最上面两层(层数可能为4或5)，下层都是使用二值图，0为4层，1为5层

	/* 当前图片归一化的梯度向量表 */
	float **edgeX[MAX_NUM_PYRAMID];
	float **edgeY[MAX_NUM_PYRAMID];

	/* 全图分辨率 */
	UINT16 u16Width;
	UINT16 u16Height;

	/* 查找表*/
	float **lookupTableX;
	float **lookupTableY;
	float **lookupTableS;
};

int createUtility(ContourUtility &contourUtility, int width, int height);
int computeUtility(ContourUtility &contourUtility, IVSOriPic ivsOriPic);
int freeUtility(ContourUtility &contourUtility);




#endif