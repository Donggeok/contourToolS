#ifndef IVS_ALGORITHM_UTILS_H
#define IVS_ALGORITHM_UTILS_H

#include <opencv.hpp>
#include "ivs_parameter.h"

#define MAX_NUM_PYRAMID 6

struct IVSOriPic
{
	UINT32	width;
	UINT32	height;
	UINT32	visize[2];
	UINT32	stride[2];
	UINT32  yuvSize;                //原始图片大小、
	UINT8	payload[0];				//附原始图片,柔性数组
};

struct IVSRgbPic
{
	UINT32  width;
	UINT32  height;
	UINT32  rgbSize;                //RGB编码图片大小
	UINT8	payload[0];				//附编码图片,柔性数组
};

/* 梯度、积分图、二值图*/  //每张待测图片不一样，需要在处理进程中重新计算
struct ContourUtility{
	
	cv::Mat gradx[MAX_NUM_PYRAMID];
	cv::Mat grady[MAX_NUM_PYRAMID];
	cv::Mat cannyGraph[MAX_NUM_PYRAMID];					// 二值化轮廓，canny的结果
	cv::Mat dilatedCannyGraph[MAX_NUM_PYRAMID];			// 膨胀二值化轮廓，canny的结果
	cv::Mat searchRegion[MAX_NUM_PYRAMID];						// 搜索区域的
	cv::Mat imageSearchInteg[MAX_NUM_PYRAMID];					//积分图只需要最上面两层(层数可能为4或5)，下层都是使用二值图，0为4层，1为5层
	cv::Mat srcFiltered;

	/* 当前位置是否计算过 */     //每张图片这部分相同，算法中重新赋值或者初始化为0,不需要在处理进程中重新计算
	bool **is_calculated[MAX_NUM_PYRAMID];     //表示当前图片归一化梯度是否计算过
	cv::Mat mask_ang_region_idx_u8[MAX_NUM_PYRAMID - 1];  //除最顶层外其余层匹配使用的候选点掩码图
	//hash_set_t *phset_coll;    //表示当前位置与角度是否计算过，这块是否可以使用一维空间，哪个速度快？？  这个用一块，固定大小

	/* 当前图片归一化的梯度向量表 */    //这三块数据结构是否所有轮廓工具用一块
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

int createUtility(ContourUtility & contourUtility, IVSOriPic ivsOriPic);
int computeUtility(ContourUtility &);
int freeUtility(ContourUtility &);


#endif