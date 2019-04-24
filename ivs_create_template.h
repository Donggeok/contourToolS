#ifndef IVS_CREATE_TEMPLATE_H
#define IVS_CREATE_TEMPLATE_H

#include <iostream>
#include <vector>
#include <opencv.hpp>

#include "ivs_parameter.h"

const int MAX_DEGREE = 360;
const int MIN_CONTOUR_PYRA = 50;

#define MAX_NUM_PYRAMID  6
const int MIN_NUM_PYRAMID = 4;
const int HIGH_PRECISION_NUM_PYRAMID = 5;
const double MIN_DIST = 0.0;

const int WIDTH = 1920;
const int HEIGHT = 1080;

// 模板子结构，代表某一层某一个角度的模板信息
struct IVSTemplateSubStruct
{
	/* 模板的由下面的几个性质进行描述：
	* 1. 模板中所有边缘点的位置；
	* 2. 模板中所有边缘点的梯度值；
	* 3. 模板中所有边缘点在X和Y方向上的梯度方向；
	* 4. 模板中边缘点的个数；
	* 5. 模板的高宽；
	* 6. 模板的重心。
	* */
	UINT8						modelDefined;
	UINT32						noOfCordinates;		//Number of elements in coordinate array 边缘点的个数
	UINT16						modelHeight;		//Template height 模板的高度
	UINT16						modelWidth;			//Template width 模板的宽度
	cv::Point					centerOfGravity;	//Center of gravity of template 重心
	std::vector<cv::Point>		cordinates;			//Coordinates array to store mo hjel points	model points 也就是所有的边缘点
	std::vector<float>			edgeDerivativeX;	//gradient in X direction
	std::vector<float>			edgeDerivativeY;	//gradient in Y direction
};


// 模板结构，代表所有模板
struct IVSTemplateStruct {
	UINT8											pyramidLevelNum;
	std::vector<float>								searchAngelStep;
	std::vector<UINT16>								searchRectWidth;
	std::vector<std::vector<IVSTemplateSubStruct>>	tpls;
};

/*
* @param yuv是传感器传上来的yuv图像，koyo_tool_contour_paramter, 是原先定义好的参数，这个和原来传给传感器的参数一样，需要是设置好的
* @return 返回值是需要向传感器发送的buf缓冲区的指针，传过去以后由调用create_template的函数进行释放。
* */
int ivs_create_template(const UINT8 *yuv, IVSToolContourParameter *ivsToolContourParameter);
int ivs_get_contours(const UINT8 *yuv, UINT8 *contours, int low_threshold, int high_threshold);

#endif //IVS_CREATE_TEMPLATE_H
