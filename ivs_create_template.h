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

// ģ���ӽṹ������ĳһ��ĳһ���Ƕȵ�ģ����Ϣ
struct IVSTemplateSubStruct
{
	/* ģ���������ļ������ʽ���������
	* 1. ģ�������б�Ե���λ�ã�
	* 2. ģ�������б�Ե����ݶ�ֵ��
	* 3. ģ�������б�Ե����X��Y�����ϵ��ݶȷ���
	* 4. ģ���б�Ե��ĸ�����
	* 5. ģ��ĸ߿�
	* 6. ģ������ġ�
	* */
	UINT8						modelDefined;
	UINT32						noOfCordinates;		//Number of elements in coordinate array ��Ե��ĸ���
	UINT16						modelHeight;		//Template height ģ��ĸ߶�
	UINT16						modelWidth;			//Template width ģ��Ŀ��
	cv::Point					centerOfGravity;	//Center of gravity of template ����
	std::vector<cv::Point>		cordinates;			//Coordinates array to store mo hjel points	model points Ҳ�������еı�Ե��
	std::vector<float>			edgeDerivativeX;	//gradient in X direction
	std::vector<float>			edgeDerivativeY;	//gradient in Y direction
};


// ģ��ṹ����������ģ��
struct IVSTemplateStruct {
	UINT8											pyramidLevelNum;
	std::vector<float>								searchAngelStep;
	std::vector<UINT16>								searchRectWidth;
	std::vector<std::vector<IVSTemplateSubStruct>>	tpls;
};

/*
* @param yuv�Ǵ�������������yuvͼ��koyo_tool_contour_paramter, ��ԭ�ȶ���õĲ����������ԭ�������������Ĳ���һ������Ҫ�����úõ�
* @return ����ֵ����Ҫ�򴫸������͵�buf��������ָ�룬����ȥ�Ժ��ɵ���create_template�ĺ��������ͷš�
* */
int ivs_create_template(const UINT8 *yuv, IVSToolContourParameter *ivsToolContourParameter);
int ivs_get_contours(const UINT8 *yuv, UINT8 *contours, int low_threshold, int high_threshold);

#endif //IVS_CREATE_TEMPLATE_H
