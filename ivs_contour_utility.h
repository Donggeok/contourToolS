#ifndef IVS_CONTOUR_UTILITY_H
#define IVS_CONTOUR_UTILITY_H

#include <opencv.hpp>
#include "ivs_parameter.h"

#define MAX_NUM_PYRAMID 6

/* �ݶȡ�����ͼ����ֵͼ*/  //ÿ�Ŵ���ͼƬ��һ������Ҫ�ڴ�����������¼���
struct ContourUtility {
	cv::Mat srcFiltered;								// ԭͼ
	cv::Mat gradx[MAX_NUM_PYRAMID];						// ����ͼ����x�����ݶ�ͼ
	cv::Mat grady[MAX_NUM_PYRAMID];						// ����ͼ����y�����ݶ�ͼ
	cv::Mat searchRegion[MAX_NUM_PYRAMID];				// ������ͼƬ������
	cv::Mat imageSearchInteg[MAX_NUM_PYRAMID];			// ����ͼ������������ͼֻ��Ҫ����������(��������Ϊ4��5)���²㶼��ʹ�ö�ֵͼ��0Ϊ4�㣬1Ϊ5��


														/* ��ǰλ���Ƿ����� */     //ÿ��ͼƬ�ⲿ����ͬ���㷨�����¸�ֵ���߳�ʼ��Ϊ0,����Ҫ�ڴ�����������¼���
																			//bool **is_calculated[MAX_NUM_PYRAMID];     //��ʾ��ǰͼƬ��һ���ݶ��Ƿ�����
																			//cv::Mat mask_ang_region_idx_u8[MAX_NUM_PYRAMID - 1];  //������������ƥ��ʹ�õĺ�ѡ������ͼ
																			//hash_set_t *phset_coll;    //��ʾ��ǰλ����Ƕ��Ƿ�����������Ƿ����ʹ��һά�ռ䣬�ĸ��ٶȿ죿��  �����һ�飬�̶���С

																			/* ��ǰͼƬ��һ�����ݶ������� */    //���������ݽṹ�Ƿ���������������һ��
	float **edgeX[MAX_NUM_PYRAMID];
	float **edgeY[MAX_NUM_PYRAMID];

	/* ȫͼ�ֱ��� */
	UINT16 u16Width;
	UINT16 u16Height;

	/* ���ұ�*/
	float **lookupTableX;
	float **lookupTableY;
	float **lookupTableS;
};

int createUtility(ContourUtility &contourUtility);
int computeUtility(ContourUtility &contourUtility, IVSOriPic ivsOriPic);
int freeUtility(ContourUtility &contourUtility);




#endif