#ifndef _CONTOURTOOLPROCESS_H
#define _CONTOURTOOLPROCESS_H

#include "ivs_algorithm_utils.h"
#include "ivs_create_template.h"
#include "ivs_contour_utility.h"
#include "ivs_parameter.h"
#include "ivs_contour_config.h"
#include <iostream>
#include <queue>

struct CandidateResult {
	int level;
	int angleIndex;			// �Ƕ��±�
	int positionX;			// ����x����
	int positionY;			// ����y����
	float score;			// �ô�λ������

	bool operator<(const CandidateResult other) const{
		return score > other.score;
	}
};

class ContourToolProcess
{
private:
	IVSToolContourParameter toolParameter;
	IVSTemplateStruct templateInfo;
	std::vector<cv::Rect> searchRect;						// ������������ȫ�ֺ;ֲ���
	std::vector<cv::Mat> cannyPyramid;						// ÿ�����߶��е�canny������
	std::vector<cv::Mat> cannyDilatePyramid;				// ÿ�����߶��е�����canny������
	std::vector<std::priority_queue<CandidateResult>> candidates;	// ÿһ��ĺ�ѡλ����Ϣ

private:
	int unpackTemplate(char* buf);
	void print_tpl(const IVSTemplateSubStruct &tpl);
	INT8 doTemplateMatch(ContourUtility &contourUtility);
	int doTopLayerMatch(ContourUtility &contourUtility, float threshold_S);
	int doOtherLayerMatch(ContourUtility &contourUtility, int cur_level, float threshold_S);


public:
	ContourToolProcess(const IVSToolContourParameter parameter);
	~ContourToolProcess(){}
	// ��ʼ������
	int initTask(ContourUtility &contourUtility);
	// ִ�����񲢷��ؽ��
	int doTask(ContourUtility &contourUtility, IVSContourResult& contourResult);
	// �ͷ�����
	int freeTask();
};

#endif