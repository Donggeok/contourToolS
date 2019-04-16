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
	int angleIndex;			// 角度下标
	int positionX;			// 质心x坐标
	int positionY;			// 质心y坐标
	float score;			// 该处位置评分

	bool operator<(const CandidateResult other) const{
		return score > other.score;
	}
};

class ContourToolProcess
{
private:
	IVSToolContourParameter toolParameter;
	IVSTemplateStruct templateInfo;
	std::vector<cv::Rect> searchRect;						// 代表搜索区域（全局和局部）
	std::vector<cv::Mat> cannyPyramid;						// 每个工具独有的canny金字塔
	std::vector<cv::Mat> cannyDilatePyramid;				// 每个工具独有的膨胀canny金字塔
	std::vector<std::priority_queue<CandidateResult>> candidates;	// 每一层的候选位置信息

private:
	int unpackTemplate(char* buf);
	void print_tpl(const IVSTemplateSubStruct &tpl);
	INT8 doTemplateMatch(ContourUtility &contourUtility);
	int doTopLayerMatch(ContourUtility &contourUtility, float threshold_S);
	int doOtherLayerMatch(ContourUtility &contourUtility, int cur_level, float threshold_S);


public:
	ContourToolProcess(const IVSToolContourParameter parameter);
	~ContourToolProcess(){}
	// 初始化任务
	int initTask(ContourUtility &contourUtility);
	// 执行任务并返回结果
	int doTask(ContourUtility &contourUtility, IVSContourResult& contourResult);
	// 释放任务
	int freeTask();
};

#endif