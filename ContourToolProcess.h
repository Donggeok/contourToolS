#ifndef _CONTOURTOOLPROCESS_H
#define _CONTOURTOOLPROCESS_H

#include "ivs_algorithm_utils.h"
#include "ivs_create_template.h"
#include "ivs_contour_utility.h"
#include "ivs_parameter.h"
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
	std::vector<cv::Rect> searchRegion;						// ������������ȫ�ֺ;ֲ���
	std::vector<cv::Mat> cannyPyramid;						// ÿ�����߶��е�canny������
	std::vector<cv::Mat> cannyDilatePyramid;				// ÿ�����߶��е�����canny������
	std::vector<std::priority_queue<CandidateResult>> candidates;	// ÿһ��ĺ�ѡλ����Ϣ

private:
	int unpackTemplate(char* buf);
	void print_tpl(const IVSTemplateSubStruct &tpl);
	INT8 do_template_match_new(cv::Mat src, ContourUtility &contourUtility);
	int do_top_match(ContourUtility &contourUtility, float threshold_S);
	/*int do_create_template(Koyo_Contour_Template_Runtime_Param *koyo_contour_template_runtime_param, char *buf);
	int create_template(Alg_Handle *palg_handle, char *buf);
	INT8 match_template_one_level(IVE_IMAGE_S *search_region_bin, IVE_IMAGE_S *dilated_search_region_bin, Koyo_Template_Match_Info *template_match_info,
		CandidateResultMinHeapStruct *candidate_heap_in,
		CandidateResultMinHeapStruct *candidate_heap_out,
		Tool_Utility *tool_utility, UINT8 level, Pic_Corr *pic_corr);
	INT8 match_template_one_degree(Koyo_Template_Match_Info *template_info, IVE_IMAGE_S *search_region_bin,
		CandidateResult *candidateResult, Tool_Utility *tool_utility, float rate);*/


public:
	ContourToolProcess(const IVSToolContourParameter parameter);
	~ContourToolProcess(){}
	// ��ʼ������
	int initTask(ContourUtility &contourUtility);
	// ִ�����񲢷��ؽ��
	int doTask(ContourUtility &contourUtility, IVSContourResult& contourResult){
		// ���ݲ�����Ϣ��������Ӧ�Ľ������䵽Width_Result.   
	};
	// �ͷ�����
	int freeTask();
};

#endif