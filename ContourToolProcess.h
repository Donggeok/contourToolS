#ifndef _CONTOURTOOLPROCESS_H
#define _CONTOURTOOLPROCESS_H

#include "ivs_parameter.h"

class ContourToolProcess
{
private:
	IVSToolContourParameter toolParameter;

private:
	int do_create_template(Koyo_Contour_Template_Runtime_Param *koyo_contour_template_runtime_param, char *buf);
	int create_template(Alg_Handle *palg_handle, char *buf);
	INT8 match_template_one_level(IVE_IMAGE_S *search_region_bin, IVE_IMAGE_S *dilated_search_region_bin, Koyo_Template_Match_Info *template_match_info,
		CandidateResultMinHeapStruct *candidate_heap_in,
		CandidateResultMinHeapStruct *candidate_heap_out,
		Tool_Utility *tool_utility, UINT8 level, Pic_Corr *pic_corr);
	INT8 match_template_one_degree(Koyo_Template_Match_Info *template_info, IVE_IMAGE_S *search_region_bin,
		CandidateResult *candidateResult, Tool_Utility *tool_utility, float rate);


public:
	ContourToolProcess(const IVSToolContourParameter parameter);
	~ContourToolProcess(){}
	// ��ʼ������
	int initTask();
	// ִ�����񲢷��ؽ��
	int doTask(IVSContourResult&){
		// ���ݲ�����Ϣ��������Ӧ�Ľ������䵽Width_Result.   
	};
	// �ͷ�����
	int freeTask();
};

#endif