
#include "ContourToolProcess.h"


ContourToolProcess::ContourToolProcess(const IVSToolContourParameter parameter){
	toolParameter = parameter;
}


//ContourToolProcess::~ContourToolProcess(){
//
//}

int ContourToolProcess::unpackTemplate(char* buf)
{

	std::size_t index = 0;

	templateInfo.pyramidLevelNum = *((UINT8 *)&buf[index]);
	index += sizeof(templateInfo.pyramidLevelNum);

	for (int i = 0; i < templateInfo.pyramidLevelNum; ++i) {
		float angle = *((float*)&buf[index]);
		index += sizeof(float);
		templateInfo.searchAngelStep.push_back(angle);
	}

	for (int i = 0; i < templateInfo.pyramidLevelNum; ++i) {
		UINT16 width = *((UINT16*)&buf[index]);
		index += sizeof(UINT16);
		templateInfo.searchRectWidth.push_back(width);
	}

	// ����ģ������
	for (int i = 0; i < templateInfo.pyramidLevelNum; ++i) {
		std::vector<IVSTemplateSubStruct> tpl_arr;

		UINT16 tpl_size = *((UINT16*)&buf[index]);
		index += sizeof(UINT16);

		for (int j = 0; j < tpl_size; ++j) {
			IVSTemplateSubStruct tpl;
			tpl.modelDefined = *((UINT8*)&buf[index]);
			index += sizeof(tpl.modelDefined);

			tpl.noOfCordinates = *((UINT32*)&buf[index]);
			index += sizeof(tpl.noOfCordinates);

			tpl.modelHeight = *((UINT16*)&buf[index]);
			index += sizeof(tpl.modelHeight);

			tpl.modelWidth = *((UINT16*)&buf[index]);
			index += sizeof(tpl.modelWidth);

			tpl.centerOfGravity.x = *((INT16*)&buf[index]);
			index += sizeof(short);

			tpl.centerOfGravity.y = *((INT16*)&buf[index]);
			index += sizeof(short);

			// ������������
			for (std::size_t t = 0; t < tpl.noOfCordinates; ++t) {
				cv::Point point;
				point = *((cv::Point*)&buf[index]);
				index += sizeof(cv::Point);
				tpl.cordinates.push_back(point);
			}

			for (std::size_t t = 0; t < tpl.noOfCordinates; ++t) {
				float edgeX = *((float *)&buf[index]);
				index += sizeof(float);
				tpl.edgeDerivativeX.push_back(edgeX);
			}

			for (std::size_t t = 0; t < tpl.noOfCordinates; ++t) {
				float edgeY = *((float*)&buf[index]);
				index += sizeof(float);
				tpl.edgeDerivativeY.push_back(edgeY);
			}
			tpl_arr.push_back(tpl);
		}
		templateInfo.tpls.push_back(tpl_arr);
	}

	return 0;
}

void ContourToolProcess::print_tpl(const IVSTemplateSubStruct &tpl) {
	cv::Mat tmp(tpl.modelHeight, tpl.modelWidth, CV_8UC1, cv::Scalar(0));
	for (int i = 0; i < tpl.noOfCordinates; ++i) {
		tmp.at<uchar>(tpl.cordinates[i].y + tpl.centerOfGravity.y, tpl.cordinates[i].x + tpl.centerOfGravity.x) = 255;
	}
	cv::imshow("tmp", tmp);
	cv::waitKey(0);
}

int ContourToolProcess::initTask(ContourUtility &contourUtility){
	
	// ����ģ���ļ�·��������ģ���ļ�
	FILE *fp = fopen((const char*)toolParameter.templatePath, "rb");
	fseek(fp, 0L, SEEK_END);
	size_t templateSize = ftell(fp);
	std::cout << "templateSize is" << templateSize << std::endl;
	rewind(fp);
	char *buf = (char *)malloc(sizeof(char)*templateSize);
	fread(buf, sizeof(UINT8), templateSize, fp);


	// ��������
	unpackTemplate(buf);
	
	// �ͷ���Դ
	fclose(fp);
	free(buf);

	/* ����ÿ����������������� */     
	//�˲���ֻ�õ���������������ڼ�����������4������ʱ��Ҫ�õ����Լӿ������ٶ�

	// ���ǵ�֮��ֲ���������Ҳ���ڽ��������⣬���������������ҪΪ������������Χ����ӿ�
	INT16 leftTopX = IVS_MIN<INT16>(4, toolParameter.searchRectX0, toolParameter.searchRectX1,
		toolParameter.searchRectX2, toolParameter.searchRectX3);
	INT16 leftTopY = IVS_MIN<INT16>(4, toolParameter.searchRectY0, toolParameter.searchRectY1,
		toolParameter.searchRectY2, toolParameter.searchRectY3);

	INT16 rightBottomX = IVS_MAX<INT16>(4, toolParameter.searchRectX0, toolParameter.searchRectX1,
		toolParameter.searchRectX2, toolParameter.searchRectX3);

	INT16 rightBottomY = IVS_MAX<INT16>(4, toolParameter.searchRectY0, toolParameter.searchRectY1,
		toolParameter.searchRectY2, toolParameter.searchRectY3);

	// ����һ��������Χ
	cv::Rect rectTmp;
	rectTmp.x = leftTopX;
	rectTmp.y = leftTopY;
	rectTmp.width = rightBottomX - leftTopX;
	rectTmp.height = rightBottomY - leftTopY;
	searchRect.push_back(rectTmp);

	for (int i = 1; i < templateInfo.pyramidLevelNum; ++i) {
		
		// ���ÿһ���������Χ
		rectTmp.x = searchRect[i - 1].x >> 1;
		rectTmp.y = searchRect[i - 1].y >> 1;
		rectTmp.width = searchRect[i - 1].width >> 1;
		rectTmp.height = searchRect[i - 1].height >> 1;
		searchRect.push_back(rectTmp);
	}
	
	// ��ӡ�鿴�����ģ���ļ��Ƿ���ȷ
	//for (int i = 0; i < templateInfo.tpls.size(); ++i) {
	//	for (int j = 0; j < templateInfo.tpls[i].size(); ++j) {
	//		print_tpl(templateInfo.tpls[i][j]);
	//	}
	//}
	
	// �������ͼ���ͼ�������(�������ֶ���contourUtility�����˴�������ֻ����cannyͼ��)
	int lowThreshold = toolParameter.sensiLowThreshold;
	int topThreshold = toolParameter.sensiTopThreshold;

	lowThreshold = lowThreshold > 255 ? 255 : lowThreshold;
	topThreshold = topThreshold > 255 ? 255 : topThreshold;

	for (int i = 0; i < templateInfo.pyramidLevelNum; ++i) {
		// ����cannyͼ���Լ���Ӧ������ͼ�񣬻���ͼͼ��(ֻ����������Χ��ͼ��)
		cv::Mat cannyMat, cannyDilateMat, integralMat;
		cv::Canny(contourUtility.searchRegion[i](searchRect[i]), cannyMat, lowThreshold, topThreshold);
		Dilation(cannyMat, cannyDilateMat, 2);
		cannyPyramid.push_back(cannyMat);
		cannyDilatePyramid.push_back(cannyDilateMat);

		//cv::imshow("cannyMat", cannyMat);
		//cv::imshow("cannyDilateMat", cannyDilateMat);
		//cv::waitKey(0);
		// todo ����ͼ����
	}

	// todo ����һ�´β�����Ȼ�󽵲���������Ĳ���


	return 0;
}

int ContourToolProcess::doTask(ContourUtility &contourUtility, IVSContourResult& ivsContourResult){

	// ����ȷ�����ٶȲ���Ҳ������ײ㣬Ϊ�˸Ķ����٣�ֻ�޸�����������һ
	//INT8 com_level = palg_handle->contour_param->algo_strategy;
	INT8 com_level = 0;
	INT8 com_score = 20;

	doTemplateMatch(contourUtility);

	// ��ʼ������ṹ��
	ivsContourResult.regionShape = toolParameter.regionShape;

	ivsContourResult.detectCircleX = toolParameter.detectCircleX;
	ivsContourResult.detectCircleY = toolParameter.detectCircleY;
	ivsContourResult.detectCircleRadius = toolParameter.detectCircleRadius;

	ivsContourResult.detectRectX0 = toolParameter.detectRectX0;
	ivsContourResult.detectRectY0 = toolParameter.detectRectY0;
	ivsContourResult.detectRectX1 = toolParameter.detectRectX1;
	ivsContourResult.detectRectY1 = toolParameter.detectRectY1;
	ivsContourResult.detectRectX2 = toolParameter.detectRectX2;
	ivsContourResult.detectRectY2 = toolParameter.detectRectY2;
	ivsContourResult.detectRectX3 = toolParameter.detectRectX3;
	ivsContourResult.detectRectY3 = toolParameter.detectRectY3;

	ivsContourResult.searchRectX0 = toolParameter.searchRectX0;
	ivsContourResult.searchRectY0 = toolParameter.searchRectY0;
	ivsContourResult.searchRectX1 = toolParameter.searchRectX1;
	ivsContourResult.searchRectY1 = toolParameter.searchRectY1;
	ivsContourResult.searchRectX2 = toolParameter.searchRectX2;
	ivsContourResult.searchRectY2 = toolParameter.searchRectY2;
	ivsContourResult.searchRectX3 = toolParameter.searchRectX3;
	ivsContourResult.searchRectY3 = toolParameter.searchRectY3;

	ivsContourResult.bitmapSize = 0;
	ivsContourResult.
	
	stop(tt3);
	Koyo_Template_Match_Info * pTemplateMatchInfo = (Koyo_Template_Match_Info *)palg_handle->run_tool_param;
	if (!checkResultValid(palg_handle, pTemplateMatchInfo)) {
		KOYO_LOG_ERROR("result is not valid. please check...\n");
		pTemplateMatchInfo->res_coor.x = 0;
		pTemplateMatchInfo->res_coor.y = 0;
		pTemplateMatchInfo->res_angel = 0;
	}

	KOYO_LOG_INFO("each phase time cost tt1: %d, tt2: %d, tt3: %d\n", duration(tt1), duration(tt2), duration(tt3));
	IPoint *res_coor = &((Koyo_Template_Match_Info *)palg_handle->run_tool_param)->res_coor;
	INT16 *res_angel = &((Koyo_Template_Match_Info *)palg_handle->run_tool_param)->res_angel;
	INT8 *res_score = &((Koyo_Template_Match_Info *)palg_handle->run_tool_param)->res_score;
	//    KOYO_LOG_INFO("tpl[4][30].noofcordinates %d;x %d;y %d.\n", ((Koyo_Template_Match_Info *)palg_handle->run_tool_param)->tpls[4][30].noOfCordinates, \
		    ((Koyo_Template_Match_Info *)palg_handle->run_tool_param)->tpls[4][30].centerOfGravity.x, ((Koyo_Template_Match_Info *)palg_handle->run_tool_param)->tpls[4][30].centerOfGravity.y);
	stop(tt);
	KOYO_LOG_INFO("template match cost: %d\n", duration(tt));
	KOYO_LOG_INFO("result: result: pos y %d x %d ang %d, score: %d\n", res_coor->y, res_coor->x, *res_angel, *res_score);
	(*palg_handle->current_time) = (UINT16)(duration(tt) / 1000);
	//    start(tt);
	/* �������䵽�ṹ����:
	* 1. result_bitmap;
	* 2. result_ret;
	* 3. current_time;
	* */
	//    KOYO_LOG_INFO("here\n");
	//


	// �µ����ֲ��ԣ�����ȫ�����ֺ;ֲ����ֶ������õģ�����ԭ���ǣ�����ó�������*���������ڵĵ���/ģ���ܵ������������������һ���޸�

	IPoint left_top_point, right_bottom_point;
	left_top_point.x = pstTemplateMatch->rect_roi[0].x;
	left_top_point.y = pstTemplateMatch->rect_roi[0].y;
	right_bottom_point.x = pstTemplateMatch->rect_roi[0].x + pstTemplateMatch->rect_roi[0].width;
	right_bottom_point.y = pstTemplateMatch->rect_roi[0].y + pstTemplateMatch->rect_roi[0].height;

	UINT32 point_sum = 0;
	for (UINT32 i = 0; i < (((Koyo_Template_Match_Info *)palg_handle->run_tool_param)->tpls[0][(*res_angel)]).noOfCordinates; ++i){
		IPoint p = (((Koyo_Template_Match_Info *)palg_handle->run_tool_param)->tpls[0][(*res_angel)]).cordinates[i];
		// �жϸõ��Ƿ���������Χ��
		p.x += res_coor->x << com_level;
		p.y += res_coor->y << com_level;
		if (p.x >= left_top_point.x && p.x <= right_bottom_point.x && p.y >= left_top_point.y && p.y <= right_bottom_point.y){
			point_sum++;
		}
	}
	double percent = (double)point_sum / (((Koyo_Template_Match_Info *)palg_handle->run_tool_param)->tpls[0][(*res_angel)]).noOfCordinates;
	KOYO_LOG_INFO("the num of points in detection area is %d, the num of model points is %d, the percent is %lf\n", point_sum, (((Koyo_Template_Match_Info *)palg_handle->run_tool_param)->tpls[0][(*res_angel)]).noOfCordinates, percent);
	*res_score *= percent;


	palg_handle->result_ret->tool_value = (short)(*res_score + 0.5); // ��������
	palg_handle->result_ret->tool_IsOk = *res_score >= palg_handle->contour_param->bot_threshold;
	KOYO_LOG_INFO("*res_score: %d, palg_handle->contour_param->bot_threshold: %d , com_score: %d, ok: %d\n", *res_score, palg_handle->contour_param->bot_threshold, com_score, palg_handle->result_ret->tool_IsOk);
	TemplateStruct *tpl;
	float temp_ang1 = ((Koyo_Template_Match_Info *)palg_handle->run_tool_param)->search_angel_nstep[1];
	float temp_ang0 = ((Koyo_Template_Match_Info *)palg_handle->run_tool_param)->search_angel_nstep[0];
	//    if (palg_handle->result_ret->tool_IsOk)
	if (*res_score >= com_score)   //fixme:score need to adjust
	{
		if (com_level)
			tpl = &(((Koyo_Template_Match_Info *)palg_handle->run_tool_param)->tpls[0][(INT32)(*res_angel*temp_ang1 / temp_ang0)]);
		else
			tpl = &(((Koyo_Template_Match_Info *)palg_handle->run_tool_param)->tpls[0][(INT32)(*res_angel)]);
	}
	else {
		KOYO_LOG_INFO("upload 0 0 tpl\n");
		tpl = &(((Koyo_Template_Match_Info *)palg_handle->run_tool_param)->tpls[0][palg_handle->contour_param->angle_range]);
	}

	int bitmap_height = tpl->modelHeight;
	int bitmap_width = tpl->modelWidth;
	//    KOYO_LOG_INFO("height:%d\n", bitmap_height);
	//    KOYO_LOG_INFO("width:%d\n", bitmap_width);

	INT32 temp;
	temp = (bitmap_height * bitmap_width) / 8;
	if ((bitmap_height * bitmap_width) % 8)
		temp++;
	palg_handle->result_ret->contour_result.bitmap_size = (UINT16)temp;
	//    HI_U8 *pU8 = malloc(sizeof(UINT8) * (bitmap_height * bitmap_width));
	HI_U8 *pU8 = palg_handle->tool_bitmap;
	if (pU8 == NULL){
		printf("Unnable to create memory.%s,%d\n", __FILE__, __LINE__);
	}
	//    KOYO_LOG_INFO("h:%d,,xy:%d.\n", bitmap_height, bitmap_width);
	memset(pU8, 0, sizeof(UINT8)* bitmap_height * bitmap_width);
	INT16 cenx = tpl->centerOfGravity.x;   //
	INT16 ceny = tpl->centerOfGravity.y;

	KOYO_LOG_INFO("here\n");
	for (UINT32 i = 0; i < tpl->noOfCordinates; i++) {
		*(pU8 + bitmap_width * (tpl->cordinates[i].y + ceny) + (tpl->cordinates[i].x + cenx)) = 1;
		//        KOYO_LOG_INFO("%d %d %d\n", (tpl->cordinates[i].y+cenx), (tpl->cordinates[i].x+ceny), *(pU8 + bitmap_width * (tpl->cordinates[i].y+cenx) + (tpl->cordinates[i].x+ceny)));
	}
	//    KOYO_LOG_INFO("here\n");
	//char2bitmap(pU8, palg_handle->result_bitmap, palg_handle->result_ret->contour_result.bitmap_size);
	char2bitmap(pU8, palg_handle->result_bitmap, bitmap_height * bitmap_width);
	if ((bitmap_height * bitmap_width) % 8){
		palg_handle->result_ret->contour_result.bitmap_size++;
	}
	//    KOYO_LOG_INFO("here\n");
	/* �������ʱ��������λ�� */
	//    KOYO_LOG_INFO("ext_x:%d\n", palg_handle->result_ret->contour_result.ext_rect_x);
	//    KOYO_LOG_INFO("ext_y:%d\n", palg_handle->result_ret->contour_result.ext_rect_y);
	//    KOYO_LOG_INFO("res.x:%d\n", res_coor.x<<1);
	//    KOYO_LOG_INFO("res.y:%d\n", res_coor.y<<1);
	//    KOYO_LOG_INFO("tpl.y:%d\n", tpl->centerOfGravity.y);
	//    KOYO_LOG_INFO("tpl.x:%d\n", tpl->centerOfGravity.x);
	//    KOYO_LOG_INFO("here\n");
	//ext_rect
	palg_handle->result_ret->contour_result.ext_rect_width = bitmap_width;
	palg_handle->result_ret->contour_result.ext_rect_height = bitmap_height;

	// �����ȫ�ֵĻ�����Ҫ��ֵ���ֲ��Ļ��Ѿ��ڽ����ط������˸�ֵ
	if ((palg_handle->contour_param->search_rect_x == 0 && palg_handle->contour_param->search_rect_y == 0 && palg_handle->contour_param->search_rect_width == IMAGE_MAX_WIDTH
		&& palg_handle->contour_param->search_rect_height == IMAGE_MAX_HEIGHT)){
		palg_handle->result_ret->contour_result.ext_rect_x = palg_handle->contour_param->ext_rect_x;
		palg_handle->result_ret->contour_result.ext_rect_y = palg_handle->contour_param->ext_rect_y;             //*****************
	}
	//circle
	palg_handle->result_ret->contour_result.detect_circ_x = palg_handle->contour_param->detect_circ_x;   //fixme: detect or search or ext
	palg_handle->result_ret->contour_result.detect_circ_y = palg_handle->contour_param->detect_circ_y;
	palg_handle->result_ret->contour_result.detect_circ_radius = palg_handle->contour_param->detect_circ_radius;

	KOYO_LOG_DEBUG("ext_rect_x:%d-------------%d\n", palg_handle->result_ret->contour_result.ext_rect_x, __LINE__);
	KOYO_LOG_DEBUG("ext_rect_y:%d-------------%d\n", palg_handle->result_ret->contour_result.ext_rect_y, __LINE__);
	KOYO_LOG_DEBUG("ext_rect_width:%d-------------%d\n", palg_handle->result_ret->contour_result.ext_rect_width, __LINE__);
	KOYO_LOG_DEBUG("ext_rect_height:%d-------------%d\n", palg_handle->result_ret->contour_result.ext_rect_height, __LINE__);


	if (*res_score >= com_score) {
		//ext_rect
		UINT16 temp_ext_x, temp_ext_y;
		if (com_level)
		{
			temp_ext_x = (res_coor->x << 1) - cenx;
			temp_ext_y = (res_coor->y << 1) - ceny;
			//circle
			palg_handle->result_ret->contour_result.detect_circ_x = (res_coor->x << 1);
			palg_handle->result_ret->contour_result.detect_circ_y = (res_coor->y << 1);
		}
		else
		{
			temp_ext_x = res_coor->x - cenx;
			temp_ext_y = res_coor->y - ceny;
			//circle
			palg_handle->result_ret->contour_result.detect_circ_x = res_coor->x;
			palg_handle->result_ret->contour_result.detect_circ_y = res_coor->y;
		}
		palg_handle->result_ret->contour_result.ext_rect_x = (UINT16)temp_ext_x >= (UINT16)640 ? 0 : temp_ext_x;
		palg_handle->result_ret->contour_result.ext_rect_y = (UINT16)temp_ext_y >= (UINT16)480 ? 0 : temp_ext_y;             //*****************

		palg_handle->result_ret->contour_result.ext_rect_x = (UINT16)temp_ext_x<(UINT16)0 ? 0 : temp_ext_x;
		palg_handle->result_ret->contour_result.ext_rect_y = (UINT16)temp_ext_y<(UINT16)0 ? 0 : temp_ext_y;             //*****************

		KOYO_LOG_DEBUG("ext_rect_x:%d-------------%d\n", palg_handle->result_ret->contour_result.ext_rect_x, __LINE__);
		KOYO_LOG_DEBUG("ext_rect_y:%d-------------%d\n", palg_handle->result_ret->contour_result.ext_rect_y, __LINE__);
		KOYO_LOG_DEBUG("ext_rect_width:%d-------------%d\n", palg_handle->result_ret->contour_result.ext_rect_width, __LINE__);
		KOYO_LOG_DEBUG("ext_rect_height:%d-------------%d\n", palg_handle->result_ret->contour_result.ext_rect_height, __LINE__);

		// ����֮ǰ��ʾ������ӿ�Ϊ����ʾ������Ҫ���м��㣬��ȻҲ������ֲ���ͻ����Ͻ��м���
		UINT16 minx = palg_handle->result_ret->contour_result.ext_rect_x;
		UINT16 miny = palg_handle->result_ret->contour_result.ext_rect_y;
		UINT16 maxx = minx + bitmap_width;
		UINT16 maxy = miny + bitmap_height;

		UINT16 height = (((Koyo_Template_Match_Info *)palg_handle->run_tool_param)->tpls[0][palg_handle->contour_param->angle_range]).modelHeight;
		UINT16 width = (((Koyo_Template_Match_Info *)palg_handle->run_tool_param)->tpls[0][palg_handle->contour_param->angle_range]).modelWidth;

		UINT16 Theta_for_cal = ((INT16)(com_level ? ((*res_angel - palg_handle->contour_param->angle_range)*temp_ang1 / temp_ang0) : (*res_angel - palg_handle->contour_param->angle_range)) + 360) % 360;
		double sin_theta = sin(Theta_for_cal*3.1415926535 / 180);

		KOYO_LOG_INFO("theta:%d\n", Theta_for_cal);

		//KOYO_LOG_DEBUG("the detect_rect_x0, detect_rect_y0 is %d,%d\n", palg_handle->result_ret->contour_result.detect_rect_x0, palg_handle->result_ret->contour_result.detect_rect_y0);
		//KOYO_LOG_DEBUG("the detect_rect_x1, detect_rect_y1 is %d,%d\n", palg_handle->result_ret->contour_result.detect_rect_x1, palg_handle->result_ret->contour_result.detect_rect_y1);
		//KOYO_LOG_DEBUG("the detect_rect_x2, detect_rect_y2 is %d,%d\n", palg_handle->result_ret->contour_result.detect_rect_x2, palg_handle->result_ret->contour_result.detect_rect_y2);
		//KOYO_LOG_DEBUG("the detect_rect_x3, detect_rect_y3 is %d,%d\n", palg_handle->result_ret->contour_result.detect_rect_x3, palg_handle->result_ret->contour_result.detect_rect_y3);

		// ���������ý׶η�ˮƽ������ʾ��ӿ������

		IPoint coor_point[4];
		IPoint left_point, top_point;
		double ratiox = 0.0, ratioy = 0.0;
		coor_point[0].x = palg_handle->result_ret->contour_result.detect_rect_x0;
		coor_point[0].y = palg_handle->result_ret->contour_result.detect_rect_y0;
		coor_point[1].x = palg_handle->result_ret->contour_result.detect_rect_x1;
		coor_point[1].y = palg_handle->result_ret->contour_result.detect_rect_y1;
		coor_point[2].x = palg_handle->result_ret->contour_result.detect_rect_x2;
		coor_point[2].y = palg_handle->result_ret->contour_result.detect_rect_y2;
		coor_point[3].x = palg_handle->result_ret->contour_result.detect_rect_x3;
		coor_point[3].y = palg_handle->result_ret->contour_result.detect_rect_y3;

		int y_min_tmp = 10000;
		int x_min_tmp = 10000;
		int y_max_tmp = -10000;
		int x_max_tmp = -10000;

		for (int i = 0; i < 4; ++i){
			if (y_min_tmp > coor_point[i].y){
				top_point = coor_point[i];
				y_min_tmp = coor_point[i].y;
			}
			if (x_min_tmp > coor_point[i].x){
				left_point = coor_point[i];
				x_min_tmp = coor_point[i].x;
			}
			if (y_max_tmp < coor_point[i].y){
				y_max_tmp = coor_point[i].y;
			}
			if (x_max_tmp < coor_point[i].x){
				x_max_tmp = coor_point[i].x;
			}
		}

		KOYO_LOG_DEBUG("the top_point is (%d, %d)\n", top_point.x, top_point.y);
		KOYO_LOG_DEBUG("the left_point is (%d, %d)\n", left_point.x, left_point.y);

		ratiox = (double)(top_point.x - x_min_tmp) / (x_max_tmp - x_min_tmp);
		ratioy = (double)(left_point.y - y_min_tmp) / (y_max_tmp - y_min_tmp);

		KOYO_LOG_DEBUG("the ratiox is %lf, the ratioy is %lf\n", ratiox, ratioy);

		if (Theta_for_cal >= 0 && Theta_for_cal < 90){
			coor_point[0].x = minx;
			coor_point[0].y = miny + sin_theta*width;
			coor_point[1].x = minx + sin_theta*height;
			coor_point[1].y = maxy;
			coor_point[2].x = maxx;
			coor_point[2].y = maxy - sin_theta*width;
			coor_point[3].x = maxx - sin_theta*height;
			coor_point[3].y = miny;
		}
		else if (Theta_for_cal >= 90 && Theta_for_cal < 180){
			coor_point[0].x = maxx - sin_theta*height;
			coor_point[0].y = maxy;
			coor_point[1].x = maxx;
			coor_point[1].y = miny + sin_theta*width;
			coor_point[2].x = minx + sin_theta*height;
			coor_point[2].y = miny;
			coor_point[3].x = minx;
			coor_point[3].y = maxy - sin_theta*width;
		}
		else if (Theta_for_cal >= 180 && Theta_for_cal < 270){
			coor_point[0].x = maxx;
			coor_point[0].y = maxy + sin_theta*width;
			coor_point[1].x = maxx + sin_theta*height;
			coor_point[1].y = miny;
			coor_point[2].x = minx;
			coor_point[2].y = miny - sin_theta*width;
			coor_point[3].x = minx - sin_theta*height;
			coor_point[3].y = maxy;
		}
		else{
			coor_point[0].x = minx - sin_theta*height;
			coor_point[0].y = miny;
			coor_point[1].x = minx;
			coor_point[1].y = maxy + sin_theta*width;
			coor_point[2].x = maxx + sin_theta*height;
			coor_point[2].y = maxy;
			coor_point[3].x = maxx;
			coor_point[3].y = miny - sin_theta*width;
		}

		//KOYO_LOG_DEBUG("the coor_point_x0, coor_point_y0 is %d,%d\n", coor_point[0].x, coor_point[0].y);
		//KOYO_LOG_DEBUG("the coor_point_x1, coor_point_y1 is %d,%d\n", coor_point[1].x, coor_point[1].y);
		//KOYO_LOG_DEBUG("the coor_point_x2, coor_point_y2 is %d,%d\n", coor_point[2].x, coor_point[2].y);
		//KOYO_LOG_DEBUG("the coor_point_x3, coor_point_y3 is %d,%d\n", coor_point[3].x, coor_point[3].y);

		if (ratiox < 0.000001 || ratioy < 0.000001){
			palg_handle->result_ret->contour_result.detect_rect_x0 = coor_point[0].x;
			palg_handle->result_ret->contour_result.detect_rect_y0 = coor_point[0].y;
			palg_handle->result_ret->contour_result.detect_rect_x1 = coor_point[1].x;
			palg_handle->result_ret->contour_result.detect_rect_y1 = coor_point[1].y;
			palg_handle->result_ret->contour_result.detect_rect_x2 = coor_point[2].x;
			palg_handle->result_ret->contour_result.detect_rect_y2 = coor_point[2].y;
			palg_handle->result_ret->contour_result.detect_rect_x3 = coor_point[3].x;
			palg_handle->result_ret->contour_result.detect_rect_y3 = coor_point[3].y;
		}
		else{

			IPoint tmp_point;
			tmp_point = cal_coor_by_ratio(coor_point[0], coor_point[1], ratioy);
			palg_handle->result_ret->contour_result.detect_rect_x0 = tmp_point.x;
			palg_handle->result_ret->contour_result.detect_rect_y0 = tmp_point.y;
			tmp_point = cal_coor_by_ratio(coor_point[2], coor_point[1], ratiox);
			palg_handle->result_ret->contour_result.detect_rect_x1 = tmp_point.x;
			palg_handle->result_ret->contour_result.detect_rect_y1 = tmp_point.y;
			tmp_point = cal_coor_by_ratio(coor_point[2], coor_point[3], ratioy);
			palg_handle->result_ret->contour_result.detect_rect_x2 = tmp_point.x;
			palg_handle->result_ret->contour_result.detect_rect_y2 = tmp_point.y;
			tmp_point = cal_coor_by_ratio(coor_point[0], coor_point[3], ratiox);
			palg_handle->result_ret->contour_result.detect_rect_x3 = tmp_point.x;
			palg_handle->result_ret->contour_result.detect_rect_y3 = tmp_point.y;
		}


		//KOYO_LOG_DEBUG("detect rect after\n");
		//KOYO_LOG_DEBUG("the detect_rect_x0, detect_rect_y0 is %d,%d\n", palg_handle->result_ret->contour_result.detect_rect_x0, palg_handle->result_ret->contour_result.detect_rect_y0);
		//KOYO_LOG_DEBUG("the detect_rect_x1, detect_rect_y1 is %d,%d\n", palg_handle->result_ret->contour_result.detect_rect_x1, palg_handle->result_ret->contour_result.detect_rect_y1);
		//KOYO_LOG_DEBUG("the detect_rect_x2, detect_rect_y2 is %d,%d\n", palg_handle->result_ret->contour_result.detect_rect_x2, palg_handle->result_ret->contour_result.detect_rect_y2);
		//KOYO_LOG_DEBUG("the detect_rect_x3, detect_rect_y3 is %d,%d\n", palg_handle->result_ret->contour_result.detect_rect_x3, palg_handle->result_ret->contour_result.detect_rect_y3);

	}
	else {
		palg_handle->result_ret->contour_result.ext_rect_x = palg_handle->contour_param->ext_rect_x;
		palg_handle->result_ret->contour_result.ext_rect_y = palg_handle->contour_param->ext_rect_y;             //*****************

		KOYO_LOG_DEBUG("ext_rect_x:%d-------------%d\n", palg_handle->result_ret->contour_result.ext_rect_x, __LINE__);
		KOYO_LOG_DEBUG("ext_rect_y:%d-------------%d\n", palg_handle->result_ret->contour_result.ext_rect_y, __LINE__);
		KOYO_LOG_DEBUG("ext_rect_width:%d-------------%d\n", palg_handle->result_ret->contour_result.ext_rect_width, __LINE__);
		KOYO_LOG_DEBUG("ext_rect_height:%d-------------%d\n", palg_handle->result_ret->contour_result.ext_rect_height, __LINE__);
	}

	//// ֻ����ֲ����������������


#if 0
	if (!(palg_handle->contour_param->search_rect_x == 0 && palg_handle->contour_param->search_rect_y == 0 && palg_handle->contour_param->search_rect_width == IMAGE_MAX_WIDTH
		&& palg_handle->contour_param->search_rect_height == IMAGE_MAX_HEIGHT)){

		palg_handle->result_ret->contour_result.detect_rect_x0 = palg_handle->contour_param->detect_rect_x0;
		palg_handle->result_ret->contour_result.detect_rect_y0 = palg_handle->contour_param->detect_rect_y0;
		palg_handle->result_ret->contour_result.detect_rect_x1 = palg_handle->contour_param->detect_rect_x1;
		palg_handle->result_ret->contour_result.detect_rect_y1 = palg_handle->contour_param->detect_rect_y1;
		palg_handle->result_ret->contour_result.detect_rect_x2 = palg_handle->contour_param->detect_rect_x2;
		palg_handle->result_ret->contour_result.detect_rect_y2 = palg_handle->contour_param->detect_rect_y2;
		palg_handle->result_ret->contour_result.detect_rect_x3 = palg_handle->contour_param->detect_rect_x3;
		palg_handle->result_ret->contour_result.detect_rect_y3 = palg_handle->contour_param->detect_rect_y3;



	}
#endif
	//    free(pU8);
	//    KOYO_LOG_INFO("here\n");
	//

	//    ������������Ҫ��0-360�ȵ�

	INT8 ang_range = palg_handle->contour_param->angle_range;

	// 0 �Ƕȵ�ģ���λ��
	UINT16 tpl0_size = ang_range / temp_ang0; // 0���ϵ�ģ�����
	KOYO_LOG_INFO("tpl0 size: %d\n", tpl0_size);
	UINT16 result_angle_for_corr = 0;

	if (*res_angel <= ang_range) {
		result_angle_for_corr = 360 - (ang_range - *res_angel);
	}
	else {
		result_angle_for_corr = *res_angel - ang_range;
	}
	KOYO_LOG_INFO("res_angel: %d, result_angle_for_corr: %d, ang_range: %d\n", *res_angel, result_angle_for_corr, ang_range);

	tpl = &(((Koyo_Template_Match_Info *)palg_handle->run_tool_param)->tpls[0][ang_range]);

	// ����ֻ����ȫ������ʱ�����������תƽ�ƾ���
	if ((palg_handle->contour_param->search_rect_x == 0 && palg_handle->contour_param->search_rect_y == 0 && palg_handle->contour_param->search_rect_width == IMAGE_MAX_WIDTH
		&& palg_handle->contour_param->search_rect_height == IMAGE_MAX_HEIGHT && fabs(pic_corr->model_point.x) < 0.000001 && fabs(pic_corr->pic_point.x) < 0.000001 && fabs(pic_corr->model_point.y) < 0.000001 && fabs(pic_corr->pic_point.y) < 0.000001 && fabs(pic_corr->angel) < 0.000001)){
		pic_corr->model_point.x = tpl->centerOfGravity.x + palg_handle->contour_param->ext_rect_x;
		pic_corr->model_point.y = tpl->centerOfGravity.y + palg_handle->contour_param->ext_rect_y;
		if (*res_score >= com_score) {
			if (com_level) {
				pic_corr->angel = (INT32)(result_angle_for_corr * temp_ang1 / temp_ang0);
				pic_corr->pic_point.x = res_coor->x << 1;
				pic_corr->pic_point.y = res_coor->y << 1;
			}
			else
			{
				pic_corr->angel = (INT32)(result_angle_for_corr * temp_ang0);
				pic_corr->pic_point.x = res_coor->x;
				pic_corr->pic_point.y = res_coor->y;
			}
		}
		else{
			pic_corr->angel = 0;
			pic_corr->pic_point.x = pic_corr->model_point.x;
			pic_corr->pic_point.y = pic_corr->model_point.y;
		}
	}
	//    stop(tt);
	//    KOYO_LOG_INFO("write result cost: %d\n", duration(tt));
	//    KOYO_LOG_INFO("here\n");
	return 0;
}

int ContourToolProcess::freeTask(){
	///* ���������ͷ��ڴ�, ֻ�ͷŹ��ߵ�����, ���ұ�Ҳ���ͷ��ˣ�Ŀǰ���ұ����ڸ��������еģ����Կ����Ƴ��� */
	//INT32 i = 0, j = 0;
	//Koyo_Template_Match_Info *pstTemplate_match_info = (Koyo_Template_Match_Info*)palg_handle->run_tool_param;

	//KOYO_LOG_INFO("here.\n");
	///* �ͷ����е�ģ�� */
	//UINT16 tplsize;
	//for (INT32 tpl_i = 0; tpl_i<pstTemplate_match_info->run_time_npyramid; ++tpl_i)
	//{
	//	/* �ͷ����е�ģ�� */
	//	free(pstTemplate_match_info->tpls[tpl_i]);
	//	//        KOYO_LOG_INFO("here, %d %d.\n", tpl_i, run_tool_param->run_time_npyramid);
	//}
	//free(pstTemplate_match_info->tpls);
	//KOYO_LOG_INFO("here.\n");

	////    INT16 nangs_temp = (INT16)ceil((double)MAX_DEGREE/pstTemplate_match_info->search_angel_nstep[1]);
	////    int width = tool_utility->u16Width >> 1;
	////    int height = tool_utility->u16Height >> 1;
	////    for (INT16 wtemp = 0; wtemp < width; ++wtemp) {
	////        for (INT16 htemp = 0; htemp < height; ++htemp) {
	////            free(pstTemplate_match_info->iscompute[wtemp][htemp]);
	////        }
	////        free(pstTemplate_match_info->iscompute[wtemp]);
	////    }
	////    free(pstTemplate_match_info->iscompute);

	//KOYO_LOG_INFO("here.\n");
	return 0;
}

// ���ݴ���ͼ������Ӧ������ͼ�����꣬�Լ����ݹ�����ģ���������߳�, ����ཻ�ľ���
cv::Rect getRuningRect(cv::Rect searchRegion, cv::Point mapCenter, UINT16 top_search_rect_width) {
	int a = (float)top_search_rect_width/2 + 0.5;
	int x0 = std::max(searchRegion.x, mapCenter.x - a);
	int x2 = std::min(searchRegion.x + searchRegion.width, mapCenter.x + a);
	int y0 = std::max(searchRegion.y, mapCenter.y - a);
	int y2 = std::min(searchRegion.y + searchRegion.height, mapCenter.y + a);

	cv::Rect result(x0, y0, x2 - x0, y2 - y0);
	return result;
}


// ���ݾ��Σ������ʱ����ͼ�е���������
int get_num_of_rect(cv::Rect rect, cv::Mat cannyBinary) {
	int x0 = rect.x;
	int y0 = rect.y;
	int x2 = rect.x + rect.width;
	int y2 = rect.y + rect.height;

	int noOfPoint = 0;
	for (int i = x0; i < x2; ++i) {
		for (int j = y0; j < y2; ++j) {
			if (cannyBinary.at<uchar>(j, i) == 255) {
				++noOfPoint;
			}
		}
	}
	return noOfPoint;
}

// �������ͼ�����Ӧ�ĵ�ĸ���
int num_of_mapping_point(cv::Rect rect, std::vector<cv::Point> &tpl_point, const cv::Mat binary, const cv::Point &mapCenter) {
	int noOfPoint = 0;
	for (auto point : tpl_point) {
		point.x += mapCenter.x;
		point.y += mapCenter.y;
		if (rect.contains(point) && binary.at<uchar>(point.y, point.x) == 255) {
			++noOfPoint;
		}
	}
	return noOfPoint;
}

// �����������ƥ�䣬��������¼���ƥ��
int ContourToolProcess::doOtherLayerMatch(ContourUtility &contourUtility, int cur_level, float threshold_S) {

	std::vector<IVSTemplateSubStruct> &cur_tpls = templateInfo.tpls[cur_level];
	float cur_angle_step = templateInfo.searchAngelStep[cur_level];
	UINT16 cur_search_rect_width = templateInfo.searchRectWidth[cur_level];

	std::priority_queue<CandidateResult> curLevelCandidates;
	std::priority_queue<CandidateResult> upperLevelCandidates = candidates[templateInfo.pyramidLevelNum - 2 - cur_level];
	int upperLevelCandidatesNum = upperLevelCandidates.size();

	// �Ƕ����ʱ����Ҫ���¼������һ�㵽�ò�ĺ�ѡ��
	// ���ȶ��д�0��ʼ

	// Ϊ�����ظ����㣬ÿ���µ�λ�ö���Ҫ�鿴��λ���Ƿ��Ѿ������
	std::vector<CandidateResult> calculatedResult;
	for (int i = 0; i < upperLevelCandidatesNum; ++i) {
		CandidateResult pos = upperLevelCandidates.top();
		upperLevelCandidates.pop();
		pos.positionX <<= 1;
		pos.positionY <<= 1;
		pos.angleIndex = (pos.angleIndex * templateInfo.searchAngelStep[cur_level + 1] /
			templateInfo.searchAngelStep[cur_level]) + 0.5;
		
		
		// ��ѡ������������Χ�ĵ��Լ��Ե�ǰ�Ƕ����Ҽ����ò�ǶȲ����ĽǶ�
		std::vector<cv::Point> point_bak;
		for (int i = pos.positionX - IVS_CONTOUR_NABOR_SIZE; i <= pos.positionX + IVS_CONTOUR_NABOR_SIZE 
			&& i < searchRect[cur_level].x + searchRect[cur_level].width; ++i) {
			for (int j = pos.positionY - IVS_CONTOUR_NABOR_SIZE; j <= pos.positionY + IVS_CONTOUR_NABOR_SIZE
				&& j < searchRect[cur_level].y + searchRect[cur_level].height; ++j) {
				point_bak.push_back(cv::Point(i, j));
			}
		}


		// �������ƶȵļ���

		for (auto centerPoint : point_bak) {
			cv::Rect rectRuning = getRuningRect(searchRect[cur_level], centerPoint, cur_search_rect_width);
			bool isCalculated = false;

			// (��ǰĬ�ϽǶȷ�ΧΪ����2)
			for (int i = -IVS_CONTOUR_NABOR_ANGLE; i <= IVS_CONTOUR_NABOR_ANGLE; ++i) {
				int angleIndex = (pos.angleIndex + i + cur_tpls.size()) % cur_tpls.size();
				float posAngle = pos.angleIndex * cur_angle_step - toolParameter.angleRange;
				float tmpAngle = angleIndex * cur_angle_step - toolParameter.angleRange;
				// ���������޸�+1��������㷨�ȶ���
				if (fabs(posAngle - tmpAngle) > (IVS_CONTOUR_NABOR_ANGLE+1) * cur_angle_step) {
					printf("��ǰ�ǶȲ��ʺ�\n");
					continue;
				}
				// ��ǰ�Ƕ��Ƿ�����
				/*for (auto res : calculatedResult) {
					float resAngle = res.angleIndex * cur_angle_step - toolParameter.angleRange;
					if (abs(centerPoint.x - res.positionX) <= IVS_CONTOUR_NABOR_SIZE
						&& abs(centerPoint.y - res.positionY) <= IVS_CONTOUR_NABOR_SIZE
						&& fabs(resAngle - tmpAngle) <= IVS_CONTOUR_NABOR_ANGLE * cur_angle_step) {
						printf("��λ�ü��Ƕ��Ѿ��������");
						isCalculated = true;
						break;
					}
				}*/
				if (isCalculated) {
					continue;
				}
			
				// ���м���
				IVSTemplateSubStruct &tpl = cur_tpls[angleIndex];
				int K1 = tpl.cordinates.size();
				int num_T = 0;
				double sum_of_s = 0.0;
				int order = 0;
				for (auto point : tpl.cordinates) {
					point.x += centerPoint.x;
					point.y += centerPoint.y;
					if (rectRuning.contains(point) && cannyPyramid[cur_level].at<uchar>(point.y, point.x) == 255) {
						++num_T;
						sum_of_s += fabs(tpl.edgeDerivativeX[order] * contourUtility.edgeX[cur_level][point.y][point.x]
							+ tpl.edgeDerivativeY[order] * contourUtility.edgeY[cur_level][point.y][point.x]);
					}
					++order;
					if (sum_of_s < (threshold_S - 1)*K1 + num_T) {
						break;
					}
				}
				double s = sum_of_s / num_T;

				// ƥ�����ƶȸߵ����ĺͽǶȼ�¼����
				if (s > IVS_CONTOUR_SCORE_THRESHOLD) {
					CandidateResult result;
					result.level = cur_level;
					result.positionX = centerPoint.x;
					result.positionY = centerPoint.y;
					result.angleIndex = angleIndex;
					result.score = s;

					std::cout << "now center is (" << result.positionX << "," << result.positionY
						<< "), and the angle is " << result.angleIndex*cur_angle_step-toolParameter.angleRange
						<< "the similarity is " << s << std::endl;

					// ����ò�ĺ�ѡ��С���ѣ�Ĭ����С��ѡ�����Ϊ1
					int curCandidateNum = (IVS_CONTOUR_CANDIDATE_NUM >> (templateInfo.pyramidLevelNum - 1 - cur_level)) > 1 ?
						(IVS_CONTOUR_CANDIDATE_NUM >> (templateInfo.pyramidLevelNum - 1 - cur_level)) : 1;
					std::cout << "level " << cur_level << " have " << curCandidateNum << " candidate point." << std::endl;
					if (curLevelCandidates.size() < curCandidateNum) {
						curLevelCandidates.push(result);
					}
					else {
						if (curLevelCandidates.top().score < result.score) {
							curLevelCandidates.pop();
							curLevelCandidates.push(result);
						}
					}
				}
			}
		}

		// ���Ѿ�������ϲ��ѡλ�ü�¼����
		calculatedResult.push_back(pos);
		
	}

	// ���ò��ѡ��λ��С���Ѷ��뵽candidates��
	candidates.push_back(curLevelCandidates);

	return 0;
}


// ģ�嶥���Ӧ�Ĵ���ͼ����������Ӧ��������ƥ��
int ContourToolProcess::doTopLayerMatch(ContourUtility &contourUtility, float threshold_S) {
	// ׼����Ҫ�Ĳ���(0��ģ��ʱ������������)
	int top_level = templateInfo.pyramidLevelNum - 1;
	std::vector<IVSTemplateSubStruct> &top_tpls = templateInfo.tpls[top_level];
	int K = top_tpls[top_tpls.size()/2].noOfCordinates;
	float top_search_angel_nstep = templateInfo.searchAngelStep[top_level];
	int top_search_rect_width = templateInfo.searchRectWidth[top_level];

	// ����һ����ѡ��Ϣ��С����
	std::priority_queue<CandidateResult> minHeapCandidate;

	//SampleStruct &spl = sampleStructs[level];
	//    std::cout << spl.height << " " << spl.width << std::endl;


	for (int i = searchRect[top_level].y; i < searchRect[top_level].y + searchRect[top_level].height; ++i) {
		for (int j = searchRect[top_level].x; j < searchRect[top_level].x + searchRect[top_level].width; ++j) {
			// ��Ӧ������ͼ������
			cv::Point mapCenter;
			mapCenter.x = j;
			mapCenter.y = i;

			cv::Rect rectRuning = getRuningRect(searchRect[top_level], mapCenter, top_search_rect_width);
			//            // �ж���������������
			cv::Mat rectPic = contourUtility.searchRegion[top_level].clone();
			cv::rectangle(rectPic,
						rectRuning,
			            cv::Scalar( 0, 0, 0 ),
			            1,
			            8 );

			int num = get_num_of_rect(rectRuning, cannyPyramid[top_level]);
			//std::cout << "this rect has num is " << num << std::endl;
			/*cv::imshow("rectangle", rectPic);
			cv::waitKey(0);*/
			if (num > K * threshold_S) {
				//                std::cout << "OK the num of Point more than K*S and now center is ("
				//                          << j << "," << i << ")" << std::endl;

				for (int k = 0; k < templateInfo.tpls[top_level].size(); ++k) {
					IVSTemplateSubStruct &tpl = top_tpls[k];
					int K1 = tpl.cordinates.size();
					//cv::imshow("cannyPyramid[top_level]", cannyPyramid[top_level]);
					//cv::waitKey(0);
					//if(num_of_mapping_point(rectRuning, tpl.cordinates, cannyPyramid[top_level], mapCenter) > K1 * threshold_S){
					    //std::cout << "OK the num of Point more than K*S and now center is ("
					               // << j << "," << i << "), and the angle is " << k << std::endl;
						// ���ƶȼ��㹫ʽ
						int num_T = 0;
						double sum_of_s = 0.0;
						int order = 0;
						for (auto point : tpl.cordinates) {
							point.x += mapCenter.x;
							point.y += mapCenter.y;
							if (rectRuning.contains(point) && cannyPyramid[top_level].at<uchar>(point.y, point.x) == 255) {
								++num_T;
								sum_of_s += fabs(tpl.edgeDerivativeX[order] * contourUtility.edgeX[top_level][point.y][point.x]
									+ tpl.edgeDerivativeY[order] * contourUtility.edgeY[top_level][point.y][point.x]);
							}
							++order;
							if (sum_of_s < (threshold_S - 1)*K1 + num_T) {
								break;
							}
						}
						double s = sum_of_s / num_T;
						//                        std::cout << "OK the num of Point more than K*S and now center is ("
						//                                  << j << "," << i << "), and the angle is " << k
						//                                  << "the similarity is " << s << std::endl;

						// ƥ�����ƶ���ߵ����ĺͽǶȼ�¼����
						if (s > IVS_CONTOUR_SCORE_THRESHOLD) {
							CandidateResult result;
							result.angleIndex = k;
							result.level = top_level;
							result.positionX = j;
							result.positionY = i;
							result.score = s;
							if (minHeapCandidate.size() < IVS_CONTOUR_CANDIDATE_NUM) {
								minHeapCandidate.push(result);
							}
							else {
								if (minHeapCandidate.top().score < result.score) {
									minHeapCandidate.pop();
									minHeapCandidate.push(result);
								}
							}
							//std::cout << "now center is (" << j << "," << i << "), and the angle is " << k
								//<< "the similarity is " << s << std::endl;
						}
					//}
				}
			}
		}
	}

	candidates.push_back(minHeapCandidate);
	return 0;
}

static void draw_template(cv::Mat &src, const IVSTemplateSubStruct &tpl, cv::Point centerPoint)
{
	for (UINT32 i = 0; i < tpl.noOfCordinates; ++i) {
		cv::circle(src, cv::Point(tpl.cordinates[i].x + centerPoint.x, tpl.cordinates[i].y + centerPoint.y),
			1, cv::Scalar(255, 255, 255));
	}
}

//src ����ͼ��
INT8 ContourToolProcess::doTemplateMatch(ContourUtility &contourUtility)
{

	INT8 ret;
	INT8 run_time_npyramid = templateInfo.pyramidLevelNum;

	// ���ڸ��ٶȲ���Ҳ��⵽��ײ㣬ͬʱ��֤�Ķ�����ֻ�޸������������
	//INT8 com_level = p_template_matchInfo->contour_parameter.algo_strategy;   ///fixme:now is zero
	INT8 com_level = 0;
	//    KOYO_LOG_INFO("begin match.\n");

	// ����ƥ��������
	doTopLayerMatch(contourUtility, 0.8);

	// �������ƥ��
	for (int l = run_time_npyramid - 2; l >= com_level; --l) {

		printf("begin match level %d.\n", l);
		doOtherLayerMatch(contourUtility, l, 0.7);

		
		//if (l == com_level)
		//{
		//	p_template_matchInfo->res_coor.y = result_out->data[0]->position.y;
		//	p_template_matchInfo->res_coor.x = result_out->data[0]->position.x;
		//	p_template_matchInfo->res_angel = result_out->data[0]->angel_idx;
		//	p_template_matchInfo->res_score = result_out->data[0]->score + 0.5;
		//}
		//if (ret < 0) {
		//	printf("Error at match_template_one_level %d.\n", l);
		//	return ret;
		//}
		//printf("level %d match %d targets.\n", l, result_out->n_scalar);
		//tmp = result_in;
		//result_in = result_out;
		//result_out = tmp;
		//printf("--------------------------------------------.\n");

	}

	// ��������Ƴ���
	CandidateResult finalResult = candidates[templateInfo.pyramidLevelNum - 1].top();
	std::cout << "the final result is x: " << finalResult.positionX << " y: " << finalResult.positionY 
		<< " the angleindex is: " << finalResult.angleIndex << " score: " << finalResult.score << std::endl;

	cv::Mat tmpresult = contourUtility.searchRegion[0].clone();
	draw_template(tmpresult, templateInfo.tpls[0][finalResult.angleIndex],
		cv::Point(finalResult.positionX, finalResult.positionY));
	cv::imshow("result", tmpresult);
	cv::waitKey(0);
	return 0;
}