
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

	// 拷贝模板数据
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

			// 拷贝特征数据
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
	
	// 根据模板文件路径，解析模板文件
	FILE *fp = fopen((const char*)toolParameter.templatePath, "rb");
	fseek(fp, 0L, SEEK_END);
	size_t templateSize = ftell(fp);
	std::cout << "templateSize is" << templateSize << std::endl;
	rewind(fp);
	char *buf = (char *)malloc(sizeof(char)*templateSize);
	fread(buf, sizeof(UINT8), templateSize, fp);


	// 解析数据
	unpackTemplate(buf);
	
	// 释放资源
	fclose(fp);
	free(buf);

	/* 设置每层金字塔的搜索区域 */     
	//此部分只用到顶层的搜索区域，在加入搜索框，在4层搜索时需要用到，以加快搜索速度

	// 考虑到之后局部轮廓可能也存在矫正的问题，这里的搜索区域需要为参数中搜索范围的外接框
	INT16 leftTopX = IVS_MIN<INT16>(4, toolParameter.searchRectX0, toolParameter.searchRectX1,
		toolParameter.searchRectX2, toolParameter.searchRectX3);
	INT16 leftTopY = IVS_MIN<INT16>(4, toolParameter.searchRectY0, toolParameter.searchRectY1,
		toolParameter.searchRectY2, toolParameter.searchRectY3);

	INT16 rightBottomX = IVS_MAX<INT16>(4, toolParameter.searchRectX0, toolParameter.searchRectX1,
		toolParameter.searchRectX2, toolParameter.searchRectX3);

	INT16 rightBottomY = IVS_MAX<INT16>(4, toolParameter.searchRectY0, toolParameter.searchRectY1,
		toolParameter.searchRectY2, toolParameter.searchRectY3);

	// 填充第一层搜索范围
	cv::Rect rectTmp;
	rectTmp.x = leftTopX;
	rectTmp.y = leftTopY;
	rectTmp.width = rightBottomX - leftTopX;
	rectTmp.height = rightBottomY - leftTopY;
	searchRegion.push_back(rectTmp);

	for (int i = 1; i < templateInfo.pyramidLevelNum; ++i) {
		
		// 填充每一层的搜索范围
		rectTmp.x = searchRegion[i - 1].x >> 1;
		rectTmp.y = searchRegion[i - 1].y >> 1;
		rectTmp.width = searchRegion[i - 1].width >> 1;
		rectTmp.height = searchRegion[i - 1].height >> 1;
		searchRegion.push_back(rectTmp);
	}
	
	// 打印查看输出的模板文件是否正确
	//for (int i = 0; i < templateInfo.tpls.size(); ++i) {
	//	for (int j = 0; j < templateInfo.tpls[i].size(); ++j) {
	//		print_tpl(templateInfo.tpls[i][j]);
	//	}
	//}
	
	// 处理待测图像的图像金字塔(其他部分都在contourUtility进行了处理，这里只处理canny图像)
	int lowThreshold = toolParameter.sensiLowThreshold;
	int topThreshold = toolParameter.sensiTopThreshold;

	lowThreshold = lowThreshold > 255 ? 255 : lowThreshold;
	topThreshold = topThreshold > 255 ? 255 : topThreshold;

	for (int i = 0; i < templateInfo.pyramidLevelNum; ++i) {
		// 创建canny图像，以及相应的膨胀图像，积分图图像(只处理搜索范围的图像)
		cv::Mat cannyMat, cannyDilateMat, integralMat;
		cv::Canny(contourUtility.searchRegion[i](searchRegion[i]), cannyMat, lowThreshold, topThreshold);
		Dilation(cannyMat, cannyDilateMat, 2);
		cannyPyramid.push_back(cannyMat);
		cannyDilatePyramid.push_back(cannyDilateMat);

		//cv::imshow("cannyMat", cannyMat);
		//cv::imshow("cannyDilateMat", cannyDilateMat);
		//cv::waitKey(0);
		// todo 积分图待续
	}

	// todo 处理一下次层膨胀然后降采样到顶层的操作


	return 0;
}

int ContourToolProcess::doTask(ContourUtility &contourUtility, IVSContourResult& ivsContourResult){

	// 由于确定高速度策略也做到最底层，为了改动较少，只修改这三处，其一
	//INT8 com_level = palg_handle->contour_param->algo_strategy;
	INT8 com_level = 0;
	INT8 com_score = 20;

		//hi_ive_integ_at(&(tool_utility->search_region_binary[l]), &(tool_utility->image_search_integ[l]), 0, 0, &integ_ctrl_s);

	// 和建模版上统一方式，将第二层轮廓二值图膨胀然后再降采样到顶层，保证点数够多

	
	do_template_match_new(palg_handle->orignal_pic, (Koyo_Template_Match_Info *)palg_handle->run_tool_param,
		tool_utility, pic_corr, is_global, is_highspeed);
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
	/* 将结果填充到结构体中:
	* 1. result_bitmap;
	* 2. result_ret;
	* 3. current_time;
	* */
	//    KOYO_LOG_INFO("here\n");
	//


	// 新的评分策略，对于全局评分和局部评分都是有用的，评分原理是（计算得出的评分*（搜索框内的点数/模板总点数）），在这里进行一次修改

	IPoint left_top_point, right_bottom_point;
	left_top_point.x = pstTemplateMatch->rect_roi[0].x;
	left_top_point.y = pstTemplateMatch->rect_roi[0].y;
	right_bottom_point.x = pstTemplateMatch->rect_roi[0].x + pstTemplateMatch->rect_roi[0].width;
	right_bottom_point.y = pstTemplateMatch->rect_roi[0].y + pstTemplateMatch->rect_roi[0].height;

	UINT32 point_sum = 0;
	for (UINT32 i = 0; i < (((Koyo_Template_Match_Info *)palg_handle->run_tool_param)->tpls[0][(*res_angel)]).noOfCordinates; ++i){
		IPoint p = (((Koyo_Template_Match_Info *)palg_handle->run_tool_param)->tpls[0][(*res_angel)]).cordinates[i];
		// 判断该点是否在搜索范围内
		p.x += res_coor->x << com_level;
		p.y += res_coor->y << com_level;
		if (p.x >= left_top_point.x && p.x <= right_bottom_point.x && p.y >= left_top_point.y && p.y <= right_bottom_point.y){
			point_sum++;
		}
	}
	double percent = (double)point_sum / (((Koyo_Template_Match_Info *)palg_handle->run_tool_param)->tpls[0][(*res_angel)]).noOfCordinates;
	KOYO_LOG_INFO("the num of points in detection area is %d, the num of model points is %d, the percent is %lf\n", point_sum, (((Koyo_Template_Match_Info *)palg_handle->run_tool_param)->tpls[0][(*res_angel)]).noOfCordinates, percent);
	*res_score *= percent;


	palg_handle->result_ret->tool_value = (short)(*res_score + 0.5); // 四舍五入
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
	/* 最后运行时检测出来的位置 */
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

	// 如果是全局的话，需要赋值，局部的话已经在矫正地方进行了赋值
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

		// 由于之前显示的是外接框，为了显示检测框，需要进行计算，当然也可以移植到客户端上进行计算
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

		// 处理在配置阶段非水平检测框显示外接框的问题

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

	//// 只计算局部计算搜索框的坐标


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

	//    矫正的轮廓需要是0-360度的

	INT8 ang_range = palg_handle->contour_param->angle_range;

	// 0 角度的模版的位置
	UINT16 tpl0_size = ang_range / temp_ang0; // 0层上的模版个数
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

	// 矫正只能在全局轮廓时候才能重置旋转平移矩阵
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
}

int ContourToolProcess::freeTask(){
	/* 由内至外释放内存, 只释放工具的内容, 查找表也得释放了，目前查找表是在各个工具中的，可以考虑移出来 */
	INT32 i = 0, j = 0;
	Koyo_Template_Match_Info *pstTemplate_match_info = (Koyo_Template_Match_Info*)palg_handle->run_tool_param;

	KOYO_LOG_INFO("here.\n");
	/* 释放所有的模板 */
	UINT16 tplsize;
	for (INT32 tpl_i = 0; tpl_i<pstTemplate_match_info->run_time_npyramid; ++tpl_i)
	{
		/* 释放所有的模板 */
		free(pstTemplate_match_info->tpls[tpl_i]);
		//        KOYO_LOG_INFO("here, %d %d.\n", tpl_i, run_tool_param->run_time_npyramid);
	}
	free(pstTemplate_match_info->tpls);
	KOYO_LOG_INFO("here.\n");

	//    INT16 nangs_temp = (INT16)ceil((double)MAX_DEGREE/pstTemplate_match_info->search_angel_nstep[1]);
	//    int width = tool_utility->u16Width >> 1;
	//    int height = tool_utility->u16Height >> 1;
	//    for (INT16 wtemp = 0; wtemp < width; ++wtemp) {
	//        for (INT16 htemp = 0; htemp < height; ++htemp) {
	//            free(pstTemplate_match_info->iscompute[wtemp][htemp]);
	//        }
	//        free(pstTemplate_match_info->iscompute[wtemp]);
	//    }
	//    free(pstTemplate_match_info->iscompute);

	KOYO_LOG_INFO("here.\n");
}
//
//INT8 match_template_one_degree(Koyo_Template_Match_Info *template_info, IVE_IMAGE_S *search_region_bin,
//	CandidateResult *candidateResult, Tool_Utility *tool_utility, float rate)  {
//
//	INT32 nCoord;
//	INT16 level = candidateResult->level;
//	TemplateStruct *tpl = &template_info->tpls[level][candidateResult->angel_idx];
//	nCoord = tpl->noOfCordinates;
//	float* tpl_edgex = tpl->edgeDerivativeX;
//	float* tpl_edgey = tpl->edgeDerivativeY;
//	IPoint *tpl_point_coord = tpl->cordinates;
//	float **edgex = tool_utility->edgeX[level];
//	float **edgey = tool_utility->edgeY[level];
//
//	INT16 prefixi, prefixj;
//	prefixi = candidateResult->position.y;
//	prefixj = candidateResult->position.x;
//	//    if (candidateResult->angel_idx == 0){
//	//    {
//	//        //统计每层待测图片二值化点数量
//	//        IVE_IMAGE_S integ;
//	//        SAMPLE_COMM_IVE_CreateImage(&integ, IVE_IMAGE_TYPE_U32C1, search_region_bin->u16Width,search_region_bin->u16Height);
//	//        IVE_INTEG_CTRL_S integ_ctrl_s = {.enOutCtrl = IVE_INTEG_OUT_CTRL_SUM};
//	//        hi_ive_integ_at(search_region_bin, &integ, 0, 0, &integ_ctrl_s);
//	//        Rect match_rect;
//	//        match_rect.width = match_rect.height = template_info->search_rect_width[level];
//	//        match_rect.x = prefixj - match_rect.width>>1;
//	//        match_rect.y = prefixi - match_rect.height>>1;
//	//        //KOYO_LOG_INFO("height:%d, width:%d, x:%d, y:%d.\n", match_rect.height, match_rect.width, match_rect.x, match_rect.y);
//	//        int points_level = get_contour_npoints_by_rect(&integ, &match_rect);
//	//        //KOYO_LOG_INFO("******************INFO*******************points_level %d: %d\n", level, points_level);
//	//        //KOYO_LOG_INFO("template ncord:%d.\n", nCoord);
//	//        HI_IMAGES_S_RELEASE(&integ);
//	//    }
//	bool **iscalculated = tool_utility->is_calculated[level];
//
//	HI_S16 *dx_s16_p, *dy_s16_p;
//	dx_s16_p = (HI_S16*)tool_utility->Sdx[level].pu8VirAddr[0];
//	dy_s16_p = (HI_S16*)tool_utility->Sdy[level].pu8VirAddr[0];
//
//	INT16 stride_dx = tool_utility->Sdx[level].u16Stride[0];
//
//	INT16 step = nCoord / STEP[level];
//	step = step ? step : 1;   //若步长小于0则步长为1
//	//    step = 3;
//	//    KOYO_LOG_INFO("level nCoord:%d %d.\n", candidateResult->level, nCoord);
//	//    KOYO_LOG_INFO("coord:%d.\n", nCoord);
//	//    if(candidateResult->level == 0)
//	//        step = 1;
//	//    if(candidateResult->level == 3 && candidateResult->angel_idx == 15)
//	//    {
//	//        KOYO_LOG_INFO("here, %d %d.\n", prefixi, prefixj);
//	//    }
//
//	ne10_vec2f_t src1, src2;
//
//#if 1
//	//    /* 根据模板点数量决定一二级搜索以及膨胀角度范围阈值*/
//	//    HI_S32 template_points = template_info->tpls[0][0].noOfCordinates;
//	//    //最小点为0度该层模板的特征点数乘以阈值
//	//    //二级筛选
//	////    float rate = (float)(template_info->contour_parameter.bot_threshold)/100.f*0.4;
//	////    float rate;
//	//    if(template_points < POINTS_MORE) {
//	//        rate = RATE_TWO_OTHER;
//	//        if(level == 4)
//	//            rate = RATE_TWO_FOUR;
//	//        else if(level == 3)
//	//            rate = RATE_TWO_THREE;
//	//        else if(level == 2)
//	//            rate = RATE_TWO_TWO;
//	//        else if(level == 1)
//	//            rate = RATE_TWO_ONE;
//	//    } else{
//	//        rate = RATE_TWO_OTHER_MORE;
//	//        if(level == 4)
//	//            rate = RATE_TWO_FOUR_MORE;
//	//        else if(level == 3)
//	//            rate = RATE_TWO_THREE_MORE;
//	//        else if(level == 2)
//	//            rate = RATE_TWO_TWO_MORE;
//	//        else if(level == 1)
//	//            rate = RATE_TWO_ONE_MORE;
//	//
//	//    }
//	if (__GMP_UNLIKELY(candidateResult->level == template_info->run_time_npyramid - 1)){
//		INT32 min_points_degree = (INT32)(rate * (float)tpl->noOfCordinates);
//		IPoint prefix;
//		prefix.x = prefixj;
//		prefix.y = prefixi;
//		//INT32 npoints_search_img = get_contour_npoints_by_coordinate(&img_for_filter2, tpl->cordinates,
//		//		tpl->noOfCordinates, &prefix, min_points_degree);
//
//		INT32 npoints_search_img = get_contour_npoints_by_coordinate(search_region_bin, tpl->cordinates,
//			tpl->noOfCordinates, &prefix, min_points_degree);
//
//		//                    IVE_IMAGE_S tmp;
//		//                    SAMPLE_COMM_IVE_CreateImage(&tmp,IVE_IMAGE_TYPE_U8C1,search_rect_width,search_rect_width);
//		//                    memset(tmp.pu8VirAddr[0],0,tmp.u16Stride[0]*tmp.u16Height);
//		//                    for (int m = 0; m < tpl_one_degree_p->noOfCordinates; ++m) {
//		//                        tmp.pu8VirAddr[0][(tpl_one_degree_p->cordinates[m].y+prefix.y)*tmp.u16Stride[0] + tpl_one_degree_p->cordinates[m].x + prefix.x] = 1;
//		//                    }
//		//                    board_tools_write_to_file_u8c1_formated("../image/search_coord_recover",tmp.u16Stride[0],tmp.u16Height,tmp.pu8VirAddr[0]);
//
//		//    点数不够，重新换个角度
//		if (__GMP_LIKELY(npoints_search_img < min_points_degree)){
//			KOYO_LOG_TRACE("unable to pass filter two...: npoints_search_img: %d, min_points_degree: %d\n", npoints_search_img, min_points_degree);
//			return 1;
//		}
//	}
//	//    return 1;
//#endif
//
//	float score = 0, tmp_score;
//	INT32 tmp_nCoord = 0;
//	for (int m = 0; m < nCoord; m += step) {
//		//                        prefix.x = tpl_one_degree_p->centerOfGravity.x;
//		//                        prefix.y = tpl_one_degree_p->centerOfGravity.y;
//		INT16 coordx, coordy;
//		coordx = tpl_point_coord->x + prefixj;
//		coordy = tpl_point_coord->y + prefixi;
//		if (coordy < 0
//			|| coordy >= search_region_bin->u16Height
//			|| coordx < 0
//			|| coordx >= search_region_bin->u16Width) {
//			//                            KOYO_LOG_INFO("y,x %d %d. prefix.y %d\n",coordy,coordx,prefix.y);
//			tpl_point_coord += step;
//			continue;
//		}
//
//		//                        KOYO_LOG_INFO("search_region_bin->u16Width cdx cdy: %d %d %d.\n",search_region_bin->u16Width,coordx,coordy);
//		if (!iscalculated[coordy][coordx]) {
//			INT16 tmpdx, tmpdy;
//			//                            KOYO_LOG_INFO("stride cdx cdy: %d %d %d sb strid.\n",stride,coordx,coordy);
//			tmpdx = dx_s16_p[stride_dx * coordy + coordx];
//			tmpdy = dy_s16_p[stride_dx * coordy + coordx];
//
//			edgex[coordy][coordx] = lookupTableX[abs(tmpdx)][abs(tmpdy)];  //y is height,x is width
//			edgey[coordy][coordx] = lookupTableY[abs(tmpdx)][abs(tmpdy)];
//			if (tmpdx < 0) {
//				edgex[coordy][coordx] *= -1.f;
//			}
//			if (tmpdy < 0) {
//				edgey[coordy][coordx] *= -1.f;
//			}
//			//            if (candidateResult->level == 4)
//			//            {
//			//                    KOYO_LOG_INFO("%d %d %f %f.\n", tmpdx, tmpdy, edgex[coordy][coordx], edgey[coordy][coordx]);
//			//            }
//			//            if (candidateResult->level == 4)
//			//            {
//			//                KOYO_LOG_INFO("%f %f.\n", edgex[coordy][coordx], edgey[coordy][coordx]);
//			//            }
//			//                            printf("%f ",edgex[coordy][coordx] * edgex[coordy][coordx] + edgey[coordy][coordx] * edgey[coordy][coordx]);
//			iscalculated[coordy][coordx] = 1;
//		}
//		//                        if(j == 42 && i == 34 && k == 0){
//		//                        KOYO_LOG_INFO("%f,%f and %f,%f, length %1.1f\n",tpl_edgex[m],tpl_edgey[m],
//		//                                   edgex[tpl_point_coord->y + prefix.y][tpl_point_coord->x + prefix.x],
//		//                                   edgey[tpl_point_coord->y + prefix.y][tpl_point_coord->x + prefix.x],
//		//                                   edgex[tpl_point_coord->y + prefix.y][tpl_point_coord->x + prefix.x]
//		//                                   *edgex[tpl_point_coord->y + prefix.y][tpl_point_coord->x + prefix.x]
//		//                                   +edgey[tpl_point_coord->y + prefix.y][tpl_point_coord->x + prefix.x]
//		//                                    *edgey[tpl_point_coord->y + prefix.y][tpl_point_coord->x + prefix.x] );
//		//                        }
//		src1.x = tpl_edgex[m];
//		src1.y = tpl_edgey[m];
//		src2.x = edgex[coordy][coordx];
//		src2.y = edgey[coordy][coordx];
//
//		ne10_dot_vec2f_neon(&tmp_score, &src1, &src2, 1);
//		tmp_nCoord++;
//		//        if (candidateResult->level == 2)
//		//        {
//		////            if (prefixi == 64 && prefixj  == 87 && candidateResult->angel_idx == 22) {
//		////                KOYO_LOG_INFO("%8.6f %8.6f %8.6f %8.6f %8.6f.\n", tmp_score, src1.x, src2.x, src1.y, src2.y);
//		////            }
//		//            if (prefixi == 47 && prefixj  == 73 && candidateResult->angel_idx == 89) {
//		////                KOYO_LOG_INFO("%d %d %d %d   %8.6f %8.6f %8.6f %8.6f.\n", tpl_point_coord->x, tpl_point_coord->y, coordx, coordy, src1.x, src2.x, src1.y, src2.y);
//		//                KOYO_LOG_INFO("%8.6f %8.6f %8.6f %8.6f %8.6f.\n", tmp_score, src1.x, src2.x, src1.y, src2.y);
//		////                KOYO_LOG_INFO("coord:%d.\n", nCoord);
//		//            }
//		//        }
//		score += tmp_score;
//		//        if((score*step+nCoord-m)*100.f/nCoord < score_th)
//		//        {
//		//            return 1;
//		//        }
//		//                        score += tpl_edgex[m]*edgex[tpl_point_coord->y + prefix.y][tpl_point_coord->x + prefix.x] +
//		//                                 tpl_edgey[m]*edgey[tpl_point_coord->y + prefix.y][tpl_point_coord->x + prefix.x];
//		tpl_point_coord += step;
//	}
//	//此处加入分数终止条件
//	candidateResult->score = score / nCoord*100.f*(float)step;
//	//    exit(0);
//	return 0;
//}
////ang_region_idx_llist
//enum {
//	ANG_REGION_LLIST_ANG, ANG_REGION_LLIST_PREV, ANG_REGION_LLIST_NEXT, ANG_REGION_LLIST_CUR_SIZE
//};
//
//
////匹配一层的模板
////输入是图像的二值轮廓图以及梯度图
////IVE_IMAGE_S *search_region,                           搜索区域的二值轮廓图
////Koyo_Template_Match_Info *pstTemplateMatch,           结构指针
////                                                      结构指针中需要建立搜索区域的梯度edgeX,edgeY，
////                                                      搜索区域的积分轮廓图image_search_integ
////CandidateResult *candidateResult_in,                  输入的候选点集合
////UINT32 ncandidate_in,                                 数量
////CandidateResult *candidateResult_out,                 输出候选点集合
////UINT32 *ncandidate_out,                               输出数量，如果为1表示最后一层，只输出一个结果，否则必须大于0，表示了数组的长度
//INT8 match_template_one_level(IVE_IMAGE_S *search_region_bin, IVE_IMAGE_S *dilated_search_region_bin, Koyo_Template_Match_Info *template_match_info,
//	CandidateResultMinHeapStruct *candidate_heap_in,
//	CandidateResultMinHeapStruct *candidate_heap_out,
//	Tool_Utility *tool_utility, UINT8 level, Pic_Corr *pic_corr) {
//	if (__GMP_UNLIKELY(search_region_bin == NULL)){
//		KOYO_LOG_ERROR("error nullptr search_region_bin.\n");
//		return -1;
//	}
//	if (__GMP_UNLIKELY(template_match_info == NULL)){
//		KOYO_LOG_ERROR("error nullptr template_match_info.\n");
//		return -1;
//	}
//	// 为了保证最大的图片匹配
//	//if(__GMP_UNLIKELY(template_match_info->search_rect_width[level] > MAX_WIDTH)){
//	//    KOYO_LOG_ERROR("error nullptr template_match_info->search_rect_width[%d](%d) invalid, must smaller than %d.\n",
//	//               level,template_match_info->search_rect_width[level],MAX_WIDTH);
//	//    return -1;
//	//}
//	if (__GMP_UNLIKELY(candidate_heap_in == NULL)){
//		KOYO_LOG_ERROR("error nullptr candidate_heap_in.\n");
//		return -1;
//	}
//	if (__GMP_UNLIKELY(candidate_heap_out == NULL)){
//		KOYO_LOG_ERROR("error nullptr candidate_heap_out.\n");
//		return -1;
//	}
//	//    KOYO_LOG_INFO("tpl[4][30].noofcordinates %d;x %d;y %d.\n", template_match_info->tpls[4][30].noOfCordinates, \
//	    template_match_info->tpls[4][30].centerOfGravity.x, template_match_info->tpls[4][30].centerOfGravity.y);
//
//	// 高速度策略也检测到底层，保证改动较少，只修改这三处，其三
//	//INT8 com_level = template_match_info->contour_parameter.algo_strategy;
//	INT8 com_level = 0;
//
//	Rect match_rect;
//	INT32 npoints_search_img;
//	struct timeval tt[2];
//	start(tt);
//	//旋转前后模板点数可能会有差异，简单起见对比率乘以0.9以统筹旋转的情况
//	//    float rate = (float)(template_match_info->contour_parameter.bot_threshold)/100.f*0.8f;
//	INT16 raw_start = 0, raw_end = 0, col_start = 0, col_end = 0, stride = 0;
//	INT16 ang_start = 0, ang_end = 0;
//	IPoint *pointArray;
//	IVE_IMAGE_S search_bin_block = *search_region_bin;
//	TemplateStruct *tpl_one_level_p = template_match_info->tpls[level];
//	//    TemplateStruct *tpl_one_degree_p = NULL;
//	float maxscore = -1;
//	float tmp_score;
//	int maxi = 0, maxj = 0;
//	int maxang = 0;
//	UINT16 search_rect_width = template_match_info->search_rect_width[level];
//	UINT16 search_rect_width_half = search_rect_width >> 1;
//	match_rect.height = match_rect.width = search_rect_width;
//	INT16 nangs = (INT16)ceil((double)(template_match_info->contour_parameter.angle_range * 2) / template_match_info->search_angel_nstep[level]), currang;
//	//    KOYO_LOG_INFO("nangs:%d\n", nangs);
//
//	CandidateResult candidateResult;
//	candidateResult.level = level;
//
//	/* 根据模板点数量决定一二级搜索以及膨胀角度范围阈值*/
//	HI_S32 template_points = template_match_info->tpls[0][0].noOfCordinates;
//	//最小点为0度该层模板的特征点数乘以阈值
//	//二级筛选
//	//    float rate = (float)(template_info->contour_parameter.bot_threshold)/100.f*0.4;
//	float rate;
//	if (template_points < POINTS_MORE) {
//		rate = RATE_TWO_OTHER;
//		if (level == 4)
//			rate = RATE_TWO_FOUR;
//		else if (level == 3)
//			rate = RATE_TWO_THREE;
//		else if (level == 2)
//			rate = RATE_TWO_TWO;
//		else if (level == 1)
//			rate = RATE_TWO_ONE;
//	}
//	else{
//		rate = RATE_TWO_OTHER_MORE;
//		if (level == 4)
//			rate = RATE_TWO_FOUR_MORE;
//		else if (level == 3)
//			rate = RATE_TWO_THREE_MORE;
//		else if (level == 2)
//			rate = RATE_TWO_TWO_MORE;
//		else if (level == 1)
//			rate = RATE_TWO_ONE_MORE;
//
//	}
//	//    IVE_IMAGE_S tmpimg;
//	//    SAMPLE_COMM_IVE_CreateImage(&tmpimg,IVE_IMAGE_TYPE_U32C1,search_region_bin->u16Width,search_region_bin->u16Height);
//	//    memset(tmpimg.pu8VirAddr[0],0,tmpimg.u16Stride[0]*tmpimg.u16Height);
//	//    float *data = tmpimg.pu8VirAddr[0];
//	//    for (int i = 0; i < search_region_bin->u16Height; ++i) {
//	//        for (int j = 0; j < search_region_bin->u16Width; ++j) {
//	//            data[i*tmpimg.u16Stride[0] + j] = edgex[i][j];
//	//        }
//	//    }
//	//    board_tools_write_to_file_floatc1_formated("../image/contour_edge_e",tmpimg.u16Stride[0],tmpimg.u16Height,tmpimg.pu8VirAddr[0]);
//	//    exit(0);
//	int ct_pass_filter1 = 0;
//	int ct_pass_filter2 = 0;
//	if (level == template_match_info->run_time_npyramid - 1){ // 顶层
//		/* 根据模板点数量决定一二级搜索以及膨胀角度范围阈值*/
//		int avg_top_points = 0;
//		ang_start = 0; ang_end = nangs;
//		for (INT16 k = ang_start; k < ang_end; k += 1) {
//			avg_top_points += template_match_info->tpls[level][k].noOfCordinates;
//		}
//		avg_top_points /= (ang_end - ang_start + 1);
//
//		HI_S32 template_points = template_match_info->tpls[0][0].noOfCordinates;
//		//最小点为0度该层模板的特征点数乘以阈值
//		INT32 min_points, max_points;
//		if (template_points > POINTS_MORE) {
//			min_points = (INT32)(RATE_ONE_MORE * (float)avg_top_points);
//			max_points = (INT32)((1.30) * (float)avg_top_points);
//		}
//		else {
//			min_points = (INT32)(RATE_ONE * (float)avg_top_points);
//			max_points = (INT32)((1.30) * (float)avg_top_points);
//		}
//		KOYO_LOG_INFO("min_points: %d, max_points: %d, avg_points: %d\n", min_points, max_points, avg_top_points);
//		//顶层
//		//        raw_start = 0;raw_end = search_region_bin->u16Height;
//		//        col_start = 0;col_end = search_region_bin->u16Width; //************************此处可根据参数决定搜索框，而不是全局搜索
//
//		raw_start = template_match_info->rect_roi[level].y;
//		raw_end = raw_start + template_match_info->rect_roi[level].height;
//		col_start = template_match_info->rect_roi[level].x;
//		col_end = col_start + template_match_info->rect_roi[level].width;
//
//
//
//		//************************此处可根据参数决定搜索框，而不是全局搜索
//
//		//        raw_start = 0;raw_end = 1;
//		//        col_start = 0;col_end = 1;
//		ang_start = 0; ang_end = nangs;
//
//		float temp_ang0 = template_match_info->search_angel_nstep[0];
//		UINT16 result_angle_for_corr = pic_corr->angel / temp_ang0; // 实际旋转的角度
//		INT16 ang_corr = 0;
//		INT8 ang_range = template_match_info->contour_parameter.angle_range;
//		if ((!(template_match_info->contour_parameter.search_rect_x == 0 && template_match_info->contour_parameter.search_rect_y == 0 && template_match_info->contour_parameter.search_rect_width == IMAGE_MAX_WIDTH
//			&& template_match_info->contour_parameter.search_rect_height == IMAGE_MAX_HEIGHT) && !(fabs(pic_corr->model_point.x) < 0.000001 && fabs(pic_corr->pic_point.x) < 0.000001 && fabs(pic_corr->model_point.y) < 0.000001 && fabs(pic_corr->pic_point.y) < 0.000001 && fabs(pic_corr->angel) < 0.000001))){
//			if (result_angle_for_corr < ang_range) {
//				ang_corr = result_angle_for_corr + ang_range;
//			}
//			else {
//				ang_corr = ang_range - (360 - result_angle_for_corr);
//			}
//
//			int local_search_angle_expand = -1;
//			int local_search_coord_expand = -1;
//			if (template_match_info->contour_parameter.algo_strategy == 0) {
//				// 高精度
//				local_search_angle_expand = LOCAL_CONTOUR_ANGLE_RANGE_EXPAND;
//				local_search_coord_expand = LOCAL_CONTOUR_COORD_RANGE_EXPAND;
//			}
//			else {
//				// 高速度
//				local_search_angle_expand = LOCAL_CONTOUR_ANGLE_RANGE_EXPAND_HIGHSPEED;
//				local_search_coord_expand = LOCAL_CONTOUR_COORD_RANGE_EXPAND_HIGHSPEED;
//			}
//			ang_corr /= template_match_info->search_angel_nstep[level];
//			ang_start = ang_corr - local_search_angle_expand;
//			ang_end = ang_corr + local_search_angle_expand;
//
//			ang_start = ang_start >= 0 ? ang_start : 0;
//			ang_end = ang_end < (ang_range * 2 / template_match_info->search_angel_nstep[level]) ? ang_end : (ang_range * 2 / template_match_info->search_angel_nstep[level] - 1);
//
//			KOYO_LOG_INFO("ang_corr : %d ang_start: %d, ang_end: %d, temp_ang0: %f, result_angle_for_corr: %d, ang_range: %d, nangs: nangs: %d, ang_step: %f\n", ang_corr, ang_start, ang_end, temp_ang0, result_angle_for_corr, ang_range, nangs, template_match_info->search_angel_nstep[level]);
//			TRMat mat;
//			IPointf srcp, dstp, center;
//			KOYO_LOG_INFO("template_match_info->contour_parameter.ext_rect_x :%d, template_match_info->contour_parameter.ext_rect_y: %d\n", template_match_info->contour_parameter.ext_rect_x, template_match_info->contour_parameter.ext_rect_y);
//
//			INT16 nangs0 = (INT16)ceil((double)(template_match_info->contour_parameter.angle_range * 2) / template_match_info->search_angel_nstep[0]), currang;
//			srcp.x = template_match_info->tpls[0][nangs0 / 2].centerOfGravity.x + (template_match_info->contour_parameter.ext_rect_x);
//			srcp.y = template_match_info->tpls[0][nangs0 / 2].centerOfGravity.y + (template_match_info->contour_parameter.ext_rect_y);
//			compute_trmat(&mat, *pic_corr);
//			compute_cord(mat, pic_corr->model_point, srcp, &dstp);
//
//			raw_start = (INT16)dstp.y - local_search_coord_expand;
//			raw_start = raw_start >= 0 ? raw_start : 0;
//
//			raw_end = (INT16)dstp.y + local_search_coord_expand;
//			raw_end = raw_end < (MAX_HEIGHT) ? raw_end : (MAX_HEIGHT);
//
//			col_start = (INT16)dstp.x - local_search_coord_expand;
//			col_start = col_start >= 0 ? col_start : 0;
//
//			col_end = (INT16)dstp.x + local_search_coord_expand;
//			col_end = col_end < (MAX_WIDTH) ? col_end : (MAX_WIDTH);
//
//			KOYO_LOG_INFO("srcp.x: %f, srcp.y: %f, dstp.x: %f, dstp.y: %f, raw_start: %d, raw_end: %d, col_start: %d, col_end: %d\n", srcp.x, srcp.y, dstp.x, dstp.y, raw_start, raw_end, col_start, col_end);
//			raw_start >>= level;
//			raw_end >>= level;
//			col_start >>= level;
//			col_end >>= level;
//
//			KOYO_LOG_INFO("srcp.x: %f, srcp.y: %f, dstp.x: %f, dstp.y: %f, raw_start: %d, raw_end: %d, col_start: %d, col_end: %d\n", srcp.x, srcp.y, dstp.x, dstp.y, raw_start, raw_end, col_start, col_end);
//		}
//
//		KOYO_LOG_INFO("search region: %d %d. search block: %d.\n", search_region_bin->u16Width, search_region_bin->u16Height, search_rect_width);
//		KOYO_LOG_INFO("raw: %d,%d. col: %d,%d.\n", raw_start, raw_end, col_start, col_end);
//		//char *fn[100];
//		//sprintf(fn, "/sd/integ_top_level_%d_%d.txt", tool_utility->image_search_integ[1][level].u16Width, tool_utility->image_search_integ[1][level].u16Height);
//		//printHiImage16(fn, &tool_utility->search_region_binary[1][level]);
//
//		//        IVE_IMAGE_S tmp_search_block;
//		//        tpl_one_degree_p = tpl_one_level_p;
//		//        SAMPLE_COMM_IVE_CreateImage(&tmp_search_block,IVE_IMAGE_TYPE_U8C1,search_rect_width,search_rect_width);
//		//        memset(tmp_search_block.pu8VirAddr[0],0,tmp_search_block.u16Stride[0]*tmp_search_block.u16Height);
//		//        for (int m = 0; m < tpl_one_degree_p->noOfCordinates; ++m) {
//		//            tmp_search_block.pu8VirAddr[0][(tpl_one_degree_p->cordinates[m].y+prefix.y)*tmp_search_block.u16Stride[0] + tpl_one_degree_p->cordinates[m].x + prefix.x] = 1;
//		//        }
//		//        board_tools_write_to_file_u8c1_formated("../image/search_coord_recover",tmp_search_block.u16Stride[0],tmp_search_block.u16Height,tmp_search_block.pu8VirAddr[0]);
//		//
//		//        board_tools_write_to_file_u8c1_formated("../image/search_region_revover",
//		//                                                search_region_bin->u16Stride[0],
//		//                                                search_region_bin->u16Height,
//		//                                                search_region_bin->pu8VirAddr[0]);
//		//
//		//
//		//        exit(0);
//		//        int filter1Passed = 0;
//		//        int canditate_cnt = 0;
//
//		// height 和 width中较大的值
//		int max_hw_all = 0;
//		for (INT16 k = ang_start; k < ang_end; k += 1){
//			int max_hw = (template_match_info->tpls[level][k].modelHeight > template_match_info->tpls[level][k].modelWidth) ? template_match_info->tpls[level][k].modelHeight : template_match_info->tpls[level][k].modelWidth;
//			if (max_hw_all < max_hw){
//				max_hw_all = max_hw;
//			}
//			KOYO_LOG_TRACE("max_hw: %d, max_hw_all: %d, template_match_info->tpls[level][k].modelHeight: %d, template_match_info->tpls[level][k].modelWidth: %d\n", max_hw, max_hw_all, template_match_info->tpls[level][k].modelHeight, template_match_info->tpls[level][k].modelWidth);
//		}
//		//max_hw /= 1;
//
//		match_rect.height = max_hw_all;
//		match_rect.width = max_hw_all;
//		KOYO_LOG_INFO("match_rect.height: %d , match_rect.width: %d, hw: %d search_rect_width_half: %d\n", match_rect.height, match_rect.width, max_hw_all, search_rect_width_half);
//		int step = 2;
//		for (INT16 i = raw_start; /*printf("%d < %d,",i,raw_end),*/i < raw_end; i += step) {
//			for (INT16 j = col_start; j < col_end; j += step) {
//				match_rect.x = j - max_hw_all / 2;
//				match_rect.y = i - max_hw_all / 2;
//				//match_rect.x = j - search_rect_width_half;
//				//match_rect.y = i - search_rect_width_half;
//				candidateResult.position.y = i;
//				candidateResult.position.x = j;
//				//                tmp_candidate_result.position.x = j;
//				//                tmp_candidate_result.position.y = i;
//				//                KOYO_LOG_INFO("match_rect: %d,%d,%d,%d.\n",match_rect.x,match_rect.y,match_rect.width,match_rect.height);
//				//                INT32 * data = template_match_info->image_search_integ[level].pu8VirAddr[0];
//				//                for (int m = 0; m < template_match_info->image_search_integ[level].u16Height; ++m) {
//				//                    for (int n = 0; n < template_match_info->image_search_integ[level].u16Width; ++n) {
//				//                        printf("%4d ",
//				//                               data
//				//                               [m*template_match_info->image_search_integ[level].u16Stride[0]+n]);
//				//                    }
//				//                    printf("\n");
//				//                }
//
//
//				npoints_search_img = get_contour_npoints_by_rect(&tool_utility->image_search_integ[level], &match_rect);
//
//
//				if (__GMP_UNLIKELY(npoints_search_img < 0)){
//					KOYO_LOG_ERROR("Error: get points error.\n");
//					KOYO_LOG_ERROR("x,y,w,h %d,%d,%d,%d.\n", match_rect.x, match_rect.y, match_rect.width, match_rect.height);
//					return -1;
//				}
//				//if(__GMP_LIKELY(npoints_search_img < min_points || npoints_search_img > max_points)){
//				//if(__GMP_LIKELY(npoints_search_img < min_points || npoints_search_img > max_points)){
//				if (__GMP_LIKELY(npoints_search_img < min_points)){
//					//                    KOYO_LOG_ERROR("npoints_search_img < min_points. %d,%d\n",npoints_search_img,min_points);
//					// continue;
//				}
//				//搜索框内点数大于K*S，开始对每个角度进行判定
//				//tpl_one_degree_p 对应一个角度的模板
//				//                tpl_one_degree_p = tpl_one_level_p-1;
//				//搜索框在这里开始就不会变了
//				search_bin_block.pu8VirAddr[0] = search_region_bin->pu8VirAddr[0] + i*search_region_bin->u16Stride[0] + j;
//				ct_pass_filter1++;
//				for (INT16 k = ang_start; k < ang_end; k += 1) {
//					KOYO_LOG_TRACE("pos start...\n");
//					candidateResult.angel_idx = k;
//
//					//二级筛选
//					//                    npoints_search_img = get_contour_npoints_by_coordinate(search_region_bin, tpl_one_degree_p->cordinates,
//					//                                                                           tpl_one_degree_p->noOfCordinates, &prefix);
//
//					//                    IVE_IMAGE_S tmp;
//					//                    SAMPLE_COMM_IVE_CreateImage(&tmp,IVE_IMAGE_TYPE_U8C1,search_rect_width,search_rect_width);
//					//                    memset(tmp.pu8VirAddr[0],0,tmp.u16Stride[0]*tmp.u16Height);
//					//                    for (int m = 0; m < tpl_one_degree_p->noOfCordinates; ++m) {
//					//                        tmp.pu8VirAddr[0][(tpl_one_degree_p->cordinates[m].y+prefix.y)*tmp.u16Stride[0] + tpl_one_degree_p->cordinates[m].x + prefix.x] = 1;
//					//                    }
//					//                    board_tools_write_to_file_u8c1_formated("../image/search_coord_recover",tmp.u16Stride[0],tmp.u16Height,tmp.pu8VirAddr[0]);
//
//					//点数不够，重新换个角度
//					//                    if(__GMP_LIKELY(npoints_search_img < min_points)){
//					//                        continue;
//					//                    }
//					//暂时去除二级筛选
//					//点数够，算分数
//					//                    IVE_IMAGE_S tmp;
//					//                    SAMPLE_COMM_IVE_CreateImage(&tmp,IVE_IMAGE_TYPE_U32C1,search_rect_width,search_rect_width);
//					//                    INT32 *datatmp = tmp.pu8VirAddr[0];
//					//                    memset(tmp.pu8VirAddr[0],0,tmp.u16Stride[0]*tmp.u16Height*sizeof(INT32));
//					//                    for (int m = 0; m < tpl_one_degree_p->noOfCordinates; ++m) {
//					//                        datatmp[(tpl_one_degree_p->cordinates[m].y + prefix.y)*tmp.u16Stride[0]+tpl_one_degree_p->cordinates[m].x + prefix.x] =
//					//                        tpl_one_degree_p->edgeDerivativeX[m]*100;
//					//                    }
//					//                    board_tools_write_to_file_u32c1_formated("../image/contour_tpl",tmp.u16Stride[0],tmp.u16Height,tmp.pu8VirAddr[0]);
//					//                    exit(0);
//					//                    UINT16 score_th = template_match_info->contour_parameter.bot_threshold / 3;
//					INT8 s8_ret = match_template_one_degree(template_match_info, dilated_search_region_bin,
//						&candidateResult, tool_utility, rate);
//
//					if (s8_ret)
//					{
//						continue;    // return 1: points is not enough
//					}
//					if (candidateResult.score > maxscore){
//						maxscore = candidateResult.score;
//						maxi = i; maxj = j;
//						maxang = k;
//					}
//					ct_pass_filter2++;
//					if (candidate_result_min_heap_if_need_add(candidate_heap_out, candidateResult.score)){
//						candidate_result_min_heap_add_item(candidate_heap_out, &candidateResult);
//						//                        ++canditate_cnt;
//						//                        KOYO_LOG_INFO("level:%d, add canditate to heap:%d\n", level, canditate_cnt);
//					}
//					KOYO_LOG_TRACE("pos end...\n");
//
//					//                    if(j == 41 && i == 34 && k == 0){
//					//                        KOYO_LOG_INFO("tpl_one_degree_p->noOfCordinates %d.\n",tpl_one_degree_p->noOfCordinates);
//					//                        KOYO_LOG_INFO("score %f.\n",score);
//					//                        for (int p = 0; p < search_region_bin->u16Height; ++p) {
//					//                            for (int q = 0; q < search_region_bin->u16Width; ++q) {
//					//                                printf("%f ",edgex[p][q]*edgex[p][q] + edgey[p][q]*edgey[p][q]);
//					//                            }
//					//                            printf("\n");
//					//                        }
//					//                        exit(0);
//					//                    }
//
//					//                    KOYO_LOG_INFO("match_template_one_degree ok.\n");
//					//                    tmp_candidate_result.angel_idx = k;
//					//                    tmp_candidate_result.score = score;
//					//
//					//                    if(__GMP_UNLIKELY(score > template_match_info->contour_parameter.bot_threshold)){
//					//                        //如果达标，加入结果集,怎么个加法？？
//					//                        //如果没满，直接加，如果满了替换分数小的添加
//					//                        KOYO_LOG_INFO("good match.\n");
//					//                        if(__GMP_LIKELY(*ncandidate_out < result_buffsize)){
//					//                            candidateResult_out[*ncandidate_out] = tmp_candidate_result;
//					//                            (*ncandidate_out)++;
//					//                        }else{
//					//                            //如果满了替换分数小的添加
//					//                            for (int l = 0; l < result_buffsize; ++l) {
//					//                                if(__GMP_LIKELY(score > candidateResult_out[l].score)){
//					//                                    candidateResult_out[l] = tmp_candidate_result;
//					//                                }
//					//                            }
//					//                        }
//					//                    }
//				}
//			}
//			//        KOYO_LOG_INFO("level %d ,%d line of %d all ok.\n",level,i,search_region_bin->u16Height-1);
//		}
//		stop(tt);
//		KOYO_LOG_INFO("duration %d. ", duration(tt));
//		KOYO_LOG_INFO("match completed max at x: %d, y: %d, ang %f, score %f.\n", maxj, maxi, maxang*template_match_info->search_angel_nstep[level], maxscore);
//		KOYO_LOG_INFO("ct_pass_filter1: %d, ct_pass_filter2: %d\n", ct_pass_filter1, ct_pass_filter2);
//
//	}
//	else{ // 非顶层
//		
//
//		maxscore = -1;
//		maxi = 0, maxj = 0;
//		maxang = 0;
//		/* 此处根据level决定膨胀范围和膨胀角度*/
//		UINT16 search_angel = SEARCH_NABOR_ANGLE_OTHER;
//		UINT16 search_size = SEARCH_NABOR_SIZE_OTHER;
//		//        UINT16 score_th = template_match_info->contour_parameter.bot_threshold;
//		if (level == 3)
//		{
//			search_angel = SEARCH_NABOR_ANGLE_THREE;
//			search_size = SEARCH_NABOR_SIZE_THREE;
//			//            score_th = score_th / 3;
//		}
//		else if (level == 2)
//		{
//			search_angel = SEARCH_NABOR_ANGLE_TWO;
//			search_size = SEARCH_NABOR_SIZE_THREE;
//			//            score_th = score_th / 3;
//		}
//		else if (level == 1)
//		{
//			search_angel = SEARCH_NABOR_ANGLE_ONE;
//			search_size = SEARCH_NABOR_SIZE_THREE;
//		}
//		//非顶层，根据顶层的传入来进行去重
//		KOYO_LOG_INFO("search_region_binary %d,%d %d candidate gives to me.\n", search_region_bin->u16Height, search_region_bin->u16Width, candidate_heap_in->n_scalar);
//		int range_step = 1;
//		int ang_step = 1;
//		if (search_size / 4) {
//			range_step = (1.0 * search_size / 4 + 0.5);
//			range_step = 1;
//		}
//
//		if (search_angel / 4) {
//			ang_step = (1.0 * search_angel / 4 + 0.5);
//			ang_step = 1;
//		}
//
//
//		//根据输入候选结果集构造搜索范围。注意，最多有100个候选结果，维护成链表以支持同一位置多角度
//		//这个结构体，第一列为角度区间中心，第二列为上一节点坐标，第三列为下一个区间的下标
//		/* 初始化位置角度计算重复空间 */
//		//        if(!set_empty(tool_utility->pset_coll))
//		//        set_clear(tool_utility->pset_coll);
//		hash_set_clear(tool_utility->phset_coll);
//		//        memset(tool_utility->iscompute, 0, sizeof(UINT8)*320*240*360);
//		//        stop(tt);
//		//        KOYO_LOG_INFO("iscompute time:%d.\n", duration(tt));
//		//        start(tt);
//		int width = tool_utility->u16Width >> level;
//		int height = tool_utility->u16Height >> level;
//		if (width < 64) width = 64;
//		if (height < 64) height = 64;
//		//        if(level != template_match_info->run_time_npyramid-2)
//		//        {
//		//            memset(tool_utility->iscompute, 0, sizeof(UINT8) * MAX_DEGREE * width/2 * height/2);  //每次对上层被污染的内存进行清空，其余内存还是0
//		//        }
//		////        for (INT16 wtemp = 0; wtemp < width; ++wtemp) {
//		////            for (INT16 htemp = 0; htemp < height; ++htemp) {
//		////                memset(tool_utility->iscompute[wtemp][htemp], 0, sizeof(UINT8) * MAX_DEGREE);
//		////            }
//		////        }
//		//
//		//        //需求，生成搜索区域，生成的时候要考虑位置NMS，角度NMS，角度要成区间，一个位置支持多个区间，位置要连成片
//		//        stop(tt);
//		//        KOYO_LOG_INFO("iscompute time:%d.\n", duration(tt));
//		//        start(tt);
//		static INT16 ang_region_idx_llist[CANDIDATE_RESULT_MAX_NUM + 1][ANG_REGION_LLIST_CUR_SIZE];
//		//存储当前候选点是否有效，1表示有效
//		INT16 candidate_idx_llist[CANDIDATE_RESULT_MAX_NUM];
//		memset(ang_region_idx_llist, 0, sizeof(INT16)*(CANDIDATE_RESULT_MAX_NUM + 1)*ANG_REGION_LLIST_CUR_SIZE);
//		memset(candidate_idx_llist, 0, sizeof(INT16)*(CANDIDATE_RESULT_MAX_NUM));
//		//这个结构体里保存:
//		//0无内容，在恢复的图里表示这个点无候选。其他值则保存
//		//        KOYO_LOG_INFO("size: %d\n", search_region_bin->u16Width*search_region_bin->u16Height);
//		UINT8 *center_p, *raw_start_p;
//		INT16 llist_idx;
//		//        KOYO_LOG_INFO("clear mask_ang_region_idx_u8.\n");
//		if (hi_ive_img_clear(&tool_utility->mask_ang_region_idx_u8[level]) != HI_SUCCESS){
//			KOYO_LOG_ERROR("error clean mask_ang_region_idx_u8.\n");
//		}
//		//        KOYO_LOG_INFO("clear mask_ang_region_idx_u8. ok\n");
//		CandidateResult *candidate_curr;
//		KOYO_LOG_INFO("candidate_heap_in->n_scalar %d\n", candidate_heap_in->n_scalar);
//
//		for (UINT8 k = 1; k <= candidate_heap_in->n_scalar; ++k) {
//			candidate_curr = candidate_heap_in->data[k - 1]; //k从1开始，而实际候选点从0开始，表示链表的下标从1开始，记录位置，防止记0忽略条件，实际位置在图像中和链表中已经表现，-1不影响取值
//			KOYO_LOG_TRACE("LINE: %d, now procese %d position: %d,%d,%f\n", __LINE__, k, candidate_curr->position.y, candidate_curr->position.x, candidate_curr->score);
//			//候选结果如果在图片外，直接略过
//
//			KOYO_LOG_TRACE("LINE: %d, pos.x: %d, pos.y: %d, degree: %lf, score: %lf\n", __LINE__, candidate_curr->position.x, candidate_curr->position.y, template_match_info->search_angel_nstep[level] * candidate_curr->angel_idx, candidate_curr->score);
//
//			if (candidate_curr->position.y < template_match_info->rect_roi[level].y
//				|| candidate_curr->position.x < template_match_info->rect_roi[level].x
//				|| candidate_curr->position.y >= template_match_info->rect_roi[level].y + template_match_info->rect_roi[level].height
//				|| candidate_curr->position.x >= template_match_info->rect_roi[level].x + template_match_info->rect_roi[level].width){
//				//                KOYO_LOG_INFO("ignored this.\n");
//				continue;
//			}
//
//			//确定考虑范围，在这个范围内，如果某个结果角度与本结果相近，则只保留分数大的
//			raw_start = candidate_curr->position.y - (INT16)search_size;
//			raw_end = candidate_curr->position.y + (INT16)search_size;
//			col_start = candidate_curr->position.x - (INT16)search_size;
//			col_end = candidate_curr->position.x + (INT16)search_size;
//
//			//确定检查区域，确定中心点
//			if (raw_start < 0) raw_start = 0;
//			if (raw_end >= search_region_bin->u16Height) raw_end = search_region_bin->u16Height - (INT16)1;
//			if (col_start < 0) col_start = 0;
//			if (col_end >= search_region_bin->u16Width) col_end = search_region_bin->u16Width - (INT16)1;
//
//
//			//候选结果左上角图像的行初始 原图的大小,初始全图为0， 如果该图上值为0表示目前不是候选点，后面会置位，值表示在heapin中的下标+1.为什么要+1，是因为零表示了不是候选点，要形成对应所以错一位
//			raw_start_p = &tool_utility->mask_ang_region_idx_u8[level].pu8VirAddr[0][raw_start *
//				tool_utility->mask_ang_region_idx_u8[level].u16Stride[0]];
//			//候选结果本身位置
//			center_p = &tool_utility->mask_ang_region_idx_u8[level].pu8VirAddr[0][candidate_curr->position.y*
//				tool_utility->mask_ang_region_idx_u8[level].u16Stride[0] + candidate_curr->position.x];
//			//检查，如果有值，判断，取舍，如果没值，直接加进结果集，有的话，如果该结果角度与本结果相近，则只保留分数大的
//			//在底层计算时是否可以跳跃计算
//			KOYO_LOG_TRACE("start search neighbor..\n");
//			for (int i = raw_start; i <= raw_end; i += range_step) {
//				for (int j = col_start; j <= col_end; j += range_step) { //check all neighbors
//					//candidate_heap_in->data 从0开始，而0表示空值
//					llist_idx = raw_start_p[j];
//					KOYO_LOG_TRACE("at %d,%d \n", i, j);
//					//                    if(!llist_idx){
//					//                        KOYO_LOG_INFO("nothing here just add in.\n");
//					//                    }else{
//					//                        KOYO_LOG_INFO("something here.\n");
//					//                    }
//					INT8 while_cnt = 1;
//					while (llist_idx){//表示在候选堆中的下标
//						KOYO_LOG_TRACE("list here now concern %dnd.\n", while_cnt++);
//						//有值，结果将在这里进行比对，如果这个结果
//						if ((candidate_curr->angel_idx + nangs - candidate_heap_in->data[llist_idx - 1]->angel_idx) % nangs
//			> search_angel){ //+nang 通过循环将可能的开始部分删除
//
//							KOYO_LOG_TRACE("candidate_curr->angel_idx: %d + nangs: %d - candidate_heap_in->data[llist_idx]->angel_idx: %d, search_angel: %d\n", candidate_curr->angel_idx, nangs, candidate_heap_in->data[llist_idx - 1]->angel_idx, search_angel);
//
//							KOYO_LOG_TRACE("one result at here ang too far, just concern next position.\n");
//							goto continue_next_position;//角度很远。考虑下一个位置,no erase。 //考虑分数要大于一定值         fill in
//						}
//						else{
//							KOYO_LOG_TRACE("one result at here ang not far, just leave one.\n");
//							KOYO_LOG_TRACE("candidate_curr->score: %lf,  candidate_heap_in->data[llist_idx]->score: %lf\n", candidate_curr->score, candidate_heap_in->data[llist_idx]->score);
//							if (candidate_curr->score < candidate_heap_in->data[llist_idx - 1]->score){
//								KOYO_LOG_TRACE("result is small break to concern next candidate.\n");
//								//比某个结果分数小，且角度距离很近，则忽略此结果。             not fill in
//								//提前退出
//								//候选点smaller than 周围点
//								goto continue_next_candidate;
//							}
//							else{ //all ops is delete current list node
//								KOYO_LOG_TRACE("result is big remove prev result ref, concern next position.\n");
//								//比某个结果分数大，且角度距离很近，将分数小的那个结果弄出链表。        fill in
//								if (ang_region_idx_llist[llist_idx][ANG_REGION_LLIST_PREV]){//是否为表头？不是表头，将结果弄出链表
//									KOYO_LOG_TRACE("prev candidate is not head.\n");
//									//上一节点的下一节点为当前节点的下一节点
//									ang_region_idx_llist[ang_region_idx_llist[llist_idx]
//										[ANG_REGION_LLIST_PREV]][ANG_REGION_LLIST_NEXT] =
//										ang_region_idx_llist[llist_idx][ANG_REGION_LLIST_NEXT];
//									//下一节点的上一节点为当前节点的上一节点
//									ang_region_idx_llist[ang_region_idx_llist[llist_idx]
//										[ANG_REGION_LLIST_NEXT]][ANG_REGION_LLIST_PREV] =
//										ang_region_idx_llist[llist_idx][ANG_REGION_LLIST_PREV];
//								}
//								else{//是表头，清出表头
//									KOYO_LOG_TRACE("prev candidate is head remove ok.\n");
//									ang_region_idx_llist[ang_region_idx_llist[llist_idx]
//										[ANG_REGION_LLIST_NEXT]][ANG_REGION_LLIST_PREV] = 0;  //head now is null. the second now is head
//									raw_start_p[j] = (UINT8)ang_region_idx_llist[llist_idx][ANG_REGION_LLIST_NEXT];    //重置二维图中坐标，有下一个点则替换为相应值，否则为0
//								}
//								ang_region_idx_llist[llist_idx][ANG_REGION_LLIST_PREV]
//									= ang_region_idx_llist[llist_idx][ANG_REGION_LLIST_NEXT] = 0; //delete current list index
//								candidate_idx_llist[llist_idx - 1] = 0; //k-1是与循环统一
//								CandidateResult *tmp = candidate_heap_in->data[llist_idx - 1];
//								KOYO_LOG_TRACE("+++++++++++++++++++++++++++++++++pos.x: %d, pos.y: %d, degree: %lf, score: %lf\n", tmp->position.x, tmp->position.y, template_match_info->search_angel_nstep[level] * tmp->angel_idx, tmp->score);
//							}
//
//						}
//					continue_next_position:
//						llist_idx = ang_region_idx_llist[llist_idx][ANG_REGION_LLIST_NEXT]; //相邻点的下一角度
//					}
//				}
//				raw_start_p += search_region_bin->u16Stride[0] * range_step;
//			}
//			//如果不加入链表，会直接跳到下面去。
//			//先保存结果到位置上
//			ang_region_idx_llist[k][ANG_REGION_LLIST_ANG] = candidate_curr->angel_idx; //now candidate score angel is enough
//			candidate_idx_llist[k - 1] = 1; //k-1是与循环统一
//			//再将位置加入链表！
//			if (!*center_p){
//				//没有值，直接加入即可，维护好链表
//				*center_p = k;      //当前候选点的第一个角度
//				ang_region_idx_llist[k][ANG_REGION_LLIST_PREV] = 0;
//				ang_region_idx_llist[k][ANG_REGION_LLIST_NEXT] = 0;
//				KOYO_LOG_TRACE("-----add to list-----pos.x: %d, pos.y: %d, degree: %lf, score: %lf\n", candidate_curr->position.x, candidate_curr->position.y, template_match_info->search_angel_nstep[level] * candidate_curr->angel_idx, candidate_curr->score);
//			}
//			else{
//				//遍历到链表尾，加入？nonono，直接在表头加入就好
//				ang_region_idx_llist[k][ANG_REGION_LLIST_PREV] = 0;
//				ang_region_idx_llist[k][ANG_REGION_LLIST_NEXT] = (*center_p);
//				ang_region_idx_llist[(*center_p)][ANG_REGION_LLIST_PREV] = k;  //当前区域其他角度放到表头后
//				(*center_p) = k;
//				KOYO_LOG_TRACE("-----add to list-----pos.x: %d, pos.y: %d, degree: %lf, score: %lf\n", candidate_curr->position.x, candidate_curr->position.y, template_match_info->search_angel_nstep[level] * candidate_curr->angel_idx, candidate_curr->score);
//
//			}
//		continue_next_candidate:;
//		}
//		//        board_tools_write_to_file_u8c1_formated("../image/region_binary",search_region_bin->u16Stride[0],search_region_bin->u16Height,search_region_bin->pu8VirAddr[0]);
//		//        board_tools_write_to_file_u8c1_formated("../image/region_rebuild",mask_ang_region_idx_u8.u16Stride[0],mask_ang_region_idx_u8.u16Height,mask_ang_region_idx_u8.pu8VirAddr[0]);
//		//        exit(0);
//		//        KOYO_LOG_INFO("rebuild.....\n");
//		raw_start_p = tool_utility->mask_ang_region_idx_u8[level].pu8VirAddr[0];
//		stride = tool_utility->mask_ang_region_idx_u8[level].u16Stride[0];
//		raw_start = 0;
//		raw_end = tool_utility->mask_ang_region_idx_u8[level].u16Height;
//		col_start = 0;
//		col_end = tool_utility->mask_ang_region_idx_u8[level].u16Width;
//		//        INT16 list_head_set[(SEARCH_NABOR_SIZE+1)*(SEARCH_NABOR_SIZE+1)];
//		INT16 list_head_set[(search_size * 2 - 1) * (search_size * 2 - 1)][2];      ///此处如果将范围扩大到与删除范围相同则找不到，为什么？？
//		INT8 list_head_set_idx = 0;
//		KOYO_LOG_INFO("raw:%d,col:%d.\n", raw_end, col_end);
//		//通过膨胀操作将候选区域完全重建，经考虑，可以
//		//        for (INT16 i = raw_start; i < raw_end; ++i) {
//		//            for (INT16 j = col_start; j < col_end; ++j) {
//
//		KOYO_LOG_TRACE("*****************before heap print***********************************\n");
//
//		for (int i = 0; i < candidate_heap_in->n_scalar; ++i) {
//			candidate_curr = candidate_heap_in->data[i];
//			if (candidate_idx_llist[i]) {
//				KOYO_LOG_TRACE("pos.x: %d, pos.y: %d, degree: %lf, score: %lf\n", candidate_curr->position.x, candidate_curr->position.y, template_match_info->search_angel_nstep[level] * candidate_curr->angel_idx, candidate_curr->score);
//			}
//		}
//
//		KOYO_LOG_TRACE("*****************before heap print(end)***********************************\n");
//		for (int can_num = 0; can_num < candidate_heap_in->n_scalar; ++can_num) {
//			if (!candidate_idx_llist[can_num]) {
//
//				KOYO_LOG_TRACE("x: %d, y: %d, ...not in candidate-------------------------------------------------------\n", candidate_heap_in->data[can_num]->position.x, candidate_heap_in->data[can_num]->position.y);
//				continue;
//			}
//			list_head_set_idx = 0;
//			/*
//			*   (-CAN_WIDTH+1,-CAN_HEIGHT+1) (-CAN_WIDTH+2,-CAN_HEIGHT+1) ... (CAN_WIDTH-1,-CAN_HEIGHT+1)
//			*   (-CAN_WIDTH+1,-CAN_HEIGHT+2)
//			*    ...
//			*   (-CAN_WIDTH+1,CAN_HEIGHT-1) (-CAN_WIDTH+2,CAN_HEIGHT-1) ... (CAN_WIDTH-1,CAN_HEIGHT-1)
//			* */
//			INT16 i = candidate_heap_in->data[can_num]->position.y; // i 是y
//			INT16 j = candidate_heap_in->data[can_num]->position.x; // j 是x
//			KOYO_LOG_TRACE("***********expansion**********:x: %d, y: %d\n", j, i);
//
//			// search_size是领域内的范围
//			for (INT16 tmp_h = -search_size + 1; tmp_h < search_size; ++tmp_h) {
//				for (INT16 tmp_w = -search_size + 1; tmp_w < search_size; ++tmp_w) {
//					if (i + tmp_h >= raw_start && i + tmp_h < raw_end && j + tmp_w >= col_start && j + tmp_w < col_end) {
//						//                            llist_idx = raw_start_p[stride * (tmp_h + i)  + j + tmp_w];
//						list_head_set[list_head_set_idx][0] = i + tmp_h; // 0是y
//						list_head_set[list_head_set_idx++][1] = j + tmp_w; // 1是x
//						//						KOYO_LOG_TRACE("------expansion--------:x: %d, y: %d\n", j + tmp_w, i + tmp_h);
//						//                            if (llist_idx)
//						//                                list_head_set[list_head_set_idx++] = llist_idx;
//					}
//				}
//			}
//			//                KOYO_LOG_INFO("%d %d.\n",i,j);
//
//			//   KOYO_LOG_INFO("%d.\n", nangs);
//			for (int n = 0; n < list_head_set_idx; ++n) {
//				//                llist_idx = list_head_set[n];
//				//                candidateResult.position.y = i;
//				//                candidateResult.position.x = j;
//				llist_idx = raw_start_p[j + stride * i];
//				INT16 cx = list_head_set[n][1];
//				INT16 cy = list_head_set[n][0];
//				candidateResult.position.y = cy;
//				candidateResult.position.x = cx;
//				//		KOYO_LOG_TRACE("------in heap each candi: cx: %d, cy: %d\n", cx, cy);
//				while (llist_idx) {
//					INT16 ang = ang_region_idx_llist[llist_idx][ANG_REGION_LLIST_ANG];
//					ang_start = ang - (INT16)search_angel;
//					ang_end = ang + (INT16)search_angel;
//					for (INT16 k = ang_start; k <= ang_end; k += ang_step) {
//						ang = (k + 2 * nangs) % nangs;
//						candidateResult.angel_idx = ang;
//						KOYO_LOG_TRACE("the ang is %d, ang_start is %d, ang_end is %d, ang_step is %d\n", ang, ang_start, ang_end, ang_step);
//
//						//							KOYO_LOG_TRACE("------in heap each candi: pos.x: %d, pos.y: %d, degree: %lf, score: %lf\n", candidateResult.position.x, candidateResult.position.y, template_match_info->search_angel_nstep[level] * candidateResult.angel_idx, candidateResult.score);
//
//						//                        if(tool_utility->iscompute[cx*width + cy*height + ang])
//						//                        {
//						////                            KOYO_LOG_INFO("here.\n");
//						//                            continue;
//						//                        }
//						//                        tool_utility->iscompute[cx*width + cy*height + ang] = 1;
//						int setNum = 0;
//						setNum |= (ang << 20);
//						setNum |= (cx << 10);
//						setNum |= cy;
//
//						//                        INT32 iscal = cx*cy + ang;
//						if (hash_set_count(tool_utility->phset_coll, setNum))
//						{
//							//                            KOYO_LOG_INFO("here.\n");
//							continue;
//						}
//						hash_set_insert(tool_utility->phset_coll, setNum);
//
//
//						INT8 s8_ret = match_template_one_degree(template_match_info, dilated_search_region_bin,
//							&candidateResult, tool_utility, rate);
//						KOYO_LOG_TRACE("******in heap each candi: pos.x: %d, pos.y: %d, degree: %lf, score: %lf\n", candidateResult.position.x, candidateResult.position.y, template_match_info->search_angel_nstep[level] * candidateResult.angel_idx, candidateResult.score);
//						if (s8_ret) {
//							continue;    // return 1: points is not enough
//						}
//						if (candidateResult.score > maxscore) {
//							maxscore = candidateResult.score;
//							maxi = cx;
//							maxj = cy;
//							maxang = k;
//						}
//						if (level == com_level) {            //***********************   0
//							if (candidateResult.score > candidate_heap_out->data[0]->score) {
//								*candidate_heap_out->data[0] = candidateResult;
//							}
//						}
//						else {
//							if (candidate_result_min_heap_if_need_add(candidate_heap_out, candidateResult.score)) {
//								candidate_result_min_heap_add_item(candidate_heap_out, &candidateResult);
//								//                      KOYO_LOG_INFO(
//								//                             "match completed max at level:%d coord:%d,%d, angel_index %d ang %f, score %f.\n", \
//								            //                        level, candidateResult.position.x, candidateResult.position.y,
//								//                           candidateResult.angel_idx, \
//								              //                      candidateResult.angel_idx * template_match_info->search_angel_nstep[level],
//								//                         candidateResult.score);
//							}
//						}
//					}
//					llist_idx = ang_region_idx_llist[llist_idx][ANG_REGION_LLIST_NEXT];
//				}
//			}
//		}
//		//            }
//		//            raw_start_p += stride;
//		//        }
//		KOYO_LOG_INFO("match completed max at x: %d, y: %d, ang %f, score %f.\n", maxi, maxj, maxang*template_match_info->search_angel_nstep[level], maxscore);
//		if (level == com_level){
//			KOYO_LOG_INFO("level: %d, result: pos y %d x %d ang %f, score: %f\n",
//				level,
//				candidate_heap_out->data[0]->position.y,
//				candidate_heap_out->data[0]->position.x,
//				candidate_heap_out->data[0]->angel_idx*template_match_info->search_angel_nstep[level],
//				candidate_heap_out->data[0]->score);
//
//			////            exit(0);
//		}
//		stop(tt);
//		KOYO_LOG_INFO("level %d cost %d\n", level, duration(tt));
//	}
//	//KOYO_LOG_INFO("*************************out*********************************\n");
//	for (UINT8 k = 1; k <= candidate_heap_out->n_scalar; ++k) {
//		CandidateResult *candidate_curr = candidate_heap_out->data[k - 1];
//		KOYO_LOG_TRACE("pos.x: %d, pos.y: %d, degree: %lf, score: %lf\n", candidate_curr->position.x, candidate_curr->position.y, template_match_info->search_angel_nstep[level] * candidate_curr->angel_idx, candidate_curr->score);
//	}
//	//    KOYO_LOG_INFO("tpl[4][30].noofcordinates %d;x %d;y %d.\n", template_match_info->tpls[4][30].noOfCordinates, \
//	    template_match_info->tpls[4][30].centerOfGravity.x, template_match_info->tpls[4][30].centerOfGravity.y);
//}


// 根据待测图长宽，对应到待测图的坐标，以及传递过来的模板的搜索框边长, 求出相交的矩形
std::vector<cv::Point> get_search_rect(int H, int W, cv::Point mapCenter, UINT16 top_search_rect_width) {
	int a = top_search_rect_width;
	int x1 = std::max(0, mapCenter.x - a);
	int x3 = std::min(W, mapCenter.x + a);
	int y1 = std::max(0, mapCenter.y - a);
	int y3 = std::min(H, mapCenter.y + a);

	std::vector<cv::Point> rectPoint = {
		{ x1, y1 },
		{ x1, y3 },
		{ x3, y3 },
		{ x3, y1 }
	};
	return rectPoint;
}


// 根据矩形，求出此时待测图中的轮廓点数
int get_num_of_rect(std::vector<cv::Point> rectPoint, std::vector<std::vector<bool>> &hasPoint) {
	int x1 = rectPoint[0].x;
	int x3 = rectPoint[2].x;
	int y1 = rectPoint[0].y;
	int y3 = rectPoint[2].y;

	int noOfPoint = 0;
	for (int i = x1; i < x3; ++i) {
		for (int j = y1; j < y3; ++j) {
			if (hasPoint[j][i]) {
				++noOfPoint;
			}
		}
	}
	return noOfPoint;
}

// 该点是否包括在该矩形区域
bool contain_point_in_rect(cv::Point &point, const std::vector<cv::Point> &rectPoint) {
	return (point.x >= rectPoint[0].x && point.x < rectPoint[2].x
		&& point.y >= rectPoint[0].y && point.y < rectPoint[2].y);
}

// 计算待测图中相对应的点的个数
int num_of_mapping_point(const std::vector<cv::Point> &rect_point, std::vector<cv::Point> &tpl_point,
	const std::vector<std::vector<bool>> &hasPoint, const cv::Point &mapCenter) {
	int noOfPoint = 0;
	for (auto point : tpl_point) {
		point.x += mapCenter.x;
		point.y += mapCenter.y;
		if (contain_point_in_rect(point, rect_point) && hasPoint[point.y][point.x]) {
			++noOfPoint;
		}
	}
	return noOfPoint;
}

// 进行其他层的匹配，从最顶层往下继续匹配
int do_other_layer_match(int cur_level, std::vector<TemplateStruct> &cur_tpls, float cur_angle_step,
	UINT16 cur_search_rect_width, ImportantInfo &upper_info, float threshold_S) {
	ImportantInfo cur_info;
	cur_info.center.x = 2 * upper_info.center.x;
	cur_info.center.y = 2 * upper_info.center.y;
	cur_info.angle = upper_info.angle;

	// 备选的质心坐标周围的点以及以当前角度左右几个该层角度步长的角度
	std::vector<cv::Point> point_bak;
	for (int i = cur_info.center.x - 4; i <= cur_info.center.x + 4; ++i) {
		for (int j = cur_info.center.y - 4; j <= cur_info.center.y + 4; ++j) {
			point_bak.push_back(cv::Point(i, j));
		}
	}
	int l = 0;
	for (double k = 0.0; k < cur_info.angle; k += cur_angle_step) {
		l++;
	}
	std::vector<int> angle_index;
	for (int i = -2; i <= 2; ++i) {
		angle_index.push_back((l + i) % cur_tpls.size());
	}

	// 进行相似度的计算
	SampleStruct &spl = sampleStructs[cur_level];

	double max_s = -1.0;
	for (auto centerPoint : point_bak) {
		std::vector<cv::Point> rectPoint = get_search_rect(spl.height, spl.width, centerPoint, cur_search_rect_width);
		for (auto l : angle_index) {
			TemplateStruct &tpl = cur_tpls[l];
			int K1 = tpl.cordinates.size();
			int num_T = 0;
			double sum_of_s = 0.0;
			int order = 0;
			for (auto point : tpl.cordinates) {
				point.x += centerPoint.x;
				point.y += centerPoint.y;
				if (contain_point_in_rect(point, rectPoint) && spl.hasPoint[point.y][point.x]) {
					++num_T;
					sum_of_s += fabs(tpl.edgeDerivativeX[order] * spl.edgeDerivativeX[point.y][point.x]
						+ tpl.edgeDerivativeY[order] * spl.edgeDerivativeY[point.y][point.x]);
				}
				++order;
				if (sum_of_s < (threshold_S - 1)*K1 + num_T) {
					break;
				}
			}
			double s = sum_of_s / num_T;

			// 匹配相似度最高的质心和角度记录下来
			if (s > max_s) {
				max_s = s;
				spl.angle = l * cur_angle_step;
				spl.center = centerPoint;
				std::cout << "now center is (" << spl.center.x << "," << spl.center.y
					<< "), and the angle is " << spl.angle
					<< "the similarity is " << s << std::endl;
			}
		}
	}
	return 0;
}


// 模板顶层对应的待测图金字塔的相应层数进行匹配
int ContourToolProcess::do_top_match(ContourUtility &contourUtility, float threshold_S) {
	// 准备必要的参数(0度模板时的轮廓点总数)
	int top_level = templateInfo.pyramidLevelNum - 1;
	std::vector<IVSTemplateSubStruct> &top_tpls = templateInfo.tpls[top_level];
	int K = top_tpls[0].noOfCordinates;
	float top_search_angel_nstep = templateInfo.searchAngelStep[top_level];
	int top_search_rect_width = templateInfo.searchRectWidth[top_level];

	//SampleStruct &spl = sampleStructs[level];
	//    std::cout << spl.height << " " << spl.width << std::endl;

	// 最大相似度
	double max_s = -1.0;
	for (int i = searchRegion[top_level].y; i < searchRegion[top_level].y + searchRegion[top_level].height; ++i) {
		for (int j = searchRegion[top_level].x; j < searchRegion[top_level].x + searchRegion[top_level].width; ++j) {
			// 对应到待测图的坐标
			cv::Point mapCenter;
			mapCenter.x = j;
			mapCenter.y = i;
			// 角度模板的序号置为0
			int l = 0;
			std::vector<cv::Point> rectPoint = get_search_rect(searchRegion[top_level].height, 
				searchRegion[top_level].width, mapCenter, top_search_rect_width);
			//            // 判断搜索框的运行情况
			//            cv::Mat rectPic = sample_pyramids[4].clone();
			//            cv::rectangle(rectPic,
			//                       rectPoint[0],
			//                       rectPoint[2],
			//                       cv::Scalar( 0, 0, 255 ),
			//                       1,
			//                       8 );

			int num = get_num_of_rect(rectPoint, spl.hasPoint);
			//            std::cout << "this rect has num is " << num << std::endl;
			//            cv::imshow("rectangle", rectPic);
			//            cv::waitKey(0);
			if (num > K * threshold_S) {
				//                std::cout << "OK the num of Point more than K*S and now center is ("
				//                          << j << "," << i << ")" << std::endl;

				for (double k = 0.0; k < MAX_DEGREE; k += top_search_angel_nstep) {
					TemplateStruct &tpl = top_tpls[l++];
					int K1 = tpl.cordinates.size();
					//                    if(num_of_mapping_point(rectPoint, tpl.cordinates, spl.hasPoint, mapCenter) > K1 * threshold_S){
					//                        std::cout << "OK the num of Point more than K*S and now center is ("
					//                                  << j << "," << i << "), and the angle is " << k << std::endl;
					// 相似度计算公式
					int num_T = 0;
					double sum_of_s = 0.0;
					int order = 0;
					for (auto point : tpl.cordinates) {
						point.x += mapCenter.x;
						point.y += mapCenter.y;
						if (contain_point_in_rect(point, rectPoint) && spl.hasPoint[point.y][point.x]) {
							++num_T;
							sum_of_s += fabs(tpl.edgeDerivativeX[order] * spl.edgeDerivativeX[point.y][point.x]
								+ tpl.edgeDerivativeY[order] * spl.edgeDerivativeY[point.y][point.x]);
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

					// 匹配相似度最高的质心和角度记录下来
					if (s > max_s) {
						max_s = s;
						spl.angle = k;
						spl.center.x = j;
						spl.center.y = i;
						std::cout << "now center is (" << j << "," << i << "), and the angle is " << k
							<< "the similarity is " << s << std::endl;
					}
					//                    }
				}
			}
		}
	}


	return 0;
}


//src 待测图像
INT8 ContourToolProcess::do_template_match_new(cv::Mat src, ContourUtility &contourUtility)
{

	INT8 ret;
	INT8 run_time_npyramid = templateInfo.pyramidLevelNum;

	// 由于高速度策略也检测到最底层，同时保证改动不大，只修改这三处，其二
	//INT8 com_level = p_template_matchInfo->contour_parameter.algo_strategy;   ///fixme:now is zero
	INT8 com_level = 0;
	//    KOYO_LOG_INFO("begin match.\n");

	// 顶层匹配分离出来


	// 其他层的匹配
	for (int l = run_time_npyramid - 2; l >= com_level; --l) {

		std::priority_queue<CandidateResult> nextLevelCandidates;

		// 非顶层的时候需要重新计算下一层的候选点
			
		// 优先队列从0开始
		for (int i = 0; i < candidates[run_time_npyramid - 2 - l].size(); ++i) {
			CandidateResult result = candidates[run_time_npyramid - 2 - l].top();
			result.positionX <<= 1;
			result.positionY <<= 1;
			result.angleIndex = (result.angleIndex * templateInfo.searchAngelStep[l + 1] / 
										templateInfo.searchAngelStep[l]) + 0.5;
			nextLevelCandidates.push(result);
		}
		

		printf("begin match level %d.\n", l);


		ret = match_template_one_level(&(tool_utility->search_region_binary[l]), &(tool_utility->dilated_search_region_binary[l]), p_template_matchInfo, result_in, result_out, tool_utility, l, pic_corr);

		if (l == com_level)
		{
			p_template_matchInfo->res_coor.y = result_out->data[0]->position.y;
			p_template_matchInfo->res_coor.x = result_out->data[0]->position.x;
			p_template_matchInfo->res_angel = result_out->data[0]->angel_idx;
			p_template_matchInfo->res_score = result_out->data[0]->score + 0.5;
		}
		if (ret < 0) {
			printf("Error at match_template_one_level %d.\n", l);
			return ret;
		}
		printf("level %d match %d targets.\n", l, result_out->n_scalar);
		tmp = result_in;
		result_in = result_out;
		result_out = tmp;
		printf("--------------------------------------------.\n");

	}
}