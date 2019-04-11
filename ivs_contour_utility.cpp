#include "ivs_contour_utility.h"


int createUtility(ContourUtility &contourUtility) {


	/* ������������ͼƬ�ݶ�����ռ� */
	int s32Ret = 0;

	//ISize size;
	//size.width = LOOKUPTABLE_SIZE;
	//size.height = LOOKUPTABLE_SIZE;

	//CreateFloatMatrix(&(tool_utility->lookupTableX), size);
	//CreateFloatMatrix(&(tool_utility->lookupTableY), size);
	//CreateFloatMatrix(&(tool_utility->lookupTableS), size);

	//INT32 i, j;
	///* ��ʼ�����ұ� */
	//for (i = 0; i < LOOKUPTABLE_SIZE; i++) {
	//	for (j = 0; j < LOOKUPTABLE_SIZE; ++j) {
	//		float length;
	//		length = sqrtf(i * i + j * j);
	//		tool_utility->lookupTableS[i][j] = length;                  //ǿ�Ȳ��ұ����ڼ����ֵ��ͼ
	//		tool_utility->lookupTableX[i][j] = 1.0f * (float)i / length;
	//		tool_utility->lookupTableY[i][j] = 1.0f * (float)j / length;
	//	}
	//}
	//tool_utility->lookupTableX[0][0] = 0;
	//tool_utility->lookupTableY[0][0] = 0;
	//tool_utility->lookupTableS[0][0] = 0;


	///* �������������ʱ��һ���ݶȷ��������Ĵ洢�� */
	//for (i = 0; i < MAX_NUM_PYRAMID; i++) {
	//	int width = u16width >> i;
	//	int height = u16height >> i;    //��������ȫ�ֶ�����ĳһ���ֵ�
	//	//        int width = pcontour_parameter->search_rect_width >> i;
	//	//        int height = pcontour_parameter->search_rect_height >> i;
	//	if (width < 64) width = 64;
	//	if (height < 64) height = 64;
	//	size.width = width;
	//	size.height = height;

	//	CreateFloatMatrix(&(tool_utility->edgeX[i]), size);	// create image to save gradient magnitude  values
	//	CreateFloatMatrix(&(tool_utility->edgeY[i]), size);	// create image to save gradient magnitude  values

	//	create_bool_matrix(&(tool_utility->is_calculated[i]), size);
	//	//        printf("build gradiant w %d, h %d. is_calculated[%d] %#p.\n",size.width,size.height,i,pstTemplateMatch->is_calculated[i]);

	//}

	///* �����ѡ������ͼ */
	//for (i = 0; i < MAX_NUM_PYRAMID - 1; i++) {
	//	int width = u16width >> i;
	//	int height = u16height >> i;
	//	if (width < 64) width = 64;
	//	if (height < 64) height = 64;
	//	SAMPLE_COMM_IVE_CreateImage(&(tool_utility->mask_ang_region_idx_u8[i]), IVE_IMAGE_TYPE_U8C1, width, height);
	//	if (s32Ret != HI_SUCCESS) {
	//		KOYO_LOG_ERROR("SAMPLE_COMM_IVE_CreateImage fail\n");
	//		return -1;
	//	}
	//}

	///* ����λ�ýǶȼ����ظ��ռ�,����ʹ��set���� */
	////    tool_utility->pset_coll = create_set(int);
	//tool_utility->phset_coll = create_hash_set(int);
	//if (tool_utility->phset_coll == NULL)
	//{
	//	KOYO_LOG_ERROR("set create fail\n");
	//	return -1;
	//}
	////    set_init(tool_utility->pset_coll);
	//hash_set_init(tool_utility->phset_coll);


	return s32Ret;
}

//�Ƿ���Ҫ�Ƚ������srcת��ΪU8C1
int computeUtility(ContourUtility &contourUtility, IVSOriPic ivsOriPic) {


	// ������ͼƬת��Ϊopencv��Mat���ݽṹ
	// Ŀǰֱ�Ӷ������ݣ�֮���ٴ���ת��������
	// ����Ҷ�ͼ
	cv::Mat srcPic = cv::imread("D:\\QQPCmgr\\Desktop\\contourToolS\\contourToolS\\timg.jpg", 0);

	contourUtility.u16Width = srcPic.cols;
	contourUtility.u16Height = srcPic.rows;

	int u16width = srcPic.cols;
	int u16height = srcPic.rows;

	// �˲�
	cv::GaussianBlur(srcPic, contourUtility.srcFiltered, cv::Size(5, 5), 3, 3);

	// �����0�������
	contourUtility.searchRegion[0] = contourUtility.srcFiltered.clone();
	cv::Sobel(contourUtility.srcFiltered, contourUtility.gradx[0], CV_16S, 1, 0);
	cv::Sobel(contourUtility.srcFiltered, contourUtility.grady[0], CV_16S, 0, 1);


	// ������Canny����Ϊÿ���������߶��в�ͬ����ֵ


	// ������������ͼ�������
	for (int l = 0; l < MAX_NUM_PYRAMID - 1; ++l) {

		cv::pyrDown(contourUtility.searchRegion[l], contourUtility.searchRegion[l + 1]);
		cv::imshow("srcfilter", contourUtility.searchRegion[l + 1]);

		//����sobelͼƬ�����ҹ�һ��
		cv::Sobel(contourUtility.searchRegion[l + 1], contourUtility.gradx[l + 1], CV_16S, 1, 0);
		cv::Sobel(contourUtility.searchRegion[l + 1], contourUtility.grady[l + 1], CV_16S, 0, 1);

		cv::Mat tmpdx, tmpdy;
		convertScaleAbs(contourUtility.gradx[l + 1], tmpdx);
		convertScaleAbs(contourUtility.grady[l + 1], tmpdy);
		cv::imshow("dx", tmpdx);
		cv::imshow("dy", tmpdy);
		cv::waitKey(0);

	}

	return 0;
}

int freeUtility(ContourUtility & contourUtility) {


	//ISize size;
	//size.width = LOOKUPTABLE_SIZE;
	//size.height = LOOKUPTABLE_SIZE;
	////SAMPLE_PRT("here.\n");
	///* �ͷŲ��ұ� */
	//ReleaseFloatMatrix(&(tool_utility->lookupTableX), size.height);
	//ReleaseFloatMatrix(&(tool_utility->lookupTableY), size.height);
	//ReleaseFloatMatrix(&(tool_utility->lookupTableS), size.height);

	////SAMPLE_PRT("here.\n");
	///* �ͷ�edgeX */
	///* �ͷ�edgeY */
	///* �ͷ�is_calculated */
	//for (INT32 i = 0; i < MAX_NUM_PYRAMID; i++) {
	//	int width = tool_utility->u16Width >> i;
	//	int height = tool_utility->u16Height >> i;
	//	if (width < 64) width = 64;
	//	if (height < 64) height = 64;
	//	size.width = width;
	//	size.height = height;
	//	ReleaseFloatMatrix(&(tool_utility->edgeX[i]), size.height);
	//	ReleaseFloatMatrix(&(tool_utility->edgeY[i]), size.height);
	//	release_bool_matrix(&(tool_utility->is_calculated[i]), size);
	//}
	////    int width = tool_utility->u16Width >> 1;
	////    int height = tool_utility->u16Height >> 1;
	////    for (INT16 wtemp = 0; wtemp < width; ++wtemp) {
	////        for (INT16 htemp = 0; htemp < height; ++htemp) {
	////            free(tool_utility->iscompute[wtemp][htemp]);
	////        }
	////        free(tool_utility->iscompute[wtemp]);
	////    }
	////    set_destroy(tool_utility->pset_coll);
	//hash_set_destroy(tool_utility->phset_coll);
	////    free(tool_utility->iscompute);
	return 0;
}