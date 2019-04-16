#include "ivs_contour_utility.h"


void CreateFloatMatrix(float ***matrix, int width, int height) {

	(*matrix) = (float **)malloc(height * sizeof(float *));
	if (!*matrix) {
		printf("ERROR: out of memory...\n");
	}
	int i, j;
	for (i = 0; i < height; ++i) {
		(*matrix)[i] = (float *)malloc(width * sizeof(float));
		if (!(*matrix)[i]) {
			printf("ERROR: out of memory, trying to malloc, i: %d, %d * %d...\n", i, width, sizeof(float));
		}
		for (j = 0; j < width; ++j) {
			(*matrix)[i][j] = -10;
		}
	}
}

// release memory
void ReleaseFloatMatrix(float ***matrix, int height) {
	//    for(int iInd = 0; iInd < size; iInd++)
	//        delete[] matrix[iInd];
	int i = 0;
	for (i = 0; i < height; ++i) {
		free((*matrix)[i]);
	}
	free(*matrix);
	*matrix = nullptr;
}

int createUtility(ContourUtility &contourUtility, IVSOriPic pic) {


	/* 分配各层金字塔图片梯度所需空间 */
	int s32Ret = 0;

	contourUtility.u16Height = pic.height;
	contourUtility.u16Width = pic.width;

	//ISize size;
	//size.width = LOOKUPTABLE_SIZE;
	//size.height = LOOKUPTABLE_SIZE;

	//CreateFloatMatrix(&(tool_utility->lookupTableX), size);
	//CreateFloatMatrix(&(tool_utility->lookupTableY), size);
	//CreateFloatMatrix(&(tool_utility->lookupTableS), size);

	//INT32 i, j;
	///* 初始化查找表 */
	//for (i = 0; i < LOOKUPTABLE_SIZE; i++) {
	//	for (j = 0; j < LOOKUPTABLE_SIZE; ++j) {
	//		float length;
	//		length = sqrtf(i * i + j * j);
	//		tool_utility->lookupTableS[i][j] = length;                  //强度查找表，用于计算二值化图
	//		tool_utility->lookupTableX[i][j] = 1.0f * (float)i / length;
	//		tool_utility->lookupTableY[i][j] = 1.0f * (float)j / length;
	//	}
	//}
	//tool_utility->lookupTableX[0][0] = 0;
	//tool_utility->lookupTableY[0][0] = 0;
	//tool_utility->lookupTableS[0][0] = 0;


	/* 各层金字塔运行时归一化梯度方向向量的存储表 */
	for (int i = 0; i < MAX_NUM_PYRAMID; i++) {
		int width = cvCeil((double)contourUtility.u16Width / (1 << i));
		int height = cvCeil((double)contourUtility.u16Height / (1 << i));    //金字塔建全局而不

		CreateFloatMatrix(&(contourUtility.edgeX[i]), width, height);	// create image to save gradient magnitude  values
		CreateFloatMatrix(&(contourUtility.edgeY[i]), width, height);	// create image to save gradient magnitude  values

	}
	return s32Ret;
}

//是否需要先将传入的src转换为U8C1
int computeUtility(ContourUtility &contourUtility, IVSOriPic ivsOriPic) {


	// 将待测图片转换为opencv中Mat数据结构
	// 目前直接读入数据，之后再处理转换的问题
	// 读入灰度图
	cv::Mat srcPic = cv::imread("D:\\QQPCmgr\\Desktop\\contourToolS\\contourToolS\\timg1.jpg", 0);

	contourUtility.u16Width = srcPic.cols;
	contourUtility.u16Height = srcPic.rows;

	int u16width = srcPic.cols;
	int u16height = srcPic.rows;

	// 滤波
	cv::GaussianBlur(srcPic, contourUtility.srcFiltered, cv::Size(5, 5), 3, 3);

	// 处理第0层的数据
	contourUtility.searchRegion[0] = contourUtility.srcFiltered.clone();
	cv::Sobel(contourUtility.srcFiltered, contourUtility.gradx[0], CV_16S, 1, 0);
	cv::Sobel(contourUtility.srcFiltered, contourUtility.grady[0], CV_16S, 0, 1);

	// 计算归一化的梯度edgeX,edgeY
	for (int i = 0; i < contourUtility.gradx[0].rows; i++) {

		for (int j = 0; j < contourUtility.gradx[0].cols; j++) {
			short fdx = contourUtility.gradx[0].at<short>(i, j);
			short fdy = contourUtility.grady[0].at<short>(i, j);

			double vector_length = sqrt(fdx * fdx + fdy * fdy);
			if (fdx != 0 || fdy != 0) {
				contourUtility.edgeX[0][i][j] = (float)fdx / vector_length;
				contourUtility.edgeY[0][i][j] = (float)fdy / vector_length;
			}
			else {
				contourUtility.edgeX[0][i][j] = 0.0f;
				contourUtility.edgeY[0][i][j] = 0.0f;
			}
		}
	}


	// 不用做Canny，因为每个轮廓工具都有不同的阈值


	// 创建降采样的图像金字塔
	for (int l = 0; l < MAX_NUM_PYRAMID - 1; ++l) {

		cv::pyrDown(contourUtility.searchRegion[l], contourUtility.searchRegion[l + 1]);
		cv::imshow("srcfilter", contourUtility.searchRegion[l + 1]);
		cv::waitKey(0);

		//计算sobel图片，并且归一化
		cv::Sobel(contourUtility.searchRegion[l + 1], contourUtility.gradx[l + 1], CV_16S, 1, 0);
		cv::Sobel(contourUtility.searchRegion[l + 1], contourUtility.grady[l + 1], CV_16S, 0, 1);

		// 计算归一化的梯度edgeX,edgeY
		for (int i = 0; i < contourUtility.gradx[l + 1].rows; i++) {

			for (int j = 0; j < contourUtility.gradx[l + 1].cols; j++) {
				short fdx = contourUtility.gradx[l + 1].at<short>(i, j);
				short fdy = contourUtility.grady[l + 1].at<short>(i, j);

				double vector_length = sqrt(fdx * fdx + fdy * fdy);
				if (fdx != 0 || fdy != 0) {
					contourUtility.edgeX[l + 1][i][j] = (float)fdx / vector_length;
					contourUtility.edgeY[l + 1][i][j] = (float)fdy / vector_length;
				}
				else {
					contourUtility.edgeX[l + 1][i][j] = 0.0f;
					contourUtility.edgeY[l + 1][i][j] = 0.0f;
				}
			}
		}

		//cv::Mat tmpdx, tmpdy;
		//convertScaleAbs(contourUtility.gradx[l + 1], tmpdx);
		//convertScaleAbs(contourUtility.grady[l + 1], tmpdy);
		//cv::imshow("dx", tmpdx);
		//cv::imshow("dy", tmpdy);
		//cv::waitKey(0);

	}

	return 0;
}

int freeUtility(ContourUtility & contourUtility) {


	//ISize size;
	//size.width = LOOKUPTABLE_SIZE;
	//size.height = LOOKUPTABLE_SIZE;
	////SAMPLE_PRT("here.\n");
	///* 释放查找表 */
	//ReleaseFloatMatrix(&(tool_utility->lookupTableX), size.height);
	//ReleaseFloatMatrix(&(tool_utility->lookupTableY), size.height);
	//ReleaseFloatMatrix(&(tool_utility->lookupTableS), size.height);

	////SAMPLE_PRT("here.\n");
	/* 释放edgeX */
	/* 释放edgeY */
	/* 释放is_calculated */
	for (INT32 i = 0; i < MAX_NUM_PYRAMID; i++) {
		int width = cvCeil((double)contourUtility.u16Width / (1 << i));
		int height = cvCeil((double)contourUtility.u16Height / (1 << i));


		ReleaseFloatMatrix(&(contourUtility.edgeX[i]), height);
		ReleaseFloatMatrix(&(contourUtility.edgeY[i]), height);
	}
	
	return 0;
}