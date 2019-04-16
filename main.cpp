#include <iostream>
#include "ivs_contour_utility.h"
#include "ivs_parameter.h"
#include "ivs_algorithm_utils.h"
#include "ivs_create_template.h"
#include "ContourToolProcess.h"

UINT8 erasureBitmap[1920 * 1080];
UINT8 resultBitmap[1024 * 1024 * 100];	//100M

										// 返回bitmap的填充大小
int fillErasureBitmap(cv::Mat src, IVSToolContourParameter &ivsToolContourParameter) {
	memset(erasureBitmap, 0, 1920 * 1080);
	// 这里使用clone()主要是因为matClean会改变src的数据
	cv::Mat matClean = src(cv::Rect(ivsToolContourParameter.extRectX, ivsToolContourParameter.extRectY,
		ivsToolContourParameter.extRectWidth, ivsToolContourParameter.extRectHeight)).clone();
	cv::Canny(matClean, matClean, ivsToolContourParameter.sensiLowThreshold, ivsToolContourParameter.sensiTopThreshold);
	//cv::imshow("matClean", matClean);
	//cv::waitKey(0);
	mat2Bitmap(matClean, erasureBitmap, ivsToolContourParameter.extRectWidth, ivsToolContourParameter.extRectHeight);
	return ivsToolContourParameter.extRectWidth*ivsToolContourParameter.extRectHeight;
}

void init_contour_parameter(cv::Mat src, IVSToolContourParameter &ivsToolContourParameter)
{
	ivsToolContourParameter.regionShape = 1;

	// 矩形检测框参数
	ivsToolContourParameter.detectRectX0 = 600;
	ivsToolContourParameter.detectRectY0 = 450;
	ivsToolContourParameter.detectRectX1 = 600;
	ivsToolContourParameter.detectRectY1 = 850;
	ivsToolContourParameter.detectRectX2 = 1000;
	ivsToolContourParameter.detectRectY2 = 850;
	ivsToolContourParameter.detectRectX3 = 1000;
	ivsToolContourParameter.detectRectY3 = 450;

	// 圆形检测框参数
	ivsToolContourParameter.detectCircleX = 0;
	ivsToolContourParameter.detectCircleY = 0;
	ivsToolContourParameter.detectCircleRadius = 0;

	// 外接矩形区域参数
	ivsToolContourParameter.extRectX = 600;
	ivsToolContourParameter.extRectY = 450;
	ivsToolContourParameter.extRectWidth = 400;
	ivsToolContourParameter.extRectHeight = 400;

	// 搜索区域参数
	ivsToolContourParameter.searchRectX0 = 0;
	ivsToolContourParameter.searchRectY0 = 0;
	ivsToolContourParameter.searchRectX1 = 0;
	ivsToolContourParameter.searchRectY1 = HEIGHT;
	ivsToolContourParameter.searchRectX2 = WIDTH;
	ivsToolContourParameter.searchRectY2 = HEIGHT;
	ivsToolContourParameter.searchRectX3 = WIDTH;
	ivsToolContourParameter.searchRectY3 = 0;

	// 算法策略相关
	ivsToolContourParameter.algoStrategy = 1;
	ivsToolContourParameter.angleRange = 45;
	ivsToolContourParameter.sensiLowThreshold = 50;
	ivsToolContourParameter.sensiTopThreshold = 150;

	// 算法处理及评分相关
	ivsToolContourParameter.scoreLowThreshold = 80;
	ivsToolContourParameter.scoreTopThreshold = 250;

	int erasureBitmapSize = fillErasureBitmap(src, ivsToolContourParameter);
	ivsToolContourParameter.erasureBitmapSize = erasureBitmapSize;
	ivsToolContourParameter.bitmaps = erasureBitmap;
	char templatePath[128] = "D:\\QQPCmgr\\Desktop\\contourToolS\\contourToolS\\timg.moban";
	memcpy(ivsToolContourParameter.templatePath, templatePath, 128);


}

int main() {
	std::string filename("D:\\QQPCmgr\\Desktop\\contourToolS\\contourToolS\\timg.jpg");
	std::cout << "ok" << std::endl;
	cv::Mat template_image;
	UINT8 *buf = nullptr;
	if (filename.substr(filename.size() - 3, 3) == "yuv") {
		FILE *yuv_file = fopen(filename.c_str(), "rb+");
		buf = new UINT8[WIDTH * HEIGHT];
		fread(buf, WIDTH * HEIGHT, 1, yuv_file);
	}
	else {
		template_image = cv::imread(filename, 0);
		buf = template_image.data;
	}

	IVSToolContourParameter ivsToolContourParameter;
	init_contour_parameter(template_image, ivsToolContourParameter);

	//int buf_size;
	//// 返回指向需要被发送的内存缓冲区的指针
	//ivs_create_template(buf, &ivsToolContourParameter, resultBitmap, 1024 * 1024 * 100, &buf_size);

	//// 创建待测图的图像金字塔
	//std::cout << std::endl << "**************待测图******************" << std::endl;
	////match_sample_template(buf, template_data);
	//FILE *fp = fopen((const char*)ivsToolContourParameter.templatePath, "wb");
	//if (!fp) {
	//	printf("file not found!\n");
	//	return -1;
	//}
	//fwrite(resultBitmap, sizeof(UINT8), buf_size, fp);
	//fclose(fp);


	ContourUtility contourUtility;

	IVSOriPic pic;
	pic.width = 1920;
	pic.height = 1080;
	// 创建轮廓工具的工具结构体
	createUtility(contourUtility, pic);

	// 对于每一帧都需要重新计算该工具结构体 
	computeUtility(contourUtility, pic);
	
	ContourToolProcess contourTool(ivsToolContourParameter);
	IVSContourResult ivsContourResult;
	// 事先赋值好id之类的
	contourTool.initTask(contourUtility);

	contourTool.doTask(contourUtility, ivsContourResult);


	// 释放轮廓工具的工具结构体
	freeUtility(contourUtility);

	return 0;
}