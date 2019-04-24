#include <iostream>
#include "ivs_contour_utility.h"
#include "ivs_parameter.h"
#include "ivs_algorithm_utils.h"
#include "ivs_create_template.h"
#include "ContourToolProcess.h"

UINT8 erasureBitmap[1920 * 1080];
UINT8 resultBitmap[1024 * 1024 * 100];	//100M

// ����bitmap������С
int fillErasureBitmap(cv::Mat src, IVSToolContourParameter &ivsToolContourParameter) {
	memset(erasureBitmap, 0, 1920 * 1080);
	// ����ʹ��clone()��Ҫ����ΪmatClean��ı�src������
	cv::Mat matClean = src(cv::Rect(ivsToolContourParameter.extRectX, ivsToolContourParameter.extRectY,
		ivsToolContourParameter.extRectWidth, ivsToolContourParameter.extRectHeight)).clone();
	cv::Canny(matClean, matClean, ivsToolContourParameter.sensiLowThreshold, ivsToolContourParameter.sensiTopThreshold);
	//cv::imshow("matClean", matClean);
	//cv::waitKey(0);
	mat2Bitmap(matClean, erasureBitmap, ivsToolContourParameter.extRectWidth, ivsToolContourParameter.extRectHeight);

	FILE *fp = fopen((const char*)ivsToolContourParameter.templatePath, "wb");
	if (!fp) {
		printf("file not found!\n");
		return -1;
	}
	fwrite(erasureBitmap, sizeof(UINT8), ivsToolContourParameter.extRectWidth*ivsToolContourParameter.extRectHeight, fp);
	fclose(fp);

	return ivsToolContourParameter.extRectWidth*ivsToolContourParameter.extRectHeight;
}

void init_contour_parameter(cv::Mat src, IVSToolContourParameter &ivsToolContourParameter)
{
	ivsToolContourParameter.regionShape = 1;

	// ���μ������
	ivsToolContourParameter.detectRectX0 = 600;
	ivsToolContourParameter.detectRectY0 = 450;
	ivsToolContourParameter.detectRectX1 = 600;
	ivsToolContourParameter.detectRectY1 = 850;
	ivsToolContourParameter.detectRectX2 = 1000;
	ivsToolContourParameter.detectRectY2 = 850;
	ivsToolContourParameter.detectRectX3 = 1000;
	ivsToolContourParameter.detectRectY3 = 450;

	// Բ�μ������
	ivsToolContourParameter.detectCircleX = 0;
	ivsToolContourParameter.detectCircleY = 0;
	ivsToolContourParameter.detectCircleRadius = 0;

	// ��Ӿ����������
	ivsToolContourParameter.extRectX = 600;
	ivsToolContourParameter.extRectY = 450;
	ivsToolContourParameter.extRectWidth = 400;
	ivsToolContourParameter.extRectHeight = 400;

	// �����������
	ivsToolContourParameter.searchRectX0 = 0;
	ivsToolContourParameter.searchRectY0 = 0;
	ivsToolContourParameter.searchRectX1 = 0;
	ivsToolContourParameter.searchRectY1 = HEIGHT;
	ivsToolContourParameter.searchRectX2 = WIDTH;
	ivsToolContourParameter.searchRectY2 = HEIGHT;
	ivsToolContourParameter.searchRectX3 = WIDTH;
	ivsToolContourParameter.searchRectY3 = 0;

	// �㷨�������
	ivsToolContourParameter.algoStrategy = 1;
	ivsToolContourParameter.angleRange = 45;
	ivsToolContourParameter.sensiLowThreshold = 50;
	ivsToolContourParameter.sensiTopThreshold = 150;

	// �㷨�����������
	ivsToolContourParameter.scoreLowThreshold = 80;
	ivsToolContourParameter.scoreTopThreshold = 250;
	char templatePath[128] = "C:\\Users\\donggeok\\Desktop\\19201080\\VI_contour10.moban";
	memcpy(ivsToolContourParameter.templatePath, templatePath, 128);

}

int main() {
	std::string filename("C:\\Users\\donggeok\\Desktop\\19201080\\VI_contour10.jpg");
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
	IVSTimer time1;
	time1.start();
	IVSToolContourParameter ivsToolContourParameter;
	init_contour_parameter(template_image, ivsToolContourParameter);

	//// ����ָ����Ҫ�����͵��ڴ滺������ָ��
	//fillErasureBitmap(template_image, ivsToolContourParameter);
	//ivs_create_template(buf, &ivsToolContourParameter);


	

	ContourUtility contourUtility;

	int picWidth = 1920;
	int picHeight = 1080;
	// �����������ߵĹ��߽ṹ��
	createUtility(contourUtility, picWidth, picHeight);

	IVSOriPic pic;
	pic.width = picWidth;
	pic.height = picHeight;
	// ����ÿһ֡����Ҫ���¼���ù��߽ṹ�� 
	IVSTimer time2;
	time2.start();
	computeUtility(contourUtility, pic);
	time2.end();
	std::cout << "computeUtility first��" << time2.getSpendTime() * 1000 << "ms" << std::endl;
	
	for (int i = 0; i < 10; ++i){
		time2.start();
		computeUtility(contourUtility, pic);
		time2.end();
		std::cout << "computeUtility second��" << time2.getSpendTime() * 1000 << "ms" << std::endl;
	}
	

	ContourToolProcess contourTool(ivsToolContourParameter);
	// ���ȸ�ֵ��id֮���
	contourTool.initTask(contourUtility);

	UINT8 *resultBuffer = (UINT8 *)malloc((sizeof(IVSContourResult)+1920 * 1080)*sizeof(UINT8));
	size_t resultBufferSize = 0;


	contourTool.doTask(contourUtility, resultBuffer, &resultBufferSize);
	time1.end();

	free(resultBuffer);

	std::cout << "��ʱ����" << time1.getSpendTime() * 1000 << "ms" << std::endl;

	// �ͷ��������ߵĹ��߽ṹ��
	freeUtility(contourUtility);

	return 0;
}