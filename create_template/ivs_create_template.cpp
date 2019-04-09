#include <iostream>
#include <queue>
#include <cstdio>
#include <fstream>
#include <basetsd.h>
#include <opencv.hpp>

#include "ivs_create_template.h"
#include "ivs_algorithm_utils.h"

int pack_template(const IVSTemplateStruct &ivsTemplateStruct, UINT8 *buf, int buf_free, int *bufsize)
{
	// 先计算所有要使用的内存大小，然后分配空间，最后一点点将数据拷贝过去
	std::size_t buf_size = 0;

	buf_size += sizeof(ivsTemplateStruct.pyramidLevelNum);					// 保存金字塔层数
	buf_size += sizeof(float)* ivsTemplateStruct.pyramidLevelNum;			// 保存金字塔每层的角度步长
	buf_size += sizeof(UINT16)* ivsTemplateStruct.pyramidLevelNum;			// 保存金字塔每层的搜索框框长
	// 要记录每层金字塔上的模板个数

	for (const auto &tpl_arr : ivsTemplateStruct.tpls) {
		// 每层金字塔上的模板个数
		buf_size += sizeof(UINT16);
		for (const auto &tpl : tpl_arr) {
			buf_size += sizeof (tpl.modelDefined);
			buf_size += sizeof (tpl.noOfCordinates);
			buf_size += sizeof (tpl.modelHeight);
			buf_size += sizeof (tpl.modelWidth);

			buf_size += sizeof(short)* 2;									// 保存重心坐标

			buf_size += sizeof(short)* 2 * tpl.noOfCordinates;
			buf_size += sizeof(float)* tpl.noOfCordinates;
			buf_size += sizeof(float)* tpl.noOfCordinates;
		}
	}

	if (buf_size > buf_free) {
		return -1;
	}

	std::size_t index = 0;
	memcpy(&buf[index], &ivsTemplateStruct.pyramidLevelNum, sizeof(ivsTemplateStruct.pyramidLevelNum));
	index += sizeof(ivsTemplateStruct.pyramidLevelNum);

	for (int i = 0; i < ivsTemplateStruct.pyramidLevelNum; ++i) {
		memcpy(&buf[index], &ivsTemplateStruct.searchAngelStep[i], sizeof(float));
		index += sizeof(float);
	}


	for (int i = 0; i < ivsTemplateStruct.pyramidLevelNum; ++i) {
		memcpy(&buf[index], &ivsTemplateStruct.searchRectWidth[i], sizeof(UINT16));
		index += sizeof(UINT16);
	}


	// 拷贝模板数据
	for (const auto &tpl_arr : ivsTemplateStruct.tpls) {
		UINT16 tpl_size = static_cast<UINT16>(tpl_arr.size()); //肯定不会超的
		memcpy(&buf[index], &tpl_size, sizeof(UINT16));
		index += sizeof(UINT16);

		for (const auto &tpl : tpl_arr) {
			memcpy(&buf[index], &tpl.modelDefined, sizeof(tpl.modelDefined));
			index += sizeof(tpl.modelDefined);

			memcpy(&buf[index], &tpl.noOfCordinates, sizeof(tpl.noOfCordinates));
			//    std::cout << "index: " << index << " coordinates this(buf): " << *((unsigned int*)(&buf[index])) << std::endl;
			index += sizeof(tpl.noOfCordinates);
			//   std::cout << "coordinates this: " << tpl.noOfCordinates << std::endl;

			memcpy(&buf[index], &tpl.modelHeight, sizeof(tpl.modelHeight));
			index += sizeof(tpl.modelHeight);

			memcpy(&buf[index], &tpl.modelWidth, sizeof(tpl.modelWidth));
			index += sizeof(tpl.modelWidth);

			memcpy(&buf[index], &tpl.centerOfGravity.x, sizeof(short));
			index += sizeof(short);

			memcpy(&buf[index], &tpl.centerOfGravity.y, sizeof(short));
			index += sizeof(short);

			for (auto const &coord : tpl.cordinates) {
				cv::Point point;
				point.x = static_cast<INT16>(coord.x);
				point.y = static_cast<INT16>(coord.y);
				memcpy(&buf[index], &point, sizeof(cv::Point));
				index += sizeof(cv::Point);

			}
			for (auto const & edgeX : tpl.edgeDerivativeX) {
				memcpy(&buf[index], &edgeX, sizeof(float));
				index += sizeof(float);
			}

			for (auto const & edgeY : tpl.edgeDerivativeY) {
				memcpy(&buf[index], &edgeY, sizeof(float));
				index += sizeof(float);
			}
		}
	}

	std::cout << buf_size << ", in MB: " << 1.0 * buf_size / 1024 / 1024 << "MB" << std::endl;
	std::cout << "address of " << bufsize << std::endl;
	*bufsize = buf_size;
	std::cout << "after pack" << std::endl;
	std::cout << *bufsize << ", in MB: " << 1.0 * *bufsize / 1024 / 1024 << "MB" << std::endl;

	return 0;
}


static int unpack_template(IVSTemplateStruct &ivsTemplateStruct, char* buf)
{

	std::size_t index = 0;

	ivsTemplateStruct.pyramidLevelNum = *((UINT8 *)&buf[index]);
	index += sizeof(ivsTemplateStruct.pyramidLevelNum);

	for (int i = 0; i < ivsTemplateStruct.pyramidLevelNum; ++i) {
		float angle = *((float*)&buf[index]);
		index += sizeof(float);
		ivsTemplateStruct.searchAngelStep.push_back(angle);
	}

	for (int i = 0; i < ivsTemplateStruct.pyramidLevelNum; ++i) {
		UINT16 width = *((UINT16*)&buf[index]);
		index += sizeof(UINT16);
		ivsTemplateStruct.searchRectWidth.push_back(width);
	}

	// 拷贝模板数据
	for (int i = 0; i < ivsTemplateStruct.pyramidLevelNum; ++i) {
		std::vector<IVSTemplateSubStruct> tpl_arr;

		UINT16 tpl_size = *((UINT16*)&buf[index]);
		index += sizeof(UINT16);

		//        std::cout << "tpl_size: " << tpl_size << std::endl;
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

			tpl.centerOfGravity.x = *((UINT16*)&buf[index]);
			index += sizeof(short);

			tpl.centerOfGravity.y = *((UINT16*)&buf[index]);
			index += sizeof(short);

			// 拷贝特征数据
			for (std::size_t i = 0; i < tpl.noOfCordinates; ++i) {
				cv::Point point;
				point = *((cv::Point*)&buf[index]);
				index += sizeof(cv::Point);
				tpl.cordinates.push_back(point);
			}

			for (std::size_t i = 0; i < tpl.noOfCordinates; ++i) {
				float edgeX = *((float *)&buf[index]);
				index += sizeof(float);
				tpl.edgeDerivativeX.push_back(edgeX);
			}

			for (std::size_t i = 0; i < tpl.noOfCordinates; ++i) {
				float edgeY = *((float*)&buf[index]);
				index += sizeof(float);
				tpl.edgeDerivativeY.push_back(edgeY);
			}
			tpl_arr.push_back(tpl);
		}
		ivsTemplateStruct.tpls.push_back(tpl_arr);
	}

	return 0;
}


void print_tpl(const IVSTemplateSubStruct &tpl) {
	cv::Mat tmp(tpl.modelHeight, tpl.modelWidth, CV_8UC1, cv::Scalar(0));
	for (int i = 0; i < tpl.noOfCordinates; ++i){
		tmp.at<uchar>(tpl.cordinates[i].y + tpl.centerOfGravity.y, tpl.cordinates[i].x + tpl.centerOfGravity.x) = 255;
	}
	cv::imshow("tmp", tmp);
	cv::waitKey(0);
}

static std::ostream &print_tpls(std::ostream &os, const IVSTemplateStruct & rhl)
{
	os << "runtime npyramid: " << static_cast<int>(rhl.pyramidLevelNum) << std::endl;
	os << "angle steps each level: " << std::endl;
	int k = 0;
	for (auto angle : rhl.searchAngelStep) {
		os << "level " << k++ << ": " << angle << std::endl;
	}

	k = 0;
	os << "templates each level: " << std::endl;
	for (auto tpl : rhl.tpls) {
		os << "level " << k++ << ": " << tpl.size() << ", Num of coordinate this level: " << tpl[0].noOfCordinates << std::endl;
	}
	return os;
}


static std::vector<float> rotate_image(const cv::Mat &src, cv::Mat &dst, cv::Point centerP, float degree)
{
	int width = src.cols;
	int height = src.rows;
	double angle = degree  * CV_PI / 180.; // 弧度
	double a = sin(angle), b = cos(angle);
	// 适当增大一点宽高，防止像素不在图像内
	int width_rotate = int(height * fabs(a) + width * fabs(b));
	int height_rotate = int(width * fabs(a) + height * fabs(b));
	float map[6];
	cv::Mat map_matrix = cv::Mat(2, 3, CV_32F, map);
	//    CvPoint2D32f center = cvPoint2D32f(centerP.x, centerP.y);
	// 旋转中心, 以原始图片中心作为旋转中心而不是质心
	CvPoint2D32f center = cvPoint2D32f(width / 2.0, height / 2.0);
	CvMat map_matrix2 = map_matrix;
	cv2DRotationMatrix(center, degree, 1.0, &map_matrix2);
	// 这里不能改
	map[2] += (width_rotate - width) / 2.0;
	map[5] += (height_rotate - height) / 2.0;
	cv::warpAffine(src, dst, map_matrix, cv::Size(width_rotate, height_rotate), CV_INTER_NN);
	std::vector<float> rotate_matrix;
	rotate_matrix.push_back(map[0]);
	rotate_matrix.push_back(map[1]);
	rotate_matrix.push_back(map[2]);
	rotate_matrix.push_back(map[3]);
	rotate_matrix.push_back(map[4]);
	rotate_matrix.push_back(map[5]);
	return rotate_matrix;
}

static int rotate_rect(std::vector<cv::Point> &rect, const std::vector<float> rotate_matrix)
{
	int plx = rect[0].x, ply = rect[0].y, prx = rect[3].x, pry = rect[3].y;
	int plxb = rect[1].x, plyb = rect[1].y, prxb = rect[2].x, pryb = rect[2].y;

	rect[0].x = rotate_matrix[0] * plx + rotate_matrix[1] * ply + rotate_matrix[2];
	rect[0].y = rotate_matrix[3] * plx + rotate_matrix[4] * ply + rotate_matrix[5];

	rect[1].x = rotate_matrix[0] * plxb + rotate_matrix[1] * plyb + rotate_matrix[2];
	rect[1].y = rotate_matrix[3] * plxb + rotate_matrix[4] * plyb + rotate_matrix[5];

	rect[2].x = rotate_matrix[0] * prxb + rotate_matrix[1] * pryb + rotate_matrix[2];
	rect[2].y = rotate_matrix[3] * prxb + rotate_matrix[4] * pryb + rotate_matrix[5];

	rect[3].x = rotate_matrix[0] * prx + rotate_matrix[1] * pry + rotate_matrix[2];
	rect[3].y = rotate_matrix[3] * prx + rotate_matrix[4] * pry + rotate_matrix[5];
	return 0;
}

static unsigned short calAlign(unsigned short len, unsigned char align)
{
	return (len + (align - len % align) % align);
}

static int cutout_template_image(const cv::Mat &template_image, std::vector<cv::Point> rect, cv::Mat &interesting_template)
{
	auto degree = MAX_DEGREE - cv::fastAtan2(rect[1].y - rect[2].y, rect[2].x - rect[1].x);
	cv::Mat img_rotate;
	int width = template_image.cols;
	int height = template_image.rows;
	std::vector<float> rotate_matrix;
	rotate_matrix = rotate_image(template_image, img_rotate, cv::Point(width / 2, height / 2), degree);
	std::cout << degree << std::endl;
	rotate_rect(rect, rotate_matrix);

#ifndef NDEBUG
	cv::circle(img_rotate, rect[0], 1, cv::Scalar(255, 255, 255));
	cv::circle(img_rotate, rect[1], 2, cv::Scalar(255, 255, 255));
	cv::circle(img_rotate, rect[2], 3, cv::Scalar(255, 255, 255));
	cv::circle(img_rotate, rect[3], 4, cv::Scalar(255, 255, 255));
#endif

	// 这里得保证一样的, 保证对齐吧
	int rect_width = calAlign(rect[3].x - rect[0].x, 16);
	int rect_height = calAlign(rect[1].y - rect[0].y, 16);
	// 保证不会超出原图大小
	// todo 隐藏的问题是16可能减得太多了
	// fixme 为什么要减掉？
	if (rect_width > rect[3].x - rect[0].x) rect_width -= 16;
	if (rect_height> rect[1].y - rect[0].y) rect_height -= 16;

	// 要保证这里的截图不出问题才行。。
	try {
		interesting_template = img_rotate(cv::Rect(rect[0].x, rect[0].y, rect_width, rect_height));
	}
	catch (std::exception e){
		std::cout << "ERROR while cut image: " << " " << rect[0].x << " " << rect[0].y << " " << rect_width << " " << rect_height << e.what() << std::endl;
		exit(-1);
	}


	return 0;
}

/*
*  输入图像是二值化图像
* */
static int get_center_numof_contour(const cv::Mat src, cv::Point &center, unsigned int &numofcontour, int height, int width)
{
	double m00, m10, m01;
	auto moments = cv::moments(src, true);
	m10 = moments.m10;
	m01 = moments.m01;
	m00 = moments.m00;
	if (m00 == 0) {
		return -1;
	}
	else {
		// 求旋转角度步长的中心应该选为模板的中心，而不是质心
		center.x = width / 2;
		center.y = height / 2;
		//center.x = static_cast<int>(m10/m00);
		//center.y = static_cast<int>(m01/m00);
	}
	numofcontour = m00;
	return 0;
}


void Dilation(const cv::Mat &src, cv::Mat &dilation_dst, int size)
{
	int dilation_type = cv::MORPH_RECT;

	cv::Mat element = cv::getStructuringElement(dilation_type,
		cv::Size(size, size));
	///膨胀操作
	dilate(src, dilation_dst, element);
}

void Erode(const cv::Mat &src, cv::Mat &erode_dst, int size)
{
	int dilation_type = cv::MORPH_RECT;

	cv::Mat element = cv::getStructuringElement(dilation_type,
		cv::Size(size, size));
	///腐蚀操作
	erode(src, erode_dst, element);
}

// 为了保证最顶层的点数足够多，将二层图像先膨胀后再向上求最顶层的二值图
void get_topLevel_binaryPic_by_dilationLevel(const IVSTemplateSubStruct &tpl, cv::Mat &ret_mat, int height, int width) {
	cv::Mat tmp(tpl.modelHeight, tpl.modelWidth, CV_8UC1, cv::Scalar(0));
	cv::Mat tmp_sameSize(height, width, CV_8UC1, cv::Scalar(0));
	for (int i = 0; i < tpl.noOfCordinates; ++i) {
		tmp.at<uchar>(tpl.cordinates[i].y + tpl.centerOfGravity.y, tpl.cordinates[i].x + tpl.centerOfGravity.x) = 255;
	}
	// 膨胀
	cv::Mat tmp_dilation, tmp_notSize;
	Dilation(tmp, tmp_dilation, 2);

	// downSample 只取偶数行列
	for (int i = 0; i < tmp_sameSize.rows; ++i) {
		for (int j = 0; j < tmp_sameSize.cols; ++j) {
			if (2 * i < tmp_dilation.rows && 2 * j < tmp_dilation.cols && tmp_dilation.at<uchar>(2 * i, 2 * j) == 255) {
				tmp_sameSize.at<uchar>(i, j) = 255;
			}
		}
	}

	ret_mat = tmp_sameSize.clone();
}

/*
* rect是相对640*480图片的坐标
* */
static int do_create_template(IVSTemplateSubStruct &tpl, const cv::Mat &src, const cv::Mat &bitmap, bool do_bitwise_and, double low_threshold, \
	double high_threshold, const cv::Mat &rmWhite, int level, int max_level, const IVSTemplateSubStruct &pre_tpl)
{
	int s32Ret = 0;
	cv::Mat gx;                //Matrix to store X derivative
	cv::Mat gy;                //Matrix to store Y derivative
	// set width and height
	tpl.modelHeight = static_cast<UINT16>(src.rows);    //Save Template height
	tpl.modelWidth = static_cast<UINT16>(src.cols);    //Save Template width

	tpl.noOfCordinates = 0;    //initialize

	cv::Sobel(src, gx, CV_16S, 1, 0, 3);        //gradient in X direction
	cv::Sobel(src, gy, CV_16S, 0, 1, 3);        //gradient in Y direction

	//    cv::Mat binaryContour;
	//    cv::Canny(src, binaryContour, low_threshold, high_threshold);

	cv::Mat binaryContour, before_filter;

	// 在这里进行最顶层的膨胀操作（为了保留更多的点，所以二层先做膨胀操作，之后再向上建立二值化的顶层）
	if (level == max_level - 1) { // 顶层
		get_topLevel_binaryPic_by_dilationLevel(pre_tpl, before_filter, src.rows, src.cols);
	}
	else {
		cv::Canny(src, before_filter, low_threshold, high_threshold);
	}

	//canny的结果和bitmap相与
	//cv::imshow("before", before_filter);
	//cv::waitKey(0);
	// 把bitmap膨胀一下
	cv::Mat dialBitmap;
	Dilation(bitmap, dialBitmap, 3);
	cv::threshold(dialBitmap, bitmap, 10, 255, CV_THRESH_BINARY);

	//    std::cout << before_filter.cols << " " << before_filter.rows << std::endl;
	//    std::cout << bitmap.cols << " " << bitmap.rows << std::endl;
	//cv::imshow("bitmat", bitmap);
	//cv::waitKey(0);
	if (do_bitwise_and) {
		cv::bitwise_and(before_filter, rmWhite, before_filter);
		cv::bitwise_and(before_filter, bitmap, binaryContour);
	}
	else {
		if (level != max_level - 1) { // 顶层
			cv::bitwise_and(before_filter, rmWhite, before_filter);
		}
		binaryContour = before_filter;
	}
	//cv::imshow("binary", binaryContour);
	//cv::waitKey(0);

	int RSum = 0, CSum = 0;

	for (int i = 0; i < tpl.modelHeight; i++) {
		for (int j = 0; j < tpl.modelWidth; j++) {
			short fdx = gx.at<short>(i, j);
			short fdy = gy.at<short>(i, j);
			unsigned char U8 = binaryContour.at<uchar>(i, j);
			// 考虑到如果擦除点数太多，导致匹配过程中一级匹配出现问题，因此点数应该统计为未擦除前的
			//unsigned char U8_before = before_filter.at<uchar>(i, j);
			cv::Point p;
			p.x = j;
			p.y = i;
			// todo 这里由于使用了位与操作，所以不用再判断距离了
			// 最小距离是多少还需要斟酌，因为在最小分辨率情况下看到边框还是没有去除掉，在最小分辨情况下这个dist太小了。
			double min_dist = MIN_DIST;
			// todo 这里由于使用了位与操作，所以不用再判断距离了
			if (U8) {
				/* 如果梯度都为零，那么不需要计算，因为分数不会有贡献 */
				if (fdx != 0 || fdy != 0) {
					/* 坐标变换到外接矩形左上角为(0, 0) */
					RSum = RSum + j;
					CSum = CSum + i;    // Row sum and column sum for center of gravity
					tpl.cordinates.push_back(p);
					//                    tpl.cordinates[tpl.noOfCordinates].x = j;
					//                    tpl.cordinates[tpl.noOfCordinates].y = i;

					/* TODO 可以修改成使用查找表的形式 */
					double vector_length = sqrt(fdx * fdx + fdy * fdy);
					if (fabs(vector_length - 0.) < 0.00001) {
						//                        printf(".............................................\n");
					}
					tpl.edgeDerivativeX.push_back(static_cast<float>(fdx / vector_length));
					tpl.edgeDerivativeY.push_back(static_cast<float>(fdy / vector_length));
					tpl.noOfCordinates++;
				}
			}
			//if (U8_before) {
			//	if (fdx != 0 || fdy != 0) {
			//		tpl.noOfCordinates++;
			//	}
			//}
		}
	}

	if (tpl.noOfCordinates == 0) {
		//        printf(".........................");
		tpl.centerOfGravity.x = tpl.modelWidth / 2;
		tpl.centerOfGravity.y = tpl.modelHeight / 2;
	}
	else {
		tpl.centerOfGravity.x = tpl.modelWidth / 2;
		tpl.centerOfGravity.y = tpl.modelHeight / 2;
		//tpl.centerOfGravity.x = RSum / tpl.noOfCordinates;    // center of gravity
		//tpl.centerOfGravity.y = CSum / tpl.noOfCordinates;    // center of gravity
	}

	// change coordinates to reflect center of gravity
	/* 将重心变换到坐标原点 */
	UINT32 m;
	for (m = 0; m < tpl.noOfCordinates; m++) {
		/*int temp;

		temp = tpl.cordinates[m].x;
		tpl.cordinates[m].x = temp - tpl.centerOfGravity.x;
		temp = tpl.cordinates[m].y;
		tpl.cordinates[m].y = temp - tpl.centerOfGravity.y;*/
		tpl.cordinates[m].x -= tpl.centerOfGravity.x;
		tpl.cordinates[m].y -= tpl.centerOfGravity.y;
	}

	tpl.modelDefined = true;

	return 1;
}

static void draw_template(cv::Mat src, const IVSTemplateSubStruct &tpl)
{
	for (UINT32 i = 0; i < tpl.noOfCordinates; ++i) {
		cv::circle(src, cv::Point(tpl.cordinates[i].x + tpl.centerOfGravity.x, tpl.cordinates[i].y + tpl.centerOfGravity.y), 1, cv::Scalar(255, 255, 255));
	}
	cv::imshow("hehe", src);
	cvWaitKey(0);
}

// TODO 发送给客户端以后需要释放
//int free_tpls(TemplateStruct tpls[][MAX_DEGREE])
//{
//    return 0;
//}

/*
*  @param src必须是二值化轮廓图
*  @return 返回当前层上最佳的旋转步长
* */
static float get_angle_step_and_search_width(const cv::Mat &src, cv::Point center, std::vector<UINT16> &search_rect_width)
{
	// 保留几个K，然后求平均值，用来排除外点的影响
	int K = 10;
	std::priority_queue<float> max_dist(K, -1);
	std::vector<cv::Point> points;
	//    double max_distance = -1;
	// 在目前没有旋转的图片中不会出现因为旋转导致的白边，所以直接在全图搜索就行了，不用考虑别的
	for (int i = 0; i < src.rows; ++i) {
		for (int j = 0; j < src.cols; ++j) {
			if (src.at<uchar>(i, j)) {
				double dist = -1 * sqrt(pow((i - center.y), 2) + pow((j - center.x), 2));
				//                if(-dist > max_distance) max_distance = -dist;
				if (dist < max_dist.top()) {
					max_dist.pop();
					max_dist.push(dist);
					points.push_back({ i, j });
				}
			}
		}
	}
	// 搜索框是最远点的位置，这样的话搜索框就不会超出了
	//    search_rect_width.push_back(static_cast<UINT16>(1 + max_distance));
	//    std::cout << "top: " << max_distance << " " << -max_dist.top() << std::endl;
	//    std::cout << "max dist....." << std::endl;
	float average_max_dist = 0;
	int i = 0;
	while (!max_dist.empty()) {
		average_max_dist += -max_dist.top();
		max_dist.pop();
	}
	average_max_dist /= K;
	//    std::cout << "average_max_dist: " << average_max_dist << std::endl;

	search_rect_width.push_back(static_cast<UINT16>(average_max_dist));
	auto range_low = acos(1 - 1 / (2 * average_max_dist * average_max_dist)) / CV_PI * 180;
	auto range_high = acos(1 - 2 / (average_max_dist * average_max_dist)) / CV_PI * 180;
	//    std::cout <<"optimal angle step: " << range_low << " ~ " << range_high << std::endl;

#ifndef NDEBUG
	cv::Mat tmp = src;
	for (unsigned int k = points.size() - K; k < points.size(); ++k) {
		cv::circle(tmp, points[k], 20, cv::Scalar(255, 255, 255));
	}
	//    cv::imshow("max dist", tmp);
	//    cvWaitKey(0);
	std::cout << range_low << " " << range_high << std::endl;
#endif
	float result = std::max((range_low + range_high) / 2, 1.0);
	//    return result;
	return result < 6.0 ? result : 6.0;
}


/*
* src是截取出来的模板图片
* bitmapCleaned是位图，大小和src一致
*/
static int do_create_template(const cv::Mat &src, const cv::Mat &bitMap, IVSToolContourParameter* koyo_tool_contour_parameter, IVSTemplateStruct &koyo_contour_template_runtime_param)
{
	// 运行完成后需要将这个内容发送给嵌入式

	//  TimeTracker tt1;
	//   tt1.start();
	std::vector<cv::Mat> pyramid_bitmaps;
	std::vector<cv::Mat> pyramid_templates;
	pyramid_templates.push_back(src);
	pyramid_bitmaps.push_back(bitMap);

	//cv::imshow("bitMap", bitMap);
	//cv::waitKey(0);

	UINT8 sensitity_threshold_low, sensitity_threshold_high;

	sensitity_threshold_low = koyo_tool_contour_parameter->sensiLowThreshold;
	sensitity_threshold_high = koyo_tool_contour_parameter->sensiLowThreshold * 3;
	printf("the koyo_tool_contour_parameter->sensitivity_Low_Threshold is %d, the koyo_tool_contour_parameter->sensitivity_Low_Threshold*3 is %d", sensitity_threshold_low, sensitity_threshold_high);

	std::cout << "create_template begin" << std::endl;

	// 建立各层金字塔, 并确定最佳金字塔层数
	int optimal_pyr_level = 0;

	for (int i = 0; i < MAX_NUM_PYRAMID - 1; ++i) {
		cv::Mat next_level;
		cv::Mat next_level_bmap;
		//        cv::GaussianBlur(pyramid_templates[i], pyramid_templates[i], cv::Size(5,5),3);
		//        cv::GaussianBlur(pyramid_bitmaps[i], pyramid_bitmaps[i], cv::Size(5,5),3);
		cv::pyrDown(pyramid_bitmaps[i], next_level_bmap);
		cv::pyrDown(pyramid_templates[i], next_level);
		pyramid_templates.push_back(next_level);
		pyramid_bitmaps.push_back(next_level_bmap);
		//cv::imshow("next_level", next_level);
		//cv::waitKey(0);
		//cv::imshow("next_level_bmap", next_level_bmap);
		//cv::waitKey(0);
	}

#if 0
	// 做过高斯滤波就不做滤波
	for (auto iter = std::begin(pyramid_templates); iter != std::end(pyramid_templates); ++iter) {
		cv::Mat after_gaus;
		cv::GaussianBlur(*iter, after_gaus, cv::Size(5, 5), 5, 5);
		*iter = after_gaus;
	}
#endif
	// 只对最底层进行滤波
	//    cv::Mat after_gaus;
	//    cv::GaussianBlur(pyramid_templates[0], after_gaus, cv::Size(5,5),3);
	//    pyramid_templates[0] = after_gaus;

	// 图像的质心
	std::vector<cv::Point> centers;
	std::vector<float> angle_steps;
	std::vector<UINT16> search_rect_width;
	for (auto iter = std::begin(pyramid_templates); iter != std::end(pyramid_templates); ++iter) {
		std::cout << "1 rows: " << iter->rows << " cols: " << iter->cols << std::endl;
	}
	std::cout << "error1" << std::endl;
	for (auto &pyr : pyramid_templates) {
#ifndef  NDEBUG
		//        saveMat(pyr, (std::string("data//") + std::to_string(pyr.rows) + std::to_string(pyr.cols)).c_str());
#endif
		cv::Mat cannyResult;
		cv::Canny(pyr, cannyResult, sensitity_threshold_low, sensitity_threshold_high);
		//cv::imshow("hahahaha", cannyResult);
		//cv::waitKey(0);
		std::cout << "2 rows: " << pyr.rows << " cols: " << pyr.cols << std::endl;
		cv::Point center;
		unsigned int num_of_contour = 0;
		get_center_numof_contour(cannyResult, center, num_of_contour, pyr.rows, pyr.cols);
		centers.push_back(center);

		// 确定角度步长, 使用Canny的轮廓图来计算最远点
		auto step = get_angle_step_and_search_width(cannyResult, center, search_rect_width);
		if (optimal_pyr_level == 0 || optimal_pyr_level == 1) {
			step = 1.0;
		}
		angle_steps.push_back(step);

		std::cout << "num of coordinate this level: " << num_of_contour << std::endl;

		if (optimal_pyr_level >= MAX_NUM_PYRAMID || (num_of_contour < MIN_CONTOUR_PYRA && optimal_pyr_level >= MIN_NUM_PYRAMID)) {
			break;
		}

		// 当策略为高精度时，保证模板层数为3层，提高匹配精度
		if (koyo_tool_contour_parameter->algoStrategy == 0 && optimal_pyr_level == 3) {
			break;
		}
		++optimal_pyr_level;
	}
	std::cout << "optimal level: " << optimal_pyr_level << std::endl;
	//   tt1.stop();
	//    std::cout << "first half: " << tt1.duration() << std::endl;

	std::cout << "error2" << std::endl;
	// 对每层每个角度建立模板
	// tpls中的内存是动态分配的, 在建立完模板后需要释放所有的内存
	// todo 重构成vector版本的
	//    TemplateStruct tpls[optimal_pyr_level][MAX_DEGREE];
	std::vector<std::vector<IVSTemplateSubStruct>> tpls;
	////   TimeTracker tt;
	//    tt.start();
	// optimal_pyr_level肯定小于pyramid_templates的size
	std::cout << "angle_range is : " << koyo_tool_contour_parameter->angleRange << std::endl;
	for (int i = 0; i < optimal_pyr_level; ++i) {
		std::vector<IVSTemplateSubStruct> cur_level_tpl;
		int k = 0;
		std::vector<cv::Point> cur_rect = { { 0, 0 }, { 0, pyramid_templates[i].rows - 1 }, { pyramid_templates[i].cols - 1, pyramid_templates[i].rows - 1 }, { pyramid_templates[i].cols - 1, 0 } };
		// todo angle_step是否考虑做成整数
		for (double j = -(koyo_tool_contour_parameter->angleRange); (int)j < koyo_tool_contour_parameter->angleRange; j += angle_steps[i]) {
			//            std::cout << j << " " << (int)j << std::endl;
			IVSTemplateSubStruct tpl;
			auto rect = cur_rect;
			cv::Mat rotated_image;
			cv::Mat rotated_image_bmap;

			// 在这里做与的操作，便于之后不再进行去除白边的繁琐操作，因此需要进行下列的操作
			cv::Mat rmWhite_image_canny;
			cv::Mat rotated_rmWhite_image(pyramid_templates[i].rows, pyramid_templates[i].cols, CV_8UC1);
			cv::Canny(pyramid_templates[i], rmWhite_image_canny, sensitity_threshold_low, sensitity_threshold_high);
			// 还是无法保证完全在图片框内
			// todo 客户端下发的bitmap也要旋转
			auto rotate_bitmap = rotate_image(pyramid_bitmaps[i], rotated_image_bmap, centers[i], j);
			auto rotate_matrix = rotate_image(pyramid_templates[i], rotated_image, centers[i], j);
			rotate_image(rmWhite_image_canny, rotated_rmWhite_image, centers[i], j);
			cv::threshold(rotated_rmWhite_image, rotated_rmWhite_image, 10, 255, CV_THRESH_BINARY);
			//Dilation(rotated_rmWhite_image, rotated_rmWhite_image, 3);

			//cv::imshow("rmWhite_image_canny", rotated_rmWhite_image);
			//cv::waitKey(0);

			//rotate_rect(rect, rotate_matrix);

			// 求一下该层当前角度对应下层角度的模板图(只针对顶层，一般情况下不考虑)
			IVSTemplateSubStruct pre_tpl;
			if (i == optimal_pyr_level - 1) {
				int angle_idx = (j + koyo_tool_contour_parameter->angleRange) / angle_steps[i - 1];
				if (angle_idx > tpls[i - 1].size() - 1) {
					angle_idx = tpls[i - 1].size() - 1;
				}
				pre_tpl = tpls[i - 1][angle_idx];
			}


			// todo 多传一个参数，旋转后的bitmap, 以及dobitwise_and的flag，只在高分辨率上做bitwiseand
			if (i <= 1) {
				do_create_template(tpl, rotated_image, rotated_image_bmap, 1, sensitity_threshold_low,
					sensitity_threshold_high, rotated_rmWhite_image, i, optimal_pyr_level, pre_tpl);
			}
			else {
				do_create_template(tpl, rotated_image, rotated_image_bmap, 0, sensitity_threshold_low,
					sensitity_threshold_high, rotated_rmWhite_image, i, optimal_pyr_level, pre_tpl);
			}
			//std::cout << "level: " << i << " num: " << tpl.noOfCordinates << std::endl;

			// 打印tpl图片信息
			print_tpl(tpl);
			cur_level_tpl.push_back(tpl);
			//            draw_template(rotated_image, tpl);
			//cv::imshow(std::string("pyr") + std::string(1, i - '0'), rotated_image);
			//cvWaitKey(0);
		}
		tpls.push_back(cur_level_tpl);
	}
	//   tt.stop();
	//  std::cout << tt.duration() << "ms" << std::endl;
	std::cout << "create_template end" << std::endl;
#ifndef NDEBUG
	//    for (auto iter = tpls.cbegin(); iter != tpls.end(); ++iter) {
	//        std::cout << iter->at(0).noOfCordinates << std::endl;
	//    }
#endif
	//建立完模板需要将模板发送给客户端，需要发送的就是tpls这个数据结构



	koyo_contour_template_runtime_param.pyramidLevelNum = optimal_pyr_level;
	koyo_contour_template_runtime_param.searchAngelStep = angle_steps;
	// todo 换成move操作会好一些吧
	koyo_contour_template_runtime_param.tpls = tpls;
	koyo_contour_template_runtime_param.searchRectWidth = search_rect_width;
#ifdef DEBUG_PRINT
	centers_debug = centers;
	angle_steps_debug = angle_steps;
	pyramid_templates_debug = pyramid_templates;
	search_rect_width_debug = search_rect_width;
#endif


	return 0;
}


/*
*  提供给客户端的接口
* */
int ivs_create_template(const UINT8 *yuv, IVSToolContourParameter *koyo_tool_contour_parameter, UINT8 *buf, int buf_free, int *buf_size)
{

	// 获取灰度图
	std::cout << "create_template" << std::endl;
	auto template_image = get_y_from_yuv(yuv, WIDTH, HEIGHT);
	//cv::GaussianBlur(template_image, template_image, cv::Size(5,5),0);
	/* TODO 考虑不要旋转了，直接像圆那样截取出来吧 */
	/* 圆形检测区域不做复杂的旋转后截取，只有矩形的才做旋转后截取 */
	cv::Mat bitmapCleaned;
	cv::Mat template_roi_ext;
	cv::Mat template_roi;

	// 不管圆形还是矩形，都是一样的操作
	// 将位图取出来，将原图外接矩形截取下来
	bitmapCleaned.create(koyo_tool_contour_parameter->extRectHeight, koyo_tool_contour_parameter->extRectWidth, CV_8UC1);

	// 从bitmap中恢复被擦除的位图
	bitmap2Mat(bitmapCleaned, koyo_tool_contour_parameter->bitmaps,
		koyo_tool_contour_parameter->extRectWidth, koyo_tool_contour_parameter->extRectHeight);
	cv::imshow("haha", bitmapCleaned);
	cv::waitKey(0);
	// 从外接矩形位图中获取模板部分的位图
	template_roi = template_image(cv::Rect(koyo_tool_contour_parameter->extRectX, koyo_tool_contour_parameter->extRectY, koyo_tool_contour_parameter->extRectWidth, koyo_tool_contour_parameter->extRectHeight));
	// 保证两次截取出来的图大小一样
	assert(template_roi.size == bitmapCleaned.size);
	//    std::cout << template_roi.cols << " " << template_roi.rows << std::endl;
	//    template_roi = template_image;

	// 使用截取出来的图片进行轮廓建立
	// 这之后擦除的代码不用改，保证这里传入的bitmap是对着的就行了
	IVSTemplateStruct koyo_contour_template_runtime_param;
	do_create_template(template_roi, bitmapCleaned, koyo_tool_contour_parameter, koyo_contour_template_runtime_param);

	// 打包后的template_data是unique_ptr上的指针，调用release来获取原始指针，但是要记得delete []这个内存
	std::cout << "test pack template" << std::endl;
	//auto template_data = ;

	int status = pack_template(koyo_contour_template_runtime_param, buf, buf_free, buf_size);
	if (status < 0) {
		return -1;
	}
	std::cout << " first coordinate num in: " << *(int*)(buf + 1 + 4 * 3 + 2 * 3 + 2 + 1) << std::endl;
	std::cout << " first coordinate num in: " << *(int*)(buf + 1 + 4 * 4 + 2 * 4 + 2 + 1) << std::endl;
	std::cout << " first coordinate num in: " << *(int*)(buf + 1 + 5 * 4 + 2 * 5 + 2 + 1) << std::endl;
	std::cout << " first coordinate num in: " << *(int*)(buf + 1 + 6 * 4 + 2 * 6 + 2 + 1) << std::endl;


	std::cout << "*****************" << buf_size << std::endl;
	//    cv::imshow("eh" ,template_roi);
	//    cvWaitKey(0);
	std::cout << "now all done" << std::endl;
	return 0;
	//return template_data.release();
}


int get_contours(const UINT8 *yuv, UINT8 *contours[3], int low_threshold, int high_threshold)
{
	int low = low_threshold;
	int high = low_threshold * 3;
	auto src = get_y_from_yuv(yuv, WIDTH, HEIGHT);
	cv::GaussianBlur(src, src, cv::Size(3, 3), 0);
	cv::Mat contour;
	cv::Canny(src, contour, low, high);
	for (int i = 0; i < HEIGHT; ++i) {
		for (int j = 0; j < WIDTH; ++j) {
			contours[0][i * WIDTH + j] = contour.at<uchar>(i, j);
		}
	}
	return 0;

}

