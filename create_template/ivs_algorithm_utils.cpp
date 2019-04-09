#include "ivs_algorithm_utils.h"

void saveMat(cv::Mat mat, const char *path) {
	FILE *fp = fopen(path, "w");
	int i, j;
	for (i = 0; i < mat.rows; ++i) {
		for (j = 0; j < mat.cols; ++j) {
			//            fprintf(fp, "%d ", (mat.ptr + i * mat.step)[j]);
			fprintf(fp, "%d ", mat.at<uchar>(i, j));
		}
		fprintf(fp, "\n");
	}
	fclose(fp);
}

void saveMatf(cv::Mat mat, const char *path) {
	FILE *fp = fopen(path, "w");
	int i, j;
	for (i = 0; i < mat.rows; ++i) {
		for (j = 0; j < mat.cols; ++j) {
			//            fprintf(fp, "%d ", (mat.ptr + i * mat.step)[j]);
			fprintf(fp, "%d ", mat.at<short>(i, j));
		}
		fprintf(fp, "\n");
	}
	fclose(fp);
}

cv::Mat get_y_from_yuv(const UINT8 *yuv, const UINT16 width, const UINT16 height)
{
	cv::Mat gray;
	if (!yuv) {
		std::cout << "ERROR: yuv NULL pointer" << std::endl;
	}
	else {
		gray.create(height, width, CV_8UC1);
		memcpy(gray.data, yuv, width * height * sizeof(unsigned char)); // 只读取y分量上的数据
	}

	return gray;
}


void bitmap2Mat(cv::Mat &dst, UINT8 bitmap[], UINT16 width, UINT16 height) {
	for (int i = 0; i < height; ++i) {
		for (int j = 0; j < width; ++j) {
			//客户端传来的是1 0
			dst.at<uchar>(i, j) = static_cast<uchar>(bitmap[i * width + j] * 255);
		}
	}
}

//bitmap要提前分配好空间
void mat2Bitmap(const cv::Mat &src, UINT8 bitmap[], UINT16 width, UINT16 height) {
	for (int i = 0; i < height; ++i) {
		for (int j = 0; j < width; ++j) {
			bitmap[i * width + j] = src.at<uchar>(i, j) == 255 ? 1 : 0;
		}
	}
}