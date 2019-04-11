#ifndef IVS_ALGORITHM_UTILS_H
#define IVS_ALGORITHM_UTILS_H

#include "ivs_parameter.h"
#include <opencv.hpp>
#include <cstdarg>

// 声明
void saveMat(cv::Mat mat, const char *path);
void saveMatf(cv::Mat mat, const char *path);
cv::Mat get_y_from_yuv(const UINT8 *yuv, const UINT16 width, const UINT16 height);
void bitmap2Mat(cv::Mat &dst, UINT8 bitmap[], UINT16 width, UINT16 height);
void mat2Bitmap(const cv::Mat &src, UINT8 bitmap[], UINT16 width, UINT16 height);
void Dilation(const cv::Mat &src, cv::Mat &dilation_dst, int size);
void Erode(const cv::Mat &src, cv::Mat &erode_dst, int size);

// 第一个参数代表要比较大小的个数
template<class T>
T IVS_MIN(int num, ...) {
	va_list arg_ptr;
	va_start(arg_ptr, num);
	T tmp = va_arg(arg_ptr, T);
	T result = tmp;
	for (int i = 1; i < num; ++i) {
		tmp = va_arg(arg_ptr, T);
		if (result > tmp) {
			result = tmp;
		}
	}
	va_end(arg_ptr);
	return result;
}

// 第一个参数代表要比较大小的个数
template<class T>
T IVS_MAX(int num, ...) {
	va_list arg_ptr;
	va_start(arg_ptr, num);
	T tmp = va_arg(arg_ptr, T);
	T result = tmp;
	for (int i = 1; i < num; ++i) {
		tmp = va_arg(arg_ptr, T);
		if (result < tmp) {
			result = tmp;
		}
	}
	va_end(arg_ptr);
	return result;
}

#endif