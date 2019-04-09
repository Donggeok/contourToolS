#ifndef IVS_ALGORITHM_UTILS_H
#define IVS_ALGORITHM_UTILS_H

#include <iostream>
#include <opencv.hpp>

#include "ivs_parameter.h"

// ÉùÃ÷
void saveMat(cv::Mat mat, const char *path);
void saveMatf(cv::Mat mat, const char *path);
cv::Mat get_y_from_yuv(const UINT8 *yuv, const UINT16 width, const UINT16 height);
void bitmap2Mat(cv::Mat &dst, UINT8 bitmap[], UINT16 width, UINT16 height);
void mat2Bitmap(const cv::Mat &src, UINT8 bitmap[], UINT16 width, UINT16 height);


#endif