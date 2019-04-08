#include <iostream>
#include "ivs_algorithm_utils.h"

int main(){
	ContourUtility contourUtility;
	// 创建轮廓工具的工具结构体
	createUtility(contourUtility);

	IVSOriPic pic;
	pic.width = 0;
	pic.height = 0;
	
	// 对于每一帧都需要重新计算该工具结构体 
	computeUtility(contourUtility, pic);




	// 释放轮廓工具的工具结构体
	freeUtility(contourUtility);

	return 0;
}