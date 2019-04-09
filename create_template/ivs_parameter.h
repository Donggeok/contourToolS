#ifndef IVS_PARAMETER_H
#define IVS_PARAMETER_H

using UINT8		= unsigned char;
using INT8		= signed char;
using UINT16	= unsigned short;
using INT16		= signed short;
using UINT32	= unsigned int;
using INT32		= signed int;

struct IVSToolContourParameter
{
	INT8	toolName[32];			// 工具名称
	UINT8	regionType;				// 检测区域形状（矩形为1，圆形为0）
	UINT8	toolType;				// 工具类型

	/* 矩形检测框参数
	*
	*		0-----------3
	*		|			|
	*		|			|
	*		1-----------2
	*/

	INT16	detectRectX0;			// 相对左上角圆点坐标左上点横坐标
	INT16	detectRectY0;			// 相对左上角圆点坐标左上点纵坐标
	INT16	detectRectX1;			// 相对左上角圆点坐标左下点横坐标
	INT16	detectRectY1;			// 相对左上角圆点坐标左下点纵坐标
	INT16	detectRectX2;			// 相对左上角圆点坐标右下点横坐标
	INT16	detectRectY2;			// 相对左上角圆点坐标右下点纵坐标
	INT16	detectRectX3;			// 相对左上角圆点坐标右上点横坐标
	INT16	detectRectY3;			// 相对左上角圆点坐标右上点纵坐标

	/* 检测圆参数 */
	INT16	detectCircleX;			// 检测圆心横坐标
	INT16	detectCircleY;			// 检测圆心纵坐标
	UINT16	detectCircleRadius;		// 检测圆半径

	/* 外接矩形区域参数 */
	INT16	extRectX;				// 检测外接矩形起始点横坐标
	INT16	extRectY;				// 检测外接矩形起始点纵坐标
	UINT16	extRectWidth;			// 检测外接矩形宽度
	UINT16	extRectHeight;			// 检测外接矩形高度

	/* 搜索区域参数 */
	INT16	searchRectX0;			// 相对左上角圆点坐标左上点横坐标
	INT16	searchRectY0;			// 相对左上角圆点坐标左上点纵坐标
	INT16	searchRectX1;			// 相对左上角圆点坐标左下点横坐标
	INT16	searchRectY1;			// 相对左上角圆点坐标左下点纵坐标
	INT16	searchRectX2;			// 相对左上角圆点坐标右下点横坐标
	INT16	searchRectY2;			// 相对左上角圆点坐标右下点纵坐标
	INT16	searchRectX3;			// 相对左上角圆点坐标右上点横坐标
	INT16	searchRectY3;			// 相对左上角圆点坐标右上点纵坐标

	/* 算法策略相关 */
	UINT8	algoStrategy;			// 高速度或者高精度算法策略（高精度为0，高速度为1）
	UINT16	angleRange;				// 搜索角度范围
	UINT16	sensiTopThreshold;		// canny算子阈值上限
	UINT16	sensiLowThreshold;		// canny算子阈值下限

	/* 算法处理及评分相关 */
	UINT16	scoreTopThreshold;		// 处理结果阈值上限
	UINT16	scoreLowThreshold;		// 处理结果阈值下限
	UINT32  erasureBitmapSize;		// 擦除位图大小,单位是bit
	UINT32	templateBitmapSize;		// 工具模板位图的大小
	INT8	templatePath[128];		// 位图路径
	UINT8	*bitmaps;				// 检测区域外接矩形位图,柔性数组
};

#endif