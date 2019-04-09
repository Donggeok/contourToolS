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
	INT8	toolName[32];			// ��������
	UINT8	regionType;				// ���������״������Ϊ1��Բ��Ϊ0��
	UINT8	toolType;				// ��������

	/* ���μ������
	*
	*		0-----------3
	*		|			|
	*		|			|
	*		1-----------2
	*/

	INT16	detectRectX0;			// ������Ͻ�Բ���������ϵ������
	INT16	detectRectY0;			// ������Ͻ�Բ���������ϵ�������
	INT16	detectRectX1;			// ������Ͻ�Բ���������µ������
	INT16	detectRectY1;			// ������Ͻ�Բ���������µ�������
	INT16	detectRectX2;			// ������Ͻ�Բ���������µ������
	INT16	detectRectY2;			// ������Ͻ�Բ���������µ�������
	INT16	detectRectX3;			// ������Ͻ�Բ���������ϵ������
	INT16	detectRectY3;			// ������Ͻ�Բ���������ϵ�������

	/* ���Բ���� */
	INT16	detectCircleX;			// ���Բ�ĺ�����
	INT16	detectCircleY;			// ���Բ��������
	UINT16	detectCircleRadius;		// ���Բ�뾶

	/* ��Ӿ���������� */
	INT16	extRectX;				// �����Ӿ�����ʼ�������
	INT16	extRectY;				// �����Ӿ�����ʼ��������
	UINT16	extRectWidth;			// �����Ӿ��ο���
	UINT16	extRectHeight;			// �����Ӿ��θ߶�

	/* ����������� */
	INT16	searchRectX0;			// ������Ͻ�Բ���������ϵ������
	INT16	searchRectY0;			// ������Ͻ�Բ���������ϵ�������
	INT16	searchRectX1;			// ������Ͻ�Բ���������µ������
	INT16	searchRectY1;			// ������Ͻ�Բ���������µ�������
	INT16	searchRectX2;			// ������Ͻ�Բ���������µ������
	INT16	searchRectY2;			// ������Ͻ�Բ���������µ�������
	INT16	searchRectX3;			// ������Ͻ�Բ���������ϵ������
	INT16	searchRectY3;			// ������Ͻ�Բ���������ϵ�������

	/* �㷨������� */
	UINT8	algoStrategy;			// ���ٶȻ��߸߾����㷨���ԣ��߾���Ϊ0�����ٶ�Ϊ1��
	UINT16	angleRange;				// �����Ƕȷ�Χ
	UINT16	sensiTopThreshold;		// canny������ֵ����
	UINT16	sensiLowThreshold;		// canny������ֵ����

	/* �㷨������������� */
	UINT16	scoreTopThreshold;		// ���������ֵ����
	UINT16	scoreLowThreshold;		// ���������ֵ����
	UINT32  erasureBitmapSize;		// ����λͼ��С,��λ��bit
	UINT32	templateBitmapSize;		// ����ģ��λͼ�Ĵ�С
	INT8	templatePath[128];		// λͼ·��
	UINT8	*bitmaps;				// ���������Ӿ���λͼ,��������
};

#endif