#include <iostream>
#include "ivs_algorithm_utils.h"

int main(){
	ContourUtility contourUtility;
	// �����������ߵĹ��߽ṹ��
	createUtility(contourUtility);

	IVSOriPic pic;
	pic.width = 0;
	pic.height = 0;
	
	// ����ÿһ֡����Ҫ���¼���ù��߽ṹ�� 
	computeUtility(contourUtility, pic);




	// �ͷ��������ߵĹ��߽ṹ��
	freeUtility(contourUtility);

	return 0;
}