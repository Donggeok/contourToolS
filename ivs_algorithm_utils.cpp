#include "ivs_algorithm_utils.h"
#include "ivs_parameter.h"

int createUtility(ContourUtility &contourUtility, IVSOriPic ivsOriPic){

	// ������ͼƬת��Ϊopencv��Mat���ݽṹ


	/* ������������ͼƬ�ݶ�����ռ� */
	INT32 s32Ret = 0;

	contourUtility->u16Width = u16width;
	tool_utility->u16Height = u16height;

	for (INT32 i = 0; i < MAX_NUM_PYRAMID; i++) {
		INT32 width = u16width >> i;
		INT32 height = u16height >> i;     ////��������ȫ�ֶ�����ĳһ���ֵ�
		if (width < 64) width = 64;
		if (height < 64) height = 64;
		/* X������ݶ�ͼƬ */
		s32Ret =
			SAMPLE_COMM_IVE_CreateImageByCached(&(tool_utility->Sdx[i]), IVE_IMAGE_TYPE_S16C1,
			width, height);
		if (s32Ret != HI_SUCCESS) {
			KOYO_LOG_ERROR("SAMPLE_COMM_IVE_CreateImage fail\n");
			return -1;
		}

		/* Y������ݶ�ͼƬ */
		s32Ret =
			SAMPLE_COMM_IVE_CreateImageByCached(&(tool_utility->Sdy[i]), IVE_IMAGE_TYPE_S16C1,
			width, height);
		if (s32Ret != HI_SUCCESS) {
			KOYO_LOG_ERROR("SAMPLE_COMM_IVE_CreateImage fail\n");
			return -1;
		}

		/* ������ͼƬ */
		s32Ret =
			SAMPLE_COMM_IVE_CreateImageByCached(&(tool_utility->search_region[i]), IVE_IMAGE_TYPE_U8C1,
			width, height);
		if (s32Ret != HI_SUCCESS) {
			KOYO_LOG_ERROR("SAMPLE_COMM_IVE_CreateImage fail\n");
			return -1;
		}

		/* ��ֵ��ͼƬ */
		s32Ret =
			SAMPLE_COMM_IVE_CreateImageByCached(&(tool_utility->search_region_binary[i]), IVE_IMAGE_TYPE_U8C1,
			width, height);
		if (s32Ret != HI_SUCCESS) {
			KOYO_LOG_ERROR("SAMPLE_COMM_IVE_CreateImage fail\n");
			return -1;
		}

		/* dilated��ֵ��ͼƬ */
		s32Ret =
			SAMPLE_COMM_IVE_CreateImageByCached(&(tool_utility->dilated_search_region_binary[i]), IVE_IMAGE_TYPE_U8C1,
			width, height);
		if (s32Ret != HI_SUCCESS) {
			KOYO_LOG_ERROR("SAMPLE_COMM_IVE_CreateImage fail\n");
			return -1;
		}


		/* ��ʾ������Ϊ4��ʱ�Ķ������ͼ*/
		s32Ret = SAMPLE_COMM_IVE_CreateImageByCached(&(tool_utility->image_search_integ[i]),
			IVE_IMAGE_TYPE_U32C1, tool_utility->search_region[i].u16Width,
			tool_utility->search_region[i].u16Height);
		if (s32Ret != HI_SUCCESS) {
			KOYO_LOG_ERROR("SAMPLE_COMM_IVE_CreateImage fail\n");
			return -1;
		}

	}


	s32Ret = SAMPLE_COMM_IVE_CreateImageByCached(&(tool_utility->src_filtered), IVE_IMAGE_TYPE_U8C1, u16width, u16height);
	if (s32Ret != HI_SUCCESS) {
		KOYO_LOG_ERROR("SAMPLE_COMM_IVE_CreateImage fail\n");
		return -1;
	}

	ISize size;
	size.width = LOOKUPTABLE_SIZE;
	size.height = LOOKUPTABLE_SIZE;

	CreateFloatMatrix(&(tool_utility->lookupTableX), size);
	CreateFloatMatrix(&(tool_utility->lookupTableY), size);
	CreateFloatMatrix(&(tool_utility->lookupTableS), size);

	INT32 i, j;
	/* ��ʼ�����ұ� */
	for (i = 0; i < LOOKUPTABLE_SIZE; i++) {
		for (j = 0; j < LOOKUPTABLE_SIZE; ++j) {
			float length;
			length = sqrtf(i * i + j * j);
			tool_utility->lookupTableS[i][j] = length;                  //ǿ�Ȳ��ұ����ڼ����ֵ��ͼ
			tool_utility->lookupTableX[i][j] = 1.0f * (float)i / length;
			tool_utility->lookupTableY[i][j] = 1.0f * (float)j / length;
		}
	}
	tool_utility->lookupTableX[0][0] = 0;
	tool_utility->lookupTableY[0][0] = 0;
	tool_utility->lookupTableS[0][0] = 0;


	/* �������������ʱ��һ���ݶȷ��������Ĵ洢�� */
	for (i = 0; i < MAX_NUM_PYRAMID; i++) {
		int width = u16width >> i;
		int height = u16height >> i;    //��������ȫ�ֶ�����ĳһ���ֵ�
		//        int width = pcontour_parameter->search_rect_width >> i;
		//        int height = pcontour_parameter->search_rect_height >> i;
		if (width < 64) width = 64;
		if (height < 64) height = 64;
		size.width = width;
		size.height = height;

		CreateFloatMatrix(&(tool_utility->edgeX[i]), size);	// create image to save gradient magnitude  values
		CreateFloatMatrix(&(tool_utility->edgeY[i]), size);	// create image to save gradient magnitude  values

		create_bool_matrix(&(tool_utility->is_calculated[i]), size);
		//        printf("build gradiant w %d, h %d. is_calculated[%d] %#p.\n",size.width,size.height,i,pstTemplateMatch->is_calculated[i]);

	}

	/* �����ѡ������ͼ */
	for (i = 0; i < MAX_NUM_PYRAMID - 1; i++) {
		int width = u16width >> i;
		int height = u16height >> i;
		if (width < 64) width = 64;
		if (height < 64) height = 64;
		SAMPLE_COMM_IVE_CreateImage(&(tool_utility->mask_ang_region_idx_u8[i]), IVE_IMAGE_TYPE_U8C1, width, height);
		if (s32Ret != HI_SUCCESS) {
			KOYO_LOG_ERROR("SAMPLE_COMM_IVE_CreateImage fail\n");
			return -1;
		}
	}

	/* ����λ�ýǶȼ����ظ��ռ�,����ʹ��set���� */
	//    tool_utility->pset_coll = create_set(int);
	tool_utility->phset_coll = create_hash_set(int);
	if (tool_utility->phset_coll == NULL)
	{
		KOYO_LOG_ERROR("set create fail\n");
		return -1;
	}
	//    set_init(tool_utility->pset_coll);
	hash_set_init(tool_utility->phset_coll);

	
	return 1;
}

//�Ƿ���Ҫ�Ƚ������srcת��ΪU8C1
INT8 compute_tool_utility(Tool_Utility *tool_utility, IVE_IMAGE_S *src, int lowThreshold, int highThreshold){
	//    struct timeval tt[2];
	//    start(tt);

	lowThreshold = lowThreshold > 255 ? 255 : lowThreshold;
	highThreshold = highThreshold > 255 ? 255 : highThreshold;

	IVE_INTEG_CTRL_S integ_ctrl_s = { .enOutCtrl = IVE_INTEG_OUT_CTRL_SUM };
	HI_S32 s32Ret = HI_SUCCESS;
	Rect roi_rect;

	UINT16 u16width = tool_utility->u16Width;
	UINT16 u16height = tool_utility->u16Height;

	//�˲�
	hi_ive_filter(src, &(tool_utility->src_filtered));

	s32Ret = HI_MPI_SYS_MmzFlushCache(tool_utility->src_filtered.u32PhyAddr[0],
		tool_utility->src_filtered.pu8VirAddr[0],
		tool_utility->src_filtered.u16Stride[0] *
		tool_utility->src_filtered.u16Height);
	if (s32Ret != HI_SUCCESS) {
		KOYO_LOG_ERROR("HI_MPI_SYS_MmzFlushCache fail,Error(%#x)\n", s32Ret);
		return;
	}


	//������������,Ŀǰ�ڹ������ݽṹ������Ƕ�ֵ
	roi_rect.x = 0;
	roi_rect.y = 0;
	roi_rect.width = u16width;
	roi_rect.height = u16height;
	//fixme:the image is filtered twice
	image_process_create_select_roi(&(tool_utility->src_filtered), &roi_rect, &(tool_utility->search_region[0]));

	s32Ret = HI_MPI_SYS_MmzFlushCache(tool_utility->search_region[0].u32PhyAddr[0],
		tool_utility->search_region[0].pu8VirAddr[0],
		tool_utility->search_region[0].u16Stride[0] *
		tool_utility->search_region[0].u16Height);
	if (s32Ret != HI_SUCCESS) {
		KOYO_LOG_ERROR("HI_MPI_SYS_MmzFlushCache fail,Error(%#x)\n", s32Ret);
		return;
	}


	//    stop(tt);
	//    KOYO_LOG_ERROR("compute complete cost: %d\n", duration(tt));
	//    start(tt);
	for (INT32 l = 1; l < MAX_NUM_PYRAMID; ++l) {
		if (image_create_and_downsampling_min64x64(&(tool_utility->search_region[l - 1]),
			&(tool_utility->search_region[l])) != HI_SUCCESS){
			KOYO_LOG_ERROR("error down sampling.\n");
		}

		s32Ret = HI_MPI_SYS_MmzFlushCache(tool_utility->search_region[l].u32PhyAddr[0],
			tool_utility->search_region[l].pu8VirAddr[0],
			tool_utility->search_region[l].u16Stride[0] *
			tool_utility->search_region[l].u16Height);
		if (s32Ret != HI_SUCCESS) {
			KOYO_LOG_ERROR("HI_MPI_SYS_MmzFlushCache fail,Error(%#x)\n", s32Ret);
			return;
		}

		//����sobelͼƬ�����ҹ�һ��

		hi_ive_sobel(&(tool_utility->search_region[l]), &(tool_utility->Sdx[l]), &(tool_utility->Sdy[l]));

		s32Ret = HI_MPI_SYS_MmzFlushCache(tool_utility->Sdx[l].u32PhyAddr[0],
			tool_utility->Sdx[l].pu8VirAddr[0],
			tool_utility->Sdx[l].u16Stride[0] *
			tool_utility->Sdx[l].u16Height);
		if (s32Ret != HI_SUCCESS) {
			KOYO_LOG_ERROR("HI_MPI_SYS_MmzFlushCache fail,Error(%#x)\n", s32Ret);
			return;
		}

		s32Ret = HI_MPI_SYS_MmzFlushCache(tool_utility->Sdy[l].u32PhyAddr[0],
			tool_utility->Sdy[l].pu8VirAddr[0],
			tool_utility->Sdy[l].u16Stride[0] *
			tool_utility->Sdy[l].u16Height);
		if (s32Ret != HI_SUCCESS) {
			KOYO_LOG_ERROR("HI_MPI_SYS_MmzFlushCache fail,Error(%#x)\n", s32Ret);
			return;
		}

		//        if(l == MAX_NUM_PYRAMID-1)

	}
	hi_ive_sobel(&(tool_utility->search_region[0]), &(tool_utility->Sdx[0]), &(tool_utility->Sdy[0]));

	s32Ret = HI_MPI_SYS_MmzFlushCache(tool_utility->Sdx[0].u32PhyAddr[0],
		tool_utility->Sdx[0].pu8VirAddr[0],
		tool_utility->Sdx[0].u16Stride[0] *
		tool_utility->Sdx[0].u16Height);
	if (s32Ret != HI_SUCCESS) {
		KOYO_LOG_ERROR("HI_MPI_SYS_MmzFlushCache fail,Error(%#x)\n", s32Ret);
		return;
	}

	s32Ret = HI_MPI_SYS_MmzFlushCache(tool_utility->Sdy[0].u32PhyAddr[0],
		tool_utility->Sdy[0].pu8VirAddr[0],
		tool_utility->Sdy[0].u16Stride[0] *
		tool_utility->Sdy[0].u16Height);
	if (s32Ret != HI_SUCCESS) {
		KOYO_LOG_ERROR("HI_MPI_SYS_MmzFlushCache fail,Error(%#x)\n", s32Ret);
		return;
	}

	long strngth = 0;
	u_char *tmpbufx, *tmpbufy;
	tmpbufx = tool_utility->Sdx[0].pu8VirAddr[0];
	tmpbufy = tool_utility->Sdy[0].pu8VirAddr[0];
	for (int i = 0; i < 480; ++i) {
		for (int j = 0; j < 640; ++j) {
			strngth += tmpbufx[j];
			strngth += tmpbufy[j];
		}
		tmpbufx += 640;
		tmpbufy += 640;
	}
	KOYO_LOG_DEBUG("strength:%d\n", strngth);
	//    get_contour_binary(&(tool_utility->search_region[0]),&(tool_utility->search_region_binary[0]),0,30,150);


	// �ͽ�ģ����ͳһ��ʽ�����ڶ���������ֵͼ����Ȼ���ٽ����������㣬��֤��������
	image_create_and_downsampling_min64x64(&(tool_utility->dilated_search_region_binary[MAX_NUM_PYRAMID - 3]), &(tool_utility->search_region_binary[MAX_NUM_PYRAMID - 2]));
	image_create_and_downsampling_min64x64(&(tool_utility->dilated_search_region_binary[MAX_NUM_PYRAMID - 2]), &(tool_utility->search_region_binary[MAX_NUM_PYRAMID - 1]));

	//    hi_ive_integ_at(&(tool_utility->search_region_binary[3]),&(tool_utility->image_search_integ[0]),0,0,&integ_ctrl_s);
	//    stop(tt);
	//    SAMPLE_PRT("compute complete cost: %d\n", duration(tt));
	return 1;
}

INT8 free_tool_utility(Tool_Utility *tool_utility){
	for (INT32 i = 0; i < MAX_NUM_PYRAMID; i++) {
		HI_IMAGES_S_RELEASE(&(tool_utility->search_region[i]));
		HI_IMAGES_S_RELEASE(&(tool_utility->search_region_binary[i]));

		HI_IMAGES_S_RELEASE(&(tool_utility->dilated_search_region_binary[i]));


		HI_IMAGES_S_RELEASE(&(tool_utility->Sdx[i]));
		HI_IMAGES_S_RELEASE(&(tool_utility->Sdy[i]));
		if (i < MAX_NUM_PYRAMID - 1)
		{
			HI_IMAGES_S_RELEASE(&(tool_utility->mask_ang_region_idx_u8[i]));
		}
	}
	for (INT32 i = 0; i < MAX_NUM_PYRAMID; i++) {
		HI_IMAGES_S_RELEASE(&(tool_utility->image_search_integ[i]));

	}
	HI_IMAGES_S_RELEASE(&(tool_utility->src_filtered));

	ISize size;
	size.width = LOOKUPTABLE_SIZE;
	size.height = LOOKUPTABLE_SIZE;
	//SAMPLE_PRT("here.\n");
	/* �ͷŲ��ұ� */
	ReleaseFloatMatrix(&(tool_utility->lookupTableX), size.height);
	ReleaseFloatMatrix(&(tool_utility->lookupTableY), size.height);
	ReleaseFloatMatrix(&(tool_utility->lookupTableS), size.height);

	//SAMPLE_PRT("here.\n");
	/* �ͷ�edgeX */
	/* �ͷ�edgeY */
	/* �ͷ�is_calculated */
	for (INT32 i = 0; i < MAX_NUM_PYRAMID; i++) {
		int width = tool_utility->u16Width >> i;
		int height = tool_utility->u16Height >> i;
		if (width < 64) width = 64;
		if (height < 64) height = 64;
		size.width = width;
		size.height = height;
		ReleaseFloatMatrix(&(tool_utility->edgeX[i]), size.height);
		ReleaseFloatMatrix(&(tool_utility->edgeY[i]), size.height);
		release_bool_matrix(&(tool_utility->is_calculated[i]), size);
	}
	//    int width = tool_utility->u16Width >> 1;
	//    int height = tool_utility->u16Height >> 1;
	//    for (INT16 wtemp = 0; wtemp < width; ++wtemp) {
	//        for (INT16 htemp = 0; htemp < height; ++htemp) {
	//            free(tool_utility->iscompute[wtemp][htemp]);
	//        }
	//        free(tool_utility->iscompute[wtemp]);
	//    }
	//    set_destroy(tool_utility->pset_coll);
	hash_set_destroy(tool_utility->phset_coll);
	//    free(tool_utility->iscompute);
	return 1;
}