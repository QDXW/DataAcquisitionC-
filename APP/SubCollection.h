#ifndef __SUB_COLLECTION_H_
#define __SUB_COLLECTION_H_
#include <stdint.h>
#include "IEC104.h"
#include <time.h>

#define SUBCHECKSIZE 8	        //���ʱ��ʱ��flash�ж�ȡ�ĳ���
#define YXBUFFSIZE 67	        //���ڴ���ʷ�����ж�ȡ��Ҫ�ϱ��Ĳ���ң������
#define YCBUFFSIZE 46*4+8	        //���ڴ���ʷ�����ж�ȡ��Ҫ�ϱ��Ĳ���ң������
#define INTERVAL_TIME_RANG 150 	//ƽָ̨��ʱ���ǰ�����ְ������
#define NO_DATA -2              //������ʷ���ݲ��Ҳ�����Ҫ����ʱ��ǰ��2.5���ӷ�Χ�ڵ�����


extern int32_t tMarkCheck(uint8_t menAddr[],time_t targetTime );

extern int32_t hMarkCheck(uint8_t array[]);

extern int32_t CollectData(IEC104_MAIN_T *pA, uint32_t *uNeedReadFlash);





#endif
