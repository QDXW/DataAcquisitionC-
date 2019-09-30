#ifndef SLAVEUNIT_H_
#define SLAVEUNIT_H_


#include "ucos_ii.h"

#ifndef _STDINT
#include <stdint.h>
#endif

#define TaskSouthInquireStkSize   256   // ��ѯ�����ջ��С
#define TaskSouthWriteStkSize     256   // д�Ĵ��������ջ��С

#define SOUTH_CMD_YK     (1)   // ����д���ң��
#define SOUTH_CMD_SD     (2)   // ����д������
#define SOUTH_CMD_SYNC   (3)   // ����д���ͬ��ʱ��
#define SOUTH_CMD_READSD (4)   // �������������


#define BAUDRATE_2400     1
#define BAUDRATE_4800     2
#define BAUDRATE_9600     3
#define BAUDRATE_19200    4
#define BAUDRATE_38400    5
#define BAUDRATE_115200   6




//extern DT1000UPDATA_DATA_T g_DT1000DataLen;
//extern static uint8_t  s_uSouthReadSd=SOUTH_CMD_READSD; //��������
extern OS_STK TaskSouthInquireStk[TaskSouthInquireStkSize];	  // ���������ջ��С
extern OS_STK TaskSouthWriteStk[TaskSouthWriteStkSize];	      // ���������ջ��С

extern void TaskSouthWrite(void *p);
extern void TaskSouthInquire(void *p);

extern void AlarmAck(uint8_t uLen,const uint8_t *puData);
extern uint8_t AlarmReport(uint8_t addr,uint8_t err_code,uint16_t reg_addr,uint8_t offset,uint8_t event,uint8_t imd);
extern int8_t CheckDevState(void);
#endif
