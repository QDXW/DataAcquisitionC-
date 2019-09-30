#ifndef LOG_H_
#define LOG_H_

#include <stdint.h>

#include "RealTime.h"


#define LOG_HEAD_A     0xBB
#define LOG_HEAD_B     0x66


//====================================================
typedef struct
{
    uint32_t  LogFlashAddr;      // ���Dataflash�洢��ַ
    uint16_t  uIsExist;          // ������//Ԥ��uReserve
    uint16_t  LogCRC;             // ����CRCУ��
}LOG_STRUCT_T;

extern LOG_STRUCT_T sLogData; // ��¼�洢�������һ����DataFlash

extern uint8_t  *SOUTH_DATA_LOG;   // ��������ָ�룬��̬����ռ�
//extern uint8_t  *NORTH_DATA_LOG;   // ��������ָ�룬��̬����ռ�
extern uint8_t CopyData(uint8_t *rec);
extern uint16_t SerchLog(uint8_t date,uint32_t *uLogDataFlash);
extern uint8_t uGetLastRecordAddr(SYSTEMTIME sTime);
extern uint32_t uSaveSouthLog(SYSTEMTIME sTime,const uint8_t *puSouthLog,const uint8_t *puNorthLog,uint16_t uSouthLogPoints,uint16_t uNorthLogPoints);
extern void SaveLogInit(uint8_t uReset);
//====================================================
#endif
