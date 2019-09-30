#include <stdio.h>
#include <time.h>
#include "log.h"
#include "IEC104.h"
#include "GlobalVar.h"
#include "DataFlash.h"
#include "InEEPROM.h"
#include "WatchDog.h"
#include "memory.h"


#define RECORD_SECTOR_SIZE 4096  // 4kB  DataFlashһ������Ϊ4KB
#define FEED_WATCH_DOG()   FEED_DOG()    // ι��

/******************************************************************************
* ��������
******************************************************************************/
uint8_t g_uSaveLog=0;            // �洢��ʷ���������յ���ʱ֮������洢����⵽���������洢
LOG_STRUCT_T sLogData={0};       // ��¼�洢�������һ����DataFlash

uint8_t  *SOUTH_DATA_LOG;        // ��������ָ�룬��̬����ռ�

/******************************************************************************
* ��     �ƣ�CopyData()
* ��     �ܣ��������ݲɼ�
* ��ڲ�����     *rec        ������                          
* ���ڲ�������
* ��     ��: ��
******************************************************************************/
uint8_t CopyData(uint8_t *rec)
{
	if(rec[0] > 0 && rec[0] <= 10)
	{
		if(rec[1] == 0x3B || rec[1] == 0xBB)
		{
			return 0;
		}

		if(SOUTH_DATA_LOG[rec[0]*2 - 1] == 0xFF)
		{
			SOUTH_DATA_LOG[(rec[0]-1)*2] = rec[0];
			SOUTH_DATA_LOG[rec[0]*2 - 1] = rec[1];
		}
		else if(!(SOUTH_DATA_LOG[rec[0]*2 - 1] & 0x80))
		{
			SOUTH_DATA_LOG[(rec[0]-1)*2] = rec[0];
			SOUTH_DATA_LOG[rec[0]*2 - 1] = rec[1];
		}
	}
    return 0;
}

/******************************************************************************
* ��     �ƣ�SerchLog()
* ��     �ܣ�������־����
* ��ڲ�����     date ʱ��  �ṹ��               
* ���ڲ�����*uLogDataFlash      ��־����dataflash��ַ
* ��     ��: �� 
******************************************************************************/
uint16_t SerchLog(uint8_t date,uint32_t *uLogDataFlash)
{
    uint8_t dst_date = 0;
    uint16_t result = 0;
    uint8_t uCheckLog[30] = {0};
    uint32_t uLogDataReadFlash;
    
    dst_date = date;

    if(dst_date == sDateTime.Date)
    {
        result = (sDateTime.Hour*12+sDateTime.Minute/5)*26/200+1;     //  26/200=0.13
    }else{
        result = 38;   //12*24*26/200
    }
    
    uLogDataReadFlash = DATAFLASH_LOG_ONE;

    DataFlash_Read(uLogDataReadFlash,uCheckLog,30);

    if(uCheckLog[3] == dst_date || uCheckLog[29] == dst_date)
    {
        if((0xBB==uCheckLog[0] && 0x66==uCheckLog[1]) || (0xBB==uCheckLog[26] && 0x66==uCheckLog[27]))
        {
             *uLogDataFlash = uLogDataReadFlash;
             DEBUGOUT("\nDATAFLASH_LOG_ONE\n");
             return result;
        }
    }
    uLogDataReadFlash = DATAFLASH_LOG_TWO;
    DataFlash_Read(uLogDataReadFlash,uCheckLog,30);

    if(uCheckLog[3] == dst_date || uCheckLog[29] == dst_date)
    {
        if((0xBB==uCheckLog[0] && 0x66==uCheckLog[1]) || (0xBB==uCheckLog[26] && 0x66==uCheckLog[27]))
        {
             *uLogDataFlash = uLogDataReadFlash;
             DEBUGOUT("\nDATAFLASH_LOG_TWO\n");
             return result;
        }
    } 
    uLogDataReadFlash = DATAFLASH_LOG_THREE;
    DataFlash_Read(uLogDataReadFlash,uCheckLog,30);
    if(uCheckLog[3] == dst_date || uCheckLog[29] == dst_date)
    {
        if((0xBB==uCheckLog[0] && 0x66==uCheckLog[1]) || (0xBB==uCheckLog[26] && 0x66==uCheckLog[27]))
        {
             *uLogDataFlash = uLogDataReadFlash;
             DEBUGOUT("\nDATAFLASH_LOG_THREE\n");
             return result;
        }
    }else{
        *uLogDataFlash = 0;
    }
    
    return 0;
}

/******************************************************************************
* ��     �ƣ�uGetLastRecordAddr()
* ��     �ܣ�������־��һ�δ洢����ʼ��ַ
* ��ڲ�����     sTime ʱ��  �ṹ��               
* ���ڲ�������
* ��     ��: ��
******************************************************************************/
uint8_t uGetLastRecordAddr(SYSTEMTIME sTime)
{
    uint8_t uSaveCheck[30] = {0};

    if((sLogData.LogFlashAddr==DATAFLASH_LOG_ONE) || (sLogData.LogFlashAddr==DATAFLASH_LOG_TWO) || (sLogData.LogFlashAddr==DATAFLASH_LOG_THREE))
    {
        return 0;
    }
    else if((DATAFLASH_LOG_ONE<sLogData.LogFlashAddr && DATAFLASH_LOG_TWO>sLogData.LogFlashAddr))
    {        
        DataFlash_Read(DATAFLASH_LOG_ONE,uSaveCheck,30);
    }
    else if(DATAFLASH_LOG_TWO<sLogData.LogFlashAddr && DATAFLASH_LOG_THREE>sLogData.LogFlashAddr)
    {
        DataFlash_Read(DATAFLASH_LOG_TWO,uSaveCheck,30);
    }
    else if(DATAFLASH_LOG_THREE<sLogData.LogFlashAddr && DATAFLASH_LOG_END>sLogData.LogFlashAddr)
    {
        DataFlash_Read(DATAFLASH_LOG_THREE,uSaveCheck,30);
    }   
        
    if(sTime.Date==uSaveCheck[3] || sTime.Date==uSaveCheck[29])
    {
        return 0;
    }
/*
    if((DATAFLASH_LOG_ONE<sLogData.LogFlashAddr && DATAFLASH_LOG_TWO>sLogData.LogFlashAddr))
    {        
        DataFlash_Read(sLogData.LogFlashAddr,uSaveCheck,30);
    }else if(DATAFLASH_LOG_TWO<sLogData.LogFlashAddr && DATAFLASH_LOG_THREE>sLogData.LogFlashAddr)
    {
        DataFlash_Read(sLogData.LogFlashAddr,uSaveCheck,30);
    }else if(DATAFLASH_LOG_THREE<sLogData.LogFlashAddr && DATAFLASH_LOG_END>sLogData.LogFlashAddr)
    {
        DataFlash_Read(sLogData.LogFlashAddr,uSaveCheck,30);
    }   
        
    if(sTime.Date==uSaveCheck[3])
    {
        return 0;
    }
*/
    if((DATAFLASH_LOG_ONE<sLogData.LogFlashAddr && DATAFLASH_LOG_TWO>sLogData.LogFlashAddr))
    {
        sLogData.LogFlashAddr = DATAFLASH_LOG_TWO-26;
        DataFlash_Sector_Erase(DATAFLASH_LOG_TWO);
        DataFlash_Sector_Erase(DATAFLASH_LOG_TWO+RECORD_SECTOR_SIZE);
    }
    else if(DATAFLASH_LOG_TWO<sLogData.LogFlashAddr && DATAFLASH_LOG_THREE>sLogData.LogFlashAddr)
    {
        sLogData.LogFlashAddr = DATAFLASH_LOG_THREE-26;
        DataFlash_Sector_Erase(DATAFLASH_LOG_THREE);
        DataFlash_Sector_Erase(DATAFLASH_LOG_THREE+RECORD_SECTOR_SIZE);

    }
    else if(DATAFLASH_LOG_THREE<sLogData.LogFlashAddr && DATAFLASH_LOG_END>sLogData.LogFlashAddr)
    {
        sLogData.LogFlashAddr = DATAFLASH_LOG_ONE-26;
        DataFlash_Sector_Erase(DATAFLASH_LOG_ONE);
        DataFlash_Sector_Erase(DATAFLASH_LOG_ONE+RECORD_SECTOR_SIZE);
    }
    return 0;
}

/******************************************************************************
* ��     �ƣ�SaveLogInit()
* ��     �ܣ���־�洢��ʼ��
* ��ڲ�����     sTime               ʱ��  �ṹ��
*           puSouthLog          ������־��ʼָ��
*           puNorthLog          ������־��ʼָ��
*           uSouthLogPoints     ������־���ݳ���
*           uNorthLogPoints     ������־���ݳ���               
* ���ڲ�������
* ��     ��: ��
******************************************************************************/
uint32_t uSaveSouthLog(SYSTEMTIME sTime,const uint8_t *puSouthLog,const uint8_t *puNorthLog,uint16_t uSouthLogPoints,uint16_t uNorthLogPoints)
{
    uint8_t uHeadMark[6] = {0};
    uint8_t  i;    
    uint8_t  uSectorErase;          // ��Ҫ��������������
    uint16_t uSouthLenAll;          // ������־���ݳ���
    uint16_t uSectorEmpty;          // ����ʣ��Ĵ�С
    uint32_t uWriteFlashAddrStart;  // ��һ�����ݵ�д��ʼ��ַ
    uint32_t uNextSectorAddr;       // ��һ��������ʼ��ַ
    uint32_t uEraseSectorAddr;      // ������������ʼ��ַ
    puNorthLog = puNorthLog;
    uNorthLogPoints = uNorthLogPoints;

    uHeadMark[0] = LOG_HEAD_A;
    uHeadMark[1] = LOG_HEAD_B;

    uHeadMark[2] = sTime.Month;
    uHeadMark[3] = sTime.Date;
    uHeadMark[4] = sTime.Hour;
    uHeadMark[5] = sTime.Minute;

    uGetLastRecordAddr(sTime);   //����־ǰ�ж�Ӧ�ô����ĸ�����

    uSouthLenAll = uSouthLogPoints + 6;   // ����ͷ�� + ʱ�� + ��Ч����
    uWriteFlashAddrStart = sLogData.LogFlashAddr + uSouthLenAll;   // д���ݵ���ʼ��ַ
    FEED_WATCH_DOG();  // ι��
    
    if((uWriteFlashAddrStart+uSouthLenAll > DATAFLASH_LOG_END) || (0==sLogData.uIsExist))  // д��ʼ��ַ��Flash������ַ�ռ䲻����һ֡���ݣ��ص�ͷ��ַд��??
    {
       uWriteFlashAddrStart = DATAFLASH_LOG_ONE;
       uNextSectorAddr      = DATAFLASH_LOG_ONE;
       uSectorEmpty         = 0x00;

       sLogData.LogFlashAddr   = DATAFLASH_LOG_ONE;  // ��֡������һ֡���洢����ʼ��ַ
    }
    else
    {
       uNextSectorAddr = (uWriteFlashAddrStart/RECORD_SECTOR_SIZE + 1)*RECORD_SECTOR_SIZE;  // ������¸���������ʼ��ַ
       uSectorEmpty    = uNextSectorAddr - uWriteFlashAddrStart;   // ���㱾�����Ŀ�������

       sLogData.LogFlashAddr = uWriteFlashAddrStart;  // ��֡������һ֡���洢����ʼ��ַ
    }
    
    DEBUGOUT("#LOG START ADDR:0x%0.6X\r\n",uWriteFlashAddrStart);  // ��֡����ʼ��ַ

   if(uSouthLenAll > uSectorEmpty)
   {
       uSectorErase = (uSouthLenAll - uSectorEmpty) / RECORD_SECTOR_SIZE;
       if((uSouthLenAll - uSectorEmpty) % RECORD_SECTOR_SIZE)
       {
           uSectorErase++;
       }
       
       for(i=0;i<uSectorErase;i++)
       {
           FEED_WATCH_DOG();  // ι��
       
           uEraseSectorAddr = uNextSectorAddr + RECORD_SECTOR_SIZE * i;
       
           if(uEraseSectorAddr > DATAFLASH_LOG_END)   // ����������������оƬ������������ʼ������??
           {
               uEraseSectorAddr = uEraseSectorAddr - (DATAFLASH_LOG_END + 1) + DATAFLASH_LOG_ONE;
           }
       
           //DataFlash_Sector_Erase(uEraseSectorAddr);   // ��������
       }
   }
       
   //---------------------------------------------------------------------------------------------
   if((uWriteFlashAddrStart+uSouthLenAll) < DATAFLASH_LOG_END)  // ��Flash������ַǰ����д��������
   {
       // �洢��DataFlash��������
       DataFlash_Write(uWriteFlashAddrStart,(uint8_t *)uHeadMark,6);            // �洢ͷ���ʱ��
       
       FEED_WATCH_DOG();  // ι��
       uWriteFlashAddrStart += 6;
       DataFlash_Write(uWriteFlashAddrStart,(uint8_t *)puSouthLog,uSouthLogPoints);   // �洢������־
   }

   sLogData.uIsExist = 1;
   // �洢
   EepSavedata(EEP_LOGGER_LOG_LADDR_HEAD,(uint8_t *)&sLogData,sizeof(sLogData),&sLogData.LogCRC);
   
   //============================================================================================
   DEBUGOUT("#LOG END ADDR:0x%0.6X\r\n",sLogData.LogFlashAddr + uSouthLenAll); // ��֡�Ľ�����ַ
   memset(SOUTH_DATA_LOG,0,20);
           
   return 0;
}

/******************************************************************************
* ��     �ƣ�SaveLogInit()
* ��     �ܣ���־�洢��ʼ��
* ��ڲ�����     uReset  1:��ʼ���ָ���������            0:������ʼ��               
* ���ڲ�������
* ��     ��: ��
******************************************************************************/
void SaveLogInit(uint8_t uReset)
{
    uint8_t uRes;
    uint8_t uSaveCheck[6] = {0};
    uint32_t uLogDataSaveFlash;

    SOUTH_DATA_LOG = WMemMalloc(SOUTH_DATA_LOG,20);  //10̨�豸�����豸��źͷ��ع�����

    if(NULL==SOUTH_DATA_LOG)
    {
        DEBUGOUT("������־�ռ�ʧ��!\r\n");
        return;
    }
    for(int i=0;i<20;i++)
    {
        SOUTH_DATA_LOG[i] = 0xFF;
    }

    if(uReset)
    {
        sLogData.LogFlashAddr = DATAFLASH_LOG_ONE;
        sLogData.uIsExist     = 0;
        DataFlash_Sector_Erase(DATAFLASH_LOG_ONE);
        DataFlash_Sector_Erase(DATAFLASH_LOG_ONE+RECORD_SECTOR_SIZE);

        EepSavedata(EEP_LOGGER_LOG_LADDR_HEAD,(uint8_t *)&sLogData,sizeof(sLogData),&sLogData.LogCRC);
    }
    else
    {   
    
        uRes = EepReadData(EEP_LOGGER_LOG_LADDR_HEAD,(uint8_t *)&sLogData,sizeof(sLogData),&sLogData.LogCRC);// ���ɶ�ȡ
        if(0==uRes) // ��ȡʧ�ܣ��ֶ��������һ����ַ
        {
            
            DataFlash_Read(DATAFLASH_LOG_ONE,uSaveCheck,6);
            if(uSaveCheck[3] == sDateTime.Date)
            {
                if(0xBB==uSaveCheck[0] && 0x66==uSaveCheck[1])
                {
                    sLogData.LogFlashAddr = DATAFLASH_LOG_ONE+(sDateTime.Hour*12+sDateTime.Minute/5)*26;
                }
            }
            
            FEED_WATCH_DOG();  // ι��
            DataFlash_Read(DATAFLASH_LOG_TWO,uSaveCheck,6);
            if(uSaveCheck[3] == sDateTime.Date)
            {
                if(0xBB==uSaveCheck[0] && 0x66==uSaveCheck[1])
                {
                    sLogData.LogFlashAddr = DATAFLASH_LOG_TWO+(sDateTime.Hour*12+sDateTime.Minute/5)*26;
                }
            }
            
            FEED_WATCH_DOG();  // ι��
            DataFlash_Read(DATAFLASH_LOG_THREE,uSaveCheck,6);
            if(uSaveCheck[3] == sDateTime.Date)
            {
                if(0xBB==uSaveCheck[0] && 0x66==uSaveCheck[1])
                {
                    sLogData.LogFlashAddr = DATAFLASH_LOG_THREE+(sDateTime.Hour*12+sDateTime.Minute/5)*26;
                }
            }
            
            sLogData.uIsExist = 1;        
            EepSavedata(EEP_LOGGER_LOG_LADDR_HEAD,(uint8_t *)&sLogData,sizeof(sLogData),&sLogData.LogCRC);
            return;
        }

        uLogDataSaveFlash = sLogData.LogFlashAddr;
        DataFlash_Read(uLogDataSaveFlash,uSaveCheck,6);
        if(uSaveCheck[3] == sDateTime.Date)
        {
            if(0xBB==uSaveCheck[0] && 0x66==uSaveCheck[1])
            {
                return;
            }
        }
    }
    return;
}
