
#include "Record.h"

#include "GlobalVar.h"
#include "DataFlash.h"
#include "RealTime.h"
#include "InEEPROM.h"
#include "WatchDog.h"
#include "ucos_ii.h"
#include "time.h"
//========================================================================
//#define  DATAFLASH_RECORD_HEAD        0x01E000     // ��ʷ���ݴ����DataFlash����ʼ��ַ
//#define  DATAFLASH_RECORD_END         0x7FFFFF     // ��ʷ���ݴ����DataFlash�Ľ�����ַ

//========================================================================
#define FEED_WATCH_DOG()   FEED_DOG()    // ι��

#define RECORD_HEAD_A  0xAA
#define RECORD_HEAD_B  0x55

#define RECORD_SECTOR_SIZE 4096  // 4kB  DataFlashһ������Ϊ4KB
/******************************************************************************
* ��������
******************************************************************************/
uint8_t g_uSaveRecord=0;     // �洢��ʷ���������յ���ʱ֮������洢����⵽���������洢
RECORD_STRUCT_T sRecord={0}; // ��¼�洢�������һ����DataFlash

/******************************************************************************
* ��    �ƣ�SetRecordAllow()
* ��    �ܣ����ݼ�¼����
* ��ڲ�����
*           uAllow        1�������¼����   0���������¼����

* ���ڲ�������
* ��    ��:
******************************************************************************/
void SetRecordAllow(uint8_t uAllow)
{
    g_uSaveRecord = uAllow;
}

/******************************************************************************
* ��    �ƣ�GetRecordAllow()
* ��    �ܣ���ȡ���ݼ�¼����
* ��ڲ�����
*           uAllow        1�������¼����   0���������¼����

* ���ڲ�������
* ��    ��:
******************************************************************************/
uint8_t GetRecordAllow(void)
{
    return g_uSaveRecord;
}
/******************************************************************************
* ��    �ƣ�GetRecordExist()
* ��    �ܣ���ȡ�Ƿ��м�ʱ���ݼ�¼��
* ��ڲ�����
*           ��

* ���ڲ�����1������ʷ���ݣ�0������ʷ����
* ��    ��:
******************************************************************************/
uint8_t GetRecordExist(void)
{
    return sRecord.uExist;
}

/******************************************************************************
* ��    �ƣ�TimeToLong()
* ��    �ܣ���ʱ��תΪ��1970�꿪ʼ�ļ���
* ��ڲ�����
*           uType           ���ͣ�1����ʷ���ݣ�2����־
            uDataLen        һ�����ݳ���

* ���ڲ��������һ����¼�Ĵ洢��ַ
* ��    ��:
******************************************************************************/
time_t TimeToLong(uint8_t *UTC)
{
    time_t uTime;
    struct tm sT;
    /*struct tm {
        int tm_sec;     // seconds after the minute - [0,59]
        int tm_min;     // minutes after the hour - [0,59]
        int tm_hour;    // hours since midnight - [0,23]
        int tm_mday;    // day of the month - [1,31]
        int tm_mon;     // months since January - [0,11]
        int tm_year;    // years since 1900
        int tm_wday;    // days since Sunday - [0,6]
        int tm_yday;    // days since January 1 - [0,365]
        int tm_isdst;   // daylight savings time flag
       };

        ���Կ�������������������ꡢ�¡��ա�ʱ���֡��롢���ڡ������е�ĳһ�졢����ʱ������������ṹ�ܷ������ʾʱ�䡣
    */

    sT.tm_year = UTC[0]+100;   // ϵͳ�����������1900��ʼ
    sT.tm_mon  = UTC[1] - 1;
    sT.tm_mday = UTC[2] & 0x1F;
    sT.tm_hour = UTC[3];
    sT.tm_min  = UTC[4];
    sT.tm_sec  = UTC[5];
    sT.tm_isdst = 0;   //��ʵ������ʱ

    uTime = mktime(&sT);

    return uTime;
}

/******************************************************************************
* ��    �ƣ�SearchRecordLastAddr()
* ��    �ܣ��������һ����¼�Ĵ洢��ַ��
* ��ڲ�����
*           uType           ���ͣ�1����ʷ���ݣ�2����־
            uDataLen        һ�����ݳ���

* ���ڲ��������һ����¼�Ĵ洢��ַ
* ��    ��:
******************************************************************************/
uint32_t SearchRecordLastAddr(uint8_t uType,uint16_t uDataLen)
{
    uint8_t  uaMark[8];
    uint32_t uFlashAddr=0;  // ���ݵ�д��ַ
    uint32_t uFlashAddrTemp;  // ���ݵ�д��ַ
    uint32_t uFlashEnd;

    time_t uTime1;
    time_t uTime2;

    if(1==uType)
    {
        uFlashAddrTemp = DATAFLASH_RECORD_HEAD;

        DataFlash_Read(uFlashAddrTemp,(uint8_t*)uaMark,8);
        if(RECORD_HEAD_A==uaMark[0] && RECORD_HEAD_B==uaMark[1])
        {
            uTime1 = TimeToLong(&uaMark[2]);
        }

        uFlashEnd = DATAFLASH_RECORD_END - uDataLen + 1; // ��ӽ��������ݵ�����֡��ַ
        uFlashAddrTemp += uDataLen;
        do
        {
            DataFlash_Read(uFlashAddrTemp,(uint8_t*)uaMark,8);
            if(RECORD_HEAD_A==uaMark[0] && RECORD_HEAD_B==uaMark[1])
            {
                uTime2 = TimeToLong(&uaMark[2]);

                if(uTime2 > uTime1)
                {
                    uTime1 = uTime2;
                    uFlashAddr = uFlashAddrTemp;
                }
            }
            uFlashAddrTemp += uDataLen;
        }while(uFlashAddrTemp<=uFlashEnd);
    }
    return uFlashAddr;
}
/******************************************************************************
* ��    �ƣ�RecordAllow()
* ��    �ܣ����ݼ�¼����
* ��ڲ�����
*           uAllow        1�������¼����   0���������¼����
            uReset        1����ʼ���ָ���������    0��������ʼ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
void RecordInit(uint8_t uReset)
{
    uint8_t uRes;

    if(uReset)
    {
        sRecord.uFlashAddr = DATAFLASH_RECORD_HEAD;
        sRecord.uExist     = 0;
        //sRecord.uLen       = g_DeviceSouth.yc_sum * 4 + g_DeviceSouth.yx_sum;  // ң���������ÿ����ռ��4���ֽڣ�ң�ŵ�������ÿ����ռ��1���ֽ�
        DataFlash_Sector_Erase(DATAFLASH_RECORD_HEAD);   // ��������

        EepSavedata(EEP_LOGGER_104_RECORD_HEAD,(uint8_t *)&sRecord,sizeof(sRecord),&sRecord.CRC);
    }
    else
    {
        uRes = EepReadData(EEP_LOGGER_104_RECORD_HEAD,(uint8_t *)&sRecord,sizeof(sRecord),&sRecord.CRC);// ���ɶ�ȡ

        if(EEP_ERR==uRes) // ��ȡʧ�ܣ��ֶ��������һ����ַ
        {
            if(0!=g_DeviceSouth.yx_sum + g_DeviceSouth.yc_sum*4)
            {
                sRecord.uFlashAddr = SearchRecordLastAddr(1,g_DeviceSouth.yx_sum + g_DeviceSouth.yc_sum*4 + 8);
            }
            else
            {
                sRecord.uFlashAddr = 0;
            }

            if(0==sRecord.uFlashAddr)
            {
                sRecord.uFlashAddr = DATAFLASH_RECORD_HEAD;
                sRecord.uExist = 0;
            }
            EepSavedata(EEP_LOGGER_104_RECORD_HEAD,(uint8_t *)&sRecord,sizeof(sRecord),&sRecord.CRC);
        }
    }
}
//========================================================================
/******************************************************************************
* ��    �ƣ�RecordHistory()
* ��    �ܣ����ݴ洢��
* ��ڲ�����
            *puYxData           ң��������ʼָ��
            *puYcData           ң��������ʼָ��
            uYxPoint            ң�ŵ���
            uYcPoint            ң�����

* ���ڲ�������
* ��    ��:
******************************************************************************/
uint8_t RecordHistory(const uint8_t *puYxData,const uint8_t *puYcData,const uint16_t uYxPoint,const uint16_t uYcPoint)
{
    uint8_t  uaMark[8];
    //uint8_t  check2[8]={2};
    uint8_t  i,result;
    uint8_t  uSectorErase;          // ��Ҫ��������������
    uint16_t uLenAll;               // �����ݳ���
    uint16_t uSectorEmpty;          // ����ʣ��Ĵ�С
    uint32_t uWriteFlashAddrStart;  // ��һ�����ݵ�д��ʼ��ַ
    uint32_t uNextSectorAddr;       // ��һ��������ʼ��ַ
    uint32_t uEraseSectorAddr;      // ������������ʼ��ַ

    SYSTEMTIME *pTime;

    pTime = RealTimeGet();

    uaMark[0] = RECORD_HEAD_A;
    uaMark[1] = RECORD_HEAD_B;

    uaMark[2] = pTime->Year;
    uaMark[3] = pTime->Month;
    uaMark[4] = pTime->Week<<5 | pTime->Date;     // ��3λΪ����
    uaMark[5] = pTime->Hour;
    uaMark[6] = pTime->Minute;
    uaMark[7] = pTime->Second;

    /*if(DATAFLASH_RECORD_HEAD == uRec.uFlashAddr)
    {
        DataFlash_Sector_Erase(DATAFLASH_RECORD_HEAD);
    }*/
    //-------------------------------------------------------------------------------------
    uLenAll = uYxPoint + uYcPoint*4 + 8;   // ����ͷ��ʶ + ʱ�� + ��Ч����

    uWriteFlashAddrStart = sRecord.uFlashAddr + uLenAll;   // д���ݵ���ʼ��ַ

    if((uWriteFlashAddrStart+uLenAll > DATAFLASH_RECORD_END) || (0==sRecord.uExist))  // д��ʼ��ַ��Flash������ַ�ռ䲻����һ֡���ݣ��ص�ͷ��ַд����
    {
        uWriteFlashAddrStart = DATAFLASH_RECORD_HEAD;
        uNextSectorAddr      = DATAFLASH_RECORD_HEAD;
        uSectorEmpty         = 0x00;

        sRecord.uFlashAddr   = DATAFLASH_RECORD_HEAD;  // ��֡������һ֡���洢����ʼ��ַ
    }
    else
    {
        uNextSectorAddr = (uWriteFlashAddrStart/RECORD_SECTOR_SIZE + 1)*RECORD_SECTOR_SIZE;  // ������¸���������ʼ��ַ
        uSectorEmpty    = uNextSectorAddr - uWriteFlashAddrStart;   // ���㱾�����Ŀ�������

        sRecord.uFlashAddr = uWriteFlashAddrStart;  // ��֡������һ֡���洢����ʼ��ַ
    }

    DEBUGOUT("#RECORD START ADDR:0x%0.6X\r\n",uWriteFlashAddrStart);  // ��֡����ʼ��ַ

    if(uLenAll > uSectorEmpty)
    {
        uSectorErase = (uLenAll - uSectorEmpty) / RECORD_SECTOR_SIZE;
        if((uLenAll - uSectorEmpty) % RECORD_SECTOR_SIZE)
        {
            uSectorErase++;
        }
        FEED_WATCH_DOG();  // ι��

        for(i=0;i<uSectorErase;i++)
        {
            uEraseSectorAddr = uNextSectorAddr + RECORD_SECTOR_SIZE * i;

            if(uEraseSectorAddr > DATAFLASH_RECORD_END)   // ����������������оƬ������������ʼ��������
            {
                uEraseSectorAddr = uEraseSectorAddr - (DATAFLASH_RECORD_END + 1) + DATAFLASH_RECORD_HEAD;
            }

            result = DataFlash_Sector_Erase(uEraseSectorAddr);   // ��������
            if(!result)
            {
                DEBUGOUT("flash uEraseSectorAddr failed at��0x%0.6X !\n",uEraseSectorAddr);
                result = DataFlash_Sector_Erase(uEraseSectorAddr);   // ��������
            }

            DEBUGOUT("flash uEraseSectorAddr at��0x%0.6X !\n",uEraseSectorAddr);
        }
    }

    //---------------------------------------------------------------------------------------------
    if((uWriteFlashAddrStart+uLenAll) < DATAFLASH_RECORD_END)  // ��Flash������ַǰ����д������
    {
        // �洢��DataFlash��������
        DataFlash_Write(uWriteFlashAddrStart,(uint8_t *)uaMark,8);            // �洢ͷ+ʱ��
        DEBUGOUT("#RECORD TIME:%d-%d-%d %d:%d:%d\n",pTime->Year,pTime->Month,pTime->Date,pTime->Hour,pTime->Minute,pTime->Second);
        OSTimeDly(5);

        FEED_WATCH_DOG();  // ι��
        //DataFlash_Read(uWriteFlashAddrStart, check2, 8);
        /*if(0x01==SouthSwich)
        {
            DEBUGOUT("#AFTER RECORD :");
            for(int i=0;i<8;i++)
            {
                DEBUGOUT("%d ",check2[i]);
            }
			DEBUGOUT("\r\n");
        }*/

        uWriteFlashAddrStart += 8;
        DataFlash_Write(uWriteFlashAddrStart,(uint8_t *)puYxData,uYxPoint);   // �洢ң�ŵ�

        FEED_WATCH_DOG();  // ι��

        uWriteFlashAddrStart += uYxPoint;
        DataFlash_Write(uWriteFlashAddrStart,(uint8_t *)puYcData,uYcPoint*4); // �洢ң���
    }
    
    sRecord.uExist = 1;
    // �洢
    EepSavedata(EEP_LOGGER_104_RECORD_HEAD,(uint8_t *)&sRecord,sizeof(sRecord),&sRecord.CRC);
    //============================================================================================
    DEBUGOUT("$RECORD END ADDR:0x%0.6X\r\n",sRecord.uFlashAddr + uLenAll); // ��֡�Ľ�����ַ
    return 0;
}









