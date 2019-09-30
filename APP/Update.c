#include "chip.h"
//#include "flash.h"

#include "InEeprom.h"
#include "DataFlash.h"
#include "Update.h"

#include "CRC16.h"
#include "WatchDog.h"
#include "GlobalVar.h"
#include "SoftTimer.h"
#include "DataFlash.h"
//=======================================================
#define UPDATA_UART_USE           uart3         // ʹ�õĴ���
#define UPDATA_UART_USE_STATE     UART3_STATE   // ����״̬

//=======================================================

#define SOURCE_CODE   0x00028000   // �洢���������ROM��ַ
//#define TARGET_CODE   0x00006000   // Ŀ�����г����ROM��ַ

#define START_Sector  26            // Ŀ�����г�����ʼ����
#define END_Sector    28            // Ŀ�����г�����ʼ����
//=======================================================


//=======================================================


//=======================================================
//=======================================================
#define SIDE_A      0x00006000
#define SIDE_B      0x00028000
//=======================================================

/****************************************************************************
* ��    �ƣ�Updata()
* ��    �ܣ������������ݽ��ձ��浽DataFlash��
* ��ڲ�����
            *puData   ����ָ��
            uLen      ���ݳ���
*
* ���ڲ�����CRCУ����󷵻�1�����Ȳ��Է���2����ȷ����0
* ��    ��: ��
****************************************************************************/
uint8_t Update(void)
{
#if(0)
    uint8_t  uDataTemp;

    uint16_t uLen;
    uint16_t uCrc;
    uint16_t uCrcRead;
    uint16_t uRecSeq;                 // ���յ���֡���
    static  uint16_t uFrameCount=0;   // ����֡����
    uint8_t  uaRecData[262];

    UPDATA_MARK_T sEepUpdataMes;
    SOFT_TIMER_T  s_sUpdataTimerDelay;


    uDataTemp = GetRunUpdataMode();

    if(0==UPDATA_UART_USE_STATE.Rec_OK && (0x1A!=uDataTemp) && (0x2A!=uDataTemp))
    {
        return 3;
    }

    UPDATA_UART_USE_STATE.Rec_OK = 0;

    uDataTemp = GetStoreAddr();   // ��ȡͨѶ��ַ

    uLen = UART_GetRxCount(UPDATA_UART_USE);

    if((uLen<258 && 0==uFrameCount) || (uLen<262 && 0!=uFrameCount))
    {
        uFrameCount = 0x00;

        Reboot();  // ��λоƬ
    }

    if(0!=uFrameCount)
    {
        uLen = UARTn_Read(UPDATA_UART_USE,uaRecData,262);
    }
    else
    {
        uaRecData[0] = uDataTemp;
        uaRecData[1] = GetRunUpdataMode();

        SetRunUpdataMode(1); // ����Ϊ����ģʽ

        //--------------------------------
        // ����DataFlash
        // DataFlash�洢�������������������96KB��һ������+8������
        FEED_DOG();
        DataFlash_Block_Erase(0);// ������0
        DataFlash_Sector_Erase(0x010000);
        DataFlash_Sector_Erase(0x011000);
        DataFlash_Sector_Erase(0x012000);
        FEED_DOG();
        DataFlash_Sector_Erase(0x013000);
        DataFlash_Sector_Erase(0x014000);
        DataFlash_Sector_Erase(0x015000);
        DataFlash_Sector_Erase(0x016000);
        DataFlash_Sector_Erase(0x017000);

        uaRecData[2] = 0x00;
        uaRecData[3] = 0x00;

        FEED_DOG();

        uLen = UARTn_Read(UPDATA_UART_USE,&uaRecData[4],258);
    }

    if(uDataTemp!=uaRecData[0] || (0x1A!=uaRecData[1] && 0x2A!=uaRecData[1]))
    {
        // ͷ�������������������,���������־
        //received_all = 0;
        uFrameCount = 0;
        return 0;
    }

    uCrc     = CRC16 (uaRecData,260);
    uCrcRead = uaRecData[260]<<8 | uaRecData[261];

    if(uCrc!=uCrcRead)
    {
        // ����У��������������������,���������־
        uaRecData[4] = 0x00;
        uaRecData[5] = 0x55;  // У��ʧ��
        uCrc = CRC16(uaRecData,6);

        uaRecData[6] = uCrc>>8;
        uaRecData[7] = uCrc&0xFF;

        //DEBUGOUT("Updata CRC ERR\r\n");
        UARTn_Send(UPDATA_UART_USE,uaRecData,8);
        return 1;
    }

    uRecSeq = (uaRecData[2] <<8) | uaRecData[3];

    if(uFrameCount!=(uRecSeq&0x7FFF))
    {
        uaRecData[2] = (uFrameCount&0x7FFF)>>8;
        uaRecData[3] = uFrameCount&0xFF;  // ��Ҫ�ش���֡

        uaRecData[4] = 0x00;
        uaRecData[5] = 0x01;  // ����֡��Ų���
        uCrc = CRC16(uaRecData,6);

        uaRecData[6] = uCrc>>8;
        uaRecData[7] = uCrc&0xFF;

        //DEBUGOUT("Updata Seq ERR\r\n");
        UARTn_Send(UPDATA_UART_USE,uaRecData,8);
        return 2;
    }

    if(0==uFrameCount)
    {
        // ����DataFlash
    }

    // �洢��DataFlash������������
    DataFlash_Write(uFrameCount * 256,(uint8_t *)&uaRecData[4],256);
    //DataFlash_Read(uFrameCount * 256,(uint8_t *)&uaRecData[4],256);

    //DataFlash_Read(uFrameCount * 256,(uint8_t *)&uaRecData[0],256);

    //DataFlash_Read(0,(uint8_t *)&uaRecData[0],256);

    uFrameCount++;

    if(uRecSeq&0x8000) // ���һ֡�����ݽ������//if(0xEDAA==((RECdata[2] <<8) | RECdata[3])) // ���һ֡�����ݽ������
    {
        uCrc = EepReadData(EEPROM_UPDATA_MARK_ADDRESS,(uint8_t *)&sEepUpdataMes,sizeof(sEepUpdataMes),&sEepUpdataMes.CRC);

        if(0==uCrc)
        {
            sEepUpdataMes.rollback_allow = 0x55;
        }

        sEepUpdataMes.frame_sum = uFrameCount;// 256���ֽ�һ֡����֡��

        sEepUpdataMes.b_version[0] = 0x02;   // ��ǰ�汾1
        sEepUpdataMes.b_version[1] = 0x02;   // ��ǰ�汾1
        sEepUpdataMes.b_version[2] = 0x02;   // ��ǰ�汾1

        sEepUpdataMes.reserve = 0x00;        // Ԥ����ռλ

        if(2==(uaRecData[1]>>4))
        {
            sEepUpdataMes.side_tobe = 0xBB;  // ������B��
        }
        else
        {
            sEepUpdataMes.side_tobe = 0xAA;  // ������A��
        }


        if(!uFrameCount)  // û������������
        {
            sEepUpdataMes.updata_mark = 0x55;  //  data[2] = 0x55;    // 0xAA������0xBB�ع���0x55��
        }
        else
        {
            sEepUpdataMes.updata_mark = 0xAA;  // data[2] = 0xAA;    // 0xAA������0xBB�ع���0x55��
        }

        EepSavedata(EEPROM_UPDATA_MARK_ADDRESS,(uint8_t *)&sEepUpdataMes,sizeof(sEepUpdataMes),&sEepUpdataMes.CRC);// �洢

        //DEBUGOUT("UPDATA REC END,REBOOT\r\n");

        //uSide = 0;
        uFrameCount = 0;

        SoftTimerSet(&s_sUpdataTimerDelay,100,SOFT_TIMER_START);
    }

    //--------------------------------------------------------------------
    uaRecData[4] = 0x00;
    uaRecData[5] = 0xAA;  // ������ȷ
    uCrc = CRC16(uaRecData,6);

    uaRecData[6] = uCrc>>8;
    uaRecData[7] = uCrc&0xFF;

    UPDATA_UART_USE_STATE.Send_OK = 0;
    UARTn_Send(UPDATA_UART_USE,uaRecData,8);


    if(uRecSeq&0x8000)
    {
        while(1)
        {
            if(SoftTimerExpired(&s_sUpdataTimerDelay))
            {
                Reboot();
            }
        }
    }
    //--------------------------------------------------------------------
#endif
    return 2;

}
