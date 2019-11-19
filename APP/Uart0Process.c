#include "Uart0process.h"


#include "stdio.h"
#include <stdarg.h>  

#include "DataFlash.h"
#include "GlobalVar.h"
#include "InEeprom.h"
#include "string.h"
#include "CRC16.h"
#include "WatchDog.h"
#include "Usart.h"
#include "tool.h"

#include "DataTransferArea.h"
#include "Memory.h"
#include "RealTime.h"

#include "ucos_ii.h"
#include "IEC104.h"

#include "ModbusMaster.h"

//=======================================================
#define UPDATE_UART_USE           uart3         // ʹ�õĴ���
#define CM_UART_USE               uart4         // ʹ�õĴ���
//=======================================================
#define JUDGE(x)  ((x)==0?'N':'Y')  // �ж���ֵ�Ƿ�Ϊ0��Ϊ0����0����0����1
//=======================================================
//====================================================================
#define msleep(x)  OSTimeDly((x)/10+1)                // usleep((x)*1000)
#define sleep(x)   OSTimeDly(OS_TICKS_PER_SEC*(x)+1)  // ��ʱx��
//====================================================================
//====================================================================
#define SIZE 128
//static uint8_t g_TestData[SIZE];
//static uint8_t g_TestOut[4096];

static uint8_t g_uRecData[200];
static uint8_t *g_uUpdateBuffer=NULL;
//=======================================================

//=======================================================
OS_STK TaskUart0ProcesStk[TASKUART0STKSIZE]@0x20004400;	      //���������ջ��С
//=======================================================
//=======================================================
void DealCmd(char *uRec,uint16_t uLen);
uint8_t UartUpdate(uint8_t *pUpdteBuffer);
//=======================================================
/*
#if(1==USE_485_DEBUG)
static uint8_t s_uRs485Debug=0;
void Rs485DebugMark(void)
{
    s_uRs485Debug = 1;
}
uint8_t Rs485DebugRead(void)
{
    return s_uRs485Debug;
}
void Rs485DebugClear(void)
{
    s_uRs485Debug = 0;
}

    #define WDEBUGOUT(...) \
    do{\
        if(s_uRs485Debug)\
        {\
            memset(g_uRecData,0,100);\
            sprintf((char*)g_uRecData,__VA_ARGS__);\
            UartWrite(UPDATE_UART_USE,g_uRecData,strlen((char*)g_uRecData));\
        }\
        printf(__VA_ARGS__);\
        while(UartTxLen(UPDATE_UART_USE)>0 && s_uRs485Debug);\
    }while(0)
#else
#define WDEBUGOUT(...) printf(__VA_ARGS__)
#endif
*/
        
#if(1==USE_485_DEBUG)
static uint8_t s_uRs485Debug=0;
void Rs485DebugMark(void)
{
    s_uRs485Debug = 1;
}
uint8_t Rs485DebugRead(void)
{
    return s_uRs485Debug;
}
void Rs485DebugClear(void)
{
    s_uRs485Debug = 0;
}

void WDEBUGOUT(const char *cmd, ...)
{
    va_list args;       //����һ��va_list���͵ı������������浥������ 
    
    if(s_uRs485Debug)
    {
        memset(g_uRecData,0,100);
        
        va_start(args,cmd); //ʹargsָ��ɱ�����ĵ�һ������  
        vsprintf((char*)g_uRecData,cmd,args);  //������vprintf�ȴ�V��  
        va_end(args);       //�����ɱ�����Ļ�ȡ
        UartWrite(UPDATE_UART_USE,g_uRecData,strlen((char*)g_uRecData));
    }
    
    va_start(args,cmd); //ʹargsָ��ɱ�����ĵ�һ������  
    vprintf(cmd,args);  //������vprintf�ȴ�V��  
    va_end(args);       //�����ɱ�����Ļ�ȡ
    
    while(UartTxLen(UPDATE_UART_USE)>0 && s_uRs485Debug);
}
#else
#define WDEBUGOUT(...) printf(__VA_ARGS__)
#endif
/****************************************************************************
* ��    �ƣ�PrintThisInfo()
* ��    �ܣ���ӡ������Ϣ��
* ��ڲ�����
*           ��

* ���ڲ�������
* ��    ��: ��
****************************************************************************/
void PrintThisInfo(void)
{
    SYSTEMTIME *pGetTime;
    pGetTime = RealTimeGet();

    WDEBUGOUT("\nʱ��  :%d-%d-%d %d:%d:%d-%d\n",pGetTime->Year,pGetTime->Month,pGetTime->Date,pGetTime->Hour,pGetTime->Minute,pGetTime->Second,pGetTime->Week);
    WDEBUGOUT("�汾  :%02X%02X%02X-%04X\n",GetVerType(),GetVerS1(),GetVerS2(),GetVerS3());
    WDEBUGOUT("ESN   :%-20.20s\n",g_LoggerInfo.esn);//WDEBUGOUT("ESN    :%-20.20s\r\n",g_LoggerInfo.esn);
    WDEBUGOUT("����  :%-20.20s\n",g_LoggerInfo.name);
    WDEBUGOUT("�ͺ�  :%-20.20s\n",g_LoggerInfo.model);
    WDEBUGOUT("����  :%-20.20s\n",g_LoggerInfo.type);
    WDEBUGOUT("IP    :%-0.32s\n",g_LoggerInfo.server_domain);
    WDEBUGOUT("�˿�  :%d\n",g_LoggerInfo.server_port);
    WDEBUGOUT("����  :%-11.11s\n",g_LoggerInfo.phonenum);
    WDEBUGOUT("����  :%d\n",g_LoggerRun.uCSQ);
    //WDEBUGOUT("����:%d\n",g_PerSetTableResq);
    WDEBUGOUT("���   :%ds\n",g_LoggerInfo.inquire_interval_time);

    if(GetVerS2()&0x01)  // С�汾�����λ��1��������A�棻0��������B��
    {
        WDEBUGOUT("A��-");
    }
    else
    {
        WDEBUGOUT("B��-");
    }
    /*
    RUNNING_ENPTY          = 0,    // �£�δ��վ
    RUNNING_SEARCH_HW      = 1,    // ������Ϊ�豸
    RUNNING_SEARCH_END     = 2,    // ������Ϊ�豸����
    RUNNING_INPUT_START    = 3,    // ��������C5
    RUNNING_INPUT_SOUTH    = 4,    // ���������豸��ϢC4 92
    RUNNING_INPUT_GOLB     = 5,    // ����ȫ����ϢBB 88
    RUNNING_INPUT_TABLE    = 6,    // ������BB 89
    RUNNING_INPUT_104      = 7,    // �����豸104����ϢBB 8A
    RUNNING_WORK_READ      = 10,   // �Ѿ���վ��ɣ���������
    */
    switch(g_LoggerRun.run_status)
    {
    case RUNNING_EMPTY:
        WDEBUGOUT("������\n");
        break;

    case RUNNING_SEARCH_HW:
        WDEBUGOUT("�����豸\n");
        break;

    case RUNNING_SEARCH_END:
        WDEBUGOUT("��������\n");
        break;

    case RUNNING_INPUT_START:
        WDEBUGOUT("��������C5\n");
        break;

    case RUNNING_INPUT_SOUTH:
        WDEBUGOUT("���豸��ϢC492\n");
        break;

    case RUNNING_INPUT_GOLB:
        WDEBUGOUT("��ȫ����ϢBB88\n");
        break;

    case RUNNING_INPUT_TABLE:
        WDEBUGOUT("�����BB89\n");
        break;

    case RUNNING_INPUT_104:
        WDEBUGOUT("����ַ����BB8A\n");
        break;

    case RUNNING_WORK_READ:
        WDEBUGOUT("������\n");
        break;
    }

    WDEBUGOUT("����:");
    switch(g_LoggerRun.north_status)
    {
    case NORTH_DISCON:
        WDEBUGOUT("����\n");
        break;

    case NORTH_CMERR:
        WDEBUGOUT("ģ��δ����>\n");
        break;

    case NORTH_RDY:
        WDEBUGOUT("ģ��׼����\n");
        break;

    case NORTH_CONNECT:
        WDEBUGOUT("ģ��������\n");
        break;

    case NORTH_POWERDOWN:
        WDEBUGOUT("ģ��ػ�\n");
        break;

    case NORTH_OK:
        WDEBUGOUT("OK\n");
        break;

    default:
        WDEBUGOUT("����\n");
        break;
    }
}
/****************************************************************************
* ��    �ƣ�TastUart0Process()
* ��    �ܣ�����0��������
* ��ڲ�����
* ���ڲ�������
* ��    ��: ��
****************************************************************************/
void TaskUart0Process(void *p)
{
    p = p;

    uint16_t iResult;
    uint32_t uEnter485Time=0;
    uint32_t uNowTime=0;

    while(1)
    {

		iResult = UartRxLen(uart0);   // ��ȡ���ڻ��������ֽ�����
        if(iResult<=0) // û�н��յ�����
        {
            msleep(20);
        }
        else
        {
            iResult = UartRead(uart0, g_uRecData, 64, 2);
           /* DEBUGOUT("iResult:%d\r\n",iResult);
            DEBUGOUT("g_uRecData");
            for(uint8_t i=0;i<iResult;i++)
            {
               DEBUGOUT("%02x ",g_uRecData[i]);
            }
            DEBUGOUT("\r\n");*/
            
            if(iResult>0)
            {
                
              DealCmd((char *)g_uRecData,iResult);	
            }

        }


        #if(1==USE_485_DEBUG)
        if(Rs485DebugRead())
        {
            uNowTime = OSTimeGet();
   
            if(TIMEOUT==TimeOut(uEnter485Time,uNowTime,180000))  // ����485���Ժ�3����û�з���ָ��
            {
                Reboot();
            }
        }

        if(0==g_LoggerRun.update)
        {
            iResult = UartRxLen(UPDATE_UART_USE);   // ��ȡ���ڻ��������ֽ�����

            if(iResult>1)
            {
                UartShowBytes(UPDATE_UART_USE, g_uRecData, 2);
                if('A'==g_uRecData[0] && 'A'==g_uRecData[1])
                {
                    iResult = UartRead(UPDATE_UART_USE, g_uRecData, 64, 2);
                    if(iResult<4)
                    {
                        UartClearRecBuffer(UPDATE_UART_USE);  // ��ջ���
                    }
                    else
                    {
                        Rs485DebugMark();  // ��ǽ���Rs485����ģʽ
                        DealCmd((char *)g_uRecData,iResult);

                        uEnter485Time = uNowTime;
                    }
                }
                else
                {
                    if(Rs485DebugRead())
                    {
                        UartClearRecBuffer(UPDATE_UART_USE);
                    }
                }
            }
        }
        #endif

        if(0x0A==g_LoggerRun.update)
        {
            UartUpdate(g_uUpdateBuffer);
        }
    }

}

/****************************************************************************
* ��    �ƣ�DealCmd()
* ��    �ܣ�MODBUSд
* ��ڲ�����
*         pMaster:��վ�����ṹ��ָ��
*         uAddr:���豸��ַ
*         uCmd:������ 0x06  0x10
* ���ڲ�������
* ��    ��: ��
****************************************************************************/
#define  PRERESET  0x01   // Ԥ�ûָ���������
#define  PREBACK   0x02   // Ԥ�ó���ع�
#define  UPDATE    0x04   // Ԥ�ó�������

void DealCmd(char *uRec,uint16_t uLen)
{
    static uint8_t uPreSet=0;
    uint8_t uCmd;
    uint8_t uMesHead,uMesTail,i;
    uint16_t uValue;
    uint32_t uTemp;

    uint16_t j;
    OS_STK_DATA sStk;
    
    if(uLen < 4)
    {
        return ;
    }
    
    uValue = CRC16((uint8_t *)uRec,uLen-2);
    
    if(uValue!=((uRec[uLen-2]<<8)|uRec[uLen-1]))
    {
        WDEBUGOUT("CRC����\r\n");
        return ;
    }

    if('A'==uRec[0] && 'A'==uRec[1])
    {
        uCmd = (uRec[2] - '0')*10 + uRec[3] - '0';
    }
    else
    {
        uCmd = (uRec[0] - '0')*10 + uRec[1] - '0';
    }

    for(i=3,uMesHead=0,uMesTail=0; i<uLen; i++)
    {
        if('\"'==uRec[i])
        {
            if(0==uMesHead)
            {
                uMesHead = i;
                continue;
            }
            if(0==uMesTail)
            {
                uMesTail = i;
                break;
            }
        }
    }

    switch(uCmd)
    {
    case 1:   // ��ʾ������Ϣ
        ReadEepData(EEP_LOGGER_INF);   // �ȴ�EEprom��ȡ��Ϣ

        PrintThisInfo();
        break;

    case 2:// ��ʾ�豸��Ϣ
        WDEBUGOUT("\n���\t��ַ\t����\tESN\t\tЭ������ ң��\tң��\tң��\tң��\t������\t�豸����汾\t %d̨\t\n",g_DeviceSouth.device_sum);
        if(0==g_DeviceSouth.device_sum)
        {
            WDEBUGOUT("û���豸\r\n");
            break;
        }
        for(i=0; i<MAX_device; i++)
        {
            switch(g_DeviceSouth.device_inf[i].baud_rate)
            {
            case 1:
                uTemp = 2400;
                break;

            case 2:
                uTemp = 4800;
                break;

            case 3:
                uTemp = 9600;
                break;

            case 4:
                uTemp = 19200;
                break;

            case 5:
                uTemp = 38400;
                break;

            case 6:
                uTemp = 115200;
                break;

            default:
                uTemp = 0;
                break;
            }
            if(0==g_DeviceSouth.device_inf[i].addr)
            {
                continue;
            }
            else
            {
                WDEBUGOUT("%02d\t%02d\t%d\t%-17.17s %d\t0x%04X\t0x%04X\t0x%04X\t0x%04X\t%d\t%-17.17s\t%c%c\n",//WDEBUGOUT("%02d    %02d    %d   %-20.20s %d    0x%04X   0x%04X   0x%04X   %d   %c%c    %0.17s\n",
                         i,g_DeviceSouth.device_inf[i].addr,g_DeviceSouth.device_inf[i].protocol_num,
                         g_DeviceEsn.cDeviceEsn[i],g_DeviceSouth.device_inf[i].protocol_type,
                         g_DeviceSouth.device_inf[i].yx_start_addr,
                         g_DeviceSouth.device_inf[i].yc_start_addr,
                         g_DeviceSouth.device_inf[i].yk_start_addr,
                         g_DeviceSouth.device_inf[i].sd_start_addr,
                         uTemp,
                         g_DeviceSoft.cDeviceSoft[i],
                         JUDGE(g_LoggerRun.err_lost & (1<<i)),
                         JUDGE(g_LoggerAlarm.dev_lost & (1<<i))
                        );
				msleep(5);
            }
        }
        break;

    case 3:// ��������ESN
        if(!strstr(uRec,"03-ESN"))
        {
            WDEBUGOUT("��ʽ��\n");
            break;
        }

        if((uMesTail-uMesHead)<1 || (uMesTail-uMesHead)>21)
        {
            WDEBUGOUT("ESN��\n");
        }
        else
        {
            memset(g_LoggerInfo.esn, 0, 20);
            strncpy(g_LoggerInfo.esn,&uRec[uMesHead+1],uMesTail-uMesHead-1);
            WDEBUGOUT("ESN:%-20.20s\n",g_LoggerInfo.esn);

            SaveEepData(EEP_LOGGER_INF);// �洢
        }
        break;

    case 4:  // ������������
        if(!strstr(uRec,"04-NAME"))
        {
            WDEBUGOUT("��ʽ��\n");
            break;
        }

        if((uMesTail-uMesHead)<3 || (uMesTail-uMesHead)>21)
        {
            WDEBUGOUT("���ƴ�\n");
        }
        else
        {
            memset(g_LoggerInfo.name, 0, 20);
            strncpy(g_LoggerInfo.name,&uRec[uMesHead+1],uMesTail-uMesHead-1);
            WDEBUGOUT("����:%-20.20s\n",g_LoggerInfo.name);

            SaveEepData(EEP_LOGGER_INF);// �洢
        }
        break;

    case 5:  // ���������ͺ�
        if(!strstr(uRec,"05-MOD"))
        {
            WDEBUGOUT("��ʽ��\n");
            break;
        }

        if((uMesTail-uMesHead)<3 || (uMesTail-uMesHead)>21)
        {
            WDEBUGOUT("�ͺŴ�\n");
        }
        else
        {
            memset(g_LoggerInfo.model, 0, 20);
            strncpy(g_LoggerInfo.model,&uRec[uMesHead+1],uMesTail-uMesHead-1);
            WDEBUGOUT("�ͺ�:%-20.20s\n",g_LoggerInfo.model);

            SaveEepData(EEP_LOGGER_INF);// �洢
        }
        break;

    case 6:  // ������������
        if(!strstr(uRec,"06-TYPE"))
        {
            WDEBUGOUT("��ʽ��\n");
            break;
        }

        if((uMesTail-uMesHead)<3 || (uMesTail-uMesHead)>21)
        {
            WDEBUGOUT("���ʹ�\n");
        }
        else
        {
            memset(g_LoggerInfo.type, 0, 20);
            strncpy(g_LoggerInfo.type,&uRec[uMesHead+1],uMesTail-uMesHead-1);
            WDEBUGOUT("����:%-20.20s\n",g_LoggerInfo.type);

            SaveEepData(EEP_LOGGER_INF);// �洢
        }
        break;

    case 7:  // ���÷�����IP
        if(!strstr(uRec,"07-IP"))
        {
            WDEBUGOUT("��ʽ��\n");
            break;
        }
        uValue = 0;
        uTemp = 0;
        for(i=uMesHead; i<=uMesTail; i++)
        {
            if('\"'!=uRec[i] && '.'!=uRec[i])
            {
                if(uRec[i]<'0' || uRec[i]>'9')
                {
                    uValue = 0;
                    break;
                }
                uTemp = uTemp*10 + uRec[i]-'0';
            }
            if('.'==uRec[i] || '\"'==uRec[i])
            {
                if(uTemp>255)
                {
                    uValue = 0;
                    break;
                }
                uTemp = 0;
                uValue++;
            }
        }
        if(uValue<3 || (uMesTail-uMesHead)>16)
        {
            WDEBUGOUT("IP��\n");
        }
        else
        {
            uRec[uMesTail+1] = '\0';
            memset(g_LoggerInfo.server_domain,0,sizeof(g_LoggerInfo.server_domain));
            strncpy(g_LoggerInfo.server_domain,&uRec[uMesHead],uMesTail-uMesHead+2);
            WDEBUGOUT("IP:%s\n",g_LoggerInfo.server_domain);

            SaveEepData(EEP_LOGGER_INF);// �洢
        }
        break;

    case 8: // ��������
        if(!strstr(uRec,"08-DOMAIN"))
        {
            WDEBUGOUT("��ʽ��\n");
            break;
        }
        if((uMesTail-uMesHead)<5 || (uMesTail-uMesHead)>31)
        {
            WDEBUGOUT("������\n");
        }
        else
        {
            uRec[uMesTail+1] = '\0';
            memset(g_LoggerInfo.server_domain,0,sizeof(g_LoggerInfo.server_domain));
            strncpy(g_LoggerInfo.server_domain,&uRec[uMesHead],uMesTail-uMesHead+2);
            WDEBUGOUT("����:%s\n",g_LoggerInfo.server_domain);

            SaveEepData(EEP_LOGGER_INF);// �洢
        }
        break;

    case 9:  // ���ö˿�
        if(!strstr(uRec,"09-PORT"))
        {
            WDEBUGOUT("��ʽ��\n");
            break;
        }

        if((uMesTail-uMesHead)<2 || (uMesTail-uMesHead)>6)
        {
            WDEBUGOUT("�˿ڴ�\n");
        }
        else
        {
            if(Str2UI(&uRec[uMesHead],&uValue))
            {
                WDEBUGOUT("�˿ڴ�\n");
            }
            else if(uValue<1 || 65534< (uValue-1))
            { 
                WDEBUGOUT("������˿ںŷ�Χ1��65535!\n");
            }
            else
            {
                g_LoggerInfo.server_port = uValue;
                WDEBUGOUT("�˿�:%d\n",g_LoggerInfo.server_port);

                SaveEepData(EEP_LOGGER_INF);// �洢
            }
        }
        break;

    
    case 10:  // ���ò�ѯ���ʱ��
        //WDEBUGOUT("���ɸ�\r\n");
		for(i=0,uMesHead=0,uMesTail=0; i<uLen; i++)
		{
			if('\"'==uRec[i])
			{
				if(0==uMesHead)
				{
					uMesHead = i;
					continue;
				}
				if(0==uMesTail)
				{
					uMesTail = i;
					break;
				}
			}
		}

        if((uMesTail-uMesHead)<1)
        {
            WDEBUGOUT("ʱ�����\r\n");
        }
        else
        {
            if(Str2UI(&uRec[uMesHead],&g_LoggerInfo.inquire_interval_time))
            {
                WDEBUGOUT("ʱ�����\r\n");
            }
            else
            {
               /* if(uValue > 600) // ����10����
                {
                    WDEBUGOUT("�600��\r\n");
                    uValue = 600;
                }*/
                if(g_LoggerInfo.inquire_interval_time > 900) // ����15����
                {
                    WDEBUGOUT("��Դ�ɼ��900��\n");
                    g_LoggerInfo.inquire_interval_time = 900;
                }
                else if(g_LoggerInfo.inquire_interval_time < 60) // С��1����
                {
                    WDEBUGOUT("��Դ�ɼ����60��\n");
                    g_LoggerInfo.inquire_interval_time = 60;
                }
                //g_LoggerInfo.inquire_interval_time = uValue;
                WDEBUGOUT("ʱ��:%ds\r\n",g_LoggerInfo.inquire_interval_time);

                SaveEepData(EEP_LOGGER_INF);// �洢
               // ChangeIntervalTime();
            }
        }
        break;
        
    case 11:  // �ָ���������
        if(uPreSet&PRERESET)
        {
            AllReset(1);
            //soft_timer_set(&SoftReboo,30000,1);  // ����ʱ�䲢��ʼ��ʱ
            g_LoggerRun.update = 0x00;
            uPreSet &= ~PRERESET;
            WDEBUGOUT("�ָ����\n");
        }
        else
        {
            uPreSet |= PRERESET;
            WDEBUGOUT("�ٴ�������ȷ��\n");
        }
        break;

    case 12:  // �ع�
        if(0==(uPreSet&PREBACK))
        {
            uPreSet |= PREBACK;
            WDEBUGOUT("�ٴ�������ȷ��,\"ESC\"ȡ��\n");
        }
        else
        {
            uPreSet &= ~PREBACK;

            // �洢������Ϣ��EEPROM��Ȼ������оƬ
            uValue = ReadEepData(EEP_UPDATA);//EEP_Read_data(EEP_LOGGER_UPDATA_HEAD,(uint8_t *)updata,sizeof(UPDATA_MARK_T),&updata->CRC);// ������Ϣ��ȡ
            if(EEP_OK==uValue) // ��ȡ���ݳɹ�,�� �ع����ݱ�־���Ϊ�лع�����
            {
                if(0xBB==g_LoggerUpdate.rollback_allow) // ����ʷ���򣬿��Իع�
                {
                    g_LoggerUpdate.frame_sum      = 0x00;
                    g_LoggerUpdate.updata_mark    = 0xBB;       // 0xAA������0xBB�ع���0x55��
                    g_LoggerUpdate.rollback_allow = 0xBB;

                    if(GetVerS2()&0x01)  // С�汾�����λ��1��������A�棻0��������B��
                    {
                        g_LoggerUpdate.a_version[0] = GetVerS2();   // ��ǰ�汾1
                        g_LoggerUpdate.a_version[1] = GetVerS1();   // ��ǰ�汾1
                        g_LoggerUpdate.a_version[2] = GetVerType(); // ��ǰ�汾1
                    }
                    else
                    {
                        g_LoggerUpdate.b_version[0] = GetVerS2();   // ��ǰ�汾1
                        g_LoggerUpdate.b_version[1] = GetVerS1();   // ��ǰ�汾1
                        g_LoggerUpdate.b_version[2] = GetVerType(); // ��ǰ�汾1
                    }

                    g_LoggerUpdate.reserve = 0x00;

                    SaveEepData(EEP_UPDATA);// EEP_Save_data(EEP_LOGGER_UPDATA_HEAD,(uint8_t *)updata,sizeof(UPDATA_MARK_T),&updata->CRC);// ������Ϣ�洢

                    WDEBUGOUT("������\n");
                    Reboot();  // ����
                }
                else
                {
                    WDEBUGOUT("����ʷ����\n");
                }
            }
        }
        break;

    case 13:  // ȡ��Ԥ�ò���
        uPreSet = 0x00;

        if(0x0A==g_LoggerRun.update)
        {
            g_LoggerRun.update &= 0xF0;
            WDEBUGOUT("UartInterface case 13 reboot!");
            Reboot();  // ����оƬ
        }

        WDEBUGOUT("����ȡ��\r\n");
        break;

    case 14:  // ��λ������оƬ
        WDEBUGOUT("UartInterface case 14 reboot��");
        Reboot();  // ����
        break;

    case 15:
        WDEBUGOUT("����-ң��:%d ң��:%d ���:%d ң��:%d ���:%d �澯��%d\n",g_DeviceSouth.yx_sum,g_DeviceSouth.yc_sum,g_DeviceSouth.dd_sum,g_DeviceSouth.yk_sum,g_DeviceSouth.sd_sum);
        for(i=0; i<MAX_device; i++)
        {
            if(g_DeviceSouth.protocol[i].protocol_num)  // �е������ ���Ų�����0
            {
                WDEBUGOUT("\n���:%d������:%d\n",i,g_DeviceSouth.protocol[i].protocol_num);
                WDEBUGOUT("�Ĵ���\t\t��Ϣ����\t��Ϣ����\t��������\n");
                for(j=0; j<g_DeviceSouth.protocol[i].mess_point_sum; j++)
                {
                    
						WDEBUGOUT("%4X\t%d\t%d\t%02d\t%d\n",

						g_pRegPoint[i][j].reg_addr,
						g_pRegPoint[i][j].reg_addr,
						g_pRegPoint[i][j].reg_type.type.mess,
						g_pRegPoint[i][j].reg_count,
						g_pRegPoint[i][j].reg_type.type.data);
                    
                }
            }
            else
            {
                continue;
            }
        }
        WDEBUGOUT("\n--��ӡ���--\n");
        break;

    case 16:
        if(!strstr(uRec,"16-PHONE"))
        {
            WDEBUGOUT("��ʽ��\n");
            break;
        }
        if(11!=(uMesTail-uMesHead-1))
        {
            WDEBUGOUT("�����\n");
        }
        else
        {
            //memset(Logger_inf., 0, 20);
            strncpy(g_LoggerInfo.phonenum,&uRec[uMesHead+1],11);
            WDEBUGOUT("����:%-11.11s\n",g_LoggerInfo.phonenum);

            SaveEepData(EEP_LOGGER_INF);// �洢
        }
        break;

    case 17:
        if(0==g_LoggerRun.err)
        {
            WDEBUGOUT("�����ɱ����澯\n");
        }
        else
        {
            if(g_LoggerRun.err & err_power_off)
            {
                WDEBUGOUT("[����]�ϵ�\n");
            }
        }
        //WDEBUGOUT("[����]RAM Space %d Bytes\n",FreeRamSpace());

        //----------------------------------------
        // �����豸�澯
        WDEBUGOUT("�����豸�澯\n");
        for(i=0; i<MAX_device&&i<g_DeviceSouth.device_sum; i++)
        {
            WDEBUGOUT("���:%2d,��ַ:%2d:\n",i,g_DeviceSouth.device_inf[i].addr);
            uValue = g_DeviceSouth.protocol[g_DeviceSouth.device_inf[i].rel_num].alarm_sum;
            for(j=0; j<uValue; j++)
            {
                WDEBUGOUT("MOD:%d,%d:NEW:0x%04X,ACK:0x%04X\n",g_psSouthAlarmCopy[i][j].mdbus_addr,g_psSouthAlarm[i][j].mdbus_addr,g_psSouthAlarmCopy[i][j].alarm_value,g_psSouthAlarm[i][j].alarm_value);
            }
        }
        WDEBUGOUT("-��ӡ���-\n");
        break;

      case 20:
		    for(i=2,uMesHead=0,uMesTail=0; i<uLen; i++)
		    {
		        if('-'==uRec[i])
		        {
		            if(0x30==uRec[i+1])
		            {
		                SouthSwich = 0;
		                WDEBUGOUT("�ر���������ʾ");
		            }
		            if(0x31==uRec[i+1])
		            {
		                SouthSwich = 1;
		                WDEBUGOUT("��ʾ�����ķ���");
		            }
		            if(0x32==uRec[i+1])
		            {
		                SouthSwich = 2;
		                WDEBUGOUT("��ʾ�����Ľ���");
		            }
		            if(0x33==uRec[i+1])
		            {
		                SouthSwich = 3;
		                WDEBUGOUT("��ʾ�����շ�����");
		            }
		        }
		    }
		    break;

  /*  case 21:  // Ԥ�õ��
        if(!strstr(uRec,"21-TABLE"))
        {
            WDEBUGOUT("��ʽ��\n");
            break;
        }
        if((uMesTail-uMesHead)<2 || (uMesTail-uMesHead)>6)
        {
            WDEBUGOUT("��Ŵ�\n");
            break;
        }

        Str2UI(&uRec[uMesHead],&uValue);
        if(uValue<=60000)
        {
            WDEBUGOUT("ȡ��Ԥ��:%d\n",uValue);
            g_PerSetTableResq = uValue;
            SaveEepData(EEP_TABLE_SEQ);
        }
        else
        {
            WDEBUGOUT("Ԥ�ñ��:%d\n",uValue);
            g_PerSetTableResq = uValue;
            SaveEepData(EEP_TABLE_SEQ);
        }

        g_LoggerInfo.phonenum[2] = (uValue/10000)%10 +0x30;
        g_LoggerInfo.phonenum[3] = (uValue/1000)%10 +0x30;
        g_LoggerInfo.phonenum[4] = (uValue/100)%10 +0x30;
        g_LoggerInfo.phonenum[5] = (uValue/10)%10 +0x30;
        g_LoggerInfo.phonenum[6] = uValue%10 +0x30;

        SaveEepData(EEP_LOGGER_INF);
        break;
*/
    case 22:
        /*
        // ����д����
        Str2UI(&uRec[uMesHead],&uValue);
        WDEBUGOUT("value=%d\n",uValue);


        if(1==uValue)
        {
            IEC104_DATA_YK[uValue] = uValue+1;
            g_uTransData = 1;
        }
        else
        {
            g_uTransData = uValue;
        }

        OSQPost(MesQ, &g_uTransData);
        */
        break;

    case 23:
        /*WDEBUGOUT("����д�뷢������\n");
        for(i=0;i<20;i++)
        {
            g_uRecData[i] = i;
        }

        CmStoreSendLink(g_uRecData,20);*/
        break;

    case 24:
        Str2UI(&uRec[uMesHead],&uValue);

        DataFlash_Read(uValue,g_uRecData,200);
        WDEBUGOUT("%d��200�ֽ�\n",uValue);
        for(i=0;i<200;i++)
        {
            WDEBUGOUT("%02X ",g_uRecData[i]);
        }
        break;
	case 25:
        DEBUGOUT("\ng_DeviceSouth.device_sum:%d\r\n",g_DeviceSouth.device_sum);
		for(uint8_t m=0;m<MAX_device+1;m++)
		 {
			 DEBUGOUT("uEsnMark[%d]:%d\r\n",m,g_DeviceEsn.uEsnMark[m]);
		 }
		break;
	/*case 26:
            for(uint8_t i=0;i<10;i++)
            {
				DEBUGOUT("%d %s\r\n",i+1,uAllocation.cDeviceEsn[i]);
			}
			
			break;*/

/*	case 30:
	    {
    	    DEBUGOUT("\n%%%%%%%%%%%%%%%%%%%%%%%%%%% DATAFLASH READ %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n");
    	    uint16_t i = 0, j = 0;
            
            do
            {
                DataFlash_Read(0x11E932 + i, g_TestData, SIZE);
                for (j = 0; j < SIZE; ++j)
                {
                    DEBUGOUT("%02x ", g_TestData[j]);
                }
                
                i = i + SIZE;
            }while (0x11F565 > (0x11E932 + i));
            DEBUGOUT("\n");
    	    break;
	    }*/
    case 33:
        /*for(i=0; i<128; i++)
        {
            g_TestData[i] = i;
            //WDEBUGOUT("%02X ",g_TestData[j*128+i]);
        }
        WDEBUGOUT("Write to EEPROM\n");
        EepSavedata(0x44,g_TestData,128,NULL);
        WDEBUGOUT("Read from EEPROM\n");
        EepReadData(0x44,g_TestOut,128,NULL);
        WDEBUGOUT("Compare Data\n");
        for(j=0;j<128;j++)
        {
            if(g_TestOut[j] != g_TestData[j])
            {
                WDEBUGOUT("Compare Err %d\n",j);
                break;
                //break;
            }
        }*/
        WDEBUGOUT("Compare Data OK\n");
        break;

    case 44:
        /*Str2UI(&uRec[uMesHead],&uValue);
        WDEBUGOUT("value=%d\n",uValue);
        if(uValue>2047)
        {
            WDEBUGOUT("��������\n");
            break;
        }

        for(j=0;j<32;j++)
        {
            for(i=0;i<128;i++)
            {
                g_TestData[j*128+i] = i;
                //WDEBUGOUT("%02X ",g_TestData[j*128+i]);
            }
            //WDEBUGOUT("\n");
        }
        WDEBUGOUT("Erase DataFlash sector %d\n",uValue);
        DataFlash_Sector_Erase(uValue*4096);
        WDEBUGOUT("Write to DataFlash\n");
        DataFlash_Write(uValue*4096,g_TestData,4096);
        WDEBUGOUT("Read from DataFlash\n");
        DataFlash_Read(uValue*4096,g_TestOut,4096);
        WDEBUGOUT("Compare Data\n");
        for(j=0;j<4096;j++)
        {
            if(g_TestOut[j] != g_TestData[j])
            {
                WDEBUGOUT("Compare Err %d\n",j);
                break;
            }
        }
        WDEBUGOUT("Compare Data OK\n");*/
        break;

    case 55:
        WDEBUGOUT("OSTime=%ld\n",OSTimeGet());
        break;

    case 88:
        WDEBUGOUT("[����]RAM Pool Free %d Bytes\n",FreeRamSpace());
        WDEBUGOUT("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n");
        OSTaskStkChk(3,&sStk);
        WDEBUGOUT("TaskModemProcess  used/free:%d/%d  usage:%d%%\n",sStk.OSUsed,sStk.OSFree,(sStk.OSUsed*100)/(sStk.OSUsed+sStk.OSFree));
        OSTaskStkChk(5,&sStk);
        WDEBUGOUT("TaskIec104Process used/free:%d/%d  usage:%d%%\n",sStk.OSUsed,sStk.OSFree,(sStk.OSUsed*100)/(sStk.OSUsed+sStk.OSFree));
        OSTaskStkChk(6,&sStk);
        WDEBUGOUT("TaskSouthInquire  used/free:%d/%d  usage:%d%%\n",sStk.OSUsed,sStk.OSFree,(sStk.OSUsed*100)/(sStk.OSUsed+sStk.OSFree));
        OSTaskStkChk(7,&sStk);
        WDEBUGOUT("TaskSouthWrite    used/free:%d/%d  usage:%d%%\n",sStk.OSUsed,sStk.OSFree,(sStk.OSUsed*100)/(sStk.OSUsed+sStk.OSFree));
        OSTaskStkChk(8,&sStk);
        WDEBUGOUT("TaskUart0Process  used/free:%d/%d  usage:%d%%\n",sStk.OSUsed,sStk.OSFree,(sStk.OSUsed*100)/(sStk.OSUsed+sStk.OSFree));
        OSTaskStkChk(9,&sStk);
        WDEBUGOUT("TaskLedCtrl       used/free:%d/%d  usage:%d%%\n",sStk.OSUsed,sStk.OSFree,(sStk.OSUsed*100)/(sStk.OSUsed+sStk.OSFree));

        OSTaskStkChk(OS_TASK_IDLE_PRIO,&sStk);
        WDEBUGOUT("TaskIDLE %d       used/free:%d/%d  usage:%d%%\n",OS_TASK_IDLE_PRIO,sStk.OSUsed,sStk.OSFree,(sStk.OSUsed*100)/(sStk.OSUsed+sStk.OSFree));
        WDEBUGOUT("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n");
        break;

    case 99:
        if(uPreSet&UPDATE)
        {
            /*if(uRec[8]>='0' && uRec[8]<='9')
            {
                if((uRec[8]-'0')&0x01)
                {
                    g_uUpdateSide = 0xAA;
                }
                else
                {
                    g_uUpdateSide = 0xBB;
                }
            }
            else if(uRec[8]>='A' && uRec[8]<='F')
            {
                if((uRec[8]-'A')&0x01)
                {
                    g_uUpdateSide = 0xAA;
                }
                else
                {
                    g_uUpdateSide = 0xBB;
                }
            }
            else
            {
                WDEBUGOUT("Version Err\r\n");
                break;
            }*/

            if(0==g_LoggerRun.update)
            {
                g_LoggerRun.update = 0x0A;  // ��������
                uPreSet &= ~UPDATE;
            }
            else
            {
                WDEBUGOUT("������\n");
                uPreSet &= ~UPDATE;
                break;
            }

            // DataFlash�洢�������������������96KB��һ������+8������
            DataFlash_Block_Erase(0);// ������0

            DataFlash_Sector_Erase(0x010000);
            DataFlash_Sector_Erase(0x011000);
            DataFlash_Sector_Erase(0x012000);
            DataFlash_Sector_Erase(0x013000);

            DataFlash_Sector_Erase(0x014000);
            DataFlash_Sector_Erase(0x015000);
            DataFlash_Sector_Erase(0x016000);
            DataFlash_Sector_Erase(0x017000);

            UartInit(UPDATE_UART_USE,9600,UART_PARITY_NONE);      // ����3��ʼ�� RS485

            g_uUpdateBuffer = (uint8_t*)WMemMalloc(g_uUpdateBuffer,270);

            if(NULL==g_uUpdateBuffer)
            {
                WDEBUGOUT("���������������,��ʱ��λ\n");
                sleep(1);
                Reboot();// ����оƬ
            }

            WDEBUGOUT("��������ģʽ\n");
        }
        else
        {
            uPreSet |= UPDATE;
            WDEBUGOUT("�������汾:%-6.6s\n",&uRec[3]);
            WDEBUGOUT("�ٴ�������ȷ��\n");
        }
        break;

    default:
        WDEBUGOUT("��Чָ��\n");
        break;
    }
}

/******************************************************************************
* ��    �ƣ�UartUpdate()
* ��    �ܣ�ͨ�����˴�����������
* ��ڲ�����
            ��
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
uint8_t UartUpdate(uint8_t *pUpdteBuffer)
{
    static  uint16_t uFrameCount=0;   // ����֡����
    uint16_t uCrc;                    // CRCУ������
    //uint16_t uDataCount;              // ���ڶ�ȡ�ֽ�����
    uint16_t uRecSeq;                 // ���յ���֡���

    int16_t iCom;
    uint8_t  *uRrecData = (uint8_t *)pUpdteBuffer;          // �������ݻ���

    iCom = UartRxLen(UPDATE_UART_USE);   // ��ȡ���ڻ��������ֽ�����
    if(iCom<=0)
    {
        return 0;
    }

    iCom = UartRead(UPDATE_UART_USE, pUpdteBuffer, 270, 2);

    if(iCom<262)
    {
        return 0;
    }

    uRecSeq = (uRrecData[2] <<8) | uRrecData[3];

    uCrc = CRC16(uRrecData,iCom-2);

    //-----------------------------------------------------
    if(uCrc!=(uRrecData[iCom-2]<<8 | uRrecData[iCom-1]))
    {
        uRrecData[4] = 0x00;
        uRrecData[5] = 0x55;  // У��ʧ��
        uCrc = CRC16(uRrecData,6);

        uRrecData[6] = uCrc>>8;
        uRrecData[7] = uCrc&0xFF;

        DEBUGOUT("Updata CRC ERR\n");
        UartWrite(UPDATE_UART_USE,uRrecData,8);
        return 0;
    }
    //-----------------------------------------------------
    if(uFrameCount != (uRecSeq&0x7fff))  // ����֡��Ų���
    {
        uRrecData[2] = uFrameCount>>8;
        uRrecData[3] = uFrameCount&0xFF;  // ��Ҫ�ش���֡
        uRrecData[4] = 0x00;
        uRrecData[5] = 0x01;  // ����֡��Ų���
        uCrc = CRC16(uRrecData,6);

        uRrecData[6] = uCrc>>8;
        uRrecData[7] = uCrc&0xFF;

        DEBUGOUT("Updata Seq ERR\n");
        UartWrite(UPDATE_UART_USE,uRrecData,8);
        return 0;
    }


    // �洢��DataFlash���������������ݳ��ȣ���ȥ104���ĺ�CRCУ��
    DataFlash_Write(uFrameCount * 256,(uint8_t *)&uRrecData[4],iCom-6);

    uFrameCount++;

    //--------------------------------------------------------------------
    if(uRecSeq&0x8000)  // ���һ֡
    {
        uCrc = ReadEepData(EEP_UPDATA);//EepReadData(EEP_LOGGER_UPDATA_HEAD,(uint8_t *)&updata,sizeof(UPDATA_MARK_T),&updata.CRC);// ������Ϣ��ȡ

        if(0==uCrc) // ��ȡ����ʧ��,�лع����ݱ�־���Ϊû�лع�����
        {
            g_LoggerUpdate.rollback_allow = 0x55;
        }
        g_LoggerUpdate.frame_sum = uFrameCount;// 256���ֽ�һ֡����֡��

        if(GetVerS2()&0x01)  // С�汾�����λ��1��������A�棻0��������B��
        {
            g_LoggerUpdate.a_version[0] = GetVerS2();   // ��ǰ�汾1
            g_LoggerUpdate.a_version[1] = GetVerS1();   // ��ǰ�汾1
            g_LoggerUpdate.a_version[2] = GetVerType(); // ��ǰ�汾1
        }
        else
        {
            g_LoggerUpdate.b_version[0] = GetVerS2();   // ��ǰ�汾1
            g_LoggerUpdate.b_version[1] = GetVerS1();   // ��ǰ�汾1
            g_LoggerUpdate.b_version[2] = GetVerType(); // ��ǰ�汾1
        }

        g_LoggerUpdate.reserve = 0x00;  // Ԥ����ռλ

        if(2==(uRrecData[1]>>4))
        {
            g_LoggerUpdate.side_tobe = 0xBB;  // ������B��
        }
        else
        {
            g_LoggerUpdate.side_tobe = 0xAA;  // ������A��
        }

        //g_LoggerUpdate.side_tobe = uSide;  // Ŀ��������AB��


        if(!uFrameCount)  // û������������
        {
            g_LoggerUpdate.updata_mark = 0x55;  //  data[2] = 0x55;    // 0xAA������0xBB�ع���0x55��
        }
        else
        {
            g_LoggerUpdate.updata_mark = 0xAA;  // data[2] = 0xAA;    // 0xAA������0xBB�ع���0x55��
        }

        SaveEepData(EEP_UPDATA);

        DEBUGOUT("UPDATA REC END,REBOOT\n");

        uFrameCount = 0;
        WMemFree(g_uUpdateBuffer);
    }
    else
    {
        DEBUGOUT("%d.",uFrameCount);
    }

    //--------------------------------------------------------------------
    uRrecData[4] = 0x00;
    uRrecData[5] = 0xAA;  // ������ȷ
    uCrc = CRC16(uRrecData,6);

    uRrecData[6] = uCrc>>8;
    uRrecData[7] = uCrc&0xFF;

    UartWrite(UPDATE_UART_USE,uRrecData,8);

    if(uRecSeq&0x8000)  // ���һ֡
    {
        sleep(2);

        Reboot();// ����оƬ
    }
    //--------------------------------------------------------------------
    return 0;
}
