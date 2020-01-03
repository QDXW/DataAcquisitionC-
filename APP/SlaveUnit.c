#include "SlaveUnit.h"
#include <time.h>
#include "ModbusMaster.h"
#include "GlobalVar.h"
#include "Usart.h"
#include "DataTransferArea.h"
#include "IEC104.h"
#include "Record.h"
//#include "PersetTable.h"
#include "Uart0process.h"
#include <stdlib.h>
//#include "tool.h"
#include "WatchDog.h"
#include <math.h>
#include "tool.h"
#include "log.h"
#include "Memory.h"
#include "DataFlash.h"
#include "CRC16.h"

//====================================================================
#define msleep(x)  OSTimeDly((x)/10+1)                // usleep((x)*1000)
#define sleep(x)   OSTimeDly(OS_TICKS_PER_SEC*(x)+1)  // ��ʱx��
//====================================================================
#define JUDGE(x)  ((x)==0?0:1)  // �ж���ֵ�Ƿ�Ϊ0��Ϊ0����0����0����1
//====================================================================
#define SLAVE_UART_USE            uart3         // ʹ�ô���3
//====================================================================
#define  DISCORY_ROUND_TIME           (300000)     // �����ѯѭ��ʱ��ms
#define  SLAVE_ROUND_TIME             (g_LoggerInfo.inquire_interval_time*1000)    
//====================================================================

// ��¼��ѯ�������Ϣ��¼
typedef struct
{
    uint8_t  uAddr;            // ���͵��豸��ַ--���Ե�ַ
    uint8_t  uFun;             // ���͵Ĺ��ܺ�
    uint16_t uRegAddr;         // ���͵ļĴ�����ַ
    uint8_t  uRegCount;        // �Ĵ�������
    uint8_t  uType;            // �������ͣ�ң�ţ�ң�⣬ң�أ���㣬���
    uint16_t uPointCount;      // ���͵ĵ�����¼

    uint16_t uYcCount;         // �ۻ�����ң������
    uint8_t  uYxCount;         // �ۻ�����ң�ŵ����
    uint8_t  uDdCount;         // �ۻ����͵�ȵ����

    uint16_t uPointHead;       // ���η��͵ĵ�����
    uint8_t  uPointSum;        // ���η��͵ĵ�ļ���
    uint8_t  uBaudRate;        // ��ǰʹ�ò�����  1:2400��2:4800��3:9600��4:19200��5:38400��6:115200
}DEVICE_INQUIRE_INFO_T;


// �����ѯ�ļĴ�����ַ�����ȣ��������¼
typedef struct
{
    uint16_t uRegAddr;     // MODBUS�Ĵ�����ַ
    uint8_t  uRegCount;    // �Ĵ�������
    uint8_t  uFun;         // ������
    uint8_t  uDevAddr;     // �豸ͨѶ��ַ
}DEVICE_COM_INFO_T;
typedef struct
{
    uint16_t uRegAddr;     // MODBUS�Ĵ�����ַ
    uint8_t  uRegCount;    // �Ĵ�������
    uint8_t  uFun;         // ������
    uint8_t  uDevAddr;     // �豸ͨѶ��ַ
}DEVICE_ALARM_COM_INFO_T;

//---------------------����ͻ��--------------------------
typedef struct
{
    uint8_t yx_data[250];       // ң��ͻ�����ݣ�һ��ң�ŵ�ռ��4�ֽڣ����60�㣬��240�ֽ�
    uint8_t yc_data[240];       // ң��ͻ�����ݣ�һ��ң���ռ��8�ֽڣ����30�㣬��240�ֽ�
    uint8_t yx_count;           // ң��ͻ���ϴ������
    uint8_t yc_count;           // ң��ͻ���ϴ������
    uint8_t alarm_report[240];  // һ���澯��ռ��6�ֽ�
    uint8_t alarm_count;        // �澯�����
}IEC104_BURST_T;

/*typedef struct
{
    char  cDeviceEsn[MAX_device][18];       // �����豸
    //uint8_t uEsnMark[MAX_device];            //�����ַ��δ�����ַ�ı��
    //uint16_t  CRC;                     // �洢���ݵ�CRCУ��  
}DEVICE_ADDR_INFO_T;*/


// ������������ֽ���
//typedef struct
//{
//    uint32_t nDataLen;  //�ļ��ֽڳ���
//    uint32_t nFileCrc;  //�ļ�����CRC
//} DT1000UPDATA_DATA_T;
// ���ڱ������


//====================================================================
//static DT1000UPDATA_DATA_T g_DT1000DataLen;

//static uint8_t  s_uSouthReadSd=SOUTH_CMD_READSD; //��������

static uint8_t uNewHWDevCount=0;      // ������Ϊ�豸ʱ���������豸��������
//static uint8_t uSetNewDeviceAddr=1;


static IEC_FORMAT_T    g_sIecSend;     // ����IEC104��֡
//static IEC104_MAIN_T   *g_sIecRec;     // ����IEC104��֡


static IEC104_BURST_T g_sBurst;       // ͻ�����ݴ洢����Ϣ�����

MODBUS_MASTER_T g_sMaster;            // ��վ�ṹ��

static uint8_t  uRecBuffer[256];      // ���ջ���

OS_EVENT *pUartLock;                   // ������

static DEVICE_INQUIRE_INFO_T sSouth;  // ��ѯ��Ϣ��¼
static DEVICE_INQUIRE_INFO_T sAlarmSouth; // ��ѯ�澯��¼

static uint8_t g_uAlarmReport;        // �澯״̬
static uint32_t g_uTimeNow;           // ��ǰ��ʱ���
//static uint8_t  s_uNorthReadSd = NORTH_CMD_READSD; //��������

//===============================================================================
//===============================================================================
uint8_t SetBaudRate(uint8_t now,uint8_t target);
void ResetIecData(uint8_t uRelAddr);
void AlarmAddDev(uint8_t uRelAddr,uint8_t uSum,uint16_t uModAddr,uint16_t uValue);
uint8_t AlarmCheckout(void);
uint8_t TimedReboot(uint8_t uRandom);
uint8_t AlarmReport(uint8_t addr,uint8_t err_code,uint16_t reg_addr,uint8_t offset,uint8_t event,uint8_t imd);
uint32_t g_South_Action_Newtime = 0,gImport_Table_time = 0;

//===============================================================================
//===============================================================================
//===============================================================================
/******************************************************************************
* ��    �ƣ�SlaveAddrConvert()
* ��    �ܣ�����ͨ���ַ���ҳ���Ӧ�������±꣨��Ե�ַ����
* ��ڲ�����
            uRealAddr   �豸��ʵ��ַ
*
* ���ڲ������豸��Ե�ַ
* ��    ��:
******************************************************************************/
uint8_t SlaveAddrConvert(uint8_t uRealAddr)
{
	uint8_t uRelAddr=0xff;
	uint8_t i=0;
    if(!uRealAddr)      //ͨѶ��ַ������
    {
    	return uRelAddr;
    }
    for(i=0; i<MAX_device; i++)
    {
        if(g_DeviceSouth.device_inf[i].addr==uRealAddr)
        {
            uRelAddr = i;
            break;
        }
    }

	return uRelAddr;
}
//===============================================================================
/******************************************************************************
* ��    DTEsnChange()
* ��    �ܣ���0x1B��ѯ���������в���ESN�����Ѵ����豸�ıȶԣ���ͬ����2B������ͬ���˳���
* ��ڲ�����
            uAddr:     �豸ͨѶ��ַ
            data       �յ�������
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
int8_t DTEsnChange(uint8_t *data)
{
    uint8_t i;
    uint8_t uEsn[17] = {0};
	
    memcpy(uEsn,data,17);
	for(i = 0;i < MAX_device;)
	{
		if(0 == memcmp(g_DeviceEsn.cDeviceEsn[g_DeviceSouth.device_inf[i].addr],uEsn,17)) // ESN����ͬ
		{
		    return 0;
		}
		i++;
	}
	return 1; 
}
/******************************************************************************
* ��    �ƣ�HwEsnChange()
* ��    �ܣ���0x2B��ѯ���������в���ESN���뱣��ıȶԣ��и��ľ��ϱ���
* ��ڲ�����
            uAddr:     �豸ͨѶ��ַ
            data       �յ�������
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void HwEsnChange(uint8_t uAddr,uint8_t* data)
{
    //static uint16_t esn_changed=0;
    uint8_t uAddrTmp;
    uint8_t uEsn[17]={0};

    uAddrTmp = SlaveAddrConvert(uAddr);  // ��ͨѶ��ַת��Ϊ��Ե�ַ

    if(uAddrTmp>=MAX_device)  // ���MAXDEVICĘ���豸��ַ���MAXDEVICE,��Ե�ַΪ0~MAXDEVICE
    {
        return;
    }
	
    if(0!=memcmp(uEsn,uAllocation.cDeviceEsn[uAddrTmp],17)) // ESN����ͬ
    {
        // ����
        g_sIecSend.format.maddrL = g_DeviceSouth.device_inf[uAddrTmp].addr;
        g_sIecSend.format.maddrM = 0x00;
        g_sIecSend.format.maddrH = 0x00;

        memcpy(g_sIecSend.format.data,&uEsn[0],17);// I֡����
        IecCreateFrameI(P_HW_ESN,0x01,R_SETTING,17,&g_sIecSend); // �ֽ���Ϊ��Ϣ����*4 - �Ѿ������ĵ�һ����Ϣ���ַ
    }
    /*else
    {
        esn_changed &= ~(1<<addr); // ����Ѿ��ϱ���ESN�Ÿ���
    }*/

}
/******************************************************************************
* ��    �ƣ�HwDeviceEsn()
* ��    �ܣ���0x2B��ѯ���������в���ESN�š�
* ��ڲ�����
            uAddr:      �豸ͨѶ��ַ
            data:       �յ�������
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void HwDeviceEsn(uint8_t uAddr,uint8_t* data)
{
    uint8_t i,k;
    uint8_t uRelAddr;

    if(uAddr>MAX_device || 0==uAddr)  // ���10̨���豸��ַ���10 || 0==addr
    {
        return;
    }

    k = data[10] + 11;
    for(i=11;i<k;i++)
    {
        if('4'==data[i])
        {
            i++;
            if('='==data[i])
            {
                break;
            }
            else
            {
                continue;
            }
        }
    }
    i++;

    for(uRelAddr=0;uRelAddr<MAX_device;uRelAddr++)
    {
        if(0==g_DeviceSouth.device_inf[uRelAddr].addr)
        {
            g_DeviceSouth.device_inf[uRelAddr].addr = uAddr;
            break;
        }
    }

    if(uRelAddr>=MAX_device)
    {
        return;
    }

    memset(g_DeviceEsn.cDeviceEsn[uRelAddr],0,20);  // ���ԭ��������

    for(k=0;k<20;k++)
    {
        if(';'!=data[i])
        {
            g_DeviceEsn.cDeviceEsn[uRelAddr][k] = data[i];
            i++;
        }
        else
        {
            break;
        }
    }
}
/******************************************************************************
* ��    �ƣ�HwDeviceSoft()
* ��    �ܣ���0x2B��ѯ���������в����豸����汾�š�
* ��ڲ�����
            addr:      �豸ͨѶ��ַ
            data       �յ�������
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void HwDeviceSoft(uint8_t uAddr,uint8_t* data)
{
	uint8_t i,k;
    uint8_t uRelAddr;

    if(uAddr>MAX_device || 0==uAddr)  // ���10̨���豸��ַ���10 || 0==addr
    {
        return;
    }
	//DEBUGOUT("data:%s",&data[0]);

    k = data[10] + 11;
    for(i=11;i<k;i++)
    {
        if('2'==data[i])
        {
            i++;
            if('='==data[i])
            {
                break;
            }
            else
            {
                continue;
            }
        }
    }
    i++;
    uRelAddr = SlaveAddrConvert(uAddr);

    if(uRelAddr>MAX_device)
    {
        return;
    }

    memset(g_DeviceSoft.cDeviceSoft[uRelAddr],0,17);  // ���ԭ��������
    for(k=0;k<17;k++)
    {
        if(';'!=data[i])
        {
            g_DeviceSoft.cDeviceSoft[uRelAddr][k] = data[i];
            i++;
        }
        else
        {
			break;
        }
    }
}
/******************************************************************************
* ��    �ƣ�HwSoftChange()
* ��    �ܣ���0x2B��ѯ���������в���soft���뱣��ıȶԣ��и��ľ��ϱ���
* ��ڲ�����
            addr:      �豸��Ե�ַ
            data       �յ�������
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void HwSoftChange(uint8_t uAddr,uint8_t* data)
{
    uint8_t i,k,addr;
    uint8_t soft[17]={0};

    addr = SlaveAddrConvert(uAddr);  // ��ͨѶ��ַת��Ϊ��Ե�ַ

    if(addr>=MAX_device)  // ���3̨���豸��ַ���3,��Ե�ַΪ0~3
    {
        return;
    }

    k = data[10] + 11;
    for(i=11;i<k;i++)
    {
        if('2'==data[i])
        {
            i++;
            if('='==data[i])
            {
                break;
            }
            else
            {
                continue;
            }
        }
    }

    i++;
    for(k=0;k<17;k++)
    {
        if(';'!=data[i+k])
        {
            soft[k] = data[i+k];
        }
        else
        {
            break;
        }
    }

    if(ReportCtrlRead(REPORT_HW_SOFT))// g_uNextFrame & REPORT_HW_DEVICE)  // �ϱ���һ֡��Ϊ�豸��Ϣ���ȴ�ƽ̨�ظ����ٴ�������汾���ϱ�
    {
        return;
    }

    if(0!=memcmp(soft,g_DeviceSoft.cDeviceSoft[addr],17)) // ����Ų���ͬ
    {
        memcpy(g_DeviceSoft.cDeviceSoft[addr],soft,17);

        // ����
        g_sIecSend.format.maddrL = 0x00;
        g_sIecSend.format.maddrM = 0x00;
        g_sIecSend.format.maddrH = 0x00;

        g_sIecSend.format.data[0] = g_DeviceSouth.device_inf[addr].addr;               //�豸��ַ
        g_sIecSend.format.data[1] = 0x01;                                             //���Ӷ˿�

        memcpy(&(g_sIecSend.format.data[2]),&data[10],data[10]+1);// I֡����
        IecCreateFrameI(P_HW_INFO,0x01,R_INFO_REPORT,data[10]+3,&g_sIecSend); // �ֽ���Ϊ��Ϣ����*4 - �Ѿ������ĵ�һ����Ϣ���ַ

        ReportCtrlSet(REPORT_HW_SOFT);  // ��Ƿ�����һ֡�ϱ���Ϊ�豸����汾��Ϣ����
    }
    return;
}
/******************************************************************************
* ��    �ƣ�SouthDeviceCheck()
* ��    ��: �����豸���
* ��ڲ�����
            ��
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
 uint8_t SouthDeviceCheck(uint8_t CheckMark)
{
	for(uint8_t k = 1;k < (MAX_device+1);)
	{
		if(0 == memcmp(&CheckMark,&g_DeviceSouth.device_inf[k-1].addr,1))
		{		

		    HwSoftChange(g_DeviceSouth.device_inf[k-1].addr,uRecBuffer);
            sleep(2);
			return 0;
		}
		k++;
	}
	return 1;

}
/******************************************************************************
* ��    �ƣ�SouthDeviceReport()
* ��    �ܣ��ϱ��Ѵ��ڵ������豸��
* ��ڲ�����
            ��
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void SouthDeviceReport(uint16_t uReason)
{
    uint8_t i,j,k=0;

    g_sIecSend.format.maddrL = 0x00;
    g_sIecSend.format.maddrM = 0x00;
    g_sIecSend.format.maddrH = 0x00;

    for(i=0; i<MAX_device; i++)
    {
        if(g_DeviceSouth.device_inf[i].addr)
        {
            j = k * 25;

            g_sIecSend.format.data[j] = g_DeviceSouth.device_inf[i].addr;
            j += 1;
            memcpy(&g_sIecSend.format.data[j],&g_DeviceEsn.cDeviceEsn[i],20);// I֡����
            j += 20;
            g_sIecSend.format.data[j] = g_DeviceSouth.device_inf[i].protocol_num&0xff;
            j += 1;
            g_sIecSend.format.data[j] = g_DeviceSouth.device_inf[i].protocol_num>>8;
            j += 1;
            g_sIecSend.format.data[j] = 1;
            j += 1;
            g_sIecSend.format.data[j] = g_DeviceSouth.device_inf[i].protocol_type;

            k++;

            if(k>=9)  // һ֡�������9���豸��Ϣ
            {
                IecCreateFrameI(P_SOUTH_INFO,1,uReason,25*k,&g_sIecSend);//R_SETTING
                k = 0;
            }
        }
        else
        {
            continue;
        }
    }

    if(k)
    {
        IecCreateFrameI(P_SOUTH_INFO,1,R_SETTING,25*k,&g_sIecSend);
        k = 0;
    }
}
/******************************************************************************
* ��    �ƣ�HwDeviceReport()
* ��    �ܣ��ϱ���Ϊ�豸�Է��֡�
* ��ڲ�����
            uStep    1����ʱ�����ݣ�ֱ���������ͣ�2��ֻ�����������ݣ�3����������������������
            pData    �������������ָ��
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void HwDeviceReport(uint8_t uStep,const uint8_t *pData)
{
    uint8_t i;
    static uint8_t uDataHead=0;   // ��Ϊ�豸�Է����ϱ��������±�
    //------------------------------------------------------
    if(NORTH_OK!=g_LoggerRun.north_status)  // ���ӷ�����δ���
    {
        return;
    }

    do
    {
        if((240-uDataHead)>(pData[10]+3) && (1==uStep || 3==uStep))  // һ֡����256�ֽڣ���ȥЭ������֡���ݹ�15�ֽ�// ��֡�Ѿ��Ų���һ̨�豸����
        {
            g_sBurst.yx_data[uDataHead] = pData[0]; // �豸��ַ
            uDataHead++;
            g_sBurst.yx_data[uDataHead] = 1;  // ���Ӷ˿�
            uDataHead++;
            g_sBurst.yx_data[uDataHead] = pData[10];   // ��Ϣ����
            uDataHead++;
            for(i=0; i<pData[10]; i++)
            {
                g_sBurst.yx_data[uDataHead] = pData[i+11];
                uDataHead++;
            }
            g_sBurst.yx_count++;  // ��ѯ����0x2B�豸������1

            if(1==uStep)
            {
                uStep = 0;
            }
        }

        if((240-uDataHead)<(pData[10]+3) || (uStep>=2 && g_sBurst.yx_count))  // һ֡����256�ֽڣ���ȥЭ������֡���ݹ�15�ֽ�// ��֡�Ѿ��Ų���һ̨�豸����
        {
            g_sIecSend.format.maddrL = 0x00;
            g_sIecSend.format.maddrM = 0x00;
            g_sIecSend.format.maddrH = 0x00;
            memcpy(g_sIecSend.format.data,&g_sBurst.yx_data,(uDataHead));// I֡����
            IecCreateFrameI(P_HW_INFO,1,R_SETTING,(uDataHead),&g_sIecSend); // 0xC7  //199  // ��Ϊ�豸��Ϣ
            g_sBurst.yx_count = 0;  // ��ѯ����0x2B�豸����
            uDataHead = 0;

            uStep = 0;

            ReportCtrlSet(REPORT_HW_DEVICE);  // ��Ƿ�����һ֡�ϱ���Ϊ�豸��Ϣ����
        }
        else
        {
            uStep = 0;
        }
    }while(uStep);

    //==============================================================
    //------------------------------------------------------
    /*if(NORTH_OK!=g_LoggerRun.north_status)  // ���ӷ�����δ���
    {
        return;
    }

    if(240>(pData[9]+3))  // һ֡����256�ֽڣ���ȥЭ������֡���ݹ�15�ֽ�// ��֡�Ѿ��Ų���һ̨�豸����
    {
        g_sIecSend.format.maddrL = 0x00;
        g_sIecSend.format.maddrM = 0x00;
        g_sIecSend.format.maddrH = 0x00;

        g_sIecSend.format.data[0] = pData[0]; // �豸��ַ
        g_sIecSend.format.data[1] = 1;        // ���Ӷ˿�
        g_sIecSend.format.data[2] = pData[9]; // ��Ϣ����

        memcpy(&g_sIecSend.format.data[3],&pData[10],pData[9]);// I֡����
        IecCreateFrameI(P_HW_INFO,1,R_SETTING,pData[9]+3,&g_sIecSend); // 0xC7  //199  // ��Ϊ�豸��Ϣ

        ReportCtrlSet(REPORT_HW_DEVICE);  // ��Ƿ�����һ֡�ϱ���Ϊ�豸��Ϣ����
    }*/
}
/******************************************************************************
* ��    �ƣ�HwDeviceNew()
* ��    �ܣ���0x2B��ѯ���������в���ESN���뱣��ıȶԣ��и��ľ��ϱ���
* ��ڲ�����
            uAddr:      �豸ͨѶ��ַ
            data:       �յ�������
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void HwDeviceNew(uint8_t uAddr,uint8_t* data)
{
    uint8_t uAddrTmp;

    uAddrTmp = SlaveAddrConvert(uAddr);  // ��ͨѶ��ַת��Ϊ��Ե�ַ
	//DEBUGOUT("uAddrTmp:%d",uAddrTmp);

    if(0xff==uAddrTmp)
    {
        //DEBUGOUT("uNewHWDevCount:%d",uNewHWDevCount);
		if(0==uNewHWDevCount)
        {
            SouthDeviceReport(R_SETTING);
        }
        HwDeviceEsn(data[0],data);             // ��ȡ�豸��ESN��
        HwDeviceSoft(data[0],data);     // ��ȡ�豸������汾��
        msleep(5);
        uNewHWDevCount++;
        HwDeviceReport(1,data);  // �ϱ� 
    }
}
/******************************************************************************
* ��    �ƣ�EmptyAdrr()
* ��    �ܣ���ѯ��վ�豸ESN�Զ������ַ��
* ��ڲ�����
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
uint8_t EmptyAddr()
{
    uint8_t EmptyAddr;
	for(EmptyAddr=1;EmptyAddr<MAX_device+1;)
	{
		if(g_DeviceEsn.uEsnMark[EmptyAddr]==0)
		{
			return EmptyAddr;
		}
		EmptyAddr++;
	}
	return 0;
}
/****************************************************************************
* ��	 �ƣ�SlaveDeviceAutoAllocation()
* ��	 �ܣ��Է����Է��֣�1B,2B,3B
* ��ڲ�����
* ���ڲ�����0�������У�
			1����������������Ҫ����
			2��������������Ҫ����
			3��ƽ̨δ����
* ��	 ��: ��
****************************************************************************/
#define  SEARCH_ING         0    // 0�������У�
#define  SEARCH_END         1    // 1����������������Ҫ����
#define  SEARCH_END_IMPORT  2    // 2��������������Ҫ����
#define  SEARCH_NOTCONNECT  3    // 3��ƽ̨δ����
int8_t SlaveDeviceAutoAllocation(void)
{
	int8_t	iSearchResult;			// �����ѯ�������
	int8_t	iAssignResult[MAX_device];		//�����ַ�������
	int8_t	iDiscoveryResult[MAX_device];			  //�Է��ֽ������			 
	uint8_t uTempAddr;
	int8_t iResult1B;
	uint8_t uLen;
	uint8_t Confirm_Count = 0;
	static uint8_t uDTAddr = 1;       //D5 1B��ѯ

	if(NORTH_OK != g_LoggerRun.north_status)
	{
		return SEARCH_NOTCONNECT;
	}

	while(Confirm_Count > 2)
	{
		if(ReportCtrlRead(REPORT_HW_DEVICE)) // �ϱ����豸��Ϣû�б�ƽ̨ȷ��
		{
			sleep(1);
			Confirm_Count = 3;
//			return SEARCH_ING;
		}
		else
		{
			Confirm_Count++;
		}
	}

	if(uDTAddr > 1) 
	{ 
		 for(uint8_t uAddr=1;uAddr<MAX_device+1;)
		 {
		     memset(uRecBuffer,0,256);
			 iDiscoveryResult[uAddr] = ComMasterRead(&g_sMaster,uAddr,DISCOVERY_REPORT,0,0,uRecBuffer,NULL);
			 if((iDiscoveryResult[uAddr] > 0) && (uRecBuffer[0] == uAddr) && (uRecBuffer[1] == DISCOVERY_REPORT) && (uRecBuffer[2] == 0x45))
		     {
				  if(RUNNING_WORK_READ == g_LoggerRun.run_status)
                  {
					 if(SouthDeviceCheck(uAddr))  //�ڽ���3B��ѯ��ʱ�����ϱ��豸��ַ��ֻ�����豸��Ϣ���ϱ�
					 {
						 DEBUGOUT("[Working Discovery]%d: %s\n",uRecBuffer[0],&uRecBuffer[11]);
					     HwDeviceNew(uRecBuffer[0],uRecBuffer);
						 g_DeviceEsn.uEsnMark[uAddr] = 1;
					 }
				  }
				  else if(RUNNING_SEARCH_HW == g_LoggerRun.run_status)// ������������Ϊ�豸
				  {
					  gImport_Table_time = OSTimeGet();
					  DEBUGOUT("[Empty Device Discovery]%d: %s\n",uRecBuffer[0],&uRecBuffer[11]);
					  g_DeviceEsn.uEsnMark[uAddr] = 1;
					  HwDeviceNew(uRecBuffer[0],uRecBuffer);
				  }  
		     }
			 else
			 {
				  g_DeviceEsn.uEsnMark[uAddr] = 0;
			 }
			 uAddr++;
		 }	 

		 if(uNewHWDevCount) // ����������Ϊ�豸
		 {
			 gImport_Table_time = OSTimeGet();
//			 DEBUGOUT("/**********Pinnet Report Have Device!!!\r\n");
			 HwDeviceReport(2,uRecBuffer);  // �ϱ������ݴ������
			 uNewHWDevCount = 0;
			 g_LoggerRun.run_status = RUNNING_SEARCH_END;  // �������״̬Ϊ������Ϊ�豸����
			 msleep(1500);// ��ʱ
			 //return SEARCH_ING;
		 }

		 if(RUNNING_SEARCH_END == g_LoggerRun.run_status)
		 {
			 msleep(200);// ��ʱ
			 g_sIecSend.format.maddrL = 0x00;
			 g_sIecSend.format.maddrM = 0x00;
			 g_sIecSend.format.maddrH = 0x00;
			 g_sIecSend.format.data[0] = 0x01;
			 IecCreateFrameI(P_TABLE,1,R_TABLE_START,1,&g_sIecSend);// ������������ƽ̨
			 gImport_Table_time = OSTimeGet();
			 uDTAddr = 1;
			 return SEARCH_END_IMPORT;					
		 }
		 else if((RUNNING_SEARCH_HW == g_LoggerRun.run_status ))
		 { 
			 OSTimeDlyHMSM(0,4,0,0);//DEBUGOUT("������������ʱ5���ӣ�");
			 uDTAddr = 1;
			 return SEARCH_ING;
		 }
		 uDTAddr = 1;
		 return SEARCH_END;
	}

	if(RUNNING_WORK_READ != g_LoggerRun.run_status)  // ���ɣ��Ѿ���վ����������
	{
		g_LoggerRun.run_status = RUNNING_SEARCH_HW;  // �������״̬Ϊ������Ϊ�豸
	}
	
	memset(uRecBuffer,0,256);
	iSearchResult = ComMasterReadDiscovery(&g_sMaster,0xD5,DISCOVERY_START,0,0,uRecBuffer,NULL);
	if(iSearchResult<0)
	{
        //DEBUGOUT("iSearchResult:%d\r\n",iSearchResult);
		if(RUNNING_WORK_READ == g_LoggerRun.run_status)  // ���й�����������Ϊ�豸
		{	
			for(uint8_t uWorkSetAddr = 1;uWorkSetAddr < MAX_device+1;)				
			{
				uLen=strlen(uAllocation.cDeviceEsn[uWorkSetAddr]);				

				if(uLen != 0)
				{
					uTempAddr = EmptyAddr();
					msleep(1);
					if(0 != uTempAddr)
					{
						memcpy(uAllocation.cDeviceEsn[uTempAddr],uAllocation.cDeviceEsn[uWorkSetAddr],17);
						//memset(uAllocation.cDeviceEsn[uWorkSetAddr],0,18);
						uAllocation.cDeviceEsn[uTempAddr][17] = uTempAddr;			
						DEBUGOUT("\n[Working Allocation Addr]��%d ,[ESN]��%-17s\n",uAllocation.cDeviceEsn[uTempAddr][17],uAllocation.cDeviceEsn[uTempAddr]);
						msleep(1);
						iResult1B = DTEsnChange((uint8_t *)uAllocation.cDeviceEsn[uTempAddr]);
						if(iResult1B > 0)
						{
							memset(uRecBuffer,0,256);
							iAssignResult[uTempAddr] = ComMasterRead(&g_sMaster,0xD5,DISCOVERY_SET_ADDR,0,0,uRecBuffer,(uint8_t *)uAllocation.cDeviceEsn[uTempAddr]);
							if((iAssignResult[uTempAddr] > 0) && (uRecBuffer[1] == DISCOVERY_SET_ADDR) && (uRecBuffer[20] == uTempAddr) && (uRecBuffer[2] == 0x12))
							{
								DEBUGOUT("\nSuccess Allocation Addr is��%d\n",uRecBuffer[20]);
								g_DeviceEsn.uEsnMark[uTempAddr] = 1;
								memset(uAllocation.cDeviceEsn[uTempAddr],0,18);
							}
							else
							{
								DEBUGOUT("\nFailed Allocation Addr is��%d\n",uRecBuffer[20]);
//								g_DeviceEsn.uEsnMark[uTempAddr] = 1;
								g_DeviceEsn.uEsnMark[uTempAddr] = 0;
								memset(uAllocation.cDeviceEsn[uTempAddr],0,18);
							}
						}
						else
						{
							DEBUGOUT("Device:%-17s Existing\n",uAllocation.cDeviceEsn[uTempAddr]);
						}
					}
				}
				uWorkSetAddr++;
		    }
			memset(&uAllocation,0,sizeof(uAllocation));
		}
		else if(RUNNING_SEARCH_HW == g_LoggerRun.run_status)// ������������Ϊ�豸
		{
			for(uint8_t uNewSetAddr = 1;uNewSetAddr < MAX_device+1;)
			{ 
				uLen=strlen(uAllocation.cDeviceEsn[uNewSetAddr]);
				if(uLen != 0)
				{
					uAllocation.cDeviceEsn[uNewSetAddr][17] = uNewSetAddr;
					DEBUGOUT("\n[Empty Device Allocation Addr]��%d ,[ESN]��%-17s\n",uAllocation.cDeviceEsn[uNewSetAddr][17],uAllocation.cDeviceEsn[uNewSetAddr]);
					memset(uRecBuffer,0,256);
					iAssignResult[uNewSetAddr] = ComMasterRead(&g_sMaster,0xD5,DISCOVERY_SET_ADDR,0,0,uRecBuffer,(uint8_t *)uAllocation.cDeviceEsn[uNewSetAddr]);
					if((iAssignResult[uNewSetAddr] >0 ) && (uRecBuffer[1] == DISCOVERY_SET_ADDR) && (uRecBuffer[20] == uNewSetAddr) && (uRecBuffer[2] == 0x12))
					{
						g_DeviceEsn.uEsnMark[uNewSetAddr] = 1;
						memset(uAllocation.cDeviceEsn[uNewSetAddr],0,18);
						DEBUGOUT("\nSuccess Allocation Addr is��%d \n",uRecBuffer[20]);
					}
					else if(iAssignResult[uNewSetAddr]<0)
					{
						g_DeviceEsn.uEsnMark[uNewSetAddr] = 1;
						memset(uAllocation.cDeviceEsn[uNewSetAddr],0,18);
					}
				}
				uNewSetAddr++;
			}
			memset(&uAllocation,0,sizeof(uAllocation));
		}
	}
	uDTAddr++;
	return SEARCH_ING;
}
//===============================================================================
/******************************************************************************
* ��    �ƣ�YxReport()
* ��    �ܣ�ͻ��ң�����ݡ�
* ��ڲ�����
            uIecAddr  IEC104��Ե�ַ
            uIecVal   IEC104ֵ
            uStep    1����ʱ�����ݣ�ֱ���������ͣ�2��ֻ�����������ݣ�3����������������������
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void YxReport(uint16_t uIecAddr,uint8_t uIecVal,uint8_t uStep)
{
    Uint_Char_Convert  uCTemp;

    if(NORTH_OK!=g_LoggerRun.north_status)  // ���ӷ�����δ���
    {
        return;
    }

    do
    {
        if(g_sBurst.yx_count<60 &&  (1==uStep || 3==uStep)) // һ֡104���ģ�����������60��ң�ŵ�
        {
            // ��Ϣ���ַ
            uCTemp.u = uIecAddr + 1;
            g_sBurst.yx_data[g_sBurst.yx_count*4]   = uCTemp.c[0];
            g_sBurst.yx_data[g_sBurst.yx_count*4+1] = uCTemp.c[1];
            g_sBurst.yx_data[g_sBurst.yx_count*4+2] = 0x00;
            // ��Ϣ��ֵ
            g_sBurst.yx_data[g_sBurst.yx_count*4+3] = uIecVal;
            // ����������
            g_sBurst.yx_count++;

            if(1==uStep)
            {
                uStep = 0;
            }
        }

        if(g_sBurst.yx_count>=60 || (uStep>=2 && g_sBurst.yx_count))// һ֡104���ģ�����������60��ң�ŵ�
        {
            // һ֡�����ϴ���
            g_sIecSend.format.maddrL = g_sBurst.yx_data[0];
            g_sIecSend.format.maddrM = g_sBurst.yx_data[1];
            g_sIecSend.format.maddrH = g_sBurst.yx_data[2];

            memcpy(g_sIecSend.format.data,&g_sBurst.yx_data[3],(g_sBurst.yx_count*4-3));// I֡����
            IecCreateFrameI(M_SP_NA_1,g_sBurst.yx_count,R_BURST,(g_sBurst.yx_count*4-3),&g_sIecSend); // �ֽ���Ϊ��Ϣ����*4 - �Ѿ������ĵ�һ����Ϣ���ַ

            // ���¿�ʼ��¼
            g_sBurst.yx_count = 0x00;

            uStep = 0;
        }
        else
        {
            uStep = 0;
        }
    }while(uStep);
}
/******************************************************************************
* ��    �ƣ�YcReport()
* ��    �ܣ�ͻ��ң�����ݡ�
* ��ڲ�����
            uType     ң�����������
            uIecAddr  IEC104��Ե�ַ
            fIecVal   IEC104ֵ
            uStep    1����ʱ�����ݣ�ֱ���������ͣ�2��ֻ�����������ݣ�3����������������������
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void YcReport(uint8_t uType,uint16_t uIecAddr,uint32_t fIecVal,uint8_t uStep)
{
    Uint_Char_Convert  uCTemp;
    U32_F_Char_Convert yc_temp;

    if(NORTH_OK!=g_LoggerRun.north_status)  // ���ӷ�����δ���
    {
        return;
    }

    do
    {
        if(g_sBurst.yc_count<30 &&  (1==uStep || 3==uStep)) // һ֡104���ģ�����������30��ң���
        {
            // ��Ϣ���ַ
            uCTemp.u = uIecAddr + 0x4001;
            g_sBurst.yc_data[g_sBurst.yc_count*8]   = uCTemp.c[0];
            g_sBurst.yc_data[g_sBurst.yc_count*8+1] = uCTemp.c[1];
            g_sBurst.yc_data[g_sBurst.yc_count*8+2] = 0x00;
            // ��Ϣ��ֵ
            yc_temp.u = fIecVal;


             
            //***********************************ͻ������ֵ��λȫFֵ************            
            if(((0x4F800000==yc_temp.u)&&(uType == T_UINT32)) ||
                ((0x477FFF00==yc_temp.u)&&(uType == T_UINT16)))
            {
                g_sBurst.yc_data[g_sBurst.yc_count*8+3] = 0xFF;
                g_sBurst.yc_data[g_sBurst.yc_count*8+4] = 0xFF;
                g_sBurst.yc_data[g_sBurst.yc_count*8+5] = 0xFF;
                g_sBurst.yc_data[g_sBurst.yc_count*8+6] = 0xFF;
            }
            else
            {
                g_sBurst.yc_data[g_sBurst.yc_count*8+3] = yc_temp.c[0];
                g_sBurst.yc_data[g_sBurst.yc_count*8+4] = yc_temp.c[1];
                g_sBurst.yc_data[g_sBurst.yc_count*8+5] = yc_temp.c[2];
                g_sBurst.yc_data[g_sBurst.yc_count*8+6] = yc_temp.c[3];
            }

            
            g_sBurst.yc_data[g_sBurst.yc_count*8+7] = 0x00;  // Ʒ������
            // ����������
            g_sBurst.yc_count++;

            if(1==uStep)
            {
                uStep = 0;
            }
        }

        if(g_sBurst.yc_count>=30 || (uStep>=2 && g_sBurst.yc_count)) // һ֡104���ģ�����������30��ң���
        {
            // һ֡�����ϴ���
            g_sIecSend.format.maddrL = g_sBurst.yc_data[0];
            g_sIecSend.format.maddrM = g_sBurst.yc_data[1];
            g_sIecSend.format.maddrH = g_sBurst.yc_data[2];

            memcpy(g_sIecSend.format.data,&g_sBurst.yc_data[3],(g_sBurst.yc_count*8-3));// I֡����
            IecCreateFrameI(M_ME_NC_1,g_sBurst.yc_count,R_BURST,(g_sBurst.yc_count*8-3),&g_sIecSend); // �ֽ���Ϊ��Ϣ����*4 - �Ѿ������ĵ�һ����Ϣ���ַ

            // ���¿�ʼ��¼
            g_sBurst.yc_count = 0x00;

            uStep = 0;
        }
        else
        {
            uStep = 0;
        }
    }while(uStep);
}

/******************************************************************************
* ��    �ƣ�SouthRecDeal()
* ��    �ܣ��������ݴ���
* ��ڲ�����
*           ��
* ���ڲ�����crcУ��ͨ������1��ʧ�ܷ���0
* ��    ��:
******************************************************************************/
static uint8_t SouthRecDeal(void)
{
    uint8_t  i;
    uint8_t  uRelNum=0;
    uint16_t uIecAddr;
    uint16_t point=0;   // ��Ϣ��
    Uint_Char_Convert   uCTemp;
    U32_F_Char_Convert  uYcU;
    uint8_t uYxTemp;
    uint8_t uRelAddr;  // �豸��Ե�ַ


    if(0==uRecBuffer[0])
    {
        return 0;
    }
    uRelAddr = SlaveAddrConvert(uRecBuffer[0]);// ��ͨѶ��ַת��Ϊ��Ե�ַ
    //--------------------------------------------------------------------------
    // ��Ƕ�֡�������豸����������
    if(uRelAddr < MAX_device)
    {
        if(g_LoggerRun.err_lost & (1<<uRelAddr))//if(g_LoggerAlarm.dev_lost & (1<<uRelAddr))
        {
            g_LoggerRun.err_lost &= ~(1<<uRelAddr);
            DEBUGOUT("�豸%d����\n",uRelAddr);
        }
    }
    //--------------------------------------------------------------------------

    switch(uRecBuffer[1])
    {
    case 0x03:
    case 0x04:
        //------------------------------------------------------------------------------------------------------
        // ������������ģʽ �յ����ݵ�ͨѶ��ַ�ͷ��͵�ͨѶ��ַ��ͬ  ��Ե�ַ����
        if((RUNNING_WORK_READ!=g_LoggerRun.run_status) || (uRecBuffer[0] != sSouth.uAddr) || (uRelAddr>=MAX_device))
        {
            return 0;
        }
        uRelNum = g_DeviceSouth.device_inf[uRelAddr].rel_num;  // ��Ե���

        switch(sSouth.uType)
        {
        case TYPE_YX:  // ���Ͳ�ѯң��
            // ����洢��IEC104�����ʼ�±�
            if(g_DeviceSouth.device_inf[uRelAddr].yx_start_addr<1)
            {
                break;
            }

            uIecAddr = g_DeviceSouth.device_inf[uRelAddr].yx_start_addr-1 + (sSouth.uYxCount - sSouth.uPointSum);
            point = sSouth.uPointHead;//Slave_sent.send_point_count;
            for(i=0; i<uRecBuffer[2];)
            {

                uYxTemp = uRecBuffer[i+4];
                //-----------------ͻ��-------------------------
                if(NULL==IEC104_DATA_YX )   // IEC104��ң��ָ��Ϊ��
                {
                    DEBUGOUT("��ң�ſռ䣡");
                    break;
                }
                if(uIecAddr>(g_DeviceSouth.yx_sum-1))    // ң�������±곬��ң������
                {
                    DEBUGOUT("uIecAddr=%d yx_sum=%d",uIecAddr,g_DeviceSouth.yx_sum);
                    break;
                }
                if(uYxTemp != IEC104_DATA_YX[uIecAddr])
                {
                    IEC104_DATA_YX[uIecAddr] = uYxTemp;

                    YxReport(uIecAddr,uYxTemp,1);  // ͻ��ң������
                }
                //----------------------------------------------
                i += 2;
                uIecAddr++;
            }

            break;

        case TYPE_YC:  // ���Ͳ�ѯң��
            // ����洢��IEC104�����ʼ�±�
            if(g_DeviceSouth.device_inf[uRelAddr].yc_start_addr<0x4001)
            {
                break;
            }

            uIecAddr = g_DeviceSouth.device_inf[uRelAddr].yc_start_addr-0x4001 + (sSouth.uYcCount - sSouth.uPointSum);
            point = sSouth.uPointHead;//Slave_sent.send_point_count;

            for(i=0; i<uRecBuffer[2];)
            {
                if(1==g_pRegPoint[uRelNum][point].reg_count) // ���ݳ���1
                {
                    uYcU.u = 0; //yc_temp.f = 0;
                    uYcU.c[1] = uRecBuffer[i+3];
                    uYcU.c[0] = uRecBuffer[i+4];
                    i += 2;
                }
                else if(2==g_pRegPoint[uRelNum][point].reg_count) // ���ݳ���2
                {
                    uYcU.u = 0;
                    if(BIG_ENDIAN == g_DeviceSouth.device_inf[uRelAddr].big_little_endian) // ���ģʽ
                    {
                        uYcU.c[3] = uRecBuffer[i+3];
                        uYcU.c[2] = uRecBuffer[i+4];
                        uYcU.c[1] = uRecBuffer[i+5];
                        uYcU.c[0] = uRecBuffer[i+6];
                    }
                    else if(LITTLE_ENDIAN == g_DeviceSouth.device_inf[uRelAddr].big_little_endian) // С��ģʽ
                    {
                        uYcU.c[3] = uRecBuffer[i+5];
                        uYcU.c[2] = uRecBuffer[i+6];
                        uYcU.c[1] = uRecBuffer[i+3];
                        uYcU.c[0] = uRecBuffer[i+4];
                    }

                    i += 4;
                }

                //--------------��������ת��-------------------------
                switch(g_pRegPoint[uRelNum][point].reg_type.type.data)
                {
                case T_UINT16:  // �޷���16λ����
                    uYcU.f = uYcU.u16;
                    break;

                case T_UINT32:  // �޷���32λ����
                    uYcU.f = uYcU.u;
                    break;

                case T_INT16:  // �з���16λ����
                    uYcU.f = uYcU.i16;
                    break;

                case T_INT32:  // �з���32λ����
                    uYcU.f = uYcU.i;
                    break;

                case T_FLOAT:  // ��������
                  //uYcU.f = uYcU.f;
                    break;

                case T_EPOCHTIME:
                    if(0xFFFFFFFF==uYcU.u)
                    {
                        break;
                    }

                default:
                    uYcU.f = uYcU.u;
                    break;
                }

                //-----------------ͻ��-------------------------
                if(NULL==IEC104_DATA_YC )   // IEC104��ң��ָ��Ϊ��
                {
                    DEBUGOUT("��ң��ռ䣡");
                    break;
                }
                if(uIecAddr>(g_DeviceSouth.yc_sum-1))     //ң�������±곬��ң������
                {
                    DEBUGOUT("uIecAddr=%d yc_sum=%d",uIecAddr,g_DeviceSouth.yc_sum);
                    break;
                }
                if(IEC104_DATA_YC[uIecAddr] != uYcU.u)//
                {
                    IEC104_DATA_YC[uIecAddr] = uYcU.u;//yc_temp.f;//

                    YcReport(g_pRegPoint[uRelNum][point].reg_type.type.data,uIecAddr,uYcU.u,1); // ͻ��ң������
                }

                uIecAddr++;
                point++;
            }
            break;

        case TYPE_DD:  // ���Ͳ�ѯ���
            uIecAddr = g_DeviceSouth.device_inf[uRelAddr].dd_start_addr-0x6401 + (sSouth.uDdCount - sSouth.uPointSum);
            point = sSouth.uPointHead;//Slave_sent.send_point_count;
            break;

        case TYPE_GJ:  // ���澯��
            point = sSouth.uPointHead;
            for(i=0; i<uRecBuffer[2];)
            {
                uCTemp.u = 0;
                //if(BIG_ENDIAN == g_DeviceSouth.device_inf[uRelAddr].big_little_endian) // ���ģʽ
                {
                    uCTemp.c[1] = uRecBuffer[i+3];
                    uCTemp.c[0] = uRecBuffer[i+4];
                }
                /*else if(LITTLE_ENDIAN == g_DeviceSouth.device_inf[uRelAddr].big_little_endian) // С��ģʽ
                {
                    uCTemp.c[1] = uRecBuffer[i+4];
                    uCTemp.c[0] = uRecBuffer[i+3];
                }*/

                //-------------------------------------------------------------------
               if(NORTH_OK==g_LoggerRun.north_status)  // ���ӷ��������
                {
                    uRelNum = g_DeviceSouth.device_inf[uRelAddr].rel_num;  // ��Ե���

                    AlarmAddDev(uRelAddr,
                                g_DeviceSouth.protocol[uRelNum].alarm_sum,
                                (sSouth.uRegAddr+i/2),
                                uCTemp.u);
                }
                //-------------------------------------------------------------------
                i += 2;
            }
            break;

        default:
            break;
        }
        break;
        //------------------------------------------------------------------------------------------------------
    case 0x83:
        // ������������ģʽ �յ����ݵ�ͨѶ��ַ�ͷ��͵�ͨѶ��ַ��ͬ  ��Ե�ַ����
        if((RUNNING_WORK_READ!=g_LoggerRun.run_status) || (uRecBuffer[0] != sSouth.uAddr) || (uRelAddr>=MAX_device))
        {
            return 0;
        }
        if(65534 != sSouth.uRegAddr)
        {
            break;
        }
        switch(uRecBuffer[2])
        {
        case 0x02:  // �Ƿ���ַ
            // ����洢��IEC104�����ʼ�±�
            if(g_DeviceSouth.device_inf[uRelAddr].yx_start_addr<1)
            {
                break;
            }
            if(0 == g_DeviceSouth.device_inf[uRelAddr].addr)
            {
                break;
            }
            uIecAddr = g_DeviceSouth.device_inf[uRelAddr].yx_start_addr-1 + (sSouth.uYxCount - sSouth.uPointSum);

            YxReport(uIecAddr,0x01,1);  // ��Ϊ���������65534��

            uIecAddr++;
            break;

//		case 0x06:  // �Ƿ���ַ
//
//			break;
        }
        break;
    default:
        break;
    }
    return 0;
}

/****************************************************************************
* ��    �ƣ�SouthInquire()
* ��    �ܣ����������ѯ
* ��ڲ�����

* ���ڲ�������
* ��    ��: ��
****************************************************************************/
#define SOUTH_EMPTY    (0)   // �豸��
#define SOUTH_WAIT     (1)   // �ȴ��豸,�����ڵ��������
#define SOUTH_OVER     (2)   // һ��ѭ�����
#define SOUTH_WORK     (3)   // ������
int8_t SouthInquire(void)
{
    static uint8_t  uRelativeAddr = 0;     // �����豸��Ե�ַ
    static uint8_t  uNextDev = 0;          // ��ѯ��һ̨�豸���

    uint16_t uMessPointSum;               // ��Ϣ������
    int16_t  iReadResult;                 // �����ѯ�������
    uint8_t  uRelativePointNum = 0;         // ��Ե���
    uint8_t  i;


    DEVICE_COM_INFO_T     sCom;    // �����ѯ�Ĵ�����Ϣ

    if(RUNNING_EMPTY == g_LoggerRun.run_status)  // �����ɣ�������ϢΪ��
    {
        return SOUTH_EMPTY;
    }
    if(RUNNING_WORK_READ != g_LoggerRun.run_status)
    {
        return SOUTH_WAIT;
    }

    //while(1)
    {
        if(uNextDev&0x01)
        {
            uNextDev = 0;

            sSouth.uAddr       = 0;
            sSouth.uFun        = 0;
            sSouth.uRegAddr    = 0;
            sSouth.uRegCount   = 0;
            sSouth.uType       = 0;
            sSouth.uPointCount = 0;
            sSouth.uYcCount    = 0;
            sSouth.uYxCount    = 0;
            sSouth.uDdCount    = 0;
            sSouth.uPointHead  = 0;
            sSouth.uPointSum   = 0;

            do
            {
                uRelativeAddr++;
                 // ��Ե�ַ�����豸����  g_DeviceSouth.device_sum  ,������豸��ɾ���豸�Ȳ�������ܳ��� uRelativeAddr�����豸���������
                if(uRelativeAddr >= MAX_device)
                {
                    break;
                }
                if(0 != g_DeviceSouth.device_inf[uRelativeAddr].addr)  // �豸ͨѶ��ַ��Ч����Ϊ0��Ϊ0�����豸��ɾ��
                {
                    break;
                }

            }while(1);

            if(uRelativeAddr >= MAX_device)   // ��Ե�ַ�����豸������һ��ѭ����ѯ���Device_south.device_sum
            {
                uRelativeAddr = 0;

                if(0 == g_DeviceSouth.device_inf[uRelativeAddr].addr)
                {
                    uNextDev = 1;
                }

                //---------------�ж��Ƿ���ͻ������û���ϱ�--------------------------------
                if(NORTH_OK == g_LoggerRun.north_status)  // ���ӷ��������
                {

//                    DEBUGOUT("************************ yc_count: %d ********************************\n", g_sBurst.yc_count);
                    if(g_sBurst.yc_count)
                    {
                        YcReport(0,0,0,2);// ���Ϳ����ݴ��ң��ͻ������
                    }
                    if(g_sBurst.yx_count)
                    {
                        YxReport(0,0,2);// ���Ϳ����ݴ��ң��ͻ������
                    }
                }
                //-------------------------------------------------------------------
                return SOUTH_OVER;
            }
        }
        //--------------------------------------------------------------------------

        if(0 == g_DeviceSouth.device_inf[uRelativeAddr].addr)
        {
            uNextDev = 1;
            return SOUTH_WORK;
        }


        uRelativePointNum = g_DeviceSouth.device_inf[uRelativeAddr].rel_num;  // ��Ե���

        if((g_pRegPoint[uRelativePointNum] != NULL) && (g_DeviceSouth.device_inf[uRelativeAddr].protocol_num == g_DeviceSouth.protocol[uRelativePointNum].protocol_num)) // ָ���������ָ��
        {
            // ��Ϣ������
            uMessPointSum = g_DeviceSouth.protocol[uRelativePointNum].mess_point_sum;

            sCom.uRegCount = 0;  // �Ĵ���������ʼ��Ϊ0��


            for(; sSouth.uPointCount < uMessPointSum; sSouth.uPointCount++)
            {
                sSouth.uType = g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_type.type.mess;

                if(TYPE_YX == sSouth.uType)//YX:0x02  YC:0x01  YK:0x04  SD:0x05   DD:0x03
                {
                    sCom.uRegAddr = g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_addr;  // �Ĵ�����ַ

                    sSouth.uYxCount++;
                    sCom.uRegCount += g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_count;


                    sSouth.uPointHead = sSouth.uPointCount;
                    sSouth.uPointCount++;
                    sSouth.uPointSum = 1;
                    break;
                }
                else if(TYPE_YC == sSouth.uType)//0x01
                {
                    sSouth.uYcCount++;

                    if(g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_count <= 2)// ��Ϣ�㳤�ȴ���2������ѯ
                    {
                        sCom.uRegAddr = g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_addr;  // �Ĵ�����ַ
                        sCom.uRegCount += g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_count;

                        sSouth.uPointHead = sSouth.uPointCount;
                        sSouth.uPointSum = 1;
                        sSouth.uPointCount++;
                    }
                    else
                    {
                        continue;
                    }
                    break;
                }
                else if(TYPE_DD == sSouth.uType)
                {
                    sCom.uRegAddr = g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_addr;  // �Ĵ�����ַ

                    sSouth.uDdCount++;
                    sCom.uRegCount += g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_count;

                    sSouth.uPointHead = sSouth.uPointCount;
                    sSouth.uPointCount++;
                    sSouth.uPointSum = 1;
                    break;
                }
              else if(TYPE_GJ == sSouth.uType)
                {
                    sCom.uRegAddr = g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_addr;  // �Ĵ�����ַ
                    sCom.uRegCount += g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_count;

                    sSouth.uPointHead = sSouth.uPointCount;
                    sSouth.uPointCount++;
                    sSouth.uPointSum = 1;
                    break;
                }
            }

            for(i = sSouth.uPointCount; i < uMessPointSum; i++)
            {
                if((sSouth.uType == g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_type.type.mess)  // ��Ϣ������ͬ
                   && ((g_pRegPoint[uRelativePointNum][sSouth.uPointCount-1].reg_addr + g_pRegPoint[uRelativePointNum][sSouth.uPointCount-1].reg_count) == g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_addr) // ǰһ�������һ�����ַ����
                   && (sSouth.uPointCount>0) 
                   && (g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_count <= 2)  // ��Ϣ�㳤�Ȳ�����2
                   )
                {
                    if((sCom.uRegCount + g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_count) <= 25)
                    {
                        sCom.uRegCount += g_pRegPoint[uRelativePointNum][sSouth.uPointCount].reg_count;

                        if(TYPE_YX == sSouth.uType)
                        {
                            sSouth.uYxCount++;
                            sSouth.uPointSum++;
                            sSouth.uPointCount++;
                        }
                        else if(TYPE_YC == sSouth.uType)
                        {
                            sSouth.uYcCount++;
                            sSouth.uPointSum++;
                            sSouth.uPointCount++;
                        }
                        else if(TYPE_DD == sSouth.uType)
                        {
                            sSouth.uDdCount++;
                            sSouth.uPointSum++;
                            sSouth.uPointCount++;
                        }
                        else if(TYPE_GJ == sSouth.uType)
                        {
                            sSouth.uPointSum++;
                            sSouth.uPointCount++;
                        }
                    }
                    else
                    {
                        break;
                    }
                }
                else
                {
                    break;
                }
            }
        }
        else
        {
            uNextDev = 1;
        }


        if(sCom.uRegCount > 0)
        {
            sSouth.uRegAddr  = sCom.uRegAddr;       // �Ĵ�����ַ

            if(0x01 == g_DeviceSouth.protocol[uRelativePointNum].protocol_type)// if(0x01==g_DeviceSouth.device_inf[uRelativeAddr].protocol_type) // ��Ϊ��ԴMODBUS
            {
                sCom.uFun = 0x03;
            }
            else if(0x02 == g_DeviceSouth.device_inf[uRelativeAddr].protocol_type) // ��׼MODBUS
            {
                if(sCom.uRegAddr >= 40000)   // �����ݱ��ּĴ���40001~49999
                {
                    sCom.uFun = 0x03;
                    sCom.uRegAddr -= 40000;
                }
                else if(sCom.uRegAddr >= 30000)  // ������Ĵ���30001~39999
                {
                    sCom.uFun = 0x04;
                    sCom.uRegAddr -= 30000;
                }
                else
                {
                    sCom.uFun = 0x03;
                }
            }
            // ���ݴ�С��
            /*if(BIG_ENDIAN==g_DeviceSouth.device_inf[uRelativeAddr].big_little_endian)
            {
                sCom.endian = 0;   // ���
            }
            else
            {
                sCom.endian = 1;   // С��
            }*/
            // ͨѶ������
            sSouth.uBaudRate = SetBaudRate(sSouth.uBaudRate,g_DeviceSouth.device_inf[uRelativeAddr].baud_rate);

            sCom.uDevAddr = g_DeviceSouth.device_inf[uRelativeAddr].addr;  // ʵ�ʵ�ͨѶ��ַ

            sSouth.uAddr          = sCom.uDevAddr;      // ���Ե�ַ
            sSouth.uFun           = sCom.uFun;          // ���͵Ĺ���ָ��
            sSouth.uRegCount      = sCom.uRegCount;     // ��ѯ�Ĵ�������

            iReadResult = ComMasterRead(&g_sMaster,sCom.uDevAddr,sCom.uFun,sCom.uRegAddr,sCom.uRegCount,uRecBuffer,NULL);
            if((iReadResult > 0) || (-MODBUS_ILLEGAL_ADDR == iReadResult) || (-MODBUS_ILLEGAL_BUSY == iReadResult))  // ����������0x83�쳣��Ϊ2
            {
                SouthRecDeal();  // �������ݴ���
                //PrintHex(01,uRecBuffer,iReadResult);
                /*for(i=0;i<iReadResult;i++)
                {
                    DEBUGOUT("%02X ",uRecBuffer[i]);
                }*/
            }
            else if(-MASTER_lOST == iReadResult)  // ��֡
            {
                if((NORTH_OK == g_LoggerRun.north_status) && (RUNNING_WORK_READ == g_LoggerRun.run_status))  // ���ӷ��������
                {
                    if(0 == (g_LoggerRun.err_lost & (1<<uRelativeAddr)))
                    {
                        ResetIecData(uRelativeAddr);
                        g_LoggerRun.err_lost |= (1<<uRelativeAddr);  // ��Ƕ�֡�������豸
                        DEBUGOUT("�豸%d����\n",uRelativeAddr);
                    }
                }
                uNextDev = 1;  // ���豸ͨѶ��֡���������豸
            }
            else if(-MASTER_CRC == iReadResult)  // ��֡
            {
                DEBUGOUT("CRC����\n");
            }
        }
        //----------------------------------------------------
        if((0 == uNextDev) && (sSouth.uPointCount >= uMessPointSum))
        {
            uNextDev = 1;
        }
    }
    return SOUTH_WORK;
}
/******************************************************************************
* ��    �ƣ�SouthRecDeal()
* ��    �ܣ��������ݴ���
* ��ڲ�����
*           ��
* ���ڲ�����crcУ��ͨ������1��ʧ�ܷ���0
* ��    ��:
******************************************************************************/
static uint8_t SouthRecDealAlarm(void)
{   
    uint8_t  i;
    uint8_t  uRelNum = 0;
//    uint16_t point = 0;   // ��Ϣ��
    Uint_Char_Convert   uCTemp;
    uint8_t uRelAddr;  // �豸��Ե�ַ

    if(0 == uRecBuffer[0])
    {
        return 0;
    }
    uRelAddr = SlaveAddrConvert(uRecBuffer[0]);// ��ͨѶ��ַת��Ϊ��Ե�ַ	
	// ��Ƕ�֡�������豸����������
	if(uRelAddr < MAX_device)
	{
		if(g_LoggerRun.err_lost & (1<<uRelAddr))//if(g_LoggerAlarm.dev_lost & (1<<uRelAddr))
		{
			g_LoggerRun.err_lost &= ~(1<<uRelAddr);
			DEBUGOUT("�豸%d����\n",uRelAddr);
		}
	}
	
    switch(uRecBuffer[1])
    {
    case 0x03:
    case 0x04:
        // ������������ģʽ �յ����ݵ�ͨѶ��ַ�ͷ��͵�ͨѶ��ַ��ͬ  ��Ե�ַ����
        if((RUNNING_WORK_READ!=g_LoggerRun.run_status) || (uRecBuffer[0] != sAlarmSouth.uAddr) || (uRelAddr >= MAX_device))
        {
            return 0;
        }
        uRelNum = g_DeviceSouth.device_inf[uRelAddr].rel_num;  // ��Ե���
        
        switch(sAlarmSouth.uType)
        {
       
        case TYPE_GJ:  // ���澯��
//            point = sAlarmSouth.uPointHead;
			
            for(i = 0; i < uRecBuffer[2];)
            {
                uCTemp.u = 0;
                uCTemp.c[1] = uRecBuffer[i+3];
                uCTemp.c[0] = uRecBuffer[i+4];
                //-------------------------------------------------------------------
                if(NORTH_OK == g_LoggerRun.north_status)  // ���ӷ��������
                {
                    uRelNum = g_DeviceSouth.device_inf[uRelAddr].rel_num;  // ��Ե���
                    AlarmAddDev(uRelAddr,
                                g_DeviceSouth.protocol[uRelNum].alarm_sum,
                                (sAlarmSouth.uRegAddr+i/2),
                                uCTemp.u);
                }
                //-------------------------------------------------------------------
                i += 2;
            }
            break;
        default:
            break;
        }
        break; 
     case 0x83:
        // ������������ģʽ �յ����ݵ�ͨѶ��ַ�ͷ��͵�ͨѶ��ַ��ͬ  ��Ե�ַ����
        if((RUNNING_WORK_READ!=g_LoggerRun.run_status) || (uRecBuffer[0] != sAlarmSouth.uAddr) || (uRelAddr >= MAX_device))
        {
            return 0;
        }
        if(65534 != sAlarmSouth.uRegAddr)
        {
            break;
        }
		break;
    default:
        break;
    }
    return 0;
}
/****************************************************************************
* ��    �ƣ�SouthInquire()
* ��    �ܣ����������ѯ
* ��ڲ�����

* ���ڲ�������
* ��    ��: ��
****************************************************************************/
//#define SOUTH_ALARM_EMPTY    (0)   // �豸��
//#define SOUTH_ALARM_WAIT     (1)   // �ȴ��豸,�����ڵ��������
//#define SOUTH_ALARM_OVER     (2)   // һ��ѭ�����
//#define SOUTH_ALARM_WORK     (3)   // ������
int8_t SouthInquireAlarm(void)
{
    static uint8_t  uRelativeAddr=0;     // �����豸��Ե�ַ
    static uint8_t  uNextDev=0;          // ��ѯ��һ̨�豸���
    uint16_t uMessPointSum;               // ��Ϣ������
    int16_t  iReadResult;                 // �����ѯ�������
    uint8_t  uRelativePointNum=0;         // ��Ե���
    uint8_t  i;
    DEVICE_ALARM_COM_INFO_T     sCom;    // �����ѯ�Ĵ�����Ϣ
    
    if(RUNNING_EMPTY==g_LoggerRun.run_status)  // �����ɣ�������ϢΪ��
    {
        return SOUTH_EMPTY;
    }
    if(RUNNING_WORK_READ!=g_LoggerRun.run_status)
    {
        return SOUTH_WAIT;
    }
    //while(1)
    {
        if(uNextDev&0x01)
        {
            uNextDev = 0;
            sAlarmSouth.uAddr       = 0;
            sAlarmSouth.uFun        = 0;
            sAlarmSouth.uRegAddr    = 0;
            sAlarmSouth.uRegCount   = 0;
            sAlarmSouth.uType       = 0;
            sAlarmSouth.uPointCount = 0;
            sAlarmSouth.uYcCount    = 0;
            sAlarmSouth.uYxCount    = 0;
            sAlarmSouth.uDdCount    = 0;
            sAlarmSouth.uPointHead  = 0;
            sAlarmSouth.uPointSum   = 0;

            do
            {
                uRelativeAddr++;
                 // ��Ե�ַ�����豸����  g_DeviceSouth.device_sum  ,������豸��ɾ���豸�Ȳ�������ܳ��� uRelativeAddr�����豸���������
                if(uRelativeAddr>=MAX_device)
                {
                    break;
                }
                if(0!=g_DeviceSouth.device_inf[uRelativeAddr].addr)  // �豸ͨѶ��ַ��Ч����Ϊ0��Ϊ0�����豸��ɾ��
                {
                    break;
                }

            }while(1);

            if(uRelativeAddr>=MAX_device)   // ��Ե�ַ�����豸������һ��ѭ����ѯ���Device_south.device_sum
            {
                uRelativeAddr = 0;

                if(0==g_DeviceSouth.device_inf[uRelativeAddr].addr)
                {
                    uNextDev = 1;
                }
                return SOUTH_OVER;
            }
        }
        //--------------------------------------------------------------------------
        if(0==g_DeviceSouth.device_inf[uRelativeAddr].addr)
        {
            uNextDev = 1;
            return SOUTH_WORK;
        }

        uRelativePointNum = g_DeviceSouth.device_inf[uRelativeAddr].rel_num;  // ��Ե���
        if((g_pRegPoint[uRelativePointNum]!=NULL) && (g_DeviceSouth.device_inf[uRelativeAddr].protocol_num==g_DeviceSouth.protocol[uRelativePointNum].protocol_num)) // ָ���������ָ��
        {
            // ��Ϣ������
            uMessPointSum = g_DeviceSouth.protocol[uRelativePointNum].mess_point_sum;
            sCom.uRegCount = 0;  // �Ĵ���������ʼ��Ϊ0��
            for(; sAlarmSouth.uPointCount<uMessPointSum; sAlarmSouth.uPointCount++)
            {
                sAlarmSouth.uType = g_pRegPoint[uRelativePointNum][sAlarmSouth.uPointCount].reg_type.type.mess;
                if(TYPE_GJ==sAlarmSouth.uType)
                {
					sCom.uRegAddr = g_pRegPoint[uRelativePointNum][sAlarmSouth.uPointCount].reg_addr;  // �Ĵ�����ַ
                    sCom.uRegCount += g_pRegPoint[uRelativePointNum][sAlarmSouth.uPointCount].reg_count;
                    sAlarmSouth.uPointHead = sAlarmSouth.uPointCount;
                    sAlarmSouth.uPointCount++;
                    sAlarmSouth.uPointSum = 1;
                    break;
                }
            }

            for(i=sAlarmSouth.uPointCount; i<uMessPointSum; i++)
            {
                if((sAlarmSouth.uType==g_pRegPoint[uRelativePointNum][sAlarmSouth.uPointCount].reg_type.type.mess) // ��Ϣ������ͬ
					&& ((g_pRegPoint[uRelativePointNum][sAlarmSouth.uPointCount-1].reg_addr+g_pRegPoint[uRelativePointNum][sAlarmSouth.uPointCount-1].reg_count) == g_pRegPoint[uRelativePointNum][sAlarmSouth.uPointCount].reg_addr)  // ǰһ�������һ�����ַ����
                    && (sAlarmSouth.uPointCount>0) 
                    && (g_pRegPoint[uRelativePointNum][sAlarmSouth.uPointCount].reg_count<=2)  // ��Ϣ�㳤�Ȳ�����2
                   )
                {
                    if((sCom.uRegCount + g_pRegPoint[uRelativePointNum][sAlarmSouth.uPointCount].reg_count) <= 25)
                    {
                        sCom.uRegCount += g_pRegPoint[uRelativePointNum][sAlarmSouth.uPointCount].reg_count;

                        if(TYPE_GJ==sAlarmSouth.uType)
                        {
                            sAlarmSouth.uPointSum++;
                            sAlarmSouth.uPointCount++;
                        }
                    }
                    else
                    {
                        break;
                    }
                }
                else
                {
                    break;
                }
            }
        }
        else
        {
            uNextDev = 1;
        }

        if(sCom.uRegCount>0)
        {
			sAlarmSouth.uRegAddr  = sCom.uRegAddr;       // �Ĵ�����ַ

            if(0x01==g_DeviceSouth.protocol[uRelativePointNum].protocol_type)// if(0x01==g_DeviceSouth.device_inf[uRelativeAddr].protocol_type) // ��Ϊ��ԴMODBUS
            {
                sCom.uFun = 0x03;
            }
            else if(0x02==g_DeviceSouth.device_inf[uRelativeAddr].protocol_type) // ��׼MODBUS
            {
                if(sCom.uRegAddr>=40000)   // �����ݱ��ּĴ���40001~49999
                {
                    sCom.uFun = 0x03;
                    sCom.uRegAddr -= 40000;
                }
                else if(sCom.uRegAddr>=30000)  // ������Ĵ���30001~39999
                {
                    sCom.uFun = 0x04;
                    sCom.uRegAddr -= 30000;
                }
                else
                {
                    sCom.uFun = 0x03;
                }
            }

            // ͨѶ������
            sAlarmSouth.uBaudRate = SetBaudRate(sAlarmSouth.uBaudRate,g_DeviceSouth.device_inf[uRelativeAddr].baud_rate);
            sCom.uDevAddr = g_DeviceSouth.device_inf[uRelativeAddr].addr;  // ʵ�ʵ�ͨѶ��ַ
            sAlarmSouth.uAddr = sCom.uDevAddr;      // ���Ե�ַ
            sAlarmSouth.uFun = sCom.uFun;          // ���͵Ĺ���ָ��
            sAlarmSouth.uRegCount = sCom.uRegCount;     // ��ѯ�Ĵ�������
            iReadResult = ComMasterRead(&g_sMaster,sCom.uDevAddr,sCom.uFun,sCom.uRegAddr,sCom.uRegCount,uRecBuffer,NULL);
            if(iReadResult>0 || -MODBUS_ILLEGAL_ADDR==iReadResult)  // ����������0x83�쳣��Ϊ2
            {
                SouthRecDealAlarm();  // �������ݴ���
            }
            else if(-MASTER_lOST==iReadResult)  // ��֡
            {
                if(NORTH_OK==g_LoggerRun.north_status && RUNNING_WORK_READ==g_LoggerRun.run_status)  // ���ӷ��������
                {
                    if(0==(g_LoggerRun.err_lost & (1<<uRelativeAddr)))
                    {
                        ResetIecData(uRelativeAddr);
                        g_LoggerRun.err_lost |= (1<<uRelativeAddr);  // ��Ƕ�֡�������豸
                        DEBUGOUT("�豸%d����\n",uRelativeAddr);
                    }
                }
                uNextDev = 1;  // ���豸ͨѶ��֡���������豸
            }
            else if(-MASTER_CRC==iReadResult)  // ��֡
            {
                DEBUGOUT("CRC����\n");
            }
        }
		
        if(0==uNextDev && (sAlarmSouth.uPointCount>=uMessPointSum))
        {
            uNextDev = 1;
        }
    }
    return SOUTH_WORK;
}

/****************************************************************************
* ��    �ƣ�DT1000ReadDataflashData
* ��    �ܣ�������������ݴ�DataFlash����
* ��ڲ�����
*           nFrame   ֡��ţ���0��ʼ
*           *pData   ����ָ��
*           nLen     ���ݳ���
* ���ڲ������洢�ɹ����ض�ȡ���ֽ��������һ֡���λ��1�������ݷ���0
* ��    ������
* ��    �ߣ�WQY
****************************************************************************/
uint16_t DT1000ReadDataflashData(const uint16_t nFrame,uint8_t *pData, const uint8_t nLen)
{
    static uint32_t nAllDataLen=0;
    uint32_t nDataFalshAddr; // DataFlash���ݴ洢��ַ
    
    uint8_t nReadLen=0;
    
    uint8_t nNeedLen=nLen;

    if(0==nAllDataLen)
    {
        nAllDataLen = g_DT1000Updata.nDataLen;
    }

    nDataFalshAddr = DATAFLASH_DT1000_UPDATA_HEAD + (nFrame & 0x7fff) * nLen; // 256Ϊÿ֡���ݳ���
   
    
    if(nAllDataLen<=nLen)
    {
        nNeedLen = nAllDataLen;
    }

    nReadLen = DataFlash_Read(nDataFalshAddr, pData, nNeedLen);
	if(nReadLen<=3)
	{
		nReadLen = DataFlash_Read(nDataFalshAddr, pData, nNeedLen);
	}
	msleep(5);
	DEBUGOUT("ReadLen:%d\n",nReadLen);

    nAllDataLen = nAllDataLen - nReadLen;
	//DEBUGOUT("nAllDataLen:%d\r\n",nAllDataLen);

    /*if(0==nAllDataLen)  // �������ݶ�����
    {
        nReadLen = nReadLen | 0x8000;
    }*/
    
    return nReadLen;
}
/****************************************************************************
* ��    �ƣ�SouthBroadcastUpdata()
* ��    �ܣ����������㲥����
* ��ڲ�����

* ���ڲ�������
* ��    ��: ��
****************************************************************************/
int8_t SouthBroadcastUpdata(void)
{
	uint8_t  *pTmpData = NULL;
	uint16_t check_crc;
	uint16_t uTmpCrc;
	uint16_t s_uSendseq=0;//s_uSendAll=0; // �������
	uint8_t uReadDataLen=0;   //��ȡ���ݳ���
	uint8_t uFrameLen = 240;
	U32_F_Char_Convert uTemp;
	//====================================================================================
	//�ȹ㲥�����ļ���Ϣ
	//====================================================================================
	//memset(uTmpData,0,250);

	pTmpData = (uint8_t *)WMemMalloc(pTmpData,27*sizeof(uint8_t));
	pTmpData[0] = 0xFF;
	pTmpData[1] = DT1000UPDATE_START;
	pTmpData[2] = 0x01; 	 //�ӹ���������0x01
	pTmpData[3] = 0x16; 	 //�ļ���Ϣ����

	memcpy(&pTmpData[4],g_DT1000Updata.version,17);  //������ư汾��
	
	uTemp.u = g_DT1000Updata.nDataLen;
	pTmpData[21] = uTemp.c[3];
	pTmpData[22] = uTemp.c[2];
	pTmpData[23] = uTemp.c[1];
	pTmpData[24] = uTemp.c[0];
	
	pTmpData[25] = uFrameLen; //ÿ֡���ݳ���
	
	uTmpCrc = CRC16(pTmpData,26);
	pTmpData[26] = uTmpCrc>>8;
	pTmpData[27] = uTmpCrc&0XFF;
	
	UartWrite(SLAVE_UART_USE,pTmpData,28);
	FEED_DOG();
	//DEBUGOUT("[����]RAM Pool Free %d Bytes\n",FreeRamSpace());

	/*if(0x01&SouthSwich)
	{
		DEBUGOUT("SOUTH_UPDATA:");
		for(uint8_t i=0;i<27;i++)
		{
			DEBUGOUT("%02X ",uTmpData[i]);
		}
		DEBUGOUT("\r\n");
	}*/
	
	DEBUGOUT("Upgrade Vertion��%17s File Len��%d\n",&pTmpData[4],g_DT1000Updata.nDataLen);
    
	sleep(3);
	
	//====================================================================================
	//�������ݴ���
	//====================================================================================
	for(;(s_uSendseq*uFrameLen) < g_DT1000Updata.nDataLen;s_uSendseq++)
	{
		//memset(uTmpData,0,256);
		pTmpData = (uint8_t *)WMemMalloc(pTmpData,248*sizeof(uint8_t));
		
		pTmpData[0] = 0xFF;
		pTmpData[1] = DT1000UPDATE_DATA;
		pTmpData[2] = 0x01; 	 //�ӹ���������0x01
		
		pTmpData[3] = s_uSendseq>>8;
		pTmpData[4] = s_uSendseq&0XFF;
	   
		uReadDataLen =DT1000ReadDataflashData(s_uSendseq,&pTmpData[6],uFrameLen);
		if(0 == uReadDataLen)
		{
			DEBUGOUT("Flash Read Fail!!!\n");
			return 0;
		}
		pTmpData[5] = uReadDataLen;  //���ݳ���
		
		check_crc = CalculateCRC(&pTmpData[6],uReadDataLen);
		//pTmpData[uReadDataLen+6] = check_crc>>8;
		//pTmpData[uReadDataLen+7] = check_crc&0XFF;
		uTmpCrc = CRC16(pTmpData,uReadDataLen+6);
		pTmpData[uReadDataLen+6]=uTmpCrc>>8;
		pTmpData[uReadDataLen+7]=uTmpCrc&0XFF;
		
		UartWrite(SLAVE_UART_USE,pTmpData,uReadDataLen+8);
		FEED_DOG();
		DEBUGOUT("\nSouth Upgrade %d Frame\n",s_uSendseq);
		/*if(0x01&SouthSwich)
		{
			DEBUGOUT("SOUTH_UPDATA:");
			for(uint8_t i=0;i<(uReadDataLen+10);i++)
			{
				DEBUGOUT("%02X ",uTmpData[i]);
			}
			DEBUGOUT("\r\n");
		}*/

		sleep(1);
	}
	
	//====================================================================================
	//�㲥����������Ϣ!!!
	//====================================================================================
	//memset(pTmpData,0,250);

	pTmpData = (uint8_t *)WMemMalloc(pTmpData,10*sizeof(uint8_t));
	
	pTmpData[0]=0xFF;
	pTmpData[1]=DT1000UPDATE_END;
	pTmpData[2]=0x01;  //�ӹ���������0x01
	pTmpData[3]=0x04;  //����
	pTmpData[4]=s_uSendseq>>8;
	pTmpData[5]=s_uSendseq&0XFF;	//����֡����
	pTmpData[6]=check_crc>>8;
	pTmpData[7]=check_crc&0XFF;
	uTmpCrc = CRC16(pTmpData,8);
	pTmpData[8]=uTmpCrc>>8;
	pTmpData[9]=uTmpCrc&0XFF;
	
	UartWrite(SLAVE_UART_USE,pTmpData,10);
	DEBUGOUT("Broadcast Upgrade End!!!\n");
	pTmpData=WMemFree(pTmpData);
   /* if(0x01&SouthSwich)
	{
		DEBUGOUT("SOUTH_UPDATA:");
		for(uint8_t i=0;i<10;i++)
		{
			DEBUGOUT("%02X ",uTmpData[i]);
		}
		DEBUGOUT("\r\n");
	}*/
	//====================================================================================

	return 0;
}

/****************************************************************************
* ��    �ƣ�SouthUpdataSoftChange()
* ��    �ܣ���������������
* ��ڲ�����

* ���ڲ�������
* ��    ��: ��
****************************************************************************/
void SouthUpdataSoftChange(void)
{
	int16_t	iResult[MAX_device];			// �����ѯ�������
	for(uint8_t i=0;i<g_DeviceSouth.device_sum;i++)
	{
	    memset(uRecBuffer,0,256);
		
		iResult[i] = ComMasterRead(&g_sMaster,g_DeviceSouth.device_inf[i].addr,DISCOVERY_REPORT,0,0,uRecBuffer,NULL);
		if(iResult[i]>0)
		{
            HwSoftChange(g_DeviceSouth.device_inf[i].addr,uRecBuffer);
			sleep(2);	
		}
	}
}
/****************************************************************************
* ��    �ƣ�SouthSingleUpdata()
* ��    �ܣ�������������
* ��ڲ�����

* ���ڲ�������
* ��    ��: ��
****************************************************************************/
void SouthSingleUpdata(void)
{
	int16_t	iResult[MAX_device];			// �����ѯ�������
    uint16_t s_uSendseq=0; // �������
    uint8_t uReadDataLen=0;   //��ȡ���ݳ���
    uint8_t uTmpAddr;
	uint8_t  uTmpData[256];

	
	for(uint8_t i=0;i<g_DeviceSouth.device_sum;i++)
	{
		uReadDataLen=0;
        FEED_DOG();     // ι��
	    if(0!=memcmp(g_DeviceSoft.cDeviceSoft[i],g_DT1000Updata.version,17))
		{
			uTmpAddr = g_DeviceSouth.device_inf[i].addr;

			DEBUGOUT("Sigle Upgrade Device Addr��%d\n",uTmpAddr);
			memset(uTmpData,0,256);
			memcpy(&uTmpData,&g_DT1000Updata.version,17);
            msleep(1);
			memset(uRecBuffer,0,256);
			
			iResult[uTmpAddr] = ComMasterRead(&g_sMaster,uTmpAddr,DT1000UPDATE_START,0,0,uRecBuffer,uTmpData);
			if((iResult[uTmpAddr]>0))
            {
                sleep(3);
				for(;(s_uSendseq*240) < g_DT1000Updata.nDataLen;s_uSendseq++)
				{
                    memset(uTmpData,0,256);
                    FEED_DOG();     // ι��
					uTmpData[0] = s_uSendseq>>8;
	                uTmpData[1] = s_uSendseq&0XFF;

	                uReadDataLen =DT1000ReadDataflashData(s_uSendseq,&uTmpData[3],240);
					msleep(1);
	                uTmpData[2] = uReadDataLen;  //���ݳ���

                    memset(uRecBuffer,0,256);
					iResult[uTmpAddr] = ComMasterRead(&g_sMaster,uTmpAddr,DT1000UPDATE_DATA,0,uReadDataLen+3,uRecBuffer,uTmpData);

					if(iResult[uTmpAddr]>0)
					{

						DEBUGOUT("\nSigle Upgrade: %d Frame\r\n",s_uSendseq);
					}else
					{
						DEBUGOUT("\nSigle Upgrade: %d Frame Fail\r\n",s_uSendseq);
						DEBUGOUT("\nQuit Sigle Upgrade��%d!!!\r\n",uTmpAddr);
						break;
					}
					sleep(1);
				}
				
				sleep(1);
				
				memset(uTmpData,0,256);
				uTmpData[0] = s_uSendseq>>8;
	            uTmpData[1] = s_uSendseq&0XFF;
				
				memset(uRecBuffer,0,256);
				iResult[uTmpAddr] = ComMasterRead(&g_sMaster,uTmpAddr,DT1000UPDATE_END,0,0,uRecBuffer,uTmpData);
				if(iResult[uTmpAddr]>0)
				{
					DEBUGOUT("\nSigle Upgrade End!!!\r\n");
					
					//SouthUpdataSoftChange();
					//memset(uRecBuffer,0,256);
					//FEED_DOG();     // ι��
					//sleep(30);
					/*iResult[uTmpAddr] = ComMasterRead(&g_sMaster,uTmpAddr,0x3B,0,0,uRecBuffer,NULL);
					if(uTmpAddr > 0)
					{
						HwSoftChange(uTmpAddr,uRecBuffer);
					}
					
					sleep(1);*/
				}
			}
			s_uSendseq=0;
			sleep(3);
		}
	}
}

/****************************************************************************
* ��    �ƣ�SouthLogStart()
* ��    �ܣ�������־��ʼ
* ��ڲ�����

* ���ڲ�������
* ��    ��: ��
****************************************************************************/
uint32_t SouthLogStart(void)
{
    int16_t iResult;
	uint8_t uTmpData[2];
	U32_F_Char_Convert uTemp;

	uTmpData[0] = S_FILE_EXPORT;  //�ļ������ӹ�����
	uTmpData[1] = g_DT1000Updata.uData;   //��������
	iResult = ComMasterRead(&g_sMaster,g_DT1000Updata.uDevAddr,DT1000LOG_START,0,0,uRecBuffer,uTmpData);
	
	if((iResult > 0) && (uRecBuffer[1]== DT1000LOG_START))
	{
        if(0 == uRecBuffer[4]) //�жϱ�ƻظ������ļ���С�Ƿ���ɣ�0x01��ʾ��Ƽ����ļ���С��ɲ�������0x00��ʾ�����ļ���Сδ��ɷ���
        {
             return 0;
			 
		}else
		{    
		     uTemp.c[0] = uRecBuffer[8];
			 uTemp.c[1] = uRecBuffer[7];
			 uTemp.c[2] = uRecBuffer[6];
			 uTemp.c[3] = uRecBuffer[5];
		     g_DT1000Updata.nDataLen = uTemp.u;
			 DEBUGOUT("\nLogFileLen: %d    ",g_DT1000Updata.nDataLen);
			 return  g_DT1000Updata.nDataLen;
		}
	}

	return 0;
}
/****************************************************************************
* ��    �ƣ�SouthLogTransmission()
* ��    �ܣ�������־���ݴ���
* ��ڲ�����

* ���ڲ�������
* ��    ��: ��
****************************************************************************/
uint8_t SouthLogTransmission(uint16_t uFrameCount)
{
    int16_t  iResult;
	uint8_t uTmpData[4];
	uint8_t uDataLen;
	Uint_Char_Convert uTemp;
	
	g_DT1000Updata.CRC=0xFFFF;
    FEED_DOG();
	
    uTmpData[0] = S_FILE_EXPORT;   //�ļ���������
    uTmpData[1] = g_DT1000Updata.uData;  //����
    uTmpData[2] = uFrameCount>>8;   //����Ÿ�λ
  	uTmpData[3] = uFrameCount&0xFF;  //����ŵ�λ
  	iResult = ComMasterRead(&g_sMaster,g_DT1000Updata.uDevAddr,DT1000LOG_DATA,0,0,uRecBuffer,uTmpData);
	
	if((iResult > 0) && (uRecBuffer[1] == DT1000LOG_DATA))
	{
         uTemp.c[0] = uRecBuffer[5];
		 uTemp.c[1] = uRecBuffer[4];
		 
		 if(uTemp.u == uFrameCount)           //��������յİ���ŶԱȣ���ȷ��ת����ƽ̨
		 { 
//			  DEBUGOUT("\nSend Package : %d   Rec Package:%d   ",uTemp.u ,uFrameCount);
			 g_sIecSend.format.maddrL = (uFrameCount - 1)&0xFF;  //����ŵ�λ
             g_sIecSend.format.maddrM = (uFrameCount - 1)>>8;  //����Ÿ�λ
//			 g_sIecSend.format.maddrL = uTmpData[3];  //����ŵ�λ
//             g_sIecSend.format.maddrM = uTmpData[2];  //����Ÿ�λ
             g_sIecSend.format.maddrH = 0x00;
			 uDataLen = uRecBuffer[6];   //��ȡ��־֡����
			  
             g_DT1000Updata.CRC = CalculateCRC(&uRecBuffer[7],uDataLen);
			 g_sIecSend.format.data[uDataLen]   = g_DT1000Updata.CRC&0xFF;
			 g_sIecSend.format.data[uDataLen+1] = g_DT1000Updata.CRC<<8;
//			 DEBUGOUT("PackageDataLen: %d  g_DT1000Updata.CRC:%04X\n",uDataLen,g_DT1000Updata.CRC);
			 //DataFlash_Write(DATAFLASH_DT1000_LOG+uFrameSum*i,&uRecBuffer[7],uRecBuffer[6]);
			 memcpy(g_sIecSend.format.data,&uRecBuffer[7],uDataLen);
			 msleep(5);
			
			 IecCreateFrameI(P_FILE_INOUT,0x01,R_DATA_TRANS,uDataLen+2,&g_sIecSend);    // ��־���ݴ���
		 }
	}
    else
	{
         return 0;
	}
		
	return 1;
}
/****************************************************************************
* ��    �ƣ�SouthLogEnd()
* ��    �ܣ�������־�������
* ��ڲ�����

* ���ڲ�������
* ��    ��: ��
****************************************************************************/
void SouthLogEnd(void)
{
    int16_t iResult;
	uint8_t uTmpData[2];
	Uint_Char_Convert uTmpCrc;
	
	uTmpData[0] = S_FILE_EXPORT;  //��־�����ӹ�����
	uTmpData[1] = g_DT1000Updata.uData;   //��������

    iResult = ComMasterRead(&g_sMaster,g_DT1000Updata.uDevAddr,DT1000LOG_END,0,0,uRecBuffer,uTmpData);
    DEBUGOUT("iResult:%d\n",iResult);
	if(iResult > 0)
	{
		uTmpCrc.c[1] = uRecBuffer[4];  //���յ�����ļ�CRC��λ
		uTmpCrc.c[0] = uRecBuffer[5];  //���յ�����ļ�CRC��λ

//        DEBUGOUT("g_DT1000Updata.CRC:%04X  uTmpCrc.u:%04X",g_DT1000Updata.CRC,uTmpCrc.u);
		if(g_DT1000Updata.CRC == uTmpCrc.u)
		{
             //�����ܰ���
		     g_sIecSend.format.maddrL = g_DT1000Updata.frame_sum&0xFF;  //����ŵ�λ
		     g_sIecSend.format.maddrM = g_DT1000Updata.frame_sum<<8;  //����Ÿ�λ
		     g_sIecSend.format.maddrH = 0x00;

			 //�ļ�CRC
			 g_sIecSend.format.data[0] = uRecBuffer[5];
			 g_sIecSend.format.data[1] = uRecBuffer[4];

			 IecCreateFrameI(P_FILE_INOUT,0x01,R_TRANS_FINISH,2,&g_sIecSend);   // ��־���ݴ���	
			 g_DT1000Updata.CRC=0xFFFF;
		}
	}
	return;
}
//===============================================================================
OS_STK TaskSouthInquireStk[TaskSouthInquireStkSize]@0x20004000;	      // ���������ջ��С
/****************************************************************************
* ��    �ƣ�TaskSouthInquire()
* ��    �ܣ������ѯ����
* ��ڲ�����

* ���ڲ�������
* ��    ��: ��
****************************************************************************/
#define  SOUTH_INQUIRE         0    // 0���������ݲ�ѯ��
#define  SOUTH_DISCORY         1    // 1�������Է��֣�
#define  SOUTH_POLL            2    // 2�������Է��֣����ݲ�ѯ���澯��ѯ��ѯ��
#define  SOUTH_TIME            3    // 3����ȡʱ��
#define  SOUTH_UPDATA          4    // 4: �������
void TaskSouthInquire(void *p)
{
    p = p;

    uint32_t uRoundStartTime;    		// ��ѭ����ʼ��ʱ���
    uint32_t uRoundNowTime;      		// ��ѭ�����ڵ�ʱ���
	uint32_t uDiscoryStartTime;
	uint32_t uDiscoryNowTime;
	
    uint8_t  uRoundEnd;                	// �����ѯһ��ѭ������
    uint8_t err = 0;
	static uint8_t  uSouthStep = SOUTH_INQUIRE;      // �����ѯ����
    static uint8_t  uSigleUpdate=0;
	uint8_t uLogOutState;    			//��־����״̬
	U32_F_Char_Convert ctemp;
	uint16_t uFrameCount = 0;
	uint16_t uRecFrame;  				//��ƽ̨����ȷ��֡
    uint8_t TimeCount=0;

    pUartLock = OSMutexCreate(4,&err);	// ����ʼ��

    // ��վ�ṹ���ʼ��
    g_sMaster.pComLock = pUartLock;		// ������ָ��
    g_sMaster.iComFd = uart3;			// �����ļ���
    g_sMaster.uRecLostMax = 3;			// ���֡������������Ϊ����              ----->>>�趨������еĶ�֡��������������Ϊ����
    g_sMaster.uRecCrcMax = 3;			// ���CRC���������������Ϊ����           ----->>>�趨������е�CRCУ���������������Ϊ����
    g_sMaster.uFailTimeOut = 3000;		// ��֡��ʱ���������ٴη�������          ----->>>�趨��֡���ٴη��͵ļ��ʱ�䣬��1msΪ��ʱ��λ����ԭ500
    g_sMaster.u1BTimeOut = 6000;		//��ѯ1B��ʱ�������ʱʱ��
    g_sMaster.uSuccessDelay = 200;		// һ֡�ɹ�����ʱ����ʱ���ѯ��һ֡        ----->>>�趨һ֡��ѯ�ɹ����ѯ��һ֡�ļ��ʱ��
    g_sMaster.sSuccessTime = 0;			// %�ڲ�%һ֡��ѯ�ɹ�ʱ��ʱ���¼
    g_sMaster.uPreAddr = 0;				// -%�ڲ�%-�ѷ��Ͳ�ѯ�豸�ĵ�ַ            ----->�ڲ�ʹ�ã���ʼ��Ϊ0
    g_sMaster.uPreAddr = 0;				// -%�ڲ�%-�ѷ��͵Ĺ�����                  ----->�ڲ�ʹ�ã���ʼ��Ϊ0
    sSouth.uBaudRate = BAUDRATE_9600;  	// ������9600
    uRoundStartTime = OSTimeGet();   	// ��ѭ����ʼʱ��
	uDiscoryStartTime = OSTimeGet();  	//�Է�����ʼʱ��
	
    while(1)
    {
        g_uTimeNow = OSTimeGet();   	// ��ȡ��ǰ��ʱ���

        if(g_LoggerRun.update)  		// ����������
        {
            sleep(10);
            continue;
        }
		
		/************************���˶�ȡ*485**********************************/
        #if(1==USE_485_DEBUG)
        if(Rs485DebugRead())
        {
        	DEBUGOUT("\r\nRs485DebugRead!!!\r\n");
            sleep(10);
            continue;
        }
        #endif
		
		/************************�������**************************************/
        if(g_LoggerRun.uFileIO_status == 0x01)
        {
			DEBUGOUT("\r\nSouth Upgrade Start!!!\r\n");
			SouthBroadcastUpdata();			//�㲥����
			FEED_DOG();     				// ι��
		    sleep(50);                      //��ʱ��� from B31028&B31029��ԭ����Ϊ30
			SouthUpdataSoftChange();		//����������汾���
			
			uSigleUpdate=1;
		}

		if((uSigleUpdate == 1) && (g_LoggerRun.uFileIO_status == 0x01))
		{
			SouthSingleUpdata();			//��������
			sleep(50);                      //��ʱ��� from B31028&B31029��ԭ����Ϊ30
			g_LoggerRun.uFileIO_status = 0x00;
			uSigleUpdate=0;
		}

		/******************************�����־����********************************/
		if(S_FILE_EXPORT == g_LoggerRun.uFileIO_status)
		{
			DEBUGOUT("\r\nSouth Log Output!!!\r\n");
			for (uint8_t i = 0;  i < 10;i++)
			{
				g_DT1000Updata.nDataLen = SouthLogStart();   	//������־������ȥ��ȡ�����־����
				
				if(g_DT1000Updata.nDataLen > 0)
				{
					
				     //��Ϣ���ַ������־��������
					 g_sIecSend.format.maddrL = S_FILE_EXPORT;
                     g_sIecSend.format.maddrM = 0x00;
                     g_sIecSend.format.maddrH = 0x00;

		 			 //��ȡ�ļ�����
		 			 ctemp.u = g_DT1000Updata.nDataLen;
					 g_sIecSend.format.data[0] = ctemp.c[0];  	//104Э����С��ģʽ
					 g_sIecSend.format.data[1] = ctemp.c[1];
					 g_sIecSend.format.data[2] = ctemp.c[2];
					 g_sIecSend.format.data[3] = ctemp.c[3];
					 g_sIecSend.format.data[4] = 0xC8;       	//������200�ֽ����ݴ���

					 if((g_DT1000Updata.nDataLen%200) != 0)  	//�ж��ļ����ȳ���200�Ƿ���������������֡����
					 {
						 g_DT1000Updata.frame_sum = g_DT1000Updata.nDataLen/200 + 1;
					 }
					 else
					 {
						 g_DT1000Updata.frame_sum = g_DT1000Updata.nDataLen/200;
					 }
					
					 uFrameCount = g_DT1000Updata.frame_sum;
					 IecCreateFrameI(P_FILE_INOUT,0x01,R_TRANS_START_ACK,5,&g_sIecSend);
                     break;            
				}
				else
				{     
				    sleep(5);
				}
			}
		   sleep(2);

		   while (uFrameCount)
		   {
			   TimeCount=0;
			   uLogOutState = SouthLogTransmission(g_DT1000Updata.frame_sum - uFrameCount + 1); //��־����

			   if(uLogOutState > 0)
			   {
				   while(1)
				   {
                       sleep(1);
					   if((g_sIEC.recv.format.type == P_FILE_INOUT) && (g_sIEC.recv.format.reasonL == R_DATA_TRANS))  //����ȷ��֡���ͼ�����ԭ��
					   {
						   uRecFrame = (g_sIEC.recv.format.data[0] | g_sIEC.recv.format.data[1]<<8);  //����֡���
						   if(uRecFrame == (g_DT1000Updata.frame_sum - uFrameCount))   //�ȶԽ����뷢������Ƿ���ͬ
						   {
							   uFrameCount--;
							   break;
						   } 
 					   }
					   
			   		   TimeCount++;
					   if(TimeCount>=30)   //���ƽ̨ȷ��֡ʱ�䳬��30s���˳���־����
					   {
						   TimeCount=0;
						   uFrameCount=0;
						   break;
					   }
				  }
			   }
			   else
			   {
                   break;
			   }    
		   }
		   
           if((uFrameCount == 0) && (g_DT1000Updata.nDataLen != 0))
           {
			   SouthLogEnd();	//�����־��������
		   }
		   g_LoggerRun.uFileIO_status = 0;
		}
		g_LoggerRun.uFileIO_status = 0;
		//--------------------------------------------------------------------------------
        TimedReboot(120);    //��ʱ����
        AlarmCheckout();     //�澯���
        //--------------------------------------------------------------------------------
        //�����ѯ����
        //--------------------------------------------------------------------------------
        switch(uSouthStep)
        {
        case SOUTH_INQUIRE:
            uRoundEnd = SouthInquire();  // ��ѯ�����豸
            if(SOUTH_WORK == uRoundEnd || SOUTH_WAIT == uRoundEnd)
            {
                msleep(100);
            }
            else if(SOUTH_OVER == uRoundEnd)
            {
                sSouth.uBaudRate = SetBaudRate(sSouth.uBaudRate,BAUDRATE_9600);// �������л���9600���������������豸
				uSouthStep = SOUTH_POLL;
                msleep(100);
            }
            else if(SOUTH_EMPTY == uRoundEnd)
            {
                sSouth.uBaudRate = SetBaudRate(sSouth.uBaudRate,BAUDRATE_9600);// �������л���9600���������������豸
                uSouthStep = SOUTH_DISCORY;
                DEBUGOUT("South Discory!!!\r\n");
            }	
            break;

        case SOUTH_DISCORY:
            uRoundEnd = SlaveDeviceAutoAllocation();   // ��������豸�������ַ
            //sleep(3);
            if(SEARCH_END == uRoundEnd) // һ��ѭ����ѯ����������Ҫ����
            {
				uDiscoryStartTime = g_uTimeNow;
				uSouthStep = SOUTH_POLL;
            }
            else if(SEARCH_END_IMPORT == uRoundEnd) // һ��ѭ����ѯ��������Ҫ����
            {
				uDiscoryStartTime = g_uTimeNow;
				uSouthStep = SOUTH_TIME;
            }
            else if(SEARCH_NOTCONNECT == uRoundEnd) // ƽ̨δ����
            {
                if(RUNNING_EMPTY == g_LoggerRun.run_status)   // ������ g_sIecPointCount
                {
                    OSTimeDlyHMSM(0,0,10,0);
                }
                else
                {
                    uSouthStep = SOUTH_POLL;
                }
            }
            else
            {
            	DEBUGOUT("South Discory uRoundEnd��%d!!!\r\n",uRoundEnd);
                msleep(100);
            }
            break;

        case SOUTH_POLL:
            uRoundNowTime = g_uTimeNow;   // ��ѭ������ʱ��
			uDiscoryNowTime = g_uTimeNow;  //�Է�������ʱ��
			if(TimeGapJudge(uDiscoryStartTime,uDiscoryNowTime,DISCORY_ROUND_TIME))
		    {
                 if(TimeGapJudge(uRoundStartTime,uRoundNowTime,SLAVE_ROUND_TIME))
                 {
					 sleep(1);
					 if(NORTH_OK == g_LoggerRun.north_status)
					 {
						 uRoundEnd = SouthInquireAlarm();  // ��ѯ�����豸 
					 }
                 }
                 else
                 {
                      uRoundStartTime = g_uTimeNow;   // ��ѭ����ʼʱ��
                      uSouthStep = SOUTH_INQUIRE;
                      DEBUGOUT("South Inquire !!!\r\n");
                 }
		    }
		    else
			{
				uDiscoryStartTime=g_uTimeNow;
                if(GetRecordAllow())
				{
					// ��������豸״̬���������豸��û��ȫ����֡ 
					if(0 == CheckDevState()) //&& g_DeviceSouth.yx_sum && g_DeviceSouth.yc_sum)
					{
						//OSMutexPend(pRecordLock,0,&err);//�����ź���
						uSaveSouthLog(sDateTime,SOUTH_DATA_LOG,NULL,20,0);
						//��ܷ��������ⵥ939��ң����Ϊ0���ض�eep��ң��ռ�Ϊ��������
						if(!g_DeviceSouth.yc_sum)
						{
							EepReadData(EEP_LOGGER_DEVICE_INF_HEAD,(uint8_t *)&g_DeviceSouth,sizeof(g_DeviceSouth),&g_DeviceSouth.CRC);// �豸��Ϣ�͵����Ϣ��ȡ
						}
						if(NULL == IEC104_DATA_YC)
						{
							Reboot();
						}
						DEBUGOUT("YCSum = %d   YXSum = %d !!!\r\n",g_DeviceSouth.yc_sum,g_DeviceSouth.yx_sum);
						RecordHistory(IEC104_DATA_YX,(uint8_t *)IEC104_DATA_YC,g_DeviceSouth.yx_sum,g_DeviceSouth.yc_sum);// ��������
						//OSMutexPost(pRecordLock);   //�ͷ��ź���
					}
				}
				else
				{
					if(NORTH_OK == g_LoggerRun.north_status)// ���ӷ��������
					{
						if(0 == (g_LoggerRun.err&err_power_off))
						{
							SetRecordAllow(1);  // ����洢��ʷ����
						}
					}
				}

				uSouthStep = SOUTH_DISCORY;
				DEBUGOUT("South Discory!!!\r\n");
			}
			
            break;
        case SOUTH_TIME:
            if(RUNNING_WORK_READ == g_LoggerRun.run_status)
            {
                uRoundNowTime = g_uTimeNow;   // ��ѭ������ʱ��
                uDiscoryNowTime = g_uTimeNow;   // ��ѭ������ʱ��
                uSouthStep = SOUTH_INQUIRE;
                DEBUGOUT("South Inquire !!!\r\n");
            }
            else
            {
                msleep(200);
            }
            break;
        default:
            uSouthStep = SOUTH_POLL;
            break;
        }
    }
}
/******************************************************************************
* ��    �ƣ�SouthWriteYk()
* ��    �ܣ�����ң��
* ��ڲ�����
*           ��      ��

* ���ڲ�����
* ��    ��:
******************************************************************************/
uint8_t SouthWriteYk(void)
{
    uint16_t uValue;       // �Ĵ���ֵ
    uint16_t uYkCount;
    uint16_t uIecYk=0;      // IEC104���ң�ص�ַ��
    uint16_t uModbusAddr=0; // ң�ص��Ӧ��MODBUS�Ĵ�����ַ
    uint16_t j;
    uint8_t  uRelDevAddr;   // �����豸��Ե�ַ
    uint8_t  uRelNum;       // �豸��Ե���
    uint8_t  uTableYkSum;   // ���ң�ص�����
    uint8_t  uYkPoint;      // �豸�ĵڼ���ң�ص�
    int8_t   iResult;
    /*
    �磬�豸1��ң����ʼ��ַΪ0x6004,ƽ̨�·���ң�ص�Ϊ0x6007
    ��Ϊ�豸1�ĵ�4��ң�ص�
    */
    if(NULL==IEC104_DATA_YK)
    {
        return 0;
    }
    for(uYkCount=0; uYkCount<g_DeviceSouth.yk_sum; uYkCount++)
    {
        if(0!=IEC104_DATA_YK[uYkCount])
        {
//            if(0x80&IEC104_DATA_YK[uYkCount])
//            {
//                IEC104_DATA_YK[uYkCount] = 0x00; //��������ػ�
//            }
            uValue = IEC104_DATA_YK[uYkCount] & 0x7F;
            uIecYk = uYkCount + 0x6001;
            break;
        }
    }

    if(uIecYk)
    {
        for(uRelDevAddr=0; uRelDevAddr<MAX_device; uRelDevAddr++)  // �����豸
        {
            uRelNum = g_DeviceSouth.device_inf[uRelDevAddr].rel_num;
            uTableYkSum = g_sIecPointCount[uRelNum].uYkSum;

            if(g_DeviceSouth.device_inf[uRelDevAddr].yk_start_addr<=uIecYk && uIecYk<(g_DeviceSouth.device_inf[uRelDevAddr].yk_start_addr+uTableYkSum))
            {
                uYkPoint = uIecYk - g_DeviceSouth.device_inf[uRelDevAddr].yk_start_addr + 1;
                break;
            }
        }
        if(uRelDevAddr<MAX_device) // ���������豸
        {
            for(j=0; j<g_DeviceSouth.protocol[uRelNum].mess_point_sum; j++) // ����MODBUS��ַ
            {
                if(TYPE_YK==g_pRegPoint[uRelNum][j].reg_type.type.mess)
                {
                    uYkPoint--;
                    if(0==uYkPoint)
                    {
                        uModbusAddr = g_pRegPoint[uRelNum][j].reg_addr;
                        break;
                    }
                }
            }
        }
    }

    if(uModbusAddr)
    {
        if(0x02==g_DeviceSouth.device_inf[uRelDevAddr].protocol_type) // ��׼MODBUS
        {
            if(uModbusAddr>=40000)   // �����ݱ��ּĴ���40001~49999
            {
                uModbusAddr -= 40000;
            }
            else if(uModbusAddr>=30000)  // ������Ĵ���30001~39999
            {
                uModbusAddr -= 30000;
            }
        }

        iResult = ComMasterWrite(&g_sMaster,g_DeviceSouth.device_inf[uRelDevAddr].addr,0x06,uModbusAddr,1,&uValue);
        if(iResult<0)
        {
            DEBUGOUT("ң��ʧ��%d",g_DeviceSouth.device_inf[uRelDevAddr].addr);
        }
        /*else
        {
            iResult = ComMasterRead(&g_sMaster,g_DeviceSouth.device_inf[uRelDevAddr].addr,0x03,uModbusAddr,1,&uRecBuffer);
            if(iResult>0 || -MODBUS_ILLEGAL_ADDR==iReadResult)  // ��ȡ�ɹ�
            {

            }
        }*/

        IEC104_DATA_YK[uYkCount] = 0x00;

        return 0;
    }

    return 0;
}

/******************************************************************************
* ��    �ƣ�SouthSdModbus()
* ��    �ܣ��������
* ��ڲ�����
*           uModbusAddr������modbus��ַ
*           uRelDevAddr������豸��ַ
*
* ���ڲ�����
* ��    ��:
******************************************************************************/
uint8_t SouthSdModbus(uint16_t uModbusAddr,uint8_t  uRelDevAddr,uint16_t uSdCount,uint32_t uValue,uint8_t uReg_count)
{
    int8_t   iResult;
	uint16_t uDataTemp[2];

	if(uModbusAddr)
	{
		if(1==uReg_count)
		{
			uDataTemp[0] = uValue & 0xFFFF;

		    iResult = ComMasterWrite(&g_sMaster,g_DeviceSouth.device_inf[uRelDevAddr].addr,0x06,uModbusAddr,uReg_count,uDataTemp);

			if(iResult<0)
			{
				 DEBUGOUT("######SdDevice��%d RegAddr��%d SdFailFailFail!!!######",g_DeviceSouth.device_inf[uRelDevAddr].addr,uModbusAddr);
			}
			else
			{
				DEBUGOUT("######SdDevice��%d RegAddr��%d Value��%d######\n",g_DeviceSouth.device_inf[uRelDevAddr].addr,uModbusAddr,uValue);
			}
			IEC104_DATA_SD[uSdCount] = 0x00;
		}
		else if(2==uReg_count)
		{
			uDataTemp[0] = uValue>>16;
			uDataTemp[1] = uValue & 0xFFFF;

		    iResult = ComMasterWrite(&g_sMaster,g_DeviceSouth.device_inf[uRelDevAddr].addr,0x10,uModbusAddr,uReg_count,uDataTemp);
		
			if(iResult<0)
			{
				 DEBUGOUT("######SdDevice��%d RegAddr��%d SdFailFailFail!!!######",g_DeviceSouth.device_inf[uRelDevAddr].addr,uModbusAddr);
			}
			else
			{
				 DEBUGOUT("######SdDevice��%d RegAddr��%d Value��0x%08X######\n",g_DeviceSouth.device_inf[uRelDevAddr].addr,uModbusAddr,uValue);
			}
			IEC104_DATA_SD[uSdCount] = 0x00;
		}
	}

	return 0;
}
/****************************************************************************
* ��	 �ƣ�SouthIecSd()
* ��	 �ܣ����104��ַ���ҵ���е�modbus��ַ
* ��ڲ�����uIecSd��IEC104�������ַ��
*           uValue��104����ַ��Ӧ��ֵ
*           uSdCount��������
*           uSdSum���������
* ���ڲ�����
* ��	 ��: ��
****************************************************************************/
uint16_t SouthIecSd(uint16_t uIecSd,uint32_t uValue,uint16_t uSdCount,uint8_t uSdSum)
{
     uint8_t  uRelNum;       	//�豸��Ե���
	 uint16_t uModbusAddr = 0; 	// ң�ص��Ӧ��MODBUS�Ĵ�����ַ
     uint16_t j;
	 uint8_t  uRelDevAddr;   	// �����豸��Ե�ַ
	 uint8_t  uSdPoint;      	// �豸�ĵڼ�������
	 uint8_t  uReg_count;   	//�źŵ�ļĴ�������

	 if(uIecSd)
     {
         for(uRelDevAddr = 0; uRelDevAddr < MAX_device; uRelDevAddr++)  // �����豸
         {
             uRelNum = g_DeviceSouth.device_inf[uRelDevAddr].rel_num;
             //uTableSdSum = g_sIecPointCount[uRelNum].uSdSum;
             if((g_DeviceSouth.device_inf[uRelDevAddr].sd_start_addr <= uIecSd) 
			 	&& (uIecSd < (g_DeviceSouth.device_inf[uRelDevAddr].sd_start_addr+(g_DeviceSouth.sd_sum/g_DeviceSouth.device_sum))))
             {
                 uSdPoint = uIecSd - g_DeviceSouth.device_inf[uRelDevAddr].sd_start_addr + 1;
                 break;
             }
         }

         if(uRelDevAddr < MAX_device) // ���������豸
         {
             for(j = 0; j < g_DeviceSouth.protocol[uRelNum].mess_point_sum; j++) // ����MODBUS��ַ
             {
                 if(TYPE_SD == g_pRegPoint[uRelNum][j].reg_type.type.mess)
                 {
                     uSdPoint--;
                     if(0 == uSdPoint)
                     {
						 uModbusAddr = g_pRegPoint[uRelNum][j].reg_addr;
						 if(1==g_pRegPoint[uRelNum][j].reg_count)
						 {
							 uReg_count=1;
							 SouthSdModbus(uModbusAddr,uRelDevAddr,uSdCount,uValue,uReg_count);
						 }
						 else if(2==g_pRegPoint[uRelNum][j].reg_count)
						 {
							 uReg_count=2;
							 SouthSdModbus(uModbusAddr,uRelDevAddr,uSdCount,uValue,uReg_count);
						 }
					     continue;
                     }
                }
            }
         }
     }
	 return 0;
}
/******************************************************************************
* ��    �ƣ�SouthWriteSD()
* ��    �ܣ��������
* ��ڲ�����
*           ��      ��
*
* ���ڲ�����
* ��    ��:
******************************************************************************/
uint8_t SouthWriteSD(void)
{
    uint32_t uValue;       // �Ĵ���ֵ
    uint16_t uIecSd = 0;      // IEC104�������ַ��
	uint8_t  uSdSum = 0;     //�������
    /*
    �磬�豸1��ң����ʼ��ַΪ0x6201,ƽ̨�·���ң�ص�Ϊ0x6203
    ��Ϊ�豸1�ĵ�3��ң�ص�
    */
    if(NULL == IEC104_DATA_SD)
    {
//        DEBUGOUT("IEC104_DATA_SDΪ��\r\n");
		return 0;
    }

    uSdSum = IEC104_DATA_SD[0];

    for(uint8_t k = 0;k < uSdSum;k++)
    {
    	uIecSd = IEC104_DATA_SD[2*k+1];
		uValue = IEC104_DATA_SD[(k+1)*2];
//		DEBUGOUT("uValue:%d uIecSd��%X k = %d\r\n",uIecSd,uValue,k);
    	SouthIecSd(uIecSd,uValue,k+1,uSdSum);
    	msleep(5);
    	IEC104_DATA_SD[2*k+1] = 0;
    	IEC104_DATA_SD[(k+1)*2] = 0;
    }
    IEC104_DATA_SD[0] = 0;

    return 0;
}

/******************************************************************************
* ��    �ƣ�SouthReadSD()
* ��    �ܣ��������
* ��ڲ�����
*           ��      ��
*
* ���ڲ�����
* ��    ��:
******************************************************************************/
uint8_t SouthReadSD(void)
{
    uint16_t uValue;       		// �Ĵ���ֵ
    uint16_t uSdCount = 0;
    uint16_t uIecSd = 0;      	// IEC104�������ַ��
    uint16_t uModbusAddr = 0; 	// ң�ص��Ӧ��MODBUS�Ĵ�����ַ
    uint16_t j,i;
    uint8_t  uRelDevAddr = 0;   	// �����豸��Ե�ַ
    uint8_t  uRelNum,sRegist_Count = 0;       	// �豸��Ե���
    uint8_t  uSdPoint;      	// �豸�ĵڼ�������
	uint8_t  uIntervalPoint = 2;
	U32_F_Char_Convert ctemp;
	uint32_t uValue_Temp = 0;       		// �Ĵ���ֵ
	DEBUGOUT("YT ACQUIRY TASK!!! \r\n");
	/* ��,�豸1��ң����ʼ��ַΪ0x6004,ƽ̨�·���ң�ص�Ϊ0x6007
    *	��Ϊ�豸1�ĵ�4��ң�ص�
    */
    if(NULL == IEC104_DATA_SD)
    {
    	DEBUGOUT("��ң���ռ� \r\n");
        return 0;
    }

	uValue = IEC104_DATA_SD[0];				//��ѯ��ң��������
	uIecSd = IEC104_DATA_SD[1];	//��ѯ��104��ַ

//	DEBUGOUT("uIecSd = 0x%d  uValue = %d!!!\r\n",uIecSd,uValue);
	if(uIecSd < 0x6201)
		return 0;

	/*********************����ƽ̨��ַ��ȡ�����ң����ַ***********************/
	memset(IEC104_DATA_SD,0,sizeof(IEC104_DATA_SD));
	uValue = uIecSd+uValue;
	for(;uIecSd < uValue;uIecSd++)
	{
		for(;uRelDevAddr < MAX_device; uRelDevAddr++)	//�����豸
		{
			uRelNum = g_DeviceSouth.device_inf[uRelDevAddr].rel_num;	//�豸����Ե���
			if(g_DeviceSouth.device_inf[uRelDevAddr].sd_start_addr != 0)
			{
				if((g_DeviceSouth.device_inf[uRelDevAddr].sd_start_addr <= uIecSd)
					&& (uIecSd < (g_DeviceSouth.device_inf[uRelDevAddr].sd_start_addr + g_sIecPointCount[uRelNum].uSdSum)))
				{
//					printf("/**** Pinnet END g_sIecPointCount[%d].uSdSum = %d\r\n",uRelNum,g_sIecPointCount[uRelNum].uSdSum);
					uSdPoint = uIecSd - g_DeviceSouth.device_inf[uRelDevAddr].sd_start_addr + 1;
					break;
				}
			}
		}

		if(uRelDevAddr < MAX_device) 		//���������豸
        {
            for(j = 0; j < g_DeviceSouth.protocol[uRelNum].mess_point_sum; j++)	//����MODBUS��ַ
            {
				if(TYPE_SD == g_pRegPoint[uRelNum][j].reg_type.type.mess)
                {
					uSdPoint--;
					if(0 == uSdPoint)
                    {
                        uModbusAddr = g_pRegPoint[uRelNum][j].reg_addr;
                        IEC104_DATA_SD[uSdCount] = (g_pRegPoint[uRelNum][j].reg_count == 1)?uModbusAddr:(uModbusAddr|(1<<16));
//                        printf("IEC104_DATA_SD[%d] = 0x%08X\r\n",uSdCount,IEC104_DATA_SD[uSdCount]);
                        uSdCount++;
                    }
                }
            }
        }
	}

	/*********************************ð��*************************************/
	if(0 != uSdCount)
	{
		for (j = 0; j < uSdCount - 1; j++)
		{
			for (i = 0; i < uSdCount - 1 - j; i++)
			{
				if(((IEC104_DATA_SD[i])<<16) > ((IEC104_DATA_SD[i+1])<<16))
				{
					memcpy(&uValue_Temp,&IEC104_DATA_SD[i],sizeof(uValue_Temp));
					memcpy(&IEC104_DATA_SD[i],&IEC104_DATA_SD[i+1],sizeof(uValue_Temp));
					memcpy(&IEC104_DATA_SD[i+1],&uValue_Temp,sizeof(uValue_Temp));
				}
			}
		}
	}
	else
	{
		DEBUGOUT("Not YT Point!!!\r\n");
		return 0;
	}

	/******************************�ֶβ�ѯ************************************/
	uValue = 0;
	for(i = 1; i < (uSdCount+1); i++)
	{
		uValue++;
		sRegist_Count += (((IEC104_DATA_SD[i-1])>>16)+1);
		if((((IEC104_DATA_SD[i])<<16)>>16) != ((((IEC104_DATA_SD[i-1])<<16)>>16) + (((IEC104_DATA_SD[i-1])>>16)+1)))
		{
//			printf("addr = %d\r\n",(i - uValue));
			memset(uRecBuffer,0,256);
			if(ComMasterRead(&g_sMaster,g_DeviceSouth.device_inf[uRelDevAddr].addr,03,IEC104_DATA_SD[i - uValue],sRegist_Count,uRecBuffer,NULL) > 0)
			{
				for(j = (i - uValue);j < i;j++)
				{
					uint8_t k = 0;
					if(0 == ((IEC104_DATA_SD[j])>>16))
					{
	//					DEBUGOUT("\r\n�Ĵ�����ַ��%d  ",((IEC104_DATA_SD[j])<<16)>>16);
						for(k = 0;k < 2;k++)
						{
							ctemp.c[k] = uRecBuffer[uIntervalPoint + (2-k)];
							IEC104_DATA_SD[j] = ctemp.u16;
						}
						uIntervalPoint += 2;
	//					DEBUGOUT("���ݳ���1 - ����Ϊ��%X\r\n",IEC104_DATA_SD[j]);
					}
					else if(1 == ((IEC104_DATA_SD[j])>>16))
					{
	//					DEBUGOUT("\r\n�Ĵ�����ַ��%d  ",((IEC104_DATA_SD[j])<<16)>>16);
						for(k = 0;k < 4;k++)
						{
							ctemp.c[k] = uRecBuffer[uIntervalPoint + (4-k)];
							IEC104_DATA_SD[j] = ctemp.u;
						}
						uIntervalPoint += 4;
	//					DEBUGOUT("���ݳ���2 - ����Ϊ��%X\r\n",IEC104_DATA_SD[j]);
					}
				}
			}
			uValue = 0;
			sRegist_Count = 0;
			uIntervalPoint = 2;
		}
	}
    return 0;
}
/******************************************************************************
* ��    �ƣ�SouthSynctime()
* ��    �ܣ�����ͬ��ʱ��
* ��ڲ�����
*           ��      ��
* ���ڲ�����
* ��    ��:
******************************************************************************/
void SouthSynctime(void)
{
    time_t uTimeTick;
    //uint32_t uTimeTick;
    uint16_t uDataTemp[2];
    struct tm *p;
    uint8_t uAddr = 0;
    int8_t iResult;
	uint8_t uSynctimeCount;
    while(1)
    {
        if(uAddr >= MAX_device)
        {
            break;
        }
        if(0 == g_DeviceSouth.device_inf[uAddr].addr)
        {
            uAddr++;
            continue;
        }

		iResult = ComMasterRead(&g_sMaster,g_DeviceSouth.device_inf[uAddr].addr,DISCOVERY_REPORT,0,0,uRecBuffer,NULL);
					
		if(iResult > 0)  // ��ȡ3B�ɹ�
		{
			msleep(300);
			uTimeTick = RealTimeGetTick();
			uDataTemp[0] = uTimeTick>>16;
			uDataTemp[1] = uTimeTick & 0xFFFF;
			//DEBUGOUT("дʱ��Ĵ�����%d",uTimeTick);
			uSynctimeCount = 0;
            if(uSynctimeCount < 3)
            {
				iResult = ComMasterWrite(&g_sMaster,g_DeviceSouth.device_inf[uAddr].addr,0x10,40002,2,uDataTemp);  //д�����Ĵ�����ʱ��
								
				if(iResult < 0)
				{
					uSynctimeCount++;
					DEBUGOUT("���%dʱ��ͬ��ʧ��%d��",g_DeviceSouth.device_inf[uAddr].addr,uSynctimeCount);
					msleep(200);
				}
				else
				{
                   uSynctimeCount = 3;
				   p = localtime(&uTimeTick);
				   DEBUGOUT("���%dͬ��ʱ��Ϊ��%d-%d-%d %d:%d:%d",g_DeviceSouth.device_inf[uAddr].addr,1900+p->tm_year,1+p->tm_mon,p->tm_mday,p->tm_hour,p->tm_min,p->tm_sec);
				}
			}	
		}
        uAddr++;
        msleep(300);
    }
}
/****************************************************************************
* ��    �ƣ�TaskSouthWrite()
* ��    �ܣ������ѯ����
* ��ڲ�����

* ���ڲ�������
* ��    ��: ��
****************************************************************************/
OS_STK TaskSouthWriteStk[TaskSouthWriteStkSize];	      // ���������ջ��С
void TaskSouthWrite(void *p)
{
    uint8_t *pMes;
    uint8_t err;
    p = p;

    while(1)
    {
        pMes = OSQPend(MesQ,0,&err);    //������Ϣ����
//        DEBUGOUT("������:%d\n",*pMes);
        if(SOUTH_CMD_YK == *pMes)     // ң��
        {
            SouthWriteYk();
        }
        else if(SOUTH_CMD_SYNC == *pMes)  // ͬ��ʱ��
        {
            SouthSynctime();
        }
        else if(SOUTH_CMD_SD == *pMes)  // ���
        {
            SouthWriteSD(); 
        }
		else if(SOUTH_CMD_READSD == *pMes)  //�����
        {
			SouthReadSD();
		}
    }
}

/******************************************************************************
* ��    �ƣ�AlarmAddDev()
* ��    �ܣ���������豸�澯��
* ��ڲ�����
            uRelAddr:      �豸��Ե�ַ
            uSum:          ���澯������
            uModAddr��     �Ĵ�����ַ
            uBit��         �澯��λ��
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void AlarmAddDev(uint8_t uRelAddr,uint8_t uSum,uint16_t uModAddr,uint16_t uValue)
{
    uint8_t temp;
    if(NULL==g_psSouthAlarmCopy[uRelAddr])
    {
        return;
    }

    for(temp=0; temp<uSum; temp++)
    {
        if(g_psSouthAlarmCopy[uRelAddr][temp].mdbus_addr == uModAddr)
        {
            break;
        }
    }
    if (0xFFFF != uValue)//�ɼ���ȫFF�澯����
    {        
        g_psSouthAlarmCopy[uRelAddr][temp].alarm_value =uValue;
    }
}
/******************************************************************************
* ��    �ƣ�AlarmCheckout()
* ��    �ܣ��ϱ������豸�澯��
* ��ڲ�����

*
* ���ڲ�����
* ��    ��:
******************************************************************************/
#define ALARM_POWER_DOWN   0x04   // ����澯
#define ALARM_COM_LOST     0x01   // ͨѶ�����澯
#define ALARM_DEV_ALARM    0x02   // �豸�澯
#define ALARM_COM_DEV      0x03   // ͨѶ���� �� �豸�澯
uint8_t AlarmCheckout(void)
{
    static uint32_t s_uAlarmTimeOut=0;            // �澯�ϱ��󣬳�ʱû��ȷ���ٴ��ϱ���
    static uint32_t s_uPowerOffAlarmTimeOut = 0;  // ��Դ�ϵ�󣬳�ʱû��ȷ���ٴ��ϱ�
    uint8_t   i,j;
    uint8_t   uRes;
    uint8_t   uSum;       // �澯������
    uint16_t  uNewAlarm;  // �¸澯ֵ
    uint16_t  uOldAlarm;  // �ϸ澯ֵ
    uint16_t  uBit;       // �澯������λ
    uint8_t   uNeedToSave; // ��Ҫ����澯��

    if(NORTH_OK!=g_LoggerRun.north_status)// || RUNNING_WORK_READ!=Logger_run.run_status)  // ���ӷ�����δ���
    {
        s_uAlarmTimeOut = g_uTimeNow;
        s_uPowerOffAlarmTimeOut = g_uTimeNow;
        return 0;
    }
    //-----------------------------------------------------------------------------------
    // ���ɶϵ�澯
    if(g_uAlarmReport & ALARM_POWER_DOWN)
    {
        if(TIMEOUT == TimeOut(s_uPowerOffAlarmTimeOut,g_uTimeNow,5000))//if(s_uPowerOffAlarmTimeOut>500)  // 5s
        {
            g_uAlarmReport &= ~ALARM_POWER_DOWN;
            s_uPowerOffAlarmTimeOut = g_uTimeNow;
            if((g_LoggerRun.err&err_power_off) != (g_LoggerAlarm.log_alarm&err_power_off))
            {
                AlarmReport(0,     	// �豸ͨѶ��ַ
                            0x03,   // �쳣����-�����豸�쳣
                            0x0001, // MODBUS�Ĵ�����ַ
                            0,      // ƫ����
                            JUDGE((g_LoggerRun.err&err_power_off)),      // �澯������ָ�
                            1);     // �����ϱ�
                g_uAlarmReport |= ALARM_POWER_DOWN;
            }
        }
    }
    else
    {
        s_uPowerOffAlarmTimeOut = g_uTimeNow;
    }

    //-----------------------------------------------------------------------------------
    if(g_uAlarmReport & ALARM_COM_DEV)					// �����豸�澯 �� �����豸ͨѶ�澯
    {
        if(TIMEOUT == TimeOut(s_uAlarmTimeOut,g_uTimeNow,30000))//if(s_uAlarmTimeOut>3000)  // 30s
        {
            g_uAlarmReport &= ~ALARM_COM_DEV;
            s_uAlarmTimeOut = g_uTimeNow;
        }
        else
        {
            return 1;
        }
    }
    else
    {
        if(TIMEOUT == TimeOut(s_uAlarmTimeOut,g_uTimeNow,5000))//if(s_uAlarmTimeOut<500)  // 5s
        {
            s_uAlarmTimeOut = g_uTimeNow;
        }
        else
        {
            return 1;
        }
    }

    //-----------------------------------------------------------------------------------
    // �����豸ͨѶ�澯
    if(g_LoggerRun.err_lost != g_LoggerAlarm.dev_lost)
    {
        for(uBit=0;uBit<16;uBit++)
        {
            if((g_LoggerRun.err_lost&(1<<uBit)) == (g_LoggerAlarm.dev_lost&(1<<uBit)) )
            {
                continue;
            }
            if(0==g_DeviceSouth.device_inf[uBit].addr)
            {
                g_LoggerAlarm.dev_lost &= ~(1<<uBit);
                g_LoggerRun.err_lost &= ~(1<<uBit);
                continue;
            }
            uRes = AlarmReport(g_DeviceSouth.device_inf[uBit].addr,              // �豸ͨѶ��ַ
                                         0x01,                                  // �����豸ͨѶ�쳣
                                         0x0000,                                // MODBUS�Ĵ�����ַ
                                         0x00,                                  // ƫ����
                                         JUDGE(g_LoggerRun.err_lost&(1<<uBit)), // �澯������ָ�
                                         0);    // �������ϱ�
            if(g_LoggerRun.err_lost&(1<<uBit))//�澯����
            {
                g_LoggerAlarm.dev_lost |= (1<<uBit);
                uNeedToSave = 1;//SaveEepData(EEP_ALARM);  // ����澯ֵ
            }

            if(uRes) // һ֡��
            {
                g_uAlarmReport |= ALARM_COM_LOST;

                if(uNeedToSave)
                {
                    uNeedToSave = 0;
                    SaveEepData(EEP_ALARM);  // ����澯ֵ
                }
                return 2;
            }
        }
    }
    //-----------------------------------------------------------------------------------
    // �����豸�澯

    for(i=0; i<MAX_device&&i<g_DeviceSouth.device_sum; i++)
    {
		

		uSum = g_DeviceSouth.protocol[g_DeviceSouth.device_inf[i].rel_num].alarm_sum;
        for(j=0; j<uSum; j++)
        {
			uNewAlarm = g_psSouthAlarmCopy[i][j].alarm_value;
            uOldAlarm = g_psSouthAlarm[i][j].alarm_value;

            if(uNewAlarm == uOldAlarm)
            {
                continue;
            }
			for(uBit=0; uBit<16; uBit++)
			{
				if((uNewAlarm&(1<<uBit)) == (uOldAlarm&(1<<uBit)))
				{
					continue;
				}
				//DEBUGOUT("uNewAlarm:%duOldAlarm:%d\n",uNewAlarm,uOldAlarm);
				uRes = AlarmReport(g_DeviceSouth.device_inf[i].addr,		// �豸ͨѶ��ַ
								   0x02,								 // �����豸�쳣
								   g_psSouthAlarmCopy[i][j].mdbus_addr,  // MODBUS�Ĵ�����ַ
								   uBit,								 // ƫ����
								   JUDGE(uNewAlarm&(1<<uBit)),			 // �澯������ָ�
								   0);	  // �������ϱ�

				if(uNewAlarm&(1<<uBit))//�澯����
				{
					g_psSouthAlarm[i][j].alarm_value |= (1<<uBit);
					uNeedToSave = 1;//SaveEepData(EEP_ALARM);  // ����澯ֵ
				}
				if(uRes) // һ֡��
				{
					g_uAlarmReport |= ALARM_DEV_ALARM;
					if(uNeedToSave)
					{
						uNeedToSave = 0;
						SaveEepData(EEP_ALARM);  // ����澯ֵ
					}
					return 2;
				}
			}
        }
    }

    uRes = AlarmReport(0,   // �豸ͨѶ��ַ
                    0,   // �����豸�쳣
                    0,   // MODBUS�Ĵ�����ַ
                    0,   // ƫ����
                    0,   // �澯������ָ�
                    1);  // �����ϱ�
    if(uRes) // һ֡��
    {
        g_uAlarmReport |= ALARM_COM_DEV;
        if(uNeedToSave)
        {
            uNeedToSave = 0;
            SaveEepData(EEP_ALARM);  // ����澯ֵ
        }
        return 2;
    }
    if(uNeedToSave)
    {
        uNeedToSave = 0;
        SaveEepData(EEP_ALARM);  // ����澯ֵ
    }
    return 0;
}
/******************************************************************************
* ��    �ƣ�AlarmAck()
* ��    �ܣ�ȷ�ϸ澯��
* ��ڲ�����
            uLen         ��Ч���ݳ���
            puData       �յ�������
*
* ���ڲ�������
* ��    ��:
* ��    ע�� �˺�����IEC104.c���յ��澯ȷ��֡����
******************************************************************************/
void AlarmAck(uint8_t uLen,const uint8_t *puData)
{
    uint8_t i;
    uint8_t temp;
    uint8_t uSum;
    uint8_t  uRelAddr;  // �豸��Ե�ַ
    uint16_t uModAddr;  // MODBUS�澯���ַ���Ĵ�����ַ
    uint8_t  uNeedToSave=0; // ��Ҫ����澯��

    for(i=0;i<uLen;i+=6)
    {
    	FEED_DOG();
        uRelAddr = SlaveAddrConvert(puData[i]);
        uModAddr = puData[i+2] | (puData[i+3]<<8);

        if(uRelAddr >= MAX_device)    //���ɱ����ַ��0��������Ե�ַ����0xFF
        {
            return;
        }
        FEED_DOG();
        switch(puData[i+1])
        {
        case 0x02:// �����豸�澯

            if(NULL==g_psSouthAlarm[uRelAddr])
            {
                break;
            }
            uSum = g_DeviceSouth.protocol[g_DeviceSouth.device_inf[uRelAddr].rel_num].alarm_sum;

            for(temp=0; temp<uSum; temp++)
            {
                if(g_psSouthAlarm[uRelAddr][temp].mdbus_addr == uModAddr)
                {
                    break;
                }
            }

            if(temp>=uSum)
            {
                break;
            }

            if(puData[i+5])  // 1�쳣������0�쳣�ָ�
            {
                //g_psSouthAlarm[uRelAddr][temp].alarm_value |= (1<<puData[i+4]);
				//uNeedToSave = 1;
            }
            else
            {
                g_psSouthAlarm[uRelAddr][temp].alarm_value &= ~(1<<puData[i+4]);
                uNeedToSave = 1;//SaveEepData(EEP_ALARM);  // ����澯ֵ
            }
            //DEBUGOUT("g_psSouthAlarm[uRelAddr][temp].alarm_value:%d i:%d",g_psSouthAlarm[uRelAddr][temp].alarm_value,i);
            g_uAlarmReport &= ~ALARM_DEV_ALARM;
            break;

        case 0x01:// ����ͨѶ�澯
            if(puData[i+5])
            {
                //g_LoggerAlarm.dev_lost |= (1<<uRelAddr);
            }
            else
            {
                g_LoggerAlarm.dev_lost &= ~(1<<uRelAddr);
            }
            uNeedToSave = 1;//SaveEepData(EEP_ALARM);  // ����澯ֵ

            g_uAlarmReport &= ~ALARM_COM_LOST;
            break;

        case 0x03:// ���ɸ澯
            if(0x0001==uModAddr && 0x00==puData[i+4])  // �ϵ�澯
            {
                g_LoggerAlarm.log_alarm = g_LoggerRun.err;
                g_uAlarmReport &= ~ALARM_POWER_DOWN;
            }
            if(0==g_LoggerRun.err&err_power_off)     //�澯�ָ����ڴ˴�����
            {
                uNeedToSave = 1;//SaveEepData(EEP_ALARM);  // ����澯ֵ
            }
            break;
		default:
            break;
        }

        if(uNeedToSave)
        {
            SaveEepData(EEP_ALARM);  // ����澯ֵ
        }
        FEED_DOG();
    }
	 return;
}
/******************************************************************************
* ��    �ƣ�AlarmReport()
* ��    �ܣ��澯�ϱ��㴴����
* ��ڲ�����
            addr:      �쳣�豸��ַ��0Ϊ���ɱ���
            err_code:  �쳣���룬Ϊ0Ϊ�ϱ�δ�ϱ��ĸ澯
            reg_addr:  MODBUS�Ĵ�����ַ
            offset:    ƫ����
            event:     1:�쳣������0:�쳣�ָ�
            imd:       1:�����ϱ���0����֡�ϱ�
*
* ���ڲ�����û�����ӵ�����������0
* ��    ��:
******************************************************************************/
uint8_t AlarmReport(uint8_t addr,uint8_t err_code,uint16_t reg_addr,uint8_t offset,uint8_t event,uint8_t imd)
{
    if(NORTH_OK!=g_LoggerRun.north_status)  // ���ӷ�����δ���
    {
        return 0;
    }
    //-----------------------------------------------------------------
    do
    {
        if(g_sBurst.alarm_count<30 && err_code)   // һ���澯��ռ��6���ֽ�
        {
            g_sBurst.alarm_report[g_sBurst.alarm_count*6]   = addr;            // �豸ͨѶ��ַ
            g_sBurst.alarm_report[g_sBurst.alarm_count*6+1] = err_code;        // �����豸�쳣
            g_sBurst.alarm_report[g_sBurst.alarm_count*6+2] = reg_addr&0xff;   // MODBUS�Ĵ�����ַ��λ
            g_sBurst.alarm_report[g_sBurst.alarm_count*6+3] = reg_addr>>8;     // MODBUS�Ĵ�����ַ��λ
            g_sBurst.alarm_report[g_sBurst.alarm_count*6+4] = offset;          // ƫ����
            g_sBurst.alarm_report[g_sBurst.alarm_count*6+5] = event;           // 1:�쳣������0���쳣�ָ�

            g_sBurst.alarm_count++;

            err_code = 0;
        }

        if((imd && 0!=g_sBurst.alarm_count) || g_sBurst.alarm_count>=30)
        {
            // һ֡�����ϴ���
            g_sIecSend.format.maddrL = 0x00;
            g_sIecSend.format.maddrM = 0x00;
            g_sIecSend.format.maddrH = 0x00;

            memcpy(g_sIecSend.format.data,&g_sBurst.alarm_report[0],g_sBurst.alarm_count*6);  // I֡����
            IecCreateFrameI(P_ERR_PROCESS,0x01,R_ACTIVE,g_sBurst.alarm_count*6,&g_sIecSend); // �ֽ���Ϊ��Ϣ����*4 - �Ѿ������ĵ�һ����Ϣ���ַ

            // ���¿�ʼ��¼
            g_sBurst.alarm_count = 0x00;

            return 1;
        }
    }while(err_code);
    //-----------------------------------------------------------------
    return 0;
}
/******************************************************************************
* ��    �ƣ�SetBaudRate()
* ��    �ܣ�
* ��ڲ�����
            addr:      �豸��ַ
            data       �յ�������
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
uint8_t SetBaudRate(uint8_t now,uint8_t target)
{
    uint32_t  uBaudRate;
//1:2400��2:4800��3:9600��4:19200��5:38400��6:115200
    if(now==target)
    {
        return now;
    }

    switch(target)
    {
    case BAUDRATE_2400:
        uBaudRate = 2400;
        break;

    case BAUDRATE_4800:
        uBaudRate = 4800;
        break;

    case BAUDRATE_9600:
        uBaudRate = 9600;
        break;

    case BAUDRATE_19200:
        uBaudRate = 19200;
        break;

    case BAUDRATE_38400:
        uBaudRate = 38400;
        break;

    case BAUDRATE_115200:
        uBaudRate = 115200;
        break;

    default:
        target = BAUDRATE_9600;
        uBaudRate = 9600;
        break;
    }

    UartInit(SLAVE_UART_USE,uBaudRate,UART_PARITY_NONE);      // ���ڳ�ʼ��

    return target;
}
/******************************************************************************
* ��    �ƣ�ResetIecData()
* ��    �ܣ����������ѯ���ݡ�
* ��ڲ�����
            ��
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void ResetIecData(uint8_t uRelAddr)
{
    uint16_t iec_addr; //
    uint16_t count=0;
    uint16_t yc_sum=0;   // ң�����ͳ��
    uint16_t yx_sum=0;   // ң�ŵ���ͳ��
    uint8_t  rel_num;


    rel_num = g_DeviceSouth.device_inf[uRelAddr].rel_num;  // ��Ե���
    /*for(count=0; count<g_DeviceSouth.protocol[rel_num].mess_point_sum; count++)
    {
        if(TYPE_YC==Reg_point[rel_num][count].reg_type.type.mess)
        {
            yc_sum++;
        }
        else if(TYPE_YX==Reg_point[rel_num][count].reg_type.type.mess)
        {
            yx_sum++;
        }
    }*/
    yc_sum = g_sIecPointCount[rel_num].uYcSum;
    yx_sum = g_sIecPointCount[rel_num].uYxSum;


    if(g_DeviceSouth.device_inf[uRelAddr].yx_start_addr >= 1)
    {
        iec_addr = g_DeviceSouth.device_inf[uRelAddr].yx_start_addr - 1; // �ҳ�ң����ʼ��ַ
        count  = iec_addr;           // ��ַƫ�ƣ��ó���һ�����Ӧ��iec104������±�
        yx_sum = yx_sum + iec_addr;  // ���ϵ�ַƫ�ƣ��ó����һ�����Ӧ��iec104������±�
        for(; count<yx_sum && NULL!=IEC104_DATA_YX; count++)
        {
            IEC104_DATA_YX[count] = 0xff;
        }
    }

    if(g_DeviceSouth.device_inf[uRelAddr].yc_start_addr>=0x4001)
    {
        iec_addr = g_DeviceSouth.device_inf[uRelAddr].yc_start_addr - 0x4001; // �ҳ�ң����ʼ��ַ
        count  = iec_addr;           // ��ַƫ�ƣ��ó���һ�����Ӧ��iec104������±�
        yc_sum = yc_sum + iec_addr;  // ���ϵ�ַƫ�ƣ��ó����һ�����Ӧ��iec104������±�
        for(; count<yc_sum && NULL!=IEC104_DATA_YC; count++)
        {
            IEC104_DATA_YC[count] = 0xffffffff;
        }
    }
}
/******************************************************************************
* ��    �ƣ�CheckDevState()
* ��    �ܣ���������豸״̬��
* ��ڲ�����
            ��
*
* ���ڲ�����-1��û���豸  0������û��ȫ����֡  ������ȫ����֡��λ���
* ��    ��:
******************************************************************************/
int8_t CheckDevState(void)
{
    uint8_t i;
    uint8_t uDevExist=0;  // ���豸
    uint8_t uAllLost=0;   // �豸��֡

    for(i=0;i<MAX_device;i++)
    {
        if(g_DeviceSouth.device_inf[i].addr)
        {
            uDevExist |= (1<<i);

            if(g_LoggerRun.err_lost&(1<<i))
            {
                uAllLost |= (1<<i);
            }
        }
    }

    if(0==uDevExist)        //���豸
    {
        return -1;
    }
    else if(0==(uDevExist&uAllLost))    //�����豸������
    {
        return 0;
    }
    else                                //���豸�����澯
    {
        return (uDevExist&uAllLost);        //�����豸���ϻ�ػ�������Ȼ�ü�¼��ʷ���ݣ����ڲ�������
    }

}
/**************************************************************************************
* ��    �ƣ� TimedRestart()
* ��    �ܣ� ��ʱ������ÿ��1��26��1�붨ʱ���0~120���Ӻ��������ɣ�ǰ���������Ѹ�ƽ̨��ʱ��
* ��ڲ�����uRandom��������ֵ
* ���ڲ������������������ʵ�ʷ�����(0~uRandom)
* ����:		
* ���ڣ�		
**************************************************************************************/
uint8_t TimedReboot(uint8_t uRandom)
{
	static uint8_t uRandTime = 121;	// �����������0-120
	static uint8_t uRebotMark = 0;    
    uint16_t uRandSeeds = 0;			// ������ӣ����ڲ��������
    uint32_t uRandNum = 0;				// ����������ڲ������������
    static uint32_t uStartTime = 0;
	uint32_t uNowTime;
    SYSTEMTIME *pGetTime;
    pGetTime = RealTimeGet();
	g_uTimeNow = OSTimeGet();   // ��ȡ��ǰ��ʱ���
    if(1 == pGetTime->Hour)
    {
        if((1 == pGetTime->Minute) && (uRebotMark == 0))
        {
           // if(1==pGetTime->Second)
            {
			     uRebotMark = 1;
				 uRandSeeds = CalculateCRC((uint8_t*)g_LoggerInfo.esn,20);	//��ESN��CRC��Ϊ�������
                 srand(uRandSeeds);                                        //�������������
                 uRandTime = 0;
				
                 uRandNum = rand();                                        //���������
                 uRandTime = uRandNum%uRandom;
				 uStartTime = g_uTimeNow;
                 DEBUGOUT("######%d���Ӻ�ʱ����######\n",uRandTime);
            }
        }
    }

    if(uRandTime < uRandom)
    {
        uNowTime = g_uTimeNow;
	    if(TimeGapJudge(uStartTime,uNowTime,uRandTime*60000))
        {
            msleep(100);
        }
		else
		{
			OSTimeDlyHMSM(0,1,5,0);
			DEBUGOUT("######��ʱʱ�䵽��ʼ����!!!######\n");
			uRebotMark = 0;
            Reboot();
		}
    } 
    return uRandTime;
}
