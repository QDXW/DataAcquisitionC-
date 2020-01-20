#include "IEC104.h"
#include "GlobalVar.h"
#include "DataTransferArea.h"
#include "Memory.h"
#include "RealTime.h"
#include "DataFlash.h"
#include "InEEPROM.h"
#include "Record.h"
#include "CRC16.h"
#include "SlaveUnit.h"
#include "tool.h"
#include "log.h"

#include "SubCollection.h"
//========================================================================
#define JUDGE(x)  ((x)==0?0:1)  // �ж���ֵ�Ƿ�Ϊ0��Ϊ0����0����0����1
//========================================================================
#define msleep(x)  OSTimeDly((x)/10+1)                // usleep((x)*1000)
#define sleep(x)   OSTimeDly(OS_TICKS_PER_SEC*(x)+1)  // ��ʱx��
//========================================================================
#define MAX_POINT_ONE_TABLE    315                  // һ�ŵ����������������ʱ���ռ���
//========================================================================
#define LOGFRAMELENTH          200                   // ��־�ϴ�һ֡���͵��ֽ���
//========================================================================
typedef struct
{
    uint16_t uLastTableNum;       // �ϴ�ϵͳ���͹����ĵ���
    uint16_t uLastPointCount;     // �ϴ��Ѿ����͵���Ϣ�㣬����ͳ��
    uint8_t  uRelPoint;           // ��Ե���
    uint8_t  uRelDev;             // �豸��Ϣ�ṹ�������±꣬����豸ͨѶ��ַ
} POINT_RECORD_TEMP_T;

typedef union
{
    uint16_t uInfAddr;
    uint8_t uInfTemp[2];
}LOG_INF_ADDE;

//========================================================================
//========================================================================
uint8_t  *IEC104_DATA_YX;   // IEC104����-ң��ָ�룬��̬����ռ�
uint32_t *IEC104_DATA_YC;   // IEC104����-ң��ָ�룬��̬����ռ�
uint8_t  *IEC104_DATA_YK;   // IEC104����-ң��ָ�룬��̬����ռ�
uint32_t *IEC104_DATA_SD;   // IEC104����-���ָ�룬��̬����ռ�
uint32_t *IEC104_DATA_DD;   // IEC104����-���ָ�룬��̬����ռ�
uint16_t preFrameDataCrc = 0;       // ��һ�ε�I֡����CRCֵ
//========================================================================
IEC104_COUNT_T g_sIecPointCount[MAX_device];  // �����Ϣ��ͳ��

IEC104_MAIN_T g_sIEC;
IEC_RUNNING_T g_sIecRun;

uint16_t      g_IecRecSeq=0;          // ��������
uint16_t      g_IecSendSeq=0;         // ��������
uint8_t 	  Data_resend_count = 0;
//========================================================================
static uint8_t g_uReportCtrl=0;                    // ���Ϳ��Ʊ��
uint8_t g_TimeMarker[7] = {0};              // ����������Ҫ�ϱ���ʱ��
static uint8_t g_uSubData[YCBUFFSIZE];             // ���ڲ���

static LOGGER_MODBUS_REG_T *pRegPointTemp=NULL;    // ��ʱ������õĵ��ָ��
static POINT_RECORD_TEMP_T s_sPointRecord;         // ��¼�������Ϣ

static uint32_t g_uNowTime;     // ��ǰʱ�ꡣ
static uint32_t g_uUpdataTime;  // ����ʱ�꣬����Զ��������������
//static uint32_t g_uDT1000UpdataTime;  //�������ʱ�꣬����Զ��������Ƴ�ʱ����

static uint8_t  s_uSouthYk=SOUTH_CMD_YK;       // ����ң��
static uint8_t  s_uSouthSd=SOUTH_CMD_SD;       // �������
static uint8_t  s_uSouthSync=SOUTH_CMD_SYNC;   // ����ͬ��ʱ��
static uint8_t  s_uSouthReadSd=SOUTH_CMD_READSD; //��������

//========================================================================
void IecCollectProcess(IEC104_MAIN_T *pA,IEC_RUNNING_T *call);  // ����
void IecSubSCollect(IEC104_MAIN_T *pA,IEC_RUNNING_T *call);     // ����
void Iec104HandleHwSoftRenovate(IEC104_MAIN_T *pA);
//========================================================================
// ����ϱ�����
void ReportCtrlClear(uint8_t uCtrl)
{
    g_uReportCtrl &= ~uCtrl;
}
// �����ϱ�����
void ReportCtrlSet(uint8_t uCtrl)
{
    g_uReportCtrl |= uCtrl;
}
// ���ϱ�����
uint8_t ReportCtrlRead(uint8_t uCtrl)
{
    return (g_uReportCtrl&uCtrl)?1:0;
}
/******************************************************************************
* ��    �ƣ�IecInit()
* ��    �ܣ�IEC104  ��ʼ����
* ��ڲ�����
*           ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
void IecInit(void)
{
    uint16_t i,uCcount;
    uint16_t uYxCount;
    uint16_t uYcCount;
    uint16_t uYkCount;
    uint16_t uSdCount;

    for(i=0;i<MAX_device;i++)
    {
        if(NULL!=g_pRegPoint[i])
        {
            uYxCount = 0;
            uYcCount = 0;
            uYkCount = 0;
            uSdCount = 0;

//            DEBUGOUT("g_sIecPointCount[%d].Sum:%d\r\n",i,g_DeviceSouth.protocol[i].mess_point_sum);
//            DEBUGOUT("g_sIecPointCount[%d].YXSum:%d\r\n",i,g_DeviceSouth.yx_sum);
//            DEBUGOUT("g_sIecPointCount[%d].YCSum:%d\r\n",i,g_DeviceSouth.yc_sum);
//            DEBUGOUT("g_sIecPointCount[%d].YTSum:%d\r\n",i,g_DeviceSouth.yk_sum);
//            DEBUGOUT("g_sIecPointCount[%d].SDSum:%d\r\n",i,g_DeviceSouth.sd_sum);
//            for(uCcount=0,uYcCount=0,uYxCount=0; uCcount<g_DeviceSouth.protocol[i].mess_point_sum; uCcount++)
            for(uCcount=0; uCcount<g_DeviceSouth.protocol[i].mess_point_sum; uCcount++)
            {
//            	DEBUGOUT("g_pRegPoint[%d][%d].reg_type.type.mess:%d\r\n",i,uCcount,g_pRegPoint[i][uCcount].reg_type.type.mess);
                if(TYPE_YC==g_pRegPoint[i][uCcount].reg_type.type.mess)
                {
                    uYcCount++;
                }
                else if(TYPE_YX==g_pRegPoint[i][uCcount].reg_type.type.mess)
                {
                    uYxCount++;
                }
                else if(TYPE_YK==g_pRegPoint[i][uCcount].reg_type.type.mess)
                {
                    uYkCount++;
                }
                else if(TYPE_SD==g_pRegPoint[i][uCcount].reg_type.type.mess)
                {
                    uSdCount++;
                }
            }
            g_sIecPointCount[i].uYxSum = uYxCount;
            g_sIecPointCount[i].uYcSum = uYcCount;
            g_sIecPointCount[i].uYkSum = uYkCount;
            g_sIecPointCount[i].uSdSum = uSdCount;
//			DEBUGOUT("g_sIecPointCount[%d].uYxSum:%d\r\n",i,g_sIecPointCount[i].uYxSum);
//			DEBUGOUT("g_sIecPointCount[%d].uYcSum:%d\r\n",i,g_sIecPointCount[i].uYcSum);
//			DEBUGOUT("g_sIecPointCount[%d].uYkSum:%d\r\n",i,g_sIecPointCount[i].uYkSum);
//			DEBUGOUT("g_sIecPointCount[%d].uSdSum:%d\r\n",i,g_sIecPointCount[i].uSdSum);
//			DEBUGOUT("g_sIecPointCount[%d].Sum:%d\r\n",i,uYxCount+uYcCount+uYkCount+uSdCount);
        }
    }
    //--------------------------------------
    // IEC104���ڴ�ռ��ʼ��
    if(g_DeviceSouth.yx_sum>0)       // ����ң�������ڴ�ռ�
    {
        IEC104_DATA_YX = (uint8_t*)WMemMalloc(IEC104_DATA_YX,g_DeviceSouth.yx_sum*sizeof(uint8_t));
        if(NULL==IEC104_DATA_YX)
        {
            DEBUGOUT("Ҫң�ſռ�ʧ��\n");
            return;
        }
        for(i=0;i<g_DeviceSouth.yx_sum;i++)
        {
            IEC104_DATA_YX[i] = 0xFF;
        }
    }
    else
    {
        IEC104_DATA_YX = NULL;
    }

    if(g_DeviceSouth.yc_sum>0)       // ����ң�������ڴ�ռ�
    {
        IEC104_DATA_YC = (uint32_t*)WMemMalloc(IEC104_DATA_YC,g_DeviceSouth.yc_sum*sizeof(uint32_t));
        if(NULL==IEC104_DATA_YC)
        {
            DEBUGOUT("Ҫң��ռ�ʧ��\n");
            return;
        }
        for(i=0;i<g_DeviceSouth.yc_sum;i++)
        {
            IEC104_DATA_YC[i] = 0xFFFFFFFF;
        }
        DEBUGOUT("Application YC Space Success!!!\r\n");
    }
    else
    {
    	DEBUGOUT("Not YC Point!!!\r\n");
        IEC104_DATA_YC = NULL;
    }

    if(g_DeviceSouth.yk_sum>0)       // ����ң�������ڴ�ռ�
    {
        IEC104_DATA_YK = (uint8_t*)WMemMalloc(IEC104_DATA_YK,g_DeviceSouth.yk_sum*sizeof(uint8_t));
        if(NULL==IEC104_DATA_YK)
        {
            DEBUGOUT("Ҫң�ؿռ�ʧ��\n");
            return;
        }
        for(i=0;i<g_DeviceSouth.yk_sum;i++)
        {
            IEC104_DATA_YK[i] = 0x00;
        }
    }
    else
    {
        IEC104_DATA_YK = NULL;
    }

    if(g_DeviceSouth.sd_sum>0)       // ������������ڴ�ռ�
    {
        IEC104_DATA_SD = (uint32_t*)WMemMalloc(IEC104_DATA_SD,g_DeviceSouth.sd_sum*sizeof(uint32_t));
        if(NULL==IEC104_DATA_SD)
        {
            DEBUGOUT("Ҫ���ռ�ʧ��\n");
            return;
        }
    }
    else
    {
        IEC104_DATA_SD = NULL;
    }

    if(g_DeviceSouth.dd_sum>0)       // �����������ڴ�ռ�
    {
        IEC104_DATA_DD = (uint32_t*)WMemMalloc(IEC104_DATA_DD,g_DeviceSouth.dd_sum*sizeof(float));
        if(NULL==IEC104_DATA_DD)
        {
            DEBUGOUT("Ҫ��ȿռ�ʧ��\n");
            return;
        }
    }
    else
    {
        IEC104_DATA_DD = NULL;
    }

    g_LoggerRun.uDevExist = 0;
    for(i=0;i<MAX_device;i++)
    {
        if(g_DeviceSouth.device_inf[i].addr)
        {
            g_LoggerRun.uDevExist |= 1<<i;
        }
    }
}
/******************************************************************************
* ��    �ƣ�Iec104FrameSend()
* ��    �ܣ��������ݵ���������
* ��ڲ�����
            bytes     ���ݳ���
            *pA         IEC��Ϣָ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
/*tatic void Iec104FrameSend(IEC104_MAIN_T *pA,uint8_t bytes)
{
    CmStoreSendLink((uint8_t *)&pA->send.buff,bytes);  // �洢������
}*/
/******************************************************************************
* ��    �ƣ�Iec104CreateFrameI()
* ��    �ܣ�IEC104 ���I֡��
* ��ڲ�����
*           type      ���ͱ�ʶ
            limit     �ɱ�ṹ�޶���
            reason    ����ԭ��
            *data     ����ָ��
            bytes     ���ݳ���  data�����ݳ���
            *pA       IEC��Ϣָ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104CreateFrameI2(uint8_t type,uint8_t limit,uint16_t reason,uint8_t bytes,IEC104_MAIN_T *pA)
{
    Uint_Char_Convert temp;

    pA->send.format.start   = IEC104_HEAD;  // ������
    pA->send.format.len     = 13 + bytes;   // ����
    temp.u = g_IecSendSeq++;
    pA->send.format.SseqL   = temp.c[0];    // ��������ֽ�1
    pA->send.format.SseqH   = temp.c[1];    // ��������ֽ�1
    temp.u = g_IecRecSeq;
    pA->send.format.RseqL   = temp.c[0];    // ��������ֽ�1
    pA->send.format.RseqH   = temp.c[1];    // ��������ֽ�2
    pA->send.format.type    = type;         // ���ͱ�ʶ
    pA->send.format.limit   = limit;        // �ɱ�ṹ�޶���
    pA->send.format.reasonL = reason&0xff;  // ����ԭ���ֽ�1
    pA->send.format.reasonH = reason>>8;    // ����ԭ���ֽ�2
    pA->send.format.addrL   = g_LoggerInfo.ADDR;         // ������ַ���ֽ�
    pA->send.format.addrH   = 0x00;         // ������ַ���ֽ�

    //Iec104FrameSend(pA,pA->send.format.len+2);
    //CmStoreSendLink((uint8_t *)&pA-> buff,pA->format.len+2);  // �洢������
}
/******************************************************************************
* ��    �ƣ�IecCreateFrameI()
* ��    �ܣ�IEC104 ���I֡��
* ��ڲ�����
*           type      ���ͱ�ʶ
            limit     �ɱ�ṹ�޶���
            reason    ����ԭ��
            *data     ����ָ��
            bytes     ���ݳ���  data�����ݳ���
            *pA       IEC��Ϣָ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
void IecCreateFrameI(uint8_t type,uint8_t limit,uint16_t reason,uint8_t bytes,IEC_FORMAT_T *pA)
{
    Uint_Char_Convert temp;

    pA->format.start   = IEC104_HEAD;  // ������
    pA->format.len     = 13 + bytes;   // ����
    temp.u = g_IecSendSeq<<1;
    pA->format.SseqL   = temp.c[0];    // ��������ֽ�1
    pA->format.SseqH   = temp.c[1];    // ��������ֽ�1
    temp.u = g_IecRecSeq<<1;
    pA->format.RseqL   = temp.c[0];    // ��������ֽ�1
    pA->format.RseqH   = temp.c[1];    // ��������ֽ�2
    pA->format.type    = type;         // ���ͱ�ʶ
    pA->format.limit   = limit;        // �ɱ�ṹ�޶���
    pA->format.reasonL = reason&0xff;  // ����ԭ���ֽ�1
    pA->format.reasonH = reason>>8;    // ����ԭ���ֽ�2
    pA->format.addrL   = g_LoggerInfo.ADDR;         // ������ַ���ֽ�
    pA->format.addrH   = 0x00;         // ������ַ���ֽ�

    //Iec104FrameSend(pA,pA->format.len+2);
    CmStoreSendLink((uint8_t *)&pA->buff,pA->format.len+2);  // �洢������

    g_IecSendSeq++;
}
/******************************************************************************
* ��    �ƣ�IecReportLogInfo()
* ��    �ܣ����ӵ�����������������Ϣ��
* ��ڲ�����
*           uint8_t uReason         ����ԭ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
void IecReportLogInfo(uint8_t uReason)//(IEC104_MAIN_T *pA)
{
    g_sIEC.send.format.maddrL = 0x00;// ��Ϣ���ַ
    g_sIEC.send.format.maddrM = 0x00;
    g_sIEC.send.format.maddrH = 0x00;

    memset(g_sIEC.send.format.data, 0, 80);//memset(g_sIEC.ibuff, 0, 80);

    memcpy(&g_sIEC.send.format.data[0],g_LoggerInfo.name,20);// �豸����0~19

    memcpy(&g_sIEC.send.format.data[20],g_LoggerInfo.esn,20); // ESN��20~39//strncpy(info,g_LoggerInfo.esn,20);  // ESN��

    memcpy(&g_sIEC.send.format.data[40],g_LoggerInfo.model,20);// �豸�ͺ�40~59

    memcpy(&g_sIEC.send.format.data[60],g_LoggerInfo.type,20);// �豸����60~79

    g_sIEC.send.format.data[80] = 0;//g_LoggerRun.IP[0];// IP��ַ80~83
    g_sIEC.send.format.data[81] = 0;//g_LoggerRun.IP[1];
    g_sIEC.send.format.data[82] = 0;//g_LoggerRun.IP[2];
    g_sIEC.send.format.data[83] = 0;//g_LoggerRun.IP[3];

    if(RUNNING_WORK_READ==g_LoggerRun.run_status)
    {
        g_sIEC.send.format.data[84] = 0x01;    // ����״̬84
    }
    else
    {
        g_sIEC.send.format.data[84] = 0x00;    //0x00
    }


    g_sIEC.send.format.data[85] = GetVerS2(); // ����汾85~87
    g_sIEC.send.format.data[86] = GetVerS1();
    g_sIEC.send.format.data[87] = GetVerType();

    IecCreateFrameI(P_DEV_INFO,1,uReason,88,&g_sIEC.send);
}


/******************************************************************************
* ��    �ƣ�IecProcessFrameS()
* ��    �ܣ�IEC104 S֡����
* ��ڲ�����
*           *pA         IEC��Ϣָ��
            dir         S֡�շ���1������S֡��0���յ�S֡
* ���ڲ�������
* ��    ��:
******************************************************************************/
static void IecProcessFrameS(IEC104_MAIN_T *pA,uint8_t dir)
{
    Uint_Char_Convert temp;
    if(dir)
    {
        pA->send.format.start = IEC104_HEAD;
        pA->send.format.len   = 0x04;
        pA->send.format.SseqL = IEC104_S_MAK;
        pA->send.format.SseqH = 0x00;
        temp.u = g_IecRecSeq;
        pA->send.format.RseqL = temp.c[0];
        pA->send.format.RseqH = temp.c[1];

        //Iec104FrameSend(pA,6);
        CmStoreSendLink((uint8_t *)&pA->send.buff,6);  // �洢������
    }
    else
    {
        if(g_LoggerRun.update)
        {
            pA->send.format.start = IEC104_HEAD;
            pA->send.format.len   = 0x04;
            pA->send.format.SseqL = IEC104_S_MAK;
            pA->send.format.SseqH = 0x00;
            temp.u = g_IecRecSeq;//pA->RecSeq << 1;
            pA->send.format.RseqL = temp.c[0];
            pA->send.format.RseqH = temp.c[1];

            //Iec104FrameSend(pA,6);
            CmStoreSendLink((uint8_t *)&pA->send.buff,6);  // �洢������
        }

    }
}
/******************************************************************************
* ��    �ƣ�IecProcessFrameU()
* ��    �ܣ�IEC104 U֡����
* ��ڲ�����
*           *pA         IEC��Ϣָ��
            dir         U֡�շ���1������U֡��0���յ�U֡
* ���ڲ�������
* ��    ��:
******************************************************************************/
static void IecProcessFrameU(IEC104_MAIN_T *pA,uint8_t cmd,uint8_t dir)
{
    if(0==dir)
    {
        switch(cmd)
        {
        case IEC104_U_STARTDT:
            pA->send.format.start = IEC104_HEAD;
            pA->send.format.len   = 0x04;
            pA->send.format.SseqL = IEC104_U_STARTDT_ACK | IEC104_U_MAK;
            pA->send.format.SseqH = 0x00;
            pA->send.format.RseqL = 0x00;
            pA->send.format.RseqH = 0x00;

            break;

        case IEC104_U_STARTDT_ACK:
            break;

        case IEC104_U_STOPDT:
            pA->send.format.start = IEC104_HEAD;
            pA->send.format.len   = 0x04;
            pA->send.format.SseqL = IEC104_U_STOPDT_ACK | IEC104_U_MAK;
            pA->send.format.SseqH = 0x00;
            pA->send.format.RseqL = 0x00;
            pA->send.format.RseqH = 0x00;
            break;

        case IEC104_U_STOPDT_ACK:
            break;

        case IEC104_U_TESTFR:
            pA->send.format.start = IEC104_HEAD;
            pA->send.format.len   = 0x04;
            pA->send.format.SseqL = IEC104_U_TESTFR_ACK | IEC104_U_MAK;
            pA->send.format.SseqH = 0x00;
            pA->send.format.RseqL = 0x00;
            pA->send.format.RseqH = 0x00;
            break;

        case IEC104_U_TESTFR_ACK:
            break;

        default:
            break;
        }
    }
    else
    {
        pA->send.format.start = IEC104_HEAD;
        pA->send.format.len   = 0x04;
        pA->send.format.SseqL = cmd | IEC104_U_MAK;
        pA->send.format.SseqH = 0x00;
        pA->send.format.RseqL = 0x00;
        pA->send.format.RseqH = 0x00;

        //Iec104FrameSend(pA,6);
    }

    //Iec104FrameSend(pA,6);
    CmStoreSendLink((uint8_t *)&pA->send.buff,6);  // �洢������
}
/******************************************************************************
* ��    �ƣ�Iec104ReportMaxDevice()
* ��    �ܣ��ϱ������豸���ֵ��
* ��ڲ�����
*           uint8_t uReason     ����ԭ��

* ���ڲ�����
* ��    ��:
******************************************************************************/
void Iec104ReportMaxDevice(uint8_t uReason,IEC104_MAIN_T *pA)
{
    pA->send.format.maddrL = 0x00;
    pA->send.format.maddrM = 0x00;
    pA->send.format.maddrH = 0x00;
    pA->send.format.data[0] = MAX_device;

    IecCreateFrameI(P_MAX_DEVICE,1,uReason,1,&pA->send);
}
/******************************************************************************
* ��    �ƣ�IecCpoyMesAddr()
* ��    �ܣ����յ���IEC���ݵ���Ϣ���ַ���Ƶ�IEC�������ݵ���Ϣ���ַ��
* ��ڲ�����
            *pA         IEC��Ϣָ��
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void IecCpoyMesAddr(IEC104_MAIN_T *pA)
{
    pA->send.format.maddrL = pA->recv.format.maddrL;
    pA->send.format.maddrM = pA->recv.format.maddrM;
    pA->send.format.maddrH = pA->recv.format.maddrH;
}
//����������豸��ŵ�ַ�����豸��Ӧ�����±�
uint8_t IecAddrConvert(uint8_t MbVirAddr)
{
	//to do
	uint8_t uMbRelAddr=0xff; // �������������Ӧ�ĵ�ַ������0xFF������ʾû����Ӧ���豸
    uint8_t i=0;
    if(!MbVirAddr)      //ͨѶ��ַ������
    {
        return uMbRelAddr;
    }
    for(i=0; i<MAX_device; i++)
    {
        if(g_DeviceSouth.device_inf[i].addr==MbVirAddr)
        {
            uMbRelAddr=i;
            break;
        }
    }

	return uMbRelAddr;
}
/******************************************************************************
* ��    �ƣ�Iec104SyncTime()
* ��    �ܣ���ʱ��67
* ��ڲ�����
            *pA         IEC��Ϣָ��
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104SyncTime(IEC104_MAIN_T *pA)
{
    uint8_t uBit=0;
    Uint_Char_Convert temp;
    SYSTEMTIME sSysTime;
    SYSTEMTIME *pGetTime;
    static uint8_t uSysTimeMark=0;
	RealTimeInit();
    if(R_ACTIVE==pA->recv.format.reasonL) // ����06
    {
        temp.c[0] = pA->recv.format.data[0]; //pA->rec_buff[15];  // �����λ
        temp.c[1] = pA->recv.format.data[1]; //pA->rec_buff[16];  // �����λ

        sSysTime.Second = temp.u/1000;
        sSysTime.Minute = pA->recv.format.data[2]; //pA->rec_buff[17]; // ��
        sSysTime.Hour   = pA->recv.format.data[3]; //pA->rec_buff[18]; // ʱ
        sSysTime.Date   = pA->recv.format.data[4]&0x1f; //pA->rec_buff[19]; // �գ�����3λΪ����
        sSysTime.Month  = pA->recv.format.data[5]; //pA->rec_buff[20]; // ��
        sSysTime.Year   = pA->recv.format.data[6]; //pA->rec_buff[21]; // ��
        sSysTime.Week   = pA->recv.format.data[4]>>5; //pA->rec_buff[19]; // �գ�����3λΪ����
        
        msleep(1);
        RealTimeSet(&sSysTime);// ���õ�ʱ��оƬ
        msleep(10);
        //HwDeviceTime_Set();
        pGetTime = RealTimeGet();

        DEBUGOUT("��ʱ%d:%d-%d-%d %d:%d:%d ��%d\n",uSysTimeMark,
                 pGetTime->Year,pGetTime->Month,pGetTime->Date,pGetTime->Hour,pGetTime->Minute,pGetTime->Second,pGetTime->Week);
        if((uSysTimeMark < 3) && (pGetTime->Year==226) && (pGetTime->Month==1) && (pGetTime->Date==1))
        {
            uSysTimeMark++;
            RealTimeInit();
            msleep(1);
            RealTimeSet(&sSysTime);// ���õ�ʱ��оƬ
            msleep(1);
            //HwDeviceTime_Set();
            pGetTime = RealTimeGet();

            DEBUGOUT("��ʱ%d:%d-%d-%d %d:%d:%d ��%d\n",uSysTimeMark,
                     pGetTime->Year,pGetTime->Month,pGetTime->Date,pGetTime->Hour,pGetTime->Minute,pGetTime->Second,pGetTime->Week);
            //continue;
        }
        
       /* if(uSysTimeMark >= 3)
        {
           sleep(2);
           Reboot();
        }*/
        
        IecCpoyMesAddr(pA);// ������Ϣ���ַ

        memcpy( pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));// I֡����
        IecCreateFrameI(C_CS_NA_1,pA->recv.format.limit,R_ACTIVE_ACK,(pA->recv.format.len-13),&pA->send);

        g_LoggerRun.north_status = NORTH_OK;  // ����ͨѶ���������ӣ��յ���ʱ֡����

        Iec104ReportMaxDevice(R_INFO_REPORT,pA);
        //Slave_Init(g_LoggerInfo.inquire_interval_time*100,1);     // �����ѯ��ʼ��
        for(uBit=0;uBit<MAX_device;uBit++)
        {
            if(g_LoggerRun.err_lost&(1<<uBit))
            {
                AlarmReport(g_DeviceSouth.device_inf[uBit].addr,              // �豸ͨѶ��ַ
                                             0x01,                                 // �����豸ͨѶ�쳣
                                             0x0000,                               // MODBUS�Ĵ�����ַ
                                             0x00,                                 // ƫ����
                                             JUDGE(g_LoggerRun.err_lost&(1<<uBit)), // �澯������ָ�
                                             1);    // �������ϱ�
                //g_LoggerAlarm.dev_lost &= ~(1<<uBit);              // ��������豸�����澯��ǣ��������ϱ�
            }
        }
        //soft_timer_stop(&OnTime);

        OSQPost(MesQ, &s_uSouthSync);
    }
}
/******************************************************************************
* ��    �ƣ�Iec104InputTable()
* ��    �ܣ�������BB 88 89 8A
* ��ڲ�����
            *pA         IEC��Ϣָ��
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104InputTable(IEC104_MAIN_T *pA)
{
    uint16_t i,j;
    uint8_t k;
    Uint_Char_Convert temp;
//    uint16_t gYcSum = 0;

    if(R_INPUT_GLO_INFO==pA->recv.format.reasonL)		// ����ԭ��-����ȫ����Ϣ0x88
    {
    	gImport_Table_time = OSTimeGet();
        g_LoggerRun.run_status = RUNNING_INPUT_GOLB;  	// �������״̬Ϊ����ȫ����Ϣ

		g_DeviceSouth.device_sum = pA->recv.format.data[0];  	// �豸����
		g_DeviceSouth.yx_sum     = pA->recv.format.data[2]<<8 | pA->recv.format.data[1];  // ң������
		g_DeviceSouth.yc_sum     = pA->recv.format.data[4]<<8 | pA->recv.format.data[3];  // ң������
		g_DeviceSouth.yk_sum     = pA->recv.format.data[6]<<8 | pA->recv.format.data[5];  // ң������
		g_DeviceSouth.sd_sum     = pA->recv.format.data[8]<<8 | pA->recv.format.data[7];  // ң������
		g_DeviceSouth.dd_sum     = pA->recv.format.data[10]<<8 | pA->recv.format.data[9]; // ң������
//		gYcSum = g_DeviceSouth.yc_sum;
//		printf("/************Pinnet gYcSum = %d\r\n",gYcSum);
		IecCpoyMesAddr(pA);

		memcpy(pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));// I֡����
		IecCreateFrameI(P_INPUT_TABLE,pA->recv.format.limit,R_INPUT_GLO_ACK,(pA->recv.format.len-13),&pA->send);
		//--------------------------------------------------------------------------------------------------
		//iec_run.running = Iec104_status_process(iec_run.running,P_INPUT_TABLE,R_INPUT_GLO_INFO);//RUNNING_INPUT_GLO;
		g_LoggerRun.run_status = RUNNING_INPUT_GOLB;
    }
    else if(R_INPUT_TYPE_INFO==pA->recv.format.reasonL)	// ����ԭ��-�����豸������Ϣ0x89
    {
    	gImport_Table_time = OSTimeGet();
    	g_LoggerRun.run_status = RUNNING_INPUT_TABLE;  	// �������״̬Ϊ��������Ϣ
		if(NULL==pRegPointTemp)
		{
			pRegPointTemp = (LOGGER_MODBUS_REG_T*)WMemMalloc(pRegPointTemp,MAX_POINT_ONE_TABLE * sizeof(LOGGER_MODBUS_REG_T));   // ����ռ�

			s_sPointRecord.uLastTableNum = 0;
			s_sPointRecord.uRelPoint = 0;
			s_sPointRecord.uLastPointCount = 0;

			if(NULL==pRegPointTemp)
			{
//				DEBUGOUT("Ҫ�����ʱ�ռ�ʧ��\n");				// ����ռ�ʧ��
				return;
			}
		}

		temp.c[0] = pA->recv.format.maddrL;
		temp.c[1] = pA->recv.format.maddrM;

		if(s_sPointRecord.uLastTableNum != temp.u)
		{
			if(0!=s_sPointRecord.uLastTableNum)  		// ��¼�ĵ��ŷ��㣬��һ�ŵ�������
			{
				g_pRegPoint[s_sPointRecord.uRelPoint] = (LOGGER_MODBUS_REG_T*)WMemMalloc(g_pRegPoint[s_sPointRecord.uRelPoint],s_sPointRecord.uLastPointCount*sizeof(LOGGER_MODBUS_REG_T));   // ����ռ�

				if(NULL!=g_pRegPoint[s_sPointRecord.uRelPoint])
				{
					memcpy(g_pRegPoint[s_sPointRecord.uRelPoint],pRegPointTemp,sizeof(LOGGER_MODBUS_REG_T)*s_sPointRecord.uLastPointCount);
				}
				else
				{
					DEBUGOUT("Ҫ���ռ�ʧ�� ! ! ! \r\n");
				}

				s_sPointRecord.uRelPoint++;
				s_sPointRecord.uLastTableNum = temp.u;  // ��¼���ε���
				s_sPointRecord.uLastPointCount = 0;		// �Ѿ���¼����Ϣ��������
			}
			else // ͷһ�ŵ��
			{
				s_sPointRecord.uLastTableNum = temp.u;  // ��¼���ε���
			}

			for(i=1,j=s_sPointRecord.uLastPointCount; i<pA->recv.format.len-14; j++) //
			{
				pRegPointTemp[j].reg_addr           = pA->recv.format.data[i+1]<<8 | pA->recv.format.data[i];  // �Ĵ�����ַ
				pRegPointTemp[j].reg_type.type.mess = pA->recv.format.data[i+2];   // ��Ϣ���ͣ�ң�š�ң�⡭��
				pRegPointTemp[j].reg_count          = pA->recv.format.data[i+3];   // ��Ϣ�㳤��
				pRegPointTemp[j].reg_type.type.data = pA->recv.format.data[i+4];   // �������ͣ����͡����㡢�ַ�������
				i += 5;
//				DEBUGOUT("/*********** Pinnet Device%d Information point:0x%d!\r\n",j,pRegPointTemp[j].reg_type.type.mess);
			}
			s_sPointRecord.uLastPointCount = j;
		}
		else //����������
		{
			if(preFrameDataCrc != CalculateCRC(pA->recv.format.data,(pA->recv.format.len-13)))
			{
				for(i=1,j=s_sPointRecord.uLastPointCount; i<pA->recv.format.len-14; j++)
				{
					if(j <= MAX_POINT_ONE_TABLE)
					{
						pRegPointTemp[j].reg_addr           = pA->recv.format.data[i+1]<<8 | pA->recv.format.data[i];  //�Ĵ�����ַ
						pRegPointTemp[j].reg_type.type.mess = pA->recv.format.data[i+2];   //��Ϣ���ͣ�ң�š�ң�⡭��
						pRegPointTemp[j].reg_count          = pA->recv.format.data[i+3];   //��Ϣ�㳤��
						pRegPointTemp[j].reg_type.type.data = pA->recv.format.data[i+4];   //�������ͣ����͡����㡢�ַ�������
						i += 5;
//						DEBUGOUT("/*********** Pinnet Device%d Information point:0x%d!\r\n",j,pRegPointTemp[j].reg_type.type.mess);
					}
					else
					{
//						DEBUGOUT("\nInformation point beyond!\r\nTotal number of information point��%d\r\n",s_sPointRecord.uLastPointCount);
						j -= 1;
						return;
					}
				}
				s_sPointRecord.uLastPointCount = j;
			}
			else
			{
				printf("Repeat the import point table!!!\r\n");
				Data_resend_count += 1;
				if(Data_resend_count > 1)
					g_LoggerRun.run_status = RUNNING_WORK_READ;  	//��������״̬-������ɿ�ʼ��������
				return;
			}
		}

		Data_resend_count = 0;
		g_DeviceSouth.protocol[s_sPointRecord.uRelPoint].protocol_num   = s_sPointRecord.uLastTableNum;  	//����
		g_DeviceSouth.protocol[s_sPointRecord.uRelPoint].protocol_type  = pA->recv.format.data[0];         	//�������
		g_DeviceSouth.protocol[s_sPointRecord.uRelPoint].mess_point_sum = s_sPointRecord.uLastPointCount;   //��Ϣ������
		DEBUGOUT("\nTotal number of information point��%d\n",s_sPointRecord.uLastPointCount);
		preFrameDataCrc = CalculateCRC(pA->recv.format.data,(pA->recv.format.len-13));

		IecCpoyMesAddr(pA);
		memcpy(pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));		// I֡����
		IecCreateFrameI(P_INPUT_TABLE,pA->recv.format.limit,R_INPUT_TYPE_ACK,(pA->recv.format.len-13),&pA->send);

		g_LoggerRun.run_status = RUNNING_INPUT_TABLE;
    }
    else if(R_INPUT_DEV_INFO==pA->recv.format.reasonL)// ����ԭ��-�����豸��Ϣ0x8A
    {
    	gImport_Table_time = OSTimeGet();
        g_LoggerRun.run_status = RUNNING_INPUT_104;  // �������״̬Ϊ�����豸104����Ϣ

        //for(i=0,j=0; i<(pA->recv.format.len-10) && j<MAX_device; j++)
        for(i=0,j=0; i<(pA->recv.format.len-10);)
        {
            j = IecAddrConvert(pA->recv.buff[i+12]);
            if(0xff==j) // û����Ӧ���豸
            {
                i += 15;
                continue;
            }
            //g_DeviceSouth.device_inf[j].addr          = pA->recv.buff[i+12];  // ͨѶ��ַ
            g_DeviceSouth.device_inf[j].protocol_num  = pA->recv.buff[i+16]<<8 | pA->recv.buff[i+15];  // ���Ե���
            g_DeviceSouth.device_inf[j].yx_start_addr = pA->recv.buff[i+18]<<8 | pA->recv.buff[i+17];  // ң����ʼ��ַ
            g_DeviceSouth.device_inf[j].yc_start_addr = pA->recv.buff[i+20]<<8 | pA->recv.buff[i+19];  // ң����ʼ��ַ
            g_DeviceSouth.device_inf[j].yk_start_addr = pA->recv.buff[i+22]<<8 | pA->recv.buff[i+21];  // ң����ʼ��ַ
            g_DeviceSouth.device_inf[j].sd_start_addr = pA->recv.buff[i+24]<<8 | pA->recv.buff[i+23];  // �����ʼ��ַ
            g_DeviceSouth.device_inf[j].dd_start_addr = pA->recv.buff[i+26]<<8 | pA->recv.buff[i+25];  // �����ʼ��ַ
            i += 15;
//            printf("/************Pinnet g_DeviceSouth.device_inf[%d].yx_start_addr =  0x%04X\r\n",j,g_DeviceSouth.device_inf[j].yx_start_addr);
//            printf("/************Pinnet g_DeviceSouth.device_inf[%d].yc_start_addr =  0x%04X\r\n",j,g_DeviceSouth.device_inf[j].yc_start_addr);
//            printf("/************Pinnet g_DeviceSouth.device_inf[%d].yk_start_addr =  0x%04X\r\n",j,g_DeviceSouth.device_inf[j].yk_start_addr);
//            printf("/************Pinnet g_DeviceSouth.device_inf[%d].sd_start_addr =  0x%04X\r\n",j,g_DeviceSouth.device_inf[j].sd_start_addr);
//            printf("/************Pinnet g_DeviceSouth.device_inf[%d].dd_start_addr =  0x%04X\r\n",j,g_DeviceSouth.device_inf[j].dd_start_addr);

            // �ҳ���Ե���
            for(k=0; k<MAX_device; k++)
            {
                if(g_DeviceSouth.device_inf[j].protocol_num == g_DeviceSouth.protocol[k].protocol_num)
                {
                    g_DeviceSouth.device_inf[j].rel_num = k;  // �����Ե�ַ���洢���������±�
                    g_DeviceSouth.device_inf[j].protocol_type = g_DeviceSouth.protocol[k].protocol_type;
                    break;
                }
            }
        }

        IecCpoyMesAddr(pA);

        memcpy(pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));// I֡����
        IecCreateFrameI(P_INPUT_TABLE,pA->recv.format.limit,R_INPUT_DEV_ACK,(pA->recv.format.len-13),&pA->send);
        //--------------------------------------------------------------------------------------------------
        //iec_run.running = Iec104_status_process(iec_run.running,P_INPUT_TABLE,R_INPUT_DEV_INFO);
        g_LoggerRun.run_status = RUNNING_INPUT_104;
    }
}
/******************************************************************************
* ��    �ƣ�Iec104TableState()
* ��    �ܣ�����ָ�C5
* ��ڲ�����
            *pA         IEC��Ϣָ��
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104TableState(IEC104_MAIN_T *pA)
{
    //static uint8_t s_uTableStart=0;
    uint8_t  i,temp,rel_num;
    uint16_t j;
    uint16_t space;  // ռ�ÿռ䣬�ֽ���
    uint32_t addr;   // �洢��ַ

    if(0x01==pA->recv.format.data[0] ) // ��������
    {
        g_LoggerRun.run_status = RUNNING_INPUT_START;  // �������״̬Ϊ��������
        gImport_Table_time = OSTimeGet();

        //s_uTableStart = 1;
    }
    else if(0x04==pA->recv.format.data[0])// && s_uTableStart) // �������
    {
    	gImport_Table_time = 0;
        //s_uTableStart = 0;
        if(s_sPointRecord.uLastPointCount)  // ��������һ�ŵ����û�и��Ƶ��������
        {
            //g_pRegPoint[s_sPointRecord.uRelPoint] = (LOGGER_MODBUS_REG_T*)calloc(s_sPointRecord.uLastPointCount,sizeof(LOGGER_MODBUS_REG_T));   // ����ռ�

            /*if(NULL!=g_pRegPoint[s_sPointRecord.uRelPoint]) // ����ռ�ָ��ǿգ����ͷſռ䣬������ռ�
            {
                g_pRegPoint[s_sPointRecord.uRelPoint] = WMemFree(g_pRegPoint[s_sPointRecord.uRelPoint]);
            }*/

            g_pRegPoint[s_sPointRecord.uRelPoint] = (LOGGER_MODBUS_REG_T*)WMemMalloc(g_pRegPoint[s_sPointRecord.uRelPoint],s_sPointRecord.uLastPointCount*sizeof(LOGGER_MODBUS_REG_T));   // ����ռ�
            if(NULL!=g_pRegPoint[s_sPointRecord.uRelPoint])
            {
                memcpy(g_pRegPoint[s_sPointRecord.uRelPoint],pRegPointTemp,sizeof(LOGGER_MODBUS_REG_T)*s_sPointRecord.uLastPointCount);
            }
            else
            {
                DEBUGOUT("Ҫ���ռ�ʧ��2\n");
            }

            s_sPointRecord.uRelPoint        = 0x00;   // �����Ե���
            s_sPointRecord.uLastTableNum    = 0x00;   // ��¼���ε���
            s_sPointRecord.uLastPointCount  = 0x00;   // �Ѿ���¼����Ϣ��������
        }

        pRegPointTemp = WMemFree(pRegPointTemp); // �ͷſռ�
        RecordInit(1);   // ��ʷ���ݴ洢����
        // ����DataFlash�ռ䣬һ����������������8KB
        DataFlash_Sector_Erase(DATAFLASH_POINT_HEAD);
        DataFlash_Sector_Erase(DATAFLASH_POINT_HEAD+0x8000);
        // ����������
        for(i=0,addr=DATAFLASH_POINT_HEAD; i<MAX_device; i++)//&&i<g_DeviceSouth.device_sum
        {
            if(0!=g_DeviceSouth.protocol[i].mess_point_sum)
            {
                space = sizeof(LOGGER_MODBUS_REG_T)*g_DeviceSouth.protocol[i].mess_point_sum;
                DataFlash_Write(addr,(uint8_t*)g_pRegPoint[i],space);  // ���뵽DataFlash

                g_DeviceSouth.protocol[i].flash_addr = addr;
                addr += space;

                for(j=0,temp=0; j<g_DeviceSouth.protocol[i].mess_point_sum; j++)
                {
                    if(TYPE_GJ==g_pRegPoint[i][j].reg_type.type.mess) // ��Ϣ������Ϊ�澯//    g_DeviceSouth.protocol[i].protocol_num)
                    {
                        temp++;
                    }
                }
                //-------------------------------------------------
                g_DeviceSouth.protocol[i].alarm_sum = temp;  // ��¼�澯�����
                //-------------------------------------------------

            }
            else
            {
                continue;
            }
        }
        SaveEepData(EEP_DEVICE_SOUTH);//EepSavedata(EEP_LOGGER_DEVICE_INF_HEAD,(uint8_t *)&g_DeviceSouth,sizeof(g_DeviceSouth),&g_DeviceSouth.CRC);// �豸��Ϣ�͵����Ϣ�洢

        //-----------------------------------------------------------------
        addr = EEP_ALARM_HEAD + sizeof(g_LoggerAlarm);  // �洢�����豸�澯ֵ����ʼ��ַ
        for(i=0; i<MAX_device; i++) // &&i<g_DeviceSouth.device_sum
        {
            if(0==g_DeviceSouth.device_inf[i].addr)
            {
                continue;
            }
            rel_num = g_DeviceSouth.device_inf[i].rel_num;   // �����豸��Ӧ����Ե���
            space   = g_DeviceSouth.protocol[rel_num].alarm_sum * sizeof(SOUTH_ALARM_T);

            if(space)     // ��Ӧ�ĵ���и澯��
            {
                /*if(NULL!=g_psSouthAlarm[i])
                {
                    g_psSouthAlarm[i] = WMemFree(g_psSouthAlarm[i]);
                }*/
                g_psSouthAlarm[i] = (SOUTH_ALARM_T*)WMemMalloc(g_psSouthAlarm[i],space);   // ����ռ�


                /*if(NULL!=g_psSouthAlarmCopy[i])
                {
                    g_psSouthAlarmCopy[i] = WMemFree(g_psSouthAlarmCopy[i]);
                }*/
                g_psSouthAlarmCopy[i] = (SOUTH_ALARM_T*)WMemMalloc(g_psSouthAlarmCopy[i],space);   // ����ռ�


                if(NULL!=g_psSouthAlarm[i])
                {
                    // �����������ĸ澯��Ĵ�����ַ������ֵ�������豸�ĸ澯�ṹ��
                    for(j=0,temp=0; j<g_DeviceSouth.protocol[rel_num].mess_point_sum; j++)
                    {
                        if(TYPE_GJ == g_pRegPoint[rel_num][j].reg_type.type.mess) // ��Ϣ������Ϊ�澯//    g_DeviceSouth.protocol[i].protocol_num)
                        {
                            g_psSouthAlarm[i][temp].mdbus_addr  = g_pRegPoint[rel_num][j].reg_addr;
                            g_psSouthAlarm[i][temp].alarm_value = 0x00;

                            g_psSouthAlarmCopy[i][temp].mdbus_addr  = g_pRegPoint[rel_num][j].reg_addr;
                            g_psSouthAlarmCopy[i][temp].alarm_value = 0x00;
                            temp++;
                        }
                    }
                }
                // �洢
                EepSavedata(addr,(uint8_t *)&g_psSouthAlarm[i][0],space,NULL);
                addr += space;
            }
        }
        //-----------------------------------------------------------------
		//g_DeviceEsn.uEsnMark[g_DeviceSouth.device_inf[s_sPointRecord.uRelDev].addr]=1;      
        //DEBUGOUT("addr:%d",g_DeviceSouth.device_inf[s_sPointRecord.uRelDev].addr);
		SaveEepData(EEP_DEVICE_ESN);  // �洢�����豸ESN
        SaveEepData(EEP_DEVICE_SOFT);  // �洢�����豸ESN
        //-----------------------------------------------------------------
        IecInit();  // iec104���ݱ����³�ʼ��
        //-----------------------------------------------------------------
        g_LoggerRun.run_status = RUNNING_WORK_READ;  // ��������״̬-������ɿ�ʼ��������

    }
    IecCpoyMesAddr(pA);
    memcpy(pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));// I֡����
	IecCreateFrameI(P_TABLE,pA->recv.format.limit,R_TABLE_START,(pA->recv.format.len-13),&pA->send);
	
}
/******************************************************************************
* ��    �ƣ�IecReportSouthDevice()
* ��    �ܣ��ϱ��Ѵ��ڵ������豸������Ӧƽ̨��ѯ��
* ��ڲ�����
            ��
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
uint8_t IecReportSouthDevice(uint16_t uReason)
{
    uint8_t j,k=0;
    static uint8_t uRelAddr=0;

    g_sIEC.send.format.maddrL = 0x00;
    g_sIEC.send.format.maddrM = 0x00;
    g_sIEC.send.format.maddrH = 0x00;

    for(;uRelAddr<MAX_device;uRelAddr++)
    {
        if(g_DeviceSouth.device_inf[uRelAddr].addr)
        {
            j = k * 25;

            g_sIEC.send.format.data[j] = g_DeviceSouth.device_inf[uRelAddr].addr;
            j += 1;
            memcpy(&g_sIEC.send.format.data[j],&g_DeviceEsn.cDeviceEsn[uRelAddr],20);// I֡����
            j += 20;
            g_sIEC.send.format.data[j] = g_DeviceSouth.device_inf[uRelAddr].protocol_num&0xff;
            j += 1;
            g_sIEC.send.format.data[j] = g_DeviceSouth.device_inf[uRelAddr].protocol_num>>8;
            j += 1;
            g_sIEC.send.format.data[j] = 1;
            j += 1;
            g_sIEC.send.format.data[j] = g_DeviceSouth.device_inf[uRelAddr].protocol_type;
            k++;

            if(k>=9)  // һ֡�������9���豸��Ϣ
            {
                uRelAddr++;
                break;
            }
        }
        else
        {
            continue;
        }
    }

    if(k)
    {
        IecCreateFrameI(P_SOUTH_INFO,1,uReason,25*k,&g_sIEC.send);
        k = 0;

        return 1;
    }
    else
    {
        if(uRelAddr>=MAX_device)
        {
            uRelAddr = 0;
            return 0;
        }
    }
    return 0;
}
/******************************************************************************
* ��    �ƣ�Iec104SouthDevInfo()
* ��    �ܣ������豸��C4
* ��ڲ�����
            *pA         IEC��Ϣָ��
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104SouthDevInfo(IEC104_MAIN_T *pA)
{
    uint8_t uRec,i;

//    if(NULL != pRegPointTemp)
//	{
//		pRegPointTemp = WMemFree(pRegPointTemp); 	  	//�ͷſռ�
//	}
	Data_resend_count = 0;

    if(R_SETTING==pA->recv.format.reasonL)// ����ԭ��-����0x92
    {
        if(RUNNING_INPUT_START==g_LoggerRun.run_status ||  // �Ѿ��յ�C5 01
           RUNNING_INPUT_GOLB ==g_LoggerRun.run_status ||  // BB 88��
           RUNNING_INPUT_TABLE==g_LoggerRun.run_status ||  // BB 89��
           RUNNING_INPUT_104  ==g_LoggerRun.run_status)    // BB 8A��
        {
            return;
        }

        if(0x01==pA->recv.format.reasonH || RUNNING_INPUT_SOUTH!=g_LoggerRun.run_status) // ͷһ֡��Ϣ
        {
            s_sPointRecord.uRelDev = 0x00;
            AllReset(1); // ��������
        }

        g_LoggerRun.run_status = RUNNING_INPUT_SOUTH;  // �������״̬Ϊ������Ϣ

        for(i=0; i<(pA->recv.format.len-13) && s_sPointRecord.uRelDev<10;)
        {
            if(pA->recv.format.data[i] > MAX_device)  // ͨѶ��ַ�����������
            {
                i += 25;
                continue;
            }
            if(0xff!=IecAddrConvert(pA->recv.format.data[i]))  // �·����豸��ַ�ظ�
            {
                i += 25;
                continue;
            }
            g_DeviceSouth.device_inf[s_sPointRecord.uRelDev].addr = pA->recv.format.data[i];     // �����豸ͨѶ��ַ
            memcpy(g_DeviceEsn.cDeviceEsn[s_sPointRecord.uRelDev],&pA->recv.format.data[i+1],20); // �����豸ESN��
            g_DeviceSouth.device_inf[s_sPointRecord.uRelDev].protocol_num = (pA->recv.format.data[i+22]<<8 | pA->recv.format.data[i+21]); // ����ʶ������
            // [i+23]Ϊ���Ӷ˿ڣ�����Bֻ��һ��ͨѶ�˿ڣ������ú���
            g_DeviceSouth.device_inf[s_sPointRecord.uRelDev].protocol_type = pA->recv.format.data[i+24]; // �����豸Э������,1:��ΪMODBUS��2����־MODBUS
            s_sPointRecord.uRelDev++;
            i += 25;
        }

        IecCpoyMesAddr(pA);

        memcpy(pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));// I֡����
        IecCreateFrameI(P_SOUTH_INFO,pA->recv.format.limit,(pA->recv.format.reasonH<<8 | R_SET_SUC),(pA->recv.format.len-13),&pA->send);

        //iec_run.running = Iec104_status_process(iec_run.running,P_SOUTH_INFO,R_SETTING);
        SaveEepData(EEP_DEVICE_ESN);
    }
    else if(R_INQUIRE==pA->recv.format.reasonL)// ����ԭ��-����0x90
    {
        uRec = IecReportSouthDevice(R_INQUIRE_SUC);  //

        if(!uRec)  // �Ѿ�ûδ�ϱ��������豸
        {
            IecCpoyMesAddr(pA);

            pA->send.format.data[0] = 0x95;
            IecCreateFrameI(P_SOUTH_INFO,pA->recv.format.limit,R_INQUIRE_SUC,1,&pA->send);
        }
    }
    else if(R_SET_SUC==pA->recv.format.reasonL)// ����ԭ��-����0x93
    {
        ReportCtrlClear(REPORT_OTHERS); // �Ѿ��������豸������
    }
}
/******************************************************************************
* ��    �ƣ�Iec104DevReportAck()
* ��    �ܣ��ϱ��Է����豸ȷ�ϡ�C7
* ��ڲ�����
            *pA         IEC��Ϣָ��
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104DevReportAck(IEC104_MAIN_T *pA)
{
    if(R_SET_SUC==pA->recv.format.reasonL)// ����ԭ��-����0x93
    {
        if(ReportCtrlRead(REPORT_HW_DEVICE))
        {
            ReportCtrlClear(REPORT_HW_DEVICE); // ȡ����Ƿ�����һ֡�ϱ���Ϊ�豸��Ϣ����
        }
        if(ReportCtrlRead(REPORT_HW_SOFT))
        {
            ReportCtrlClear(REPORT_HW_SOFT); // ȡ����Ƿ�����һ֡�ϱ���Ϊ�豸��Ϣ����
            Iec104HandleHwSoftRenovate(pA);
        }
    }
    gImport_Table_time = 0;
}

/******************************************************************************
* ��    �ƣ�Iec104LoggerUpdate()
* ��    �ܣ������������ݡ�
* ��ڲ�����
*           *pA         IEC��Ϣָ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
static void Iec104LoggerUpdate(IEC104_MAIN_T *pA)
{
    uint16_t check_crc;
    uint16_t uTmp;
    uint16_t uTmpCrc;
    Uint_Char_Convert temp;
    static uint16_t s_uRecseq=0,s_uRecAll=0; // �������
    static uint8_t  s_uUpdateMark=0;         // ��ʼ�������
    //static uint16_t uUpdataDataAllLen=0;   //������������ܳ���
    //OS_CPU_SR  cpu_sr;

    uint8_t uTmpData[200];

    if(R_TRANS_START==pA->recv.format.reasonL) // ��ʼ��������0x80
    {
        if(0==g_LoggerRun.update)
        {
            if(RUNNING_INPUT_START ==g_LoggerRun.run_status || // ��������C5
                RUNNING_INPUT_SOUTH==g_LoggerRun.run_status || // ���������豸��ϢC4 92
                RUNNING_INPUT_GOLB ==g_LoggerRun.run_status || // ����ȫ����ϢBB 88
                RUNNING_INPUT_TABLE==g_LoggerRun.run_status || // ������BB 89
                RUNNING_INPUT_104  ==g_LoggerRun.run_status    // �����豸104����ϢBB 8A
              )
            {
                return;
            }
        }
        else
        {
            return;
        }

        IecCpoyMesAddr(pA);

        pA->send.format.data[0] = 0x81;

        //FEED_DOG();  // ι���Ź�

        // DataFlash�洢�������������������96KB��һ������+8������
        //FEED_DOG();     // ι��
        DataFlash_Block_Erase(0);// ������0
        DataFlash_Sector_Erase(0x010000);
        DataFlash_Sector_Erase(0x011000);
        DataFlash_Sector_Erase(0x012000);
        DataFlash_Sector_Erase(0x013000);
        DataFlash_Sector_Erase(0x014000);
        DataFlash_Sector_Erase(0x015000);
        DataFlash_Sector_Erase(0x016000);
        DataFlash_Sector_Erase(0x017000);

        s_uRecseq = 0;
        s_uRecAll = 0;

        //uSide = pA->recv.format.maddrL&0x0f;
        if(pA->recv.format.maddrL&0x01)
        {
            g_LoggerRun.update = 0xA0;   // ������A��
        }
        else
        {
            g_LoggerRun.update = 0xB0;   // ������B��
        }

        g_uNowTime = OSTimeGet();
        g_uUpdataTime = g_uNowTime;  // ���³�ʱ����ʱ��

        DEBUGOUT("\n��������\n");

        IecCreateFrameI(P_UPDATA,0x01,R_TRANS_START_ACK,1,&pA->send);

        s_uUpdateMark = 1;
    }
    else if(R_DATA_TRANS==pA->recv.format.reasonL) // ���ݴ�����0x82
    {
        if(0==s_uUpdateMark)
        {
            DEBUGOUT("\n��������\n");
            return;
        }

        temp.c[1] = pA->recv.buff[pA->recv.format.len+1];
        temp.c[0] = pA->recv.buff[pA->recv.format.len];

        check_crc = CalculateCRC(pA->recv.format.data,pA->recv.format.len-15);//CRC16(pA->recv.format.data,pA->recv.format.len-15);

        if((check_crc==temp.u))  // CRCУ��ͨ��
        {
            temp.c[0] = pA->recv.buff[12];
            temp.c[1] = pA->recv.buff[13];

            if((temp.u == (s_uRecseq+1)) || (0==temp.u && 0==s_uRecseq))
            {
                s_uRecseq = temp.u;
                s_uRecAll++;
                // ���յİ����
                IecCpoyMesAddr(pA);

                uTmp = pA->recv.format.len - 15;

                if(uTmp>200)
                {
                    DEBUGOUT("����ʧ��\n");
                    s_uUpdateMark = 0;
                    msleep(100);
                    Reboot();
                    return;
                }


                //OS_ENTER_CRITICAL();
                // �洢��DataFlash���������������ݳ��ȣ���ȥ104���ĺ�CRCУ��
                DataFlash_Write(s_uRecseq * 200,(uint8_t *)pA->recv.format.data,uTmp);
                //OS_EXIT_CRITICAL();

                msleep(10);

                memset(uTmpData,0,200);

                //OS_ENTER_CRITICAL();
                temp.u = DataFlash_Read(s_uRecseq * 200,uTmpData,uTmp);
                uTmpCrc = CalculateCRC(uTmpData,uTmp);
                //OS_EXIT_CRITICAL();

                if(check_crc!=uTmpCrc)   //������У��CRC
                {
                    DEBUGOUT("�����ض�DF\n");
                    temp.u = DataFlash_Read(s_uRecseq * 200,uTmpData,uTmp);
                    uTmpCrc = CalculateCRC(uTmpData,uTmp);

                    if(check_crc!=uTmpCrc)   //������У��CRC
                    {
                        DEBUGOUT("����%d��ʧ��%d-%d,��%04X,��%04X\n",s_uRecseq,temp.u,uTmp,uTmpCrc,check_crc);

                        for(temp.u=0;temp.u<uTmp;temp.u++)
                        {
                            DEBUGOUT("%02X ",uTmpData[temp.u]);
                        }

                        s_uUpdateMark = 0;

                        msleep(100);
                        Reboot();
                        return;
                    }
                }

                pA->send.format.data[0] = 0x01;//iec.ibuff[2] = 0x01;   // ����ɹ�

                IecCreateFrameI(P_UPDATA,0x01,R_DATA_TRANS,1,&pA->send);
                DEBUGOUT("\n����%d\n",s_uRecseq);
				//uUpdataDataAllLen += uTmp;
            }
            /*else if(temp.u <= s_uRecseq) // �ظ�����
            {
                // ���յİ����
                pA->send.format.maddrL = pA->recv.format.maddrL;
                pA->send.format.maddrM = pA->recv.format.maddrM;
                pA->send.format.maddrH = pA->recv.format.maddrH;
                pA->send.format.data[0] = 0x01;//iec.ibuff[2] = 0x01;   // ����ɹ�
                IecCreateFrameI(P_UPDATA,0x01,R_DATA_TRANS,1,pA);
            }*/
            else// if(temp.u > (s_uRecseq+1))
            {
                // ���յİ����
                IecCpoyMesAddr(pA);

                DEBUGOUT("\n������%d��%d\n",temp.u,s_uRecseq + 1);

                temp.u = s_uRecseq + 1;
                pA->send.format.data[0] = temp.c[0];// pA->rec_buff[12];// ��Ҫ�ش��İ����
                pA->send.format.data[1] = temp.c[1];// pA->rec_buff[13];
                pA->send.format.data[2] = 0x00;
                IecCreateFrameI(P_UPDATA,0x01,R_DATA_RETRY,3,&pA->send); // Ҫ�����ش�
            }

        }
        else // CRCУ�鲻ͨ��
        {
            // ���յİ����
            IecCpoyMesAddr(pA);

            // ��Ҫ�ش��İ����
            pA->send.format.data[0] = pA->recv.format.maddrL;
            pA->send.format.data[1] = pA->recv.format.maddrM;
            pA->send.format.data[2] = pA->recv.format.maddrH;

            IecCreateFrameI(P_UPDATA,0x01,R_DATA_RETRY,3,&pA->send); // Ҫ�����ش�
            DEBUGOUT("\n����CRC����,��%04X,��%04X\n",check_crc,temp.u);
        }

        g_uUpdataTime = g_uNowTime;  // ���³�ʱ����ʱ��
    }
    else if(R_TRANS_FINISH==pA->recv.format.reasonL) // ���ݴ������0x86
    {
        //iec_run.running = Iec104_status_process(iec_run.running,P_UPDATA,R_TRANS_FINISH);

        s_uRecseq = pA->recv.format.maddrM<<8 | pA->recv.format.maddrL;  // ����������

        IecCpoyMesAddr(pA);

        pA->send.format.data[0] = R_TRANS_FINISH_ACK;
        IecCreateFrameI(P_UPDATA,0x01,R_TRANS_FINISH_ACK,1,&pA->send);
        DEBUGOUT("\n������:%d������:%d\n",s_uRecAll,s_uRecseq);
		//DEBUGOUT("����������ܳ���:%d\n",uUpdataDataAllLen);
        if(s_uRecAll!=s_uRecseq)
        {
            return;
        }

        // �洢������Ϣ��EEPROM��Ȼ������оƬ

        //EepReadData(EEP_LOGGER_UPDATA_HEAD,(uint8_t *)data,11,NULL);// ������Ϣ�洢

        check_crc = ReadEepData(EEP_UPDATA);//EepReadData(EEP_LOGGER_UPDATA_HEAD,(uint8_t *)&updata,sizeof(UPDATA_MARK_T),&updata.CRC);// ������Ϣ��ȡ

        if(0==check_crc) // ��ȡ����ʧ��,�лع����ݱ�־���Ϊû�лع�����
        {
            g_LoggerUpdate.rollback_allow = 0x55;
        }
        g_LoggerUpdate.frame_sum = s_uRecseq * 200 / 256 + 1;// 256���ֽ�һ֡����֡��

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

        if(0xA0==g_LoggerRun.update) // ������A��
        {
            g_LoggerUpdate.side_tobe = 0xAA;  // Ŀ��������A��
        }
        else
        {
            g_LoggerUpdate.side_tobe = 0xBB;  // Ŀ��������B��
        }



        if(!s_uRecseq)  // û������������
        {
            g_LoggerUpdate.updata_mark = 0x55;  //  data[2] = 0x55;    // 0xAA������0xBB�ع���0x55��
        }
        else
        {
            g_LoggerUpdate.updata_mark = 0xAA;  // data[2] = 0xAA;    // 0xAA������0xBB�ع���0x55��
        }

        SaveEepData(EEP_UPDATA);//EepSavedata(EEP_LOGGER_UPDATA_HEAD,(uint8_t *)&updata,sizeof(UPDATA_MARK_T),&updata.CRC);// ������Ϣ�洢

        s_uRecseq = 0x00;

        sleep(3);

        Reboot();// ����оƬ
        //uSide = 0x55;// �������оƬ reboot();  // ����оƬ
    }
    else if(R_TRANS_STOP==pA->recv.format.reasonL) // ֹͣ����0x84
    {
        pA->send.format.maddrL = 0;
        pA->send.format.maddrM = 0;
        pA->send.format.maddrH = 0;
        pA->send.format.data[0] = R_TRANS_STOP_ACK;
        IecCreateFrameI(P_UPDATA,0x01,R_TRANS_STOP_ACK,1,&pA->send);
    }
}
/******************************************************************************
* ��    �ƣ�Iec104DT1000Update()
* ��    �ܣ������������ݡ�
* ��ڲ�����
*           *pA         IEC��Ϣָ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
static void Iec104DT1000Update(IEC104_MAIN_T *pA)
{
    static uint32_t uUpdateDataAllLen=0;   //������������ܳ���
	uint16_t check_crc;
    uint16_t uTmp;
    uint16_t uTmpCrc;
    Uint_Char_Convert temp;
	U32_F_Char_Convert ctemp;
    static uint16_t s_uRecseq=0,s_uRecAll=0; // �������
    static uint8_t  s_uUpdateMark=0;         // ��ʼ�������

    static uint8_t uFrameLen = 0;
    uint8_t uTmpData[200];
	static uint8_t uFileStateIO = 0;

    if(R_TRANS_START==pA->recv.format.reasonL) // ��ʼ��������0x80
    {

		if(0==g_LoggerRun.update)
		{
			if(RUNNING_INPUT_START ==g_LoggerRun.run_status || // ��������C5
				RUNNING_INPUT_SOUTH==g_LoggerRun.run_status || // ���������豸��ϢC4 92
				RUNNING_INPUT_GOLB ==g_LoggerRun.run_status || // ����ȫ����ϢBB 88
				RUNNING_INPUT_TABLE==g_LoggerRun.run_status || // ������BB 89
				RUNNING_INPUT_104  ==g_LoggerRun.run_status ||  // �����豸104����ϢBB 8A
				(g_LoggerRun.uFileIO_status != 0)
			  )
			{
				return;
			}
		}
		else
		{
			return;
		}

        IecCpoyMesAddr(pA);

		if(pA->recv.format.maddrL == S_FILE_EXPORT)  //�ļ�����״̬
		{
			 g_DT1000Updata.uDevAddr = pA->recv.format.data[0];//��ȡ�豸��ַ
			 g_DT1000Updata.uData = pA->recv.format.data[1];			 
			 g_LoggerRun.uFileIO_status = S_FILE_EXPORT;       //������������־����״̬
			 
			 uFileStateIO = S_FILE_EXPORT;  
			 return;
		}else if(pA->recv.format.maddrL == S_FILE_IMPORT)  //�ļ�����״̬
		{
             uFileStateIO = S_FILE_IMPORT;       
			 DEBUGOUT("\n�����ļ�����\n");
		}

		if(S_FILE_IMPORT == uFileStateIO)
		{
			//��ȡ�ļ�����
			ctemp.c[0] = pA->recv.format.data[19];
			ctemp.c[1] = pA->recv.format.data[20];
			ctemp.c[2] = pA->recv.format.data[21];
			ctemp.c[3] = pA->recv.format.data[22];
			g_DT1000Updata.nDataLen = ctemp.u;
			DEBUGOUT("\nUpdate FileLen:%d	 ",g_DT1000Updata.nDataLen);
	
			uFrameLen = pA->recv.format.data[23];
			memcpy(pA->send.format.data,pA->recv.format.data,24);
	
			if(pA->recv.format.data[18]&0x01)	//��ʱ������������Ϣ
			{
				g_LoggerRun.update = 0xA0;	 // ������A��
			}
			else
			{
				g_LoggerRun.update = 0xB0;	 // ������B��
			}
	
			// DataFlash�洢�������������������96KB��һ������+8������	   
			DataFlash_Block_Erase(5);// ������3
			msleep(1);
			DataFlash_Block_Erase(6);// ������4
			msleep(1);
	
			s_uRecseq = 0;
			s_uRecAll = 0;
	
			memcpy(g_DT1000Updata.version,&pA->recv.format.data[2],17);
		   
			DEBUGOUT("DT1000_Vertion: ");
			for(uint8_t i=0;i<17;i++)
			{
				DEBUGOUT("%c",g_DT1000Updata.version[i]);
			}
			DEBUGOUT("\r\n");
	
			g_uNowTime = OSTimeGet();
			g_uUpdataTime = g_uNowTime;  // ���³�ʱ����ʱ��
	
			DEBUGOUT("\n������������ļ�����\n");
			IecCreateFrameI(P_FILE_INOUT,0x01,R_TRANS_START_ACK,24,&pA->send);
			uUpdateDataAllLen=0;
			s_uUpdateMark = 1;
		}
    }
    else if(R_DATA_TRANS==pA->recv.format.reasonL) // ���ݴ�����0x82
    {
		if(S_FILE_IMPORT == uFileStateIO)
		{
			if(0==s_uUpdateMark)
			{
				DEBUGOUT("\n��������\n");
				return;
			}
	
			//֡�ļ�CRCУ��
			temp.c[1] = pA->recv.buff[pA->recv.format.len+1];
			temp.c[0] = pA->recv.buff[pA->recv.format.len];
	
			check_crc = CalculateCRC(pA->recv.format.data,pA->recv.format.len-15);//CRC16(pA->recv.format.data,pA->recv.format.len-15);
	
			if((check_crc==temp.u))  // CRCУ��ͨ��
			{
				//������ж�
				temp.c[0] = pA->recv.buff[12];
				temp.c[1] = pA->recv.buff[13];
	
				if((temp.u == (s_uRecseq+1)) || (0==temp.u && 0==s_uRecseq))
				{
					s_uRecseq = temp.u;
					s_uRecAll++;
					
					//IecCpoyMesAddr(pA);// ���յİ����
					pA->send.format.maddrL = 0x00;
					pA->send.format.maddrM = 0x00;
					pA->send.format.maddrH = 0x00;
	
					uTmp = pA->recv.format.len - 15;
	
					if(uTmp>200)
					{
						DEBUGOUT("����ʧ��\n");
						s_uUpdateMark = 0;
						uFileStateIO = 0;
						msleep(100);
						Reboot();
						return;
					}
	
					//OS_ENTER_CRITICAL();
					// �洢��DataFlash���������������ݳ��ȣ���ȥ104���ĺ�CRCУ��
					DataFlash_Write(DATAFLASH_DT1000_UPDATA_HEAD+s_uRecseq * uFrameLen,(uint8_t *)pA->recv.format.data,uTmp);
					//OS_EXIT_CRITICAL();
	
					msleep(10);
					memset(uTmpData,0,200);
	
					//OS_ENTER_CRITICAL();
					temp.u = DataFlash_Read(DATAFLASH_DT1000_UPDATA_HEAD+s_uRecseq * uFrameLen,uTmpData,uTmp);
					uTmpCrc = CalculateCRC(uTmpData,uTmp);
					//OS_EXIT_CRITICAL();
	
					if(check_crc!=uTmpCrc)	 //������У��CRC
					{
						DEBUGOUT("�����ض�DF\n");
						temp.u = DataFlash_Read(DATAFLASH_DT1000_UPDATA_HEAD+s_uRecseq * uFrameLen,uTmpData,uTmp);
						uTmpCrc = CalculateCRC(uTmpData,uTmp);
	
						if(check_crc!=uTmpCrc)	 //������У��CRC
						{
							DEBUGOUT("����%d��ʧ��%d-%d,��%04X,��%04X\n",s_uRecseq,temp.u,uTmp,uTmpCrc,check_crc);
	
							for(temp.u=0;temp.u<uTmp;temp.u++)
							{
								DEBUGOUT("%02X ",uTmpData[temp.u]);
							}
	
							s_uUpdateMark = 0;
							uFileStateIO = 0;
	
							msleep(100);
							Reboot();
							return;
						}
					}
	
					//pA->send.format.data[0] = 0x01;//iec.ibuff[2] = 0x01;   // ����ɹ�
					pA->send.format.data[0] = pA->recv.format.maddrL;
					pA->send.format.data[1] = pA->recv.format.maddrM;
					pA->send.format.data[2] = pA->recv.format.maddrH;
	
					IecCreateFrameI(P_FILE_INOUT,0x01,R_DATA_TRANS,3,&pA->send);
					DEBUGOUT("\n����֡%d\n",s_uRecseq);
					uUpdateDataAllLen += uTmp;
					DEBUGOUT("\nUpdateAllLen:%d\n",uUpdateDataAllLen);
				}
				else
				{
					IecCpoyMesAddr(pA);// ���յİ����
	
					DEBUGOUT("\n����֡��%d��%d\n",temp.u,s_uRecseq + 1);
	
					temp.u = s_uRecseq + 1;
					pA->send.format.data[0] = temp.c[0];// pA->rec_buff[12];// ��Ҫ�ش��İ����
					pA->send.format.data[1] = temp.c[1];// pA->rec_buff[13];
					pA->send.format.data[2] = 0x00;
					IecCreateFrameI(P_FILE_INOUT,0x01,R_DATA_RETRY,3,&pA->send); // Ҫ�����ش�
				}
			}
			else // CRCУ�鲻ͨ��
			{
				IecCpoyMesAddr(pA);// ���յİ����
	
				// ��Ҫ�ش��İ����
				pA->send.format.data[0] = pA->recv.format.maddrL;
				pA->send.format.data[1] = pA->recv.format.maddrM;
				pA->send.format.data[2] = pA->recv.format.maddrH;
	
				IecCreateFrameI(P_FILE_INOUT,0x01,R_DATA_RETRY,3,&pA->send); // Ҫ�����ش�
				DEBUGOUT("\n����CRC����,��%04X,��%04X\n",check_crc,temp.u);
			}
	
			g_uUpdataTime = g_uNowTime;  // ���³�ʱ����ʱ��
		}
    }
    else if(R_TRANS_FINISH==pA->recv.format.reasonL) // ���ݴ������0x86
    {
		if(S_FILE_IMPORT == uFileStateIO)
		{
			//iec_run.running = Iec104_status_process(iec_run.running,P_UPDATA,R_TRANS_FINISH);
	
			s_uRecseq = pA->recv.format.maddrM<<8 | pA->recv.format.maddrL;  // ����������
	
			IecCpoyMesAddr(pA);
	
			pA->send.format.data[0] = pA->recv.format.data[0];	//�����ֽڵ��ļ�CRCУ��
			pA->send.format.data[1] = pA->recv.format.data[1];
			IecCreateFrameI(P_FILE_INOUT,0x01,R_TRANS_FINISH_ACK,1,&pA->send);
			DEBUGOUT("\n������:%d������:%d\n",s_uRecAll,s_uRecseq);
			g_DT1000Updata.frame_sum=s_uRecAll;
			
			DEBUGOUT("DT1000 File Len:%d Frame Sum:%d\n",g_DT1000Updata.nDataLen,g_DT1000Updata.frame_sum);
			
			if(s_uRecAll!=s_uRecseq)
			{
				g_LoggerRun.update = 0x00;	  //��ʱ������������Ϣ
	
				return;
			}
			
			/*if(g_DT1000Updata.version[16]&0x01)  // С�汾�����λ��1��������A�棻0��������B��
			{
				g_LoggerRun.uFileIO_status = 0x01;	 // ������B��
			}
			else //if((g_DT1000Updata.version[16])%2 == 0)
			{
				g_LoggerRun.uFileIO_status = 0x01;	 // ������B��
			}*/
			
		   g_LoggerRun.uFileIO_status = 0x01;	   // �����������뿪ʼ
		   g_LoggerRun.update = 0x00;	 //ȡ������������Ϣ
		   
	       uFileStateIO = 0;
		   s_uRecseq = 0x00;
		   uUpdateDataAllLen = 0;
		}     
    }
    else if(R_TRANS_STOP==pA->recv.format.reasonL) // ֹͣ����0x84
    {
		if(S_FILE_IMPORT == uFileStateIO)
		{
			g_LoggerRun.update = 0x00;
			s_uRecseq = 0x00;
			s_uRecAll = 0x00;
			uUpdateDataAllLen = 0;
			
			pA->send.format.maddrL = 0;
			pA->send.format.maddrM = 0;
			pA->send.format.maddrH = 0;
			pA->send.format.data[0] = R_TRANS_STOP_ACK;
			IecCreateFrameI(P_FILE_INOUT,0x01,R_TRANS_STOP_ACK,1,&pA->send);
			uFileStateIO = 0;
		}
    }
}
/******************************************************************************
* ��    �ƣ�Iec104IP()
* ��    �ܣ�IP���ò�ѯ��B5
* ��ڲ�����
            *pA         IEC��Ϣָ��
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104IP(IEC104_MAIN_T *pA)
{
    if(R_SETTING==pA->recv.format.reasonL) // ����92
    {
        memcpy( pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));// I֡����  memcpy(&iec.ibuff[0],&pA->rec_buff[12],(pA->rec_buff[1]-10));// I֡����
        IecCreateFrameI(P_SET_IP,pA->recv.format.limit,R_SET_SUC,(pA->recv.format.len-13),&pA->send);
    }
    else if(R_INQUIRE==pA->recv.format.reasonL) // ��ѯ90
    {
        IecCpoyMesAddr(pA);

        pA->send.format.data[0] = 0;//Logger_run.IP[0];
        pA->send.format.data[1] = 0;//Logger_run.IP[1];
        pA->send.format.data[2] = 0;//Logger_run.IP[2];
        pA->send.format.data[3] = 0;//Logger_run.IP[3];

        IecCreateFrameI(P_SET_IP,pA->recv.format.limit,R_INQUIRE_SUC,4,&pA->send);
    }
}
/******************************************************************************
* ��    �ƣ�Iec104HandleModBusEndian()
* ��    �ܣ�104���ò�ѯMODBUS��С�ˡ�
* ��ڲ�����
*           *pA         IEC��Ϣָ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
uint8_t Iec104HandleModBusEndian(IEC104_MAIN_T *pA)
{
    uint8_t uMbEndian;//��С��
    //uint8_t uMbVirAddr;//�豸��ַ:�޶�����0-9ʮ�������
    uint8_t uMbRelAddr=0;//����MbVirAddrӳ������豸ʵ�ʵ�ַ
    memcpy( pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));
    //uMbVirAddr = pA->recv.format.data[0];
    uMbEndian  = pA->recv.format.data[1];
    uMbRelAddr = IecAddrConvert(pA->recv.format.data[0]);

    if(R_SETTING==pA->recv.format.reasonL) // ����92
    {
        if(uMbRelAddr<MAX_device)
        {
            g_DeviceSouth.device_inf[uMbRelAddr].big_little_endian = uMbEndian;
            SaveEepData(EEP_DEVICE_SOUTH);
        }

        IecCreateFrameI(P_MODBUS_ENDIAN,pA->recv.format.limit,pA->recv.format.reasonL+1,(pA->recv.format.len-13),&pA->send);
    }
    else if(R_INQUIRE==pA->recv.format.reasonL) // ��ѯ90
    {
        if(uMbRelAddr<MAX_device)
        {
            pA->send.format.data[1] = g_DeviceSouth.device_inf[uMbRelAddr].big_little_endian;
        }
        else
        {
            pA->send.format.data[1] = 1;
        }

        IecCreateFrameI(P_MODBUS_ENDIAN,pA->recv.format.limit,pA->recv.format.reasonL+1,(pA->recv.format.len-13),&pA->send);
    }

    return 0;
}
/******************************************************************************
* ��    �ƣ�Iec104HandleModBusBd()
* ��    �ܣ�IEC104 �豸�����ʴ���
* ��ڲ�����
*           *pA         IEC��Ϣָ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104HandleModBusBd(IEC104_MAIN_T *pA)
{
	uint32_t uMbBd;//������
	uint8_t uMbVirAddr;//�豸��ַ:�޶�����0-9�Ǹ������(�����е��±�)
	uint8_t uMbRelAddr=0;//����MbVirAddrӳ������豸ʵ�ʵ�ַ(���ҵ���������豸���±�)
	uint8_t uSendReason;
	uint8_t uMbBR=0;

	memcpy( pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));
	uMbVirAddr = pA->recv.format.data[0];
	uMbBd      = (pA->recv.format.data[3]<<16)+(pA->recv.format.data[2]<<8)+(pA->recv.format.data[1]);
	uMbRelAddr = IecAddrConvert(uMbVirAddr);


	if(R_SETTING==pA->recv.format.reasonL) // ����92
    {
        if(uMbRelAddr<MAX_device)
        {
            switch(uMbBd)
            {
            case 2400:
                uMbBR = BAUDRATE_2400;
                break;

            case 4800:
                uMbBR = BAUDRATE_4800;
                break;

            case 9600:
                uMbBR = BAUDRATE_9600;
                break;

            case 19200:
                uMbBR = BAUDRATE_19200;
                break;

            case 38400:
                uMbBR = BAUDRATE_38400;
                break;

            case 115200:
                uMbBR = BAUDRATE_115200;
                break;

            default:
                uMbBR = BAUDRATE_9600;
                break;
            }
            //
            g_DeviceSouth.device_inf[uMbRelAddr].baud_rate = uMbBR;
            SaveEepData(EEP_DEVICE_SOUTH);
        }
    	uSendReason=pA->recv.format.reasonL+1;

        IecCreateFrameI(P_MODBUS_BAUD,pA->recv.format.limit,uSendReason,(pA->recv.format.len-13),&pA->send);
    }
    else if(R_INQUIRE==pA->recv.format.reasonL) // ��ѯ90
    {
        if(uMbRelAddr<MAX_device)
        {
            uMbBR = g_DeviceSouth.device_inf[uMbRelAddr].baud_rate;
        }
        switch(uMbBR)
        {
        case BAUDRATE_2400:
            uMbBd = 2400;
            break;

        case BAUDRATE_4800:
            uMbBd = 4800;
            break;

        case BAUDRATE_9600:
            uMbBd = 9600;
            break;

        case BAUDRATE_19200:
            uMbBd = 19200;
            break;

        case BAUDRATE_38400:
            uMbBd = 38400;
            break;

        case BAUDRATE_115200:
            uMbBd = 115200;
            break;

        default:
            uMbBd = 9600;
            break;
        }
        pA->send.format.data[3] = uMbBd>>16;
        pA->send.format.data[2] = uMbBd>>8;
        pA->send.format.data[1] = uMbBd;

        IecCreateFrameI(P_MODBUS_BAUD,pA->recv.format.limit,R_INQUIRE_SUC,(pA->recv.format.len-13),&pA->send);
    }
}
/******************************************************************************
* ��    �ƣ�Iec104PublicAddr()
* ��    �ܣ�������ַ��B8
* ��ڲ�����
            *pA         IEC��Ϣָ��
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104PublicAddr(IEC104_MAIN_T *pA)
{
    if(R_SETTING==pA->recv.format.reasonL) // ����92
    {
        g_LoggerInfo.ADDR = pA->recv.format.data[0];//pA->recv.format.addrL;// >rec_buff[15];  // �µĹ�����ַ����Ҫ����eeprom

        IecCpoyMesAddr(pA);

        memcpy(pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));// I֡����
        IecCreateFrameI(P_ADDR,pA->recv.format.limit,R_SET_SUC,(pA->recv.format.len-13),&pA->send);

        DEBUGOUT("���ַ\n");
        SaveEepData(EEP_LOGGER_INF);
    }
    else if(R_INQUIRE==pA->recv.format.reasonL) // ��ѯ90
    {
        IecCpoyMesAddr(pA);

        pA->send.format.data[0] = g_LoggerInfo.ADDR;
        pA->send.format.data[1] = 0x00;
        IecCreateFrameI(P_ADDR,pA->recv.format.limit,R_INQUIRE_SUC,2,&pA->send);
    }
}
/******************************************************************************
* ��    �ƣ�Iec104InquireVer()
* ��    �ܣ��汾��ѯ��B9
* ��ڲ�����
            *pA         IEC��Ϣָ��
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104InquireVer(IEC104_MAIN_T *pA)
{
    uint8_t uRes;
    IecCpoyMesAddr(pA);

    pA->send.format.data[0] = GetVerS2();
    pA->send.format.data[1] = GetVerS1();
    pA->send.format.data[2] = GetVerType();

    uRes = ReadEepData(EEP_UPDATA);//EepReadData(EEP_LOGGER_UPDATA_HEAD,(uint8_t *)&eepdata,sizeof(UPDATA_MARK_T),&eepdata.CRC);// ������Ϣ�洢
    if(uRes)
    {
        if(GetVerS2()&0x01)// С�汾�����λ��1��������A�棻0��������B��
        {
            pA->send.format.data[3] = g_LoggerUpdate.a_version[0];
            pA->send.format.data[4] = g_LoggerUpdate.a_version[1];
            pA->send.format.data[5] = g_LoggerUpdate.a_version[2];
        }
        else
        {
            pA->send.format.data[3] = g_LoggerUpdate.b_version[0];
            pA->send.format.data[4] = g_LoggerUpdate.b_version[1];
            pA->send.format.data[5] = g_LoggerUpdate.b_version[2];
        }
    }

    IecCreateFrameI(P_VERSION,0x01,R_INQUIRE_SUC,6,&pA->send);
}
/******************************************************************************
* ��    �ƣ�Iec104LoggerRollBack()
* ��    �ܣ������������ݡ�
* ��ڲ�����
*           *pA         IEC��Ϣָ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
static void Iec104LoggerRollBack(IEC104_MAIN_T *pA)
{
    uint8_t check_crc;

    IecCpoyMesAddr(pA);

    pA->send.format.data[0] = pA->recv.format.data[0];
    pA->send.format.data[1] = pA->recv.format.data[1];
    pA->send.format.data[2] = pA->recv.format.data[2];
    IecCreateFrameI(P_ROLLBAOCK,0x01,R_SET_SUC,3,&pA->send);


    //iec_run.running = Iec104_status_process(iec_run.running,P_ROLLBAOCK,R_TRANS_FINISH);

    // �洢������Ϣ��EEPROM��Ȼ������оƬ
    //EepReadData(EEP_LOGGER_UPDATA_HEAD,(uint8_t *)data,11,NULL);// ������Ϣ�洢
    check_crc = ReadEepData(EEP_UPDATA);//EepReadData(EEP_LOGGER_UPDATA_HEAD,(uint8_t *)&updata,sizeof(UPDATA_MARK_T),&updata.CRC);// ������Ϣ��ȡ

    //check_crc = CRC16(data,9);
    //if(check_crc==((data[9]<<8)|data[10]) && 0xBB==data[3])
    if(0!=check_crc) // ��ȡ���ݳɹ�,�� �ع����ݱ�־���Ϊ�лع�����
    {
        if(0xBB==g_LoggerUpdate.rollback_allow) // ����ʷ���򣬿��Իع�
        {
            g_LoggerUpdate.frame_sum      = 0x00;
            g_LoggerUpdate.updata_mark    = 0xBB;       // 0xAA������0xBB�ع���0x55��
            g_LoggerUpdate.rollback_allow = 0xBB;


            if(GetVerS2()&0x01)// С�汾�����λ��1��������A�棻0��������B��
            {
                pA->send.format.data[3] = g_LoggerUpdate.a_version[0];
                pA->send.format.data[4] = g_LoggerUpdate.a_version[1];
                pA->send.format.data[5] = g_LoggerUpdate.a_version[2];
            }
            else
            {
                pA->send.format.data[3] = g_LoggerUpdate.b_version[0];
                pA->send.format.data[4] = g_LoggerUpdate.b_version[1];
                pA->send.format.data[5] = g_LoggerUpdate.b_version[2];
            }

            g_LoggerUpdate.reserve = 0x00;

            SaveEepData(EEP_UPDATA);//EepSavedata(EEP_LOGGER_UPDATA_HEAD,(uint8_t *)&updata,sizeof(UPDATA_MARK_T),&updata.CRC);// ������Ϣ�洢

            sleep(5);
            Reboot();  // ����оƬ
        }
    }
}
/******************************************************************************
* ��    �ƣ�Iec104PhoneNum()
* ��    �ܣ��ϱ����롣BC
* ��ڲ�����
            *pA         IEC��Ϣָ��
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104PhoneNum(IEC104_MAIN_T *pA)
{
    //uint16_t uValue=0;
    if(R_SETTING==pA->recv.format.reasonL) // ����92
    {
        IecCpoyMesAddr(pA);

		memcpy(g_LoggerInfo.phonenum,pA->recv.format.data,11);
        memcpy(pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));// I֡����
        IecCreateFrameI(P_REPORT_NUM,pA->recv.format.limit,R_SET_SUC,(pA->recv.format.len-13),&pA->send);

        DEBUGOUT("�����\n");
        SaveEepData(EEP_LOGGER_INF);

        // ����Ԥ�õ��ţ�3~6λ����
        //uValue = (g_LoggerInfo.phonenum[2]-'0')*10000 + (g_LoggerInfo.phonenum[3]-'0')*1000 + (g_LoggerInfo.phonenum[4]-'0')*100 + (g_LoggerInfo.phonenum[5]-'0')*10 +(g_LoggerInfo.phonenum[6]-'0');
       /* if(uValue<=60000)
        {
            DEBUGOUT("���ȡ��Ԥ��:%d\n",uValue);
            g_PerSetTableResq = uValue;
            SaveEepData(EEP_TABLE_SEQ);
        }
        else
        {
            DEBUGOUT("���Ԥ��:%d\n",uValue);
            g_PerSetTableResq = uValue;
            SaveEepData(EEP_TABLE_SEQ);
        }*/
    }
    else if(R_INQUIRE==pA->recv.format.reasonL) // ��ѯ90
    {
        IecCpoyMesAddr(pA);

        memcpy(pA->send.format.data,g_LoggerInfo.phonenum,11);

        IecCreateFrameI(P_REPORT_NUM,pA->recv.format.limit,R_INQUIRE_SUC,11,&pA->send);
    }
}
/******************************************************************************
* ��    �ƣ�Iec104CommMode()
* ��    �ܣ�ͨ�ŷ�ʽ��C6
* ��ڲ�����
            *pA         IEC��Ϣָ��
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104CommMode(IEC104_MAIN_T *pA)
{
    if(R_SETTING==pA->recv.format.reasonL) // ����92
    {
        IecCpoyMesAddr(pA);

        pA->send.format.data[0] = pA->recv.format.data[0];

        IecCreateFrameI(P_COM_MODE,pA->recv.format.limit,R_SET_SUC,1,&pA->send);
    }
    else if(R_INQUIRE==pA->recv.format.reasonL) // ��ѯ0x90
    {
        IecCpoyMesAddr(pA);

        pA->send.format.data[0] = 0x01;  //���߷�ʽͨѶ
        IecCreateFrameI(P_COM_MODE,pA->recv.format.limit,R_INQUIRE_SUC,1,&pA->send);
    }
}
/******************************************************************************
* ��    �ƣ�Iec104ServiceIP()
* ��    �ܣ�����������/IP��C8
* ��ڲ�����
            *pA         IEC��Ϣָ��
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104ServiceIP(IEC104_MAIN_T *pA)
{
    uint8_t uLen;
    if(R_SETTING==pA->recv.format.reasonL) // ����92
    {
        // ���Ľ�������
        uLen = strlen((char *)pA->recv.format.data);
        memset(g_LoggerInfo.server_domain,'\0',34);
        if(uLen>=30)
        {
            uLen = 30;
        }
        strncpy(&g_LoggerInfo.server_domain[1],(char *)pA->recv.format.data,uLen);
        g_LoggerInfo.server_domain[0] = '\"';
        if(uLen<30)
        {
            g_LoggerInfo.server_domain[uLen+1] = '\"';
        }
        else
        {
            memset(&g_LoggerInfo.server_domain[31],'\0',3);
            g_LoggerInfo.server_domain[31]='\"';
        }
        g_LoggerInfo.server_port = pA->recv.format.data[31]<<8 | pA->recv.format.data[30];
        DEBUGOUT("������\n");
        SaveEepData(EEP_LOGGER_INF);  // ������Ϣ

        IecCpoyMesAddr(pA);

        memcpy(pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));// I֡����
        IecCreateFrameI(P_SERVICE_IP,pA->recv.format.limit,R_SET_SUC,(pA->recv.format.len-13),&pA->send);
    }
    else if(R_INQUIRE==pA->recv.format.reasonL) // ��ѯ0x90
    {
        memset(pA->send.format.data,0x00,30);
        uLen = strlen(g_LoggerInfo.server_domain);
        strncpy((char *)pA->send.format.data,&g_LoggerInfo.server_domain[1],uLen-2); // �洢������Ϊ "����"��Ҫȥ��˫����
        pA->send.format.data[30] = g_LoggerInfo.server_port&0xff;
        pA->send.format.data[31] = g_LoggerInfo.server_port>>8;

        pA->send.format.maddrL = 0x00;
        pA->send.format.maddrM = 0x00;
        pA->send.format.maddrH = 0x00;
        IecCreateFrameI(P_SERVICE_IP,pA->recv.format.limit,R_INQUIRE_SUC,32,&pA->send);
    }
}
/******************************************************************************
* ��    �ƣ�Iec104LogExport()
* ��    �ܣ�ƽ̨��־������
* ��ڲ�����
*           *pA         IEC��Ϣָ��
*
* ���ڲ�������
* ��    ��:
* ADD BY: chenyang
******************************************************************************/
uint32_t Iec104LogExport(IEC104_MAIN_T *pA)
{
    
    static uint8_t uMenLogData[LOGFRAMELENTH] = {0};
    static uint32_t uLogDataFlash= 0;
    static LOG_INF_ADDE Iaddr = {0};
    static LOG_INF_ADDE counts = {0};

    if(NORTH_OK!=g_LoggerRun.north_status)  // ���ӷ�����δ���
    {
        return 0 ;
    }

    switch(pA->recv.format.reasonL)
    {
    case R_TRANS_START: //��ʼ����0x80

        counts.uInfAddr = SerchLog(pA->recv.format.data[0],&uLogDataFlash);

        pA->send.format.maddrL = 0x01;  // ��Ϣ���ַ
        pA->send.format.maddrM = 0x00;
        pA->send.format.maddrH = 0x00;
        pA->send.format.data[0] = counts.uInfTemp[0];  // ������ܹ���Ҫ���͵���־������
        pA->send.format.data[1] = counts.uInfTemp[1];
        pA->send.format.data[2] = 0x00;
        IecCreateFrameI(P_LOG,0x01,R_TRANS_START_ACK,3,&pA->send);
        break;

    case R_DATA_TRANS:  //���ݴ���0x82
        if(Iaddr.uInfAddr>=counts.uInfAddr)//�����͵�����֡�����ܰ��������ɷ���0x86 ֹͣ����
        {

            pA->send.format.maddrL = Iaddr.uInfTemp[0];  // ��Ϣ���ַ���ܰ���
            pA->send.format.maddrM = Iaddr.uInfTemp[1];
            pA->send.format.maddrH = 0x00;
            pA->send.format.data[0] = 0x86;
            IecCreateFrameI(P_LOG,0x01,R_TRANS_FINISH,1,&pA->send); //�������

            memset(uMenLogData,0,LOGFRAMELENTH);
            uLogDataFlash= 0;
            Iaddr.uInfAddr = 0;
            counts.uInfAddr = 0;
            break;
        }
        pA->send.format.maddrL = Iaddr.uInfTemp[0]; // ��Ϣ���ַ��λ
        pA->send.format.maddrM = Iaddr.uInfTemp[1];
        pA->send.format.maddrH = 0x00;

        DataFlash_Read(uLogDataFlash+LOGFRAMELENTH*Iaddr.uInfAddr,&uMenLogData[0],LOGFRAMELENTH);
        for(int i=0; i<LOGFRAMELENTH; i++)
        {
            pA->send.format.data[i] = uMenLogData[i];  // ��־����
        }
        pA->send.format.data[200] = 0x55;   //CRCУ���λ
        pA->send.format.data[201] = 0xAA;   //CRCУ���λ
        IecCreateFrameI(P_LOG,0x01,R_DATA_TRANS,LOGFRAMELENTH+2,&pA->send);

        Iaddr.uInfAddr++;

        break;

//        ƽ̨����У��CRC
//        }
//        else if (0x02==pA->recv.format.data[0])
//        {
//            for(int i=0; i<LOGFRAMELENTH; i++)
//            {
//                pA->send.format.data[i] = uMenLogData[i];  // ��־����
//            }
//            pA->send.format.data[200] = 0xAA;   //CRCУ���λ
//            pA->send.format.data[201] = 0x55;   //CRCУ���λ
//            IecCreateFrameI(P_LOG,0x01,R_DATA_TRANS,LOGFRAMELENTH+2,pA);
//        CRCУ��ʧ��ʱ����Ҫֱ�����·���

    case R_DATA_RETRY:  //�����ش�0x83

        if(Iaddr.uInfAddr>=counts.uInfAddr)
        {
            return 0;
        }
        Iaddr.uInfTemp[0] = pA->recv.format.data[0];     //�ش�����
        Iaddr.uInfTemp[1] = pA->recv.format.data[1];
        DataFlash_Read(uLogDataFlash+LOGFRAMELENTH*Iaddr.uInfAddr,&uMenLogData[0],LOGFRAMELENTH);

        pA->send.format.maddrL = Iaddr.uInfTemp[0];     // ��Ϣ���ַ��λ
        pA->send.format.maddrM = Iaddr.uInfTemp[1];
        pA->send.format.maddrH = 0x00;
        for(int i=0; i<LOGFRAMELENTH; i++)
        {
            pA->send.format.data[i] = uMenLogData[i];  // ��־����
        }
        pA->send.format.data[200] = 0x55;   //CRCУ���λ
        pA->send.format.data[201] = 0xAA;   //CRCУ���λ
        IecCreateFrameI(P_LOG,0x01,R_DATA_TRANS,LOGFRAMELENTH+2,&pA->send);

        Iaddr.uInfAddr++;
        break;

    case R_TRANS_STOP:

        Iaddr.uInfAddr = Iaddr.uInfAddr -1;
        pA->send.format.maddrL = 0x00;  // ��Ϣ���ַ
        pA->send.format.maddrM = 0x00;
        pA->send.format.maddrH = 0x00;
        pA->send.format.data[0] = 0x85;
        IecCreateFrameI(P_LOG,0x01,R_TRANS_STOP_ACK,1,&pA->send);   //ֹͣ����ȷ��

        pA->send.format.maddrL = Iaddr.uInfTemp[0];  // ��Ϣ���ַ���ܰ���
        pA->send.format.maddrM = Iaddr.uInfTemp[1];
        pA->send.format.maddrH = 0x00;
        pA->send.format.data[0] = 0x86;
        IecCreateFrameI(P_LOG,0x01,R_TRANS_FINISH,1,&pA->send); //�������

        memset(uMenLogData,0,LOGFRAMELENTH);
        uLogDataFlash= 0;
        Iaddr.uInfAddr = 0;
        counts.uInfAddr = 0;

        break;

    case R_TRANS_FINISH_ACK:

        break;
    }
    
    return 0;
}
/******************************************************************************
* ��    �ƣ�Iec104HandleEsn()
* ��    �ܣ�����ESN����
* ��ڲ�����
*           *pA         IEC��Ϣָ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104HandleEsn(IEC104_MAIN_T *pA)
{
	if(R_SETTING==pA->recv.format.reasonL) // ����92
    {
        memcpy(pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));

        memcpy(g_LoggerInfo.esn,pA->recv.format.data,20);
        DEBUGOUT("��Esn\n");
        SaveEepData(EEP_LOGGER_INF);
    }
	else if(R_INQUIRE==pA->recv.format.reasonL) // ��ѯ90
    {
        memcpy(pA->send.format.data,g_LoggerInfo.esn,20);
    }

    IecCreateFrameI(P_CL_ESN,pA->recv.format.limit,pA->recv.format.reasonL+1,(pA->recv.format.len-13),&pA->send);
}
/******************************************************************************
* ��    �ƣ�Iec104HandleLoggerType()
* ��    �ܣ���������TYPE��
* ��ڲ�����
*           *pA         IEC��Ϣָ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104HandleLoggerType(IEC104_MAIN_T *pA)
{
    if(R_SETTING==pA->recv.format.reasonL) // ����92
    {
        memcpy( pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));

        memcpy(g_LoggerInfo.type,pA->recv.format.data,20);
        DEBUGOUT("������\n");

        SaveEepData(EEP_LOGGER_INF);
    }
    else if(R_INQUIRE==pA->recv.format.reasonL) // ��ѯ90
    {
        memcpy(pA->send.format.data,g_LoggerInfo.type,20);
    }

    IecCreateFrameI(P_CL_TYPE,pA->recv.format.limit,pA->recv.format.reasonL+1,(pA->recv.format.len-13),&pA->send);
}
/******************************************************************************
* ��    �ƣ�Iec104HandleLoggerModel()
* ��    �ܣ���������MODEL��
* ��ڲ�����
*           *pA         IEC��Ϣָ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104HandleLoggerModel(IEC104_MAIN_T *pA)
{
    if(R_SETTING==pA->recv.format.reasonL) // ����92
    {
        memcpy( pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));

        memcpy(g_LoggerInfo.model,pA->recv.format.data,20);
        DEBUGOUT("���ͺ�\n");
        SaveEepData(EEP_LOGGER_INF);
    }
    else if(R_INQUIRE==pA->recv.format.reasonL) // ��ѯ90
    {
        memcpy(pA->send.format.data,g_LoggerInfo.model,20);
    }

    IecCreateFrameI(P_CL_MODEL,pA->recv.format.limit,pA->recv.format.reasonL+1,(pA->recv.format.len-13),&pA->send);
}
/******************************************************************************
* ��    �ƣ�Iec104HandleLoggerName()
* ��    �ܣ���������TYPE��
* ��ڲ�����
*           *pA         IEC��Ϣָ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104HandleLoggerName(IEC104_MAIN_T *pA)
{
    if(R_SETTING==pA->recv.format.reasonL) // ����92
    {
        memcpy( pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));

        memcpy(g_LoggerInfo.name,pA->recv.format.data,20);
        DEBUGOUT("������\n");
        SaveEepData(EEP_LOGGER_INF);
    }
    else if(R_INQUIRE==pA->recv.format.reasonL) // ��ѯ90
    {
        memcpy(pA->send.format.data,g_LoggerInfo.name,20);
    }

    IecCreateFrameI(P_CL_NAME,pA->recv.format.limit,pA->recv.format.reasonL+1,(pA->recv.format.len-13),&pA->send);
}
/******************************************************************************
* ��    �ƣ�Iec104HandleHwEsnRenovate()
* ��    �ܣ������豸esn��ѯ(���޻�Ϊ�豸��D4
* ��ڲ�����
*           *pA         IEC��Ϣָ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104HandleHwEsnRenovate(IEC104_MAIN_T *pA)
{
	uint8_t uMbVirAddr;   // �豸��ַ:�޶�����0-9ʮ�������
	uint8_t uMbRelAddr=0; // ����MbVirAddrӳ������豸ʵ�ʵ�ַ

    if(0x93==pA->recv.format.reasonL)
    {
        uMbVirAddr = pA->recv.format.maddrL;
        uMbRelAddr = IecAddrConvert(uMbVirAddr);

        if(uMbRelAddr<MAX_device)
        {
            memcpy(g_DeviceEsn.cDeviceEsn[uMbRelAddr],pA->recv.format.data,20);
            SaveEepData(EEP_DEVICE_ESN);
        }
    }
}
/******************************************************************************
* ��    �ƣ�Iec104HandleHwSoftRenovate()
* ��    �ܣ������豸����汾n��ѯ(���޻�Ϊ�豸��
* ��ڲ�����
*           *pA         IEC��Ϣָ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104HandleHwSoftRenovate(IEC104_MAIN_T *pA)
{
    uint8_t i,k;

    IEC104_MAIN_T *pstIec104=pA;
    uint8_t uMbVirAddr;   // �豸��ַ:�޶�����0-9ʮ�������
    uint8_t uMbRelAddr=0; // ����MbVirAddrӳ������豸ʵ�ʵ�ַ


    uMbVirAddr = pstIec104->recv.format.data[0];
    uMbRelAddr = IecAddrConvert(uMbVirAddr);

    if(uMbRelAddr<MAX_device)
    {
        k = pstIec104->recv.format.len -13;
        for(i=0;i<k;i++)
        {
            if('2'==pstIec104->recv.format.data[i])
            {
                i++;
                if('='==pstIec104->recv.format.data[i])
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
        /*
        if(0==g_uNextFrame)  // �ϱ���һ֡��Ϊ�豸����汾��Ϣ��δ�ϱ�C4 92��
        {
            memcpy(Device_soft.device_soft[uMbRelAddr],&pstIec104->recv.format.data[i],17);
        }*/
        DEBUGOUT("\n�豸%d����汾��Ϊ: %0.17s\n",uMbVirAddr,g_DeviceSoft.cDeviceSoft[uMbRelAddr]);
        SaveEepData(EEP_DEVICE_SOFT);

    }
}
/******************************************************************************
* ��    �ƣ�Iec104HandleDelHwDev()
* ��    �ܣ������豸ɾ�������޻�Ϊ�豸��D5
* ��ڲ�����
*           *pA         IEC��Ϣָ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104HandleDelHwDev(IEC104_MAIN_T *pA)
{
	//uint8_t uMbVirAddr;//�豸��ַ:�޶�����0-9ʮ�������
	uint8_t uMbRelAddr=0;//����MbVirAddrӳ������豸ʵ�ʵ�ַ
	uint8_t uDelCmd;

	//memcpy( pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));

	IecCpoyMesAddr(pA);

	pA->send.format.data[0] = pA->recv.format.data[0];

	uMbRelAddr=IecAddrConvert(pA->recv.format.maddrL);

	uDelCmd=pA->recv.format.data[0];

    if(R_ACTIVE==pA->recv.format.reasonL && 0x81==uDelCmd)
    {
        //Iec104PreDelHwDev();
    }
    else if(R_ACTIVE==pA->recv.format.reasonL && 0x01==uDelCmd)
    {
        if(uMbRelAddr<MAX_device)
        {
            g_DeviceSouth.device_inf[uMbRelAddr].addr=0;
			g_DeviceEsn.uEsnMark[pA->recv.format.maddrL]=0;
			DEBUGOUT("addr:%d uEsnMark:%d",pA->recv.format.maddrL,g_DeviceEsn.uEsnMark[g_DeviceSouth.device_inf[uMbRelAddr].addr]);
            if(g_DeviceSouth.device_sum>=1)
            {
                g_DeviceSouth.device_sum--;
            }
            g_LoggerRun.err_lost &= ~(1<<uMbRelAddr);
            g_LoggerAlarm.dev_lost &= ~(1<<uMbRelAddr);

            SaveEepData(EEP_DEVICE_SOUTH);
			SaveEepData(EEP_DEVICE_ESN);  //Ϊ�˱�������ַ���
            SaveEepData(EEP_ALARM);  // ����澯ֵ
        }
    }

    IecCreateFrameI(P_HW_DEL,pA->recv.format.limit,pA->recv.format.reasonL+1,1,&pA->send);
}
/******************************************************************************
* ��    �ƣ�Iec104HandleReset()
* ��    �ܣ��ָ���������
* ��ڲ�����
*           *pA         IEC��Ϣָ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104HandleReset(IEC104_MAIN_T *pA)
{
	uint8_t uRstCmd;

	uRstCmd=pA->recv.format.data[0];
	//memcpy( pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));

    IecCpoyMesAddr(pA);

	pA->send.format.data[0] = pA->recv.format.data[0];

    if(R_ACTIVE==pA->recv.format.reasonL && 0x81==uRstCmd)
    {
        //�����Ԥ�ò���
        //Iec104PreSet();
        IecCreateFrameI(P_CL_RESET,pA->recv.format.limit,pA->recv.format.reasonL+1,1,&pA->send);
    }
    else if(R_ACTIVE==pA->recv.format.reasonL && 0x01==uRstCmd)
    {
        AllReset(1);
        IecCreateFrameI(P_CL_RESET,pA->recv.format.limit,pA->recv.format.reasonL+1,1,&pA->send);

        sleep(5);
        Reboot();
    }
}
/******************************************************************************
* ��    �ƣ�Iec104HandleReboot()
* ��    �ܣ����ɸ�λ
* ��ڲ�����
*           *pA         IEC��Ϣָ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104HandleReboot(IEC104_MAIN_T *pA)
{
	uint8_t uRbtCmd;

	uRbtCmd=pA->recv.format.data[0];
	//memcpy( pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-13));

	IecCpoyMesAddr(pA);
	pA->send.format.data[0] = pA->recv.format.data[0];

    if(R_ACTIVE==pA->recv.format.reasonL && 0x81==uRbtCmd)
    {
        //�����Ԥ�ò���
        IecCreateFrameI(P_CL_REBOOT,pA->recv.format.limit,pA->recv.format.reasonL+1,1,&pA->send);
    }
    else if(R_ACTIVE==pA->recv.format.reasonL && 0x01==uRbtCmd)
    {
        IecCreateFrameI(P_CL_REBOOT,pA->recv.format.limit,pA->recv.format.reasonL+1,1,&pA->send);
        sleep(5);
        Reboot();
    }
}
/******************************************************************************
* ��    �ƣ�Iec104TimeCollectPre()
* ��    �ܣ�����Ԥ����
* ��ڲ�����
*           *pA         IEC��Ϣָ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104TimeCollectPre(IEC104_MAIN_T *pA)
{
    if(R_ACTIVE==pA->recv.format.reasonL) // ����06
    {
        // ���ԣ��ֽڽ�������
        //if(SUBCOLLECT_NULL == iec_subcollect_run.subcollect)
        {
            //���յ���ʱ�걣����������ʱ�����ٱ��Ĵ���
            g_TimeMarker[0] = pA->recv.format.data[1];
            g_TimeMarker[1] = pA->recv.format.data[2];
            g_TimeMarker[2] = pA->recv.format.data[3];
            g_TimeMarker[3] = pA->recv.format.data[4];
            g_TimeMarker[4] = pA->recv.format.data[5];
            g_TimeMarker[5] = pA->recv.format.data[6];
            g_TimeMarker[6] = pA->recv.format.data[7];

            g_sIecRun.subcollect = SUBCOLLECT_START;
        }

    }
    else if(R_RECOLLECT_END==pA->recv.format.reasonL) // 0x8F   // ���ɽ���
    {
        if(0==pA->recv.format.data[0]) // ��ѯ�Ƿ������ݴ���
        {
            // ����ֱ�ӷ���������
            IecCpoyMesAddr(pA);

            pA->send.format.data[0] = 0x01;
            IecCreateFrameI(P_TIME_COLLECT,pA->recv.format.limit,R_RECOLLECT_END,1,&pA->send);
        }
        else if(0x8F==pA->recv.format.data[0]) // ���ɽ���
        {
            IecProcessFrameS(pA,1);// ����S֡ȷ��
        }
    }
    else if(R_ACTIVE_END==pA->recv.format.reasonL) // 0x0A   //�������
    {
        IecProcessFrameS(pA,1);// ����S֡ȷ��
    }
}
/******************************************************************************
* ��    �ƣ�Iec104Yk()
* ��    �ܣ�ң�ء�2E
* ��ڲ�����
            *pA         IEC��Ϣָ��
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104Yk(IEC104_MAIN_T *pA)
{
    Uint_Char_Convert temp;
    if(R_ACTIVE==pA->recv.format.reasonL) // ����06
    {
        temp.c[0] = pA->recv.format.maddrL;
        temp.c[1] = pA->recv.format.maddrM;

        temp.u = temp.u - 0x6001;  // ��������ң�ص�ַ

        if(temp.u >= g_DeviceSouth.yk_sum)
        {
            DEBUGOUT("�Ƿ�ң�ص�ַ\n");
        }
//        if(g_uRecYk&0x01)
//        {
//            DEBUGOUT("�Ѵ���δ��ɵ�ң��ָ��!\n");
//            return;
//        }
        if(pA->recv.format.data[0]&0x80)
        {
            IEC104_DATA_YK[temp.u] = pA->recv.format.data[0];  // ң�ص�ֵ
        }

        if(0==(pA->recv.format.data[0]&0x80) && IEC104_DATA_YK[temp.u]&0x80)  // ң��ȷ��
        {
            //g_uRecYk |= 0x01;  // ������յ�ң����Ϣ
            OSQPost(MesQ, &s_uSouthYk);
        }

        IecCpoyMesAddr(pA);

        pA->send.format.data[0] = pA->recv.format.data[0];
        IecCreateFrameI(C_DC_NA_1,pA->recv.format.limit,R_ACTIVE_ACK,1,&pA->send);
    }
    else if(R_ACTIVE_STOP==pA->recv.format.reasonL) // ֹͣ����08
    {
        temp.c[0] = pA->recv.format.maddrL;
        temp.c[1] = pA->recv.format.maddrM;

        temp.u = temp.u - 0x6001;  // ��������ң�ص�ַ
        IEC104_DATA_YK[temp.u] = 0x00;  // ң�ص�ֵ

        IecCpoyMesAddr(pA);

        pA->send.format.data[0] = pA->recv.format.data[0];
        IecCreateFrameI(C_DC_NA_1,pA->recv.format.limit,R_ACTIVE_STOP_ACK,1,&pA->send);
    }
}
/******************************************************************************
* ��    �ƣ�Iec104SD()
* ��    �ܣ���㡣D9
* ��ڲ�����
            *pA         IEC��Ϣָ��
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void Iec104SD(IEC104_MAIN_T *pA)
{
    Uint_Char_Convert temp,temp_104;
	U32_F_Char_Convert  ctemp;
	uint8_t SdSum;

	if(R_SETTING==pA->recv.format.reasonL) // ����06
	{
		SdSum = pA->recv.format.limit & 0x7f;
		SdContinuousAddr=0;    //ң���ķ�������ַ
		temp.c[0] = pA->recv.format.maddrL;
		temp.c[1] = pA->recv.format.maddrM;

		temp_104.u= temp.u;

		if(temp.u > 0x6400)
		{
			return;
		}
		temp.u = temp.u - 0x6201;  // ��������ң�ص�ַ

		if(temp.u >= g_DeviceSouth.sd_sum)
		{
			DEBUGOUT("�Ƿ�ң����ַ\n");
		}
		memcpy(ctemp.c,pA->recv.format.data,sizeof(ctemp.c));
		ctemp.u=ctemp.f;
		IEC104_DATA_SD[0] = SdSum;  // ң������
		IEC104_DATA_SD[1] = temp_104.u;  // ң����ַ
		IEC104_DATA_SD[2] = ctemp.u;  	// ң����ֵ

		for(uint8_t i=1;i< SdSum;i++)
		{
			temp.c[0] = pA->recv.format.data[i*7-3];
			temp.c[1] = pA->recv.format.data[i*7-2];

			temp_104.u= temp.u;

			if(temp.u > 0x6400)
			{
				return;
			}
			temp.u = temp.u - 0x6201;  // ��������ң����ַ
			memcpy(ctemp.c,&pA->recv.format.data[i*7],sizeof(ctemp.c));
			ctemp.u=ctemp.f;
			
			IEC104_DATA_SD[i*2+1] = temp_104.u;  	// ң����ַ
			IEC104_DATA_SD[(i+1)*2] = ctemp.u;  	// ң����ֵ
		}
		OSQPost(MesQ, &s_uSouthSd);

		sleep(SdSum/2);
		IecCpoyMesAddr(pA);
		memcpy(pA->send.format.data,pA->recv.format.data,(pA->recv.format.len-10));// I֡����
		IecCreateFrameI(p_SD,pA->recv.format.limit,R_SET_SUC,7*SdSum-3,&pA->send);
	}
    else if(R_INQUIRE==pA->recv.format.reasonL) // ֹͣ����08
   {
		if(pA->recv.format.limit & 0x80)	 //ң����������ַ
		{
			 SdSum = pA->recv.format.limit - 0x80;

			 SdContinuousAddr=1;    //ң����������ַ 
			 temp.c[0] = pA->recv.format.maddrL;
			 temp.c[1] = pA->recv.format.maddrM;
             if(temp.u > 0x6400 || temp.u < 0x6201)
             {
            	 DEBUGOUT("�Ƿ�ң����ַ\n");
                 return;
             }
			 IEC104_DATA_SD[0] = SdSum;		// ��ѯ��ң�������
			 IEC104_DATA_SD[1] = temp.u;	// ��ѯ�ĵڼ���ң����
			 OSQPost(MesQ, &s_uSouthReadSd);
			 sleep(5);
			 for(uint8_t i=0;i<SdSum;i++)
			 {
				ctemp.u=IEC104_DATA_SD[i];
				ctemp.f=ctemp.u;
				pA->send.format.data[0+i*4]=ctemp.c[0];
				pA->send.format.data[1+i*4]=ctemp.c[1];
				pA->send.format.data[2+i*4]=ctemp.c[2];
				pA->send.format.data[3+i*4]=ctemp.c[3];
				IEC104_DATA_SD[i]=0;
			 }
			IecCpoyMesAddr(pA);
			IecCreateFrameI(p_SD,pA->recv.format.limit,R_INQUIRE_SUC,4*SdSum,&pA->send);
		} 
		DEBUGOUT("YT ACQUIRY TASK END!!! \r\n");
    }
}
/******************************************************************************
* ��    �ƣ�IecProcessFrameI()
* ��    �ܣ�IEC104 I֡����
* ��ڲ�����
*           *pA         IEC��Ϣָ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
static void IecProcessFrameI(IEC104_MAIN_T *pA)
{
    switch(pA->recv.format.type)  				// ���ͱ�ʶ
    {
    case C_IC_NA_1:								// ���ٻ�����
        if(R_ACTIVE==pA->recv.format.reasonL) 	// ���ټ���
        {
            IecProcessFrameS(pA,1);    			// ����S֡
            g_sIecRun.collect = COLLECT_START; 	// ��ʼ����
            DEBUGOUT("��������\n");
        }
        break;

    case C_CI_NA_1:   							// ������ٻ�����
        g_sIecRun.dd_collect = COLLECT_DD_START;// ��ʼ����
        break;

    case M_SP_TB_1:     						//0x1E  //30   // ��ʱ��CP56Time2a�ĵ�����Ϣ
        break;

    case M_DP_TB_1:     						//0x1F  //31   // ��ʱ��CP56Time2a��˫����Ϣ
        break;

    case M_ME_TF_1:     						//0x24  //36   // ��ʱ��CP56Time2a�Ĳ���ֵ���̸�����
        break;

    case C_CS_NA_1:   							// ʱ��ͬ������67
        Iec104SyncTime(pA);
        break;

    case C_SC_NA_1:   							// ������Ϣ0x2D
        break;

    case C_DC_NA_1:   							// ˫����Ϣ0x2E
        Iec104Yk(pA);
        break;
	case p_SD:       							// ƽ̨ң�����0xD9
        Iec104SD(pA);
        break;

    case P_UPDATA:   							// ����B4
        Iec104LoggerUpdate(pA);
        break;
    case P_FILE_INOUT:
		Iec104DT1000Update(pA); 				//�������F0
		break;
    case P_SET_IP:   							//�豸IP����B5
        Iec104IP(pA);
        break;

    case P_MODBUS_ENDIAN:						// MODBUS��С������B6
        Iec104HandleModBusEndian(pA);
      	break;

    case P_MODBUS_BAUD:							// MODBUS����������B7
        Iec104HandleModBusBd(pA);
		 break;

    case P_ADDR:								// �豸������ַ����B8
        Iec104PublicAddr(pA);
        break;

    case P_VERSION:								// �汾��ѯB9
        Iec104InquireVer(pA);
        break;

    case P_ROLLBAOCK:							// �汾�ع�BA
        Iec104LoggerRollBack(pA);
        break;

    case P_INPUT_TABLE:							// �����BB
        Iec104InputTable(pA);
        break;

    case P_REPORT_NUM:							// �ϱ���������BC
        Iec104PhoneNum(pA);
        break;

    case P_INIT_FINISH:							// �豸��ʼ���Ƿ����BD

        break;

    case P_TIME_COLLECT:						// ��ʱ�����ٻ�BE  ����
        Iec104TimeCollectPre(pA);
        DEBUGOUT("���𲹲�\n");
        break;

    case P_ERR_PROCESS:							// �쳣����BF
		if(R_ACTIVE_ACK==pA->recv.format.reasonL)	//�쳣����
	   {
		   AlarmAck(pA->recv.format.len-13,pA->recv.format.data);
	   }
        break;

    case P_DEV_INFO:							// �豸������ϢC0
        IecReportLogInfo(R_INQUIRE_SUC);  		// �ϱ�������Ϣ
        break;

    case P_STATION_INFO:						// վ�������ϢC1
        break;

    case P_SERVICE_INFO:						// ƽ̨������ϢC2
        break;

    case P_CREATE_STATION:						// ��վָ��C3
        break;

    case P_SOUTH_INFO:							// �����豸��ϢC4
        Iec104SouthDevInfo(pA);
        break;

    case P_TABLE:								// ����ָ��C5
        Iec104TableState(pA);
        break;

    case P_COM_MODE:							// ͨ�ŷ�ʽC6
        Iec104CommMode(pA);
        break;

    case P_HW_INFO:								// ��Ϊ�豸��ϢC7
        Iec104DevReportAck(pA);
        break;

    case P_SERVICE_IP:							// ������IP/����C8
        Iec104ServiceIP(pA);
        break;

    case P_SERVICE_PORT:						// �������˿�C9

        break;

    case P_LOG:									// ��־����CA
        Iec104LogExport(pA);
        break;

    case P_LOCATION:							// ��γ��CB
        break;

    case P_SOUTH_STATUS:						// �����豸״̬CC
        break;

    case P_CL_ESN:								// ����ESN����D0
        Iec104HandleEsn(pA);
        break;

    case P_CL_TYPE:								// ������������D1
        Iec104HandleLoggerType(pA);
        break;

    case P_CL_MODEL:							// �����ͺ�����D2
        Iec104HandleLoggerModel(pA);
        break;

    case P_CL_NAME:								// �����豸��������D3
        Iec104HandleLoggerName(pA);
        break;

    case P_HW_ESN:								// �����豸esn��ѯ(���޻�Ϊ�豸��D4
        Iec104HandleHwEsnRenovate(pA);
        break;

    case P_HW_DEL:								// �����豸ɾ�������޻�Ϊ�豸��D5
        Iec104HandleDelHwDev(pA);
        break;

    case P_SIM_ID: 								// SIM��ID�ϱ�
        break;

    case P_MAX_DEVICE:

        break;

    case P_CL_RESET:							// �ָ���������FA
        Iec104HandleReset(pA);
        break;

    case P_CL_REBOOT:							// ��λFB
        Iec104HandleReboot(pA);
        break;

    default:
        break;
    }
}

//===================================================================================
OS_STK TaskIec104ProcessStk[TaskIec104ProcessStkSize]@0x20000400;	      // ���������ջ��С
/****************************************************************************
* ��    �ƣ�TaskSouthInquire()
* ��    �ܣ������ѯ����
* ��ڲ�����

* ���ڲ�������
* ��    ��: ��
****************************************************************************/
void TaskIec104Process(void *p)
{
    p = p;
    uint8_t uDataLen;

    IecInit();

    while(1)
    {
        if(g_sIecRun.collect || g_sIecRun.dd_collect)
        {
            IecCollectProcess(&g_sIEC,&g_sIecRun);  // ����
        }
        else if(g_sIecRun.subcollect)
        {
            IecSubSCollect(&g_sIEC,&g_sIecRun);   // ����        RECORD END ADDR:
        }

        if(g_LoggerRun.update&0xF0)  // Զ��������ʱ����
        {
            g_uNowTime = OSTimeGet();

            if(TIMEOUT==TimeOut(g_uUpdataTime,g_uNowTime,90000)) // ������ʱ����90��
            {
                DEBUGOUT("������ʱ,��λ\n");
                Reboot();
            }
        }

        uDataLen = CmRecLinkLen();

        if(0==uDataLen)  //�����ݾ��˳�
        {
            msleep(20);
            continue;
        }

        memset(g_sIEC.recv.buff,0,sizeof(g_sIEC.recv.buff));

        uDataLen = CmReadRecLink(g_sIEC.recv.buff);    //������

        if(IEC104_HEAD == g_sIEC.recv.format.start)     //��ͷ�ж��Լ�֡���ദ��
        {
            if(uDataLen != (g_sIEC.recv.format.len+2))
            {
                DEBUGOUT("\n104���Ȳ���\n");
                return;
            }
            g_sIEC.FrameCommand = g_sIEC.recv.format.SseqL & 0x03;

            if(1==g_sIEC.FrameCommand)   // �յ�S֡
            {
                IecProcessFrameS(&g_sIEC,0);
            }
            else if(3==g_sIEC.FrameCommand) // �յ�U֡
            {
                IecProcessFrameU(&g_sIEC,g_sIEC.recv.format.SseqL & 0xfc,0);
            }
            else // �յ�I֡
            {
                g_IecRecSeq++;  // ��������+1
                IecProcessFrameI(&g_sIEC);
            }
        }
    }
}
/******************************************************************************
* ��    �ƣ�IecCollectProcess()
* ��    �ܣ�IEC104 ���ٴ���
* ��ڲ�����
            bytes     ���ݳ���
            *pA       IEC��Ϣָ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
void IecCollectProcess(IEC104_MAIN_T *pA,IEC_RUNNING_T *call)
{
//	static uint16_t s_uIecHead,uPointCount = 0; // Ҫ�������ݵ�ͷ
//	static uint8_t Record_RelAddr;
	static uint16_t s_uIecHead; // Ҫ�������ݵ�ͷ
    uint16_t uIecCount;         // ��Ϣ������
    uint16_t uCollectSum;       //
    uint8_t qty=0;              // һ֡IEC I֡�������ٵ�������
    uint8_t temp;
    uint8_t uRelTable;          // ��Ե���

    Uint_Char_Convert  addr;
    U32_F_Char_Convert yctemp;


    if(NORTH_OK!=g_LoggerRun.north_status)  // ���ӷ�����δ���
    {
        call->collect = COLLECT_NULL;
        call->dd_collect = COLLECT_NULL;
        return;
    }

    if(g_LoggerRun.update)  // ��������
    {
        if(COLLECT_NULL!=call->collect)
        {
            call->collect = COLLECT_END;      // // ���ٽ���
        }

        if(COLLECT_NULL!=call->dd_collect)
        {
            call->dd_collect = COLLECT_DD_END;   // // ���ٽ���
        }
    }

    switch(call->collect)
    {
    case COLLECT_START:  // ��ʼ����
        //Iec104ProcessFrameS(pA,1);    // ����S֡
        pA->send.format.maddrL = 0x00;// ��Ϣ���ַ
        pA->send.format.maddrM = 0x00;
        pA->send.format.maddrH = 0x00;
        pA->send.format.data[0] = 0x14;  // ������������
        IecCreateFrameI(C_IC_NA_1,0x01,R_ACTIVE_ACK,1,&pA->send);

        if(RUNNING_WORK_READ!=g_LoggerRun.run_status|| -1==CheckDevState())  // ���ɣ�û�п�վ����ͣ�����������������豸��ַ��Ϊ0
        {
            call->collect = COLLECT_END;  // ���ٽ���
            break;
        }

        s_uIecHead = 0;  // ң����ʼ��ַΪ0x0001����Դ洢���ݵ������±�0��ʼ
        call->collect = COLLECT_YX;  // �´η���ң������
        call->uRelAddr = 0;
        break;

    case COLLECT_YX:     // ����ң������
        // һ֡255�ֽڣ�������Ϣռ��12�ֽڣ�����ռ��4���ֽڣ�һ���ɴ���

        msleep(500);  // ��ʱ500ms

        if(NULL==IEC104_DATA_YX)
        {
            DEBUGOUT("��ң�ſռ�\n");
            if(g_DeviceSouth.yx_sum)  // ���ң�ŵ�����Ϊ0���򲻱���Ϣ
            {
                DEBUGOUT("��ң�ŵ�\n");
            }

            call->collect = COLLECT_YC;  // �´η���ң��
        }

        //----------------------------------------------------------------
        // һ֡255�ֽڣ�������Ϣռ��12�ֽڣ���ʼ��Ϣ���ַռ��3�ֽڣ�����ռ��5�ֽڣ�һ���ɴ���48����
        if(!s_uIecHead)
        {
            for(temp=call->uRelAddr;temp<g_DeviceSouth.device_sum;temp++)
            {
                 // ���û��ң�ŵ�ʱ��ң����ʼ��ַΪ0
                if(g_DeviceSouth.device_inf[call->uRelAddr].addr && g_DeviceSouth.device_inf[call->uRelAddr].yx_start_addr)
                {
                    s_uIecHead = g_DeviceSouth.device_inf[call->uRelAddr].yx_start_addr - 0x01;
                    break;
                }
                else
                {
                    call->uRelAddr++;
                }
            }

            if(temp>=g_DeviceSouth.device_sum || temp>=MAX_device)
            {
                s_uIecHead = 0;  // ң����ʼ��ַΪ0x0001����Դ洢���ݵ������±�0��ʼ
                call->uRelAddr = 0;
                call->collect = COLLECT_YC;  // �´η���ң������
                break;
            }
        }

        uRelTable = g_DeviceSouth.device_inf[call->uRelAddr].rel_num; // ��Ե���

        //addr.u = s_uIecHead+0x01;  // ��Ϣ���ַ
        //pA->send.format.maddrL = addr.c[0];// ��Ϣ���ַ
        //pA->send.format.maddrM = addr.c[1];
        //pA->send.format.maddrH = 0x00;

        uCollectSum = g_DeviceSouth.device_inf[call->uRelAddr].yx_start_addr - 0x01 + g_sIecPointCount[uRelTable].uYxSum;

        //for(uIecCount=s_uIecHead,qty=0; uIecCount<g_DeviceSouth.yx_sum && qty<60; uIecCount++)
        for(uIecCount=s_uIecHead,qty=0; uIecCount<uCollectSum && qty<60; uIecCount++)
        {
            temp = qty*4;
            addr.u = s_uIecHead + 0x01;          // ��Ϣ���ַ
            pA->send.buff[temp+12] = addr.c[0]; // iec.ibuff[temp]   = addr.c[0]; // uIecCount&0xff;��Ϣ���ַ��λ
            pA->send.buff[temp+13] = addr.c[1]; // iec.ibuff[temp+1] = addr.c[1]; // uIecCount>>8;  ��Ϣ���ַ��λ
            pA->send.buff[temp+14] = 0x00;      // iec.ibuff[temp+2] = 0x00;      // ��Ϣ���ַ����Чλ
            pA->send.buff[temp+15] = IEC104_DATA_YX[uIecCount];//iec.ibuff[temp+3] = IEC104_DATA_YX[uIecCount];

            qty++;
            s_uIecHead++;
        }

        if(uIecCount >= uCollectSum)  // һ̨ң�����ٷ������
        {
            if(call->uRelAddr < (g_DeviceSouth.device_sum-1) && call->uRelAddr < (MAX_device-1))
            {
                s_uIecHead = 0;
                call->uRelAddr++;
            }
            else
            {
                s_uIecHead = 0;
                call->uRelAddr = 0;
                call->collect = COLLECT_YC;  // �´η���ң��
            }
        }

        if(qty)
        {
            IecCreateFrameI(M_SP_NA_1,qty,R_COLLECT_ACK,qty*4-3,&pA->send);
        }

        /*if(uIecCount>=g_DeviceSouth.yx_sum)  // ң�������Ѿ��������
        {
            s_uIecHead = 0;// ң����ʼ��ַΪ0x4001����Դ洢���ݵ������±�0��ʼ
            call->collect = COLLECT_YC;  // �´η���ң������
            call->uRelAddr = 0; // ����Ե�ַΪ0���豸��ʼ��Ӧ����
        }*/
        break;

    case COLLECT_YC:     // ����ң������

        msleep(500);  // ��ʱ500ms

        if(NULL==IEC104_DATA_YC)
        {

            DEBUGOUT("��ң��ռ�\n");
			
            if(g_DeviceSouth.yc_sum)  // ���ң�������Ϊ0���򲻱���Ϣ
            {
                DEBUGOUT("��ң���\n");
            }

            call->collect = COLLECT_END;  // ���ٽ���
        }
        //----------------------------------------------------------------
        // һ֡255�ֽڣ�������Ϣռ��12�ֽڣ���ʼ��Ϣ���ַռ��3�ֽڣ�����ռ��5�ֽڣ�һ���ɴ���48����
        if(!s_uIecHead)
        {
            for(temp=call->uRelAddr;temp<g_DeviceSouth.device_sum;temp++)
            {
                if(g_DeviceSouth.device_inf[call->uRelAddr].addr && g_DeviceSouth.device_inf[call->uRelAddr].yc_start_addr>=0x4001)
                {
                    s_uIecHead = g_DeviceSouth.device_inf[call->uRelAddr].yc_start_addr - 0x4001;
                    break;
                }
                else
                {
                    call->uRelAddr++;
                }
            }
            if(temp>=g_DeviceSouth.device_sum || temp>=MAX_device)
            {
                s_uIecHead = 0;  // ң����ʼ��ַΪ0x0001����Դ洢���ݵ������±�0��ʼ
                call->uRelAddr = 0;
                call->collect = COLLECT_END;  // ���ٽ���
                break;
            }
        }

        //----------------------------------------------------------------

        uRelTable = g_DeviceSouth.device_inf[call->uRelAddr].rel_num; // ��Ե���

        addr.u = s_uIecHead + 0x4001;  // ��Ϣ���ַ

        pA->send.format.maddrL = addr.c[0];// ��Ϣ���ַ
        pA->send.format.maddrM = addr.c[1];
        pA->send.format.maddrH = 0x00;

        uCollectSum = g_DeviceSouth.device_inf[call->uRelAddr].yc_start_addr - 0x4001 + g_sIecPointCount[uRelTable].uYcSum;
//        if(Record_RelAddr != call->uRelAddr)
//		{
//			Record_RelAddr = call->uRelAddr;
//			uPointCount = 0;
//		}

        for(uIecCount=s_uIecHead,qty=0; uIecCount<uCollectSum && qty<48; uIecCount++)
        {
            temp = qty*5;

//            DEBUGOUT("*************************** ������λ�ж�ǰ **********************************\n");
            //***********************************����ֵ��λȫFֵ************            
//            if(((0x4F800000==IEC104_DATA_YC[uIecCount])&&(g_pRegPoint[uRelTable][uPointCount].reg_type.type.data == T_UINT32)) ||
//            ((0x477FFF00==IEC104_DATA_YC[uIecCount])&&(g_pRegPoint[uRelTable][uPointCount].reg_type.type.data == T_UINT16)))
//            {
//  //          DEBUGOUT("**************************** ������λ�жϺ� **************************************\n");
//                pA->send.format.data[temp]   = 0xFF;
//                pA->send.format.data[temp+1] = 0xFF;
//                pA->send.format.data[temp+2] = 0xFF;
//                pA->send.format.data[temp+3] = 0xFF;
//
//            }
//            else
            {
                yctemp.u = IEC104_DATA_YC[uIecCount];
                pA->send.format.data[temp]   = yctemp.c[0];
                pA->send.format.data[temp+1] = yctemp.c[1];
                pA->send.format.data[temp+2] = yctemp.c[2];
                pA->send.format.data[temp+3] = yctemp.c[3];
            }

            pA->send.format.data[temp+4] = 0x00; // Ʒ������

            qty++;
			s_uIecHead++;
//			uPointCount++;
//			if(uPointCount >= g_sIecPointCount[uRelTable].uYcSum)
//			{
//				uPointCount = 0;
//			}
        }

        if(uIecCount >= uCollectSum)  // һ̨ң�����ٷ������
        {
            if(call->uRelAddr < (g_DeviceSouth.device_sum-1) && call->uRelAddr < (MAX_device-1))
            {
                s_uIecHead = 0;
                call->uRelAddr++;
            }
            else
            {
                call->collect = COLLECT_END;  // ���ٽ���
            }
        }

        if(qty)  // ����Ҫ�ϱ���ң���
        {
            IecCreateFrameI(M_ME_NC_1,qty|0x80,R_COLLECT_ACK,qty*5,&pA->send); // ������Ϣ�㣬�ɱ�ṹ�޶������λ��1
        }

        /*if(uIecCount>=g_DeviceSouth.yc_sum)  // ң�������Ѿ��������
        {
            s_uIecHead = 0;
            call->collect = COLLECT_END;  // ���ٽ���
        }*/
        break;

    case COLLECT_END:    // ���ٽ���
        pA->send.format.maddrL = 0x00;// ��Ϣ���ַ
        pA->send.format.maddrM = 0x00;
        pA->send.format.maddrH = 0x00;
        pA->send.format.data[0] = 0x14;  // ������������
        IecCreateFrameI(C_IC_NA_1,0x01,R_ACTIVE_END,1,&pA->send);

        call->collect = COLLECT_NULL;  // û������
        break;

    default:
        call->collect = COLLECT_NULL;  // û������
        break;
    }
    // �������
    switch(call->dd_collect)
    {
    case COLLECT_DD_START:  // �������
        /*pA->send.format.maddrL = 0x00;// ��Ϣ���ַ
        pA->send.format.maddrM = 0x00;
        pA->send.format.maddrH = 0x00;
        pA->send.format.data[0] = 0x45;  // ������������
        IecCreateFrameI(C_CI_NA_1,0x01,R_ACTIVE_ACK,1,pA); // �������ȷ��
        break;*/

    case COLLECT_DD_END:    // ������ٽ���
        pA->send.format.maddrL = 0x00;// ��Ϣ���ַ
        pA->send.format.maddrM = 0x00;
        pA->send.format.maddrH = 0x00;
        pA->send.format.data[0] = 0x45;  // ������������
        IecCreateFrameI(C_CI_NA_1,0x01,R_ACTIVE_END,1,&pA->send);

        call->dd_collect = COLLECT_NULL;  // û������
        break;

    default:
        call->dd_collect = COLLECT_NULL;  // û������
        break;
    }
    return;
}

/******************************************************************************
* ��    �ƣ�IecSubSCollect()
* ��    �ܣ�ƽ̨���ݲ��ɡ�
* ��ڲ�����
*           *pA         IEC��Ϣָ��
*			*subcall
* ���ڲ�������
* ��    ��:
* ADD BY: chenyang
******************************************************************************/
void IecSubSCollect(IEC104_MAIN_T *pA,IEC_RUNNING_T *subcall)
{
    //static uint16_t uTimer=0;                   // ���Ͳ������ݼ��ʱ��
    static uint16_t uPerDeviceSum = 0;	         // ��ǰ�豸�ѷ��͵���Ϣ������sum++
    static uint16_t sAllSum = 0;                // �ѷ��͵�����ң��ң����Ϣ����ܺ�
    static int32_t uRecord = NO_DATA;
    static uint32_t uNeedReadFlash = 0;         // ��Ҫ��ȡ���͵�dataflash��ַ

    uint8_t temp,tmp;
    uint8_t qty=0;			                    // һ֡IEC I֡�������ɵ�������
    uint16_t uRest;                              // ÿ̨�豸ʣ��Ĵ�������Ϣ���������͸�Ϊuint16_t from B31028&B31029, ԭuint8_t̫С����Ϣ�������
    uint8_t uRelTable;		                    // ��Ե���
    uint16_t iec_count; 	                    // ��Ϣ�����
    uint16_t uCollectSum;			            // ÿ̨�豸����Ϣ������
    uint32_t uReadFlashAddr;                    // DataFlash��ַ
    //uint8_t uMemYxAddr[YXBUFFSIZE] = {0};       // ��ȡң�ŵĳ��ȣ�һ֡��ȡ58������,Ԥ��8����־λ��
    //uint8_t uMemYcAddr[YCBUFFSIZE] = {0};       // ��ȡң��ĳ��ȣ�һ֡��ȡ46*4�����ȣ�Ԥ��ǰ��8����־λ��
    Uint_Char_Convert  addr;
    U32_F_Char_Convert yctemp;

    //uint8_t err;

    if(SUBCOLLECT_NULL==subcall->subcollect || COLLECT_NULL!= subcall->collect)// û�в��ɻ��������ڽ���
    {
    	return;
    }
    if(NORTH_OK!=g_LoggerRun.north_status)  // ���ӷ�����δ���
    {
        subcall->subcollect = SUBCOLLECT_NULL;
        return;
    }

    /*����ƽ̨�·���ʱ��������Ӧ���ϱ�������*/
    if(SUBCOLLECT_START == subcall->subcollect)
    {
        uRecord = CollectData(pA,&uNeedReadFlash);
        if(NO_DATA == uRecord)
        {
            subcall->subcollect = SUBCOLLECT_END;
        }
        else
        {

            subcall->uRelAddr = 0;
            subcall->subcollect = SUBCOLLECT_YX;
        }

        pA->send.format.maddrL = pA->recv.format.maddrL;// ��Ϣ���ַ
        pA->send.format.maddrM = pA->recv.format.maddrM;
        pA->send.format.maddrH = pA->recv.format.maddrH;
        pA->send.format.data[0] = R_COLLECT_ACK;    //��Ӧ���ٻ�
        pA->send.format.data[1] = g_TimeMarker[0];
        pA->send.format.data[2] = g_TimeMarker[1];
        pA->send.format.data[3] = g_TimeMarker[2];
        pA->send.format.data[4] = g_TimeMarker[3];
        pA->send.format.data[5] = g_TimeMarker[4];
        pA->send.format.data[6] = g_TimeMarker[5];
        pA->send.format.data[7] = g_TimeMarker[6];

        IecCreateFrameI(P_TIME_COLLECT,0x01,R_ACTIVE_ACK,8,&pA->send);

        return;//��ʷ������δ���ҵ���Ҫ�ϱ���ʱ�꣬��ʱ��μ�¼�Ѷ�ʧ
    }

    switch(subcall->subcollect)
    {
    case SUBCOLLECT_YX:     // ����ң������

        if(NULL==IEC104_DATA_YX)
        {

            DEBUGOUT("��ң�ſռ�\n");
            if(g_DeviceSouth.yx_sum)  // ���ң�ŵ�����Ϊ0���򲻱���Ϣ
            {
                DEBUGOUT("��ң�ŵ�\n");
            }

            subcall->uRelAddr = 0;
            subcall->subcollect = SUBCOLLECT_YC;  // ��һ�ο�ʼң�ⲹ��

            break;
        }
        /*
         *һ֡255�ֽڣ�������Ϣռ��12�ֽڣ���ʼ��Ϣ���ַռ��3�ֽڣ�
         *��1����Ϣ��ռ��data[0]������ʼ��Ϣ���ַ3�ֽ�
         *β��Я��7���ֽڵ�ʱ�꣬����ռ��4�ֽڣ��ӵ�2���㿪ʼ���㣬���Դ浽59
         **************************************************************/
        if(!uPerDeviceSum)
        {
            for(temp=subcall->uRelAddr; temp<MAX_device; temp++)
            {
                if(g_DeviceSouth.device_inf[subcall->uRelAddr].addr)//�ж��豸�Ƿ����
                {
                    break;
                }
                else
                {
                    subcall->uRelAddr++;
                }
            }

            if(temp>=g_DeviceSouth.device_sum || temp>=MAX_device)//�����豸��Ŵ�������豸����ң�Ų������
            {
                subcall->uRelAddr = 0;
                subcall->subcollect = SUBCOLLECT_YC;  // �´η���ң������
                return;
            }
        }

        //----------------------------------------------------------------

        uRelTable = g_DeviceSouth.device_inf[subcall->uRelAddr].rel_num; // ��Ե���
        uCollectSum =g_sIecPointCount[uRelTable].uYxSum;
        uRest = g_sIecPointCount[uRelTable].uYxSum - uPerDeviceSum;
        if(uRest>0&&uRest<=59)
        {
            uReadFlashAddr = uNeedReadFlash+8+sAllSum;

            //OSMutexPend(pRecordLock,0,&err);//�����ź���

            DataFlash_Read(uReadFlashAddr,&g_uSubData[8],uRest);

            //OSMutexPost(pRecordLock);   //�ͷ��ź���

        }
        else if(uRest>59)
        {
            //ÿ̨�豸��δ���͵���Ϣ��������һ֡���ܷ��͵�59��Ϣ����
            uReadFlashAddr = uNeedReadFlash+8+sAllSum;

            //OSMutexPend(pRecordLock,0,&err);//�����ź���

            DataFlash_Read(uReadFlashAddr,&g_uSubData[8],59);

            //OSMutexPost(pRecordLock);   //�ͷ��ź���
        }
        else
        {
            subcall->uRelAddr++;
            break;  //û��ң�ŵ��ֱ����������
        }
        addr.u = sAllSum +0x01;  // ��Ϣ���ַ
        pA->send.format.maddrL = addr.c[0];// ÿ֡���ĵ�1�������Ϣ���ַ
        pA->send.format.maddrM = addr.c[1];
        pA->send.format.maddrH = 0x00;
        pA->send.format.data[0] = g_uSubData[8];
        sAllSum ++;
        qty++;
        for(iec_count=uPerDeviceSum,qty=1; iec_count<(uCollectSum-1) && qty<58; )
        {
            temp = qty*4;
            addr.u = sAllSum +1+ 0x01;  // �ӵ�2�������Ϣ���ַ��ʼ
            pA->send.format.data[temp+1] = addr.c[0]; // iec.ibuff[temp]   = addr.c[0]; // iec_count&0xff;��Ϣ���ַ��λ
            pA->send.format.data[temp+2] = addr.c[1]; // iec.ibuff[temp+1] = addr.c[1]; // iec_count>>8;  ��Ϣ���ַ��λ
            pA->send.format.data[temp+3] = 0x00;      // iec.ibuff[temp+2] = 0x00;      // ��Ϣ���ַ����Чλ
            pA->send.format.data[temp+4] = g_uSubData[qty+1+8];
            pA->send.format.data[temp+5] = g_TimeMarker[0];
            pA->send.format.data[temp+6] = g_TimeMarker[1];
            pA->send.format.data[temp+7] = g_TimeMarker[2];
            pA->send.format.data[temp+8] = g_TimeMarker[3];
            pA->send.format.data[temp+9] = g_TimeMarker[4];
            pA->send.format.data[temp+10] = g_TimeMarker[5];
            pA->send.format.data[temp+11] = g_TimeMarker[6];

            sAllSum++;
            uPerDeviceSum++;
            qty++;
        }

        if(iec_count >= (uCollectSum-1))  // һ̨ң�Ų��ɷ������
        {
            uPerDeviceSum=0;
            if(subcall->uRelAddr < (g_DeviceSouth.device_sum-1) && subcall->uRelAddr < (MAX_device-1))
            {
                subcall->uRelAddr++;
            }
            else
            {

                subcall->uRelAddr = 0;
                sAllSum = 0;
                subcall->subcollect = SUBCOLLECT_YC;  // ��һ�ο�ʼң�ⲹ��
            }
        }

        if(qty)
        {
            IecCreateFrameI(M_SP_TB_1,qty,R_COLLECT_ACK,((qty-1)*4+1+7),&pA->send);
        }
        break;

    case SUBCOLLECT_YC:     // ����ң������

        msleep(500);// ��ʱ500ms
        //----------------------------------------------------------------
        /*
         *һ֡255�ֽڣ�������Ϣռ��12�ֽڣ���ʼ��Ϣ���ַռ��3�ֽڣ�����ĩβЯ��7�ֽ�ʱ��
         *����ռ��5�ֽڣ�һ���ɴ���46����
         */
        if(!uPerDeviceSum)
        {
            for(temp=subcall->uRelAddr; temp<g_DeviceSouth.device_sum; temp++)
            {
                if(g_DeviceSouth.device_inf[subcall->uRelAddr].addr)
                {
                    break;
                }
                else
                {
                    subcall->uRelAddr++;
                }
            }
            if(temp>=g_DeviceSouth.device_sum || temp>=MAX_device)
            {
                subcall->uRelAddr = 0;
                uRecord = NO_DATA;
                subcall->subcollect = SUBCOLLECT_END;  // ���ɽ���
                break;
            }
        }

        //----------------------------------------------------------------

        uRelTable = g_DeviceSouth.device_inf[subcall->uRelAddr].rel_num; // ��Ե���

        uCollectSum =g_sIecPointCount[uRelTable].uYcSum;
        uRest = g_sIecPointCount[uRelTable].uYcSum - uPerDeviceSum;
//        DEBUGOUT("######################## YcSum : %d ##########################\n", g_sIecPointCount[uRelTable].uYcSum);
        if(uRest<=46)
        {
            uReadFlashAddr = uNeedReadFlash + 8 + g_DeviceSouth.yx_sum + sAllSum*4;

            //OSMutexPend(pRecordLock,0,&err);//�����ź���

            DataFlash_Read(uReadFlashAddr,&g_uSubData[8],uRest*4);

//            DEBUGOUT("*************************************** uSubData **************************\n");
//            for (uint16_t j = 8; j < 8+uRest*4; ++j)
//            {
//                DEBUGOUT("%x ", g_uSubData[j]);
//            }
//            DEBUGOUT("*************************************** uSubData **************************\n");

            //OSMutexPost(pRecordLock);   //�ͷ��ź���
        }
        else
        {
            //ÿ̨�豸��δ���͵���Ϣ��������һ֡���ܷ��͵�46��Ϣ����
            uReadFlashAddr = uNeedReadFlash + 8 + g_DeviceSouth.yx_sum + sAllSum*4;

            //OSMutexPend(pRecordLock,0,&err);//�����ź���

            DataFlash_Read(uReadFlashAddr,&g_uSubData[8],46*4);

//            DEBUGOUT("*************************************** out of 46 **************************\n");
//            for (uint16_t j = 8; j < 46*4; ++j)
//            {
//                DEBUGOUT("%x ", g_uSubData[j]);
//            }
//            DEBUGOUT("\n*************************************** out of 46 **************************\n");
            //OSMutexPost(pRecordLock);   //�ͷ��ź���
        }

        addr.u = sAllSum + 0x4001;  // ��Ϣ���ַ
        pA->send.format.maddrL = addr.c[0];// ��Ϣ���ַ
        pA->send.format.maddrM = addr.c[1];
        pA->send.format.maddrH = 0x00;
        for(iec_count=uPerDeviceSum,qty=0; iec_count<uCollectSum && qty<46; iec_count++)
        {
            temp = qty*5;
            tmp  = qty*4;
            yctemp.c[0] = g_uSubData[tmp+8];
            yctemp.c[1] = g_uSubData[tmp+9];
            yctemp.c[2] = g_uSubData[tmp+10];
            yctemp.c[3] = g_uSubData[tmp+11];

            pA->send.format.data[temp]   = yctemp.c[0];
            pA->send.format.data[temp+1] = yctemp.c[1];
            pA->send.format.data[temp+2] = yctemp.c[2];
            pA->send.format.data[temp+3] = yctemp.c[3];
            pA->send.format.data[temp+4] = 0x00; // Ʒ������
            pA->send.format.data[temp+5] = g_TimeMarker[0];
            pA->send.format.data[temp+6] = g_TimeMarker[1];
            pA->send.format.data[temp+7] = g_TimeMarker[2];
            pA->send.format.data[temp+8] = g_TimeMarker[3];
            pA->send.format.data[temp+9] = g_TimeMarker[4]; //�ϱ������ݣ���5λ�����ڣ���3λ������
            pA->send.format.data[temp+10] = g_TimeMarker[5];
            pA->send.format.data[temp+11] = g_TimeMarker[6];

            sAllSum++;
            uPerDeviceSum++;
            qty++;
        }
        if(iec_count >= uCollectSum)  // һ̨ң�ⲹ�ɷ������
        {
            uPerDeviceSum=0;
            if(subcall->uRelAddr < (g_DeviceSouth.device_sum-1) && subcall->uRelAddr < (MAX_device-1))
            {
                subcall->uRelAddr++;
            }
            else
            {

                subcall->uRelAddr = 0;
                uNeedReadFlash = 0;
                uRecord = NO_DATA;
                sAllSum = 0;
                subcall->subcollect = SUBCOLLECT_END;  // ���ɽ���
            }
        }

        if(qty)  // ����Ҫ�ϱ���ң���
        {
            IecCreateFrameI(M_ME_TF_1,qty|0x80,R_COLLECT_ACK,(qty*5+7),&pA->send); // ������Ϣ�㣬�ɱ�ṹ�޶������λ��1
        }
        break;

    case SUBCOLLECT_END:    // һ�����ݷ��ͽ���

        pA->send.format.maddrL = 0x00;// ��Ϣ���ַ
        pA->send.format.maddrM = 0x00;
        pA->send.format.maddrH = 0x00;
        pA->send.format.data[0] = R_COLLECT_ACK;    //��Ӧ���ٻ�
        pA->send.format.data[1] = g_TimeMarker[0];
        pA->send.format.data[2] = g_TimeMarker[1];
        pA->send.format.data[3] = g_TimeMarker[2];
        pA->send.format.data[4] = g_TimeMarker[3];
        pA->send.format.data[5] = g_TimeMarker[4];
        pA->send.format.data[6] = g_TimeMarker[5];
        pA->send.format.data[7] = g_TimeMarker[6];


        subcall->uRelAddr = 0;
        subcall->subcollect = SUBCOLLECT_NULL;  // û�в���
        IecCreateFrameI(P_TIME_COLLECT,0x01,R_ACTIVE_END,8,&pA->send);
        break;

        /*case SUBCOLLECT_CHECK:	// ƽ̨��ѯ�Ƿ���������Ҫ�ɼ�

            pA->send.format.maddrL = 0x00;	// ��Ϣ���ַ
            pA->send.format.maddrM = 0x00;
            pA->send.format.maddrH = 0x00;
            pA->send.format.data[0] = 0x01;	//00�������ڣ�01������

            Iec104CreateFrameI(P_TIME_COLLECT,0x01,R_RECOLLECT_END,1,pA);
            break;

        case SUBCOLLECT_END_CONFIRM:

            pA->send.format.start   = IEC104_HEAD;  // ������
            pA->send.format.len     = 0x04;
            pA->send.format.SseqL   = 0x01;    // ��������ֽ�1
            pA->send.format.SseqH   = 0x00;    // ��������ֽ�2
            pA->send.format.RseqL   = 0x0A;    // ��������ֽ�1
            pA->send.format.RseqH   = 0x00;    // ��������ֽ�2

            Iec104FrameSend(pA,pA->send.format.len+2);
            break;
           */

    default:

        subcall->collect = SUBCOLLECT_NULL;  // û�в���
        memset(g_TimeMarker,0,7);
        break;

    }
    return;
}
