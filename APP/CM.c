#include "CM.h"
#include "GlobalVar.h"
#include "DataTransferArea.h"
#include "Usart.h"
#include "GPIO.h"
#include "IEC104.h"
#include "LED.h"
#include "tool.h"
#include "WatchDog.h"

//========================================================================

//========================================================================
#define  MI_JUDGE_OK         0     // �����ж���ȷ
#define  MI_JUDGE_TIMEOUT    1     // �ȴ����ݳ�ʱ
#define  MI_JUDGE_ERR        2     // �����ж�����


#define  MI_START            10     // ģ���ʼ����ʼ
#define  MI_AT               11     // ����AT
#define  MI_NO_ECHO          12     // �ػ���
#define  MI_BAUDRATE         13     // ���沨����
#define  MI_CHECK_MODEM      14     // �鿴ģ���ͺ�
#define  MI_FIND_PIN         15     // �鿴SIM��PIN
#define  MI_GET_CSQ          16     // ��ȡ�ź�ǿ��
#define  MI_FIND_NETWORK     17     // ��ѯGSM����״̬
#define  MI_FIND_GPRS        18     // ��ѯGPRS����״̬
#define  MI_PDP_ONE          19     // ����PDP����
#define  MI_ACT_WLAN         20     // �������߳���
#define  MI_ASK_IP           21     // ����IP
#define  MI_DEACT_PDP        22     // �ر�PDP����
#define  MI_OPEN_DNS         23     // ��DNS
#define  MI_FIND_SIMID       24     // ����IMSI
#define  MI_OPEN_SERVER      25     // ���ӷ�����
#define  MI_CLOSE            26     // �ر�����

#define  MI_ADD_HEAD         61     // ������������ͷ
#define  MI_QIRDI            62     // ����������ʾ

#define  MI_REBOOT           40     // ģ������
#define  MI_SEND_INFO        50     // ģ�����ӵ�ƽ̨����������Ϣ
#define  MI_INIT_OK          51     // ģ���ʼ�����


//====================================================================
#define CM_UART_USE           uart4           // ͨѶģ��ʹ�õĴ���
//====================================================================

#define msleep(x)  OSTimeDly((x)/10+1)                // usleep((x)*1000)
#define sleep(x)   OSTimeDly(OS_TICKS_PER_SEC*(x)+1)  // ��ʱx��
//====================================================================
#define CmPowerIoInit()        GPIO_SetDir(1,26,Output)   // ͨѶģ���Դ����IO��
#define CmPowerKeyIoInit()     GPIO_SetDir(1,4,Output)    // ͨѶģ�鿪�ػ�����IO��

#define CmPowerOn()            GPIO_SetBit(1,26,0)
#define CmPowerOff()           GPIO_SetBit(1,26,1)

#define CmPowerKeyHigh()       GPIO_SetBit(1,4,0)
#define CmPowerKeyLow()        GPIO_SetBit(1,4,1)
//====================================================================
// ���λ������ṹ��
#define RING_LEN  524
typedef struct
{
    uint16_t uDataIn;         // ���������±�
    uint16_t uDataOut;        // ��ȡ�����±�
    uint16_t uDataLen;        // ������������
    uint8_t  uDataBuf[RING_LEN];   // ��������
    uint16_t uDataSizeMax;    // �������ݴ�С
} RING_BUFFER_t;
//====================================================================
#define DATA_BUFFER_LEN   350
#define READ_MODEM_LEN    270  // ��ģ�鳤��270�ֽڣ�������524�ֽڡ�
static RING_BUFFER_t g_sRingData={0,0,0,{0},RING_LEN};   // 104���Ļ��λ�����
static char g_uDataBuffer[DATA_BUFFER_LEN];         // ��ʱ�������ݴ洢����
static char g_uSendBuffer[256];                     // ��ʱ�������ݴ洢����

static uint32_t g_uTimerForSyncTime;  // ���ڶ�ʱ��ʱ��ʱʱ��
static uint32_t g_uTimerForRecData;   // ���ڳ�ʱ��û���յ����ݼ�ʱʱ��

static IEC_FORMAT_T   g_sIecCmSend;     // ����IEC104��֡
//====================================================================
//====================================================================
//====================================================================
/****************************************************************************
* ��    �ƣ�RingClear()
* ��    �ܣ���ջ������ݡ�
* ��ڲ�����
*           ������ָ��
* ���ڲ����������� -1
*            ��ȷ���� 0
* ��    ��: ��
****************************************************************************/
int8_t RingClear(RING_BUFFER_t *p)
{
    p->uDataIn = 0;
    p->uDataOut = 0;
    p->uDataLen = 0;

    return 0;
}
/****************************************************************************
* ��    �ƣ�RingNotEmpty
* ��    �ܣ���ȡ�������Ƿ�������
* ��ڲ�����
            *p   ����ṹ��ָ��
* ���ڲ������շ���0���ǿշ���1
* ��    ��: ��
****************************************************************************/
static uint8_t RingNotEmpty(RING_BUFFER_t *p)
{
    return (p->uDataLen?1:0);
}
/****************************************************************************
* ��    �ƣ�RingDataIn
* ��    �ܣ�д�����ݵ�������
* ��ڲ�����
            *p      ����ṹ��ָ��
            *data   ����ָ��
            bytes   ��������
* ���ڲ�����д�����������
* ��    ��: ��
****************************************************************************/
static uint16_t RingDataIn(RING_BUFFER_t *p, const void *data,uint16_t bytes)
{
    uint16_t ret=0;
    uint8_t *p8 = (uint8_t *) data;

    while(0!=bytes)
    {
        if(p->uDataLen < p->uDataSizeMax)
        {
            // buf not full yet.
            p->uDataBuf[p->uDataIn++] = *p8;
            p->uDataIn %= p->uDataSizeMax;
            p->uDataLen++;

            bytes--;
            p8++;
            ret++;
        }
        else
        {
            return ret;
        }
    }

    return ret;
}
/****************************************************************************
* ��    �ƣ�RingDataOut
* ��    �ܣ��ӻ�������ȡ����
* ��ڲ�����
            *p      ����ṹ��ָ��
            *data   ����ָ��
            bytes   ��������
* ���ڲ�������ȡ����������
* ��    ��: ��
****************************************************************************/
static uint16_t RingDataOut(RING_BUFFER_t *p, void *data,uint16_t bytes)
{
    uint16_t ret=0;
    uint8_t *p8 = (uint8_t *) data;

    while(0!=bytes)
    {
        if(p->uDataLen)
        {
            *p8 = p->uDataBuf[p->uDataOut++];
            p->uDataOut %= p->uDataSizeMax;

            p->uDataLen--;

            bytes--;
            p8++;
            ret++;
        }
        else
        {
            return ret;
        }
    }

    return ret;
}
/****************************************************************************
* ��    �ƣ�RingDataShow
* ��    �ܣ���ʾ����������
* ��ڲ�����
            *p      ����ṹ��ָ��
            *data   ����ָ��
            bytes   ��������
* ���ڲ�������ʾ����������
* ��    ��: ��
****************************************************************************/
static uint16_t RingDataShow(RING_BUFFER_t *p, void *data,uint16_t bytes)
{
    uint16_t ret=0;
    uint16_t uDataOut;
    uint16_t uDataLen;
    uint8_t *p8 = (uint8_t *) data;

    uDataOut = p->uDataOut;
    uDataLen = p->uDataLen;

    while(0!=bytes)
    {
        if(uDataLen)
        {
            *p8 = p->uDataBuf[uDataOut++];
            uDataOut %= p->uDataSizeMax;

            uDataLen--;

            bytes--;
            p8++;
            ret++;
        }
        else
        {
            return ret;
        }
    }

    return ret;
}
/****************************************************************************
* ��    �ƣ�RingDataLen
* ��    �ܣ���ȡ��������������
* ��ڲ�����
            *p      ����ṹ��ָ��
* ���ڲ�������������������
* ��    ��: ��
****************************************************************************/
static uint16_t RingDataLen(RING_BUFFER_t *p)
{
    return p->uDataLen;
}
/****************************************************************************
* ��    �ƣ�RingDataThrow
* ��    �ܣ���������������
* ��ڲ�����
            *p       ����ṹ��ָ��
             bytes   ��������
* ���ڲ�����������������
* ��    ��: ��
****************************************************************************/
static uint16_t RingDataThrow(RING_BUFFER_t *p,uint16_t bytes)
{
    uint16_t ret=0;
    while(0!=bytes)
    {
        if(p->uDataLen)
        {
            p->uDataOut++;
            p->uDataOut %= p->uDataSizeMax;

            p->uDataLen--;

            bytes--;

            ret++;
        }
        else
        {
            return ret;
        }
    }
    return ret;
}
//====================================================================
//====================================================================
//====================================================================
//====================================================================
/****************************************************************************
* ��    �ƣ�ReadModem()
* ��    �ܣ���ģ������
* ��ڲ�����
            uLen������
* ���ڲ�����
* ��    ��: ��
****************************************************************************/
int16_t ReadModem(uint16_t uLen)
{
    int16_t iLen;

    iLen = UartRead(CM_UART_USE, g_uDataBuffer, uLen, 3);

    if(iLen>0)
    {
        DEBUGOUT("<CM>:%s\n",g_uDataBuffer);
    }

    return iLen;
}
/****************************************************************************
* ��    �ƣ�WriteModem()
* ��    �ܣ��������ݵ�ģ��
* ��ڲ�����
            pData������ָ��
            uLen�� ����
* ���ڲ�����
* ��    ��: ��
****************************************************************************/
int16_t WriteModem(const void* pData,uint16_t uLen)
{
    int16_t iResult;
    iResult = UartWrite(CM_UART_USE,pData,uLen);

    return iResult;
}
/****************************************************************************
* ��    �ƣ�JudgeFeedback()
* ��    �ܣ��ж�ģ�鷵�ص�����
* ��ڲ�����
            Timeout����ʱʱ��
            pTarget��Ŀ���ַ���
* ���ڲ�����0���ɹ���1����ʱ��2������
* ��    ��: ��
****************************************************************************/
int8_t JudgeFeedback(uint32_t Timeout,const char *pTarget)
{
    int16_t iDataLen;

    uint32_t uStartTime;      // ��ʼ��ʱ���
    uint32_t uNowTime;        // ���ڵ�ʱ���

    uStartTime = OSTimeGet();
    memset(g_uDataBuffer,0,256);

    do
    {
        iDataLen = UartRxLen(CM_UART_USE);
        if(iDataLen<=0)
        {
            uNowTime = OSTimeGet();
            if(NOTIMEOUT == TimeOut(uStartTime,uNowTime,Timeout))  // û�г�ʱ
            {
                msleep(20);
            }
            else
            {
                return MI_JUDGE_TIMEOUT;
            }
        }
        else
        {
            iDataLen = ReadModem(256);
            if(NULL!=strstr(g_uDataBuffer,pTarget))
            {
                return MI_JUDGE_OK;
            }
            else
            {
                return MI_JUDGE_ERR;
            }
        }
    }
    while(1);
}
/******************************************************************************
* ��    �ƣ�ReportSIMID()
* ��    �ܣ��ϱ�SIM��ID��
* ��ڲ�����
            *pA         IEC��Ϣָ��
*
* ���ڲ�������
* ��    ��:
******************************************************************************/
void ReportSIMID(void)
{
    g_sIecCmSend.format.maddrL = 0x00;
    g_sIecCmSend.format.maddrM = 0x00;
    g_sIecCmSend.format.maddrH = 0x00;

    //strncpy(pA->send.format.data,Logger_run.IMSI,15);
    memcpy(g_sIecCmSend.format.data,g_LoggerRun.IMSI,15);
    g_sIecCmSend.format.data[15] = 0x00;
    g_sIecCmSend.format.data[16] = 0x00;
    g_sIecCmSend.format.data[17] = 0x00;
    g_sIecCmSend.format.data[18] = 0x00;
    g_sIecCmSend.format.data[19] = 0x00;

    IecCreateFrameI(P_SIM_ID,1,R_INFO_REPORT,20,&g_sIecCmSend);
}
/****************************************************************************
* ��    �ƣ�GetCsq()
* ��    �ܣ����������߹��ʲ��ϱ�
* ��ڲ�����
            Timeout����ʱʱ��
            pTarget��Ŀ���ַ���
* ���ڲ�����0���ɹ���1����ʱ��2������
* ��    ��: ��
****************************************************************************/
uint8_t GetCsq(const char* cCsqData,uint8_t uCmd)
{
    char *pRec;
    uint8_t i;
    uint8_t uCsq=0;

    pRec = strstr(cCsqData,"+CSQ");

    if(NULL!=pRec)  // ģ�����յ�������ʾ
    {
        pRec += 5;

        for(i=0;i<5;i++)
        {
            if(','==*pRec)
            {
                break;
            }
            else if(*pRec>='0' && *pRec<='9')
            {
                uCsq = uCsq*10 + *pRec-'0';
            }
            pRec++;
        }
        DEBUGOUT("���߹���:%d\n",uCsq);
        g_LoggerRun.uCSQ = uCsq;

        if(99!=uCsq)
        {
            if(uCmd)
            {
                g_sIecCmSend.format.maddrL = 0x00;
                g_sIecCmSend.format.maddrM = 0x00;
                g_sIecCmSend.format.maddrH = 0x00;
                g_sIecCmSend.format.data[0] = uCsq;

                IecCreateFrameI(P_WL_QUALITY,0x01,R_INFO_REPORT,1,&g_sIecCmSend);
            }

        }
    }
    return uCsq;
}
/****************************************************************************
* ��    �ƣ�CmInit()
* ��    �ܣ�ͨѶģ���ʼ��
* ��ڲ�����
            uCmInitStep��ģ���ʼ������
* ���ڲ�����1����ʼ����ɣ�0����ʼ����
* ��    ��: ��
****************************************************************************/
#define CM_INIT_OK      0
#define CM_INIT_ING     1
int8_t CmInit(uint8_t uCmInitStep)
{
    static uint8_t s_uStep=MI_START;  // ģ���ʼ������
    static uint8_t s_uRound=0;        // ����ִ�д���
    static uint8_t s_uRinitCount=0;   // ģ�鸴λ����
    static uint8_t s_uReOpen=0;       // ���´����ӷ���������

    uint8_t uMIFeedback;

   // char   strtemp[7]={'\0'};

    int16_t iDataLen;

    if(uCmInitStep)
    {
        s_uStep = uCmInitStep;
    }

    switch(s_uStep)
    {
    case MI_START:
        sleep(1);
        DEBUGOUT("ģ�鹩��-%d\n",s_uRinitCount);
        CmPowerOn();
        CmPowerKeyHigh();

        DEBUGOUT("ģ�鿪��\n");

        sleep(1);
        CmPowerKeyLow();
        sleep(2);
        CmPowerKeyHigh();

        UartClear(CM_UART_USE);  // ���һ�´��ڻ�����

        sleep(1);

        s_uStep = MI_AT;
        break;

    case MI_AT:

        if(s_uRound>15)
        {
            s_uRound = 0;
            s_uStep = MI_REBOOT;
            break;
        }
        DEBUGOUT("AT-%d\n",s_uRound);

        strncpy(g_uSendBuffer,"AT\r\n",4);
        iDataLen = 4;
        WriteModem(g_uSendBuffer,iDataLen);
        s_uRound++;

        uMIFeedback = JudgeFeedback(500,"OK");

        if(MI_JUDGE_OK==uMIFeedback)
        {
            s_uRound = 0;
            msleep(500);
            s_uStep = MI_NO_ECHO;
            g_LoggerRun.north_status = NORTH_RDY;
        }
        break;

    case MI_NO_ECHO:

        if(s_uRound>15)
        {
            s_uRound = 0;
            s_uStep = MI_REBOOT;
            break;
        }
        DEBUGOUT("�ػ���-%d\n",s_uRound);

        strncpy(g_uSendBuffer,"ATE0\r\n",6);
        iDataLen = 6;
        WriteModem(g_uSendBuffer,iDataLen);
        s_uRound++;

        uMIFeedback = JudgeFeedback(500,"OK");
        if(MI_JUDGE_OK==uMIFeedback)
        {
            s_uRound = 0;
            msleep(500);
            s_uStep = MI_BAUDRATE;
        }
        break;

    case MI_BAUDRATE:
        if(s_uRound>15)
        {
            s_uRound = 0;
            s_uStep = MI_REBOOT;
            break;
        }
        DEBUGOUT("ģ�鲨����-%d\n",s_uRound);

        strncpy(g_uSendBuffer,"AT+IPR=115200&W\r\n",17);
        iDataLen = 17;
        WriteModem(g_uSendBuffer,iDataLen);
        s_uRound++;

        uMIFeedback = JudgeFeedback(1000,"OK");
        if(MI_JUDGE_OK==uMIFeedback)
        {
            s_uRound = 0;
            msleep(300);
            s_uStep = MI_CHECK_MODEM;
        }
        break;

    case MI_CHECK_MODEM:
        if(s_uRound>15)
        {
            s_uRound = 0;
            s_uStep = MI_REBOOT;
            break;
        }
        DEBUGOUT("���ģ��-%d\n",s_uRound);

        strncpy(g_uSendBuffer,"AT+GMR\r\n",8);
        iDataLen = 8;
        WriteModem(g_uSendBuffer,iDataLen);
        s_uRound++;

        uMIFeedback = JudgeFeedback(6000,"OK");
        if(MI_JUDGE_OK==uMIFeedback)
        {
            s_uRound = 0;
            msleep(300);
            s_uStep = MI_FIND_PIN;
        }
        break;

    case MI_FIND_PIN:
        if(s_uRound>10)
        {
            s_uRound = 0;
            s_uStep = MI_REBOOT;
            break;
        }
        DEBUGOUT("��SIM-%d\n",s_uRound);

        strncpy(g_uSendBuffer,"AT+CPIN?\r\n",10);
        iDataLen = 10;
        WriteModem(g_uSendBuffer,iDataLen);
        s_uRound++;

        uMIFeedback = JudgeFeedback(10000,"READY");
        if(MI_JUDGE_OK==uMIFeedback)
        {
            s_uRound = 0;
            msleep(300);
            s_uStep = MI_FIND_NETWORK;
        }
        else if(MI_JUDGE_TIMEOUT==uMIFeedback)
        {
            s_uRound = 0;
            s_uStep = MI_REBOOT;
        }
        else
        {
            sleep(1);
        }
        break;

    case MI_FIND_NETWORK:
        if(s_uRound>30)
        {
            s_uRound = 0;
            s_uStep = MI_REBOOT;
            break;
        }
        sleep(3);
        DEBUGOUT("������-%d\n",s_uRound);

        strncpy(g_uSendBuffer,"AT+CREG?\r\n",10);
        iDataLen = 10;
        WriteModem(g_uSendBuffer,iDataLen);
        s_uRound++;

        uMIFeedback = JudgeFeedback(60000,"0,");
        if(MI_JUDGE_OK==uMIFeedback)
        {
            if(NULL!=strstr(g_uDataBuffer,"0,1") || NULL!=strstr(g_uDataBuffer,"0,5"))
            {
                s_uRound = 0;
                msleep(300);
                s_uStep = MI_FIND_GPRS;
            }
        }
        else if(MI_JUDGE_TIMEOUT==uMIFeedback)
        {
            s_uRound = 0;
            s_uStep = MI_REBOOT;
        }
        else
        {
            sleep(1);
        }
        break;

    case MI_FIND_GPRS:
        if(s_uRound>30)
        {
            s_uRound = 0;
            s_uStep = MI_PDP_ONE;
            break;
        }
        sleep(3);
        DEBUGOUT("��GPRS-%d\n",s_uRound);

        strncpy(g_uSendBuffer,"AT+CGREG?\r\n",11);
        iDataLen = 11;
        WriteModem(g_uSendBuffer,iDataLen);
        s_uRound++;

        uMIFeedback = JudgeFeedback(60000,"0,");
        if(MI_JUDGE_OK==uMIFeedback)
        {
            if(NULL!=strstr(g_uDataBuffer,"0,1") || NULL!=strstr(g_uDataBuffer,"0,5"))
            {
                s_uRound = 0;
                msleep(300);
                s_uStep = MI_GET_CSQ;
            }
        }
        else if(MI_JUDGE_TIMEOUT==uMIFeedback)
        {
            s_uRound = 0;
            s_uStep = MI_REBOOT;
        }
        else
        {
            sleep(1);
        }
        break;

    case MI_GET_CSQ:
        if(s_uRound>10)
        {
            s_uRound = 0;
            s_uStep = MI_REBOOT;
            break;
        }
        DEBUGOUT("�ź�ǿ��-%d\n",s_uRound);

        strncpy(g_uSendBuffer,"AT+CSQ\r\n",8);
        iDataLen = 8;
        WriteModem(g_uSendBuffer,iDataLen);
        s_uRound++;

        uMIFeedback = JudgeFeedback(60000,"+CSQ");
        if(MI_JUDGE_OK==uMIFeedback)
        {
            s_uRound = 0;
            GetCsq(g_uDataBuffer,0);
            msleep(300);
            s_uStep = MI_PDP_ONE;
        }
        else if(MI_JUDGE_TIMEOUT==uMIFeedback)
        {
            s_uRound = 0;
            s_uStep = MI_REBOOT;
        }
        else
        {
            sleep(1);
        }
        break;

    case MI_PDP_ONE:
        if(s_uRound>10)
        {
            s_uRound = 0;
            s_uStep = MI_DEACT_PDP;
            break;
        }
        DEBUGOUT("����PDP-%d\n",s_uRound);

        strncpy(g_uSendBuffer,"AT+QIREGAPP\r\n",13);
        iDataLen = 13;
        WriteModem(g_uSendBuffer,iDataLen);
        s_uRound++;

        uMIFeedback = JudgeFeedback(180000,"OK");
        if(MI_JUDGE_OK==uMIFeedback)
        {
            s_uRound = 0;
            msleep(300);
            s_uStep = MI_ACT_WLAN;
        }
        else if(MI_JUDGE_TIMEOUT==uMIFeedback)
        {
            s_uRound = 0;
            s_uStep = MI_REBOOT;
        }
        else
        {
            sleep(1);
        }
        break;

    case MI_ACT_WLAN:
        if(s_uRound>3)
        {
            s_uRound = 0;
            s_uStep = MI_REBOOT;
            break;
        }
        DEBUGOUT("�������߳���-%d\n",s_uRound);

        strncpy(g_uSendBuffer,"AT+QIACT\r\n",10);
        iDataLen = 10;
        WriteModem(g_uSendBuffer,iDataLen);
        s_uRound++;

        uMIFeedback = JudgeFeedback(180000,"OK");
        if(MI_JUDGE_OK==uMIFeedback)
        {
            s_uRound = 0;
            msleep(300);
            s_uStep = MI_ASK_IP;
        }
        else if(MI_JUDGE_TIMEOUT==uMIFeedback)
        {
            s_uRound = 0;
            s_uStep = MI_REBOOT;
        }
        else
        {
            sleep(1);
        }
        break;

    case MI_ASK_IP:
        if(s_uRound>3)
        {
            s_uRound = 0;
            s_uStep = MI_DEACT_PDP;
            break;
        }
        DEBUGOUT("����IP-%d\n",s_uRound);

        strncpy(g_uSendBuffer,"AT+QILOCIP\r\n",12);
        iDataLen = 12;
        WriteModem(g_uSendBuffer,iDataLen);
        s_uRound++;

        uMIFeedback = JudgeFeedback(180000,".");
        if(MI_JUDGE_OK==uMIFeedback)
        {
            s_uRound = 0;
            msleep(300);
            s_uStep = MI_OPEN_DNS;
        }
        else if(MI_JUDGE_TIMEOUT==uMIFeedback)
        {
            s_uRound = 0;
            s_uStep = MI_REBOOT;
        }
        else
        {
            sleep(1);
        }
        break;

    case MI_DEACT_PDP:
        if(s_uRound>10)
        {
            s_uRound = 0;
            s_uStep = MI_REBOOT;
            break;
        }
        DEBUGOUT("�ر�PDP-%d\n",s_uRound);

        strncpy(g_uSendBuffer,"AT+QIDEACT\r\n",12);
        iDataLen = 12;
        WriteModem(g_uSendBuffer,iDataLen);
        s_uRound++;

        uMIFeedback = JudgeFeedback(90000,"OK");
        if(MI_JUDGE_OK==uMIFeedback)
        {
            s_uRound = 0;
            msleep(300);
            s_uStep = MI_FIND_PIN;
        }
        else if(MI_JUDGE_TIMEOUT==uMIFeedback)
        {
            s_uRound = 0;
            s_uStep = MI_REBOOT;
        }
        else
        {
            sleep(1);
        }
        break;

    case MI_OPEN_DNS:
        if(s_uRound>3)
        {
            s_uRound = 0;
            s_uStep = MI_DEACT_PDP;
            break;
        }
        DEBUGOUT("��DNS-%d\n",s_uRound);

        strncpy(g_uSendBuffer,"AT+QIDNSIP=1\r\n",14);
        iDataLen = 14;
        WriteModem(g_uSendBuffer,iDataLen);
        s_uRound++;

        uMIFeedback = JudgeFeedback(90000,"OK");
        if(MI_JUDGE_OK==uMIFeedback)
        {
            s_uRound = 0;
            msleep(300);
            s_uStep = MI_FIND_SIMID;
        }
        else if(MI_JUDGE_TIMEOUT==uMIFeedback)
        {
            s_uRound = 0;
            s_uStep = MI_REBOOT;
        }
        else
        {
            sleep(1);
        }
        break;

    case MI_FIND_SIMID:
        if(s_uRound>10)
        {
            s_uRound = 0;
            s_uStep = MI_DEACT_PDP;
            break;
        }
        DEBUGOUT("����IMSI-%d\n",s_uRound);

        strncpy(g_uSendBuffer,"AT+CIMI\r\n",9);
        iDataLen = 9;
        WriteModem(g_uSendBuffer,iDataLen);
        s_uRound++;

        uMIFeedback = JudgeFeedback(90000,"OK");
        if(MI_JUDGE_OK==uMIFeedback)
        {
            strncpy(g_LoggerRun.IMSI,&g_uDataBuffer[2],15);
            DEBUGOUT("IMSI:%-15.15s\r\n",g_LoggerRun.IMSI);
            s_uRound = 0;
            msleep(300);
            s_uStep = MI_QIRDI;
        }
        else if(MI_JUDGE_TIMEOUT==uMIFeedback)
        {
            s_uRound = 0;
            s_uStep = MI_REBOOT;
        }
        else
        {
            sleep(1);
        }
        break;

    /*case MI_ADD_HEAD:
        if(s_uRound>3)
        {
            s_uRound = 0;
            s_uStep = MI_DEACT_PDP;
            break;
        }
        DEBUGOUT("���ͷ-%d\n",s_uRound);

        strncpy(g_uSendBuffer,"AT+QIHEAD=1\r\n",13);
        iDataLen = 13;
        WriteModem(g_uSendBuffer,iDataLen);
        s_uRound++;

        uMIFeedback = JudgeFeedback(400,"OK");
        if(MI_JUDGE_OK==uMIFeedback)
        {
            s_uRound = 0;
            msleep(300);
            s_uStep = MI_OPEN_SERVER;
        }
        else if(MI_JUDGE_TIMEOUT==uMIFeedback)
        {
            s_uRound = 0;
            s_uStep = MI_REBOOT;
        }
        else
        {
            sleep(1);
        }
        break;*/

    case MI_QIRDI:
        if(s_uRound>3)
        {
            s_uRound = 0;
            s_uStep = MI_DEACT_PDP;
            break;
        }
        DEBUGOUT("������ʾ-%d\n",s_uRound);

        strncpy(g_uSendBuffer,"AT+QINDI=1\r\n",12);
        iDataLen = 12;
        WriteModem(g_uSendBuffer,iDataLen);
        s_uRound++;

        uMIFeedback = JudgeFeedback(400,"OK");
        if(MI_JUDGE_OK==uMIFeedback)
        {
            s_uRound = 0;
            msleep(300);
            s_uStep = MI_OPEN_SERVER;
        }
        else if(MI_JUDGE_TIMEOUT==uMIFeedback)
        {
            s_uRound = 0;
            s_uStep = MI_REBOOT;
        }
        else
        {
            sleep(1);
        }
        break;

    case MI_OPEN_SERVER:
        if(s_uRound>5)
        {
            s_uRound = 0;
            s_uStep = MI_DEACT_PDP;
            break;
        }
        s_uReOpen++;
        if(s_uReOpen>5)
        {
            s_uRound = 0;
            s_uReOpen = 0;
            s_uStep = MI_REBOOT;
            break;
        }
        g_LoggerRun.north_status = NORTH_DISCON;

        DEBUGOUT("��������-%d\n",s_uRound);

        memset(g_uSendBuffer,0,256);
        //strncpy(g_uSendBuffer,"AT+QIOPEN=\"TCP\",",16);
        //strcat(g_uSendBuffer,g_LoggerInfo.server_domain);
        //strcat(g_uSendBuffer,",");

        //I2Str(g_LoggerInfo.server_port,strtemp);
        //strcat(g_uSendBuffer,strtemp);
        //strcat(g_uSendBuffer,"\r\n");
        snprintf(g_uSendBuffer,sizeof(g_uSendBuffer),"AT+QIOPEN=\"TCP\",%s,%d\r\n",g_LoggerInfo.server_domain,g_LoggerInfo.server_port);
        //iDataLen = strlen(g_uSendBuffer);

        WriteModem(g_uSendBuffer,strlen(g_uSendBuffer));
		DEBUGOUT("%s\n",g_uSendBuffer);
        s_uRound++;

        uMIFeedback = JudgeFeedback(90000,"OK");
        if(MI_JUDGE_OK==uMIFeedback)
        {
            //s_uRound = 0;
            //s_uStep = MI_SEND_INFO;
        }
        else if(MI_JUDGE_TIMEOUT==uMIFeedback)
        {
            s_uRound = 0;
            s_uStep = MI_DEACT_PDP;
            break;
        }
        else
        {
            sleep(1);
            break;
        }
        uMIFeedback = JudgeFeedback(90000,"CONNECT OK");
        if(MI_JUDGE_OK==uMIFeedback)
        {
            s_uRound = 0;
            s_uRinitCount = 0;
            s_uStep = MI_SEND_INFO;
            g_LoggerRun.north_status = NORTH_CONNECT;
        }
        else if(MI_JUDGE_TIMEOUT==uMIFeedback)
        {
            s_uRound = 0;
            s_uStep = MI_DEACT_PDP;
        }
        else
        {
            if(NULL!=strstr(g_uDataBuffer,"ALREADY CONNECT"))  // �Ѿ�����
            {
                s_uRound = 0;
                s_uRinitCount = 0;
                s_uStep = MI_SEND_INFO;
                g_LoggerRun.north_status = NORTH_CONNECT;
            }
            sleep(3);
        }
        break;

    case MI_CLOSE:
        if(s_uRound>3)
        {
            s_uRound = 0;
            s_uStep = MI_REBOOT;
            break;
        }
        DEBUGOUT("������-%d\n",s_uRound);

        g_LoggerRun.north_status = NORTH_DISCON;

        strncpy(g_uSendBuffer,"AT+QICLOSE\r\n",12);
        iDataLen = 12;
        WriteModem(g_uSendBuffer,iDataLen);
        s_uRound++;

        uMIFeedback = JudgeFeedback(90000,"OK");
        if(MI_JUDGE_OK==uMIFeedback)
        {
            s_uRound = 0;
            msleep(300);
            s_uStep = MI_OPEN_SERVER;
        }
        else if(MI_JUDGE_TIMEOUT==uMIFeedback)
        {
            s_uRound = 0;
            s_uStep = MI_REBOOT;
        }
        else
        {
            sleep(1);
        }
        break;

    case MI_REBOOT:

        DEBUGOUT("ģ��ػ�\n");
        CmPowerKeyLow();
        sleep(1);
        CmPowerKeyHigh();

        g_LoggerRun.north_status = NORTH_POWERDOWN;

        sleep(13);
        DEBUGOUT("ģ��ϵ�-%d\n",s_uRinitCount);
        CmPowerOff();

        s_uRinitCount++;

        if(s_uRinitCount<2)
        {
            DEBUGOUT("ģ���ʼ��ʧ��%d�Σ����³�ʼ��\r\n", s_uRinitCount);
            OSTimeDlyHMSM(0,0,10,0);
        }
        else if(2==s_uRinitCount)
        {
            DEBUGOUT("ģ���ʼ��ʧ��%d�Σ�5���Ӻ����³�ʼ��\r\n", s_uRinitCount);
            OSTimeDlyHMSM(0,5,0,0);
        }
        else if(3==s_uRinitCount)
        {
            DEBUGOUT("ģ���ʼ��ʧ��%d�Σ�10���Ӻ����³�ʼ��\r\n", s_uRinitCount);
            OSTimeDlyHMSM(0,10,0,0);
        }
        else if(s_uRinitCount>=4 && s_uRinitCount<=18)
        {
            DEBUGOUT("ģ���ʼ��ʧ��%d�Σ�15���Ӻ����³�ʼ��\r\n", s_uRinitCount);
            OSTimeDlyHMSM(0,15,0,0); 
        }
        else
        { 
            //sleep(5);
            DEBUGOUT("��ƽ̨����4Сʱ��Reboot!!!\n");
            Reboot();
        }

        s_uStep = MI_START;
        break;

    case MI_SEND_INFO:
        s_uReOpen = 0;

        CmDelSendLink();                          // ��շ�������
        CmDelRecLink();                           // ��ս�������
        RingClear(&g_sRingData);                  // ��ջ���
        DEBUGOUT("�ϱ�������Ϣ\n");
        IecReportLogInfo(R_INFO_REPORT);          // �ϱ�������Ϣ
        ReportSIMID();                            // �ϱ�SIM��ID
        g_uTimerForSyncTime = OSTimeGet();        // �ϱ�������Ϣ���¼ʱ�����ڶ�ʱ��ʱ�Ƚ�
        g_uTimerForRecData = g_uTimerForSyncTime; // ��ʱ��û�н��յ�����ʱ��
        s_uStep = MI_INIT_OK;
        break;

    case MI_INIT_OK:
        return CM_INIT_OK;
        break;

    default:
        s_uStep = MI_START;
        s_uRound = 0;
		DEBUGOUT("ģ���쳣�ϵ�\n");
        CmPowerOff();    // ������
        break;
    }

    return s_uStep;
}


//======================================================================================
#define W_WAIT_DATA     0  // �ȴ����ݲ���
#define W_SEND_AT       1  // ���ͷ�������ATָ��
#define W_WAIT_SIGN     2  // �ȴ���>��
#define W_SEND_DATA     3  // ��������
#define W_WAIT_OK       4  // �ȴ���SEND OK��
#define W_PAUSE         5  // ��ͣ��������
#define W_WAIT_TIME     6  // �ȴ���ͣ��ʱ

#define R_WAIT_TIME     0   // �ȴ���ʱ���������ѯ����
#define R_WAIT_COM      1   // �ȴ�������Դ
#define R_SEND_AT       2   // ����ATָ���ȡ����
#define R_WAIT_DATA     3   // �ȴ�����
#define R_PROCESS_DATA  4   // ��������
#define R_RESTART       5   // ���¿�ʼ

OS_STK TaskModemProcessStk[TaskModemProcessStkSize]@0x20000000;	      // ���������ջ��С��ָ���ڴ��ַΪ@0x20000000
/****************************************************************************
* ��    �ƣ�TaskModemProcess()
* ��    �ܣ�ͨѶģ������
* ��ڲ�����

* ���ڲ�������
* ��    ��: ��
****************************************************************************/
void TaskModemProcess(void *p)
{
    p = p;
    static uint8_t uCmInitStep=0;
    static uint8_t s_uMainInit=CM_INIT_ING;   // ��ͨ����ʼ��״̬

    int16_t iRecLen;
    char *pRec;
    uint16_t i;
    uint16_t uDataLen;

    static uint8_t s_uWriteStep=0;      // дģ��״̬����
    static uint8_t s_uReadStep=0;       // ��ģ��״̬����
    static uint32_t s_uWriteTimeMark;   // дģ��ʱ��
    static uint32_t s_uReadTimeMark;    // ��ģ��ʱ��
    static uint32_t s_uTimeForCsq;      // ��ѯ�ź�ǿ��ʱ��

    uint32_t uNowTimemark;

    CmPowerIoInit();                     // ģ���Դ����IO�ڳ�ʼ��
    CmPowerKeyIoInit();                  // ģ�鿪�ػ�����IO�ڳ�ʼ��

    s_uWriteTimeMark = OSTimeGet();      // ��ȡ��ǰʱ��
    s_uReadTimeMark  = s_uWriteTimeMark;
    s_uTimeForCsq    = s_uReadTimeMark;

    // ģ�������շ������ʼ��
    CmLinkInit();

    while(1)
    {

		//FEED_DOG(); 	// ι��
		if(0x0A==g_LoggerRun.update)
        {
            sleep(10);
            continue;
        }


        if(s_uMainInit)
        {
            s_uMainInit = CmInit(uCmInitStep);  // ģ���ʼ��

            uCmInitStep = 0;

            s_uWriteStep = W_WAIT_DATA;
            s_uReadStep  = R_WAIT_TIME;
            continue;
        }


        uNowTimemark = OSTimeGet();  // ���µ���ʱ��ʱ��

        //==========================================================================================================
        // ��������ģ������
        if(UartRxLen(CM_UART_USE)<=0) // ��ȡģ�鷢�͸�оƬ������
        {

		// ��ʱ��ʱ����
            if(NORTH_OK!=g_LoggerRun.north_status)
            {
                if(TIMEOUT == TimeOut(g_uTimerForSyncTime,uNowTimemark,60000))  // ��ʱδ�յ���ʱ����@60��
                {
                    uCmInitStep = MI_CLOSE;      // ģ��ر�����
                    s_uMainInit = CM_INIT_ING;   // ����ģ���ʼ������
                    DEBUGOUT("δ��ʱ\n");
                    continue;
                }
            }

            // ������ʱ����
            if(TIMEOUT == TimeOut(g_uTimerForRecData,uNowTimemark,100000))    // ��ʱδ�յ����ݱ���@100��
            {
                uCmInitStep = MI_CLOSE;      // ģ��ر�����
                s_uMainInit = CM_INIT_ING;   // ����ģ���ʼ������
                DEBUGOUT("������\n");
                continue;
            }

            // ���ڲ�ѯ�ź�ǿ��
            if(NORTH_OK==g_LoggerRun.north_status)
            {
                if(W_WAIT_DATA==s_uWriteStep && R_WAIT_TIME==s_uReadStep && 0==g_LoggerRun.update)  // ģ���д���ǿ��е�ʱ�� �� ������������
                {
                    if(TIMEOUT == TimeOut(s_uTimeForCsq,uNowTimemark,60000))       // ÿ���Ӳ�ѯ�ź�ǿ�ȱ���@60��
                    {
                        WriteModem("AT+CSQ\r\n",8);   // ��ѯģ���ź�ǿ��
                        s_uTimeForCsq = uNowTimemark; // ����ʱ��
                        s_uWriteStep = W_PAUSE;       // ��ͣ��������
                        s_uReadStep  = R_RESTART;     // ���ö�ģ���ʱ
                    }
                }
            }
            else
            {
                s_uTimeForCsq = uNowTimemark; // ����ʱ��
            }

			FEED_DOG();
            msleep(10);
            
            LedTwinkle1();

        }
        else
        {


			memset(g_uDataBuffer,0,DATA_BUFFER_LEN);
            iRecLen = UartRead(CM_UART_USE, g_uDataBuffer, DATA_BUFFER_LEN, 3);

            if(iRecLen<=0)
            {
                continue;
            }
            DEBUGOUT("CM %d:%s\n",iRecLen,g_uDataBuffer);


            if(strstr(g_uDataBuffer,">"))  // �����������յ���>��
            {
                if(W_WAIT_SIGN==s_uWriteStep)
                {
                    s_uWriteStep = W_SEND_DATA;
                }
            }

            if(strstr(g_uDataBuffer,"SEND OK"))  // �����������յ���SEND OK��
            {
                if(W_WAIT_OK==s_uWriteStep)
                {
                    s_uWriteStep = W_WAIT_DATA;
                }
            }
            else if(strstr(g_uDataBuffer,"OK"))  // ������ѯģ�����ݣ�����OK�����¿�ʼ
            {
                if(R_WAIT_DATA==s_uReadStep)
                {
                    s_uReadStep = R_RESTART;
                }
            }

            if(strstr(g_uDataBuffer,"+QIRDI"))  // ģ�����յ�������ʾ
            {
                s_uReadStep = R_WAIT_COM;
            }
            else if(strstr(g_uDataBuffer,"+QIRD"))   // �յ�ģ�����͵�����
            {
                //if(R_WAIT_DATA==s_uReadStep)
                {
                    s_uReadStep = R_PROCESS_DATA;
                }
            }

            if(strstr(g_uDataBuffer,"+CSQ"))  // ģ����յ��ź�ǿ��
            {
                GetCsq(g_uDataBuffer,1);
            }

            if(strstr(g_uDataBuffer,"CLOSED"))  // ģ�����͡�close��
            {
                uCmInitStep = MI_OPEN_SERVER;
                s_uMainInit = CM_INIT_ING;   // ����ģ���ʼ������
                continue;
            }

            if(strstr(g_uDataBuffer,"+PDP DEACT"))
            {
                uCmInitStep = MI_PDP_ONE;
                s_uMainInit = CM_INIT_ING;   // ����ģ���ʼ������
            }
        }

        
        //==========================================================================================================
        // ����ģ�����ݷ���
        switch(s_uWriteStep)
        {
        case W_WAIT_DATA:
            uDataLen = CmSendLinkLen();
            if(uDataLen && R_WAIT_TIME==s_uReadStep)  // ���������� �� ���ٶ�ģ�����ݹ�����
            {
                s_uWriteStep = W_SEND_AT;
            }
            break;
        case W_SEND_AT:
            memset(g_uSendBuffer,0,256);

            strncpy(g_uSendBuffer,"AT+QISEND=",10);
            I2Str(uDataLen,&g_uSendBuffer[10]);
            strcat(g_uSendBuffer,"\r\n");

            WriteModem(g_uSendBuffer,strlen(g_uSendBuffer));

            DEBUGOUT("\n--");
            s_uWriteTimeMark = uNowTimemark;
            s_uWriteStep = W_WAIT_SIGN;
            break;
        case W_WAIT_SIGN:// �ȴ�">"
            if(TIMEOUT == TimeOut(s_uWriteTimeMark,uNowTimemark,500))  // ��ʱ
            {
                DEBUGOUT("��>\n");
                //uCmInitStep = MI_CLOSE;  // ģ��ر�����
                //s_uMainInit = CM_INIT_ING;   // ����ģ���ʼ������
                s_uWriteStep = W_SEND_DATA;    // �ղ���>������������
            }
            break;
        case W_SEND_DATA:
            memset(g_uSendBuffer,0,256);
            uDataLen = CmReadSendLink((uint8_t *)g_uSendBuffer); // ������
            WriteModem(g_uSendBuffer,uDataLen);
            if(1 == (g_uSendBuffer[2] & 0x03))
            {
                DEBUGOUT("SendS:");
            }
            else if(3 == (g_uSendBuffer[2] & 0x03))
            {
                DEBUGOUT("SendU:");
            }
            else
            {
                DEBUGOUT("SendI:");
            }

            for(i=0; i<uDataLen; i++)
            {
                DEBUGOUT("%02X ",g_uSendBuffer[i]);
            }
            DEBUGOUT("\n");

            s_uWriteTimeMark = uNowTimemark;
            s_uWriteStep = W_WAIT_OK;
            break;
        case W_WAIT_OK: // �ȴ���SEND OK��
            if(TIMEOUT == TimeOut(s_uWriteTimeMark,uNowTimemark,500))  // ��ʱ
            {
                DEBUGOUT("��SEND OK,����\n");
                uCmInitStep = MI_CLOSE;  // ģ��ر�����
                s_uMainInit = CM_INIT_ING;   // ����ģ���ʼ������
            }
            break;

        case W_PAUSE:
            s_uWriteTimeMark = uNowTimemark;
            s_uWriteStep = W_WAIT_TIME;
            break;

        case W_WAIT_TIME:
            if(TIMEOUT == TimeOut(s_uWriteTimeMark,uNowTimemark,500))  // ��ͣ500ms
            {
                s_uWriteStep = W_WAIT_DATA;
            }
            break;

        default:
            s_uWriteStep = W_WAIT_DATA;
            break;
        }
        //==========================================================================================================
        // ����ģ�����ݽ���
        // DEBUGOUT("3333\n");
        switch(s_uReadStep)
        {
        case R_WAIT_TIME:
            if(TIMEOUT == TimeOut(s_uReadTimeMark,uNowTimemark,3000))   // ��ǰ��10��//���ڽ��տ���ʱ��5S
            {
                s_uReadStep = R_WAIT_COM;
            }
            break;
        case R_WAIT_COM:
            if(W_WAIT_DATA==s_uWriteStep)  // ��������û��ռ��ģ����Դ
            {
                s_uReadStep = R_SEND_AT;
            }
            break;
        case R_SEND_AT:
            DEBUGOUT("��ģ��\n");
            WriteModem("AT+QIRD=0,1,0,270\r\n",19);
            s_uReadTimeMark  = uNowTimemark;
            s_uReadStep = R_WAIT_DATA;
            break;
        case R_WAIT_DATA: // �ȴ�����
            if(TIMEOUT == TimeOut(s_uReadTimeMark,uNowTimemark,400))  // ��ʱ
            {
                DEBUGOUT("��ģ�鳬ʱ\n");
                s_uReadStep = R_RESTART;
            }
            break;
        case R_PROCESS_DATA:
			 //DEBUGOUT("44444\n");
            pRec = strstr(g_uDataBuffer,"TCP");

            if(NULL == pRec)
            {
                s_uReadTimeMark  = uNowTimemark;
                s_uReadStep = R_WAIT_TIME;
                break;
            }

            for(i=0,uDataLen=0;i<7;i++)
            {
                if(*pRec<='9' && *pRec>='0')
                {
                    uDataLen = uDataLen*10 + *pRec - '0';
                }
                else if('\r'==*pRec)
                {
                    break;
                }
                pRec++;
            }

            if((iRecLen-(pRec - g_uDataBuffer)-8) < uDataLen)  // �յ���������С��ģ�����͵���������
            {
                /*DEBUGOUT("iRecLen:%d\n",iRecLen);
                DEBUGOUT("pRec:%d\n",pRec);
                DEBUGOUT("g_uDataBuffer:%d\n",g_uDataBuffer);
                DEBUGOUT("XXXXXX:%d\n",iRecLen-(pRec - g_uDataBuffer)-8);

                for(i=0;i<iRecLen;i++)
                {
                    DEBUGOUT("%02X ",g_uDataBuffer[i]);
                }
                DEBUGOUT("\n");*/

                s_uReadTimeMark  = uNowTimemark;
                s_uReadStep = R_WAIT_TIME;
                RingClear(&g_sRingData);   // ��ջ���
                DEBUGOUT("������:%d<%d\n",(iRecLen-(pRec - g_uDataBuffer)-8),uDataLen);
                break;
            }

            DEBUGOUT("DataLen:%d\n",uDataLen);

            pRec++;
            pRec++;
            RingDataIn(&g_sRingData,pRec,uDataLen); // ����ģ��������ݽ��ջ��λ������������֡

            g_uTimerForRecData = uNowTimemark;   // �����յ�����ʱ��
            //=============================================================
            // �������������ݣ���������
            //DEBUGOUT("5555\n");
            while(RingNotEmpty(&g_sRingData))
            {

			//DEBUGOUT("66666\n");

				if(2==RingDataShow(&g_sRingData,g_uDataBuffer,2))  // ����ʾ����������������ֽ�
                {
                    if(0x68==g_uDataBuffer[0])  // ͷ�ֽ�Ϊ0x68��Ϊ104ͷ�������ӵ�����
                    {
                        if(RingDataLen(&g_sRingData) >= (g_uDataBuffer[1]+2)) // �����������������һ֡104���ģ���������
                        {
                            RingDataOut(&g_sRingData,g_uDataBuffer,g_uDataBuffer[1]+2);
                        }
                        else
                        {
                            break;
                        }
                    }
                    else
                    {
                        RingDataThrow(&g_sRingData,1);
                        continue;
                    }
                }
                else
                {
                    break;
                }


                if(1==(g_uDataBuffer[2]&0x03))
                {
                    DEBUGOUT("\nRecvS:");
                }
                else if(3==(g_uDataBuffer[2]&0x03))
                {
                    DEBUGOUT("\nRecvU:");
                }
                else
                {
                    DEBUGOUT("\nRecvI:");
                }

                uDataLen = g_uDataBuffer[1] + 2;
                CmStoreRecLink((uint8_t*)g_uDataBuffer,uDataLen);

                for(i=0; i<uDataLen; i++)
                {
                    DEBUGOUT("%02X ",g_uDataBuffer[i]);
                }
			 //DEBUGOUT("8888\n");
				
            }
            //=============================================================
            s_uReadTimeMark  = uNowTimemark;
            s_uReadStep = R_WAIT_TIME;

            break;

        case R_RESTART:
            s_uReadTimeMark  = uNowTimemark;
            s_uReadStep = R_WAIT_TIME;
            break;

        default:
            s_uReadStep = R_WAIT_TIME;
            break;
        }
        //==========================================================================================================
    }
}

