#ifndef MODBUS_MASTER_H_
#define MODBUS_MASTER_H_
#include "ucos_ii.h"
#include "GlobalVar.h"
#ifndef _STDINT
#include <stdint.h>
#endif
//-----------------------------------------------------------------------------------------
#define MASTER_OK       (0)     // �㲥�󷵻�
#define MASTER_lOST     (41)    // ��֡
#define MASTER_CRC      (42)    // CRCУ�����
#define MASTER_ERR      (43)    // ����

#define MODBUS_ILLEGAL_FUN    (1)  //
#define MODBUS_ILLEGAL_ADDR   (2)  //
#define MODBUS_ILLEGAL_VALUE  (3)  //
#define MODBUS_ILLEGAL_DEV    (4)  //
#define MODBUS_ILLEGAL_BUSY   (6)  //

#define DT1000UPDATE_START   0x1C     //��������֡
#define DT1000UPDATE_DATA    0x2C     //�������ݴ���֡
#define DT1000UPDATE_END     0x3C     //��������֡

#define DT1000LOG_START      0x1D     //��־����֡
#define DT1000LOG_DATA       0x2D     //��־���ݴ���֡
#define DT1000LOG_END        0x3D     //��־����֡


#define DISCOVERY_START      0x1B   //�Է��ֹ㲥֡
#define DISCOVERY_SET_ADDR   0x2B   //��ַ�Է���
#define DISCOVERY_REPORT     0x3B   //�Է����豸�ϱ�
/*
0 - No error.
1 - Illegal function.
2 - Illegal data address.
3 - Illegal data value.
4 - Slave device failure.
5 - Acknowledge.
6 - Slave device busy.
7 - Negative acknowledge.
8 - Memory parity error.
*/
//-----------------------------------------------------------------------------------------
#define MASTER_ROUND_OVER   (0)   // 1:������ѭ�����ƣ�0:�رմ�ѭ������
//-----------------------------------------------------------------------------------------
typedef struct
{
    OS_EVENT *pComLock;         // ������ָ��

    int16_t iComFd;             // �����ļ���

    uint8_t  uRecLostMax;       // ���֡������������Ϊ����              ----->>>�趨������еĶ�֡��������������Ϊ����
    uint8_t  uRecCrcMax;        // ���CRC���������������Ϊ����           ----->>>�趨������е�CRCУ���������������Ϊ����
    uint16_t uFailTimeOut;      // ��֡��ʱ���������ٴη�������          ----->>>�趨��֡���ٴη��͵ļ��ʱ�䣬��1msΪ��ʱ��λ
    uint16_t u1BTimeOut;        //1B��ʱʱ��
    uint16_t uSuccessDelay;     // һ֡�ɹ�����ʱ����ʱ���ѯ��һ֡        ----->>>�趨һ֡��ѯ�ɹ����ѯ��һ֡�ļ��ʱ�䣬��1msΪ��ʱ��λ,�趨��ֵ��Ҫ�������ֵС100mS���ҡ�

    uint32_t sSuccessTime;      // %�ڲ�%һ֡��ѯ�ɹ�ʱ��ʱ���¼

    #if(1==MASTER_ROUND_OVER)
    uint16_t uRoundTimeOut;     // һ����ѯѭ����ʱ��                      ----->>>�趨һ����ѯ���ڵ�ʱ�䣬��1sΪ��ʱ��λ
    uint8_t  uRoundOver;        // %�ڲ�%һ����ѯѭ������
    uint32_t uRoundStartTime;   // %�ڲ�%��ѭ����ʼʱ��
    #endif


    //uint8_t  uRecLostCount;  // -%�ڲ�%-��֡��������                    ----->�ڲ�ʹ�ã���ʼ��Ϊ0
    //uint8_t  uRecCrcCount;   // -%�ڲ�%-CRC�����������                 ----->�ڲ�ʹ�ã���ʼ��Ϊ0

    uint8_t  uPreAddr;          // -%�ڲ�%-�ѷ��Ͳ�ѯ�豸�ĵ�ַ            ----->�ڲ�ʹ�ã���ʼ��Ϊ0
    uint8_t  uPreFun;           // -%�ڲ�%-�ѷ��͵Ĺ�����                  ----->�ڲ�ʹ�ã���ʼ��Ϊ0
    //uint8_t  uSendLen;       // -%�ڲ�%-�����ֽ�����                    ----->�ڲ�ʹ�ã���ʼ��Ϊ0

}MODBUS_MASTER_T;
typedef struct
{
	char  cDeviceEsn[MAX_device+1][18];		// �����豸
    
	//uint8_t uEsnMark[MAX_device]; 		   //�����ַ��δ�����ַ�ı��
	//uint16_t	CRC;					 // �洢���ݵ�CRCУ��  
}DEVICE_ADDR_INFO_T;

//-----------------------------------------------------------------------------------------
extern DEVICE_ADDR_INFO_T uAllocation;

extern int16_t ComMasterWrite(MODBUS_MASTER_T *pMaster,uint8_t uAddr,uint8_t uCmd,uint16_t uRegAddr,uint16_t uQty,const uint16_t *pReg);
extern int16_t ComMasterRead(MODBUS_MASTER_T *pMaster,uint8_t uAddr,uint8_t uCmd,uint16_t uRegAddr,uint16_t uQty,uint8_t *pRecData,uint8_t *pSendData);
extern int16_t ComMasterReadDiscovery(MODBUS_MASTER_T *pMaster,uint8_t uAddr,uint8_t uCmd,uint16_t uRegAddr,uint16_t uQty,uint8_t *pRecData,uint8_t *pSendData);
extern int16_t ComMasterUpdata(MODBUS_MASTER_T *pMaster,uint8_t uAddr,uint8_t uCmd,uint16_t uRegAddr,uint16_t uQty,uint8_t *pRecData,uint8_t *pSendData);


#if(1==MASTER_ROUND_OVER)
extern int16_t ComMasterRoundOver(MODBUS_MASTER_T *pMaster);
#endif
extern int16_t TimeGapJudge(uint32_t sStart,uint32_t sNow,uint32_t uGap);
//-----------------------------------------------------------------------------------------


#endif
