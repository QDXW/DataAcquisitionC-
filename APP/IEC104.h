#ifndef __IEC104_H_
#define __IEC104_H_

#include "ucos_ii.h"

#include <stdint.h>
//=================================================================

#define TaskIec104ProcessStkSize   256   // ��ѯ�����ջ��С
extern OS_STK TaskIec104ProcessStk[TaskIec104ProcessStkSize];	      // ���������ջ��С

//=================================================================
//--------------------------------------
// ���ӷ���Ĺ�����Ϣ
#define  M_SP_NA_1         0x01  //1    // ������Ϣ
#define  M_DP_NA_1         0x03  //3    // ˫����Ϣ
#define  M_ME_NA_1         0x09  //9    // ����ֵ����һ��ֵ
#define  M_ME_NC_1         0x0D  //13   // ����ֵ���̸�����
#define  M_IT_NA_1         0x0F  //15   // �ۼ�ֵ
#define  M_SP_TB_1         0x1E  //30   // ��ʱ��CP56Time2a�ĵ�����Ϣ
#define  M_DP_TB_1         0x1F  //31   // ��ʱ��CP56Time2a��˫����Ϣ
#define  M_ME_TF_1         0x24  //36   // ��ʱ��CP56Time2a�Ĳ���ֵ���̸�����
#define  M_IT_TB_1         0x25  //37   // ��ʱ��CP56Time2a���ۼ���
//-----------------------------------------
// ���Ʒ���Ĺ�����Ϣ
#define  C_SC_NA_1         0x2D  //45   // ������Ϣ
#define  C_DC_NA_1         0x2E  //46   // ˫����Ϣ
#define  C_SE_NA_1         0x30  //48   // �趨�����һ��ֵ
#define  C_SE_NB_1         0x31  //49   // �趨�����Ȼ�ֵ
#define  C_SE_NC_1         0x32  //50   // �趨����̸�����
//------------------------------------------
// ���Ʒ����ϵͳ��Ϣ
#define  C_IC_NA_1         0x64  //100  // ���ٻ�����
#define  C_CI_NA_1         0x65  //101  // ������ٻ�����
#define  C_CS_NA_1         0x67  //103  // ʱ��ͬ������

//#define  P_DT1000_UPDATA   0xF0  //179   //�������
#define  P_FILE_INOUT      0xF0  //179   //�ļ����뵼��

#define  P_UPDATA          0xB4  //180  // ����
#define  P_SET_IP          0xB5  //181  // �豸IP����
#define  P_MODBUS_ENDIAN   0xB6  //182  // MODBUS��С������
#define  P_MODBUS_BAUD     0xB7  //183  // MODBUS����������
#define  P_ADDR            0xB8  //184  // �豸������ַ����
#define  P_VERSION         0xB9  //185  // �汾��ѯ
#define  P_ROLLBAOCK       0xBA  //186  // �汾�ع�
#define  P_INPUT_TABLE     0xBB  //187  // �����
#define  P_REPORT_NUM      0xBC  //188  // �ϱ���������
#define  P_INIT_FINISH     0xBD  //189  // �豸��ʼ���Ƿ����
#define  P_TIME_COLLECT    0xBE  //190  // ��ʱ�����ٻ�
#define  P_ERR_PROCESS     0xBF  //191  // �쳣����
#define  P_DEV_INFO        0xC0  //192  // �豸������Ϣ
#define  P_STATION_INFO    0xC1  //193  // վ�������Ϣ
#define  P_SERVICE_INFO    0xC2  //194  // ƽ̨������Ϣ
#define  P_CREATE_STATION  0xC3  //195  // ��վָ��
#define  P_SOUTH_INFO      0xC4  //196  // �����豸��Ϣ
#define  P_TABLE           0xC5  //197  // ����ָ��
#define  P_COM_MODE        0xC6  //198  // ͨ�ŷ�ʽ
#define  P_HW_INFO         0xC7  //199  // ��Ϊ�豸��Ϣ
#define  P_SERVICE_IP      0xC8  //200  // ������IP/����
#define  P_SERVICE_PORT    0xC9  //201  // �������˿�
#define  P_LOG             0xCA  //202  // ��־����
#define  P_LOCATION        0xCB  //203  // ��γ��
#define  P_SOUTH_STATUS    0xCC  //204  // �����豸״̬
#define  P_CL_ESN          0xD0  //208  // ����ESN����      CL=CLEAN LOGGER
#define  P_CL_TYPE         0xD1  //209  // ������������
#define  P_CL_MODEL        0xD2  //210  // �����ͺ�����
#define  P_CL_NAME         0xD3  //211  // �����豸��������
#define  P_HW_ESN          0xD4  //212  // ������Ϊ�豸ESN����
#define  P_HW_DEL          0xD5  //213  // ������Ϊ�豸ɾ��
#define  P_SIM_ID          0xD6  //214  // SIM��ID�ϱ�
#define  P_WL_QUALITY      0xD7  //215  // ���߹����ϱ�
#define  P_MAX_DEVICE      0xD8  //216  // �����豸���ֵ�ϱ�
#define  p_SD              0xD9  //ƽ̨���
#define  P_CL_RESET        0xFA  //250  // �ָ���������
#define  P_CL_REBOOT       0xFB  //251  // �豸��λ   ---1.1.7 ���� ---����
//------------------------------------------
//�ӹ�����subfunction
#define  S_FILE_IMPORT   0x01
#define  S_FILE_EXPORT   0x33


//------------------------------------------
// ����ԭ��
#define   R_ROUND              0x01   // ���ڡ�ѭ��
#define   R_BACK_SCAN          0x02   // ����ɨ��
#define   R_BURST              0x03   // ͻ�����Է��ϴ�
#define   R_INIT               0x04   // ��ʼ��
#define   R_ASK                0x05   // ���������
#define   R_ACTIVE             0x06   // ����
#define   R_ACTIVE_ACK         0x07   // ����ȷ��
#define   R_ACTIVE_STOP        0x08   // ֹͣ����
#define   R_ACTIVE_STOP_ACK    0x09   // ֹͣ����ȷ��
#define   R_ACTIVE_END         0x0A   // �������
#define   R_COLLECT_ACK        0x14   // ��Ӧ���ٻ�

#define   R_TRANS_START        0x80   // ��ʼ����
#define   R_TRANS_START_ACK    0x81   // ��ʼ����ȷ��
#define   R_DATA_TRANS         0x82   // ���ݴ���
#define   R_DATA_RETRY         0x83   // �����ط�
#define   R_TRANS_STOP         0x84   // ֹͣ����
#define   R_TRANS_STOP_ACK     0x85   // ֹͣ����ȷ��
#define   R_TRANS_FINISH       0x86   // �������
#define   R_TRANS_FINISH_ACK   0x87   // �������ȷ��
#define   R_INPUT_GLO_INFO     0x88   // ����ȫ����Ϣ
#define   R_INPUT_TYPE_INFO    0x89   // �����豸������Ϣ
#define   R_INPUT_DEV_INFO     0x8A   // �����豸��Ϣ
#define   R_STATION_START      0x8B   // ��վ����
#define   R_TABLE_START        0x8C   // ��������
#define   R_STOP               0x8D   // ֹͣ
#define   R_STOP_ACK           0x8E   // ֹͣȷ��
#define   R_RECOLLECT_END      0x8F   // ���ɽ���
#define   R_INQUIRE            0x90   // ��ѯ
#define   R_INQUIRE_SUC        0x91   // ��ѯ�ɹ�
#define   R_SETTING            0x92   // ����
#define   R_SET_SUC            0x93   // ���óɹ�
#define   R_INFO_REPORT        0x94   // ��Ϣ�ϱ�
#define   R_INFO_REPORT_END    0x95   // �ϱ�����
//#define   0x96   //
//#define   0x97   //
#define   R_INPUT_GLO_ACK      0x98   // ����ȫ����Ϣȷ��
#define   R_INPUT_TYPE_ACK     0x99   // �����豸������Ϣȷ��
#define   R_INPUT_DEV_ACK      0x9A   // �����豸��Ϣȷ��
#define   R_INTERCHANGE_ERR    0xEE   // �����쳣

//==================================================
//==================================================
// IEC104֡��ʼΪ0x68
#define IEC104_HEAD			    0x68
// U֡��S֡
#define IEC104_U_MAK			0x03
#define IEC104_U_STARTDT		0x04
#define IEC104_U_STARTDT_ACK	0x08
#define IEC104_U_STOPDT		    0x10
#define IEC104_U_STOPDT_ACK	    0x20
#define IEC104_U_TESTFR		    0x40
#define IEC104_U_TESTFR_ACK	    0x80
#define IEC104_S_MAK			0x01

//==================================================
//==================================================
// ��Ϣ������
#define TYPE_YX      0x02    // ң��
#define TYPE_YC      0x01    // ң��
#define TYPE_YK      0x04    // ң��
#define TYPE_SD      0x07    // ���
#define TYPE_DD      0x03    // ���
#define TYPE_GJ      0x09    // �澯

#define T_UINT16     0x01    // �޷���16λ����
#define T_STRING     0x02    // �ַ���
#define T_UINT32     0x03    // �޷���32λ����
#define T_INT16      0x04    // �з���16λ����
#define T_INT32      0x05    // �з���32λ����
#define T_NULLDATA   0x06    // ��ֵ
#define T_EPOCHTIME  0x07    // ʱ������
#define T_BIT        0x08    // Ϊλ����
#define T_FLOAT      0x09    // Ϊ��������
//==================================================
//==================================================
typedef union
{
    struct
    {
        uint8_t start;   		// ������
        uint8_t len;     		// ����
        uint8_t SseqL;   		// �������е�λ
        uint8_t SseqH;   		// �������и�λ
        uint8_t RseqL;   		// �������е�λ
        uint8_t RseqH;   		// �������и�λ
        uint8_t type;    		// ���ͱ�ʶ
        uint8_t limit;   		// �ɱ�ṹ�޶���
        uint8_t reasonL; 		// ����ԭ���λ
        uint8_t reasonH; 		// ����ԭ���λ
        uint8_t addrL;   		// ������ַ��λ
        uint8_t addrH;   		// ������ַ��λ
        uint8_t maddrL;  		// ��Ϣ���ַ��λ
        uint8_t maddrM;  		// ��Ϣ���ַ��λ
        uint8_t maddrH;  		// ��Ϣ���ַ��λ
        uint8_t data[240]; 		// ����
    }format;
    uint8_t buff[255];
}IEC_FORMAT_T;

typedef struct
{
    uint8_t      FrameCommand;    // ֡��ʽ I֡��U֡��S֡
    uint8_t      SendBytes;       // �����ֽ���
    IEC_FORMAT_T send;            // uint8_t   send_buff[255];  // ���͵���������
    IEC_FORMAT_T recv;            // uint8_t   rec_buff[255];   // ���յ���������
}IEC104_MAIN_T;
//========================================================================
// ͳ�Ƹ�������ң�ţ�ң������
typedef struct
{
    uint16_t uYxSum;
    uint16_t uYcSum;
    uint16_t uYkSum;
    uint16_t uSdSum;
}IEC104_COUNT_T;
//========================================================================
extern IEC104_COUNT_T g_sIecPointCount[];  // �����Ϣ��ͳ��
//========================================================================

extern uint8_t  *IEC104_DATA_YX;   // IEC104����-ң��ָ�룬��̬����ռ�
extern uint32_t *IEC104_DATA_YC;   // IEC104����-ң��ָ�룬��̬����ռ�
extern uint8_t  *IEC104_DATA_YK;   // IEC104����-ң��ָ�룬��̬����ռ�
extern uint32_t *IEC104_DATA_SD;   // IEC104����-���ָ�룬��̬����ռ�
extern uint32_t *IEC104_DATA_DD;   // IEC104����-���ָ�룬��̬����ռ�
//extern static uint8_t  s_uSouthReadSd; //��������
extern IEC104_MAIN_T g_sIEC;
extern uint16_t      g_IecRecSeq;          // ��������
extern uint16_t      g_IecSendSeq;         // ��������
//==================================================
//������������
#define COLLECT_NULL           0   // ������
#define COLLECT_START          1   // ���ٿ�ʼ
#define COLLECT_YX             2   // ����ң������
#define COLLECT_YC             3   // ����ң������
#define COLLECT_END            9   // ���ٽ���
#define COLLECT_DD_START       10  // �������
#define COLLECT_DD_END         11  // ������ٽ���

//�������ڲ���
#define SUBCOLLECT_NULL        0   // �޲���

#define SUBCOLLECT_START       21  // ���ɿ�ʼ
#define SUBCOLLECT_YX          22  // ���Ͳ���ң������
#define SUBCOLLECT_YC          23  // ����ң������
#define SUBCOLLECT_END         25  // �����������ٻ�
#define SUBCOLLECT_CHECK	   26  // ƽ̨��ѯ�Ƿ񻹴�����ʷ����
#define SUBCOLLECT_END_CONFIRM 27  // ���ɽ���


#define NORTH_CMD_READSD  (5)


typedef struct
{
    uint8_t running;     // IEC104״̬
    uint8_t collect;     // ����״̬
    uint8_t dd_collect;  // �������
    uint8_t uRelAddr;    // ���ϱ��������ݵ�����豸��ַ
    uint8_t subcollect;  // ����״̬
    uint8_t logext;
} IEC_RUNNING_T;

extern IEC_RUNNING_T g_sIecRun;
extern IEC_RUNNING_T iec_subcollect_run;
//====================================================================================================
//====================================================================================================
//====================================================================================================
extern void TaskIec104Process(void *p);
//extern void Iec104CreateFrameI(uint8_t type,uint8_t limit,uint16_t reason,uint8_t bytes,IEC104_MAIN_T *pA);
extern void IecCreateFrameI(uint8_t type,uint8_t limit,uint16_t reason,uint8_t bytes,IEC_FORMAT_T *pA);
extern void IecReportLogInfo(uint8_t uReason);

//====================================================================================================
//====================================================================================================
//====================================================================================================

//======================================================================
extern void ReportCtrlClear(uint8_t uCtrl);
// �����ϱ�����
extern void ReportCtrlSet(uint8_t uCtrl);
// ���ϱ�����
extern uint8_t ReportCtrlRead(uint8_t uCtrl);
#define REPORT_HW_DEVICE       0x01
#define REPORT_OTHERS          0x02
#define REPORT_HW_SOFT         0X04
//extern uint8_t  g_uNextFrame; // ȫ�ֱ�������IEC104.c�б�ǡ�
//extern uint8_t uPdateMark;     //������־�����ڸ�λ���ж��Ƿ����

//======================================================================

#endif
