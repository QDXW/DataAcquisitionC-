#ifndef __GLOBAL_VAR_H__
#define __GLOBAL_VAR_H__

#include <stdint.h>
#include <stdio.h>
#include "tool.h"
#include "Update.h"
#include "log.h"

//----------------------------------------------------------
#define DEBUG_PRINT  (1)    // DEBUGģʽ�����Դ���0��ӡ�����Ϣ

#if (1==DEBUG_PRINT)
#define DEBUGOUT(...) printf(__VA_ARGS__)

#else
#define DEBUGOUT(...) ;
#endif
//==============================================================================
// ���������ͺ�
#define  Logger_C        (1)
#define  Logger_CP       (2)

#define  LOGGER_TYPE    Logger_CP
//==============================================================================

//==============================================================================
#define MAX_device      (10)  // ��������豸����
#define USE_485_DEBUG   (1)  // 1��ʹ������485����Ϊ���Դ��ڣ�0������485�ڽ���Ϊ����ͨѶ����
//==============================================================================
// ������EEPROM�еĴ洢��ַ
#define  EEP_LOGGER_INF_HEAD          0x00000044   // ���ɱ����洢��ʼ��ַ   eeprom��ʼ64�ֽڲ��ܴ洢
#define  EEP_LOGGER_DEVICE_ESN_HEAD   0x00000100   // �豸ESN�洢��ʼ��ַ
#define  EEP_LOGGER_DEVICE_INF_HEAD   0x00000300   // �豸��Ϣ�͵����Ϣ�洢��ʼ��ַ
#define  EEP_LOGGER_UPDATA_HEAD       0x00000500   // ���ɱ���������ʶ�洢��ʼ��ַ
#define  EEP_LOGGER_104_RECORD_HEAD   0x00000550   // IEC104��ʷ�������һ��DataFlash�洢��ַ
#define  EEP_LOGGER_LOG_LADDR_HEAD    0x00000560   // ��־���һ���洢��ַ
#define  EEP_LOGGER_LOG_HEAD          0x00000570   // ��־�洢��ʼ��ַ
//#define  EEP_TABLE_INFO_HEAD          0x00000580   // Ԥ�õ����Ϣ������
#define  EEP_DT1000_UPDATA_HEAD       0x00000580   // ���������ʶ�洢��ʼ��ַ

#define  EEP_LOGGER_DEVICE_SOFT_HEAD  0x00000600    //�豸����汾�Ŵ洢��ʼ��ַ
#define  EEP_ALARM_HEAD               0x00000800   // 2KB��ʼ�㣬�洢�澯��Ϣ
#define  EEP_DT1000UPDATA_SN          0x00000E00   //�洢�ɹ������--4���ֽ�
#define  EEP_DT1000UPDATALEN_SN       0x00000E10   //�洢�ɹ����ݳ���---8���ֽ�

//------------------------------------------------------------------------------
// ������DataFlash�еĴ洢��ַ
#define  DATAFLASH_UPDATA_HEAD        0x000000     // ������������DataFlash����ʼ��ַ  ռ��96KB
#define  DATAFLASH_POINT_HEAD         0x018000     // �������DataFlash����ʼ��ַ      ռ��8KB

#define  DATAFLASH_LOG_ONE            0x01A000     // ������־��һ������ʼ��ַ
#define  DATAFLASH_LOG_TWO            0x01C000     // ������־��һ������ʼ��ַ
#define  DATAFLASH_LOG_THREE          0x01E000     // ������־��һ������ʼ��ַ
#define  DATAFLASH_LOG_END            0x020000     // ������־��ŵĽ�����ַ
#define  DATAFLASH_LOGGER_INFO        0x022000     // ���ɻ�����Ϣ����
#define  DATAFLASH_DT1000_LOG         0x023000     //��ű����־
#define  DATAFLASH_DT1000_UPDATA_HEAD 0x050000     //���������������falsh����ʼ��ַռ��128KB����0x070000ΪA��,0x070000��0x090000ΪB��
#define  DATAFLASH_RECORD_HEAD        0x11A000     // 0x01E000��ʷ���ݴ����DataFlash����ʼ��ַ
#define  DATAFLASH_RECORD_END         0x7FFFFF     // ��ʷ���ݴ����DataFlash�Ľ�����ַ
//==============================================================================
// ��EEP��������
#define EEP_LOGGER_INF     1    // ��ȡ���ɻ�����Ϣ
#define EEP_DEVICE_SOUTH   2    // ��ȡ�����豸��Ϣ
#define EEP_DEVICE_ESN     3    // ��ȡ�����豸ESN
#define EEP_UPDATA         4    // ��ȡ������Ϣ
#define EEP_ALARM          5    // ��ȡ����澯��Ϣ
#define EEP_DEVICE_SOFT    6    // ��ȡ�����豸�����Ϣ
//#define EEP_TABLE_SEQ      7    // ��ȡԤ�õ����Ϣ������
#define EEP_DT1000_UPDATA  7   //��ȡ���������Ϣ

//==============================================================================
//==============================================================================
/*******************************************************************************
*���ɱ�����Ϣ
*��Ҫ�洢
*ռ�ÿռ䣺88�ֽ�
*�洢EEPROM��ʼ��ַ��0x00000050~0x000000AF
*******************************************************************************/
typedef struct
{
    uint16_t  inquire_interval_time;   // ��ѯ���ʱ��
    char      esn[20];                 // ����ESN��
    char      name[20];                // ��������
    char      model[20];               // �����ͺ�
    char      type[20];                // ��������
    uint16_t  server_port;             // �������˿�
    char      server_domain[34];       // ���������� �������30�ֽڣ�"����"Ȼ��\0һ��53�ֽ�

    uint8_t   ADDR;                    // �豸��ַ
    char      phonenum[11];            // �ֻ�����
    uint8_t   IP[4];                   // IP��ַ
    //uint8_t   reserve;                 // ռλ

    uint16_t  CRC;                     // �洢���ݵ�CRCУ��
}LOGGER_INF_T;
//==============================================================================

/*******************************************************************************
*�豸ESN����Ϣ
*��Ҫ�洢
*ռ�ÿռ䣺304�ֽ�
*�洢EEPROM��ʼ��ַ��0x000000B0~0x000001DF
*******************************************************************************/
typedef struct
{
    char  cDeviceEsn[MAX_device][20];       // �����豸
	uint8_t uEsnMark[MAX_device+1];			 //�����ַ��δ�����ַ�ı��

    //uint16_t  reserve;                 // Ԥ��
    uint16_t  CRC;                     // �洢���ݵ�CRCУ��
}LOGGER_DEVICE_ESN_T;

/*******************************************************************************
*�豸�豸�������Ϣ
*��Ҫ�洢
*ռ�ÿռ䣺304�ֽ�
*�洢EEPROM��ʼ��ַ��0x000000B0~0x000001DF
*******************************************************************************/
typedef struct
{
    uint8_t  cDeviceSoft[MAX_device][17];       // CplusΪ3̨�����豸
    uint8_t  uSoftMark[MAX_device];
    //uint16_t  reserve;                 // Ԥ��
    uint16_t  CRC;                     // �洢���ݵ�CRCУ��
}LOGGER_DEVICE_SOFT_T;

/*******************************************************************************
*�豸��Ϣ
*����Ҫ�洢
*ռ�ÿռ䣺20�ֽ�
*******************************************************************************/
#define  BIG_ENDIAN      0
#define  LITTLE_ENDIAN   1
typedef struct
{
    uint8_t   addr;               // �豸��ַ
    uint8_t   rel_num;            // ��Ե���--��Դ洢�������Ϣ��
    uint16_t  protocol_num;       // ���Ե���--����ϵͳ�·��ĵ���
    uint8_t   baud_rate;          // ͨѶ�����ʣ�1:2400��2:4800��3:9600��4:19200��5:38400��6:115200
    uint8_t   protocol_type;      // Э������,1:��ΪMODBUS��2����׼MODBUS
    uint8_t   big_little_endian;  // ���ݴ�С��
    uint8_t   reserve;            // Ԥ��ռλ
    uint16_t  yx_start_addr;      // ң����ʼ��ַ
    uint16_t  yc_start_addr;      // ң����ʼ��ַ
    uint16_t  yk_start_addr;      // ң����ʼ��ַ
    uint16_t  sd_start_addr;      // �����ʼ��ַ
    uint16_t  dd_start_addr;      // �����ʼ��ַ
}LOGGER_DEVICE_INF_T;

/*******************************************************************************
*�����Ϣ���ܡ�
*����Ҫ�洢
*ռ�ÿռ䣺12�ֽ�
*******************************************************************************/
typedef struct
{
    uint16_t  protocol_num;       // ����
    uint8_t   protocol_type;      // Э������
    uint8_t   alarm_sum;          // �澯������
    uint16_t  reserve2;           // Ԥ��
    uint16_t  mess_point_sum;     // ��Ϣ������

    uint32_t  flash_addr;         // ���Dataflash�洢��ַ
}LOGGER_PROTOCOL_INF_T;

/*******************************************************************************
*�豸��Ϣ
*��Ҫ�洢
*ռ�ÿռ䣺484�ֽ�
*�洢EEPROM��ʼ��ַ��0x000001E0~0x000003C3
*******************************************************************************/
typedef struct
{
    LOGGER_DEVICE_INF_T      device_inf[MAX_device]; // �����豸
    LOGGER_PROTOCOL_INF_T    protocol[MAX_device];   // �����豸

    uint8_t   reserve;           // ռλ
    uint8_t   device_sum;              // �豸����
    uint16_t  yx_sum;                  // ң������
    uint16_t  yc_sum;                  // ң������
    uint16_t  yk_sum;                  // ң������
    uint16_t  sd_sum;                  // �������
    uint16_t  dd_sum;                  // �������

    uint16_t  reserve2;           // ռλ
    uint16_t  CRC;                // �洢���ݵ�CRCУ��
}LOGGER_DEVICE_PROTOCOL_T;

//==============================================================================
//==============================================================================
/*******************************************************************************
*�������Ϣ
*����Ҫ�洢
*ռ�ÿռ䣺4�ֽ�
*******************************************************************************/
typedef union
{
    uint8_t type_type;
    struct
    {
        uint8_t  mess:4;  // ��Ϣ������ ң�أ�ң�⣬ң�ţ���㣬���
        uint8_t  data:4;  // ��������
    }type;
}LOG_MESS_TYPE_T;
typedef struct
{
    uint16_t         reg_addr;             // MODBUS�Ĵ�����ַ
    LOG_MESS_TYPE_T  reg_type;             // ��Ϣ������ ң�أ�ң�⣬ң�ţ���㣬���
    uint8_t          reg_count;            // ���ݳ���
}LOGGER_MODBUS_REG_T;



//==============================================================================
//==============================================================================
/*******************************************************************************
*IEC104�����Ϣ
*����Ҫ�洢
*ռ�ÿռ䣺4�ֽ�
*******************************************************************************/
// ң��
typedef struct
{
    uint16_t data;
    //uint16_t  reserve;
}IEC104_YX;

// ң��
typedef struct
{
    Float_Uint_Convert data;
}IEC104_YC;

// ң��
typedef struct
{
    uint16_t data;
    //uint16_t  reserve;
}IEC104_YK;

// ���
typedef struct
{
    uint16_t data;
    //uint16_t  reserve;
}IEC104_SD;

// ���
typedef struct
{
    Float_Uint_Convert data;
}IEC104_DD;

//==============================================================================
// ���������豸�澯
typedef struct
{
    uint16_t     mdbus_addr;          // �澯MODBUS��ַ
    uint16_t     alarm_value;         // �澯ֵ
}SOUTH_ALARM_T;

typedef struct
{
    uint16_t    dev_lost;        // �����豸���ߣ�һλ����һ̨�豸
    uint16_t    log_alarm;       // ���ɱ����쳣��һλ����һ���쳣״̬
    //uint16_t    dev_alarm_sum;   // ����澯������
}LOGGER_ALARM_T;
//==============================================================================
//==============================================================================

// ����״̬
enum  LOGGER_RUNNING_STATE
{
    RUNNING_EMPTY          = 0,    // �£�δ��վ
    //RUNNING_SEARCH_PREPARE = 1,    // ׼��������Ϊ�豸
    RUNNING_SEARCH_HW      = 1,    // ������Ϊ�豸
    RUNNING_SEARCH_END     = 2,    // ������Ϊ�豸����

    RUNNING_INPUT_START    = 3,    // ��������C5
    RUNNING_INPUT_SOUTH    = 4,    // ���������豸��ϢC4 92
    RUNNING_INPUT_GOLB     = 5,    // ����ȫ����ϢBB 88
    RUNNING_INPUT_TABLE    = 6,    // ������BB 89
    RUNNING_INPUT_104      = 7,    // �����豸104����ϢBB 8A
    //RUNNING_INPUT_INFO     = 4,    // ������Ϣ�����У�����ȫ����Ϣ���豸��Ϣ�������Ϣ�ȣ��ڵ�����Ϣ�����У������ѯ��ͣ

    RUNNING_WORK_READ      = 10,   // �Ѿ���վ��ɣ���������
};//LOGGER_RUNNING_STATE_E;

// ͨѶģ�����ӷ�����״̬
enum NORTH_RUNNING_STATE
{
    NORTH_DISCON    = 0,   // �������
    NORTH_CMERR     = 1,   // ������������û���յ�ģ�鷵��>
    NORTH_RDY       = 2,   // ģ��׼����
    NORTH_CONNECT   = 3,   // ģ��������
    NORTH_POWERDOWN = 4,   // ģ��ػ�
    NORTH_OK        = 5,   // ���ӵ����������Ѿ��ϱ�������Ϣ����ɶ�ʱ
};

//------------------------------------------------
// ���й���
enum LOGGER_RUNNING_ERR
{
    err_power_off = 0x0001,   // �ϵ�
};//LOGGER_RUNNING_ERR_E;


//==============================================================================
// ����״̬������Ҫ�洢
typedef struct
{
    uint8_t  run_status;      // ����״̬
    uint8_t  update;          // �л�������ģʽ��0x0A������������0xA0��Զ������
    uint16_t err;             // ���ϱ���洢
    uint16_t err_lost;        // �����豸��ʧ
    uint16_t uDevExist;       // ������Ч�豸��ǣ�һλһ��
    uint8_t  IP[4];           // ����IP��ַ
    char     IMSI[15];        // �����ƶ�̨�豸��ʾ��IMSI��
    uint8_t  north_status;    // ��������״̬
    uint8_t  uGPS;            // GPS״̬��1:������GPS;0:û��������GPS
    uint8_t  uCom;            // ����ͨѶ״̬��bit��2�������ݣ�bit:1�յ����ݣ�bit3����ʱ�У�
    uint16_t uCMType;        // ͨ��ģ������
    uint8_t  uCSQ;           // ���߹���
    //uint8_t  uDt1000_update;  //�������״̬
    uint8_t  uFileIO_status;  // �����ļ����뵼��״̬

}LOGGER_RUNNING_T;
//==============================================================================
typedef struct
{
    uint8_t   updata_mark;      // ������־0xAA������0xBB�ع���0x55��
    uint8_t   version[17];      // ����汾
    uint8_t   uDevAddr;        //�豸��ַ
    uint8_t   uData;           // ����
    uint32_t  nDataLen;
    uint16_t  frame_sum;        // ������֡����256�ֽ�һ֡
    uint16_t  CRC;              // ����CRCУ��
}DT1000UPDATA_MARK_T;

//==============================================================================
//==============================================================================
//==============================================================================
// �������ݣ����洢
extern LOGGER_RUNNING_T          g_LoggerRun;    // �������в���
//-----------------------------------------------------------------
// �������ݣ��洢���ڲ�eeprom
extern LOGGER_DEVICE_PROTOCOL_T  g_DeviceSouth;  // �����豸�͵����Ϣ
extern LOGGER_INF_T              g_LoggerInfo;    // ������Ϣ
extern LOGGER_DEVICE_ESN_T       g_DeviceEsn;    // �����豸ESN
extern LOGGER_DEVICE_SOFT_T      g_DeviceSoft;    // �����豸����汾��



// �������ݣ��洢���ⲿDataFlash
extern LOGGER_MODBUS_REG_T       *g_pRegPoint[MAX_device]; // �����Ϣ

// ����澯��Ϣ���ݶ��洢���ڲ�eeprom
extern SOUTH_ALARM_T             *g_psSouthAlarm[MAX_device];
extern SOUTH_ALARM_T             *g_psSouthAlarmCopy[MAX_device];
extern LOGGER_ALARM_T            g_LoggerAlarm;

// ������Ϣ���洢��ÿ��eeprom
extern UPDATA_MARK_T             g_LoggerUpdate;
extern DT1000UPDATA_MARK_T        g_DT1000Updata;

//extern uint16_t g_PerSetTableResq;             //������
extern uint8_t SouthSwich;
extern uint8_t SdContinuousAddr;
extern uint32_t g_South_Action_Newtime;

//==============================================================================
extern uint8_t GetVerType(void);
extern uint8_t GetVerS1(void);
extern uint8_t GetVerS2(void);
extern uint16_t GetVerS3(void);
//extern void PrintThisInfo(void);
extern void SetLoggerDefInfo(void);
extern uint16_t ReadEepData(uint8_t t);
extern uint16_t SaveEepData(uint8_t t);
extern void AllReset(uint8_t uSave);
extern void Reboot(void);

#endif
