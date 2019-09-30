#ifndef __UPDATE_H_
#define __UPDATE_H_

#include <stdint.h>

#define EEPROM_UPDATA_MARK_ADDRESS  0x00000500  // ��eeprom��д������־

// ��������
typedef struct
{
    uint16_t  frame_sum;       // ������֡����256�ֽ�һ֡
    uint8_t   updata_mark;     // ������־0xAA������0xBB�ع���0x55��
    uint8_t   rollback_allow;  // �ع�����0xBB�лع����ݣ�0x55�޻ع�����
    //uint32_t  flash_addr;      // ����洢����ʼflash��ַ
    uint8_t   a_version[3];    // A�����汾
    uint8_t   b_version[3];    // B�����汾
    uint8_t   sucess;          // ��ת�ɹ�0xAA:�ɹ���0x55ʧ��
    uint8_t   side;            // 0xAA:������A�棻0xBB��������B��
    uint8_t   side_tobe;       // ����Ŀ��λ�ã�0xAA:������A�棻0xBB��������B��
    uint8_t   reserve;         // Ԥ��
    uint16_t  CRC;             // ����CRCУ��
}UPDATA_MARK_T;


#define UPDATE_ENTER_NORMAL    1
#define UPDATE_ENTER_ALL       2
#define UPDATE_ENTER_SINGEL    3
#define UPDATE_LEAVE           0

extern  uint8_t Update(void);

#endif
