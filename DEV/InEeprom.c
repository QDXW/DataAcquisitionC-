#include "chip.h"
#include "eeprom.h"
#include "InEeprom.h"
#include "CRC16.h"
#include "GlobalVar.h"


#define   DISABLE_IRQ  0   // 1����������ʱ���жϣ�0����������ʱ�����ж�
/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* EEPROM address location */
#define EEPROM_ADDRESS      0x00000100

void InEEPROM_Write(uint32_t dstAdd, uint8_t *ptr, uint32_t byteswrt)
{
    Chip_EEPROM_Write(dstAdd, ptr, byteswrt);
}

void InEEPROM_Read(uint32_t srcAdd, uint8_t *ptr, uint32_t bytesrd)
{
    Chip_EEPROM_Read(srcAdd, ptr, bytesrd);
}

/******************************************************************************
* ��    �ƣ�EEP_Save_data()
* ��    �ܣ��洢���ݵ��ڲ�EEPROM
* ��ڲ�����
*           addr      eeprom��ַ
            data      ����������ָ��
            crc       NULL������CRCУ�飬��������CRCУ��ֵָ��

* ���ڲ�������crcУ��,У��ɹ�����1 ��ʧ�ܷ���0���޷��ض�ȡ�ֽ���
* ��    ��:
           EepReadData(EEP_LOGGER_INF_HEAD,(uint8_t *)&Logger_inf,sizeof(Logger_inf),&Logger_inf.CRC);
           EepReadData(EEP_LOGGER_INF_HEAD,(uint8_t *)&Logger_inf,sizeof(Logger_inf),NULL)
******************************************************************************/
uint16_t EepReadData(uint32_t addr,uint8_t *data,uint16_t bytes,uint16_t *crc)//,uint8_t check)
{
    uint16_t crc_r;
    if(crc)
    {
        Chip_EEPROM_Read(addr, data, bytes);
        crc_r =  CRC16(data,bytes-2);

        if(crc_r == *crc)
        {
            return EEP_OK;
        }
        else
        {
            return EEP_ERR;
        }
    }
    else
    {
        return Chip_EEPROM_Read(addr, data, bytes);
    }
}

/******************************************************************************
* ��    �ƣ�EEP_Save_data()
* ��    �ܣ��洢���ݵ��ڲ�EEPROM
* ��ڲ�����
*           addr      eeprom��ַ
            data      ����������ָ��
            crc       NULL������CRCУ�飬��������CRCУ��ֵָ��

* ���ڲ�������CRCУ��0����������ʧ�ܣ�1���������ݳɹ����������ش��������ֽ���
* ��    ��:E
           EepSavedata(EEP_LOGGER_INF_HEAD,(uint8_t *)&Logger_inf,sizeof(Logger_inf),&Logger_inf.CRC);
           EepSavedata(EEP_LOGGER_INF_HEAD,(uint8_t *)&Logger_inf,sizeof(Logger_inf),NULL)
******************************************************************************/
uint16_t EepSavedata(uint32_t addr,uint8_t *data,uint16_t bytes,uint16_t *crc)//,uint8_t check)
{
    uint8_t write_count=0;  // д�����

    if(crc)
    {
        *crc = CRC16(data,bytes-2);

        #if(1==DISABLE_IRQ)
        __disable_irq();
        #endif

        while(write_count<3)
        {
            Chip_EEPROM_Write(addr, data, bytes);//&G_status.longitude

            if(EepReadData(addr,data,bytes,crc))
            {
                #if(1==DISABLE_IRQ)
                __enable_irq();
                #endif
                return EEP_OK;
            }
            else
            {
                write_count++;
                continue;
            }
        }

        #if(1==DISABLE_IRQ)
        __enable_irq();
        #endif
    }
    else
    {
        return Chip_EEPROM_Write(addr, (uint8_t *)data, bytes);
    }
    return EEP_ERR;
}
//==========================================================================

