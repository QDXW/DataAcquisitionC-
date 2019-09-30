/* ---------------------------------- File Description -------------------------------------
���ļ����ܡ������ļ�Ϊ MAX25L1606E  SpiFLASHоƬ�����ļ�
              MAX25L1606E �洢��Ϊ16M bit  ��Ϊ512��������ÿ��������4K byte  16������Ϊһ��block �ܹ���32��block
              ÿpage Ϊ256���ֽ�
              �������ٶȸߴ�86MHz
������ʱ�䡿��2017��4��11��
���ļ��汾����V1.0
�����������������ļ�����Ϊ
------------------------------------------------------------------------------------------*/
#include "DataFlash.h"
#include "stdio.h"

#include "ucos_ii.h"

#include "SPI.h"
#include "GPIO.h"
//======================================================================================
#define  SPI_NUM   SPI0

//------IO�ڳ�ʼ��-----
// DataFlashʹ�����ų�ʼ��
#define Data_Flash_CTRL()   GPIO_SetDir(0,2,Output) // CS�������ų�ʼ��
#define Data_Flash_EN()     GPIO_SetBit(0,2,Low)    // Data_Flashʹ��
#define Data_Flash_DIS()    GPIO_SetBit(0,2,High)   // Data_Flash��ֹ

// SPI IO�ڳ�ʼ��
#define  SPI_MISO_INIT()     Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 8, 0x91)  // MISO P58
#define  SPI_MOSI_INIT()     Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 9, 0x91)  // MOSI P59
#define  SPI_SCLK_INIT()     Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 6, 0x92)  // SCLK P44
//======================================================================================
//-------����--------
#define CMD_WREN      (0x06)  // write enable
#define CMD_WRDI      (0x04)  // write disable
#define CMD_WRSR      (0x01)  // write status register
#define CMD_RDID      (0x9F)  // read identification
#define CMD_RDSR      (0x05)  // read status register
#define CMD_READ      (0x03)  // read data
#define CMD_FASTREAD  (0x0B)  // fast read data
#define CMD_RDSFDP    (0x5A)  // read SFDP
#define CMD_RES       (0xAB)  // read electronic ID
#define CMD_REMS      (0x90)  // read electronic manufacturer & device ID
#define CMD_DREAD     (0x3B)  // Double Output Mode command
#define CMD_SE        (0x20)  // sector erase
#define CMD_BE        (0x52)  // or 0xD8  block erase
#define CMD_CE        (0x60)  // or 0xC7  chip erase
#define CMD_PP        (0x02)  // page program
#define CMD_RDSCUR    (0x2B)  // read security register
#define CMD_WRSCUR    (0x2F)  // write security register
#define CMD_ENSO      (0xB1)  // enter secured OTP
#define CMD_EXSO      (0xC1)  // exit secured OTP
#define CMD_DP        (0xB9)  // Deep power down
#define CMD_RDP       (0xAB)  // Release from deep power down

#define DUMMY_BYTE    (0x55)

#define FLASH_BUSY_TIMEOUT   300
//======================================================================================
OS_EVENT *pDFLock;  // DataFlash��д��
/****************************************************************************
* ��    �ƣ�Delay()
* ��    �ܣ���ʱ��
* ��ڲ�����
*           tms     ������ʱʱ�䣬���ٸ�10ms
* ���ڲ�������
* ��    ��: ��
****************************************************************************/
/*void DataFlashDelay(uint8_t tms)
{
    SOFT_TIMER_T timer;

    SoftTimerSet(&timer,tms,1); // ��ʱ��

    while(!SoftTimerExpired(&timer));

    SoftTimerStop(&timer);
}*/
/****************************************************************************
* ��    �ƣ� ShortDelay()
* ��    �ܣ�����ʱ��
* ��ڲ�����
*           uInDelay     ������ʱʱ�䣬���ٸ�
* ���ڲ�������
* ��    ��: ��
****************************************************************************/
void ShortDelay(uint16_t uInDelay)
{
    // 10000ԼΪ2.71mS������ԼΪ0.271uS
    volatile uint16_t uDelay;

    uDelay = uInDelay;

    while(uDelay--);
}
/****************************************************************************
* ��    �ƣ�DataFlash_Init()
* ��    �ܣ���ʼ��Data Flash��SPI0��
* ��ڲ�����
*           ��
* ���ڲ�������
* ��    ��: ��
****************************************************************************/
void DataFlashInit(void)
{
    uint8_t err;

    pDFLock = OSMutexCreate(0,&err);           // DataFlash��д��;

    Data_Flash_CTRL();

    SPI_MISO_INIT();
    SPI_MOSI_INIT();
    SPI_SCLK_INIT();

    SPI_Init(SPI_NUM);

    Data_Flash_DIS();
}

/****************************************************************************
* ��    �ƣ�DataFlash_Read_ID()
* ��    �ܣ���ȡData FlashоƬID��
* ��ڲ�����
*           ��
* ���ڲ�������
* ��    ��: ��
****************************************************************************/
void DataFlash_Read_ID(void)
{
    uint8_t flash_data[4]={0};

    Data_Flash_EN();
    __NOP();
    __NOP();

    flash_data[0] = CMD_RDID;
    flash_data[3] = SPI_Send(SPI_NUM,flash_data,1);
    __NOP();
    flash_data[3] = SPI_Read(SPI_NUM,flash_data,3);

    flash_data[3] = 9;
    Data_Flash_DIS();
    if(0xFF==flash_data[0] && 0xFF==flash_data[1] && 0xFF==flash_data[2])
    {
        printf("\nDataFlash �쳣\n");
    }
    else
    {
        printf("\nDataFlash ID:%02X %02X %02X\n",flash_data[0],flash_data[1],flash_data[2]);
    }
}
/****************************************************************************
* ��    �ƣ�DataFlash_WriteEnable()
* ��    �ܣ�Data Flashд����
* ��ڲ�����
*           ��
* ���ڲ�������
* ��    ��: ��
****************************************************************************/
void DataFlash_WriteEnable(void)
{
    uint8_t flash_data;

    Data_Flash_EN();
    __NOP();
    __NOP();

    flash_data = CMD_WREN;
    SPI_Send(SPI_NUM,&flash_data,1);

    Data_Flash_DIS();
}
/****************************************************************************
* ��    �ƣ�DataFlash_WriteDisable()
* ��    �ܣ�Data Flashд����
* ��ڲ�����
*           ��
* ���ڲ�������
* ��    ��: ��
****************************************************************************/
void DataFlash_WriteDisable(void)
{
    uint8_t flash_data;

    Data_Flash_EN();
    __NOP();
    __NOP();

    flash_data = CMD_WRDI;
    SPI_Send(SPI_NUM,&flash_data,1);

    Data_Flash_DIS();
}
/****************************************************************************
* ��    �ƣ�DataFlash_WaitBusy()
* ��    �ܣ�Data Flashд����
* ��ڲ�����
*           ��
* ���ڲ�������
* ��    ��: ��
****************************************************************************/
void DataFlash_WaitBusy(void)
{
    uint8_t flash_data;
    uint8_t flash_state;
    uint16_t retry=0;

    Data_Flash_EN();
    __NOP();
    __NOP();

    flash_data = CMD_RDSR;
    SPI_Send(SPI_NUM,&flash_data,1);

    do{
        SPI_Read(SPI_NUM,&flash_state,1);
        if(retry++>FLASH_BUSY_TIMEOUT)
            break;
    }while(flash_state&0x80);

    Data_Flash_DIS();
}
/****************************************************************************
* ��    �ƣ�DataFlash_Chip_Erase()
* ��    �ܣ�Data FlashƬ������
* ��ڲ�����
*           ��
* ���ڲ������ɹ�����1��ʧ�ܷ���0
* ��    ��: ��
****************************************************************************/
uint8_t DataFlash_Chip_Erase(void)
{
    uint8_t flash_data;
    uint8_t temp;
    uint8_t err;

    OSMutexPend(pDFLock,0,&err);//�����ź���

    DataFlash_WaitBusy();
    DataFlash_WriteEnable();

    Data_Flash_EN();
    __NOP();
    __NOP();

    flash_data = CMD_CE;
    temp = SPI_Send(SPI_NUM,&flash_data,1);

    Data_Flash_DIS();

    OSTimeDly(OS_TICKS_PER_SEC/20);// 50ms

    OSMutexPost(pDFLock);   //�ͷ��ź���

    return temp;
}
/****************************************************************************
* ��    �ƣ�DataFlash_Block_Erase()
* ��    �ܣ�Data Flash�������
* ��ڲ�����
*           blockAddr   ���ַ
* ���ڲ������ɹ�����1��ʧ�ܷ���0
* ��    ��: ��
****************************************************************************/
uint8_t DataFlash_Block_Erase(uint8_t blockAddr)
{
    uint8_t  flash_data[4]={0};
    uint32_t dstAdd;
    uint8_t temp;

    uint8_t err;

    OSMutexPend(pDFLock,0,&err);//�����ź���


    DataFlash_WaitBusy();
    DataFlash_WriteEnable();

    Data_Flash_EN();
    __NOP();
    __NOP();

    blockAddr++;
    // dstAdd = blockAddr * 16 * 4096
    dstAdd = ((uint32_t)blockAddr)<<16;//ÿ��block��16��������ÿ��������4KByte;
    dstAdd--;
    flash_data[0] = CMD_BE;
    flash_data[1] = (uint8_t)(dstAdd>>16);
    flash_data[2] = (uint8_t)(dstAdd>>8);
    flash_data[3] = (uint8_t)(dstAdd);

    temp = SPI_Send(SPI_NUM,flash_data,4);

    Data_Flash_DIS();

    OSTimeDly(OS_TICKS_PER_SEC/2);// 500ms

    OSMutexPost(pDFLock);   //�ͷ��ź���

    return temp;
}
/****************************************************************************
* ��    �ƣ�Data_Flash_Sector_Erase()
* ��    �ܣ�Data Flash����������
* ��ڲ�����
*           sectorAddr   ������ַ
* ���ڲ������ɹ�����1��ʧ�ܷ���0
* ��    ��: ��
****************************************************************************/
uint8_t DataFlash_Sector_Erase(uint32_t sectorAddr)
{
    uint8_t  flash_data[4]={0};
    uint32_t dstAdd;
    uint8_t temp;

    uint8_t err;

    OSMutexPend(pDFLock,0,&err);//�����ź���


    DataFlash_WaitBusy();
    DataFlash_WriteEnable();

    Data_Flash_EN();
    __NOP();
    __NOP();

    // dstAdd = blockAddr * 4096
    //dstAdd = ((uint32_t)sectorAddr)<<12;// ÿ��������4KByte;
    dstAdd = (sectorAddr / 4096) *4096;

    flash_data[0] = CMD_SE;
    flash_data[1] = (uint8_t)(dstAdd>>16);
    flash_data[2] = (uint8_t)(dstAdd>>8);
    flash_data[3] = (uint8_t)(dstAdd);

    temp = SPI_Send(SPI_NUM,flash_data,4);

    Data_Flash_DIS();

    OSTimeDly(OS_TICKS_PER_SEC/20);// 50ms

    OSMutexPost(pDFLock);   //�ͷ��ź���

    return temp;
}
/****************************************************************************
* ��    �ƣ�DataFlash_WritePP()
* ��    �ܣ�дData Flash��
* ��ڲ�����
*           dstAdd    Data Flash�洢��ַ
*           *ptr      ���洢���ݵ�ͷָ��
*           byteswrt  ���洢���ݵ��ֽ���
* ���ڲ������洢�ɹ����ش洢�ֽ�����ʧ�ܷ���0
* ��    ��: ��
****************************************************************************/
uint16_t DataFlash_WritePP(uint32_t dstAdd,uint8_t *ptr,uint16_t byteswrt)
{
    uint8_t  flash_data[4]={0};
    uint16_t temp;


    DataFlash_WaitBusy();
    DataFlash_WriteEnable();

    Data_Flash_EN();
    __NOP();
    __NOP();

    // dstAdd = blockAddr * 4096
    //dstAdd = ((uint32_t)sectorAddr)<<12;// ÿ��������4KByte;

    flash_data[0] = CMD_PP;
    flash_data[1] = (uint8_t)(dstAdd>>16);
    flash_data[2] = (uint8_t)(dstAdd>>8);
    flash_data[3] = (uint8_t)(dstAdd);

    temp = SPI_Send(SPI_NUM,flash_data,4);

    temp = SPI_Send(SPI_NUM,ptr,byteswrt);

    Data_Flash_DIS();

    __NOP();
    __NOP();

    return temp;
}
/****************************************************************************
* ��    �ƣ�DataFlash_Write()
* ��    �ܣ�дData Flash��
* ��ڲ�����
*           dstAdd    Data Flash�洢��ַ
*           *ptr      ���洢���ݵ�ͷָ��
*           byteswrt  ���洢���ݵ��ֽ���
* ���ڲ������洢�ɹ����ش洢�ֽ�����ʧ�ܷ���0
* ��    ��: ��
****************************************************************************/
uint16_t DataFlash_Write(uint32_t dstAdd,uint8_t *ptr,uint16_t byteswrt)
{
    //uint8_t  flash_data[4]={0};
    uint16_t temp;
    uint16_t write_bytes;

    uint8_t err;

    OSMutexPend(pDFLock,0,&err);//�����ź���


    //ShortDelay(20000);

    do
    {
        ShortDelay(25000);

        write_bytes = 0x100-(dstAdd&0x0000ff);
        if(byteswrt>write_bytes)  // д���ֽ�������һҳ(256bytes)����Ŀռ�
        {
            temp = DataFlash_WritePP(dstAdd,ptr,write_bytes);
            if(temp==write_bytes)
            {
                dstAdd += write_bytes;
                ptr += write_bytes;
                byteswrt -= write_bytes;
            }
            else
            {
                temp = 0;
                break;
            }
        }
        else
        {
            temp = DataFlash_WritePP(dstAdd,ptr,byteswrt);
            break;
        }
    }while(byteswrt>0);

    OSMutexPost(pDFLock);   //�ͷ��ź���

    return temp;
}
/****************************************************************************
* ��    �ƣ�DataFlash_Read()
* ��    �ܣ���Data Flash��
* ��ڲ�����
*           dstAdd    Data Flash�洢��ַ
*           *ptr      ����ȡ���ݵ�ͷָ��
*           byteswrt  ����ȡ���ݵ��ֽ���
* ���ڲ�������ȡ�ɹ����ض�ȡ�ֽ�����ʧ�ܷ���0
* ��    ��: ��
****************************************************************************/
uint16_t DataFlash_Read(uint32_t dstAdd,uint8_t *ptr,uint16_t bytesrd)
{
    uint8_t flash_data[5]={0};
    uint16_t temp;

    uint8_t err;

    OSMutexPend(pDFLock,0,&err);//�����ź���


    Data_Flash_EN();
    __NOP();
    __NOP();

    flash_data[0] = CMD_FASTREAD;//CMD_READ;//
    flash_data[1] = (uint8_t)(dstAdd>>16);
    flash_data[2] = (uint8_t)(dstAdd>>8);
    flash_data[3] = (uint8_t)(dstAdd);
    flash_data[4] = (uint8_t)(DUMMY_BYTE);

    temp = SPI_Send(SPI_NUM,flash_data,5);
    __NOP();
    __NOP();
    __NOP();

    temp = SPI_Read(SPI_NUM,ptr,bytesrd);
    __NOP();
    __NOP();

    Data_Flash_DIS();
    __NOP();

    OSMutexPost(pDFLock);   //�ͷ��ź���

    return temp;
}
