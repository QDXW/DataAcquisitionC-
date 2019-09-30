
#include "chip.h"

#include "KEY_SCAN.h"

#include "SWITCH_SCAN.h"
#include "GPIO.h"
#include "GlobalVar.h"
#include "InEEPROM.h"
//==============================================================================
//�˿ڶ���
//==============================================================================
//#define LOAD_165_L              GPIO1PIN_P3B_PUT(0)    // 74HC165 �ź����ݼ��� PL
//#define LOAD_165_H              GPIO1PIN_P3B_PUT(1)    // 74HC165 �ź����ݼ��� PL  LDB6
//#define CLK_165_L               GPIO1PIN_P3C_PUT(0)    // 74HC165 ��ʱ��  CLK
//#define CLK_165_H               GPIO1PIN_P3C_PUT(1)    // 74HC165 ��ʱ��  CLK  LDB7
//#define INPUT_165               GPIO1PIN_P3A_GET       // 74HC165 ��������     LDB5
//#define KEY_Enable              GPIO1PIN_P3A_INIT(INPUT_LOW)  // ����165����
//#define KEY_Disable             GPIO1PIN_P3A_INIT(OUTPUT)// ��ֹ165����

#define  Serial165LoadIO(x)        GPIO_SetBit(2,17,x)
#define  Serial165ClkIO(x)         GPIO_SetBit(1,30,x)
#define  Serial165InIO()           (Chip_GPIO_ReadPortBit(LPC_GPIO,2,18))


#define S01_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,2,22))
#define S02_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,2,21))
#define S03_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,2,20))
#define S04_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,1, 3))
#define S05_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,0,14))
#define S06_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,0,13))
#define S07_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,1,30))
#define S08_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,0,12))
#define S09_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,1,11))
#define S10_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,0,11))
#define S11_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,0,22))
#define S12_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,2,19))
#define S13_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,1,21))
#define S14_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,2,18))
#define S15_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,2,17))
#define S16_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,2,16))

//----------------��������---------------------
SWITCHINT  SwitchDown =0;  // ���´���
SWITCHINT  SwitchUp   =0;  // �ɿ�����
SWITCHINT  SwitchOn   =0;  // ������ס


static uint16_t Read165(uint8_t n);

// LED�ȺͰ���IO�ڳ�ʼ��
void SWITCH_IO_Init(void)
{
    /*// S01 - S16
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 22, 0x90);  // S01
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 21, 0x90);  // S02
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 20, 0x90);  // S03
    Chip_IOCON_PinMuxSet(LPC_IOCON, 1,  3, 0x90);  // S04
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 14, 0x91);  // S05
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 13, 0x91);  // S06
    Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 30, 0x90);  // S07
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 12, 0x91);  // S08
    Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 11, 0x90);  // S09
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 11, 0x91);  // S10
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 22, 0x90);  // S11
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 19, 0x90);  // S12
    Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 21, 0x90);  // S13
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 18, 0x90);  // S14
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 17, 0x90);  // S15
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 16, 0x90);  // S16

    // S01 - S16
    GPIO_SetDir(2, 22, Input);  // S01
    GPIO_SetDir(2, 21, Input);  // S02
    GPIO_SetDir(2, 20, Input);  // S03
    GPIO_SetDir(1,  3, Input);  // S04
    GPIO_SetDir(0, 14, Input);  // S05
    GPIO_SetDir(0, 13, Input);  // S06
    GPIO_SetDir(1, 30, Input);  // S07
    GPIO_SetDir(0, 12, Input);  // S08
    GPIO_SetDir(1, 11, Input);  // S09
    GPIO_SetDir(0, 11, Input);  // S10
    GPIO_SetDir(0, 22, Input);  // S11
    GPIO_SetDir(2, 19, Input);  // S12
    GPIO_SetDir(1, 21, Input);  // S13
    GPIO_SetDir(2, 18, Input);  // S14
    GPIO_SetDir(2, 17, Input);  // S15
    GPIO_SetDir(2, 16, Input);  // S16
    */

    GPIO_SetDir(2, 17, Output);  // LOAD
    GPIO_SetDir(2, 18, Input);   // DATA
    GPIO_SetDir(1, 30, Output);  // CLK

    Serial165LoadIO(1);
    Serial165LoadIO(0);
    Serial165LoadIO(1);

    Serial165ClkIO(1);
    Serial165ClkIO(0);
    Serial165ClkIO(1);
}

/*******************************************************************************
* ���ƣ�����ɨ���ӳ���
* ���룺��
* ���������ֵ���洢��ȫ�ֱ�����
* �漰������KeyDown,Keyup,Keyon
*******************************************************************************/
void SwitchScan(void)
{
    /*
    uint8_t ReadData = P2IN^0xff;   // ����ֵ
    //Trg = ReadData & (ReadData^Cont);
    Trg = ReadData&(~Cont);               // �õ����´���ֵ
    //Release=  (ReadData ^ Trg ^ Cont);
    Release = Cont &(~ReadData);          // �õ��ͷŴ���ֵ
    Cont = ReadData;                      // �õ�����δ�ͷŵļ�ֵ
    */
    //���²��ֲο���http://blog.csdn.net/xuechaojie/article/details/6761772
    static SWITCHINT   PreKey=0x00;      // ��¼�ϴ�KeyScan()��ȡ��IO�ڼ�ֵ
    SWITCHINT           CurrReadKey;      // ��¼����KeyScan()��ȡ��IO�ڼ�ֵ
    SWITCHINT           NowKey;           // ��¼���ξ���������������Ч����ֵ

    static uint8_t  uDelay;
    //--------------------------------------------------------------------------
    /*����Ҫ�������������ɿ���������ȡ�������ֵ�ע��
    static uint8_t LongKey=0;         // ��������
    static uint8_t X=0;               // �°��µĵ���λ
    static uint16_t  LongTime=0;        // ��������ʱ����¼��ѭ������
    */
    //--------------------------------------------------------------------------
    //if(0==GetRunKeyBoardOn())
    {
       // return;
    }
    if(KeyOn & Key_button)
    {
        SetRunKeyBoardOn(1);
    }
    else
    {
        SetRunKeyBoardOn(0);
        return;
    }


    CurrReadKey = Read165(2);   // ����Ƭ74HC165����

    if(0xFFFF==CurrReadKey)  // ���̱��Ƴ�
    {
        if(0xFFFF!=PreKey)
        {
            /*if(SET_OK==SetStateLastSwitch(SwitchOn))
            {
                SaveManual();
            }*/

            PreKey = 0xFFFF;
        }
        uDelay = 0;
        return;
    }
    else
    {
        if(uDelay<10)
        {
            uDelay++;
            return;
        }
    }

    /*
    #define FIRE_IO              GPIO1PIN_P0A_GET   //(KeyOn & BIT6)   // �����ź�����˿�
    #define Feedback_I_ON        GPIO1PIN_NP02_GET  // I·��բ
    #define Feedback_II_ON       GPIO1PIN_NP04_GET  // II·��բ
    #define Feedback_OFF         GPIO1PIN_NP00_GET  // ��բ
    */
/*
    CurrReadKey = S16_INPUT;
    CurrReadKey = CurrReadKey<<1 | S15_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S14_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S13_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S12_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S11_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S10_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S09_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S08_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S07_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S06_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S05_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S04_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S03_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S02_INPUT;   //
    CurrReadKey = CurrReadKey<<1 | S01_INPUT;   //
*/

    /***************************************************************************
    * ����ԭ��ܼ򵥣�
    * ����ϴ�LastReadKey�͵�ǰCurReadKey��ȡ�ļ�ֵ��Ϊ1����ô��ǰ��Ч��ֵCurrKeyһ��Ϊ1
    * ����ϴκ͵�ǰ��ȡ�ļ�ֵ��һ���Ļ��������ϴ���Ч��ֵLastKey����һ�¡�
    * ͨ�����ַ�ʽ����������������ʱ�����С��keyscan()��ʱɨ�����ڣ��������ִ���
    ***************************************************************************/
    NowKey = (CurrReadKey & PreKey) | (SwitchOn & (CurrReadKey ^ PreKey));

    //��¼�������¼��ͷ�
    SwitchDown = (~SwitchOn) & NowKey;    // ���°���������λ
    SwitchUp   = SwitchOn & (~NowKey);    // �ɿ�����������λ

    //-------------------------�������������ɿ�����-----------------------------
    /* ����Ҫ�������������ɿ���������ȡ�������ֵ�ע��
    LongTime++;
    if(KeyDown)
    {
        X = KeyDown ^ KeyOn;      // ��ȡ�°��µļ�ֵ��KeyOnλ֮ǰ�Ѱ��µļ�ֵ��KeyDownΪ�µļ�ֵ�����ǲ�ͬ��λ�����µļ�ֵ
        LongTime = 0;
    }
    if(LongTime>30000)            // ��ʱ����λʱ��Ϊ��ѭ����ʱ��
    {
        LongKey |= X;             // �����볤������
        X = 0;
        LongTime = 0;
    }
    //KeyUp = (KeyOn & (~NowKey)) & (~LongKey);  // ���ǳ���������û���ɿ�����
    KeyUp &= ~LongKey;            // ɾ���������ɿ�����λ
    LongKey &= NowKey;            // �ӳ�������ɾ�����ɿ���
    */
    //--------------------------------------------------------------------------

    PreKey = CurrReadKey;         // ������ʷ����
    SwitchOn = NowKey;

    SetRunKeyboard(SwitchOn);
}



/*******************************************************************************
* ���ƣ���165
* ���룺165����Ƭ��
* ���������ֵ���洢��ȫ�ֱ�����
* �漰������KeyDown,Keyup,Keyon
*******************************************************************************/
static uint16_t Read165(uint8_t n)               // 74HC165
{
    uint8_t read[2]={0,0};    // �źű����ݴ�
    uint16_t readback;
    uint8_t  i,j;

    //KEY_Enable;
    // ��ȡ����74HC165��
    Serial165LoadIO(1);//LOAD_165_L;
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    Serial165LoadIO(0);//LOAD_165_H;
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    Serial165LoadIO(1);


    asm("nop");
    asm("nop");
    Serial165ClkIO(0);//CLK_165_L;


    for(j=0;j<n;j++)
    {
        read[j] = 0;
        for(i=0;i<8;i++)
        {
            read[j] <<= 1;
            read[j] |= Serial165InIO();//INPUT_165;

            Serial165ClkIO(0);//CLK_165_L;
            asm("nop");
            asm("nop");
            Serial165ClkIO(1);//CLK_165_H;
            asm("nop");
            asm("nop");
        }
    }

    readback = read[0] | (read[1]<<8);//((read&0xff)<<8) | (read>>8);
    //KEY_Disable;

    return readback;
}

