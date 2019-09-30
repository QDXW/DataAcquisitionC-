
#include "chip.h"


#include "KEY_SCAN.h"
#include "GPIO.h"


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


#define KEY1_INPUT    (Chip_GPIO_ReadPortBit(LPC_GPIO,1,25))
#define KEY2_INPUT    (Chip_GPIO_ReadPortBit(LPC_GPIO,1,0))
#define KEY3_INPUT    (Chip_GPIO_ReadPortBit(LPC_GPIO,1,16))
#define KEY4_INPUT    (Chip_GPIO_ReadPortBit(LPC_GPIO,0,17))
#define SCARM_INPUT   (Chip_GPIO_ReadPortBit(LPC_GPIO,1,13))
#define BUTTON_DETECT (Chip_GPIO_ReadPortBit(LPC_GPIO,2,19))

//----------------��������---------------------
KEYINT  KeyDown =0;  // ���´���
KEYINT  KeyUp   =0;  // �ɿ�����
KEYINT  KeyOn   =0;  // ������ס


//static uint16_t Read(uint8_t n);

// LED�ȺͰ���IO�ڳ�ʼ��
void LED_KEY_IO_Init(void)
{
    GPIO_SetDir(1,26,Output);  // LED1
    GPIO_SetDir(1,27,Output);  // LED2
    GPIO_SetDir(1,4,Output);   // LED3

    LED1(ON);
    LED2(OF);
    LED3(OF);
    
    Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 25, 0xd0);  // ���뷭ת
    Chip_IOCON_PinMuxSet(LPC_IOCON, 1,  0, 0xd0);
    Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 16, 0xd0);
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 17, 0xd0);
    Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 13, 0xd0);

    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 19, 0x88);  // ��������

    GPIO_SetDir(1,25,Input);  // KEY1
    GPIO_SetDir(1, 0,Input);  // KEY2
    GPIO_SetDir(1,16,Input);  // KEY3
    GPIO_SetDir(0,17,Input);  // KEY4
    GPIO_SetDir(1,13,Input);  // SCARM  ��ͣ
    GPIO_SetDir(2,19,Input);  // ��������
}

/*******************************************************************************
* ���ƣ�����ɨ���ӳ���
* ���룺��
* ���������ֵ���洢��ȫ�ֱ�����
* �漰������KeyDown,Keyup,Keyon
*******************************************************************************/
void KeyScan(void)
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
    static KEYINT  PreKey=0x00;      // ��¼�ϴ�KeyScan()��ȡ��IO�ڼ�ֵ
    KEYINT          CurrReadKey;      // ��¼����KeyScan()��ȡ��IO�ڼ�ֵ
    KEYINT          NowKey;           // ��¼���ξ���������������Ч����ֵ
    //--------------------------------------------------------------------------
    /*����Ҫ�������������ɿ���������ȡ�������ֵ�ע��
    static uint8_t LongKey=0;         // ��������
    static uint8_t X=0;               // �°��µĵ���λ
    static uint16_t  LongTime=0;        // ��������ʱ����¼��ѭ������
    */
    //--------------------------------------------------------------------------


    //CurrReadKey = Read(2);   // ����Ƭ74HC165����

    /*
    #define FIRE_IO              GPIO1PIN_P0A_GET   //(KeyOn & BIT6)   // �����ź�����˿�
    #define Feedback_I_ON        GPIO1PIN_NP02_GET  // I·��բ
    #define Feedback_II_ON       GPIO1PIN_NP04_GET  // II·��բ
    #define Feedback_OFF         GPIO1PIN_NP00_GET  // ��բ
    */
    
    CurrReadKey = KEY4_INPUT;
    CurrReadKey = CurrReadKey<<1 | KEY3_INPUT;   //
    
    CurrReadKey = CurrReadKey<<1 | KEY2_INPUT;   //
    
    CurrReadKey = CurrReadKey<<1 | KEY1_INPUT;   //
    
    CurrReadKey = CurrReadKey<<1 | SCARM_INPUT;   //
    
    CurrReadKey = CurrReadKey<<1 | BUTTON_DETECT;

    /***************************************************************************
    * ����ԭ��ܼ򵥣�
    * ����ϴ�LastReadKey�͵�ǰCurReadKey��ȡ�ļ�ֵ��Ϊ1����ô��ǰ��Ч��ֵCurrKeyһ��Ϊ1
    * ����ϴκ͵�ǰ��ȡ�ļ�ֵ��һ���Ļ��������ϴ���Ч��ֵLastKey����һ�¡�
    * ͨ�����ַ�ʽ����������������ʱ�����С��keyscan()��ʱɨ�����ڣ��������ִ���
    ***************************************************************************/
    NowKey = (CurrReadKey & PreKey) | (KeyOn & (CurrReadKey ^ PreKey));

    //��¼�������¼��ͷ�
    KeyDown = (~KeyOn) & NowKey;    // ���°���������λ
    KeyUp   = KeyOn & (~NowKey);    // �ɿ�����������λ

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
    KeyOn = NowKey;
}



/*******************************************************************************
* ���ƣ���165
* ���룺165����Ƭ��
* ���������ֵ���洢��ȫ�ֱ�����
* �漰������KeyDown,Keyup,Keyon
*******************************************************************************/
/*static uint16_t Read(uint8_t n)               // 74HC165
{
    uint8_t read[2];    // �źű����ݴ�
    uint16_t readback;
    uint8_t  i,j;

    KEY_Enable;
    // ��ȡ����74HC165��
    LOAD_165_L;
    asm("nop");
    asm("nop");
    asm("nop");
    LOAD_165_H;

    asm("nop");
    asm("nop");
    CLK_165_L;


    for(j=0;j<n;j++)
    {
        read[j] = 0;
        for(i=0;i<8;i++)
        {
            read[j] <<= 1;
            read[j] |= INPUT_165;

            CLK_165_L;
            asm("nop");
            asm("nop");
            CLK_165_H;
            asm("nop");
            asm("nop");
        }
    }

    readback = read[0] | (read[1]<<8);//((read&0xff)<<8) | (read>>8);
    KEY_Disable;

    return readback;
}*/
