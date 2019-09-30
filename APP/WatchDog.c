#include "chip.h"

#include "InWDT.h"
#include "GPIO.h"

#include "WatchDog.h"

static uint8_t g_uDog=0;
void WatchDog_Init(void)
{
    InWDT_Init();
    GPIO_SetDir(0,16,Output);   // �ⲿ���Ź�2-8
    g_uDog = 1;
}

void FEED_DOG(void)
{
    if(g_uDog)
    {
        GPIO_TwinkleBit(0,16);  // ι�ⲿ����
        FeedDog();              // ι�ڲ����Ź�
    }
}

void MCU_Reboot(void)
{
    //GPIO_SetDir(2,23,Output);
    //GPIO_SetBit(2,23,Low);
}
