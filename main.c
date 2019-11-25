#include "chip.h"
#include "ucos_ii.h"
#include "GlobalVar.h"
#include "KEY_SCAN.h"
#include "Usart.h"
#include "SPI.h"
#include "GPIO.h"
#include "InEEPROM.h"
#include "WatchDog.h"
#include "InTIMER.h"
#include "Update.h"
#include "CRC16.h"
#include "RealTime.h"
#include "DataFlash.h"
#include "Memory.h"
#include "DataTransferArea.h"
#include "SlaveUnit.h"
#include "Record.h"
#include "Uart0process.h"
#include "CM.h"
#include "IEC104.h"
#include "LED.h"
#include "log.h"

//============================================================================================
#define DEBUG_MODE   (0)   // ����ģʽ

//============================================================================================
void DataInit(void);
void DeviceInit(void);
uint8_t Crystal_oscillator_Init2(void);
// �ϱ���������ת��
void NorthToSouth(void);
void EepUpdataMes(void);

//============================================================================================
#define SIDE_A_VTOR_OFFSET         (0x00006000)
#define SIDE_B_VTOR_OFFSET         (0x00028000)
void ReAllocateNVIC(uint32_t offset)
{
    /*__disable_irq();
    NVIC_SetVTOR(VTOR_OFFSET);
    __enable_irq();*/

    uint32_t* src,*dst;
    int32_t size;

    __disable_irq();
    // copy vector table
    src = (uint32_t*)offset;
    dst = (uint32_t*)0x10000000;  // RAM����ʼ��ַ
    size = 192;  // һ��48���ж����

    while(size > 0)  // �����ж��������ڴ�
    {
        *dst++ = *src++;
        size -= 4;
    }
     LPC_SYSCTL->SYSMEMREMAP = 0x1;    /*LPC_SYSCON remap to internal RAM */
    //__enable_irq();
}

//============================================================================================
int main(void)
{
    DeviceInit();   // �豸��ʼ��
    PrintThisInfo();   // ��ӡ������Ϣ
    OSInit();    // init uc/os-ii
    SysMesInit();   // ϵͳ�źų�ʼ��

    OSTaskCreateExt (TaskModemProcess,                                      // ������չ����(�������ָ��)
                        (void *)0,                                          // ���ݲ���ָ��
                        &TaskModemProcessStk[TaskModemProcessStkSize-1],    // ���������ջջ��ָ��
                        3,                                                  // �����������ȼ�
                        3,                                                  // (δ����)���ȼ���ʶ(�����ȼ���ͬ)
                        &TaskModemProcessStk[0],                            // ���������ջջ��ָ��
                        TaskModemProcessStkSize,                            // ָ����ջ������(������)
                        (void *)0,                                          // ָ���û����ӵ��������ָ��
                        OS_TASK_OPT_STK_CHK+OS_TASK_OPT_STK_CLR);           // ���������趨ѡ��

    OSTaskCreateExt (TaskIec104Process,                                     // ������չ����(�������ָ��)
                        (void *)0,                                          // ���ݲ���ָ��
                        &TaskIec104ProcessStk[TaskIec104ProcessStkSize-1],  // ���������ջջ��ָ��
                        5,                                                  // �����������ȼ�
                        5,                                                  // (δ����)���ȼ���ʶ(�����ȼ���ͬ)
                        &TaskIec104ProcessStk[0],                           // ���������ջջ��ָ��
                        TaskIec104ProcessStkSize,                           // ָ����ջ������(������)
                        (void *)0,                                          // ָ���û����ӵ��������ָ��
                        OS_TASK_OPT_STK_CHK+OS_TASK_OPT_STK_CLR);           // ���������趨ѡ��

    OSTaskCreateExt (TaskSouthInquire,                                      // ������չ����(�������ָ��)
                        (void *)0,                                          // ���ݲ���ָ��
                        &TaskSouthInquireStk[TaskSouthInquireStkSize-1],    // ���������ջջ��ָ��
                        6,                                                  // �����������ȼ�
                        6,                                                  // (δ����)���ȼ���ʶ(�����ȼ���ͬ)
                        &TaskSouthInquireStk[0],                            // ���������ջջ��ָ��
                        TaskSouthInquireStkSize,                            // ָ����ջ������(������)
                        (void *)0,                                          // ָ���û����ӵ��������ָ��
                        OS_TASK_OPT_STK_CHK+OS_TASK_OPT_STK_CLR);           // ���������趨ѡ��

    OSTaskCreateExt (TaskSouthWrite,                                        // ������չ����(�������ָ��)
                        (void *)0,                                          // ���ݲ���ָ��
                        &TaskSouthWriteStk[TaskSouthWriteStkSize-1],        // ���������ջջ��ָ��
                        7,                                                  // �����������ȼ�
                        7,                                                  // (δ����)���ȼ���ʶ(�����ȼ���ͬ)
                        &TaskSouthWriteStk[0],                              // ���������ջջ��ָ��
                        TaskSouthWriteStkSize,                              // ָ����ջ������(������)
                        (void *)0,                                          // ָ���û����ӵ��������ָ��
                        OS_TASK_OPT_STK_CHK+OS_TASK_OPT_STK_CLR);           // ���������趨ѡ��

    OSTaskCreateExt (TaskUart0Process,                                      // ������չ����(�������ָ��)
                        (void *)0,                                          // ���ݲ���ָ��
                        &TaskUart0ProcesStk[TASKUART0STKSIZE-1],            // ���������ջջ��ָ��
                        8,                                                  // �����������ȼ�
                        8,                                                  // (δ����)���ȼ���ʶ(�����ȼ���ͬ)
                        &TaskUart0ProcesStk[0],                             // ���������ջջ��ָ��
                        TASKUART0STKSIZE,                                   // ָ����ջ������(������)
                        (void *)0,                                          // ָ���û����ӵ��������ָ��
                        OS_TASK_OPT_STK_CHK+OS_TASK_OPT_STK_CLR);           // ���������趨ѡ��

    OSTaskCreateExt (TaskLedCtrl,                                           // ������չ����(�������ָ��)
                        (void *)0,                                          // ���ݲ���ָ��
                        &TaskLedCtrlStk[TaskLedCtrlStkSize-1],              // ���������ջջ��ָ��
                        9,                                                  // �����������ȼ�
                        9,                                                  // (δ����)���ȼ���ʶ(�����ȼ���ͬ)
                        &TaskLedCtrlStk[0],                                 // ���������ջջ��ָ��
                        TaskLedCtrlStkSize,                                 // ָ����ջ������(������)
                        (void *)0,                                          // ָ���û����ӵ��������ָ��
                        OS_TASK_OPT_STK_CHK+OS_TASK_OPT_STK_CLR);           // ���������趨ѡ��
    OSStart();
}

/****************************************************************************
* ��    �ƣ�DeviceInit()
* ��    �ܣ��豸��ʼ����
* ��ڲ�������
* ���ڲ�������
* ��    ��: ��
****************************************************************************/
void DeviceInit(void)
{
    __disable_irq();
    g_South_Action_Newtime = OSTimeGet();
    // ƫ���ж����������Ƶ�RAM
    if(GetVerS2()&0x0001)  // С�汾�����λ��1��������A�棻0��������B��
    {
        ReAllocateNVIC(SIDE_A_VTOR_OFFSET);
    }
    else
    {
        ReAllocateNVIC(SIDE_B_VTOR_OFFSET);
    }

    Chip_GPIO_Init(LPC_GPIO);
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);  // ���룬��ʼ���ⲿ����ǰ���У���Ȼ���ܷ����ⲿ��������Chip_GPIO_Init(LPC_GPIO);

    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_RAM1);    // ʹ��SRAM1ʱ��
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_USBRAM);  // ʹ��SRAM2ʱ��

    // �����ʼ��
    if(0==Crystal_oscillator_Init2())  // �ⲿ�����ʼ��ʧ��
    {
        LED3(ON);
    }

    SystemCoreClockUpdate();

    SysTick_Config(SystemCoreClock/100);   // SysTick interrupt @ 100 Hz = 10 msec

	//RealTimeInit(); 	// ʱ�ӳ�ʼ����IIC��ʼ��

    UartInit(uart0,115200,UART_PARITY_NONE);      // ����0��ʼ�� RS232
    //UartInit(uart1,9600,  UART_PARITY_NONE);      // ����1��ʼ�� Zigbee
    //UartInit(uart2,9600,  UART_PARITY_NONE);      // ����2��ʼ�� RS485
    UartInit(uart3,9600,  UART_PARITY_NONE);      // ����3��ʼ�� RS232
    UartInit(uart4,115200,UART_PARITY_NONE);      // ����4��ʼ�� RS232  ģ��

    #if(0==DEBUG_MODE)
    WatchDog_Init();// ��ʼ�����Ź�
    FEED_DOG();     // ι��
    #endif

    DataFlashInit();   // DataFlash��ʼ��
    
    #if(0==DEBUG_MODE)
    FEED_DOG();     // ι��
    #endif

    #if(1==DEBUG_MODE)
    DEBUGOUT("--------------����ģʽ����--------------\n");
    #endif

    DataFlash_Read_ID();   // ��ȡDataFlsh ID

    __enable_irq();    // ʹ���ж�

    DataInit();        // ���ݳ�ʼ��
    RecordInit(0);     // ��ʷ���ݴ洢��ʼ��

	if(NULL!=SOUTH_DATA_LOG)
    {
        SOUTH_DATA_LOG = WMemFree(SOUTH_DATA_LOG);
    }
    SaveLogInit(0);

    // ��ȡ����eeprom���������Ϣ
    EepUpdataMes();
}
/****************************************************************************
* ��    �ƣ�DataInit()
* ��    �ܣ����ݳ�ʼ����
* ��ڲ�������
* ���ڲ�������
* ��    ��: ��
****************************************************************************/
void DataInit(void)
{
    uint16_t uEepRead=0;
    uint16_t sucess=0;
    uint8_t  i=0;
    SYSTEMTIME *pTime;

    // ��ȡ���ɱ�����Ϣ
    uEepRead = ReadEepData(EEP_LOGGER_INF);
    if(EEP_ERR==uEepRead) // ��ȡ����CRCУ�����
    {
        SetLoggerDefInfo();  // ����ΪĬ����Ϣ
        SaveEepData(EEP_LOGGER_INF);
        sucess |= 0x01;  // ������Ϣ��ȡ�ɹ�
    }
    else
    {
        SaveEepData(EEP_LOGGER_INF);
        sucess |= 0x01;  // ������Ϣ��ȡ�ɹ�
    }

    // ���豸������Ϣ �� �����Ϣ
    uEepRead = ReadEepData(EEP_DEVICE_SOUTH);
    if(EEP_ERR==uEepRead) // ��ȡ����CRCУ�����
    {
        AllReset(1);
    }
    else
    {
        sucess |= 0x02;  // ��������Ϣ��ȡ�ɹ�

        // ��ȡ�����豸ESN��
        uEepRead = ReadEepData(EEP_DEVICE_ESN);
        uEepRead = ReadEepData(EEP_DEVICE_SOFT);
    }

    //-----------------------------------------------------------------------------
    // �������Ϣ
    for(i=0;i<MAX_device;i++)//&&i<g_DeviceSouth.device_sum
    {
        if(0!=g_DeviceSouth.protocol[i].protocol_num && 0!=g_DeviceSouth.protocol[i].mess_point_sum && 0!=g_DeviceSouth.protocol[i].flash_addr)
        {
            g_pRegPoint[i] = (LOGGER_MODBUS_REG_T*)WMemMalloc(g_pRegPoint[i],g_DeviceSouth.protocol[i].mess_point_sum*sizeof(LOGGER_MODBUS_REG_T));   // ����ռ�
            if(NULL!=g_pRegPoint[i]) // ����ռ�ɹ�
            {
                DataFlash_Read(g_DeviceSouth.protocol[i].flash_addr,(uint8_t*)&g_pRegPoint[i][0],g_DeviceSouth.protocol[i].mess_point_sum*sizeof(LOGGER_MODBUS_REG_T));
                sucess |= 0x04;   // ����ȡ�ɹ�
            }
            else
            {
                DEBUGOUT("Ҫ���ռ�ʧ��-%d\n",i);
                sucess &= ~0x04;  // ����ȡʧ��
            }
        }
    }
    //-----------------------------------------------------------------------------
    //-----------------------------------------------------------------------------
    // ���澯��Ϣ
    ReadEepData(EEP_ALARM);

    // ��ȡԤ�õ����Ϣ������
    //ReadEepData(EEP_TABLE_SEQ);
    //Init_DevicesTypes_Table();

    //-------------------------------------------------------------------
    if((0x07)==sucess && 0!=g_DeviceSouth.device_sum)
    {
        g_LoggerRun.run_status = RUNNING_WORK_READ;
    }
    else
    {
        g_LoggerRun.run_status = RUNNING_EMPTY;//RUNNING_WORK_READ;
    }
    //-----------------------------------------------------------------------------


    //===========================================================
    pTime = RealTimeGet();

//    if(!g_TimeStamp)
//    {
//        InitInRtc();
//    }

    if(pTime->Year>100 || pTime->Month>12 || pTime->Date>31 || pTime->Hour>23 || pTime->Minute>59 || pTime->Second>59)
    {
//        Date_Time.Year   = 0x00;
//        Date_Time.Month  = 0x00;
//        Date_Time.Date   = 0x00;
//        Date_Time.Week   = 0x00;
//        Date_Time.Hour   = 0x00;
//        Date_Time.Minute = 0x00;
//        Date_Time.Second = 0x00;
        //RealTime_Set();

        SetRecordAllow(0);  // ʱ�䲻�ԣ�����¼��ʷ����
    }
    else
    {
        SetRecordAllow(1);  // ʱ��ԣ���¼��ʷ����
    }
}

//============================================================================
// �ȳ�ʼ���ⲿ���񣬳��������ⲿ����ΪPLLԴ���������ɹ������ⲿ����Ϊ��ʱ�ӣ���ʧ���ڲ�IRCΪ��ʱ�ӣ��ر�PLL
uint8_t Crystal_oscillator_Init(void)
{
	volatile int i;

    for (i = 0; i < 250; i++) {}
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 0, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_ADMODE_EN)); // �ⲿ��������
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 1, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_ADMODE_EN)); // �ⲿ��������
    for (i = 0; i < 250; i++) {}

	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_SYSOSC_PD);  // ��ʱ�Ӿ��񹩵�

    //Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_IRC_PD);

    Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_IRC);  // ��ʱ������Ϊ�ڲ�IRC


	for (i = 0; i < 2500; i++) {} // Wait for at least 580uS for osc to stabilize

    Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);     // �ر�PLL


	Chip_Clock_SetSystemPLLSource(SYSCTL_PLLCLKSRC_MAINOSC);  // ����PLLԴΪ��ʱ�Ӿ���

    while (!(LPC_SYSCTL->SYSPLLCLKUEN & 0x01));     //�ȴ��������

	/* Setup FLASH access */
    //MCU����Ƶ����Ϊ48MHz��FLASHTIMӦ������Ϊ0x02��3��ϵͳ�ȴ����ڣ��� LPCOPENʵ������Ϊ2��ʱ�ӵȴ����ڣ������UM�涨���󡣰���UM�޸ĳ�0x02�󣬸��²��Ժ������ϵ�/��λ����
    Chip_FMC_SetFLASHAccess(FLASHTIM_3CLK_CPU);
	//Chip_FMC_SetFLASHAccess(FLASHTIM_2CLK_CPU);

	// Power down PLL to change the PLL divider ratio
	Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);

	/* Setup PLL for main oscillator rate (FCLKIN = 12MHz) * 4 = 48MHz
	   MSEL = 3 (this is pre-decremented), PSEL = 1 (for P = 2)
	   FCLKOUT = FCLKIN * (MSEL + 1) = 12MHz * 4 = 48MHz
	   FCCO = FCLKOUT * 2 * P = 48MHz * 2 * 2 = 192MHz (within FCCO range) */
	Chip_Clock_SetupSystemPLL(3, 1);

	// Powerup system PLL
	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_SYSPLL_PD);

	// Wait for PLL to lock
    i = 0;
	while (!Chip_Clock_IsSystemPLLLocked())
    {
        i++;
        if(i>100)
        {
            Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);  // �ر�PLL
            Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSOSC_PD);  // ��ʱ�Ӿ���ϵ�
            Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 0, 0x90);
            Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 1, 0x90);

            for (i = 0; i < 2500; i++) {}

            Chip_Clock_SetSysClockDiv(1);
            return 0;
        }
    }

    Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);

	// Set system clock divider to 1
	Chip_Clock_SetSysClockDiv(1);

	/* Set main clock source to the system PLL. This will drive 48MHz
	   for the main clock and 48MHz for the system clock */
	//Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_PLLOUT);
    Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_PLLIN);

    for (i = 0; i < 250; i++) {}
    for (i = 0; i < 250; i++) {}

    return 1;
}
//============================================================================
// �ȳ�ʼ���ⲿ���񣬳��������ⲿ����ΪPLLԴ���������ɹ������ⲿ����ΪPLLԴ��PLL����Ϊ��ʱ�ӣ���ʧ���ڲ�IRCΪ��ʱ�ӣ��ر�PLL
uint8_t Crystal_oscillator_Init2(void)
{
	volatile int i;

    for (i = 0; i < 250; i++) {}
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 0, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_ADMODE_EN)); // �ⲿ��������
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 1, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_ADMODE_EN)); // �ⲿ��������
    for (i = 0; i < 250; i++) {}

	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_SYSOSC_PD);  // ��ʱ�Ӿ��񹩵�

    //Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_IRC_PD);

    Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_IRC);  // ��ʱ������Ϊ�ڲ�IRC

	for (i = 0; i < 2500; i++) {} // Wait for at least 580uS for osc to stabilize

    Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);     // �ر�PLL

	Chip_Clock_SetSystemPLLSource(SYSCTL_PLLCLKSRC_MAINOSC);  // ����PLLԴΪ��ʱ�Ӿ���

    while (!(LPC_SYSCTL->SYSPLLCLKUEN & 0x01));     //�ȴ��������

	/* Setup FLASH access */
    //MCU����Ƶ����Ϊ48MHz��FLASHTIMӦ������Ϊ0x02��3��ϵͳ�ȴ����ڣ��� LPCOPENʵ������Ϊ2��ʱ�ӵȴ����ڣ������UM�涨���󡣰���UM�޸ĳ�0x02�󣬸��²��Ժ������ϵ�/��λ����
    Chip_FMC_SetFLASHAccess(FLASHTIM_3CLK_CPU);
	//Chip_FMC_SetFLASHAccess(FLASHTIM_2CLK_CPU);

	// Power down PLL to change the PLL divider ratio
	Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);

	/* Setup PLL for main oscillator rate (FCLKIN = 12MHz) * 4 = 48MHz
	   MSEL = 3 (this is pre-decremented), PSEL = 1 (for P = 2)
	   FCLKOUT = FCLKIN * (MSEL + 1) = 12MHz * 4 = 48MHz
	   FCCO = FCLKOUT * 2 * P = 48MHz * 2 * 2 = 192MHz (within FCCO range) */
	Chip_Clock_SetupSystemPLL(3, 1);

	// Powerup system PLL
	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_SYSPLL_PD);

	// Wait for PLL to lock
    i = 0;
	while (!Chip_Clock_IsSystemPLLLocked())
    {
        i++;
        if(i>100)
        {
            Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);  // �ر�PLL
            Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSOSC_PD);  // ��ʱ�Ӿ���ϵ�
            Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 0, 0x90);
            Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 1, 0x90);

            for (i = 0; i < 2500; i++) {}

            /*
            Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);  // �ر�PLL

            Chip_Clock_SetSystemPLLSource(SYSCTL_PLLCLKSRC_IRC);  // ����PLLԴΪ�ڲ�RC
            while (!(LPC_SYSCTL->SYSPLLCLKUEN & 0x01));          // �ȴ��������

            //Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);
            Chip_Clock_SetupSystemPLL(3, 1);
            Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_SYSPLL_PD);

            while (!Chip_Clock_IsSystemPLLLocked()){}


            // Set system clock divider to 1
            Chip_Clock_SetSysClockDiv(1);


            //Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_PLLOUT);
            */

            Chip_Clock_SetSysClockDiv(1);
            return 0;
        }
    }

	// Set system clock divider to 1
	Chip_Clock_SetSysClockDiv(1);

	/* Set main clock source to the system PLL. This will drive 48MHz
	   for the main clock and 48MHz for the system clock */
	Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_PLLOUT);

    for (i = 0; i < 250; i++) {}
    for (i = 0; i < 250; i++) {}

    return 1;
}
//============================================================================
// �����ڲ�����ΪPLLԴ���������ɹ������ⲿ����Ϊ��ʱ�ӣ���ʧ������ѭ��
#if 0
void Chip_SetupIrcClocking2(void)
{
    volatile int i;

	/* Turn on the IRC by clearing the power down bit */
	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_IRC_PD);

    Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_IRC);  // ��ʱ������Ϊ�ڲ�IRC

    for (i = 0; i < 250; i++) {}

	/* Select the PLL input in the IRC */
	Chip_Clock_SetSystemPLLSource(SYSCTL_PLLCLKSRC_IRC);

	/* Setup FLASH access */
    //MCU����Ƶ����Ϊ48MHz��FLASHTIMӦ������Ϊ0x02��3��ϵͳ�ȴ����ڣ��� LPCOPENʵ������Ϊ2��ʱ�ӵȴ����ڣ������UM�涨���󡣰���UM�޸ĳ�0x02�󣬸��²��Ժ������ϵ�/��λ����
    Chip_FMC_SetFLASHAccess(FLASHTIM_3CLK_CPU);
	//Chip_FMC_SetFLASHAccess(FLASHTIM_2CLK_CPU);


	/* Power down PLL to change the PLL divider ratio */
	Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);

	/* Configure the PLL M and P dividers */
	/* Setup PLL for main oscillator rate (FCLKIN = 12MHz) * 4 = 48MHz
	   MSEL = 3 (this is pre-decremented), PSEL = 1 (for P = 2)
	   FCLKOUT = FCLKIN * (MSEL + 1) = 12MHz * 4 = 48MHz
	   FCCO = FCLKOUT * 2 * P = 48MHz * 2 * 2 = 192MHz (within FCCO range) */
	Chip_Clock_SetupSystemPLL(3, 1);

	/* Turn on the PLL by clearing the power down bit */
	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_SYSPLL_PD);

	/* Wait for PLL to lock */
	while (!Chip_Clock_IsSystemPLLLocked()) {}

	/* Set system clock divider to 1 */
	Chip_Clock_SetSysClockDiv(1);

	/* Set main clock source to the system PLL. This will drive 24MHz
	   for the main clock and 24MHz for the system clock */
	Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_PLLOUT);

}
#endif
//-------------------------------------------------------------
// �ϱ���������ת��
/*void NorthToSouth(void)
{
    #define COM_N         uart2
    #define COM_STATE_N   UART2_STATE
    #define COM_S         uart1
    #define COM_STATE_S   UART1_STATE
    uint16_t  read_count;
    uint8_t  inbuf[265]={0};

    // �յ�����ͨѶ
    if(1==COM_STATE_N.Rec_OK)
    {
        //LED2_twinkle();  // LED3��˸

        COM_STATE_N.Rec_OK = 0;

        read_count = UART_GetRxCount(COM_N);

        read_count = UARTn_Read(COM_N,(uint8_t*)&inbuf[0],read_count);

        UARTn_Send(COM_S,inbuf,read_count);   // ��485ת��
        UART_ClearRxBuff(COM_N);              // ���³�ʼ������������ա�
    }

    // �յ�����ͨѶ
    if(1==COM_STATE_S.Rec_OK)
    {
        //LED3_twinkle();  // LED3��˸

        COM_STATE_S.Rec_OK = 0;

        read_count = UART_GetRxCount(COM_S);

        read_count = UARTn_Read(COM_S,(uint8_t*)&inbuf[0],read_count);

        UARTn_Send(COM_N,inbuf,read_count);  // ��232ת��

        UART_ClearRxBuff(COM_S);             // ���³�ʼ������������ա�
    }
}*/
/******************************************************************************
* ��    �ƣ�EepUpdataMes()
* ��    �ܣ�EEPROM������Ϣ�жϡ�
* ��ڲ�����
*           ��      ��

* ���ڲ�����
* ��    ��:
******************************************************************************/
void EepUpdataMes(void)
{
    UPDATA_MARK_T sUpdata;

    uint16_t uSoftVer;

    uSoftVer = GetVerS2();//���λ1:��A�棻0:��B��

    EepReadData(EEPROM_UPDATA_MARK_ADDRESS,(uint8_t *)&sUpdata,sizeof(UPDATA_MARK_T),&sUpdata.CRC);// ������Ϣ��ȡ
    if(0xAA!=sUpdata.sucess || ((uSoftVer&0x0001) && 0xAA!=sUpdata.side) || (0==(uSoftVer&0x0001) && 0xBB!=sUpdata.side))
    {
        sUpdata.sucess = 0xAA;

        if(uSoftVer&0x0001)  // С�汾�����λ��1��������A�棻0��������B��
        {
            sUpdata.side = 0xAA;
        }
        else
        {
            sUpdata.side = 0xBB;
        }
        EepSavedata(EEPROM_UPDATA_MARK_ADDRESS,(uint8_t *)&sUpdata,sizeof(UPDATA_MARK_T),&sUpdata.CRC);// ������Ϣ�洢
    }
}

