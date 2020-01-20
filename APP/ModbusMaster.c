#include "ModbusMaster.h"
#include <stdio.h>      // ��׼�����������
#include <string.h>
#include "ucos_ii.h"
#include "Memory.h"
#include "GlobalVar.h"
#include "Crc16.h"
#include "Usart.h"
#include "tool.h"

//=================================================================================
#define msleep(x)  OSTimeDly((x)/10+1)              // usleep((x)*1000)
#define sleep(x)   OSTimeDly(OS_TICKS_PER_SEC*(x))  // ��ʱx��
//=================================================================================
static uint8_t g_uWriteReg[256];  // д�Ĵ�����ʱ����
//=================================================================================
DEVICE_ADDR_INFO_T uAllocation={0};
/****************************************************************************
* ��    �ƣ�TimeGapJudge()
* ��    �ܣ��ж�ʱ���
* ��ڲ�����
*         sStart����ʼʱ��
*         sNow��  ���ڵ�ʱ��
*         uGap:�� ʱ���ֵ
* ���ڲ�����
          δ�ﵽʱ���� 1���ﵽʱ����0
* ��    ��: ��
****************************************************************************/
int16_t TimeGapJudge(uint32_t sStart,uint32_t sNow,uint32_t uGap)
{
    // ucos��tickΪ32λ��������ʱ��0xFFFFFFFF����ֹ������ж���ʼʱ�������ʱ��Ĵ�С
    if(((sNow>=sStart)?(sNow - sStart):(0xFFFFFFFF-sStart+sNow))<(uGap/10))   //if((sNow - sStart)<(uGap/10))
    {
        return 1;
    }

    return 0;
}
/****************************************************************************
* ��    �ƣ�ComMasterWrite()
* ��    �ܣ�MODBUSд
* ��ڲ�����
*         pMaster:  ��վ�����ṹ��ָ��
*         uAddr:    ���豸��ַ
*         uCmd:     ������ 0x06  0x10
*         uRegAddr���Ĵ�����ַ
*         uQty��    �Ĵ�������
*         *pReg��   ��������ָ��
* ���ڲ�����д�Ĵ���״̬
* ��    ��: ��
****************************************************************************/
int16_t ComMasterWrite(MODBUS_MASTER_T *pMaster,uint8_t uAddr,uint8_t uCmd,uint16_t uRegAddr,uint16_t uQty,const uint16_t *pReg)
{
    //uint8_t pDRAM[256];     // �շ����ݻ���
    uint8_t  uDataBuffer[8];
    uint16_t i=0,uCrc=0,uLen=0;
    int16_t iRecCount=0;
    uint8_t uSendCount=1,uCrcErr=0;
    uint32_t uFrameSendTime;
    uint32_t sNowTime;
    //uint8_t *pDRAM=NULL;
    uint8_t err;
    //pthread_mutex_lock(pMaster->pComLock);
    OSMutexPend(pMaster->pComLock,0,&err);//�����ź���
    // �ж���֡����֮��ļ��
    do
    {
        //gettimeofday(&sNowTime,NULL);
        sNowTime = OSTimeGet();
        if(TimeGapJudge(pMaster->sSuccessTime,sNowTime,pMaster->uSuccessDelay))
        {
            msleep(20);
            continue;
        }
        else
        {
            break;
        }
    }while(1);

    if(0x06==uCmd)  // д�����Ĵ�����֡
    {
        uDataBuffer[0] = uAddr;
        uDataBuffer[1] = uCmd;
        uDataBuffer[2] = uRegAddr>>8;
        uDataBuffer[3] = uRegAddr&0xFF;

        uDataBuffer[4] = pReg[0]>>8;
        uDataBuffer[5] = pReg[0]&0xFF;

        uCrc = CRC16(uDataBuffer,6);
        uDataBuffer[6] = uCrc>>8;
        uDataBuffer[7] = uCrc&0xFF;

        uLen = 8;

        pMaster->uPreAddr = uAddr;
        pMaster->uPreFun = uCmd;

        UartWrite(pMaster->iComFd,uDataBuffer,uLen);//write(pMaster->iComFd,pDRAM,uLen);
        if(0x01&SouthSwich)
		{
		    DEBUGOUT("SOUTH_WRITE:");
		    for(int i=0;i<uLen;i++)
		    {
		        DEBUGOUT("%02X ",uDataBuffer[i]);
		    }
		    DEBUGOUT("\r\n");
		}
    }
    else if(0x10==uCmd)    // д����Ĵ�����֡
    {
        /*pDRAM = (uint8_t *)WMemMalloc(pDRAM,uQty*2+9);
        if(NULL == pDRAM)
        {
            //�ͷ��ź���
            OSMutexPost(pMaster->pComLock);
            return -MASTER_ERR;
        }*/

        g_uWriteReg[0] = uAddr;
        g_uWriteReg[1] = uCmd;
        g_uWriteReg[2] = uRegAddr>>8;
        g_uWriteReg[3] = uRegAddr&0xFF;
        g_uWriteReg[4] = uQty>>8;
        g_uWriteReg[5] = uQty&0xFF;
        g_uWriteReg[6] = uQty*2;

        for(i=0;i<uQty;i++)
        {
            g_uWriteReg[7+i*2] = pReg[i]>>8;
            g_uWriteReg[8+i*2] = pReg[i]&0xFF;
        }

        uCrc = CRC16(g_uWriteReg,7+uQty*2);
        g_uWriteReg[uQty*2+7] = uCrc>>8;
        g_uWriteReg[uQty*2+8] = uCrc&0xFF;

        uLen = uQty*2+9;

        pMaster->uPreAddr = uAddr;
        pMaster->uPreFun = uCmd;

        UartWrite(pMaster->iComFd,g_uWriteReg,uLen);
		if(0x01&SouthSwich)
		{
		    DEBUGOUT("SOUTH_WRITE:");
		    for(int i=0;i<uLen;i++)
		    {
		        DEBUGOUT("%02X ",g_uWriteReg[i]);
		    }
		    DEBUGOUT("\r\n");
		}
        //WMemFree(pDRAM);
    }
    else
    {
        pMaster->sSuccessTime = OSTimeGet();
        //�ͷ��ź���
        OSMutexPost(pMaster->pComLock);

        return -MODBUS_ILLEGAL_FUN;
    }

    uFrameSendTime = OSTimeGet();    // ��ȡϵͳtick
    //printf("send data %ld:%ld\n",uFrameSendTime.tv_sec,uFrameSendTime.tv_usec);
    if(0==uAddr)   // �㲥
    {
        //pthread_mutex_unlock(pMaster->pComLock);
        //msleep(500);
        pMaster->sSuccessTime = OSTimeGet();
        //�ͷ��ź���
        OSMutexPost(pMaster->pComLock);

        return -MASTER_OK;
    }

    do
    {
        iRecCount = UartRxLen(pMaster->iComFd);// ��ѯ���뻺�����е��ֽ���
        if(iRecCount<=0)
        {
            // gettimeofday(&sNowTime,NULL);
            sNowTime = OSTimeGet();

            if(TimeGapJudge(uFrameSendTime,sNowTime,pMaster->uFailTimeOut))//if(((sNowTime.tv_sec - uFrameSendTime.tv_sec)*1000000 + sNowTime.tv_usec-uFrameSendTime.tv_usec)<(pMaster->uFailTimeOut*1000))
            {
                msleep(5);
                continue;
            }

            if(uSendCount<pMaster->uRecLostMax)
            {
                //gettimeofday(&uFrameSendTime,NULL);
                uFrameSendTime = OSTimeGet();
                uSendCount++;
                //---------------------------------------------------------
                if(0x06==uCmd)  // д�����Ĵ�����֡
                {
                    uDataBuffer[0] = uAddr;
                    uDataBuffer[1] = uCmd;
                    uDataBuffer[2] = uRegAddr>>8;
                    uDataBuffer[3] = uRegAddr&0xFF;

                    uDataBuffer[4] = pReg[0]>>8;
                    uDataBuffer[5] = pReg[0]&0xFF;

                    uCrc = CRC16(uDataBuffer,6);
                    uDataBuffer[6] = uCrc>>8;
                    uDataBuffer[7] = uCrc&0xFF;

                    uLen = 8;

                    pMaster->uPreAddr = uAddr;
                    pMaster->uPreFun = uCmd;

                    UartWrite(pMaster->iComFd,uDataBuffer,uLen);
					if(0x01&SouthSwich)
					{
					    DEBUGOUT("SOUTH_WRITE:");
					    for(int i=0;i<uLen;i++)
					    {
					        DEBUGOUT("%02X ",uDataBuffer[i]);
					    }
					    DEBUGOUT("\r\n");
					}
                }
                else if(0x10==uCmd)    // д����Ĵ�����֡
                {
                    /*pDRAM = (uint8_t *)WMemMalloc(pDRAM,uQty*2+9);
                    if(NULL == pDRAM)
                    {
                        OSMutexPost(pMaster->pComLock);
                        return -MASTER_ERR;
                    }*/

                    g_uWriteReg[0] = uAddr;
                    g_uWriteReg[1] = uCmd;
                    g_uWriteReg[2] = uRegAddr>>8;
                    g_uWriteReg[3] = uRegAddr&0xFF;
                    g_uWriteReg[4] = uQty>>8;
                    g_uWriteReg[5] = uQty&0xFF;
                    g_uWriteReg[6] = uQty*2;

                    for(i=0; i<uQty; i++)
                    {
                        g_uWriteReg[7+i*2] = pReg[i]>>8;
                        g_uWriteReg[8+i*2] = pReg[i]&0xFF;
                    }

                    uCrc = CRC16(g_uWriteReg,7+uQty*2);
                    g_uWriteReg[uQty*2+7] = uCrc>>8;
                    g_uWriteReg[uQty*2+8] = uCrc&0xFF;

                    uLen = uQty*2+9;

                    pMaster->uPreAddr = uAddr;
                    pMaster->uPreFun = uCmd;

                    UartWrite(pMaster->iComFd,g_uWriteReg,uLen);//write(pMaster->iComFd,pDRAM,uLen);
                    if(0x01&SouthSwich)
					{
					    DEBUGOUT("SOUTH_WRITE:");
					    for(int i=0;i<uLen;i++)
					    {
					        DEBUGOUT("%02X ",g_uWriteReg[i]);
					    }
					    DEBUGOUT("\r\n");
					}
                    //WMemFree(pDRAM);
                }
                //---------------------------------------------------------

                continue;
            }
            else
            {
                pMaster->sSuccessTime = OSTimeGet();
                //�ͷ��ź���   start-0
                OSMutexPost(pMaster->pComLock);
                return -MASTER_lOST;
            }
        }

        msleep(200);
        iRecCount = UartRead(pMaster->iComFd, uDataBuffer, 8, 3);//iRecCount = read(pMaster->iComFd,uReadBuffer,300);
        if(0x02&SouthSwich)
	    {
	        DEBUGOUT("SOUTH_WRITE_RECV:");
	        for(int i=0;i<iRecCount;i++)
	        {
	            DEBUGOUT("%02X ",uDataBuffer[i]);
	        }
	        DEBUGOUT("\r\n");
			msleep(1);
	    }
        if(iRecCount<4)
        {
            OSMutexPost(pMaster->pComLock);
            return -MASTER_ERR;
        }
        //printf("Rec data %d\n",iRecCount);
        uCrc = CRC16(uDataBuffer,iRecCount-2);

        if(uCrc!=((uDataBuffer[iRecCount-2]<<8)|(uDataBuffer[iRecCount-1])))
        {
            msleep(pMaster->uFailTimeOut);
            uCrcErr++;
            if(uCrcErr < pMaster->uRecCrcMax)
            {
                //---------------------------------------------------------
                if(0x06==uCmd)  // д�����Ĵ�����֡
                {
                    uDataBuffer[0] = uAddr;
                    uDataBuffer[1] = uCmd;
                    uDataBuffer[2] = uRegAddr>>8;
                    uDataBuffer[3] = uRegAddr&0xFF;

                    uDataBuffer[4] = pReg[0]>>8;
                    uDataBuffer[5] = pReg[0]&0xFF;

                    uCrc = CRC16(uDataBuffer,6);
                    uDataBuffer[6] = uCrc>>8;
                    uDataBuffer[7] = uCrc&0xFF;

                    uLen = 8;

                    pMaster->uPreAddr = uAddr;
                    pMaster->uPreFun = uCmd;

                    uFrameSendTime = OSTimeGet();
                    UartWrite(pMaster->iComFd,uDataBuffer,uLen);
					if(0x01&SouthSwich)
					{
					    DEBUGOUT("SOUTH_WRITE:");
					    for(int i=0;i<uLen;i++)
					    {
					        DEBUGOUT("%02X ",uDataBuffer[i]);
					    }
					    DEBUGOUT("\r\n");
					}	
                }
                else if(0x10==uCmd)    // д����Ĵ�����֡
                {
                    /*pDRAM = (uint8_t *)WMemMalloc(pDRAM,uQty*2+9);
                    if(NULL == pDRAM)
                    {
                        OSMutexPost(pMaster->pComLock);
                        return -MASTER_ERR;
                    }*/

                    g_uWriteReg[0] = uAddr;
                    g_uWriteReg[1] = uCmd;
                    g_uWriteReg[2] = uRegAddr>>8;
                    g_uWriteReg[3] = uRegAddr&0xFF;
                    g_uWriteReg[4] = uQty>>8;
                    g_uWriteReg[5] = uQty&0xFF;
                    g_uWriteReg[6] = uQty*2;

                    for(i=0; i<uQty; i++)
                    {
                        g_uWriteReg[7+i*2] = pReg[i]>>8;
                        g_uWriteReg[8+i*2] = pReg[i]&0xFF;
                    }

                    uCrc = CRC16(g_uWriteReg,7+uQty*2);
                    g_uWriteReg[uQty*2+7] = uCrc>>8;
                    g_uWriteReg[uQty*2+8] = uCrc&0xFF;

                    uLen = uQty*2+9;

                    pMaster->uPreAddr = uAddr;
                    pMaster->uPreFun = uCmd;

                    uFrameSendTime = OSTimeGet();
                    UartWrite(pMaster->iComFd,g_uWriteReg,uLen);//write(pMaster->iComFd,pDRAM,uLen);
                    if(0x01&SouthSwich)
					{
					    DEBUGOUT("SOUTH_WRITE:");
					    for(int i=0;i<uLen;i++)
					    {
					        DEBUGOUT("%02X ",g_uWriteReg[i]);
					    }
					    DEBUGOUT("\r\n");
					}
                    //WMemFree(pDRAM);
                }

                //---------------------------------------------------------
                continue;
            }
            else
            {
                pMaster->sSuccessTime = OSTimeGet();
                //�ͷ��ź���
                OSMutexPost(pMaster->pComLock);
                return -MASTER_CRC;
            }
        }
        else
        {
            pMaster->sSuccessTime = OSTimeGet();
            //�ͷ��ź���
            OSMutexPost(pMaster->pComLock);

            if(uDataBuffer[1]&0x80)
            {
                return -uDataBuffer[2];
            }
            else
            {
                return iRecCount;
            }
        }
    }while(1);

    //�ͷ��ź���
    //OSMutexPost(pMaster->pComLock);
    //return -MASTER_ERR;
}
/****************************************************************************
* ��    �ƣ�ComMasterRead()
* ��    �ܣ�MODBUS��
* ��ڲ�����
*         pMaster:   ��վ�����ṹ��ָ��
*         uAddr:     ���豸��ַ
*         uCmd:      ������ 0x03  0x04 0x2B
*         uRegAddr�� �Ĵ�����ַ
*         uQty��     �Ĵ�������
*         *pRecData���յ����ݴ洢�����ַ
* ���ڲ��������Ĵ���״̬
* ��    ��: ��
****************************************************************************/
int16_t ComMasterRead(MODBUS_MASTER_T *pMaster,uint8_t uAddr,uint8_t uCmd,uint16_t uRegAddr,uint16_t uQty,uint8_t *pRecData,uint8_t *pSendData)
{
    //uint8_t uReadBuffer[256];
    uint8_t uSendBuf[256];
    uint16_t uCrc=0,uLen=0;
    int16_t iRecCount=0;
    uint8_t uSendCount=0,uCrcErr=0,uUpdateErr=0,uSingleCrcErr=0;
    uint8_t err;
    uint8_t uReadQty;  // ��վӦ�÷��ر��ĵ��ֽ�����
	static uint16_t check_crc=0xFFFF;
    uint8_t uMark=1;

    uint32_t uFrameSendTime;   // ��֡����ʱ���
    uint32_t sNowTime;         // ���ڵ�ʱ���

	U32_F_Char_Convert uTemp;

    // �ж�һ����ѭ�������Ƿ����
    #if(1==MASTER_ROUND_OVER)
    uint32_t uRoundNowTime;    // ��ѭ�����ڵ�ʱ���

    while(pMaster->uRoundOver)
    {
        //time(&uRoundNowTime);
        uRoundNowTime = OSTimeGet();
        if(TimeGapJudge(pMaster->uRoundStartTime,uRoundNowTime,pMaster->uRoundTimeOut*1000))//if((uRoundNowTime-pMaster->uRoundStartTime)<pMaster->uRoundTimeOut)
        {
            sleep(1);
            continue;
        }
        else
        {
            pMaster->uRoundOver = 0;
            //time(&pMaster->uRoundStartTime);
            pMaster->uRoundStartTime = OSTimeGet();
            break;
        }
    }
    #endif

    if(0x03==uCmd || 0x04==uCmd)
    {
        uReadQty = uQty * 2 + 5;
    }
    else if(0x01==uCmd)
    {
        uReadQty = uQty/8 + ((uQty%8)==0?0:1) + 5;
    }
    else if((DISCOVERY_SET_ADDR == uCmd) || 
		      (DISCOVERY_REPORT == uCmd) || 
		    (DT1000UPDATE_START == uCmd) || 
		     (DT1000UPDATE_DATA == uCmd) || 
		      (DT1000UPDATE_END == uCmd) || 
               (DT1000LOG_START == uCmd) || 
                (DT1000LOG_DATA == uCmd) || 
                 (DT1000LOG_END== uCmd))
    {
        uReadQty = 255;
    }
    else 
    {
        return -MODBUS_ILLEGAL_FUN;
    }
    
    //pthread_mutex_lock(pMaster->pComLock);// ��ʼ��������
    OSMutexPend(pMaster->pComLock,0,&err); //�����ź���
    // �ж���֡����֮��ļ��
    do
    {
        //gettimeofday(&sNowTime,NULL);
        sNowTime = OSTimeGet();

        if(TimeGapJudge(pMaster->sSuccessTime,sNowTime,pMaster->uSuccessDelay))
        {
            msleep(20);
            continue;
        }
        else
        {
            break;
        }
    }while(1);


	switch (uCmd)    //�����ѯ�������ж�
	{
    case 0x03:
	case 0x04:
	     uSendBuf[0] = uAddr;
         uSendBuf[1] = uCmd;
         uSendBuf[2] = uRegAddr>>8;                //�Ĵ�����ַ
         uSendBuf[3] = uRegAddr&0xFF;
         uSendBuf[4] = uQty>>8;                     //Ҫ��ȡ�Ĵ����ĸ���    
         uSendBuf[5] = uQty&0xFF;

         uCrc = CRC16(uSendBuf,6);
         uSendBuf[6] = uCrc>>8;
         uSendBuf[7] = uCrc&0xFF;

         pMaster->uPreAddr = uAddr;
         pMaster->uPreFun = uCmd;

         uLen = 8;
		 
	break;
	case DISCOVERY_SET_ADDR:
         uSendBuf[0] = 0xD5;
		 uSendBuf[1] = uCmd;
		 uSendBuf[2] = 0x12;               //���ݳ���

		 memcpy(&uSendBuf[3],&pSendData[0],18);

		 uCrc = CRC16(uSendBuf,21);
		 uSendBuf[21] = uCrc>>8;
		 uSendBuf[22] = uCrc&0xFF;

		 pMaster->uPreAddr = uAddr;
		 pMaster->uPreFun = uCmd;

		 uLen = 23;
			
	break;
	case DISCOVERY_REPORT:
		
		uSendBuf[0] = uAddr;
		uSendBuf[1] = uCmd;
		uSendBuf[2] = 0x03;
		uSendBuf[3] = 0x0e; 					 // MEI����
		uSendBuf[4] = 0x03; 					 // ReadDevId��
		uSendBuf[5] = 0x88; 					 // ����ID

		uCrc = CRC16(uSendBuf,6);
		uSendBuf[6] = uCrc>>8;
		uSendBuf[7] = uCrc&0xFF;

		pMaster->uPreAddr = uAddr;
		pMaster->uPreFun = uCmd;

		uLen = 8;
		
	break;
	case DT1000UPDATE_START:
		uMark=0;

		uSendBuf[0] = uAddr;
		uSendBuf[1] = uCmd;
		uSendBuf[2] = 0x01;
		uSendBuf[3] = 0x16;
		memcpy(&uSendBuf[4],&pSendData[0],17);

		uTemp.u = g_DT1000Updata.nDataLen;
		uSendBuf[21] = uTemp.c[3];
		uSendBuf[22] = uTemp.c[2];
		uSendBuf[23] = uTemp.c[1];
		uSendBuf[24] = uTemp.c[0];

		uSendBuf[25] = 240;  //֡����

		uCrc = CRC16(uSendBuf,26);
		uSendBuf[26] = uCrc>>8;
		uSendBuf[27] = uCrc&0xFF;

		pMaster->uPreAddr = uAddr;
		pMaster->uPreFun = uCmd;

		uLen = 28;
		
	break;
	case DT1000UPDATE_DATA:
	    uMark=0;

	    uSendBuf[0] = uAddr;
	    uSendBuf[1] = uCmd;
	    uSendBuf[2] = 0x01;
	   
	    memcpy(&uSendBuf[3],&pSendData[0],uQty);
	    msleep(5);

	    check_crc = CalculateCRC(&uSendBuf[6],uQty-3);
	   
	    uCrc = CRC16(uSendBuf,uQty+3);
	    uSendBuf[uQty+3] = uCrc>>8;
	    uSendBuf[uQty+4] = uCrc&0xFF;

	    pMaster->uPreAddr = uAddr;
	    pMaster->uPreFun = uCmd;

	    uLen = uQty+5;
		
	break;
	case DT1000UPDATE_END:
	    uMark=0;

	    uSendBuf[0] = uAddr;
	    uSendBuf[1] = uCmd;
	    uSendBuf[2]=0x01;  //�ӹ���������0x01
	    uSendBuf[3]=0x04;  //����
	    uSendBuf[4] = pSendData[0];
	    uSendBuf[5] = pSendData[1];
	    uSendBuf[6] = check_crc>>8;
	    uSendBuf[7] = check_crc&0XFF;

	    uCrc = CRC16(uSendBuf,8);
	    uSendBuf[8] = uCrc>>8;
	    uSendBuf[9] = uCrc&0xFF;

	    pMaster->uPreAddr = uAddr;
	    pMaster->uPreFun = uCmd;
	   
	    check_crc=0xFFFF;

	    uLen = 10;
		
	break;
	case DT1000LOG_START:
		
	    uSendBuf[0] = uAddr;
	    uSendBuf[1] = uCmd;
	    uSendBuf[2]=pSendData[0];  //�ӹ���������0x01
	    uSendBuf[3]=pSendData[1];  //����

	    uCrc = CRC16(uSendBuf,4);
	    uSendBuf[4] = uCrc>>8;
	    uSendBuf[5] = uCrc&0xFF;
	   
	    pMaster->uPreAddr = uAddr;
	    pMaster->uPreFun = uCmd;

	    uLen = 6;

	break;
	case DT1000LOG_DATA:
	     uSendBuf[0] = uAddr;
	     uSendBuf[1] = uCmd;
	     uSendBuf[2]=pSendData[0];  //�ӹ���������0x01
	     uSendBuf[3]=pSendData[1];  //����
	     uSendBuf[4]=pSendData[2];  //�ӹ���������0x01
	     uSendBuf[5]=pSendData[3];  //����

	     uCrc = CRC16(uSendBuf,6);
	     uSendBuf[6] = uCrc>>8;
	     uSendBuf[7] = uCrc&0xFF;
	   
	     pMaster->uPreAddr = uAddr;
	     pMaster->uPreFun = uCmd;

	     uLen = 8;
		 
	break;
	case DT1000LOG_END:
	     uSendBuf[0] = uAddr;
	     uSendBuf[1] = uCmd;
	     uSendBuf[2]=pSendData[0];  //�ӹ���������0x01
	     uSendBuf[3]=pSendData[1];  //����

	     uCrc = CRC16(uSendBuf,4);
	     uSendBuf[4] = uCrc>>8;
	     uSendBuf[5] = uCrc&0xFF;
	  
	     pMaster->uPreAddr = uAddr;
	     pMaster->uPreFun = uCmd;

	     uLen = 6;
    break;

	default:
		break;
	}

    uFrameSendTime = OSTimeGet();	
	UartClearSendBuffer(pMaster->iComFd);	// ��մ��ڷ��ͻ�����
	UartClearRecBuffer(pMaster->iComFd);	// ��մ��ڽ��ջ�����
    UartWrite(pMaster->iComFd,uSendBuf,uLen);//write(pMaster->iComFd,uSendBuf,uLen);
    if((0x01&SouthSwich)&uMark)
    {
        DEBUGOUT("SOUTH_SEND:");
        for(int i=0;i<uLen;i++)
        {
            DEBUGOUT("%02X ",uSendBuf[i]);
        }
        DEBUGOUT("\r\n");
	}
    
    if(0x00==uAddr)   // �㲥
    {
        pMaster->sSuccessTime = OSTimeGet();
        OSMutexPost(pMaster->pComLock);        //�ͷ��ź���
        return -MASTER_OK;
    }

    do
    {
        iRecCount = UartRxLen(pMaster->iComFd);   // ��ȡ���ڻ��������ֽ�����	
        if((iRecCount<=0))
        {
            sNowTime = OSTimeGet();

            if(TimeGapJudge(uFrameSendTime,sNowTime,pMaster->uFailTimeOut))
			{
				msleep(20);
				continue;
			}
            //if((pMaster->uPreFun == 0x03)?(uSendCount < 99):(uSendCount < pMaster->uRecLostMax))  
            // �ط�����С������������֡��ʱδ�ظ��ط�
            if(uSendCount < pMaster->uRecLostMax)  			// �ط�����С������������֡��ʱδ�ظ��ط�
            {
                uFrameSendTime = sNowTime;
                uSendCount++;

				UartClearSendBuffer(pMaster->iComFd);	// ��մ��ڷ��ͻ�����
				UartWrite(pMaster->iComFd,uSendBuf,uLen);//write(pMaster->iComFd,uSendBuf,uLen);
				if((0x01&SouthSwich)&uMark)
				{
					DEBUGOUT("SOUTH_RESEND:");
					for(int i=0;i<uLen;i++)
					{
						DEBUGOUT("%02X ",uSendBuf[i]);
					}
					DEBUGOUT("\r\n");
				}
               continue;
            }
            else 
            {
                pMaster->sSuccessTime = OSTimeGet();
                
                //�ͷ��ź���
                OSMutexPost(pMaster->pComLock);
                return -MASTER_lOST;  // ���ض�֡
            }
        }

		iRecCount = UartRead(pMaster->iComFd, pRecData, uReadQty, 3);//iRecCount = read(pMaster->iComFd,pRecData,256);
		CopyData(pRecData);
	    if((0x01&SouthSwich)&uMark)
	    {
	        DEBUGOUT("SOUTH_RECV:");
	        for(int i=0;i<iRecCount;i++)
	        {
	            DEBUGOUT("%02X ",pRecData[i]);
	        }
	        DEBUGOUT("\r\n");
			msleep(1);
	    }
		
        if((DT1000UPDATE_START == pRecData[1])) //�صİ汾�ż����Ȳ���
        {
            if(0 != memcmp(uSendBuf,pRecData,uLen))
            {
                DEBUGOUT("update message err!\n");
				UartClearRecBuffer(pMaster->iComFd);  // ��ջ���
		        msleep(pMaster->uFailTimeOut);
		        uSingleCrcErr++;
		        if(uSingleCrcErr < 3)  // �������ط�
		        {
		            uFrameSendTime = OSTimeGet();
					
		            UartWrite(pMaster->iComFd,uSendBuf,uLen);
					
					msleep(5);
					//DEBUGOUT("update message resend!\n");
		            continue;
		        }
		        else
		        {
		            pMaster->sSuccessTime = OSTimeGet();
		            OSMutexPost(pMaster->pComLock);
		            return -MASTER_lOST;
		        }
			}
		}
        if((DT1000UPDATE_DATA == pRecData[1]) && (0!= pRecData[6]))
        {
			DEBUGOUT("update data err!\n");
			UartClearRecBuffer(pMaster->iComFd);  // ��ջ���
			msleep(pMaster->uFailTimeOut);

            uUpdateErr++;
            if(uUpdateErr < 3)  // �������ط�
            {
                uFrameSendTime = OSTimeGet();
                UartWrite(pMaster->iComFd,uSendBuf,uLen);
				//DEBUGOUT("update data resend!\n");
                continue;
            }
            else
            {
                pMaster->sSuccessTime = OSTimeGet();
                OSMutexPost(pMaster->pComLock);
                return -MASTER_lOST;
            }
		}
        if(iRecCount>=4)
        {
            uCrc = CRC16(pRecData,iRecCount-2);
        }
        else
        {
            uCrc = iRecCount;
        }

        if(uCrc!=((pRecData[iRecCount-2]<<8)|(pRecData[iRecCount-1])) || (iRecCount<4))  // CRCУ�����
        {
            UartClearRecBuffer(pMaster->iComFd);  // ��ջ���

            msleep(pMaster->uFailTimeOut);
            uCrcErr++;
            if(uCrcErr < pMaster->uRecCrcMax)  // CRC У������ط�
            {
                uFrameSendTime = OSTimeGet();
                UartWrite(pMaster->iComFd,uSendBuf,uLen);//write(pMaster->iComFd,uSendBuf,uLen);
                continue;
            }
            else
            {
                pMaster->sSuccessTime = OSTimeGet();
                //�ͷ��ź���
                OSMutexPost(pMaster->pComLock);

                return -MASTER_CRC;
            }
        }
        else
        {
            pMaster->sSuccessTime = OSTimeGet();
            //�ͷ��ź���
            OSMutexPost(pMaster->pComLock);

            if(pRecData[1]&0x80)
            {
                return -pRecData[2];
            }
            else
            {
                return iRecCount;
            }
        }
    }while(1);
}
/****************************************************************************
* ��    �ƣ�ComMasterReadDiscovery()
* ��    �ܣ�MODBUS��
* ��ڲ�����
*         pMaster:   ��վ�����ṹ��ָ��
*         uAddr:     ���豸��ַ
*         uCmd:      ������ 0x03  0x04 0x2B
*         uRegAddr�� �Ĵ�����ַ
*         uQty��     �Ĵ�������
*         *pRecData���յ����ݴ洢�����ַ
* ���ڲ��������Ĵ���״̬
* ��    ��: ��
****************************************************************************/
int16_t ComMasterReadDiscovery(MODBUS_MASTER_T *pMaster,uint8_t uAddr,uint8_t uCmd,uint16_t uRegAddr,uint16_t uQty,uint8_t *pRecData,uint8_t *pSendData)
{
    uint8_t uSendBuf[30];
    uint16_t uCrc=0,uLen=0;
    int16_t iRecCount=0;
    //uint8_t uSendCount=0,uCrcErr=0;
    uint8_t err;
    uint8_t uReadQty;  // ��վӦ�÷��ر��ĵ��ֽ�����
    uint32_t uFrameSendTime;   // ��֡����ʱ���
    uint32_t sNowTime;         // ���ڵ�ʱ���
    static uint8_t uSetAddr = 1;
   
    if(DISCOVERY_START == uCmd)
    {
        uReadQty = 255;
    }
    else
    {
        return -MODBUS_ILLEGAL_FUN;
    }
    // ��ʼ��������
    //pthread_mutex_lock(pMaster->pComLock);
    OSMutexPend(pMaster->pComLock,0,&err); //�����ź���

    if(DISCOVERY_START == uCmd)
    {
        uSendBuf[0] = 0xD5;
        uSendBuf[1] = uCmd;
		uSendBuf[2] = 0x12;                      //��ʽ����

	    memcpy(&uSendBuf[3],g_LoggerInfo.esn,17);
		if(g_LoggerRun.run_status != RUNNING_WORK_READ)
		{
            uSendBuf[20] = 0x00;//δ��վ
		}
		else
		{
			uSendBuf[20] = 0x01;//�ѿ�վ
		}
        uCrc = CRC16(uSendBuf,21);
        uSendBuf[21] = uCrc>>8;
        uSendBuf[22] = uCrc&0xFF;

        pMaster->uPreAddr = uAddr;
        pMaster->uPreFun = uCmd;

        uLen = 23;
    }
	
    uFrameSendTime = OSTimeGet();	
	UartClearSendBuffer(pMaster->iComFd);	// ��մ��ڷ��ͻ�����
    UartClearRecBuffer(pMaster->iComFd);	// ��մ��ڽ��ջ�����
    UartWrite(pMaster->iComFd,uSendBuf,uLen);//write(pMaster->iComFd,uSendBuf,uLen);
    if(0x01&SouthSwich)
    {
        DEBUGOUT("SOUTH_SEND_DISCOVERY:");
        for(int i=0;i<uLen;i++)
        {
            DEBUGOUT("%02X ",uSendBuf[i]);
        }
        DEBUGOUT("\r\n");
    }
	
    memset(&uAllocation,0,sizeof(uAllocation));
    do
    {
		sNowTime = OSTimeGet();
		if(!TimeGapJudge(uFrameSendTime,sNowTime,pMaster->u1BTimeOut))
		{
			 pMaster->sSuccessTime = OSTimeGet();
			 OSMutexPost(pMaster->pComLock);//�ͷ��ź���
			 return -MASTER_lOST;  // ���ض�֡
		}


		iRecCount = UartRxLen(pMaster->iComFd);   // ��ȡ���ڻ��������ֽ�����	
        if(iRecCount<=0)
        {
			 msleep(20);
			 continue;
		}
		else
		{
			iRecCount = UartRead(pMaster->iComFd, pRecData, uReadQty, 3);
			CopyData(pRecData);
			if(0x02&SouthSwich)
			{
				DEBUGOUT("SOUTH_RECV_DISCOVERY:");
				for(int i=0;i<iRecCount;i++)
				{
					DEBUGOUT("%02X ",pRecData[i]);
				}
				DEBUGOUT("\r\n");
				msleep(1);
			}

			if((pRecData[0] == 0xD5) && (pRecData[1] == DISCOVERY_START) && (pRecData[2] == 0x11))
			{
                if(uSetAddr>=MAX_device+1)
                {
					uSetAddr=1;
				}
				
				if(iRecCount>=4)
				{
					uCrc = CRC16(pRecData,iRecCount-2);
				}
				else
				{
					uCrc = iRecCount;
				}
				
				if(uCrc != ((pRecData[iRecCount-2]<<8)|(pRecData[iRecCount-1])) || (iRecCount != 22))
                {
                    memset(uAllocation.cDeviceEsn[uSetAddr],0,18);
				}else
				{
					memset(uAllocation.cDeviceEsn[uSetAddr],0,18);
                    memcpy(uAllocation.cDeviceEsn[uSetAddr],&pRecData[3],17);
				}
				uSetAddr++;
				//DEBUGOUT("uSetAddr:%d",uSetAddr);
			}
		}  
    }while(1);
}
//===============================================================================
#if(1==MASTER_ROUND_OVER)
/****************************************************************************
* ��    �ƣ�ComMasterRoundOver()
* ��    �ܣ�MODBUS���ò�ѯһ��ѭ������
* ��ڲ�����
*         pMaster:��վ�����ṹ��ָ��
* ���ڲ�������
* ��    ��: ��
****************************************************************************/
int16_t ComMasterRoundOver(MODBUS_MASTER_T *pMaster)
{
    pMaster->uRoundOver = 1;
    return 0;
}
/****************************************************************************
* ��    �ƣ�ComMasterRoundStart()
* ��    �ܣ�MODBUS���ò�ѯһ��ѭ������
* ��ڲ�����
*         pMaster:��վ�����ṹ��ָ��
* ���ڲ�������
* ��    ��: ��
****************************************************************************/
int16_t ComMasterRoundStart(MODBUS_MASTER_T *pMaster)
{
    //time(&pMaster->uRoundStartTime);
    pMaster->uRoundStartTime = OSTimeGet();
    return 0;
}
#endif
