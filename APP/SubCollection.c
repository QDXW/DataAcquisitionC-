/**
 * FileName:
 * Company: pinnenger
 * @author chenyang
 * @date 2017-07-30����11:59:59
 * @version 1.0
 * Description:������Ч
 *
 * ......................�ҷ�ȱ�......................
 *                       _oo0oo_
 *                      o8888888o
 *                      88" . "88
 *                      (| -_- |)
 *                      0\  =  /0
 *                    ___/`---'\___
 *                  .' \\|     |// '.
 *                 / \\|||  :  |||// \
 *                / _||||| -�d-|||||- \
 *               |   | \\\  -  /// |   |
 *               | \_|  ''\---/''  |_/ |
 *               \  .-\__  '-'  ___/-. /
 *             ___'. .'  /--.--\  `. .'___
 *          ."" '<  `.___\_<|>_/___.' >' "".
 *         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *         \  \ `_.   \_ __\ /__ _/   .-` /  /
 *     =====`-.____`.___ \_____/___.-`___.-'=====
 *                       `=---='
 *
 *..................���濪�� ,����BUG...................
 *
 */
#include "SubCollection.h"

#include <stdio.h>
#include <time.h>
#include "IEC104.h"
#include "GlobalVar.h"
#include "WatchDog.h"

#include "DataFlash.h"
#include "Record.h"
#include "InEEPROM.h"
#include "CM.h"

#define OK 0
#define ERR -1
#define INTERVAL_TIME_RANG 150 	//ƽָ̨��ʱ���ǰ�����ְ������
#define NO_DATA -2 //������ʷ���ݲ��Ҳ�����Ҫ����ʱ��ǰ��2.5���ӷ�Χ�ڵ�����
#define FEED_WATCH_DOG()   FEED_DOG()    // ι��

extern uint8_t g_TimeMarker[7];
/******************************************************************************
* ��    �ƣ�tMarkCheck()
* ��    �ܣ�����ȡ������ʷ����ʱ���Ƿ�����Ҫ���ɵ�ʱ�귶Χ��
* ��ڲ�����
*			char menAddr[]		��ȡ����ʷ�����е�ͷ���ʱ��
*			time_t targetTime	ƽ̨�·�����Ҫ���ɵ�ʱ��,�Ѿ�����ɾ���������
*
* ���ڲ�����
* ��    ��:
* ADD BY: chenyang
******************************************************************************/
int32_t tMarkCheck(uint8_t menAddr[],time_t targetTime )
{
    struct tm src = {0};
    time_t recordTime;
    float result;

    src.tm_sec = menAddr[7];
    src.tm_min = menAddr[6];
    src.tm_hour = menAddr[5];
    src.tm_mday = menAddr[4]&0x1F;//�յĸ���λ�����ڣ�����ֻȡ����λ
    src.tm_wday = menAddr[4]&0xE0;//ֻȡ����λ
    src.tm_mon = menAddr[3]-1;
    src.tm_year = menAddr[2]+100;//�������ر�ע�⣺�����Ǵ�1900�������������˴�Ҫ��dst�����ֵ���Ժ��Ծ������
    src.tm_isdst = 0;   //��ʵ������ʱ
    recordTime = mktime(&src);

    if(targetTime>=recordTime)
    {
        result = difftime(targetTime,recordTime);    //����ƽ̨�·�ʱ�������־��ʱ����ļ����
    }else{
        result = difftime(recordTime,targetTime);    // �����ʱ����ڵ�һ�����
    }
    if(result<=540  && result>=0)
    {
        /*
        g_TimeMarker[0]=menAddr[7]*1000%256;
        g_TimeMarker[1]=menAddr[7]*1000/256;
        g_TimeMarker[2]=menAddr[6];
        g_TimeMarker[3]=menAddr[5];
        g_TimeMarker[4]=menAddr[4];
        g_TimeMarker[5]=menAddr[3]-1;   //�ϱ�ƽ̨ʱ���·ݴ�0��ʼ����
        g_TimeMarker[6]=menAddr[2];
        */
//        DEBUGOUT("\\\\\\\\\\\\\\\\\\\\\\\ tMarkCheck Success \\\\\\\\\\\\\\\\\n");
    	return OK;
    }
    return NO_DATA;
}
/******************************************************************************
* ��    �ƣ�hMarkCheck()
* ��    �ܣ�����ȡ������ʷ������ʼ��ַ�Ƿ�Ϊͷ��
* ��ڲ�����
*		char array[]	����ʷ�����ж�ȡ��ǰ�˸��ַ���������ͷ���ʱ��
*
* ���ڲ�����
* ��    ��:
* ADD BY: chenyang
******************************************************************************/
int32_t hMarkCheck(uint8_t array[])
{
    if((array[0] == 0xAA)&&(array[1] == 0x55))
    {
        return OK;
    }
    return ERR;
}
/******************************************************************************
* ��    CheckTarget()
* ��    �ܣ�����ȡ������ʷ������ʼ��ַ�Ƿ�Ϊͷ��
* ��ڲ�����
*			char array[]	����ʷ�����ж�ȡ��ǰ�˸��ַ���������ͷ���ʱ��
*
* ���ڲ�����
* ��    ��:
* ADD BY: chenyang
******************************************************************************/
int CheckTarget(uint32_t uReadFlash,time_t targetTime,uint32_t uFrameLenth,uint32_t uTimes,uint32_t *uNeedReadFlash)
{
    int32_t iRecord = NO_DATA;
    uint8_t uSubCheckAddr[SUBCHECKSIZE] = {0};
    uint32_t i=0;
    DataFlash_Read(uReadFlash,uSubCheckAddr,SUBCHECKSIZE); //����ʷ���ݴ洢����ʼλ�ÿ���ͷ����ʱ�꣨2+6����������    
//    DEBUGOUT("@@@@@@@@@@@@@@@@ uSubCheckAddr @@@@@@@@@@@@@@@@@@*********First before markCheck****\n");
/*    for (uint8_t j = 0; j < SUBCHECKSIZE; ++j)
    {
        DEBUGOUT("%x ", uSubCheckAddr[j]);
    }
    DEBUGOUT("\n@@@@@@@@@@@@@@@@ uSubCheckAddr @@@@@@@@@@@@@@@@@@*********First before markCheck****\n");
    */
    //Delay(1);
    for(; i<uTimes && uReadFlash<=(DATAFLASH_RECORD_END-uFrameLenth);i++ )
    {
        if(OK == hMarkCheck(uSubCheckAddr))
        {
 //           DEBUGOUT("\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ hMarkCheck Success \\\\\\\\\\\\\\\\\\\\\\\\\n");
            if(OK == tMarkCheck(uSubCheckAddr ,targetTime))
            {
                
                iRecord = i;
                *uNeedReadFlash = uReadFlash;
  //              DEBUGOUT("@@@@@@@@@@@@@@@@@@@@@@@@@@ tMarkCheck: %d @@@@@@@@@@@@@@@@@@@@@@@@@\n", iRecord);
                return iRecord;
            }
        }
        uReadFlash = uReadFlash + uFrameLenth;
        if(i/20)
        {
            FEED_WATCH_DOG();  // ι��
        }
        DataFlash_Read(uReadFlash,uSubCheckAddr,SUBCHECKSIZE); //����ʷ���ݴ洢����ʼλ�ÿ���ͷ����ʱ�꣨2+6����������
/*        DEBUGOUT("@@@@@@@@@@@@@@@@ uSubCheckAddr @@@@@@@@@@@@@@@@@@\n");
        for (uint8_t j = 0; j < SUBCHECKSIZE; ++j)
        {
            DEBUGOUT("%x ", uSubCheckAddr[j]);
        }
        DEBUGOUT("\n@@@@@@@@@@@@@@@@ uSubCheckAddr @@@@@@@@@@@@@@@@@@\n");
        */
    }
    return iRecord;

}
/******************************************************************************
* ��    �ƣ� CollectData()
* ��    �ܣ�����ƽ̨��Ҫ���ɵ�����
* ��ڲ�����
*	    uint16_t *uSubCheckAddr    ����ʷ�����ж�ȡ��ͷ���ʱ������
*           IEC104_MAIN_T *pA         IEC��Ϣָ��
*
* ���ڲ�����uint32_t uRecord ��Ҫ�ϱ���ʱ�������ʷ�����еĵڼ�֡
* ��    ��:
* ADD BY: chenyang
******************************************************************************/
int32_t CollectData(IEC104_MAIN_T *pA, uint32_t *uNeedReadFlash)
{

    int32_t iRecord = NO_DATA;
    uint8_t uRes;
    uint32_t uFrameLenth,uOver,uTimes;
    uint32_t uFirstReadFlash,ret1;
    time_t targetTime,tCurrentTime;
    //time_t inteval;

    struct tm dst = {0};
    struct tm cuTm = {0};
    float result = 0;
    IEC104_MAIN_T *recvpkt = pA;
    SYSTEMTIME *pTime;
    
    pTime = RealTimeGet();

    /*************************************************************
    //����ƽ̨���ʱ��ƫ����5���ӣ�������Ҫ�����ɵ�������ǰ����5����
    dst.tm_sec = 0;
    dst.tm_min = 5;	//5����ƫ��
    dst.tm_hour = 0;
    dst.tm_mday = 0;
    dst.tm_wday = 0;
    dst.tm_mon = 0;
    dst.tm_year = 0;
    dst.tm_isdst = 0;   //��ʵ������ʱ
    inteval = mktime(&dst);
    *************************************************************/
    //������890�����ɵ�һ֡ʱ�����
/*    DEBUGOUT("******************************************************************\n");
    DEBUGOUT("%x %x %x %x %x %x %x\n", recvpkt->recv.format.data[1],
                                       recvpkt->recv.format.data[2],
                                       recvpkt->recv.format.data[3],
                                       recvpkt->recv.format.data[4],
                                       recvpkt->recv.format.data[5],
                                       recvpkt->recv.format.data[6],
                                       recvpkt->recv.format.data[7]);
    DEBUGOUT("******************************************************************\n");
*/

    //�����ĵ���������7���ַ��������ʱ������
//    dst.tm_sec = (recvpkt->recv.format.data[1]+recvpkt->recv.format.data[2]*256)/1000;	//�Ͷ�ģʽ
//    dst.tm_min = recvpkt->recv.format.data[3];
//    dst.tm_hour = recvpkt->recv.format.data[4];
//    dst.tm_mday = recvpkt->recv.format.data[5]&0x1F;
//    dst.tm_wday = recvpkt->recv.format.data[5]&0xE0;
//    dst.tm_mon = recvpkt->recv.format.data[6];
//    dst.tm_year = recvpkt->recv.format.data[7]+100;//�������ر�ע�⣺�����Ǵ�1900�������������˴�Ҫ��src�����ֵ���Ժ��Ծ������
//    dst.tm_isdst = 0;   //��ʵ������ʱ

    dst.tm_sec = (g_TimeMarker[0]+g_TimeMarker[1]*256)/1000;	//�Ͷ�ģʽ
    dst.tm_min  = g_TimeMarker[2];
    dst.tm_hour = g_TimeMarker[3];
    dst.tm_mday = g_TimeMarker[4]&0x1F;
    dst.tm_wday = g_TimeMarker[4]&0xE0;
    dst.tm_mon  = g_TimeMarker[5];
    dst.tm_year = g_TimeMarker[6]+100;//�������ر�ע�⣺�����Ǵ�1900�������������˴�Ҫ��src�����ֵ���Ժ��Ծ������
    dst.tm_isdst = 0;   //��ʵ������ʱ
    targetTime = mktime(&dst);
    struct tm*tm_tar = localtime(&targetTime);
    
    DEBUGOUT("start-0: %d-%d-%d %d:%d:%d\n",tm_tar->tm_year+1900, 
                                            tm_tar->tm_mon+1,
                                            tm_tar->tm_mday, 
                                            tm_tar->tm_hour,
                                            tm_tar->tm_min, 
                                            tm_tar->tm_sec);

    uFrameLenth = g_DeviceSouth.yx_sum + g_DeviceSouth.yc_sum*4+8;
    if(uFrameLenth == 8)
    {
        return iRecord;
    }

    if(0 == sRecord.uExist)//�������һ������
    {
        uRes = EepReadData(EEP_LOGGER_104_RECORD_HEAD,(uint8_t *)&sRecord,sizeof(sRecord),&sRecord.CRC);// ���ɶ�ȡ
        if(0 == uRes)
        {
            sRecord.uFlashAddr = SearchRecordLastAddr(1,uFrameLenth);
        }
    }

    cuTm.tm_sec = pTime->Second;
    cuTm.tm_min = pTime->Minute;
    cuTm.tm_hour = pTime->Hour;
    cuTm.tm_mday = pTime->Date;
    cuTm.tm_mon = pTime->Month-1;
    cuTm.tm_year = pTime->Year+100;//�������ر�ע�⣺�����Ǵ�1900�������������˴�Ҫ��src�����ֵ���Ժ��Ծ������
    cuTm.tm_isdst = 0;   //��ʵ������ʱ
    tCurrentTime = mktime(&cuTm);

    //5����������432000����
    if(tCurrentTime<targetTime||tCurrentTime>targetTime+432000){
       return iRecord;
    }

    result = difftime(tCurrentTime,targetTime);
    ret1 = (uint32_t)((result/(2*INTERVAL_TIME_RANG)))*uFrameLenth; //��ʷ���ݵ���Ӧ֡������
    if(ret1<=sRecord.uFlashAddr)
    {

        DEBUGOUT("ret1<=sRecord.uFlashAddr");
        uFirstReadFlash = sRecord.uFlashAddr-(uint32_t)((result/(2*INTERVAL_TIME_RANG)))*uFrameLenth;
    }else
    {
        DEBUGOUT("ret1>sRecord.uFlashAddr");
        uOver = ret1 - (sRecord.uFlashAddr-DATAFLASH_RECORD_HEAD);
    }
    if(uFirstReadFlash>=DATAFLASH_RECORD_HEAD && ret1<=sRecord.uFlashAddr)
    {
//        DEBUGOUT("&&&&&&&&&&&&&&&&&&&&&&&&&&&uFirstReadFlash>=DATAFLASH_RECORD_HEAD && ret1<=sRecord.uFlashAddr&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n");
        uTimes = (uint32_t)result/(2*INTERVAL_TIME_RANG);
        iRecord = CheckTarget(uFirstReadFlash,targetTime,uFrameLenth,uTimes,uNeedReadFlash);
    }else
    {
        //��Ŀ���ַС����ʼ��ַ����Ҫ����β��
        if(ret1<=sRecord.uFlashAddr)
        {
            uOver = DATAFLASH_RECORD_HEAD - uFirstReadFlash;    //����������ĵ�ַ��dataflash����ʼ��ַ��ֵ
        }

        uFirstReadFlash = DATAFLASH_RECORD_END-uOver-(DATAFLASH_RECORD_END-DATAFLASH_RECORD_HEAD)%uFrameLenth;
        uTimes = uOver/uFrameLenth;
        iRecord = CheckTarget(uFirstReadFlash,targetTime,uFrameLenth,uTimes,uNeedReadFlash);
        if(NO_DATA == iRecord)
        {
            uFirstReadFlash = DATAFLASH_RECORD_HEAD;
            uTimes = (sRecord.uFlashAddr-DATAFLASH_RECORD_HEAD)/uFrameLenth;
            iRecord = CheckTarget(uFirstReadFlash,targetTime,uFrameLenth,uTimes,uNeedReadFlash);
        }
    }
    return iRecord;
}
