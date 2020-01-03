#include "tool.h"

#define ABS(cond) (cond>0?cond:-cond)

/****************************************************************************
* ��    �ƣ�ToolDelay()
* ��    �ܣ�����ʱ��
* ��ڲ�����
*           uInDelay     ������ʱʱ�䣬���ٸ�
* ���ڲ�������
* ��    ��: ��
****************************************************************************/
void ToolDelay(uint16_t uInDelay)
{
    // 10000ԼΪ3.1mS��
    volatile uint16_t uDelay;

    uDelay = uInDelay;

    while(uDelay--);
}
/****************************************************************************
* ��    NoTimeOut()
* ��    �ܣ��ж�ʱ���
* ��ڲ�����
*         sStart����ʼʱ��
*         sNow��  ���ڵ�ʱ��
*         uGap:�� ʱ���ֵ
* ���ڲ�����
          δ�ﵽʱ���� NOTIMEOUT���ﵽʱ����TIMEOUT
* ��    ��: ��
****************************************************************************/
int8_t NoTimeOut(uint32_t sStart,uint32_t sNow,uint32_t uGap)
{
    // ucos��tickΪ32λ��������ʱ��0xFFFFFFFF����ֹ������ж���ʼʱ�������ʱ��Ĵ�С
    if(((sNow>=sStart)?(sNow - sStart):(0xFFFFFFFF-sStart+sNow)) < (uGap/10))   //if((sNow - sStart)<(uGap/10))
    {
        return NOTIMEOUT;
    }

    return TIMEOUT;
}
/****************************************************************************
* ��    TimeOut()
* ��    �ܣ��ж�ʱ���
* ��ڲ�����
*         sStart����ʼʱ��
*         sNow��  ���ڵ�ʱ��
*         uGap:�� ʱ���ֵ
* ���ڲ�����
          �ﵽʱ���� 1��δ�ﵽʱ����0
* ��    ��: ��
****************************************************************************/
int8_t TimeOut(uint32_t sStart,uint32_t sNow,uint32_t uGap)
{
    // ucos��tickΪ32λ��������ʱ��0xFFFFFFFF����ֹ������ж���ʼʱ�������ʱ��Ĵ�С
    if(((sNow>=sStart)?(sNow - sStart):(0xFFFFFFFF-sStart+sNow)) > (uGap/10))   //if((sNow - sStart)<(uGap/10))
    {
        return TIMEOUT;
    }

    return NOTIMEOUT;
}
/******************************************************************************
* ��    �ƣ�Str2IP()
* ��    �ܣ��ҳ��ַ����е�IP��ַ��ת��Ϊ����
* ��ڲ�����
*           *str     �����ַ���
            *ip      ���IP��ַ������ָ��

* ���ڲ�������
* ��    ��:
******************************************************************************/
void Str2IP(const char* str,uint8_t *ip)
{
    uint8_t i,k;
    uint8_t iptemp;
    for(i=0;i<18;i++)
    {
        if('\"'==str[i])
        {
            break;
        }
    }
    for(k=0; k<4; k++)
    {
        i++;
        iptemp = str[i] - '0';
        i++;
        if('.'==str[i] || '\"'==str[i])
        {
            ip[k] = iptemp;
            continue;
        }
        else
        {
            iptemp = iptemp*10 + str[i] - '0';
        }
        i++;
        if('.'==str[i] || '\"'==str[i])
        {
            ip[k] = iptemp;
            continue;
        }
        else
        {
            iptemp = iptemp*10 + str[i] - '0';
            ip[k] = iptemp;
        }

        i++;
        if('.'==str[i] || '\"'==str[i])
        {
            continue;
        }
    }
}

/******************************************************************************
* ��    �ƣ�Str2UI()
* ��    �ܣ��ҳ��ַ����е����ֲ�ת��Ϊ����
* ��ڲ�����
*           *str     �����ַ���
            *ip      ���IP��ַ������ָ��

* ���ڲ�����ʧ�ܷ���1
* ��    ��:
******************************************************************************/
uint8_t Str2UI(const char* str,uint16_t *result)
{
    uint8_t i,k=1;
    uint16_t temp;

    for(i=0;i<20;i++)
    {
        if('\"'==str[i])
        {
            if(k)
            {
                k = 0;
                temp = 0;
                continue;
            }
            else
            {
                *result = temp;
                return 0;
            }
        }
        else if(str[i]>'9' || str[i]<'0')
        {
            continue;//return 1;
        }
        else
        {
            temp = temp*10 + (str[i]-'0');
        }
    }
    return 1;
}
/******************************************************************************
* ��    �ƣ�UI2Str()
* ��    �ܣ��ҳ��ַ����е����ֲ�ת��Ϊ����
* ��ڲ�����
*           in           ��������
            *result      ����ַ�����ָ��

* ���ڲ������ַ���ָ��
* ��    ��:
******************************************************************************/
char *I2Str(const uint16_t in,char* result)
{
    char str[7]={'\0'};
    uint16_t temp;
    int8_t i,j;


    temp = ABS(in);
    /*if(in>99999)
    {
        return 1;
    }*/

    for(i=0;i<5;i++)
    {
        str[i] = (temp%10)+'0';
        temp /= 10;
        if(0==temp)
        {
            break;
        }
    }

    /*if(in<0)
    {
        str[++i] = '-';
    }*/

    for(j=0;i>=0;i--)
    {
        result[j++] = str[i];
    }

    result[j] = '\0';

    return result;
}
/******************************************************************************
* ��    �ƣ�UI2Str()
* ��    �ܣ��ҳ��ַ����е����ֲ�ת��Ϊ����
* ��ڲ�����
*           in           ��������
            *result      ����ַ�����ָ��

* ���ڲ������ַ���ָ��
* ��    ��:
******************************************************************************/
/*char * strset(char *s, char c,uint8_t byts)
{
    uint8_t i;

    for(i=0;i<bytes;i++)
    {
        s[
    }
}*/
