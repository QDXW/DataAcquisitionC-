#include "Memory.h"

#include "ucos_ii.h"

#include <string.h>
//#include <stdlib.h>

//#define NULL ((void *) 0)

#define MEM_BLOCK_SIZE          32                             // �ڴ���С
#define MAX_MEM_SIZE            17408                          // ���ڴ��С 17KB
#define MEM_ALLOC_TABLE_SIZE    (MAX_MEM_SIZE/MEM_BLOCK_SIZE)  // �ڴ���С
static char memory_map[MEM_ALLOC_TABLE_SIZE]={0};   //�ڴ��
static unsigned char memory_heap[MAX_MEM_SIZE];     //���ڴ�

/*
* @fn malloc
* @param unsigned int size ������ڴ��С
* @return ���뵽���ڴ��ָ��
*/
/******************************************************************************
* ��    �ƣ�WMemMalloc()
* ��    �ܣ����붯̬�ڴ�
* ��ڲ�����
            *ptr         �����붯̬�ڴ��ָ��
            size         �ڴ��С���ֽ�
*
* ���ڲ���������ɹ�����ָ�룬����ʧ�ܷ���NULL
* ��    ��:
******************************************************************************/
void *WMemMalloc(void *ptr,unsigned int size)
{
    int offset;
    int nmemb;
    int i;
    int vacantSize;

    OS_CPU_SR  cpu_sr;

    if(size==0)
    {
        return NULL;
    }

    if(NULL!=ptr)
    {
        ptr = WMemFree(ptr);  // ���ͷ��ڴ棬����ͬһ��ָ���ظ������ڴ�ռ�
    }

    OS_ENTER_CRITICAL();

    nmemb=size/MEM_BLOCK_SIZE;  //��Ҫ���ڴ�����
    if(size%MEM_BLOCK_SIZE)
    {
        nmemb++;
    }

    for(offset=MEM_ALLOC_TABLE_SIZE-1;offset>=0;offset--)  //�Ӻ���ǰ���õ����ڴ�����ڴ��������Ͽ�����ı��
    {
        if(!memory_map[offset] && ((offset+nmemb)<=MEM_ALLOC_TABLE_SIZE))
        {
            vacantSize=0;
            for(vacantSize=0;vacantSize<nmemb && !memory_map[offset+vacantSize];vacantSize++);
            if(vacantSize==nmemb)
            {
                for(i=0;i<nmemb;i++)
                {
                    memory_map[offset+i]=nmemb;
                }
                memset(memory_heap+(offset*MEM_BLOCK_SIZE),0,size);

                OS_EXIT_CRITICAL();

                return memory_heap+(offset*MEM_BLOCK_SIZE);  //������ʼ��ĵ�ַ
            }
        }
    }
    OS_EXIT_CRITICAL();
    return NULL;
}
/*
* @fn free
* @param void *ptr �ͷ��ڴ��׵�ַ
* @return void
*/
/******************************************************************************
* ��    �ƣ�WMemFree()
* ��    �ܣ����붯̬�ڴ�
* ��ڲ�����
            *ptr         �����붯̬�ڴ��ָ��
*
* ���ڲ���������NULL
* ��    ��:
******************************************************************************/
void *WMemFree(void *ptr)
{
    int offset;
    int i;
    int index;
    int nmemb;

    OS_CPU_SR  cpu_sr;

    if(ptr==NULL)
    {
        return NULL;
    }

    if((int)ptr < (int)memory_heap)
    {
        return NULL;
    }

    OS_ENTER_CRITICAL();

    offset=(int)ptr-(int)memory_heap; //���ڴ���е�ƫ��
    if(offset<MAX_MEM_SIZE)
    {
        index=offset/MEM_BLOCK_SIZE;
        nmemb=memory_map[index];
        for(i=0;i<nmemb;i++)
        {
            memory_map[index+i]=0;  //�ͷŵ��ڴ��ڱ�����0
        }
    }

    OS_EXIT_CRITICAL();
    return NULL;
}
/******************************************************************************
* ��    �ƣ�FreeRamSpace()
* ��    �ܣ�ͳ�ƶ�̬�ڴ�ʣ���ֽ���
* ��ڲ�����
            ��
*
* ���ڲ��������ؿ��õ��ڴ�����
* ��    ��:
******************************************************************************/
uint16_t FreeRamSpace(void)
{
    uint16_t uFreeSpace=0;
    uint16_t i;
    for(i=0;i<MEM_ALLOC_TABLE_SIZE;i++)
    {
        if(0==memory_map[i])
        {
            uFreeSpace++;
        }
    }

    return uFreeSpace*MEM_BLOCK_SIZE;
}

