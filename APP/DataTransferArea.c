#include "DataTransferArea.h"

#include "GlobalVar.h"
#include "Memory.h"


/***********************************************************
���ļ�Ϊ���ݴ�������
***********************************************************/

//=======================================================================
#define    N_MESSAGES     23
OS_EVENT   *MesQ;                   // �����¼����ƿ�ָ�� ���е��¼����ƿ�ָ�� ���ڴ�Ŵ�������Ϣ���е�ָ��
void       *MsgGrp[N_MESSAGES];     // ������Ϣָ������

//uint16_t   g_uTransData;

//=======================================================================
//=======================================================================
// ����ṹ��
typedef struct LINK_LIST
{
    uint8_t *buff;      // ����ռ�
    uint8_t bytes;      // �����ֽ���
    struct LINK_LIST *next;
}LINK_LIST_T;

static LINK_LIST_T *LinkCreateNew(uint8_t uSize);
static uint16_t LinkGetLen(LINK_LIST_T *link);
static uint8_t LinkStoreData(LINK_LIST_T* *link,uint8_t *input,uint8_t bytes);
static uint8_t LinkReadData(LINK_LIST_T* *link,uint8_t *output);
static void LinkFree(LINK_LIST_T *del);
static void LinkDeleteAll(LINK_LIST_T* *head);
//=======================================================================
//=======================================================================
//ģ�������շ��������
static LINK_LIST_T *pLinkRecHead;   // �������ݵ�����ͷ
static LINK_LIST_T *pLinkSendHead;  // �������ݵ�����ͷ

OS_EVENT *pLinkRecLock;  // ���������д��
OS_EVENT *pLinkSendLock; // ���������д��
//OS_EVENT *pRecordLock;   // ��ʷ���ݴ洢��ȡ��
//=======================================================================


/*
// �ڴ�鶨��
#define MEM_BLOCK_NUM      50
#define MEM_BLOCK_SIZE     32
OS_MEM        *pMemBank;         // �����ڴ���ƿ�ָ�룬Ҳ����ָ���ڴ������ָ�룬����һ��
                                 // �ڴ����ʱ������ֵ������  OS_MEM �ڴ���ƿ����͵�ָ��
uint8_t       uMemBank[MEM_BLOCK_NUM][MEM_BLOCK_SIZE];  // ����һ������MEM_BLOCK_NUM���ڴ�飬ÿ���ڴ�鳤����MEM_BLOCK_SIZE���ֽڵ��ڴ����
//=======================================================================

// �ڴ���ʼ��
void MemBankInit(void)
{
    uint8_t err;
    pMemBank = OSMemCreate(uMemBank,MEM_BLOCK_NUM,MEM_BLOCK_SIZE,&err);    //������̬�ڴ��� ��������Ϊ��IntPartΪ�ڴ��������ʼ��ַ

    //ǰ���Ѿ�������uint8_t  uMemBank[MEM_BLOCK_NUM][MEM_BLOCK_SIZE];��ʾ�ڴ����������������ʾ��ʼ��ַ
    //�ڶ�������MEM_BLOCK_NUM��ʾ�������ڴ�����Ŀ������������MEM_BLOCK_SIZE��ʾÿ���ڴ����ֽ��������&errΪ������Ϣ
}*/
//=======================================================================
// ϵͳ�źų�ʼ��
void SysMesInit(void)
{
    //uint8_t err = 0;
    MesQ = OSQCreate(&MsgGrp[0],N_MESSAGES);       // ������Ϣ����
    //pRecordLock = OSMutexCreate(4,&err);           // ��ʷ���ݴ洢��ȡ��
}

//=======================================================================
//=======================================================================
//=======================================================================

/******************************************************************************
* ��    �ƣ�CmLinkInit()
* ��    �ܣ������ʼ��
* ��ڲ�����
*           ��
* ���ڲ�������
* ��    ��:
******************************************************************************/
void CmLinkInit(void)
{
    uint8_t err = 0;

    pLinkRecHead  = NULL;//LinkCreateNew(255);  // ������������
    pLinkSendHead = NULL;//LinkCreateNew(255);  // ������������

    pLinkRecLock  = OSMutexCreate(1,&err);
    pLinkSendLock = OSMutexCreate(2,&err);
}
/******************************************************************************
* ��    �ƣ�CmRecLinkLen()
* ��    �ܣ������������ֽ���
* ��ڲ�����
*           ��
* ���ڲ����������ֽ���
* ��    ��:
******************************************************************************/
uint16_t CmRecLinkLen(void)
{
    return LinkGetLen(pLinkRecHead);
}
/******************************************************************************
* ��    �ƣ�CmRecLinkLen()
* ��    �ܣ������������ֽ���
* ��ڲ�����
*           ��
* ���ڲ����������ֽ���
* ��    ��:
******************************************************************************/
uint16_t CmSendLinkLen(void)
{
    return LinkGetLen(pLinkSendHead);
}
/******************************************************************************
* ��    �ƣ�CmStoreRecLink()
* ��    �ܣ����ݴ洢����������
* ��ڲ�����
*           *input     ��������ͷָ��
            bytes      �ֽ���
* ���ڲ�������
* ��    ��:
******************************************************************************/
uint8_t CmStoreRecLink(uint8_t *input,uint8_t bytes)
{
    uint8_t err = 0;
    //�����ź���
    OSMutexPend(pLinkRecLock,0,&err);
    err =  LinkStoreData(&pLinkRecHead,input,bytes);
    //�ͷ��ź���
    OSMutexPost(pLinkRecLock);
    return err;
}
/******************************************************************************
* ��    �ƣ�CmReadRecLink()
* ��    �ܣ�����������
* ��ڲ�����
*           *output     ���������ͷָ��
* ���ڲ���������ȡ���ֽ���
* ��    ��:
******************************************************************************/
uint8_t CmReadRecLink(uint8_t *output)
{
    uint8_t err = 0;
    //�����ź���
    OSMutexPend(pLinkRecLock,0,&err);
    err =  LinkReadData(&pLinkRecHead,output);
    //�ͷ��ź���
    OSMutexPost(pLinkRecLock);
    return err;
}
/******************************************************************************
* ��    �ƣ�CM_store_send_link()
* ��    �ܣ����ݴ洢����������
* ��ڲ�����
*           *input     ��������ͷָ��
            bytes      �ֽ���
* ���ڲ�������
* ��    ��:
******************************************************************************/
uint8_t CmStoreSendLink(uint8_t *input,uint8_t bytes)
{
    uint8_t err = 0;
    //�����ź���
    OSMutexPend(pLinkSendLock,0,&err);
    err = LinkStoreData(&pLinkSendHead,input,bytes);
    //�ͷ��ź���
    OSMutexPost(pLinkSendLock);
    return err;
}
/******************************************************************************
* ��    �ƣ�CmReadSendLink()
* ��    �ܣ�����������
* ��ڲ�����
*           *output     ���������ͷָ��
* ���ڲ���������ȡ���ֽ���
* ��    ��:
******************************************************************************/
uint8_t CmReadSendLink(uint8_t *output)
{
    uint8_t err = 0;
    //�����ź���
    OSMutexPend(pLinkSendLock,0,&err);
    err =  LinkReadData(&pLinkSendHead,output);
    //�ͷ��ź���
    OSMutexPost(pLinkSendLock);
    return err;
}
/******************************************************************************
* ��    �ƣ�CmDelRecLink()
* ��    �ܣ�ɾ�����н�������
* ��ڲ�������
* ���ڲ�������
* ��    ��:
******************************************************************************/
void CmDelRecLink(void)
{
    uint8_t err = 0;
    //�����ź���
    OSMutexPend(pLinkRecLock,0,&err);
    LinkDeleteAll(&pLinkRecHead);
    //�ͷ��ź���
    OSMutexPost(pLinkRecLock);
}
/******************************************************************************
* ��    �ƣ�CmDelSendLink()
* ��    �ܣ�ɾ�����з�������
* ��ڲ�������
* ���ڲ�������
* ��    ��:
******************************************************************************/
void CmDelSendLink(void)
{
    uint8_t err = 0;
    //�����ź���
    OSMutexPend(pLinkSendLock,0,&err);
    LinkDeleteAll(&pLinkSendHead);
    //�ͷ��ź���
    OSMutexPost(pLinkSendLock);
}
//=======================================================================
//=======================================================================
//=======================================================================
//=======================================================================
//=======================================================================
/******************************************************************************
* ��    �ƣ�LinkStoreData()
* ��    �ܣ������������
* ��ڲ�����
*           **link      ����ͷָ���ָ��
            *input      ����������ָ��
            bytes       �������ֽ���

* ���ڲ�����0����������ʧ�ܣ�����Ϊ�����ֽ���
* ��    ��:LinkStoreData(link_recv_head,input,bytes);
******************************************************************************/
uint8_t LinkStoreData(LINK_LIST_T **link,uint8_t *input,uint8_t bytes)
{
    LINK_LIST_T *pnew,*ptail;//,*link;

    if(0==bytes)
    {
        return 0;
    }

    pnew = LinkCreateNew(bytes);//pnew = Link_Creat();  // ����һ������ڵ�
    if(NULL!=pnew)
    {
        memcpy(pnew->buff,(const uint8_t*)input,bytes);
        pnew->bytes = bytes;// ������
        pnew->next  = NULL;

        if(NULL==*link)  // ����û���κνڵ�
        {
            *link = pnew;
        }
        else
        {
            ptail = *link;

            while(NULL!=ptail->next)  // �ҳ�β�ڵ�
            {
                ptail = ptail->next;
            }

            ptail->next = pnew;
        }

        return bytes;
    }
    else
    {
        return 0;
    }
}
/******************************************************************************
* ��    �ƣ�LinkReadData()
* ��    �ܣ������������
* ��ڲ�����
*           **link      ����ͷָ���ָ��
            *output     ��Ŵ���ȡ����ָ��

* ���ڲ�����0����ȡ����ʧ�ܻ�û�����ݣ�����Ϊ��ȡ�ֽ���
* ��    ��:LinkReadData(link_recv_head,output);
******************************************************************************/
uint8_t LinkReadData(LINK_LIST_T **link,uint8_t *output)
{
    LINK_LIST_T *nlink;
    uint8_t rbytes;

    if(NULL==*link)
    {
        return 0;
    }

    nlink = (*link)->next;

    rbytes = (*link)->bytes;

    if(0!=rbytes)
    {
        memcpy((uint8_t*)output,(const uint8_t*)(*link)->buff,rbytes);
    }

    LinkFree(*link); // �ͷ��Ѿ�����ȡ���ݵĽڵ�ռ�

    *link = nlink;

    return rbytes;
}
/******************************************************************************
* ��    �ƣ�LinkGetLen()
* ��    �ܣ���ȡ������ֽ���
* ��ڲ�����
*           *link      ����������ͷ

* ���ڲ����������ֽ���
* ��    ��:
******************************************************************************/
uint16_t LinkGetLen(LINK_LIST_T *link)
{
    if(NULL==link)
    {
        return 0;
    }

    return link->bytes;
}
/******************************************************************************
* ��    �ƣ�LinkCreateNew()
* ��    �ܣ���������
* ��ڲ�����
*           uSize      ��������

* ���ڲ���������������ָ��
* ��    ��:
******************************************************************************/
LINK_LIST_T *LinkCreateNew(uint8_t uSize)
{
    LINK_LIST_T *head=NULL;
    uint8_t *data=NULL;

    data = (uint8_t *)WMemMalloc(data,uSize*sizeof(uint8_t));
    if(NULL==data)
    {
        DEBUGOUT("��������ʧ��\r\n");
        Reboot();// �������оƬ
        return NULL;
    }

    head = (LINK_LIST_T *)WMemMalloc(head,sizeof(LINK_LIST_T));

    if(NULL==head)
    {
        //free(data);
        WMemFree(data);
        DEBUGOUT("����ڵ�ͷʧ��\r\n");
        return NULL;
    }

    head->buff = data;
    head->next = NULL;
    head->bytes = 0x00;

    return head;
}

/******************************************************************************
* ��    �ƣ�Link_Creat()
* ��    �ܣ���������ڵ�
* ��ڲ�����
*           ��

* ���ڲ���������������ָ��
* ��    ��:
******************************************************************************/
/*void Link_Insert(LINK_LIST_T *head,LINK_LIST_T *pnew,uint8_t i)
{
    LINK_LIST_T *p;
    uint8_t j;

    p = head;

    for(j=0; j<i&&p!=NULL; j++)
    {
        p = p->next;
    }

    if(NULL==p)
    {
        return;
    }

    pnew->next = p->next;
    p->next = pnew;
}
*/
/******************************************************************************
* ��    �ƣ�Link_Delete()
* ��    �ܣ�ɾ������һ���ڵ�
* ��ڲ�����
*           ��

* ���ڲ���������������ָ��
* ��    ��:
******************************************************************************/
/*void Link_Delete(LINK_LIST_T *head,uint8_t i)
{
    LINK_LIST_T *p,*del;
    uint8_t j;

    if(0==i)  // ɾ������ͷָ��
    {
        return;
    }

    p = head;
    for(j=1;j<i&&p->next!=NULL;j++) // ��pָ��Ҫɾ���ĵ�i���ڵ��ǰ���ڵ�
    {
        p = p->next;
    }
    if(NULL==p->next) // ���������еĽڵ㲻����
    {
        return;
    }
    del = p->next;  // delָ���ɾ���Ľڵ�
    p->next = del->next;  // ɾ���ڵ�

    free(del);  // �ͷſռ�
}*/
/******************************************************************************
* ��    �ƣ�LinkFree()
* ��    �ܣ��ͷ�����һ���ڵ�ռ�
* ��ڲ�����
*           *del     ��ɾ������ڵ�ָ��

* ���ڲ�����
* ��    ��:
******************************************************************************/
void LinkFree(LINK_LIST_T *del)
{
    WMemFree(del->buff);
    WMemFree(del);  // �ͷſռ�
}

/******************************************************************************
* ��    �ƣ�Link_Delete_All()
* ��    �ܣ��ͷ���������ռ�
* ��ڲ�����
*           **head     ��ɾ������ͷָ���ָ��

* ���ڲ���������������ָ��
* ��    ��:
******************************************************************************/
void LinkDeleteAll(LINK_LIST_T **head)
{
    LINK_LIST_T *p,*del;

    p = *head;

    if(NULL!=*head)
    {
        while(NULL!=p->next)
        {
            del = p->next;
            p->next = del->next;
            LinkFree(del);// ɾ���ڵ�
        }

        LinkFree(*head);// ɾ���ڵ�
        *head = NULL;
    }
}
