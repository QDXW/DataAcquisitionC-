#ifndef __CM_H_
#define __CM_H_

#include "ucos_ii.h"

#define TaskModemProcessStkSize   256   // ��ѯ�����ջ��С
extern OS_STK TaskModemProcessStk[TaskModemProcessStkSize];	      // ���������ջ��С


extern void TaskModemProcess(void *p);

#endif
