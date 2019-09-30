#ifndef __SWITCH_SCAN_H__
#define __SWITCH_SCAN_H__

#ifndef _STDINT
#include <stdint.h>
#endif

typedef  uint16_t   SWITCHINT;

//���°������ڵ�λ����Ҫ����ʵ�ʵ�·����
/*#define S01     0x0001
#define S02     0x0002
#define S03     0x0004
#define S04     0x0008
#define S05     0x0010
#define S06     0x0020
#define S07     0x0040
#define S08     0x0080
#define S09     0x0100
#define S10     0x0200
#define S11     0x0400
#define S12     0x0800
#define S13     0x1000
#define S14     0x2000
#define S15     0x4000
#define S16     0x8000
*/
#define S01     0x0800
#define S02     0x0400
#define S03     0x0200
#define S04     0x0100
#define S05     0x1000
#define S06     0x2000
#define S07     0x8000
#define S08     0x4000
#define S09     0x0001
#define S10     0x0002
#define S11     0x0004
#define S12     0x0008
#define S13     0x0020
#define S14     0x0010
#define S15     0x0040
#define S16     0x0080

extern SWITCHINT  SwitchDown;  // ���´���
extern SWITCHINT  SwitchUp;    // �ɿ�����
extern SWITCHINT  SwitchOn;    // ������ס


extern  void SwitchScan(void);
extern  void SWITCH_IO_Init(void);

#endif
