#ifndef _TASK2_H_
#define _TASK2_H_

#include "zf_common_headfile.h"

#define TASK2_TURN_TARGET   8     /* 目标转弯总次数 */
#define TASK2_T_SEQ_LEN     10    /* T字路口固定序列长度 */
#define TASK2_BEEP_TICKS   10     /* 蜂鸣器响铃时长 (10ms/tick) */

void Task2_Init(void);
void Task2_CountTurn(void);        /* 直角弯完成，计一次 */
void Task2_TComplete(void);        /* T字路口转弯完成，计一次 + 推进序列 */
void Task2_TSkip(void);            /* T字路口直行通过，不计转弯只推进序列 */
void Task2_Update(void);           /* 更新停车延迟 */
uint8 Task2_GetNextTDir(void);     /* 获取下一个T字路口方向: 0=右, 1=左, 2=直行 */
uint8 Task2_GetCount(void);
uint8 Task2_GetSeqIndex(void);
uint8 Task2_IsFinished(void);

#endif
