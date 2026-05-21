#ifndef _TASK1_H_
#define _TASK1_H_

#include "zf_common_headfile.h"

#define TASK1_TURN_TARGET  8     /* 目标转弯总次数 */
#define TASK1_T_SEQ_LEN    6     /* T字路口固定序列长度 */

void Task1_Init(void);
void Task1_CountTurn(void);        /* 直角弯完成，计一次 */
void Task1_TComplete(void);        /* T字路口转弯完成，计一次 + 推进序列 */
void Task1_TSkip(void);            /* T字路口直行通过，不计转弯只推进序列 */
uint8 Task1_GetNextTDir(void);     /* 获取下一个T字路口方向: 0=右, 1=左, 2=直行 */
uint8 Task1_GetCount(void);
uint8 Task1_IsFinished(void);

#endif
