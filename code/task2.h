#ifndef _TASK2_H_
#define _TASK2_H_

#include "zf_common_headfile.h"

#define TASK2_TURN_TARGET  8     /* 目标总转弯个数 */

/* T字路口转弯方向 */
typedef enum {
    TURN_LEFT     = -1,
    TURN_STRAIGHT =  0,
    TURN_RIGHT    =  1,
} turn_dir_t;

void Task2_Init(void);
void Task2_CountTurn(void);
void Task2_Update(void);
void Task2_CheckSW2(void);          /* 检测 SW2 (KEY_2) 短按，触发计数 */
int8 Task2_GetTJunTurnDir(void);    /* T字路口转弯方向，相机检测到T字路口时调用 */
uint8 Task2_GetCount(void);
uint8 Task2_IsFinished(void);

#endif
