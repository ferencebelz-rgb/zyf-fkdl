#ifndef _TASK1_H_
#define _TASK1_H_

#include "zf_common_headfile.h"

#define TASK1_TURN_TARGET  8     /* 目标直角弯个数 */

void Task1_Init(void);
void Task1_Update(void);
uint8 Task1_GetCount(void);
uint8 Task1_IsFinished(void);

#endif
