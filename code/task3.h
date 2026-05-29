#ifndef _TASK3_H_
#define _TASK3_H_

#include "zf_common_headfile.h"

#define TASK3_TURN_TARGET   14
#define TASK3_T_SEQ_LEN     15
#define TASK3_BEEP_TICKS    10

void Task3_Init(void);
void Task3_CountTurn(void);
void Task3_TComplete(void);
void Task3_TSkip(void);
void Task3_Update(void);
uint8 Task3_GetNextTDir(void);
uint8 Task3_GetCount(void);
uint8 Task3_GetSeqIndex(void);
uint8 Task3_IsFinished(void);
float Task3_GetExitYaw(void);

#endif
