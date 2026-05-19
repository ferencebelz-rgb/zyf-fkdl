#ifndef _TASK2_H_
#define _TASK2_H_

#include "zf_common_headfile.h"

#define TASK2_TURN_TARGET  8

typedef enum
{
    DECISION_LEFT     = -1,
    DECISION_STRAIGHT =  0,
    DECISION_RIGHT    =  1,
} route_decision_t;

void Task2_Init(void);
void Task2_CountTurn(void);
void Task2_Update(void);
route_decision_t Task2_GetRouteDecision(void);
uint8 Task2_GetCount(void);
uint8 Task2_IsFinished(void);

#endif
