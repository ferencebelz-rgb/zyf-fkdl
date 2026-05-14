#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "zf_common_headfile.h"

#define CONTROL_STRAIGHT_ONLY           0
#define CONTROL_TURN_SIGN               1

#define CONTROL_BASE_SPEED_DEFAULT      288
#define CONTROL_BASE_SPEED_MIN          0
#define CONTROL_BASE_SPEED_MAX          600
#define CONTROL_BASE_SPEED_STEP         10
#define CONTROL_BASE_RAMP_STEP          4

#define CONTROL_TURN_KP_DEFAULT         0.8f
#define CONTROL_TURN_KD_DEFAULT         0.2f
#define CONTROL_TURN_KP_MIN             0.0f
#define CONTROL_TURN_KP_MAX             5.0f
#define CONTROL_TURN_KP_STEP            0.1f
#define CONTROL_TURN_KD_MIN             0.0f
#define CONTROL_TURN_KD_MAX             5.0f
#define CONTROL_TURN_KD_STEP            0.1f
#define CONTROL_TURN_LIMIT_DEFAULT      120

typedef enum
{
    CONTROL_PARAM_BASE_SPEED = 0,
    CONTROL_PARAM_SPEED_KP,
    CONTROL_PARAM_SPEED_KI,
    CONTROL_PARAM_TURN_KP,
    CONTROL_PARAM_TURN_KD,
    CONTROL_PARAM_COUNT
} ControlParamPage_e;

void Control_Init(void);
void Control_Task10ms(void);
void Control_DisplayStatusTask(void);
void Control_ClearFault(void);

int16 Control_GetBaseSpeedCommand(void);
int16 Control_GetBaseSpeedTarget(void);
int16 Control_GetTargetL(void);
int16 Control_GetTargetR(void);
int16 Control_GetTurnOutput(void);
uint8 Control_GetFaultStop(void);
ControlParamPage_e Control_GetParamPage(void);
float Control_GetSpeedKp(void);
float Control_GetSpeedKi(void);
float Control_GetTurnKp(void);
float Control_GetTurnKd(void);

#endif
