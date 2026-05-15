#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "zf_common_headfile.h"

#define CONTROL_STRAIGHT_ONLY           0
#define CONTROL_TURN_SIGN               1

#define CONTROL_BASE_SPEED_DEFAULT      288
#define CONTROL_BASE_SPEED_MAX          600
#define CONTROL_BASE_RAMP_STEP          4

#define CONTROL_TURN_KP_DEFAULT         0.8f
#define CONTROL_TURN_KD_DEFAULT         0.2f
#define CONTROL_TURN_LIMIT_DEFAULT      120
#define CONTROL_GYRO_DAMPING_GAIN       5.0f

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

#endif
