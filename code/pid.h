#ifndef _PID_H_
#define _PID_H_

#include "zf_common_headfile.h"

typedef struct
{
    float kp;
    float ki;
    float last_error;
} IncrementalPI_t;

typedef struct
{
    float kp;
    float kd;
    float last_error;
} PositionPD_t;

typedef PositionPD_t TrackAnglePD_t;

void IncrementalPI_Init(IncrementalPI_t *pid, float kp, float ki);
void IncrementalPI_SetParam(IncrementalPI_t *pid, float kp, float ki);
void IncrementalPI_Reset(IncrementalPI_t *pid);
float IncrementalPI_Calculate(IncrementalPI_t *pid, float error);

void PositionPD_Init(PositionPD_t *pid, float kp, float kd);
void PositionPD_SetParam(PositionPD_t *pid, float kp, float kd);
void PositionPD_Reset(PositionPD_t *pid);
float PositionPD_Calculate(PositionPD_t *pid, float error);

#endif
