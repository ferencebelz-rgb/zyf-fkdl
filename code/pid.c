#include "pid.h"

void IncrementalPI_Init(IncrementalPI_t *pid, float kp, float ki)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->last_error = 0.0f;
}

void IncrementalPI_SetParam(IncrementalPI_t *pid, float kp, float ki)
{
    pid->kp = kp;
    pid->ki = ki;
}

void IncrementalPI_Reset(IncrementalPI_t *pid)
{
    pid->last_error = 0.0f;
}

float IncrementalPI_Calculate(IncrementalPI_t *pid, float error)
{
    float delta = pid->kp * (error - pid->last_error) + pid->ki * error;

    pid->last_error = error;
    return delta;
}

void PositionPD_Init(PositionPD_t *pid, float kp, float kd)
{
    pid->kp = kp;
    pid->kd = kd;
    pid->last_error = 0.0f;
}

void PositionPD_SetParam(PositionPD_t *pid, float kp, float kd)
{
    pid->kp = kp;
    pid->kd = kd;
}

void PositionPD_Reset(PositionPD_t *pid)
{
    pid->last_error = 0.0f;
}

float PositionPD_Calculate(PositionPD_t *pid, float error)
{
    float output = pid->kp * error + pid->kd * (error - pid->last_error);

    pid->last_error = error;
    return output;
}
