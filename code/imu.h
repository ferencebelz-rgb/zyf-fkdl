#ifndef _IMU_H_
#define _IMU_H_

#include "zf_common_headfile.h"

extern uint8 attitude_flag;
extern uint8 imu_attitude_ready;

extern float gyro[3];
extern float accel[3];
extern float gyro_correct[3];

extern float roll_mahony;
extern float pitch_mahony;
extern float yaw_mahony;

extern float imu_roll;
extern float imu_pitch;
extern float imu_yaw;

void imu_init(void);
void imu_task(void);

void INS_Task(void);
void IMU_task(void);

void Mahony_Init(float sampleFrequency);
void Mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void Mahony_computeAngles(void);
void MahonyAHRSinit(float ax, float ay, float az, float mx, float my, float mz);

float getRoll(void);
float getPitch(void);
float getYaw(void);
float getRollRadians(void);
float getPitchRadians(void);
float getYawRadians(void);

void printf_data(void);

#endif
