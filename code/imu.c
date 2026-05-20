#include "imu.h"

#define IMU_SAMPLE_HZ              1000.0f
#define IMU_SAMPLE_US              1000u
#define IMU_CALIBRATION_SAMPLES    1000u

#define DEG_TO_RAD                 0.01745329252f
#define RAD_TO_DEG                 57.29577951308f
#define MAHONY_TWOKP_DEF           (2.0f * 0.5f)
#define MAHONY_TWOKI_DEF           (2.0f * 0.0f)

float twoKi = MAHONY_TWOKI_DEF;
float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
float integralFBx = 0.0f;
float integralFBy = 0.0f;
float integralFBz = 0.0f;
float invSampleFreq = 1.0f / IMU_SAMPLE_HZ;

float roll_mahony = 0.0f;
float pitch_mahony = 0.0f;
float yaw_mahony = 0.0f;

float imu_roll = 0.0f;
float imu_pitch = 0.0f;
float imu_yaw = 0.0f;

float gyro[3] = {0.0f, 0.0f, 0.0f};
float accel[3] = {0.0f, 0.0f, 0.0f};
float gyro_correct[3] = {0.0f, 0.0f, 0.0f};
float temp = 0.0f;

uint8 anglesComputed = 0;
uint8 attitude_flag = 0;
uint8 imu_attitude_ready = 0;

static uint8 imu_init_ok = 0;
static uint8 mahony_inited = 0;
static uint32 correct_times = 0;

static float Mahony_invSqrt(float x)
{
    if(x <= 0.0f)
    {
        return 0.0f;
    }
    return 1.0f / sqrtf(x);
}

void imu_init(void)
{
    uint8 i;

    Mahony_Init(IMU_SAMPLE_HZ);

    for(i = 0; i < 3; i++)
    {
        gyro[i] = 0.0f;
        accel[i] = 0.0f;
        gyro_correct[i] = 0.0f;
    }

    roll_mahony = 0.0f;
    pitch_mahony = 0.0f;
    yaw_mahony = 0.0f;
    imu_roll = 0.0f;
    imu_pitch = 0.0f;
    imu_yaw = 0.0f;
    correct_times = 0;
    attitude_flag = 0;
    imu_attitude_ready = 0;
    mahony_inited = 0;

    imu_init_ok = (0 == imu660rb_init());
}

void imu_task(void)
{
    static uint32 last_time_us = 0;
    uint32 now_time_us;
    uint32 delta_time_us;

    if(!imu_init_ok)
    {
        return;
    }

    now_time_us = system_getval_us();
    if(0 == last_time_us)
    {
        last_time_us = now_time_us;
        return;
    }

    delta_time_us = (uint32)(now_time_us - last_time_us);
    if(delta_time_us < IMU_SAMPLE_US)
    {
        return;
    }
    last_time_us = now_time_us;
    invSampleFreq = (float)delta_time_us * 0.000001f;

    INS_Task();
}

void IMU_task(void)
{
    imu_task();
}

void INS_Task(void)
{
    imu660rb_get_gyro();
    imu660rb_get_acc();

    gyro[0] = imu660rb_gyro_transition(imu660rb_gyro_x) * DEG_TO_RAD;
    gyro[1] = imu660rb_gyro_transition(imu660rb_gyro_y) * DEG_TO_RAD;
    gyro[2] = imu660rb_gyro_transition(imu660rb_gyro_z) * DEG_TO_RAD;

    accel[0] = imu660rb_acc_transition(imu660rb_acc_x);
    accel[1] = imu660rb_acc_transition(imu660rb_acc_y);
    accel[2] = imu660rb_acc_transition(imu660rb_acc_z);

    if(!mahony_inited)
    {
        MahonyAHRSinit(accel[0], accel[1], accel[2], 0.0f, 0.0f, 0.0f);
        mahony_inited = 1;
    }

    if(attitude_flag)
    {
        Mahony_update(gyro[0] - gyro_correct[0],
                      gyro[1] - gyro_correct[1],
                      gyro[2] - gyro_correct[2],
                      accel[0],
                      accel[1],
                      accel[2],
                      0.0f,
                      0.0f,
                      0.0f);
        Mahony_computeAngles();
        imu_attitude_ready = 1;
    }
    else
    {
        gyro_correct[0] += gyro[0];
        gyro_correct[1] += gyro[1];
        gyro_correct[2] += gyro[2];
        correct_times++;

        if(correct_times >= IMU_CALIBRATION_SAMPLES)
        {
            gyro_correct[0] /= (float)correct_times;
            gyro_correct[1] /= (float)correct_times;
            gyro_correct[2] /= (float)correct_times;
            attitude_flag = 1;
            imu_attitude_ready = 1;
        }
    }
}

void Mahony_Init(float sampleFrequency)
{
    twoKi = MAHONY_TWOKI_DEF;
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
    invSampleFreq = 1.0f / sampleFrequency;
    anglesComputed = 0;
}

void MahonyAHRSinit(float ax, float ay, float az, float mx, float my, float mz)
{
    float roll;
    float pitch;
    float yaw;
    float cy;
    float sy;
    float cr;
    float sr;
    float cp;
    float sp;

    (void)mx;
    (void)my;
    (void)mz;

    if((ax != 0.0f) || (ay != 0.0f) || (az != 0.0f))
    {
        roll = atan2f(ay, az);
        pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
        yaw = 0.0f;

        cy = cosf(yaw * 0.5f);
        sy = sinf(yaw * 0.5f);
        cr = cosf(roll * 0.5f);
        sr = sinf(roll * 0.5f);
        cp = cosf(pitch * 0.5f);
        sp = sinf(pitch * 0.5f);

        q0 = cy * cr * cp + sy * sr * sp;
        q1 = cy * sr * cp - sy * cr * sp;
        q2 = cy * cr * sp + sy * sr * cp;
        q3 = sy * cr * cp - cy * sr * sp;
        anglesComputed = 0;
    }
}

void Mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float recipNorm;
    float q0q0;
    float q0q1;
    float q0q2;
    float q0q3;
    float q1q1;
    float q1q2;
    float q1q3;
    float q2q2;
    float q2q3;
    float q3q3;
    float hx;
    float hy;
    float bx;
    float bz;
    float halfvx;
    float halfvy;
    float halfvz;
    float halfwx;
    float halfwy;
    float halfwz;
    float halfex;
    float halfey;
    float halfez;
    float qa;
    float qb;
    float qc;

    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        recipNorm = Mahony_invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        recipNorm = Mahony_invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        if(twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * invSampleFreq;
            integralFBy += twoKi * halfey * invSampleFreq;
            integralFBz += twoKi * halfez * invSampleFreq;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        gx += MAHONY_TWOKP_DEF * halfex;
        gy += MAHONY_TWOKP_DEF * halfey;
        gz += MAHONY_TWOKP_DEF * halfez;
    }

    gx *= (0.5f * invSampleFreq);
    gy *= (0.5f * invSampleFreq);
    gz *= (0.5f * invSampleFreq);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    recipNorm = Mahony_invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    anglesComputed = 0;
}

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float halfvx;
    float halfvy;
    float halfvz;
    float halfex;
    float halfey;
    float halfez;
    float qa;
    float qb;
    float qc;

    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        recipNorm = Mahony_invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        if(twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * invSampleFreq;
            integralFBy += twoKi * halfey * invSampleFreq;
            integralFBz += twoKi * halfez * invSampleFreq;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        gx += MAHONY_TWOKP_DEF * halfex;
        gy += MAHONY_TWOKP_DEF * halfey;
        gz += MAHONY_TWOKP_DEF * halfez;
    }

    gx *= (0.5f * invSampleFreq);
    gy *= (0.5f * invSampleFreq);
    gz *= (0.5f * invSampleFreq);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    recipNorm = Mahony_invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    anglesComputed = 0;
}

void Mahony_computeAngles(void)
{
    float sin_pitch;

    sin_pitch = -2.0f * (q1 * q3 - q0 * q2);
    if(sin_pitch > 1.0f)
    {
        sin_pitch = 1.0f;
    }
    else if(sin_pitch < -1.0f)
    {
        sin_pitch = -1.0f;
    }

    roll_mahony = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * RAD_TO_DEG;
    pitch_mahony = asinf(sin_pitch) * RAD_TO_DEG;
    yaw_mahony = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * RAD_TO_DEG;

    imu_roll = roll_mahony;
    imu_pitch = pitch_mahony;
    imu_yaw = yaw_mahony;
    anglesComputed = 1;
}

float getRoll(void)
{
    if(!anglesComputed)
    {
        Mahony_computeAngles();
    }
    return roll_mahony;
}

float getPitch(void)
{
    if(!anglesComputed)
    {
        Mahony_computeAngles();
    }
    return pitch_mahony;
}

float getYaw(void)
{
    if(!anglesComputed)
    {
        Mahony_computeAngles();
    }
    return yaw_mahony;
}

float getRollRadians(void)
{
    return getRoll() * DEG_TO_RAD;
}

float getPitchRadians(void)
{
    return getPitch() * DEG_TO_RAD;
}

float getYawRadians(void)
{
    return getYaw() * DEG_TO_RAD;
}

void printf_data(void)
{
    printf("%f,%f,%f\r\n", getRoll(), getPitch(), getYaw());
}
