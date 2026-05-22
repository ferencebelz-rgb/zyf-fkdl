#include "control.h"
#include "camera.h"
#include "imu.h"
#include "motor.h"
#include "pid.h"
#include "task1.h"
#include "zf_device_ips200.h"
#include <stdlib.h>

static IncrementalPI_t speed_pid_l;
static IncrementalPI_t speed_pid_r;
static TrackAnglePD_t turn_pid;

static int16 base_speed_cmd = CONTROL_BASE_SPEED_DEFAULT;
static int16 base_speed_target = 0;
static int16 target_l = 0;
static int16 target_r = 0;
static int16 turn_output = 0;
static int16 pwm_l = 0;
static int16 pwm_r = 0;

static float speed_kp = SPEED_KP_DEFAULT;
static float speed_ki = SPEED_KI_DEFAULT;
static float turn_kp = CONTROL_TURN_KP_DEFAULT;
static float turn_kd = CONTROL_TURN_KD_DEFAULT;

static uint8 fault_stop = 0;
static uint16 start_ticks = 0;  /* 起步计时 (10ms/tick) */

static uint16 stall_count_l = 0;
static uint16 stall_count_r = 0;
static uint16 cooldown_l = 0;
static uint16 cooldown_r = 0;

static int16 clamp_i16(int32 value, int16 min_value, int16 max_value)
{
    if(value < min_value) return min_value;
    if(value > max_value) return max_value;
    return (int16)value;
}

static void control_reset_runtime(void)
{
    target_l = 0;
    target_r = 0;
    turn_output = 0;
    pwm_l = 0;
    pwm_r = 0;
    stall_count_l = 0;
    stall_count_r = 0;
    cooldown_l = 0;
    cooldown_r = 0;
    IncrementalPI_Reset(&speed_pid_l);
    IncrementalPI_Reset(&speed_pid_r);
    PositionPD_Reset(&turn_pid);
    Motor_Stop();
}

void Control_Init(void)
{
    IncrementalPI_Init(&speed_pid_l, speed_kp, speed_ki);
    IncrementalPI_Init(&speed_pid_r, speed_kp, speed_ki);
    PositionPD_Init(&turn_pid, turn_kp, turn_kd);
    Task1_Init();
    start_ticks = 0;
    control_reset_runtime();
}

void Control_ClearFault(void)
{
    fault_stop = 0;
    control_reset_runtime();
}

static int16 approach_i16(int16 current, int16 target, int16 step)
{
    if(current < target)
    {
        current += step;
        if(current > target) current = target;
    }
    else if(current > target)
    {
        current -= step;
        if(current < target) current = target;
    }
    return current;
}

static int16 pwm_decay_to_zero(int16 pwm)
{
    if(pwm > OVERSPEED_PWM_DECAY) return pwm - OVERSPEED_PWM_DECAY;
    if(pwm < -OVERSPEED_PWM_DECAY) return pwm + OVERSPEED_PWM_DECAY;
    return 0;
}

static int16 pwm_ramp_limit(int16 target_pwm, int16 last_pwm)
{
    int16 diff = target_pwm - last_pwm;

    if(diff > PWM_STEP_LIMIT) return last_pwm + PWM_STEP_LIMIT;
    if(diff < -PWM_STEP_LIMIT) return last_pwm - PWM_STEP_LIMIT;
    return target_pwm;
}

static uint8 speed_too_fast(int16 target, int16 current)
{
    if(target > 0) return current > (target + SPEED_OVERSHOOT_MARGIN);
    if(target < 0) return current < (target - SPEED_OVERSHOOT_MARGIN);
    return abs(current) > SPEED_OVERSHOOT_MARGIN;
}

static int16 update_one_speed_loop(IncrementalPI_t *pid,
                                   int16 target,
                                   int16 current,
                                   int16 pwm,
                                   uint16 *stall_count,
                                   uint16 *cooldown)
{
    int16 target_pwm;
    float err;
    float delta_pwm;

    if(*cooldown > 0)
    {
        (*cooldown)--;
        *stall_count = 0;
        IncrementalPI_Reset(pid);
        return 0;
    }

    if(target == 0)
    {
        *stall_count = 0;
        IncrementalPI_Reset(pid);
        return 0;
    }

    if(speed_too_fast(target, current))
    {
        *stall_count = 0;
        return pwm_decay_to_zero(pwm);
    }

    err = (float)target - (float)current;
    delta_pwm = IncrementalPI_Calculate(pid, err);
    target_pwm = clamp_i16((int32)pwm + (int32)delta_pwm, -MAX_PWM, MAX_PWM);
    pwm = pwm_ramp_limit(target_pwm, pwm);

    if((abs(current) <= 1) && (abs(pwm) > STALL_PWM_LIMIT))
    {
        (*stall_count)++;
        if(*stall_count > STALL_TICKS)
        {
            *cooldown = STALL_COOLDOWN_TICKS;
            *stall_count = 0;
            IncrementalPI_Reset(pid);
            return 0;
        }
    }
    else
    {
        *stall_count = 0;
    }

    return pwm;
}

static void update_targets_from_camera(void)
{
    int16 turn_limit;
    float error;
    float turn;

    // 进弯用3帧确认，出弯靠IMU偏航角变化超过60度判断
    static uint8  turn_entry_cnt = 0;  // 连续检测帧数
    static uint8  turn_active = 0;     // 是否处于直角弯状态
    static float turn_entry_yaw = 0;   // 进弯时的偏航角

    if (junction_type_from_camera >= 3)
    {
        if (turn_entry_cnt < 255) turn_entry_cnt++;
    }
    else
    {
        turn_entry_cnt = 0;
    }

    if (!turn_active)
    {
        if (turn_entry_cnt >= 3)
        {
            turn_active = 1;
            turn_entry_yaw = imu_yaw;
        }
    }
    else
    {
        // 计算偏航角变化（处理0-360回绕）
        float dyaw = imu_yaw - turn_entry_yaw;
        if (dyaw < 0) dyaw = -dyaw;
        if (dyaw > 180.0f) dyaw = 360.0f - dyaw;
        if (dyaw >= 75.0f)
        {
            turn_active = 0;
            Task1_CountTurn();  /* IMU 确认转弯完成，计一次 */
        }
    }

    base_speed_target = approach_i16(base_speed_target, base_speed_cmd, CONTROL_BASE_RAMP_STEP);

#if (CONTROL_STRAIGHT_ONLY == 1)
    turn_output = 0;
    PositionPD_Reset(&turn_pid);
#else
    error = (float)(CONTROL_TURN_SIGN * Camera_GetTrackOffset());
    turn = PositionPD_Calculate(&turn_pid, error);
    turn -= gyro[2] * CONTROL_GYRO_DAMPING_GAIN;
    turn_limit = CONTROL_TURN_LIMIT_DEFAULT;
    if(turn_limit > base_speed_target) turn_limit = base_speed_target;
    turn_output = clamp_i16((int32)turn, -turn_limit, turn_limit);

    // Boost turn on confirmed sharp right-angle turn
    if (turn_active)
    {
        turn_output = (int16)(turn_output * CONTROL_SHARP_TURN_GAIN);
        turn_output = clamp_i16((int32)turn_output, -turn_limit, turn_limit);
    }
#endif

    target_l = clamp_i16((int32)base_speed_target + turn_output,
                         -CONTROL_BASE_SPEED_MAX,
                         CONTROL_BASE_SPEED_MAX);
    target_r = clamp_i16((int32)base_speed_target - turn_output,
                         -CONTROL_BASE_SPEED_MAX,
                         CONTROL_BASE_SPEED_MAX);

    if(base_speed_target >= 0)
    {
        if(target_l < 0) target_l = 0;
        if(target_r < 0) target_r = 0;
    }
}

void Control_Task10ms(void)
{
    int16 speed_l;
    int16 speed_r;

    Motor_ReadEncoder10ms(&speed_l, &speed_r);

    if((abs(speed_l) > ENCODER_SPEED_STOP_LIMIT) || (abs(speed_r) > ENCODER_SPEED_STOP_LIMIT))
    {
        fault_stop = 1;
    }

    if(fault_stop)
    {
        control_reset_runtime();
        return;
    }

    update_targets_from_camera();

    // 起步 500ms 内速度环 PID 参数翻倍，线性衰减
    if (start_ticks < 250) start_ticks++;
    if (start_ticks <= 50)
    {
        float boost = 1.0f + (50.0f - (float)start_ticks) / 50.0f;
        IncrementalPI_SetParam(&speed_pid_l, speed_kp * boost, speed_ki * boost);
        IncrementalPI_SetParam(&speed_pid_r, speed_kp * boost, speed_ki * boost);
    }

    pwm_l = update_one_speed_loop(&speed_pid_l,
                                  target_l,
                                  speed_l,
                                  pwm_l,
                                  &stall_count_l,
                                  &cooldown_l);
    pwm_r = update_one_speed_loop(&speed_pid_r,
                                  target_r,
                                  speed_r,
                                  pwm_r,
                                  &stall_count_r,
                                  &cooldown_r);

    if (start_ticks <= 50)
    {
        IncrementalPI_SetParam(&speed_pid_l, speed_kp, speed_ki);
        IncrementalPI_SetParam(&speed_pid_r, speed_kp, speed_ki);
    }

    Motor_SetPWM(pwm_l, pwm_r);

    Task1_Update();
    if (Task1_IsFinished())
    {
        base_speed_cmd = 0;
        control_reset_runtime();
    }
}

void Control_DisplayStatusTask(void)
{
}

int16 Control_GetBaseSpeedCommand(void) { return base_speed_cmd; }
int16 Control_GetBaseSpeedTarget(void) { return base_speed_target; }
int16 Control_GetTargetL(void) { return target_l; }
int16 Control_GetTargetR(void) { return target_r; }
int16 Control_GetTurnOutput(void) { return turn_output; }
uint8 Control_GetFaultStop(void) { return fault_stop; }
