#include "control.h"
#include "camera.h"
#include "imu.h"
#include "motor.h"
#include "pid.h"
#include "zf_device_ips200.h"
#include "zf_device_key.h"
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
static ControlParamPage_e param_page = CONTROL_PARAM_BASE_SPEED;

static uint16 stall_count_l = 0;
static uint16 stall_count_r = 0;
static uint16 cooldown_l = 0;
static uint16 cooldown_r = 0;

static float clamp_float(float value, float min_value, float max_value)
{
    if(value < min_value) return min_value;
    if(value > max_value) return max_value;
    return value;
}

static int16 clamp_i16(int32 value, int16 min_value, int16 max_value)
{
    if(value < min_value) return min_value;
    if(value > max_value) return max_value;
    return (int16)value;
}

static void set_speed_param(float kp, float ki)
{
    speed_kp = clamp_float(kp, SPEED_KP_MIN, SPEED_KP_MAX);
    speed_ki = clamp_float(ki, SPEED_KI_MIN, SPEED_KI_MAX);
    IncrementalPI_SetParam(&speed_pid_l, speed_kp, speed_ki);
    IncrementalPI_SetParam(&speed_pid_r, speed_kp, speed_ki);
}

static void set_turn_param(float kp, float kd)
{
    turn_kp = clamp_float(kp, CONTROL_TURN_KP_MIN, CONTROL_TURN_KP_MAX);
    turn_kd = clamp_float(kd, CONTROL_TURN_KD_MIN, CONTROL_TURN_KD_MAX);
    PositionPD_SetParam(&turn_pid, turn_kp, turn_kd);
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

static void key_adjust_current_param(int8 direction)
{
    switch(param_page)
    {
        case CONTROL_PARAM_BASE_SPEED:
            base_speed_cmd = clamp_i16((int32)base_speed_cmd + direction * CONTROL_BASE_SPEED_STEP,
                                       CONTROL_BASE_SPEED_MIN,
                                       CONTROL_BASE_SPEED_MAX);
            break;

        case CONTROL_PARAM_SPEED_KP:
            set_speed_param(speed_kp + direction * SPEED_KP_STEP, speed_ki);
            break;

        case CONTROL_PARAM_SPEED_KI:
            set_speed_param(speed_kp, speed_ki + direction * SPEED_KI_STEP);
            break;

        case CONTROL_PARAM_TURN_KP:
            set_turn_param(turn_kp + direction * CONTROL_TURN_KP_STEP, turn_kd);
            break;

        case CONTROL_PARAM_TURN_KD:
            set_turn_param(turn_kp, turn_kd + direction * CONTROL_TURN_KD_STEP);
            break;

        default:
            break;
    }
}

static void key_task_10ms(void)
{
    key_scanner();

    if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        key_adjust_current_param(1);
        key_clear_state(KEY_1);
    }

    if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        key_adjust_current_param(-1);
        key_clear_state(KEY_2);
    }

    if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
        param_page = (ControlParamPage_e)((param_page + 1) % CONTROL_PARAM_COUNT);
        key_clear_state(KEY_3);
    }

    if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
    {
        base_speed_cmd = CONTROL_BASE_SPEED_DEFAULT;
        set_speed_param(SPEED_KP_DEFAULT, SPEED_KI_DEFAULT);
        set_turn_param(CONTROL_TURN_KP_DEFAULT, CONTROL_TURN_KD_DEFAULT);
        Control_ClearFault();
        key_clear_state(KEY_4);
    }

    if(key_get_state(KEY_4) == KEY_LONG_PRESS)
    {
        Control_ClearFault();
        key_clear_state(KEY_4);
    }
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

    base_speed_target = approach_i16(base_speed_target, base_speed_cmd, CONTROL_BASE_RAMP_STEP);

#if (CONTROL_STRAIGHT_ONLY == 1)
    turn_output = 0;
    PositionPD_Reset(&turn_pid);
#else
    error = (float)(CONTROL_TURN_SIGN * Camera_GetTrackOffset());
    turn = PositionPD_Calculate(&turn_pid, error);
    turn_limit = CONTROL_TURN_LIMIT_DEFAULT;
    if(turn_limit > base_speed_target) turn_limit = base_speed_target;
    turn_output = clamp_i16((int32)turn, -turn_limit, turn_limit);

    // Boost turn differential 1.5x on L-junction or sharp right-angle turn
    if (junction_type_from_camera >= 2)
    {
        turn_output = (int16)(turn_output * 3 / 2);
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

    key_task_10ms();
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

    Motor_SetPWM(pwm_l, pwm_r);
}

static const char *param_page_name(void)
{
    switch(param_page)
    {
        case CONTROL_PARAM_BASE_SPEED: return "BASE";
        case CONTROL_PARAM_SPEED_KP:   return "SKP";
        case CONTROL_PARAM_SPEED_KI:   return "SKI";
        case CONTROL_PARAM_TURN_KP:    return "TKP";
        case CONTROL_PARAM_TURN_KD:    return "TKD";
        default:                       return "UNK";
    }
}

void Control_DisplayStatusTask(void)
{
    static uint32 last_ms = 0;
    uint32 now_ms = system_getval_ms();

    if((uint32)(now_ms - last_ms) < 100) return;
    last_ms = now_ms;

    ips200_show_string(0, 122, "                              ");
    ips200_show_string(0, 140, "                              ");
    ips200_show_string(0, 158, "                              ");
    ips200_show_string(0, 176, "                              ");
    ips200_show_string(0, 194, "                              ");

    ips200_show_string(0, 122, "Pg:");
    ips200_show_string(24, 122, param_page_name());
    ips200_show_string(72, 122, "B:");
    ips200_show_int(90, 122, base_speed_cmd, 4);
    ips200_show_string(142, 122, "Off:");
    ips200_show_int(178, 122, Camera_GetTrackOffset(), 4);

    ips200_show_string(0, 140, "T:");
    ips200_show_int(18, 140, target_l, 5);
    ips200_show_int(72, 140, target_r, 5);
    ips200_show_string(132, 140, "S:");
    ips200_show_int(150, 140, Motor_GetSpeedL(), 5);
    ips200_show_int(198, 140, Motor_GetSpeedR(), 4);

    ips200_show_string(0, 158, "P:");
    ips200_show_int(18, 158, pwm_l, 5);
    ips200_show_int(72, 158, pwm_r, 5);
    ips200_show_string(132, 158, "F:");
    ips200_show_int(150, 158, fault_stop, 1);
    ips200_show_string(174, 158, "L:");
    ips200_show_int(192, 158, Camera_GetLineLostCount(), 3);

    ips200_show_string(0, 176, "KP:");
    ips200_show_float(26, 176, speed_kp, 2, 1);
    ips200_show_string(72, 176, "KI:");
    ips200_show_float(98, 176, speed_ki, 2, 1);
    ips200_show_string(144, 176, "TK:");
    ips200_show_float(170, 176, turn_kp, 2, 1);

    ips200_show_string(0, 194, "TD:");
    ips200_show_float(26, 194, turn_kd, 2, 1);
    ips200_show_string(72, 194, "Y:");
    ips200_show_float(90, 194, imu_yaw, 4, 1);
    ips200_show_string(154, 194, "G:");
    ips200_show_float(172, 194, gyro[2], 2, 1);
}

int16 Control_GetBaseSpeedCommand(void) { return base_speed_cmd; }
int16 Control_GetBaseSpeedTarget(void) { return base_speed_target; }
int16 Control_GetTargetL(void) { return target_l; }
int16 Control_GetTargetR(void) { return target_r; }
int16 Control_GetTurnOutput(void) { return turn_output; }
uint8 Control_GetFaultStop(void) { return fault_stop; }
ControlParamPage_e Control_GetParamPage(void) { return param_page; }
float Control_GetSpeedKp(void) { return speed_kp; }
float Control_GetSpeedKi(void) { return speed_ki; }
float Control_GetTurnKp(void) { return turn_kp; }
float Control_GetTurnKd(void) { return turn_kd; }
