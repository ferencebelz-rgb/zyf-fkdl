#include "motor.h"
#include "control.h"
#include "zf_device_ips200.h"
#include "zf_device_key.h"
#include <stdlib.h>

static int16 motor_pwm_l = 0;
static int16 motor_pwm_r = 0;

int16 current_speed_l = 0;
int16 current_speed_r = 0;

static int16 clamp_pwm(int32 pwm)
{
    if(pwm > MAX_PWM) return MAX_PWM;
    if(pwm < -MAX_PWM) return -MAX_PWM;
    return (int16)pwm;
}

void Motor_SetPWM(int16 pwm_l, int16 pwm_r)
{
    uint8 dir_l;
    uint8 dir_r;
    int16 duty_l;
    int16 duty_r;

#if (MOTOR_OUTPUT_ENABLE == 0)
    gpio_set_level(DIR_L, MOTOR_L_FORWARD_LEVEL);
    gpio_set_level(DIR_R, MOTOR_R_FORWARD_LEVEL);
    pwm_set_duty(PWM_L, 0);
    pwm_set_duty(PWM_R, 0);
    motor_pwm_l = 0;
    motor_pwm_r = 0;
    return;
#endif

    pwm_l = clamp_pwm(pwm_l);
    pwm_r = clamp_pwm(pwm_r);

    dir_l = (pwm_l >= 0) ? MOTOR_L_FORWARD_LEVEL : (1 - MOTOR_L_FORWARD_LEVEL);
    dir_r = (pwm_r >= 0) ? MOTOR_R_FORWARD_LEVEL : (1 - MOTOR_R_FORWARD_LEVEL);
    duty_l = (pwm_l >= 0) ? pwm_l : -pwm_l;
    duty_r = (pwm_r >= 0) ? pwm_r : -pwm_r;

    gpio_set_level(DIR_L, dir_l);
    pwm_set_duty(PWM_L, duty_l);
    gpio_set_level(DIR_R, dir_r);
    pwm_set_duty(PWM_R, duty_r);

    motor_pwm_l = pwm_l;
    motor_pwm_r = pwm_r;
}

void Motor_Stop(void)
{
    Motor_SetPWM(0, 0);
}

void Motor_ReadEncoder10ms(int16 *speed_l, int16 *speed_r)
{
    current_speed_l = ENCODER_L_SIGN * encoder_get_count(ENCODER_L);
    encoder_clear_count(ENCODER_L);

    current_speed_r = ENCODER_R_SIGN * encoder_get_count(ENCODER_R);
    encoder_clear_count(ENCODER_R);

    if(speed_l != NULL) *speed_l = current_speed_l;
    if(speed_r != NULL) *speed_r = current_speed_r;
}

int16 Motor_GetPwmL(void)
{
    return motor_pwm_l;
}

int16 Motor_GetPwmR(void)
{
    return motor_pwm_r;
}

int16 Motor_GetSpeedL(void)
{
    return current_speed_l;
}

int16 Motor_GetSpeedR(void)
{
    return current_speed_r;
}

void Motor_Init(void)
{
    gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM_L, PWM_FREQ, 0);
    pwm_init(PWM_R, PWM_FREQ, 0);

    encoder_dir_init(ENCODER_L, ENCODER_L_CH1, ENCODER_L_CH2);
    encoder_dir_init(ENCODER_R, ENCODER_R_CH1, ENCODER_R_CH2);
    encoder_clear_count(ENCODER_L);
    encoder_clear_count(ENCODER_R);

    Motor_Stop();
}

void motor_init(void)
{
    Motor_Init();
    key_init(CONTROL_PERIOD_MS);
    Control_Init();

#if (MOTOR_TEST_MODE == 0)
    pit_ms_init(CCU60_CH1, 1);
    pit_ms_init(CONTROL_TIMER, CONTROL_PERIOD_MS);
#endif
}

void motor_display_status_task(void)
{
    Control_DisplayStatusTask();
}

void motor_encoder_test_task(void)
{
    int16 raw_l = 0;
    int16 raw_r = 0;
    int16 signed_l = 0;
    int16 signed_r = 0;
    int32 total_l = 0;
    int32 total_r = 0;

    Motor_Stop();
    encoder_clear_count(ENCODER_L);
    encoder_clear_count(ENCODER_R);

    ips200_clear();
    ips200_show_string(0, 0, "ENCODER TEST");
    ips200_show_string(0, 20, "Push wheel forward");
    ips200_show_string(0, 50, "L raw:");
    ips200_show_string(0, 70, "L sign:");
    ips200_show_string(0, 90, "L total:");
    ips200_show_string(0, 120, "R raw:");
    ips200_show_string(0, 140, "R sign:");
    ips200_show_string(0, 160, "R total:");

    while(TRUE)
    {
        raw_l = encoder_get_count(ENCODER_L);
        encoder_clear_count(ENCODER_L);
        raw_r = encoder_get_count(ENCODER_R);
        encoder_clear_count(ENCODER_R);

        signed_l = ENCODER_L_SIGN * raw_l;
        signed_r = ENCODER_R_SIGN * raw_r;
        total_l += signed_l;
        total_r += signed_r;

        ips200_show_int(80, 50, raw_l, 6);
        ips200_show_int(80, 70, signed_l, 6);
        ips200_show_int(80, 90, total_l, 8);
        ips200_show_int(80, 120, raw_r, 6);
        ips200_show_int(80, 140, signed_r, 6);
        ips200_show_int(80, 160, total_r, 8);

        Motor_Stop();
        system_delay_ms(100);
    }
}

void motor_openloop_pwm_test_task(void)
{
    int16 test_pwm_l = 1000;
    int16 test_pwm_r = 1000;
    int16 signed_l = 0;
    int16 signed_r = 0;

    Motor_Stop();
    encoder_clear_count(ENCODER_L);
    encoder_clear_count(ENCODER_R);

    ips200_clear();
    ips200_show_string(0, 0, "PWM SPEED TEST");
    ips200_show_string(0, 20, "S2/S3:L  S4/S5:R");
    ips200_show_string(0, 45, "Lpwm:");
    ips200_show_string(0, 70, "Rpwm:");
    ips200_show_string(0, 95, "Lspd:");
    ips200_show_string(0, 120, "Rspd:");

    while(TRUE)
    {
        key_scanner();

        if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
        {
            test_pwm_l += 100;
            key_clear_state(KEY_1);
        }
        if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
        {
            test_pwm_l -= 100;
            key_clear_state(KEY_2);
        }
        if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
        {
            test_pwm_r += 100;
            key_clear_state(KEY_3);
        }
        if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
        {
            test_pwm_r -= 100;
            key_clear_state(KEY_4);
        }

        test_pwm_l = clamp_pwm(test_pwm_l);
        test_pwm_r = clamp_pwm(test_pwm_r);
        if(test_pwm_l < 1000) test_pwm_l = 1000;
        if(test_pwm_l > 3000) test_pwm_l = 3000;
        if(test_pwm_r < 1000) test_pwm_r = 1000;
        if(test_pwm_r > 3000) test_pwm_r = 3000;

        signed_l = ENCODER_L_SIGN * encoder_get_count(ENCODER_L);
        encoder_clear_count(ENCODER_L);
        signed_r = ENCODER_R_SIGN * encoder_get_count(ENCODER_R);
        encoder_clear_count(ENCODER_R);

        Motor_SetPWM(test_pwm_l, test_pwm_r);

        ips200_show_int(56, 45, Motor_GetPwmL(), 6);
        ips200_show_int(56, 70, Motor_GetPwmR(), 6);
        ips200_show_int(56, 95, signed_l, 6);
        ips200_show_int(56, 120, signed_r, 6);

        system_delay_ms(10);
    }
}

IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH0);
    Control_Task10ms();
}
