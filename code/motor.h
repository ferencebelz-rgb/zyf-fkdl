#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_headfile.h"

#define DIR_L               P11_9
#define PWM_L               ATOM3_CH5_P11_10
#define DIR_R               P11_11
#define PWM_R               ATOM2_CH7_P11_12

#define ENCODER_L           TIM4_ENCODER
#define ENCODER_L_CH1       TIM4_ENCODER_CH1_P02_8
#define ENCODER_L_CH2       TIM4_ENCODER_CH2_P00_9

#define ENCODER_R           TIM6_ENCODER
#define ENCODER_R_CH1       TIM6_ENCODER_CH1_P20_3
#define ENCODER_R_CH2       TIM6_ENCODER_CH2_P20_0

// 0=closed loop; 1=encoder hand test; 2=open-loop PWM key test.
#define MOTOR_TEST_MODE     0

// Set to 0 to keep PWM output disabled while preserving logic and display.
#define MOTOR_OUTPUT_ENABLE 1

#define MOTOR_L_FORWARD_LEVEL 1
#define MOTOR_R_FORWARD_LEVEL 0

#define ENCODER_L_SIGN       (-1)
#define ENCODER_R_SIGN       (1)

#define SPEED_KP_DEFAULT    2.0f
#define SPEED_KI_DEFAULT    0.6f
#define SPEED_KP_MIN        1.0f
#define SPEED_KP_MAX        5.0f
#define SPEED_KP_STEP       0.5f
#define SPEED_KI_MIN        0.0f
#define SPEED_KI_MAX        5.0f
#define SPEED_KI_STEP       0.2f

#define MAX_PWM             6666
#define PWM_STEP_LIMIT      200
#define STALL_PWM_LIMIT     5200
#define STALL_TICKS         30
#define STALL_COOLDOWN_TICKS 100
#define SPEED_OVERSHOOT_MARGIN 120
#define OVERSPEED_PWM_DECAY 500
#define ENCODER_SPEED_STOP_LIMIT 2000
#define PWM_FREQ            17000

#define CONTROL_TIMER       CCU60_CH0
#define CONTROL_PERIOD_MS   10

extern int16 current_speed_l;
extern int16 current_speed_r;

void Motor_Init(void);
void Motor_SetPWM(int16 pwm_l, int16 pwm_r);
void Motor_Stop(void);
void Motor_ReadEncoder10ms(int16 *speed_l, int16 *speed_r);
int16 Motor_GetPwmL(void);
int16 Motor_GetPwmR(void);
int16 Motor_GetSpeedL(void);
int16 Motor_GetSpeedR(void);

void motor_init(void);
void motor_encoder_test_task(void);
void motor_openloop_pwm_test_task(void);
void motor_display_status_task(void);

#endif
