#include "fan.h"

void fan_init(void)
{
    pwm_init(FAN_PWM, FAN_FREQ, FAN_DUTY_DEFAULT * (PWM_DUTY_MAX / 100));
}

void fan_set_duty(int8 duty)
{
    pwm_set_duty(FAN_PWM, duty * (PWM_DUTY_MAX / 100));
}
