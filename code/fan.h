#ifndef _FAN_H_
#define _FAN_H_

#include "zf_common_headfile.h"

#define FAN_PWM             ATOM1_CH0_P21_2
#define FAN_DUTY_DEFAULT    0
#define FAN_FREQ            17000

void fan_init(void);
void fan_set_duty(int8 duty);

#endif
