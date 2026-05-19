#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"

#include "camera.h"
#include "control.h"
#include "fan.h"
#include "imu.h"
#include "motor.h"
#include "zf_device_ips200.h"
#include "zf_device_key.h"

static uint8 fan_on = 0;  /* 负压风扇开关状态 */

int core0_main(void)
{
    clock_init();
    debug_init();
    disable_Watchdog();
    system_delay_init();

    interrupt_global_enable(0);

    ips200_init(IPS200_TYPE_SPI);
    ips200_clear();

    cam_init();
    fan_init();  // 上电默认关闭，KEY4 切换
    imu_init();
    pit_ms_init(CCU60_CH1, 1);  // 1 ms IMU trigger
    key_init(10);

    cpu_wait_event_ready();

    // KEY1=启动任务1  KEY2=启动任务2  KEY4=切换风扇
    task_mode_t selected_mode = TASK_MODE_1;
    while(TRUE)
    {
        key_scanner();

        if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
        {
            key_clear_state(KEY_1);
            selected_mode = TASK_MODE_1;
            break;
        }
        if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
        {
            key_clear_state(KEY_2);
            selected_mode = TASK_MODE_2;
            break;
        }

        if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
        {
            key_clear_state(KEY_4);
            fan_on = !fan_on;
            fan_set_duty(fan_on ? 40 : 0);
        }

        image_process_task();
        image_display_task();
        system_delay_ms(20);
    }

    motor_init();
    Control_SetTaskMode(selected_mode);

#if (MOTOR_TEST_MODE == 1)
    motor_encoder_test_task();
#elif (MOTOR_TEST_MODE == 2)
    motor_openloop_pwm_test_task();
#endif

    while(TRUE)
    {
        key_scanner();

        // KEY4 运行中切换负压风扇
        if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
        {
            key_clear_state(KEY_4);
            fan_on = !fan_on;
            fan_set_duty(fan_on ? 40 : 0);
        }

        // KEY3 运行中校准陀螺仪
        if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
        {
            key_clear_state(KEY_3);
            imu_init();
        }

        image_process_task();
        image_display_task();
        motor_display_status_task();
    }
}

#pragma section all restore
