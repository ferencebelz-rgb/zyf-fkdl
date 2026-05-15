#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"

#include "camera.h"
#include "fan.h"
#include "imu.h"
#include "motor.h"
#include "zf_device_ips200.h"
#include "zf_device_key.h"

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
    fan_init();
    imu_init();
    pit_ms_init(CCU60_CH1, 1);  // 1 ms IMU trigger
    key_init(10);

    cpu_wait_event_ready();

    // Wait for KEY1 before starting motors (camera runs in background)
    while(TRUE)
    {
        key_scanner();
        if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
        {
            key_clear_state(KEY_1);
            break;
        }
        image_process_task();
        image_display_task();
        system_delay_ms(20);
    }

    motor_init();

#if (MOTOR_TEST_MODE == 1)
    motor_encoder_test_task();
#elif (MOTOR_TEST_MODE == 2)
    motor_openloop_pwm_test_task();
#endif

    while(TRUE)
    {
        image_process_task();
        image_display_task();
        motor_display_status_task();
    }
}

#pragma section all restore
