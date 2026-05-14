#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"

#include "camera.h"
#include "imu.h"
#include "motor.h"
#include "zf_device_ips200.h"

#define USE_IMU_MODULE 0

int core0_main(void)
{
    // 系统基础初始化
    clock_init();
    debug_init();
    disable_Watchdog();
    system_delay_init();

    // 必须开启全局中断，DMA抓图和 10ms 控制中断都依赖于此
    interrupt_global_enable(0);

    // 屏幕初始化
    ips200_init(IPS200_TYPE_SPI);
    ips200_clear();

    // ============================================
    // 核心模块初始化
    cam_init();
#if USE_IMU_MODULE
    imu_init();
#endif
    motor_init();  // 执行完毕后，底层 10ms 中断闭环开始独立运行！
    // ============================================

    cpu_wait_event_ready();

#if (MOTOR_TEST_MODE == 1)
    motor_encoder_test_task();
#elif (MOTOR_TEST_MODE == 2)
    motor_openloop_pwm_test_task();
#endif

    // 主循环：全速狂奔，纯算图像！再也不用操心电机的延时阻塞。
    while(TRUE)
    {
        image_process_task();   // 识别赛道、算偏差
        image_display_task();   // 显示屏幕
        motor_display_status_task();
    }
}

#pragma section all restore
