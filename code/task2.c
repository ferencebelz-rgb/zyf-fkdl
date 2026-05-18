#include "task2.h"
#include "zf_device_key.h"

static uint8  turn_count = 0;
static uint16 stop_delay = 0;   /* 达标后延迟滴答计数 (10ms/tick) */

/* T字路口转弯顺序: R, L, L, R, 直行, 直行 */
static const int8 tjun_sequence[] = {1, -1, -1, 1, 0, 0};
static uint8 tjun_index = 0;

void Task2_Init(void)
{
    turn_count = 0;
    stop_delay = 0;
    tjun_index = 0;
}

void Task2_CountTurn(void)
{
    if (turn_count < 255)
    {
        turn_count++;
    }
}

void Task2_Update(void)
{
    /* 达标后开始倒计时 */
    if (turn_count >= TASK2_TURN_TARGET)
    {
        if (stop_delay < 666)
        {
            stop_delay += 10;  /* 每次调用 +10ms */
        }
    }
}

void Task2_CheckSW2(void)
{
    if (key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_2);
        Task2_CountTurn();
    }
}

/*
 * T字路口转弯方向查询
 * 每次 T 字路口检测到时调用一次（由 camera.c 上升沿触发），
 * 返回 -1=左转  0=直行  1=右转，超过序列后默认直行。
 */
int8 Task2_GetTJunTurnDir(void)
{
    if (tjun_index >= 6) return 0;
    int8 dir = tjun_sequence[tjun_index];
    tjun_index++;
    return dir;
}

uint8 Task2_GetCount(void)
{
    return turn_count;
}

uint8 Task2_IsFinished(void)
{
    return (stop_delay >= 666) ? 1 : 0;
}
