#include "task1.h"

static uint8  turn_count = 0;
static uint16 stop_delay = 0;   /* 达标后延迟滴答计数 (10ms/tick) */

void Task1_Init(void)
{
    turn_count = 0;
    stop_delay = 0;
}

void Task1_CountTurn(void)
{
    if (turn_count < 255)
    {
        turn_count++;
    }
}

void Task1_Update(void)
{
    /* 达标后开始倒计时 */
    if (turn_count >= TASK1_TURN_TARGET)
    {
        if (stop_delay < 1000)
        {
            stop_delay += 10;  /* 每次调用 +10ms */
        }
    }
}

uint8 Task1_GetCount(void)
{
    return turn_count;
}

uint8 Task1_IsFinished(void)
{
    return (stop_delay >= 1000) ? 1 : 0;
}
