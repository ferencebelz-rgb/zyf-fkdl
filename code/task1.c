#include "task1.h"

static uint8  turn_count = 0;
static uint16 stop_delay = 0;
static uint8  t_seq_index = 0;

/* T字路口固定序列: 右(0), 左(1), 左(1), 右(0), 直行(2), 直行(2) */
static const uint8 t_seq[TASK1_T_SEQ_LEN] = {0, 1, 1, 0, 2, 2};

void Task1_Init(void)
{
    turn_count = 0;
    stop_delay = 0;
    t_seq_index = 0;
}

void Task1_CountTurn(void)
{
    if (turn_count < 255)
    {
        turn_count++;
    }
}

void Task1_TComplete(void)
{
    /* T字路口转弯完成：计一次转弯 + 推进序列索引 */
    if (turn_count < 255)
    {
        turn_count++;
    }
    if (t_seq_index < TASK1_T_SEQ_LEN - 1)
    {
        t_seq_index++;
    }
}

void Task1_TSkip(void)
{
    /* T字路口直行通过：只推进序列，不计转弯 */
    if (t_seq_index < TASK1_T_SEQ_LEN - 1)
    {
        t_seq_index++;
    }
}

uint8 Task1_GetNextTDir(void)
{
    if (t_seq_index >= TASK1_T_SEQ_LEN) return 2;
    return t_seq[t_seq_index];
}

void Task1_Update(void)
{
    if (turn_count >= TASK1_TURN_TARGET)
    {
        if (stop_delay < 500)
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
    return (stop_delay >= 500) ? 1 : 0;
}
