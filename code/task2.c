#include "task2.h"

#define BUZZER_PIN (P33_10)

static uint8  turn_count = 0;
static uint16 stop_delay = 0;
static uint8  t_seq_index = 0;
static uint8  beep_timer = 0;

/* T字路口固定序列: 右(0), 左(1), 左(1), 右(0), 直行(2), 直行(2) */
static const uint8 t_seq[TASK2_T_SEQ_LEN] = {0, 0, 1, 1, 0, 2, 2};

void Task2_Init(void)
{
    turn_count = 0;
    stop_delay = 0;
    t_seq_index = 0;
    beep_timer = 0;
    gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

void Task2_CountTurn(void)
{
    if (turn_count < 255)
    {
        turn_count++;
    }
    if (t_seq_index < TASK2_T_SEQ_LEN - 1)
    {
        t_seq_index++;
        beep_timer = TASK2_BEEP_TICKS;
    }
}

void Task2_TComplete(void)
{
    if (turn_count < 255)
    {
        turn_count++;
    }
    if (t_seq_index < TASK2_T_SEQ_LEN - 1)
    {
        t_seq_index++;
        beep_timer = TASK2_BEEP_TICKS;
    }
}

void Task2_TSkip(void)
{
    if (t_seq_index < TASK2_T_SEQ_LEN - 1)
    {
        t_seq_index++;
        beep_timer = TASK2_BEEP_TICKS;
    }
}

uint8 Task2_GetNextTDir(void)
{
    if (t_seq_index >= TASK2_T_SEQ_LEN) return 2;
    return t_seq[t_seq_index];
}

void Task2_Update(void)
{
    if (beep_timer > 0)
    {
        gpio_toggle_level(BUZZER_PIN);
        beep_timer--;
        if (beep_timer == 0)
        {
            gpio_set_level(BUZZER_PIN, GPIO_LOW);
        }
    }

    if (turn_count >= TASK2_TURN_TARGET)
    {
        if (stop_delay < 500)
        {
            stop_delay += 10;
        }
    }
}

uint8 Task2_GetCount(void)
{
    return turn_count;
}

uint8 Task2_GetSeqIndex(void)
{
    return t_seq_index;
}

uint8 Task2_IsFinished(void)
{
    return (stop_delay >= 500) ? 1 : 0;
}
