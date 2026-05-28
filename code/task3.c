#include "task3.h"

#define BUZZER_PIN (P33_10)

static uint8  turn_count = 0;
static uint16 stop_delay = 0;
static uint8  t_seq_index = 0;
static uint8  beep_timer = 0;

/* 0=right, 1=left, 2=straight. Stop target counts turns only. */
static const uint8 t_seq[TASK3_T_SEQ_LEN] = {0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 2, 0, 0, 0, 0};

static void advance_sequence(void)
{
    if (t_seq_index < TASK3_T_SEQ_LEN - 1)
    {
        t_seq_index++;
        beep_timer = TASK3_BEEP_TICKS;
    }
}

void Task3_Init(void)
{
    turn_count = 0;
    stop_delay = 0;
    t_seq_index = 0;
    beep_timer = 0;
    gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

void Task3_CountTurn(void)
{
    if (turn_count < 255)
    {
        turn_count++;
    }
    advance_sequence();
}

void Task3_TComplete(void)
{
    Task3_CountTurn();
}

void Task3_TSkip(void)
{
    advance_sequence();
}

uint8 Task3_GetNextTDir(void)
{
    if (t_seq_index >= TASK3_T_SEQ_LEN) return 0;
    return t_seq[t_seq_index];
}

void Task3_Update(void)
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

    if (turn_count >= TASK3_TURN_TARGET)
    {
        if (stop_delay < 500)
        {
            stop_delay += 10;
        }
    }
}

uint8 Task3_GetCount(void)
{
    return turn_count;
}

uint8 Task3_GetSeqIndex(void)
{
    return t_seq_index;
}

uint8 Task3_IsFinished(void)
{
    return (stop_delay >= 500) ? 1 : 0;
}
