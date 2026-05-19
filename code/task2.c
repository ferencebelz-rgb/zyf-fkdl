#include "task2.h"

static uint8  turn_count = 0;
static uint16 stop_delay = 0;

static const route_decision_t route_sequence[] =
{
    DECISION_RIGHT,
    DECISION_LEFT,
    DECISION_LEFT,
    DECISION_RIGHT,
    DECISION_STRAIGHT,
    DECISION_STRAIGHT,
};
static uint8 route_index = 0;

void Task2_Init(void)
{
    turn_count = 0;
    stop_delay = 0;
    route_index = 0;
}

void Task2_CountTurn(void)
{
    if(turn_count < 255)
    {
        turn_count++;
    }
}

void Task2_Update(void)
{
    if(turn_count >= TASK2_TURN_TARGET)
    {
        if(stop_delay < 666)
        {
            stop_delay += 10;
        }
    }
}

route_decision_t Task2_GetRouteDecision(void)
{
    route_decision_t decision;
    uint8 route_count = (uint8)(sizeof(route_sequence) / sizeof(route_sequence[0]));

    if(route_index >= route_count)
    {
        return DECISION_STRAIGHT;
    }

    decision = route_sequence[route_index];
    route_index++;
    return decision;
}

uint8 Task2_GetCount(void)
{
    return turn_count;
}

uint8 Task2_IsFinished(void)
{
    return (stop_delay >= 666) ? 1 : 0;
}
