#ifndef CODE_CAMERA_H_
#define CODE_CAMERA_H_

#include "zf_common_headfile.h"
#include "zf_device_mt9v03x_double.h"
#include "zf_device_ips200.h"
#include "task2.h"

#define SHARP_TURN_EDGE_MARGIN     3
#define SHARP_TURN_CORNER_DERIV    8

typedef enum
{
    JUNCTION_NONE = 0,
    JUNCTION_CROSS,
    JUNCTION_SIDE_LEFT_T,
    JUNCTION_SIDE_RIGHT_T,
    JUNCTION_STANDARD_T,
    JUNCTION_LEFT_CORNER,
    JUNCTION_RIGHT_CORNER,
} junction_kind_t;

extern int16 line_mid[MT9V03X_1_H];
extern int16 track_offset;
extern uint8 junction_type_from_camera;

void cam_init(void);
void image_process_task(void);
void image_display_task(void);

int16 Camera_GetTrackOffset(void);
uint8 Camera_GetLineLostCount(void);
int16 Camera_GetCenterLine(uint8 row);
uint8 Camera_GetJunctionError(void);
junction_kind_t Camera_GetJunctionKind(void);
route_decision_t Camera_GetRouteDecision(void);

#endif
