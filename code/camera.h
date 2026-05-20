#ifndef CODE_CAMERA_H_

#define CODE_CAMERA_H_



#include "zf_common_headfile.h"

#include "zf_device_mt9v03x_double.h"

#include "zf_device_ips200.h"



// Binarisation 鈥� offset added to Otsu threshold (positive = darker)

#define THRESHOLD_DARK_OFFSET      12



// Sharp (right-angle) turn detection via edge-boundary tracking

#define SHARP_TURN_EDGE_MARGIN     15

#define SHARP_TURN_LOST_MIN        6

#define SHARP_TURN_CENTER_SHIFT   15

#define SHARP_TURN_CORNER_DERIV    8     // edge-position jump per row to locate corner



// junction_type_from_camera: 0=normal, 1=T/corner, 2=cross/L, 3=sharp turn

extern int16 line_mid[MT9V03X_1_H];

extern int16 track_offset;

extern uint8 junction_type_from_camera;



void cam_init(void);

void image_process_task(void);

void image_display_task(void);

int16 Camera_GetTrackOffset(void);

uint8 Camera_GetLineLostCount(void);

int16 Camera_GetCenterLine(uint8 row);



#endif

