#include "camera.h"
#include "imu.h"
#include "zf_device_ips200.h"
#include "zf_device_mt9v03x_double.h"
#include "zf_driver_dma.h"
#include <string.h>

#define ENABLE_DISPLAY 1             
#define LOST_LINE_REPLACE_VAL (MT9V03X_1_W / 2)    
#define TRACK_LINE_IS_BLACK 0        
#define ENABLE_TURN_DEBUG 1          

#define THRESHOLD_DARK_MIN         30
#define THRESHOLD_DARK_MAX         30
#define THRESHOLD_CLAMP_LO         30
#define THRESHOLD_CLAMP_HI        220

// line_mid[] and track_offset are consumed by motor.c in the 10ms control ISR.
// If compiler optimization is enabled later, consider making shared variables volatile.
int16 line_mid[MT9V03X_1_H];         // centreline buffer
int16 track_offset = 0;              // track offset (px)
uint8 junction_type_from_camera = 0; // 0=normal, 1=T/corner, 2=cross/L, 3=sharp turn (dir from sign of track_offset)
static uint8 line_lost_count = 0;

// raw_snapshot stores one stable grayscale frame copied from the camera DMA buffer.
// binary_buf_0/1 and line_mid_buf_0/1 are swapped so display does not read data
// while the next frame is being processed.
static uint8 raw_snapshot[MT9V03X_1_H][MT9V03X_1_W];
static uint8 binary_buf_0[MT9V03X_1_H][MT9V03X_1_W];
static uint8 binary_buf_1[MT9V03X_1_H][MT9V03X_1_W];

static int16 line_mid_buf_0[MT9V03X_1_H];
static int16 line_mid_buf_1[MT9V03X_1_H];

static uint8 (*process_image)[MT9V03X_1_W] = binary_buf_0;
static int16 *process_line_mid = line_mid_buf_0;

static uint8 (*display_image)[MT9V03X_1_W] = binary_buf_1;
static int16 *display_line_mid = line_mid_buf_1;

static uint8 image_ready = 0;

// Sharp-turn debug overlay data (write-once from process, read from display)
static int  turn_dbg_active  = 0;
static int  turn_dbg_start_x = 0;
static int  turn_dbg_start_y = 0;
static int  turn_dbg_end_x   = 0;
static int  turn_dbg_end_y   = 0;
static int  turn_dbg_is_left = 0;

void cam_init(void)
{
    mt9v03x_set_confing_buffer_1[MT9V03X_DOUBLE_EXP_TIME][1] = 400;
    mt9v03x_double_init(mt9v03x_1);
}

static void image_swap_buffer(void)
{
    // After processing one frame, swap process/display buffers in O(1) time.
    uint8 (*temp_image)[MT9V03X_1_W] = process_image;
    process_image = display_image;
    display_image = temp_image;

    int16 *temp_line_mid = process_line_mid;
    process_line_mid = display_line_mid;
    display_line_mid = temp_line_mid;
}

static void camera_copy_stable_frame(void)
{

    memcpy(raw_snapshot[0], mt9v03x_image_1[0], MT9V03X_1_W * MT9V03X_1_H);
}

// upward. When one edge disappears (hits image boundary) while the other
// remains visible, a sharp right-angle turn is detected.
// Mid-line is then drawn as a straight line from the bottom-centre to the

// approach in the reference implementation.
static void detect_boundary_sharp_turn(void)
{
    int left_edge[MT9V03X_1_H];
    int right_edge[MT9V03X_1_H];
    int left_lost_row = -1;
    int right_lost_row = -1;
    int left_visible = 0;
    int right_visible = 0;

    // 1. Extract leftmost and rightmost white pixel per row
    for (int i = 0; i < MT9V03X_1_H; i++)
    {
        int l = -1;
        int r = -1;

        for (int j = 0; j < MT9V03X_1_W; j++)
        {
            if (process_image[i][j])
            {
                if (l < 0) l = j;
                r = j;
            }
        }

        left_edge[i]  = l;
        right_edge[i] = r;

        if (l >= 0 && l <= SHARP_TURN_EDGE_MARGIN)
        {

            if (left_lost_row < 0) left_lost_row = i;
        }
        else if (l > SHARP_TURN_EDGE_MARGIN)
        {
            left_visible++;
        }

        if (r >= 0 && r >= MT9V03X_1_W - 1 - SHARP_TURN_EDGE_MARGIN)
        {
            if (right_lost_row < 0) right_lost_row = i;
        }
        else if (r >= 0 && r < MT9V03X_1_W - 1 - SHARP_TURN_EDGE_MARGIN)
        {
            right_visible++;
        }
    }

    // 2. Decide: is this a sharp turn?
    // Only trigger when one edge is lost AND the other edge is still clearly
    // visible (prevents false positives on straights / crosses).
    int is_left_turn  = (left_lost_row >= 0)  && (left_lost_row > MT9V03X_1_H / 2)
                     && (right_visible > MT9V03X_1_H / 4);
    int is_right_turn = (right_lost_row >= 0) && (right_lost_row > MT9V03X_1_H / 2)
                     && (left_visible > MT9V03X_1_H / 4);

    if (!is_left_turn && !is_right_turn)
    {
        turn_dbg_active = 0;
        return;
    }

    junction_type_from_camera = 3;  // sharp turn

    // 3. Locate the actual corner row via edge-position derivative.
    //    The corner is where the disappearing edge jumps fastest toward the
    //    image boundary (highest per-row position change).  This is more
    //    precise than the edge-loss row and anchors the diagonal directly
    //    on the corner rather than somewhere near the top.
    //    Scan from the bottom upward; row indices increase toward the car.
    int corner_row = MT9V03X_1_H - 1;
    int max_jump   = 0;
    int *edge = is_left_turn ? left_edge : right_edge;

    for (int i = MT9V03X_1_H - 2; i >= 1; i--)
    {
        if (edge[i] < 0 || edge[i + 1] < 0) continue;
        int jump = edge[i] - edge[i + 1];
        if (jump < 0) jump = -jump;
        if (jump > max_jump)
        {
            max_jump = jump;
            corner_row = i;
        }
    }

    // Fall back to the edge-loss row if no clear corner was found
    if (max_jump < SHARP_TURN_CORNER_DERIV)
    {
        corner_row = is_left_turn ? left_lost_row : right_lost_row;
        if (corner_row < 0) corner_row = MT9V03X_1_H - 1;
    }

    // 4. Generate curve mid-line (reference: Left_curve_line / Right_curve_line).
    //    Start from the bottom-centre, end at the corner point on the lost edge.
    int start_x  = (int)process_line_mid[MT9V03X_1_H - 1];
    int start_y  = MT9V03X_1_H - 1;
    int end_y    = corner_row;
    int end_x    = is_left_turn ? SHARP_TURN_EDGE_MARGIN
                                : MT9V03X_1_W - 1 - SHARP_TURN_EDGE_MARGIN;

    if (end_y > start_y) end_y = start_y;

    // 5. Rasterise the line from (start_x, start_y) to (end_x, end_y)
    //    and write it into process_line_mid[].
    int dx = end_x - start_x;
    int dy = end_y - start_y;
    int steps = (abs(dy) > abs(dx)) ? abs(dy) : abs(dx);
    if (steps < 1) steps = 1;

    float x_inc = (float)dx / (float)steps;
    float y_inc = (float)dy / (float)steps;
    float x = (float)start_x;
    float y = (float)start_y;

    for (int s = 0; s <= steps; s++)
    {
        int row = (int)(y + 0.5f);
        int col = (int)(x + 0.5f);
        if (row >= 0 && row < MT9V03X_1_H && col >= 0 && col < MT9V03X_1_W)
        {
            process_line_mid[row] = (int16)col;
        }
        x += x_inc;
        y += y_inc;
    }

    // Save debug overlay data for image_display_task
    turn_dbg_active  = 1;
    turn_dbg_start_x = start_x;
    turn_dbg_start_y = start_y;
    turn_dbg_end_x   = end_x;
    turn_dbg_end_y   = end_y;
    turn_dbg_is_left = is_left_turn;
}

static uint8 compute_otsu_threshold(void)
{
    // Otsu selects a threshold from the grayscale histogram. It adapts better
    // than a fixed threshold when track lighting changes.
    int histogram[256] = {0};
    int pixel_count = MT9V03X_1_H * MT9V03X_1_W;
    uint8 *img_ptr = &raw_snapshot[0][0];

    for(int i = 0; i < pixel_count; i++) {
        histogram[img_ptr[i]]++;
    }

    int sum = 0;
    for(int i = 0; i < 256; i++) {
        sum += i * histogram[i];
    }

    int sumB = 0, wB = 0, wF = 0;
    float varMax = 0.0;
    uint8 threshold = 0;

    for(int i = 0; i < 256; i++) {
        wB += histogram[i];
        if (wB == 0) continue;

        wF = pixel_count - wB;
        if (wF == 0) break;

        sumB += i * histogram[i];
        int sumF = sum - sumB;

        float varBetween = (float)sumB * sumB / wB + (float)sumF * sumF / wF;

        if (varBetween > varMax) {
            varMax = varBetween;
            threshold = i;
        }
    }

    // Clamp Otsu result, then apply an exposure-adaptive dark offset.

    if(threshold < THRESHOLD_CLAMP_LO) threshold = THRESHOLD_CLAMP_LO;
    if(threshold > THRESHOLD_CLAMP_HI) threshold = THRESHOLD_CLAMP_HI;

    {
        int range = THRESHOLD_CLAMP_HI - THRESHOLD_CLAMP_LO;
        int offset = THRESHOLD_DARK_MIN
                   + (THRESHOLD_DARK_MAX - THRESHOLD_DARK_MIN)
                   * (threshold - THRESHOLD_CLAMP_LO) / range;
        threshold += offset;
    }
    if(threshold > 245) threshold = 245;

    return threshold;
}

void image_process_task(void)
{
    if (mt9v03x_finish_flag_1 == 1)
    {
        // The camera driver sets mt9v03x_finish_flag_1 after one full DMA frame.
        // Clear it first so the next frame can be detected.
        mt9v03x_finish_flag_1 = 0;

        camera_copy_stable_frame();

        uint8 dynamic_threshold = compute_otsu_threshold();
        uint8 *src = &raw_snapshot[0][0];
        uint8 *dst = &process_image[0][0];
        int count = MT9V03X_1_H * MT9V03X_1_W;
        while (count--)
        {
#if TRACK_LINE_IS_BLACK
            *dst++ = (*src++ < dynamic_threshold) ? 255 : 0;
#else
            *dst++ = (*src++ > dynamic_threshold) ? 255 : 0;
#endif
        }

        int lost_line_count = 0; // consecutive lost-line counter

        for (int i = MT9V03X_1_H - 1; i >= 0; i--)
        {
            // Search starts from the previous row center. This makes the scan
            // faster and helps reject isolated noise far away from the lane.
            int center_seed = (i == MT9V03X_1_H - 1) ? (MT9V03X_1_W / 2) : process_line_mid[i + 1];

            if (center_seed < 0) center_seed = 0;
            if (center_seed >= MT9V03X_1_W) center_seed = MT9V03X_1_W - 1;

            int line_pos = -1;
            int left;
            int right;

            for (int span = 0; span < MT9V03X_1_W / 2; span++)
            {
                left = center_seed - span;
                right = center_seed + span;

                if ((left >= 0) && (process_image[i][left] != 0))
                {
                    line_pos = left;
                    break;
                }
                if ((right < MT9V03X_1_W) && (process_image[i][right] != 0))
                {
                    line_pos = right;
                    break;
                }
            }

            if (line_pos < 0)
            {
                process_line_mid[i] = center_seed;
                lost_line_count++;
                continue;
            }

            left = line_pos;
            right = line_pos;
            while ((left > 0) && (process_image[i][left] != 0)) left--;
            while ((right < MT9V03X_1_W - 1) && (process_image[i][right] != 0)) right++;

            if (right - left < 3)
            {
                process_line_mid[i] = center_seed;
                lost_line_count++;
            }
            else
            {
                process_line_mid[i] = (int16)((left + right) / 2);
            }
        }

        detect_boundary_sharp_turn();

        line_lost_count = (lost_line_count > 255) ? 255 : (uint8)lost_line_count;

        {
            int32 sum = 0;
            int32 weight_sum = 0;
            int start = MT9V03X_1_H / 3;
            int end   = MT9V03X_1_H * 5 / 6;

            for (int i = start; i < end; i++)
            {
                int weight = i;
                sum += (process_line_mid[i] - (MT9V03X_1_W / 2)) * weight;
                weight_sum += weight;
            }
            track_offset = (weight_sum > 0) ? (int16)(sum / weight_sum) : 0;
        }

        for (int i = 0; i < MT9V03X_1_H; i++)
        {
            line_mid[i] = process_line_mid[i];
        }

        // 7. 浜ゆ崲鏄剧ず缂撳啿
        image_swap_buffer();
        image_ready = 1;
    }
}

void image_display_task(void)
{
#if ENABLE_DISPLAY
    static uint8 refresh_cnt = 0;
    if (!image_ready) return;

    // IPS200 refresh is slow compared with image processing, so only show
    // every third processed frame. Set ENABLE_DISPLAY to 0 for race runs.
    if (++refresh_cnt < 3) return; // skip 2 of 3 frames
    refresh_cnt = 0;

    ips200_show_gray_image(0, 0, display_image[0], MT9V03X_1_W, MT9V03X_1_H, MT9V03X_1_W, MT9V03X_1_H, 0);

    for (int i = 0; i < MT9V03X_1_H; i++)
    {
        if ((display_line_mid[i] >= 0) && (display_line_mid[i] < MT9V03X_1_W))
        {
            ips200_draw_point((uint16)display_line_mid[i], (uint16)i, RGB565_RED);
        }
    }

    ips200_set_font(IPS200_6X8_FONT);
    ips200_set_color(RGB565_RED, RGB565_WHITE);
    ips200_show_string(0, 122, "GZ");
    ips200_show_int(18, 122, (int32)(gyro[2] * 57.3f), 4);
    ips200_show_string(54, 122, "GY");
    ips200_show_int(72, 122, (int32)(gyro[1] * 57.3f), 4);
    ips200_show_string(108, 122, "GX");
    ips200_show_int(126, 122, (int32)(gyro[0] * 57.3f), 4);

#if ENABLE_TURN_DEBUG
    // Sharp-turn detection debug overlay (read-only, does not affect motor)
    if (turn_dbg_active)
    {
        // Draw the generated diagonal mid-line in CYAN
        ips200_draw_line((uint16)turn_dbg_start_x, (uint16)turn_dbg_start_y,
                         (uint16)turn_dbg_end_x,   (uint16)turn_dbg_end_y,
                         RGB565_CYAN);

        // Mark start point (bottom centre) in GREEN
        ips200_draw_point((uint16)turn_dbg_start_x, (uint16)turn_dbg_start_y, RGB565_GREEN);

        // Mark end point (corner aim) in YELLOW
        ips200_draw_point((uint16)turn_dbg_end_x, (uint16)turn_dbg_end_y, RGB565_YELLOW);

        // Show junction type text at top-right
        if (turn_dbg_is_left)
        {
            ips200_show_string(MT9V03X_1_W - 30, 0, "LT");
        }
        else
        {
            ips200_show_string(MT9V03X_1_W - 30, 0, "RT");
        }
    }
#endif

#endif
}

int16 Camera_GetTrackOffset(void)
{
    return track_offset;
}

uint8 Camera_GetLineLostCount(void)
{
    return line_lost_count;
}

int16 Camera_GetCenterLine(uint8 row)
{
    if(row >= MT9V03X_1_H)
    {
        return MT9V03X_1_W / 2;
    }
    return line_mid[row];
}
