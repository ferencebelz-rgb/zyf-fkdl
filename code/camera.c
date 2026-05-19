/*
 * camera.c - MT9V03X 双摄像头图像处理与中线提取
 *
 * 本模块负责：
 *   1) 从摄像头 DMA 缓冲区获取一帧原始灰度图
 *   2) 用 Otsu 大津法自适应二值化
 *   3) 逐行搜索赛道左右边界，计算中线
 *   4) 直角弯检测（边界丢失法 + 角点定位）
 *   5) 加权平均计算赛道偏移量（供 control.c 使用）
 *   6) 显示摄像头图像 + 中线 + IMU 陀螺仪数据
 */

#include "camera.h"
#include "control.h"
#include "imu.h"
#include "task1.h"
#include "task2.h"
#include "zf_device_ips200.h"
#include "zf_device_mt9v03x_double.h"
#include "zf_driver_dma.h"
#include <string.h>

#define EXIT_SIDE_MIN_ROWS           6
#define EXIT_ROI_LEFT                (MT9V03X_1_W / 6)
#define EXIT_ROI_RIGHT               (MT9V03X_1_W * 5 / 6)
#define EXIT_ROI_TOP                 (MT9V03X_1_H / 6)
#define EXIT_ROI_BOTTOM              (MT9V03X_1_H * 5 / 6)
#define EXIT_EDGE_MARGIN             3
#define FRONT_TOP_ROWS               6
#define FRONT_HALF_WIDTH             10
#define FRONT_ROW_MIN_PIXELS         4
#define FRONT_EXIT_MIN_ROWS          3
#define JUNCTION_MISSING_UNLOCK_FRAMES 3
#define ROUTE_AIM_ADVANCE_ROWS       20

/* ======================== 编译开关 ======================== */

#define ENABLE_DISPLAY 1             /* 1=开启屏幕显示，比赛时可关闭以提高帧率 */
#define LOST_LINE_REPLACE_VAL (MT9V03X_1_W / 2)  /* 丢线时填充图像中心 */
#define TRACK_LINE_IS_BLACK 0        /* 1=白色底板黑色赛道  0=黑色底板白色赛道 */
#define ENABLE_TURN_DEBUG 1          /* 1=显示直角弯诊断线条（赛后清零 */

/* ====================== 二值化参数 ====================== */

#define THRESHOLD_DARK_MIN         30 /* Otsu 结果的最小偏移补偿 */
#define THRESHOLD_DARK_MAX         30 /* 最大偏移补偿（与 MIN 相等时固定偏移 */
#define THRESHOLD_CLAMP_LO         30 /* Otsu 结果下限 */
#define THRESHOLD_CLAMP_HI        220 /* Otsu 结果上限 */

/* ====================== 对外全局变量 ====================== */

int16 line_mid[MT9V03X_1_H];         /* 各行的中线列坐标，供 control.c 前视用 */
int16 track_offset = 0;              /* 赛道偏移量（像素），供 control.c 转向用 */
uint8 junction_type_from_camera = 0; /* 0=直道 1=T字路口 2=十字路口 3=急转弯/直角弯 */
static uint8 line_lost_count = 0;    /* 连续丢线行数统计 */

/* =================== 双缓冲缓冲区定义 =================== */

/* raw_snapshot：从 DMA 缓冲区稳定拷贝的一帧原始灰度图 */
static uint8 raw_snapshot[MT9V03X_1_H][MT9V03X_1_W];
static uint8 binary_buf_0[MT9V03X_1_H][MT9V03X_1_W];
static uint8 binary_buf_1[MT9V03X_1_H][MT9V03X_1_W];

static int16 line_mid_buf_0[MT9V03X_1_H];
static int16 line_mid_buf_1[MT9V03X_1_H];

/* 处理缓冲区和显示缓冲区指针，每帧交换一次，避免 DMA 和显示冲突 */
static uint8 (*process_image)[MT9V03X_1_W] = binary_buf_0;
static int16 *process_line_mid = line_mid_buf_0;

static uint8 (*display_image)[MT9V03X_1_W] = binary_buf_1;
static int16 *display_line_mid = line_mid_buf_1;

static uint8 image_ready = 0;

/* 直角弯调试叠加数据（process 写入，display 读取 */
static int  turn_dbg_active  = 0;
static int  turn_dbg_start_x = 0;
static int  turn_dbg_start_y = 0;
static int  turn_dbg_end_x   = 0;
static int  turn_dbg_end_y   = 0;
static int  turn_dbg_is_left = 0;

/* 路口检测状态（control.c 和显示共用 */
static junction_kind_t   current_junction_kind   = JUNCTION_NONE;
static route_decision_t  current_route_decision  = DECISION_STRAIGHT;
static uint8             junction_error          = 0;

/* ==================== 初始化函数 ==================== */

void cam_init(void)
{
    /* 设置曝光时间为 400（默认值，可根据环境调整 */
    mt9v03x_set_confing_buffer_1[MT9V03X_DOUBLE_EXP_TIME][1] = 400;
    mt9v03x_double_init(mt9v03x_1);
}

/* ================== 缓冲区交换 ================== */

static void image_swap_buffer(void)
{
    /* O(1) 时间交换 process/display 指针，避免大块内存拷贝 */
    uint8 (*temp_image)[MT9V03X_1_W] = process_image;
    process_image = display_image;
    display_image = temp_image;

    int16 *temp_line_mid = process_line_mid;
    process_line_mid = display_line_mid;
    display_line_mid = temp_line_mid;
}

/* ================== 稳定帧拷贝 ================== */

static void camera_copy_stable_frame(void)
{
    /* 在 DMA 中断间隙将摄像头图像拷贝到 raw_snapshot，保证处理时不会撕裂 */
    memcpy(raw_snapshot[0], mt9v03x_image_1[0], MT9V03X_1_W * MT9V03X_1_H);
}


/* =============== 十字路口与T字路口检测 =============== */

/*
 * 原理（参考草莽）：
 *   统计四边缘（底/顶/左/右）的边界丢失情况。
 *   四边缘全丢 = 十字路口 → 底部中线垂直向上直行。
 *   底+顶丢失、左右正常 = T字路口 → 底部中线垂直向上直行。
 *   底+左/右丢失已由直角弯检测处理。
 */
static int clamp_route_aim_row(int aim_row, int start_y)
{
    aim_row -= ROUTE_AIM_ADVANCE_ROWS;
    if(aim_row < 0) aim_row = 0;
    if(aim_row > start_y) aim_row = start_y;
    return aim_row;
}

static int find_boundary_corner_row(route_decision_t decision,
                                    int left_edge[],
                                    int right_edge[],
                                    int fallback_row)
{
    int corner_row = fallback_row;
    int max_jump = 0;
    int *edge = (decision == DECISION_LEFT) ? left_edge : right_edge;

    for(int i = MT9V03X_1_H - 2; i >= 1; i--)
    {
        if(edge[i] < 0 || edge[i + 1] < 0) continue;
        int jump = edge[i] - edge[i + 1];
        if(jump < 0) jump = -jump;
        if(jump > max_jump)
        {
            max_jump = jump;
            corner_row = i;
        }
    }

    if(max_jump < SHARP_TURN_CORNER_DERIV || corner_row < 0)
    {
        corner_row = fallback_row;
    }
    if(corner_row < 0)
    {
        corner_row = EXIT_ROI_TOP;
    }
    return corner_row;
}

static void draw_route_line(route_decision_t decision,
                            junction_kind_t kind,
                            int left_edge[],
                            int right_edge[],
                            int left_exit_row,
                            int right_exit_row,
                            int entry_center,
                            int row_start,
                            int row_end)
{
    int h = MT9V03X_1_H;
    int w = MT9V03X_1_W;
    int margin = SHARP_TURN_EDGE_MARGIN;
    int start_x = (int)process_line_mid[h - 1];
    int start_y = h - 1;
    int aim_row = EXIT_ROI_TOP;
    int end_y;
    int end_x = w / 2;

    if(start_x < 0 || start_x >= w) start_x = w / 2;
    if(entry_center < 0 || entry_center >= w) entry_center = start_x;
    if(row_start < 0) row_start = 0;
    if(row_end > h) row_end = h;
    if(row_end < row_start) row_end = row_start;

    if(decision == DECISION_STRAIGHT)
    {
        for(int i = row_start; i < row_end; i++)
        {
            process_line_mid[i] = (int16)entry_center;
        }
        turn_dbg_active = 0;
        return;
    }

    end_x = (decision == DECISION_LEFT) ? margin : (w - 1 - margin);
    if(kind == JUNCTION_LEFT_CORNER || kind == JUNCTION_RIGHT_CORNER)
    {
        int fallback_row = (decision == DECISION_LEFT) ? left_exit_row : right_exit_row;
        aim_row = find_boundary_corner_row(decision, left_edge, right_edge, fallback_row);
    }
    else
    {
        aim_row = (decision == DECISION_LEFT) ? left_exit_row : right_exit_row;
        if(aim_row < 0) aim_row = EXIT_ROI_TOP;
    }
    end_y = clamp_route_aim_row(aim_row, start_y);

    int dx = end_x - start_x;
    int dy = end_y - start_y;
    int steps = (abs(dy) > abs(dx)) ? abs(dy) : abs(dx);
    if(steps < 1) steps = 1;

    float x_inc = (float)dx / (float)steps;
    float y_inc = (float)dy / (float)steps;
    float x = (float)start_x;
    float y = (float)start_y;

    for(int s = 0; s <= steps; s++)
    {
        int row = (int)(y + 0.5f);
        int col = (int)(x + 0.5f);
        if(row >= 0 && row < h && col >= 0 && col < w)
        {
            process_line_mid[row] = (int16)col;
        }
        x += x_inc;
        y += y_inc;
    }
    for(int row = 0; row < end_y; row++)
    {
        process_line_mid[row] = (int16)end_x;
    }

    turn_dbg_active  = 1;
    turn_dbg_start_x = start_x;
    turn_dbg_start_y = start_y;
    turn_dbg_end_x   = end_x;
    turn_dbg_end_y   = end_y;
    turn_dbg_is_left = (decision == DECISION_LEFT);
}

static uint8 decision_is_valid(junction_kind_t kind, route_decision_t decision)
{
    if(kind == JUNCTION_CROSS) return 1;
    if(kind == JUNCTION_SIDE_LEFT_T)
    {
        return (decision == DECISION_LEFT || decision == DECISION_STRAIGHT);
    }
    if(kind == JUNCTION_SIDE_RIGHT_T)
    {
        return (decision == DECISION_RIGHT || decision == DECISION_STRAIGHT);
    }
    if(kind == JUNCTION_STANDARD_T)
    {
        return (decision == DECISION_LEFT || decision == DECISION_RIGHT);
    }
    return 0;
}

static junction_kind_t classify_exits(uint8 front_exists, uint8 left_exists, uint8 right_exists)
{
    if(front_exists && left_exists && right_exists) return JUNCTION_CROSS;
    if(front_exists && left_exists && !right_exists) return JUNCTION_SIDE_LEFT_T;
    if(front_exists && !left_exists && right_exists) return JUNCTION_SIDE_RIGHT_T;
    if(!front_exists && left_exists && right_exists) return JUNCTION_STANDARD_T;
    if(!front_exists && left_exists && !right_exists) return JUNCTION_LEFT_CORNER;
    if(!front_exists && !left_exists && right_exists) return JUNCTION_RIGHT_CORNER;
    return JUNCTION_NONE;
}

static uint8 junction_needs_route_decision(junction_kind_t kind)
{
    return (kind == JUNCTION_CROSS ||
            kind == JUNCTION_SIDE_LEFT_T ||
            kind == JUNCTION_SIDE_RIGHT_T ||
            kind == JUNCTION_STANDARD_T) ? 1 : 0;
}

static void detect_exits(int left_edge[], int right_edge[],
                         uint8 *front_exists,
                         uint8 *left_exists,
                         uint8 *right_exists,
                         int *left_exit_row,
                         int *right_exit_row,
                         int *entry_center_out,
                         int *row_start_out,
                         int *row_end_out)
{
    int h = MT9V03X_1_H;
    int w = MT9V03X_1_W;
    int row_start = EXIT_ROI_TOP;
    int row_end = EXIT_ROI_BOTTOM;
    int front_row_end = row_start + FRONT_TOP_ROWS;
    int entry_row_start = row_end - FRONT_TOP_ROWS;
    int entry_sum = 0;
    int entry_count = 0;
    int entry_center = w / 2;
    int left_hits = 0;
    int right_hits = 0;
    int front_hits = 0;
    int left_row_sum = 0;
    int right_row_sum = 0;

    if(front_row_end > row_end) front_row_end = row_end;
    if(entry_row_start < row_start) entry_row_start = row_start;

    for(int row = entry_row_start; row < row_end; row++)
    {
        int center = process_line_mid[row];
        if(center >= 0 && center < w)
        {
            entry_sum += center;
            entry_count++;
        }
    }
    if(entry_count > 0)
    {
        entry_center = entry_sum / entry_count;
    }

    for(int row = row_start; row < row_end; row++)
    {
        if(left_edge[row] >= 0 && left_edge[row] <= EXIT_ROI_LEFT + EXIT_EDGE_MARGIN)
        {
            left_hits++;
            left_row_sum += row;
        }
        if(right_edge[row] >= 0 && right_edge[row] >= EXIT_ROI_RIGHT - EXIT_EDGE_MARGIN)
        {
            right_hits++;
            right_row_sum += row;
        }
    }

    for(int row = row_start; row < front_row_end; row++)
    {
        int pixel_hits = 0;
        int col_start = entry_center - FRONT_HALF_WIDTH;
        int col_end = entry_center + FRONT_HALF_WIDTH;
        if(col_start < 0) col_start = 0;
        if(col_end >= w) col_end = w - 1;

        for(int col = col_start; col <= col_end; col++)
        {
            if(process_image[row][col] != 0)
            {
                pixel_hits++;
            }
        }
        if(pixel_hits >= FRONT_ROW_MIN_PIXELS)
        {
            front_hits++;
        }
    }

    *front_exists = (front_hits >= FRONT_EXIT_MIN_ROWS) ? 1 : 0;
    *left_exists = (left_hits >= EXIT_SIDE_MIN_ROWS) ? 1 : 0;
    *right_exists = (right_hits >= EXIT_SIDE_MIN_ROWS) ? 1 : 0;
    *left_exit_row = (left_hits > 0) ? (left_row_sum / left_hits) : -1;
    *right_exit_row = (right_hits > 0) ? (right_row_sum / right_hits) : -1;
    *entry_center_out = entry_center;
    *row_start_out = row_start;
    *row_end_out = row_end;
}

static void detect_cross_t_junction(int left_edge[], int right_edge[])
{
    static uint8 junction_locked = 0;
    static uint8 junction_missing_frames = 0;
    static route_decision_t locked_decision = DECISION_STRAIGHT;
    static junction_kind_t locked_kind = JUNCTION_NONE;

    uint8 front_exists = 0;
    uint8 left_exists = 0;
    uint8 right_exists = 0;
    int left_exit_row = -1;
    int right_exit_row = -1;
    int entry_center = 0;
    int row_start = 0;
    int row_end = 0;
    junction_kind_t detected_kind;

    detect_exits(left_edge, right_edge, &front_exists, &left_exists, &right_exists,
                 &left_exit_row, &right_exit_row, &entry_center, &row_start, &row_end);
    detected_kind = classify_exits(front_exists, left_exists, right_exists);

    if(detected_kind == JUNCTION_NONE)
    {
        current_junction_kind = JUNCTION_NONE;
        if(junction_locked)
        {
            junction_missing_frames++;
            if(junction_missing_frames >= JUNCTION_MISSING_UNLOCK_FRAMES)
            {
                junction_locked = 0;
                locked_kind = JUNCTION_NONE;
                locked_decision = DECISION_STRAIGHT;
            }
        }
        return;
    }

    junction_missing_frames = 0;
    if(!junction_locked)
    {
        locked_kind = detected_kind;
        if(junction_needs_route_decision(detected_kind)
           && Control_GetTaskMode() == TASK_MODE_2)
            locked_decision = Task2_GetRouteDecision();
        else if(detected_kind == JUNCTION_LEFT_CORNER)
            locked_decision = DECISION_LEFT;
        else if(detected_kind == JUNCTION_RIGHT_CORNER)
            locked_decision = DECISION_RIGHT;
        else
            locked_decision = DECISION_STRAIGHT;
        junction_locked = 1;
    }

    current_junction_kind = locked_kind;
    current_route_decision = locked_decision;
    junction_error = junction_needs_route_decision(locked_kind) ?
                     (decision_is_valid(locked_kind, locked_decision) ? 0 : 1) :
                     0;
    if(junction_error)
    {
        junction_type_from_camera = 0;
        turn_dbg_active = 0;
        return;
    }

    if(locked_kind == JUNCTION_LEFT_CORNER || locked_kind == JUNCTION_RIGHT_CORNER)
    {
        junction_type_from_camera = 3;
    }
    else if(locked_decision == DECISION_STRAIGHT)
    {
        junction_type_from_camera = (locked_kind == JUNCTION_CROSS) ? 2 : 1;
    }
    else
    {
        junction_type_from_camera = 3;
    }
    draw_route_line(locked_decision, locked_kind, left_edge, right_edge,
                    left_exit_row, right_exit_row, entry_center, row_start, row_end);
}

/* =============== Otsu 大津法自适应二值化 =============== */

static uint8 compute_otsu_threshold(void)
{
    /*
     * 大津法自动选取最优二值化阈值：
     *   遍历 0-255 所有灰度级，计算类间方差（between-class variance），
     *   取方差最大的灰度级作为阈值。
     *   优点是自适应光照变化，比固定阈值更鲁棒。
     */
    int histogram[256] = {0};
    int pixel_count = MT9V03X_1_H * MT9V03X_1_W;
    uint8 *img_ptr = &raw_snapshot[0][0];

    /* 统计灰度直方图 */
    for(int i = 0; i < pixel_count; i++) {
        histogram[img_ptr[i]]++;
    }

    /* 计算全局灰度总和，用于后续计算类间方差 */
    int sum = 0;
    for(int i = 0; i < 256; i++) {
        sum += i * histogram[i];
    }

    int sumB = 0, wB = 0, wF = 0;
    float varMax = 0.0;
    uint8 threshold = 0;

    /* 遍历所有灰度级，找使类间方差最大的阈值 */
    for(int i = 0; i < 256; i++) {
        wB += histogram[i];            /* 前景像素数 */
        if (wB == 0) continue;

        wF = pixel_count - wB;         /* 背景像素数 */
        if (wF == 0) break;

        sumB += i * histogram[i];      /* 前景灰度累加 */
        int sumF = sum - sumB;         /* 背景灰度累加 */

        /* 类间方差公式：Var = wB * wF * (uB - uF)^2 */
        float varBetween = (float)sumB * sumB / wB + (float)sumF * sumF / wF;

        if (varBetween > varMax) {
            varMax = varBetween;
            threshold = i;
        }
    }

    /* 对大津法结果做限幅，避免极端值 */
    if(threshold < THRESHOLD_CLAMP_LO) threshold = THRESHOLD_CLAMP_LO;
    if(threshold > THRESHOLD_CLAMP_HI) threshold = THRESHOLD_CLAMP_HI;

    /* 增加曝光自适应偏移（暗场调高阈值压噪，亮场调低保留细节 */
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

/* =============== 主处理任务 =============== */

void image_process_task(void)
{
    /* 检查摄像头一帧是否采集完毕 */
    if (mt9v03x_finish_flag_1 == 1)
    {
        mt9v03x_finish_flag_1 = 0;

        /* 步骤 1：稳定拷贝一帧到 raw_snapshot */
        camera_copy_stable_frame();

        /* 步骤 2：Otsu 自适应二值化 */
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

        // 3x3 去噪：孤立白点（邻域白点数 < 2）视为噪声抹掉
        {
            static uint8 clean[MT9V03X_1_H][MT9V03X_1_W];  /* 放静态区，避免栈溢出 */
            for (int y = 1; y < MT9V03X_1_H - 1; y++)
            {
                for (int x = 1; x < MT9V03X_1_W - 1; x++)
                {
                    if (!process_image[y][x]) { clean[y][x] = 0; continue; }
                    int nb = 0;
                    if (process_image[y-1][x-1]) nb++;
                    if (process_image[y-1][x  ]) nb++;
                    if (process_image[y-1][x+1]) nb++;
                    if (process_image[y  ][x-1]) nb++;
                    if (process_image[y  ][x+1]) nb++;
                    if (process_image[y+1][x-1]) nb++;
                    if (process_image[y+1][x  ]) nb++;
                    if (process_image[y+1][x+1]) nb++;
                    clean[y][x] = (nb >= 2) ? 255 : 0;
                }
            }
            // 边界行/列保持不变
            for (int x = 0; x < MT9V03X_1_W; x++)
            {
                clean[0][x] = process_image[0][x];
                clean[MT9V03X_1_H-1][x] = process_image[MT9V03X_1_H-1][x];
            }
            for (int y = 0; y < MT9V03X_1_H; y++)
            {
                clean[y][0] = process_image[y][0];
                clean[y][MT9V03X_1_W-1] = process_image[y][MT9V03X_1_W-1];
            }
            memcpy(&process_image[0][0], &clean[0][0], MT9V03X_1_H * MT9V03X_1_W);
        }

        int lost_line_count = 0;  /* 本轮连续丢线的行数统计 */
        int boundary_left[MT9V03X_1_H];   /* 每行赛道左边界，供直角弯检测复用 */
        int boundary_right[MT9V03X_1_H];  /* 每行赛道右边界，供直角弯检测复用 */

        int gap_start = -1;  /* 当前空隙的底部行号，-1 表示不在空隙中 */

        /* 步骤 3：逐行扫描中线（从近处往远处扫 */
        for (int i = MT9V03X_1_H - 1; i >= 0; i--)
        {
            /* 从上一行中线位置出发搜索，提高速度并抑制噪声 */
            int center_seed = (i == MT9V03X_1_H - 1) ? (MT9V03X_1_W / 2) : process_line_mid[i + 1];
            if (center_seed < 0) center_seed = 0;
            if (center_seed >= MT9V03X_1_W) center_seed = MT9V03X_1_W - 1;

            int line_pos = -1;
            int left;
            int right;

            /* 从种子点向两侧扩散搜索白色像素 */
            for (int span = 0; span < MT9V03X_1_W / 2; span++)
            {
                left  = center_seed - span;
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

            /* 该行完全找不到赛道 — 丢线，用种子点填充 */
            if (line_pos < 0)
            {
                if (gap_start < 0) gap_start = i;
                boundary_left[i]  = -1;
                boundary_right[i] = -1;
                process_line_mid[i] = center_seed;
                lost_line_count++;
                continue;
            }

            /* 找到赛道后，向左右扩展找到完整边界 */
            left = line_pos;
            right = line_pos;
            while ((left > 0) && (process_image[i][left] != 0)) left--;
            while ((right < MT9V03X_1_W - 1) && (process_image[i][right] != 0)) right++;

            boundary_left[i]  = left;
            boundary_right[i] = right;

            /* 如果赛道宽度小于 3 像素，视为噪声，用种子点代替 */
            if (right - left < 3)
            {
                if (gap_start < 0) gap_start = i;
                process_line_mid[i] = center_seed;
                lost_line_count++;
                continue;
            }

            int mid = (int16)((left + right) / 2);

            /* 刚退出一个小空隙（≤10 行），用上下有效中线线性插值补齐 */
            if (gap_start >= 0)
            {
                int gap_size = gap_start - i;
                if (gap_size <= 20 && gap_start < MT9V03X_1_H - 1)
                {
                    int mid_below = process_line_mid[gap_start + 1];
                    for (int j = gap_start; j > i; j--)
                    {
                        float t = (float)(gap_start - j) / (float)gap_size;
                        process_line_mid[j] = (int16)(mid_below + (mid - mid_below) * t);
                    }
                }
                gap_start = -1;
            }

            process_line_mid[i] = mid;
        }

        // 黑区补线：取丢失段前后各5行的有效中线，直线连接
        {
            uint8 lost_flag[MT9V03X_1_H];
            for (int i = 0; i < MT9V03X_1_H; i++)
                lost_flag[i] = (boundary_left[i] < 0) ? 1 : 0;

            for (int i = 1; i < MT9V03X_1_H - 1; )
            {
                if (!lost_flag[i]) { i++; continue; }
                int gap_start = i;
                while (i < MT9V03X_1_H - 1 && lost_flag[i]) i++;
                int gap_end = i - 1;
                if (gap_start < 5 || gap_end >= MT9V03X_1_H - 6) continue;

                int16 val_before = process_line_mid[gap_start - 5];
                int16 val_after  = process_line_mid[gap_end + 5];
                if (val_before < 0 || val_after < 0) continue;

                for (int r = gap_start; r <= gap_end; r++)
                {
                    process_line_mid[r] = val_before
                        + (int16)((val_after - val_before) * (r - (gap_start - 5)) / (gap_end + 5 - (gap_start - 5) + 1));
                }
            }
        }

        /* Step 4: classify cross, T-junctions, and corners from exit flags. */
        junction_type_from_camera = 0;
        detect_cross_t_junction(boundary_left, boundary_right);

        line_lost_count = (lost_line_count > 255) ? 255 : (uint8)lost_line_count;

        /* 步骤 5：加权平均计算赛道偏移量（近处行权重大）
         *   只取画面中间段（1/3 到 5/6），忽略顶部太远的行和底部太近的行 */
        {
            int32 sum = 0;
            int32 weight_sum = 0;
            int start = MT9V03X_1_H / 3;
            int end   = MT9V03X_1_H * 5 / 6;

            for (int i = start; i < end; i++)
            {
                int weight = i;  /* 越靠近底部的行权重越大 */
                sum += (process_line_mid[i] - (MT9V03X_1_W / 2)) * weight;
                weight_sum += weight;
            }
            track_offset = (weight_sum > 0) ? (int16)(sum / weight_sum) : 0;
        }

        /* 步骤 6：将结果同步到对外数组给 motor.c 使用 */
        for (int i = 0; i < MT9V03X_1_H; i++)
        {
            line_mid[i] = process_line_mid[i];
        }

        /* 步骤 7：交换处理/显示缓冲区 */
        image_swap_buffer();
        image_ready = 1;
    }
}

/* =============== 显示任务 =============== */

void image_display_task(void)
{
#if ENABLE_DISPLAY
    static uint8 refresh_cnt = 0;
    if (!image_ready) return;

    /* 降低显示刷新率（每 3 帧显示一次），为图像处理腾出 CPU */
    if (++refresh_cnt < 3) return;
    refresh_cnt = 0;

    /* 显示二值化后的摄像头图像 */
    ips200_show_gray_image(0, 0, display_image[0], MT9V03X_1_W, MT9V03X_1_H, MT9V03X_1_W, MT9V03X_1_H, 0);

    /* 用红点画中线、蓝点画左边界、绿点画右边界 */
    for (int i = 0; i < MT9V03X_1_H; i++)
    {
        if ((display_line_mid[i] >= 0) && (display_line_mid[i] < MT9V03X_1_W))
        {
            ips200_draw_point((uint16)display_line_mid[i], (uint16)i, RGB565_RED);
        }

        int l = -1, r = -1;
        for (int j = 0; j < MT9V03X_1_W; j++)
        {
            if (display_image[i][j])
            {
                if (l < 0) l = j;
                r = j;
            }
        }
        if (l >= 0) ips200_draw_point((uint16)l, (uint16)i, RGB565_CYAN);
        if (r >= 0 && r > l) ips200_draw_point((uint16)r, (uint16)i, RGB565_YELLOW);
    }

    /* IMU 陀螺仪数据显示在摄像头图像下方 */
    ips200_set_font(IPS200_6X8_FONT);
    ips200_set_color(RGB565_RED, RGB565_WHITE);
    ips200_show_string(0, 122, "GZ");
    ips200_show_int(18, 122, (int32)(gyro[2] * 57.3f), 4);
    ips200_show_string(54, 122, "GY");
    ips200_show_int(72, 122, (int32)(gyro[1] * 57.3f), 4);
    ips200_show_string(108, 122, "GX");
    ips200_show_int(126, 122, (int32)(gyro[0] * 57.3f), 4);

    ips200_show_string(0, 140, "T:");
    if(Control_GetTaskMode() == TASK_MODE_2)
    {
        ips200_show_int(18, 140, Task2_GetCount(), 2);
        ips200_show_string(36, 140, "/");
        ips200_show_int(42, 140, TASK2_TURN_TARGET, 2);
    }
    else
    {
        ips200_show_int(18, 140, Task1_GetCount(), 2);
        ips200_show_string(36, 140, "/");
        ips200_show_int(42, 140, TASK1_TURN_TARGET, 2);
    }

    // 路口类型与决策显示（始终显示，不只错误时才显示）
    if(junction_error) ips200_show_string(72, 140, "ERR");
    else               ips200_show_string(72, 140, "   ");

    if(current_junction_kind == JUNCTION_CROSS)
        ips200_show_string(96, 140, "CRS");
    else if(current_junction_kind == JUNCTION_SIDE_LEFT_T)
        ips200_show_string(96, 140, "SLT");
    else if(current_junction_kind == JUNCTION_SIDE_RIGHT_T)
        ips200_show_string(96, 140, "SRT");
    else if(current_junction_kind == JUNCTION_STANDARD_T)
        ips200_show_string(96, 140, "STD");
    else if(current_junction_kind == JUNCTION_LEFT_CORNER)
        ips200_show_string(96, 140, "L90");
    else if(current_junction_kind == JUNCTION_RIGHT_CORNER)
        ips200_show_string(96, 140, "R90");
    else
        ips200_show_string(96, 140, "   ");

    if(current_route_decision == DECISION_LEFT)
        ips200_show_string(120, 140, "L");
    else if(current_route_decision == DECISION_RIGHT)
        ips200_show_string(120, 140, "R");
    else if(current_route_decision == DECISION_STRAIGHT)
        ips200_show_string(120, 140, "S");
    else
        ips200_show_string(120, 140, " ");
    }


#if ENABLE_TURN_DEBUG
    /* 直角弯诊断叠加层 */
    if (turn_dbg_active)
    {
        /* 青色斜线表示生成的转弯中线 */
        ips200_draw_line((uint16)turn_dbg_start_x, (uint16)turn_dbg_start_y,
                         (uint16)turn_dbg_end_x,   (uint16)turn_dbg_end_y,
                         RGB565_CYAN);

        /* 绿色点标记起点（底部中心） */
        ips200_draw_point((uint16)turn_dbg_start_x, (uint16)turn_dbg_start_y, RGB565_GREEN);

        /* 黄色点标记终点（弯角瞄准点） */
        ips200_draw_point((uint16)turn_dbg_end_x, (uint16)turn_dbg_end_y, RGB565_YELLOW);

        /* 右上角显示转弯方向 */
        if (turn_dbg_is_left)
        {
            ips200_show_string(MT9V03X_1_W - 30, 0, "LT"); /* Left Turn */
        }
        else
        {
            ips200_show_string(MT9V03X_1_W - 30, 0, "RT"); /* Right Turn */
        }
    }
#endif

#endif
}

/* ====================== 对外接口 ====================== */

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


uint8 Camera_GetJunctionError(void)
{
    return junction_error;
}

junction_kind_t Camera_GetJunctionKind(void)
{
    return current_junction_kind;
}

route_decision_t Camera_GetRouteDecision(void)
{
    return current_route_decision;
}
