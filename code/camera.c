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
uint8 junction_type_from_camera = 0; /* 0=直道 1=T字路口 2=十字 3=直角弯 */
uint8 junction_side = 0;           /* 0=无路口, 1=左转, 2=右转 */
uint8 t_junction_seen = 0;         /* 当前帧检测到T字路口（含直行通过的 */
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
static int  turn_dbg_corner_x = 0;
static int  turn_dbg_corner_y = 0;
static int  turn_dbg_is_left = 0;

static void draw_clamped_box(int x, int y, int radius, uint16 color)
{
    int x0 = x - radius;
    int x1 = x + radius;
    int y0 = y - radius;
    int y1 = y + radius;

    if (x0 < 0) x0 = 0;
    if (x1 >= MT9V03X_1_W) x1 = MT9V03X_1_W - 1;
    if (y0 < 0) y0 = 0;
    if (y1 >= MT9V03X_1_H) y1 = MT9V03X_1_H - 1;

    for (int px = x0; px <= x1; px++)
    {
        ips200_draw_point((uint16)px, (uint16)y0, color);
        ips200_draw_point((uint16)px, (uint16)y1, color);
    }
    for (int py = y0; py <= y1; py++)
    {
        ips200_draw_point((uint16)x0, (uint16)py, color);
        ips200_draw_point((uint16)x1, (uint16)py, color);
    }
}

/* ==================== 初始化函数 ==================== */

static void get_turn_detect_box(uint16 *x0, uint16 *y0, uint16 *x1, uint16 *y1)
{
    uint16 box_w = MT9V03X_1_W / 2;
    uint16 box_h = MT9V03X_1_H / 2;

    *x0 = (MT9V03X_1_W - box_w) / 2 - 24;
    *y0 = (MT9V03X_1_H - box_h) / 2;
    *x1 = *x0 + box_w + 48;
    *y1 = MT9V03X_1_H - 1;
}

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

/* =============== 直角转弯检测（边界跟踪法） =============== */

/*
 * 原理：
 *   从左下往右上逐行扫描，分别跟踪左右边界。
 *   当某侧边界触及图像边缘（丢失），而另一侧边界依然可见时，判定为直角弯。
 *   然后用"角点定位"找到边界跳动最大的行，从该行到底部中心画一条斜线作为新中线。
 *   参考实现来自草莽。
 */
static void detect_boundary_sharp_turn(int left_edge[], int right_edge[])
{
    int left_lost_row = -1;        /* 左侧丢线的起始行，-1 表示未丢 */
    int right_lost_row = -1;       /* 右侧丢线的起始行 */
    int left_visible  = 0;         /* 左侧可见的行数统计 */
    int right_visible = 0;         /* 右侧可见的行数统计 */

    /* 步骤 1：使用搜线阶段提取的左右边界（已过滤赛道外噪声） */
    for (int i = 0; i < MT9V03X_1_H; i++)
    {
        int l = left_edge[i];
        int r = right_edge[i];

        /* 左边界触及图像左边缘 — 判定左侧丢线 */
        if (l >= 0 && l <= SHARP_TURN_EDGE_MARGIN)
        {
            if (left_lost_row < 0) left_lost_row = i;
        }
        else if (l > SHARP_TURN_EDGE_MARGIN)
        {
            left_visible++;  /* 左边界正常可见 */
        }

        /* 右边界触及图像右边缘 — 判定右侧丢线 */
        if (r >= 0 && r >= MT9V03X_1_W - 1 - SHARP_TURN_EDGE_MARGIN)
        {
            if (right_lost_row < 0) right_lost_row = i;
        }
        else if (r >= 0 && r < MT9V03X_1_W - 1 - SHARP_TURN_EDGE_MARGIN)
        {
            right_visible++;
        }
    }

    /* 步骤 2：判断是否为直角弯（一侧丢线且另一侧可见行数足够多 */
    int is_left_turn  = (left_lost_row >= 0)  && (left_lost_row > MT9V03X_1_H / 2)
                        && (right_visible > MT9V03X_1_H / 4);
    int is_right_turn = (right_lost_row >= 0) && (right_lost_row > MT9V03X_1_H / 2)
                        && (left_visible > MT9V03X_1_H / 4);

    if (!is_left_turn && !is_right_turn)
    {
        junction_type_from_camera = 0;  /* 未检测到直角弯，清零 */
        turn_dbg_active = 0;
        return;
    }

    junction_type_from_camera = 3;  /* 标记为急转弯 */

    /*
     * 步骤 3：角点定位
     *   边界跳动最大的行就是弯角所在行。从底部往上扫描，
     *   计算每行边界位置的变化率（导数），取最大跳变点。
     *   比直接用丢线行更精确。
     */
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

    /* 如果跳变不够明显，回退到丢线起始行 */
    if (max_jump < SHARP_TURN_CORNER_DERIV)
    {
        corner_row = is_left_turn ? left_lost_row : right_lost_row;
        if (corner_row < 0) corner_row = MT9V03X_1_H - 1;
    }

    /*
     * 步骤 4：生成斜线中线
     *   从底部中心画到弯角所在点，以此作为转弯期间的参考中线，
     *   代替实际丢失的赛道边界。
     */
    int start_x  = (int)process_line_mid[MT9V03X_1_H - 1];
    int start_y  = MT9V03X_1_H - 1;
    int end_y    = corner_row;
    int end_x    = is_left_turn ? SHARP_TURN_EDGE_MARGIN
                                : MT9V03X_1_W - 1 - SHARP_TURN_EDGE_MARGIN;
    int corner_x = edge[corner_row];

    if (corner_x < 0 || corner_x >= MT9V03X_1_W)
        corner_x = is_left_turn ? 0 : (MT9V03X_1_W - 1);

    // 终点行与拐点行保持一致
    if (end_y < 0) end_y = 0;
    if (end_y > start_y) end_y = start_y;

    /* 步骤 5：用 DDA 光栅化直线，写入 process_line_mid[] */
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

    // 斜线上方行统一指向终点，消除黑色区域残留红线
    for (int row = 0; row < end_y; row++)
    {
        process_line_mid[row] = (int16)end_x;
    }

    /* 保存调试用数据，供 image_display_task 画线 */
    turn_dbg_active  = 1;
    turn_dbg_start_x = start_x;
    turn_dbg_start_y = start_y;
    turn_dbg_end_x   = end_x;
    turn_dbg_end_y   = end_y;
    turn_dbg_corner_x = corner_x;
    turn_dbg_corner_y = corner_row;
    turn_dbg_is_left = is_left_turn;
}

/* =============== Otsu 大津法自适应二值化 =============== */

/* =============== 框检测直角弯（task1 使用） =============== */
static void detect_box_sharp_turn(void)
{
    uint16 x0, y0, x1, y1;
    uint8 top_hit = 0, bottom_hit = 0, left_hit = 0, right_hit = 0;
    uint16 side_scan_start;
    int left_hit_y = -1;
    int right_hit_y = -1;

    get_turn_detect_box(&x0, &y0, &x1, &y1);
    side_scan_start = y0 + 1;
    if (side_scan_start <= MT9V03X_1_H / 2)
        side_scan_start = MT9V03X_1_H / 2 + 1;

    for (uint16 x = x0; x <= x1 && !top_hit; x++)
        if (process_image[y0][x] != 0) top_hit = 1;
    for (uint16 x = x0; x <= x1 && !bottom_hit; x++)
        if (process_image[y1][x] != 0) bottom_hit = 1;
    for (uint16 y = side_scan_start; y < y1 && !left_hit; y++)
    {
        if (process_image[y][x0] != 0)
        {
            left_hit = 1;
            left_hit_y = y;
        }
    }
    for (uint16 y = side_scan_start; y < y1 && !right_hit; y++)
    {
        if (process_image[y][x1] != 0)
        {
            right_hit = 1;
            right_hit_y = y;
        }
    }

    int is_left_turn = bottom_hit && left_hit && !right_hit && !top_hit;
    int is_right_turn = bottom_hit && right_hit && !left_hit && !top_hit;

    if (!is_left_turn && !is_right_turn)
    {
        junction_type_from_camera = 0;
        turn_dbg_active = 0;
        return;
    }

    /* 拐角确认：命中行上下5行内，至少2行从边界向内连续50白点才认定出口 */
    {
        int hit_y = is_left_turn ? left_hit_y : right_hit_y;
        int edge_x = is_left_turn ? (int)x0 : (int)x1;
        int col_step = is_left_turn ? 1 : -1;
        int chk_top = hit_y - 5;
        int chk_bot = hit_y + 5;
        if (chk_top < 0) chk_top = 0;
        if (chk_bot >= MT9V03X_1_H) chk_bot = MT9V03X_1_H - 1;

        int pass_rows = 0;
        for (int row = chk_top; row <= chk_bot; row++)
        {
            int run = 0;
            for (int col = edge_x; col >= 0 && col < MT9V03X_1_W; col += col_step)
            {
                if (process_image[row][col] != 0)
                    run++;
                else
                    break;
            }
            if (run >= 50) pass_rows++;
        }

        if (pass_rows < 2)
        {
            junction_type_from_camera = 0;
            turn_dbg_active = 0;
            return;
        }
    }

    int start_x = (int)process_line_mid[MT9V03X_1_H - 1];
    int start_y = MT9V03X_1_H - 1;
    int corner_x = is_left_turn ? (int)x0 : (int)x1;
    int corner_y = is_left_turn ? left_hit_y : right_hit_y;
    int end_x = is_left_turn ? SHARP_TURN_EDGE_MARGIN
                             : MT9V03X_1_W - 1 - SHARP_TURN_EDGE_MARGIN;
    int end_y;

    if (start_x < 0 || start_x >= MT9V03X_1_W) start_x = MT9V03X_1_W / 2;
    if (corner_y < 0) corner_y = y1;

    end_y = corner_y;
    if (end_y < 0) end_y = 0;
    if (end_y > start_y) end_y = start_y;

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
            process_line_mid[row] = (int16)col;
        x += x_inc;
        y += y_inc;
    }

    for (int row = 0; row < end_y; row++)
        process_line_mid[row] = (int16)end_x;

    junction_type_from_camera = 3;
    turn_dbg_active = 1;
    turn_dbg_start_x = start_x;
    turn_dbg_start_y = start_y;
    turn_dbg_end_x = end_x;
    turn_dbg_end_y = end_y;
    turn_dbg_corner_x = corner_x;
    turn_dbg_corner_y = corner_y;
    turn_dbg_is_left = is_left_turn;
}

/* =============== 中心框边缘检测转弯 + T字/十字路口 =============== */
/*
 * 原理：
 *   扫描中心框四条边，根据白线出现的边组合分类：
 *     下+左         → 左直角弯   (type 3)
 *     下+右         → 右直角弯   (type 3)
 *     下+左+上       → 左T字路口  (type 1, 触发左转)
 *     下+右+上       → 右T字路口  (type 1, 触发右转)
 *     下+左+右       → 正T字路口  (type 1, 方向不确定不走线)
 *     下+左+右+上    → 十字路口   (type 2)
 *   type 1 和 type 3 均触发转弯，type 2 直行。
 *   只判断有无，不关心数量。
 */
static void detect_box_edge_turn(int left_edge[], int right_edge[])
{
    uint16 x0, y0, x1, y1;
    uint8 top_hit = 0, bottom_hit = 0, left_hit = 0, right_hit = 0;
    uint16 side_scan_start;
    int left_hit_y = -1;
    int right_hit_y = -1;

    get_turn_detect_box(&x0, &y0, &x1, &y1);
    side_scan_start = y0 + 1;
    if (side_scan_start <= MT9V03X_1_H / 2)
        side_scan_start = MT9V03X_1_H / 2 + 1;

    for (uint16 x = x0; x <= x1 && !top_hit; x++)
        if (process_image[y0][x] != 0) top_hit = 1;
    for (uint16 x = x0; x <= x1 && !bottom_hit; x++)
        if (process_image[y1][x] != 0) bottom_hit = 1;
    for (uint16 y = side_scan_start; y < y1 && !left_hit; y++)
    {
        if (process_image[y][x0] != 0)
        {
            left_hit = 1;
            left_hit_y = y;
        }
    }
    for (uint16 y = side_scan_start; y < y1 && !right_hit; y++)
    {
        if (process_image[y][x1] != 0)
        {
            right_hit = 1;
            right_hit_y = y;
        }
    }

    if (!bottom_hit)
    {
        junction_type_from_camera = 0;
        junction_side = 0;
        t_junction_seen = 0;
        turn_dbg_active = 0;
        return;
    }

    /* 分类：仅下+左=直角弯左，仅下+右=直角弯右，其余=查序列 */
    int is_left  = bottom_hit && left_hit  && !right_hit && !top_hit;
    int is_right = bottom_hit && right_hit && !left_hit  && !top_hit;

    if (is_left || is_right)
    {
        junction_type_from_camera = 3;
        junction_side = is_left ? 1 : 2;
        t_junction_seen = 0;
    }
    else if (bottom_hit && (top_hit || left_hit || right_hit))
    {
        /* T字/十字路口：方向由序列写死 */
        uint8 t_dir = Task2_GetNextTDir();
        junction_type_from_camera = 1;
        junction_side = 0;
        t_junction_seen = 1;
        is_left  = 0;
        is_right = 0;

        if (t_dir == 0)      { junction_side = 2; is_right = 1; }
        else if (t_dir == 1) { junction_side = 1; is_left  = 1; }
        /* t_dir == 2 直行，is_left/is_right 保持 0 */
    }
    else
    {
        junction_type_from_camera = 0;
        junction_side = 0;
        t_junction_seen = 0;
        turn_dbg_active = 0;
        return;
    }

    if (!is_left && !is_right)
    {
        turn_dbg_active = 0;
        return;
    }

    /* 拐角确认：命中行上下5行内，至少2行从边界向内连续50白点才认定出口 */
    {
        int hit_y = is_left ? left_hit_y : right_hit_y;
        if (hit_y < 0) hit_y = y1;
        int edge_x = is_left ? (int)x0 : (int)x1;
        int col_step = is_left ? 1 : -1;
        int chk_top = hit_y - 5;
        int chk_bot = hit_y + 5;
        if (chk_top < 0) chk_top = 0;
        if (chk_bot >= MT9V03X_1_H) chk_bot = MT9V03X_1_H - 1;

        int pass_rows = 0;
        for (int row = chk_top; row <= chk_bot; row++)
        {
            int run = 0;
            for (int col = edge_x; col >= 0 && col < MT9V03X_1_W; col += col_step)
            {
                if (process_image[row][col] != 0)
                    run++;
                else
                    break;
            }
            if (run >= 50) pass_rows++;
        }

        if (pass_rows < 2)
        {
            junction_type_from_camera = 0;
            junction_side = 0;
            t_junction_seen = 0;
            turn_dbg_active = 0;
            return;
        }
    }

    int start_x = (int)process_line_mid[MT9V03X_1_H - 1];
    int start_y = MT9V03X_1_H - 1;
    int corner_y = is_left ? left_hit_y : right_hit_y;
    int end_x = is_left ? SHARP_TURN_EDGE_MARGIN
                         : MT9V03X_1_W - 1 - SHARP_TURN_EDGE_MARGIN;
    int end_y;

    if (start_x < 0 || start_x >= MT9V03X_1_W) start_x = MT9V03X_1_W / 2;
    if (corner_y < 0) corner_y = y1;

    end_y = corner_y;
    if (end_y < 0) end_y = 0;
    if (end_y > start_y) end_y = start_y;

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
            process_line_mid[row] = (int16)col;
        x += x_inc;
        y += y_inc;
    }
    for (int row = 0; row < end_y; row++)
        process_line_mid[row] = (int16)end_x;

    turn_dbg_active  = 1;
    turn_dbg_start_x = start_x;
    turn_dbg_start_y = start_y;
    turn_dbg_end_x   = end_x;
    turn_dbg_end_y   = end_y;
    turn_dbg_corner_x = is_left ? (int)x0 : (int)x1;
    turn_dbg_corner_y = corner_y;
    turn_dbg_is_left = is_left;
}

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
                process_line_mid[i] = center_seed;
                lost_line_count++;
            }
            else
            {
                process_line_mid[i] = (int16)((left + right) / 2);
            }
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

        /* 步骤 4：检测转弯（task1用框检测直角弯，task2用中心框边缘+T字路口） */
        if (control_task_mode == 1)
            detect_box_sharp_turn();
        else
            detect_box_edge_turn(boundary_left, boundary_right);

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

    /* 绿色检测框 */
    {
        uint16 x0, y0, x1, y1;
        get_turn_detect_box(&x0, &y0, &x1, &y1);
        ips200_draw_line(x0, y0, x1, y0, RGB565_GREEN);
        ips200_draw_line(x1, y0, x1, y1, RGB565_GREEN);
        ips200_draw_line(x1, y1, x0, y1, RGB565_GREEN);
        ips200_draw_line(x0, y1, x0, y0, RGB565_GREEN);
    }

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

    /* 路口类型 */
    ips200_show_string(0, 130, "J:");
    if (junction_type_from_camera == 1)
        ips200_show_string(18, 130, "T");
    else if (junction_type_from_camera == 3)
        ips200_show_string(18, 130, turn_dbg_is_left ? "L90" : "R90");
    else
        ips200_show_string(18, 130, "---");

    ips200_show_string(0, 140, "T:");
    if (control_task_mode == 1)
    {
        ips200_show_int(18, 140, Task1_GetCount(), 2);
        ips200_show_string(36, 140, "/");
        ips200_show_int(42, 140, TASK1_TURN_TARGET, 2);
    }
    else
    {
        ips200_show_int(18, 140, Task2_GetCount(), 2);
        ips200_show_string(36, 140, "/");
        ips200_show_int(42, 140, TASK2_TURN_TARGET, 2);
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

        /* 紫色小框标记识别到的框边拐点 */
        draw_clamped_box(turn_dbg_corner_x, turn_dbg_corner_y, 3, RGB565_MAGENTA);

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
