/*
 * camera.c - MT9V03X 双摄像头图像处理与中线提取
 *
 * ============================================================
 * 模块功能概述
 * ============================================================
 *   1) 从摄像头 DMA 缓冲区获取一帧原始灰度图
 *   2) 用 Otsu 大津法自适应二值化，无需手动调节阈值
 *   3) 逐行从下往上扫描赛道左右边界，计算中线位置
 *   4) 路口检测：十字路口、T字路口、直角弯，采用四边缘丢失法
 *   5) 按预设路线序列决定路口转弯方向（左转/右转/直行）
 *   6) 加权平均计算赛道偏移量，供 control.c 差速转向使用
 *   7) 在 IPS200 屏幕上显示二值化图像、中线、边界及诊断信息
 *
 * ============================================================
 * 数据流概要
 * ============================================================
 *   摄像头 DMA 缓冲区 → 稳定帧拷贝(raw_snapshot)
 *     → Otsu 自适应阈值二值化 → 3×3 去噪(孤立白点过滤)
 *     → 逐行搜线(种子点扩散法) → 左右边界提取
 *     → 黑区补线(前后5行插值) → 路口检测(四边缘丢失分析)
 *     → 路线决策(查表或默认) → DDA 光栅化画中线
 *     → 加权平均计算偏移量(track_offset) → 同步到对外数组(line_mid)
 *     → 交换双缓冲区(处理/显示) → IPS200 屏幕显示
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

/* ============================================================
 * 检测区域与阈值宏定义
 * ============================================================
 * EXIT_ROI 系列定义了画面中用于检测路口出口的感兴趣区域(ROI)边界。
 * 画面被划分为：左半区(0 ~ W/6)、中央区(W/6 ~ W*5/6)、右半区(W*5/6 ~ W)。
 * 顶部 FRONT_TOP_ROWS 行用于检测"前方出口"是否存在赛道。
 * 底部 FRONT_TOP_ROWS 行用于计算"入口中线"位置。
 */

/* ---- 出口检测 ROI 边界 ---- */
#define EXIT_ROI_LEFT                (MT9V03X_1_W / 6)    /* 左侧区域右边界 */
#define EXIT_ROI_RIGHT               (MT9V03X_1_W * 5 / 6)/* 右侧区域左边界 */
#define EXIT_ROI_TOP                 (MT9V03X_1_H / 6)    /* ROI 顶部行 */
#define EXIT_ROI_BOTTOM              (MT9V03X_1_H * 5 / 6)/* ROI 底部行 */

/* ---- 出口判定阈值 ---- */
#define EXIT_SIDE_MIN_ROWS           6   /* 侧向出口最少需要 N 行边界触边 */
#define EXIT_EDGE_MARGIN             3   /* 边界触及图像边缘的容差(像素) */
#define FRONT_TOP_ROWS               6   /* 前方出口检测区域高度(行) */
#define FRONT_HALF_WIDTH             10  /* 前方出口搜索框的半宽(像素) */
#define FRONT_ROW_MIN_PIXELS         4   /* 一行中需要的最少白点数 */
#define FRONT_EXIT_MIN_ROWS          3   /* 前方出口最少需要 N 行满足条件 */

/* ---- 路口锁定/解锁 ---- */
#define JUNCTION_MISSING_UNLOCK_FRAMES 3  /* 路口消失后持续N帧才解锁 */

/* ---- 路线绘制 ---- */
#define ROUTE_AIM_ADVANCE_ROWS       20  /* 斜线瞄准点向上提前的行数 */

/* ============================================================
 * 编译开关
 * ============================================================ */

/* 屏幕显示开关：比赛时可设为 0 提高帧率 */
#define ENABLE_DISPLAY 1

/* 丢线时中线填充值：使用图像水平中心 */
#define LOST_LINE_REPLACE_VAL (MT9V03X_1_W / 2)

/* 赛道颜色模式：0=黑色底板白色赛道(默认)  1=白色底板黑色赛道 */
#define TRACK_LINE_IS_BLACK 0

/* 直角弯诊断叠加层开关：赛后分析时可清零 */
#define ENABLE_TURN_DEBUG 1

/* ============================================================
 * 二值化参数
 * ============================================================
 * THRESHOLD_CLAMP_LO/HI：Otsu 结果的有效范围限制，防止极端光照误判。
 * THRESHOLD_DARK_MIN/MAX：在 Otsu 结果上叠加的偏移量，
 *   暗光场景提高阈值压制噪点，亮光场景降低阈值保留细节。
 */

#define THRESHOLD_DARK_MIN         30   /* 最小偏移补偿(暗场) */
#define THRESHOLD_DARK_MAX         30   /* 最大偏移补偿(亮场) */
#define THRESHOLD_CLAMP_LO         30   /* Otsu 结果下限 */
#define THRESHOLD_CLAMP_HI        220   /* Otsu 结果上限 */

/* ============================================================
 * 对外全局变量
 * ============================================================ */

/* 各行的中线列坐标，供 control.c 前视计算偏移量 */
int16 line_mid[MT9V03X_1_H];

/* 赛道偏移量（像素），正值=赛道偏右，负值=赛道偏左 */
int16 track_offset = 0;

/* 路口/弯道类型编码：
 *   0 = 直道
 *   1 = T字路口
 *   2 = 十字路口
 *   3 = 急转弯/直角弯 */
uint8 junction_type_from_camera = 0;

/* 连续丢线行数统计（用于调试） */
static uint8 line_lost_count = 0;

/* ============================================================
 * 双缓冲机制
 * ============================================================
 * 处理缓冲区和显示缓冲区独立存在，每帧通过指针交换实现 O(1) 切换，
 * 从而避免 DMA 传输与屏幕显示同时读写同一块内存导致的画面撕裂。
 */

/* ---- 图像缓冲区 ---- */
/* raw_snapshot：DMA 稳定帧拷贝的原始灰度图 */
static uint8 raw_snapshot[MT9V03X_1_H][MT9V03X_1_W];
static uint8 binary_buf_0[MT9V03X_1_H][MT9V03X_1_W];   /* 二值化缓冲 A */
static uint8 binary_buf_1[MT9V03X_1_H][MT9V03X_1_W];   /* 二值化缓冲 B */

/* ---- 中线缓冲区 ---- */
static int16 line_mid_buf_0[MT9V03X_1_H];
static int16 line_mid_buf_1[MT9V03X_1_H];

/* ---- 缓冲区指针：当前帧在 process_* 中处理，完成后交换到 display_* 供显示 ---- */
static uint8 (*process_image)[MT9V03X_1_W] = binary_buf_0;
static int16 *process_line_mid = line_mid_buf_0;

static uint8 (*display_image)[MT9V03X_1_W] = binary_buf_1;
static int16 *display_line_mid = line_mid_buf_1;

/* 图像就绪标志：处理完一帧后置 1，显示任务据此决定是否刷新 */
static uint8 image_ready = 0;

/* ============================================================
 * 直角弯诊断叠加数据
 * ============================================================
 * 由 process 端（detect_cross_t_junction）写入，
 * 由 display 端（image_display_task）读取并画在屏幕上。
 */

static int  turn_dbg_active  = 0;   /* 是否有诊断线要画 */
static int  turn_dbg_start_x = 0;   /* 斜线起点 x 坐标(底部) */
static int  turn_dbg_start_y = 0;   /* 斜线起点 y 坐标(底部) */
static int  turn_dbg_end_x   = 0;   /* 斜线终点 x 坐标(弯角瞄准点) */
static int  turn_dbg_end_y   = 0;   /* 斜线终点 y 坐标(弯角瞄准点) */
static int  turn_dbg_is_left = 0;   /* 1=左转  0=右转 */

/* ============================================================
 * 路口检测状态（control.c 和显示任务共用）
 * ============================================================ */

/* 当前检测到的路口类型 */
static junction_kind_t   current_junction_kind   = JUNCTION_NONE;

/* 当前路口的路线决策（左转/直行/右转） */
static route_decision_t  current_route_decision  = DECISION_STRAIGHT;

/* 决策无效标志：路线与路口类型不匹配时为 1，触发故障停车 */
static uint8             junction_error          = 0;


/* =================================================================
 * 初始化函数
 * =================================================================
 * 设置摄像头曝光时间并初始化双摄像头模组。
 * 曝光时间越大图像越亮，但帧率越低，需根据实际赛道光照条件调整。
 */

void cam_init(void)
{
    /* 曝光时间 400（默认值），光线暗时可增大，光线强时可减小 */
    mt9v03x_set_confing_buffer_1[MT9V03X_DOUBLE_EXP_TIME][1] = 400;
    mt9v03x_double_init(mt9v03x_1);
}

/* =================================================================
 * 缓冲区交换
 * =================================================================
 * 在 O(1) 时间内交换处理缓冲区和显示缓冲区的指针，
 * 取代大块内存拷贝，是双缓冲机制的核心。
 */

static void image_swap_buffer(void)
{
    uint8 (*temp_image)[MT9V03X_1_W] = process_image;
    process_image = display_image;
    display_image = temp_image;

    int16 *temp_line_mid = process_line_mid;
    process_line_mid = display_line_mid;
    display_line_mid = temp_line_mid;
}

/* =================================================================
 * 稳定帧拷贝
 * =================================================================
 * 在 DMA 中断间隙将摄像头图像拷贝到 raw_snapshot。
 * 与摄像头 DMA 写入错开时间，保证处理时图像不会"撕裂"。
 */

static void camera_copy_stable_frame(void)
{
    memcpy(raw_snapshot[0], mt9v03x_image_1[0], MT9V03X_1_W * MT9V03X_1_H);
}


/* =================================================================
 * 路线绘制辅助函数
 * =================================================================
 * 以下三个函数为 detect_cross_t_junction 服务，负责：
 *   clamp_route_aim_row    — 计算斜线的瞄准行(提前 ROUTE_AIM_ADVANCE_ROWS)
 *   find_boundary_corner_row — 角点定位：找边界跳变最大的行
 *   draw_route_line        — 用 DDA 光栅化算法绘制转弯中线或直行中线
 */

/*
 * 将瞄准行向上提前指定像素数，并限制在有效范围内。
 * 提前探测让车在到达弯角之前就开始转向，补偿舵机响应延迟。
 */
static int clamp_route_aim_row(int aim_row, int start_y)
{
    aim_row -= ROUTE_AIM_ADVANCE_ROWS;  /* 向上提前 */
    if(aim_row < 0) aim_row = 0;        /* 不超过图像顶部 */
    if(aim_row > start_y) aim_row = start_y;  /* 不超过起点 */
    return aim_row;
}

/*
 * 角点定位：从图像底部向上扫描边界位置的变化率，
 * 找到边界跳变最大的行作为转弯瞄准点。
 * 如果跳变不够明显(< SHARP_TURN_CORNER_DERIV)，回退到 fallback_row。
 */
static int find_boundary_corner_row(route_decision_t decision,
                                    int left_edge[],
                                    int right_edge[],
                                    int fallback_row)
{
    int corner_row = fallback_row;
    int max_jump = 0;
    /* 根据转弯方向选择对应的边界数组 */
    int *edge = (decision == DECISION_LEFT) ? left_edge : right_edge;

    for(int i = MT9V03X_1_H - 2; i >= 1; i--)
    {
        if(edge[i] < 0 || edge[i + 1] < 0) continue;  /* 跳过无效行 */
        int jump = edge[i] - edge[i + 1];              /* 相邻行边界位置差 */
        if(jump < 0) jump = -jump;
        if(jump > max_jump)
        {
            max_jump = jump;
            corner_row = i;
        }
    }

    /* 跳变不明显则回退 */
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

/*
 * 核心函数：绘制路线中线。
 * 根据决策类型(直行/左转/右转)和路口类型，选择合适的终点坐标，
 * 然后用 DDA 数字微分分析算法画直线，并将斜线上方行统一指向终点。
 */
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
    int h = MT9V03X_1_H;               /* 图像高度 */
    int w = MT9V03X_1_W;               /* 图像宽度 */
    int margin = SHARP_TURN_EDGE_MARGIN; /* 边缘容差 */
    int start_x = (int)process_line_mid[h - 1];  /* 起点：底部中线 x 坐标 */
    int start_y = h - 1;               /* 起点：图像最底行 */
    int aim_row = EXIT_ROI_TOP;        /* 瞄准行(默认) */
    int end_y;                         /* 终点行 */
    int end_x = w / 2;                 /* 终点列(默认图像中心) */

    /* 边界值保护 */
    if(start_x < 0 || start_x >= w) start_x = w / 2;
    if(entry_center < 0 || entry_center >= w) entry_center = start_x;
    if(row_start < 0) row_start = 0;
    if(row_end > h) row_end = h;
    if(row_end < row_start) row_end = row_start;

    /* ---- 直行决策：画垂直向上的直线 ---- */
    if(decision == DECISION_STRAIGHT)
    {
        for(int i = row_start; i < row_end; i++)
        {
            process_line_mid[i] = (int16)entry_center;
        }
        turn_dbg_active = 0;  /* 不画诊断线 */
        return;
    }

    /* ---- 转弯决策：计算终点坐标 ---- */
    end_x = (decision == DECISION_LEFT) ? margin : (w - 1 - margin);

    if(kind == JUNCTION_LEFT_CORNER || kind == JUNCTION_RIGHT_CORNER)
    {
        /* 直角弯：用角点定位找精确瞄准点 */
        int fallback_row = (decision == DECISION_LEFT) ? left_exit_row : right_exit_row;
        aim_row = find_boundary_corner_row(decision, left_edge, right_edge, fallback_row);
    }
    else
    {
        /* T字/十字路口：直接使用出口检测的平均行 */
        aim_row = (decision == DECISION_LEFT) ? left_exit_row : right_exit_row;
        if(aim_row < 0) aim_row = EXIT_ROI_TOP;
    }
    end_y = clamp_route_aim_row(aim_row, start_y);

    /* ---- DDA 数字微分分析算法 ---- */
    /* 计算从底部起点到转弯终点的直线，逐像素写入 process_line_mid */
    int dx = end_x - start_x;
    int dy = end_y - start_y;

    /* 步长取 dx 和 dy 中较大的绝对值，保证线段连续无断点 */
    int steps = (abs(dy) > abs(dx)) ? abs(dy) : abs(dx);
    if(steps < 1) steps = 1;

    float x_inc = (float)dx / (float)steps;  /* 每步 x 增量 */
    float y_inc = (float)dy / (float)steps;  /* 每步 y 增量 */
    float x = (float)start_x;
    float y = (float)start_y;

    for(int s = 0; s <= steps; s++)
    {
        int row = (int)(y + 0.5f);  /* 四舍五入取整 */
        int col = (int)(x + 0.5f);
        if(row >= 0 && row < h && col >= 0 && col < w)
        {
            process_line_mid[row] = (int16)col;
        }
        x += x_inc;
        y += y_inc;
    }

    /* 斜线上方的行统一指向终点坐标，
       消除画面上半部因旧中线残留导致的转向方向被"拽偏" */
    for(int row = 0; row < end_y; row++)
    {
        process_line_mid[row] = (int16)end_x;
    }

    /* 保存诊断数据供 image_display_task 在屏幕上画叠加层 */
    turn_dbg_active  = 1;
    turn_dbg_start_x = start_x;
    turn_dbg_start_y = start_y;
    turn_dbg_end_x   = end_x;
    turn_dbg_end_y   = end_y;
    turn_dbg_is_left = (decision == DECISION_LEFT);
}

/* =================================================================
 * 路口分类与路线决策
 * =================================================================
 * 以下函数实现了从"检测出口"到"决定路线"的完整逻辑链：
 *
 *   detect_exits      — 扫描画面中前方/左侧/右侧三个方向的出口
 *   classify_exits    — 根据三个出口的组合状态分类路口类型
 *   decision_is_valid — 验证路线决策与路口类型是否匹配
 *   junction_needs_route_decision — 判断路口类型是否需要查表决策
 */

/*
 * 验证路线决策与路口类型是否兼容。
 *   十字路口：任何决策都有效(左/右/直行均可)
 *   侧T字路口(左+前)：只允许左转或直行
 *   侧T字路口(右+前)：只允许右转或直行
 *   标准T字路口(左+右)：只允许左转或右转(不能直行)
   直角弯：不打此函数校验(由 classify_exits 直接分派方向)
 */
static uint8 decision_is_valid(junction_kind_t kind, route_decision_t decision)
{
    if(kind == JUNCTION_CROSS) return 1;  /* 十字路口无限制 */
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

/*
 * 根据三个方向的出口存在与否分类路口类型。
 *
 *   前+左+右  → 十字路口 (JUNCTION_CROSS)
 *   前+左     → 侧T字路口(左出口) (JUNCTION_SIDE_LEFT_T)
 *   前+右     → 侧T字路口(右出口) (JUNCTION_SIDE_RIGHT_T)
 *   左+右     → 标准T字路口 (JUNCTION_STANDARD_T)
 *   仅左      → 左直角弯 (JUNCTION_LEFT_CORNER)
 *   仅右      → 右直角弯 (JUNCTION_RIGHT_CORNER)
 *   其他      → 无路口 (JUNCTION_NONE)
 */
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

/*
 * 判断当前路口类型是否需要从预设路线序列查表获取决策。
 * 十字路口和 T字路口(各种)需要查表，
 * 直角弯不需要(弯的方向即决定了决策方向)。
 */
static uint8 junction_needs_route_decision(junction_kind_t kind)
{
    return (kind == JUNCTION_CROSS ||
            kind == JUNCTION_SIDE_LEFT_T ||
            kind == JUNCTION_SIDE_RIGHT_T ||
            kind == JUNCTION_STANDARD_T) ? 1 : 0;
}

/*
 * 出口检测：在 ROI 区域内扫描三个方向(前方/左侧/右侧)的赛道出口。
 *
 *   前方出口：在画面顶部 FRONT_TOP_ROWS 行内，以入口中线为中心向左右各
 *             FRONT_HALF_WIDTH 像素的搜索框内，统计白点行数。
 *             白点充足 = 前方有赛道延伸。
 *
 *   左侧出口：在画面左半区(0 ~ W/6)内，左边界触及左边缘的连续行数。
 *
 *   右侧出口：在画面右半区(W*5/6 ~ W)内，右边界触及右边缘的连续行数。
 *
 * 输出参数：
 *   front_exists/left_exists/right_exists — 三个方向的出口是否存在
 *   left_exit_row/right_exit_row           — 左右出口的平均行号
 *   entry_center_out                       — 入口中线 x 坐标
 *   row_start_out/row_end_out              — ROI 行范围
 */
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

    /* 边界修正 */
    if(front_row_end > row_end) front_row_end = row_end;
    if(entry_row_start < row_start) entry_row_start = row_start;

    /* ---- 计算入口中线：底部区域 process_line_mid 的平均值 ---- */
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

    /* ---- 扫描左右出口：统计边界触边的行数 ---- */
    for(int row = row_start; row < row_end; row++)
    {
        /* 左边界触及或更靠左 = 左侧出口 */
        if(left_edge[row] >= 0 && left_edge[row] <= EXIT_ROI_LEFT + EXIT_EDGE_MARGIN)
        {
            left_hits++;
            left_row_sum += row;
        }
        /* 右边界触及或更靠右 = 右侧出口 */
        if(right_edge[row] >= 0 && right_edge[row] >= EXIT_ROI_RIGHT - EXIT_EDGE_MARGIN)
        {
            right_hits++;
            right_row_sum += row;
        }
    }

    /* ---- 扫描前方出口：画面顶部搜索框内的白点密度 ---- */
    for(int row = row_start; row < front_row_end; row++)
    {
        int pixel_hits = 0;
        int col_start = entry_center - FRONT_HALF_WIDTH;
        int col_end = entry_center + FRONT_HALF_WIDTH;
        if(col_start < 0) col_start = 0;
        if(col_end >= w) col_end = w - 1;

        for(int col = col_start; col <= col_end; col++)
        {
            if(process_image[row][col] != 0)  /* 白点计数 */
            {
                pixel_hits++;
            }
        }
        if(pixel_hits >= FRONT_ROW_MIN_PIXELS)  /* 该行满足白点密度要求 */
        {
            front_hits++;
        }
    }

    /* ---- 最终判定：有效行数达到阈值才认为出口存在 ---- */
    *front_exists = (front_hits >= FRONT_EXIT_MIN_ROWS) ? 1 : 0;
    *left_exists = (left_hits >= EXIT_SIDE_MIN_ROWS) ? 1 : 0;
    *right_exists = (right_hits >= EXIT_SIDE_MIN_ROWS) ? 1 : 0;

    /* 出口行号：取所有触边行的平均值 */
    *left_exit_row = (left_hits > 0) ? (left_row_sum / left_hits) : -1;
    *right_exit_row = (right_hits > 0) ? (right_row_sum / right_hits) : -1;

    *entry_center_out = entry_center;
    *row_start_out = row_start;
    *row_end_out = row_end;
}

/* =================================================================
 * 十字路口与T字路口检测（主入口）
 * =================================================================
 *
 * 原理（参考草莽项目的"四边缘丢失法"）：
 *   检测画面中前方、左侧、右侧三个方向的赛道出口是否存在。
 *   根据出口组合分类路口类型，然后做出路线决策：
 *     - 直角弯：按弯的方向转弯
 *     - 十字/T字路口：如果是任务2模式，从预设序列查表决定方向
 *                     如果是任务1模式，直行通过
 *
 * 路口锁定机制：
 *   检测到路口后锁定判断结果，路口消失后延迟解锁，
 *   避免路口期间检测抖动导致决策来回切换。
 *
 * 路线验证：
 *   如果路线决策与路口类型不兼容(如T字路口只有左右却给了直行)，
 *   设置 junction_error 标志，触发 control.c 故障停车。
 */

static void detect_cross_t_junction(int left_edge[], int right_edge[])
{
    /* 路口锁定状态：检测到路口后锁定，消失后延迟 N 帧才解锁 */
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

    /* ---- 步骤 1：检测三个方向的出口 ---- */
    detect_exits(left_edge, right_edge, &front_exists, &left_exists, &right_exists,
                 &left_exit_row, &right_exit_row, &entry_center, &row_start, &row_end);

    /* ---- 步骤 2：根据出口组合分类路口类型 ---- */
    detected_kind = classify_exits(front_exists, left_exists, right_exists);

    /* ---- 步骤 3：无路口时的解锁逻辑 ---- */
    if(detected_kind == JUNCTION_NONE)
    {
        current_junction_kind = JUNCTION_NONE;
        if(junction_locked)
        {
            junction_missing_frames++;
            /* 路口持续消失 N 帧后才解锁，防止短暂信号丢失误判 */
            if(junction_missing_frames >= JUNCTION_MISSING_UNLOCK_FRAMES)
            {
                junction_locked = 0;
                locked_kind = JUNCTION_NONE;
                locked_decision = DECISION_STRAIGHT;
            }
        }
        return;
    }

    /* ---- 步骤 4：首次检测到路口，锁定并做路线决策 ---- */
    junction_missing_frames = 0;
    if(!junction_locked)
    {
        locked_kind = detected_kind;

        /* 需要查表的路口类型：任务2模式下从预设序列取值 */
        if(junction_needs_route_decision(detected_kind)
           && Control_GetTaskMode() == TASK_MODE_2)
        {
            locked_decision = Task2_GetRouteDecision();
        }
        /* 直角弯：弯的方向即为决策方向 */
        else if(detected_kind == JUNCTION_LEFT_CORNER)
        {
            locked_decision = DECISION_LEFT;
        }
        else if(detected_kind == JUNCTION_RIGHT_CORNER)
        {
            locked_decision = DECISION_RIGHT;
        }
        else
        {
            locked_decision = DECISION_STRAIGHT;
        }
        junction_locked = 1;
    }

    /* ---- 步骤 5：输出当前路口状态 ---- */
    current_junction_kind = locked_kind;
    current_route_decision = locked_decision;

    /* 验证路线决策有效性 */
    junction_error = junction_needs_route_decision(locked_kind) ?
                     (decision_is_valid(locked_kind, locked_decision) ? 0 : 1) :
                     0;

    /* 决策无效 → 清空路口类型，触发故障停车 */
    if(junction_error)
    {
        junction_type_from_camera = 0;
        turn_dbg_active = 0;
        return;
    }

    /* ---- 步骤 6：设置 junction_type，供 control.c 使用 ---- */
    /* 直角弯 → 类型 3，触发转向状态机 */
    if(locked_kind == JUNCTION_LEFT_CORNER || locked_kind == JUNCTION_RIGHT_CORNER)
    {
        junction_type_from_camera = 3;
    }
    /* 直行决策 → 类型 1(T字) 或 2(十字)，control.c 不触发转向 */
    else if(locked_decision == DECISION_STRAIGHT)
    {
        junction_type_from_camera = (locked_kind == JUNCTION_CROSS) ? 2 : 1;
    }
    /* 转弯决策 → 类型 3，触发转向状态机 */
    else
    {
        junction_type_from_camera = 3;
    }

    /* ---- 步骤 7：根据决策绘制中线 ---- */
    draw_route_line(locked_decision, locked_kind, left_edge, right_edge,
                    left_exit_row, right_exit_row, entry_center, row_start, row_end);
}

/* =================================================================
 * Otsu 大津法自适应二值化
 * =================================================================
 *
 * 原理：
 *   遍历图像 0-255 所有灰度级，对每个灰度级计算"类间方差"，
 *   取方差最大的灰度级作为最优二值化阈值。
 *
 *   类间方差 = 前景像素数 × 背景像素数 × (前景均值 - 背景均值)²
 *   方差越大，说明按该值分割后前景与背景的差异越明显。
 *
 * 优点：
 *   自动适应环境光变化，不需要手动调整阈值。
 *
 * 后处理：
 *   - 对 Otsu 结果做限幅(30-220)，防止极端光照下的误判。
 *   - 叠加曝光自适应偏移：暗光场景提高阈值压噪，亮光降低保留细节。
 */

static uint8 compute_otsu_threshold(void)
{
    int histogram[256] = {0};           /* 灰度直方图 */
    int pixel_count = MT9V03X_1_H * MT9V03X_1_W;
    uint8 *img_ptr = &raw_snapshot[0][0];

    /* 步骤 1：统计灰度直方图（256 级灰度各出现了多少次） */
    for(int i = 0; i < pixel_count; i++)
    {
        histogram[img_ptr[i]]++;
    }

    /* 步骤 2：计算全局灰度总和（用于后续计算类间方差） */
    int sum = 0;
    for(int i = 0; i < 256; i++)
    {
        sum += i * histogram[i];
    }

    int sumB = 0;          /* 前景(暗部)灰度累加 */
    int wB = 0;            /* 前景像素数 */
    int wF = 0;            /* 背景像素数 */
    float varMax = 0.0f;   /* 最大类间方差 */
    uint8 threshold = 0;   /* 最终阈值 */

    /* 步骤 3：遍历 0-255 每个灰度级作为候选阈值 */
    for(int i = 0; i < 256; i++)
    {
        wB += histogram[i];             /* 前景像素数累加 */
        if (wB == 0) continue;          /* 没有前景像素则跳过 */

        wF = pixel_count - wB;          /* 背景像素数 = 总像素 - 前景 */
        if (wF == 0) break;             /* 没有背景像素则终止 */

        sumB += i * histogram[i];       /* 前景灰度累加 */
        int sumF = sum - sumB;          /* 背景灰度累加 */

        /*
         * 类间方差简化公式：
         *   Var = (sumB² / wB) + (sumF² / wF)
         * 等价于 wB × wF × (均值B - 均值F)²
         */
        float varBetween = (float)sumB * sumB / wB + (float)sumF * sumF / wF;

        if (varBetween > varMax)
        {
            varMax = varBetween;
            threshold = i;              /* 记录当前最佳阈值 */
        }
    }

    /* 步骤 4：限幅，防止极端光照导致阈值过高或过低 */
    if(threshold < THRESHOLD_CLAMP_LO) threshold = THRESHOLD_CLAMP_LO;
    if(threshold > THRESHOLD_CLAMP_HI) threshold = THRESHOLD_CLAMP_HI;

    /* 步骤 5：曝光自适应偏移 */
    {
        int range = THRESHOLD_CLAMP_HI - THRESHOLD_CLAMP_LO;
        int offset = THRESHOLD_DARK_MIN
                   + (THRESHOLD_DARK_MAX - THRESHOLD_DARK_MIN)
                   * (threshold - THRESHOLD_CLAMP_LO) / range;
        threshold += offset;
    }
    if(threshold > 245) threshold = 245;  /* 最终上限保护 */

    return threshold;
}

/* =================================================================
 * 主处理任务 — image_process_task
 * =================================================================
 *
 * 本函数是整个图像处理模块的核心，每帧调用一次。
 * 处理流程共 7 个步骤：
 *
 *   步骤 1：稳定帧拷贝      — 从 DMA 缓冲区拷贝一帧到 raw_snapshot
 *   步骤 2：Otsu 二值化     — 自适应阈值转为黑白图
 *         3×3 去噪          — 过滤孤立白点
 *   步骤 3：逐行搜线        — 种子点扩散法找每行中线
 *          空隙插值         — ≤20行的空隙用上下中线线性插值补齐
 *          黑区补线         — 大段丢线取前后5行中线直线连接
 *   步骤 4：路口检测        — 检测十字/T字/直角弯，决定路线
 *   步骤 5：加权平均        — 计算赛道偏移量 track_offset
 *   步骤 6：同步外发        — 将中线同步到 line_mid[] 供 control.c 使用
 *   步骤 7：缓冲区交换      — 处理/显示指针切换
 */

void image_process_task(void)
{
    /* 等待摄像头一帧采集完成 */
    if (mt9v03x_finish_flag_1 == 1)
    {
        mt9v03x_finish_flag_1 = 0;  /* 清除完成标志 */

        /* ============ 步骤 1：稳定帧拷贝 ============ */
        camera_copy_stable_frame();

        /* ============ 步骤 2：Otsu 自适应二值化 ============ */
        /*
         * 用大津法自动计算的阈值将原始灰度图转为二值图。
         * TRACK_LINE_IS_BLACK=0 时：灰度 > 阈值 → 白色(255)，否则黑色(0)
         * 即默认的"黑色底板白色赛道"模式。
         */
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

        /* ============ 3×3 去噪 ============ */
        /*
         * 扫描每个像素的 8 邻域，如果周围白点数 < 2 则视为孤立噪声抹掉。
         * 静态缓冲区放静态区 .bss 段，避免 30KB+ 的栈溢出。
         */
        {
            static uint8 clean[MT9V03X_1_H][MT9V03X_1_W];
            for (int y = 1; y < MT9V03X_1_H - 1; y++)
            {
                for (int x = 1; x < MT9V03X_1_W - 1; x++)
                {
                    if (!process_image[y][x]) { clean[y][x] = 0; continue; }
                    int nb = 0;  /* 邻域白点数 */
                    if (process_image[y-1][x-1]) nb++;
                    if (process_image[y-1][x  ]) nb++;
                    if (process_image[y-1][x+1]) nb++;
                    if (process_image[y  ][x-1]) nb++;
                    if (process_image[y  ][x+1]) nb++;
                    if (process_image[y+1][x-1]) nb++;
                    if (process_image[y+1][x  ]) nb++;
                    if (process_image[y+1][x+1]) nb++;
                    clean[y][x] = (nb >= 2) ? 255 : 0;  /* 邻域 ≥2 个白点才保留 */
                }
            }
            /* 边界行/列保持不变（不去噪，因为没有完整邻域） */
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

        /* ============ 步骤 3：逐行扫描中线 ============ */
        /*
         * 从图像底部(H-1)向顶部(0)逐行扫描。
         * 每行以"上一行的中线位置"作为种子点，向左右两侧扩散搜索最近的
         * 白色像素。找到后向两侧扩展得到完整左右边界。
         *
         * 在线空隙插值：
         *   如果遇到 ≤25 行的空隙（无白点或赛道太窄），
         *   且空隙上下方都有有效中线，则用线性插值连接两端中线。
         */
        int lost_line_count = 0;                  /* 本轮丢线行数 */
        int boundary_left[MT9V03X_1_H];           /* 每行左边界列坐标 */
        int boundary_right[MT9V03X_1_H];          /* 每行右边界列坐标 */
        int gap_start = -1;                       /* 当前空隙底部行号(-1=不在空隙中) */

        for (int i = MT9V03X_1_H - 1; i >= 0; i--)
        {
            /*
             * 种子点：行 H-1 用图像中心，其他行用上一行的中线位置。
             * 这样利用赛道的连续性，比每次从中心搜快且抗噪。
             */
            int center_seed = (i == MT9V03X_1_H - 1)
                              ? (MT9V03X_1_W / 2)
                              : process_line_mid[i + 1];
            if (center_seed < 0) center_seed = 0;
            if (center_seed >= MT9V03X_1_W) center_seed = MT9V03X_1_W - 1;

            int line_pos = -1;   /* 找到的白色像素位置 */
            int left, right;

            /* 种子点向两侧扩散搜索——每次扩散左右各1像素 */
            for (int span = 0; span < MT9V03X_1_W / 2; span++)
            {
                left  = center_seed - span;
                right = center_seed + span;

                /* 左边先找到 */
                if ((left >= 0) && (process_image[i][left] != 0))
                {
                    line_pos = left;
                    break;
                }
                /* 右边先找到 */
                if ((right < MT9V03X_1_W) && (process_image[i][right] != 0))
                {
                    line_pos = right;
                    break;
                }
            }

            /* 没找到白点 → 丢线，先用种子点填充 */
            if (line_pos < 0)
            {
                if (gap_start < 0) gap_start = i;  /* 记录空隙起始行 */
                boundary_left[i]  = -1;
                boundary_right[i] = -1;
                process_line_mid[i] = center_seed;
                lost_line_count++;
                continue;
            }

            /* 从找到的白点向左右扩展，获取完整边界 */
            left = line_pos;
            right = line_pos;
            while ((left > 0) && (process_image[i][left] != 0)) left--;
            while ((right < MT9V03X_1_W - 1) && (process_image[i][right] != 0)) right++;

            boundary_left[i]  = left;
            boundary_right[i] = right;

            /* 赛道宽度 < 3 像素 → 视为噪声，丢弃 */
            if (right - left < 3)
            {
                if (gap_start < 0) gap_start = i;
                process_line_mid[i] = center_seed;
                lost_line_count++;
                continue;
            }

            /* 有效中线 = 左右边界的中心点 */
            int mid = (int16)((left + right) / 2);

            /* ---- 空隙插值：≤25行的空隙用上下中线线性连接 ---- */
            if (gap_start >= 0)
            {
                int gap_size = gap_start - i;
                if (gap_size <= 25 && gap_start < MT9V03X_1_H - 1)
                {
                    int mid_below = process_line_mid[gap_start + 1];  /* 空隙下方有效中线 */
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

        /* ============ 黑区补线 ============ */
        /*
         * 对较大段的连续丢线区域（前文未处理的），
         * 取丢失段前后各5行有效中线的坐标，直线连接。
         * 这解决了赛道上有大面积黑块(如阴影、脏污)导致的丢线问题。
         */
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

                /* 前后至少需要5行有效数据 */
                if (gap_start < 5 || gap_end >= MT9V03X_1_H - 6) continue;

                int16 val_before = process_line_mid[gap_start - 5];
                int16 val_after  = process_line_mid[gap_end + 5];
                if (val_before < 0 || val_after < 0) continue;

                /* 线性插值连接 */
                for (int r = gap_start; r <= gap_end; r++)
                {
                    process_line_mid[r] = val_before
                        + (int16)((val_after - val_before)
                            * (r - (gap_start - 5))
                            / (gap_end + 5 - (gap_start - 5) + 1));
                }
            }
        }

        /* ============ 步骤 4：路口检测与路线决策 ============ */
        /*
         * 先清空 junction_type，然后调用 detect_cross_t_junction
         * 统一处理十字路口、T字路口和直角弯的检测。
         * 检测结果通过全局变量 junction_type_from_camera 和
         * current_junction_kind / current_route_decision 输出。
         */
        junction_type_from_camera = 0;
        detect_cross_t_junction(boundary_left, boundary_right);

        line_lost_count = (lost_line_count > 255) ? 255 : (uint8)lost_line_count;

        /* ============ 步骤 5：加权平均计算赛道偏移量 ============ */
        /*
         * 只取画面中间段(1/3 到 5/6 高度)的行参与计算。
         * 靠近底部的行权重更大（weight = 行号），因为近处的偏差
         * 对小车当前姿态的影响比远处大。
         *
         * track_offset > 0 → 赛道偏右 → 需要右转修正
         * track_offset < 0 → 赛道偏左 → 需要左转修正
         */
        {
            int32 sum = 0;
            int32 weight_sum = 0;
            int start = MT9V03X_1_H / 3;
            int end   = MT9V03X_1_H * 5 / 6;

            for (int i = start; i < end; i++)
            {
                int weight = i;  /* 行号越大(越靠下)权重越大 */
                sum += (process_line_mid[i] - (MT9V03X_1_W / 2)) * weight;
                weight_sum += weight;
            }
            track_offset = (weight_sum > 0) ? (int16)(sum / weight_sum) : 0;
        }

        /* ============ 步骤 6：同步中线到对外数组 ============ */
        for (int i = 0; i < MT9V03X_1_H; i++)
        {
            line_mid[i] = process_line_mid[i];
        }

        /* ============ 步骤 7：交换处理/显示缓冲区 ============ */
        image_swap_buffer();
        image_ready = 1;
    }
}

/* =================================================================
 * 显示任务 — image_display_task
 * =================================================================
 *
 * 在 IPS200 屏幕上显示以下内容：
 *   - 二值化后的摄像头图像(灰度图)
 *   - 中线(红色点)、左边界(青色点)、右边界(黄色点)
 *   - IMU 陀螺仪三轴数据(GX/GY/GZ，单位为度/秒)
 *   - 当前任务模式和已转过弯数(T: N/M)
 *   - 路口类型代码(CRS/SLT/SRT/STD/L90/R90)和路线决策(L/R/S)
 *   - 直角弯诊断叠加层(青色斜线+绿色起点+黄色终点)
 *
 * 注意：每3帧才刷新一次显示，为图像处理任务腾出 CPU 时间。
 */

void image_display_task(void)
{
#if ENABLE_DISPLAY
    static uint8 refresh_cnt = 0;
    if (!image_ready) return;

    /* 降低刷新率：每3帧(约66ms)更新一次屏幕 */
    if (++refresh_cnt < 3) return;
    refresh_cnt = 0;

    /* ---- 显示二值化图像 ---- */
    ips200_show_gray_image(0, 0, display_image[0],
                           MT9V03X_1_W, MT9V03X_1_H,
                           MT9V03X_1_W, MT9V03X_1_H, 0);

    /* ---- 叠加中线(红)和左右边界(青/黄) ---- */
    for (int i = 0; i < MT9V03X_1_H; i++)
    {
        /* 红色点 = 中线 */
        if ((display_line_mid[i] >= 0) && (display_line_mid[i] < MT9V03X_1_W))
        {
            ips200_draw_point((uint16)display_line_mid[i], (uint16)i, RGB565_RED);
        }

        /* 扫描此行找到左右边界 */
        int l = -1, r = -1;
        for (int j = 0; j < MT9V03X_1_W; j++)
        {
            if (display_image[i][j])
            {
                if (l < 0) l = j;  /* 第一个白点 = 左边界 */
                r = j;             /* 最后一个白点 = 右边界 */
            }
        }
        if (l >= 0) ips200_draw_point((uint16)l, (uint16)i, RGB565_CYAN);   /* 青色 */
        if (r >= 0 && r > l) ips200_draw_point((uint16)r, (uint16)i, RGB565_YELLOW); /* 黄色 */
    }

    /* ---- IMU 陀螺仪数据 (单位: 度/秒) ---- */
    ips200_set_font(IPS200_6X8_FONT);
    ips200_set_color(RGB565_RED, RGB565_WHITE);

    /* GZ: 偏航角速度, GY: 俯仰角速度, GX: 横滚角速度 */
    ips200_show_string(0, 122, "GZ");
    ips200_show_int(18, 122, (int32)(gyro[2] * 57.3f), 4);
    ips200_show_string(54, 122, "GY");
    ips200_show_int(72, 122, (int32)(gyro[1] * 57.3f), 4);
    ips200_show_string(108, 122, "GX");
    ips200_show_int(126, 122, (int32)(gyro[0] * 57.3f), 4);

    /* ---- 当前任务模式及已转过弯数 ---- */
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

    /* ---- 路口类型与决策显示 ---- */
    /*
     * 第一行(第140行)：
     *   位置 72-74: ERR=路线决策无效 / 空白=正常
     *   位置 96-98: 路口类型代码
     *     CRS = 十字路口
     *     SLT = 侧T字路口(左出口)
     *     SRT = 侧T字路口(右出口)
     *     STD = 标准T字路口
     *     L90 = 左直角弯
     *     R90 = 右直角弯
     *   位置 120-121: 路线决策 L=左转 R=右转 S=直行
     */
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

    /* ---- 直角弯诊断叠加层 ---- */
    /*
     * 当检测到转弯且 ENABLE_TURN_DEBUG=1 时，在图像上叠加以下内容：
     *   青色斜线 = 生成的转弯中线(DDA 光栅化结果)
     *   绿色点   = 起点(图像底部中线位置)
     *   黄色点   = 终点(弯角瞄准点)
     *   右上角   = LT/R 表示左/右转弯
     */
#if ENABLE_TURN_DEBUG
    if (turn_dbg_active)
    {
        /* 青色斜线：从起点到终点的转向路径 */
        ips200_draw_line((uint16)turn_dbg_start_x, (uint16)turn_dbg_start_y,
                         (uint16)turn_dbg_end_x,   (uint16)turn_dbg_end_y,
                         RGB565_CYAN);

        /* 绿色点：起点标记（图像底部的中线位置） */
        ips200_draw_point((uint16)turn_dbg_start_x, (uint16)turn_dbg_start_y, RGB565_GREEN);

        /* 黄色点：终点标记（弯角瞄准点） */
        ips200_draw_point((uint16)turn_dbg_end_x, (uint16)turn_dbg_end_y, RGB565_YELLOW);

        /* 右上角方向标识：LT=左转  RT=右转 */
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

#endif /* ENABLE_DISPLAY */
}

/* =================================================================
 * 对外接口函数
 * =================================================================
 * 以下函数供 control.c 和 display 调用，用于获取图像处理模块的内部状态。
 */

/* 获取赛道偏移量（像素），正值=赛道偏右，负值=赛道偏左 */
int16 Camera_GetTrackOffset(void)
{
    return track_offset;
}

/* 获取连续丢线行数 */
uint8 Camera_GetLineLostCount(void)
{
    return line_lost_count;
}

/* 获取指定行的中线列坐标，行号越界时返回图像中心 */
int16 Camera_GetCenterLine(uint8 row)
{
    if(row >= MT9V03X_1_H)
    {
        return MT9V03X_1_W / 2;
    }
    return line_mid[row];
}

/* 获取路口决策错误标志：1=决策与路口类型不匹配 */
uint8 Camera_GetJunctionError(void)
{
    return junction_error;
}

/* 获取当前路口类型 */
junction_kind_t Camera_GetJunctionKind(void)
{
    return current_junction_kind;
}

/* 获取当前路线的决策方向（左转/直行/右转） */
route_decision_t Camera_GetRouteDecision(void)
{
    return current_route_decision;
}
