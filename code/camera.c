/*
 * camera.c - MT9V03X 鍙屾憚鍍忓ご鍥惧儚澶勭悊涓庝腑绾挎彁鍙?
 *
 * 鏈ā鍧楄礋璐ｏ細
 *   1) 浠庢憚鍍忓ご DMA 缂撳啿鍖鸿幏鍙栦竴甯у師濮嬬伆搴﹀浘
 *   2) 鐢?Otsu 澶ф触娉曡嚜閫傚簲浜屽€煎寲
 *   3) 閫愯鎼滅储璧涢亾宸﹀彸杈圭晫锛岃绠椾腑绾?
 *   4) 鐩磋寮娴嬶紙杈圭晫涓㈠け娉?+ 瑙掔偣瀹氫綅锛?
 *   5) 鍔犳潈骞冲潎璁＄畻璧涢亾鍋忕Щ閲忥紙渚?control.c 浣跨敤锛?
 *   6) 鏄剧ず鎽勫儚澶村浘鍍?+ 涓嚎 + IMU 闄€铻轰华鏁版嵁
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

/* ======================== 缂栬瘧寮€鍏?======================== */

#define ENABLE_DISPLAY 1             /* 1=寮€鍚睆骞曟樉绀猴紝姣旇禌鏃跺彲鍏抽棴浠ユ彁楂樺抚鐜?*/
#define LOST_LINE_REPLACE_VAL (MT9V03X_1_W / 2)  /* 涓㈢嚎鏃跺～鍏呭浘鍍忎腑蹇?*/
#define TRACK_LINE_IS_BLACK 0        /* 1=鐧借壊搴曟澘榛戣壊璧涢亾  0=榛戣壊搴曟澘鐧借壊璧涢亾 */
#define ENABLE_TURN_DEBUG 1          /* 1=鏄剧ず鐩磋寮瘖鏂嚎鏉★紙璧涘悗娓呴浂 */

/* ====================== 浜屽€煎寲鍙傛暟 ====================== */

#define THRESHOLD_DARK_MIN         30 /* Otsu 缁撴灉鐨勬渶灏忓亸绉昏ˉ鍋?*/
#define THRESHOLD_DARK_MAX         30 /* 鏈€澶у亸绉昏ˉ鍋匡紙涓?MIN 鐩哥瓑鏃跺浐瀹氬亸绉?*/
#define THRESHOLD_CLAMP_LO         30 /* Otsu 缁撴灉涓嬮檺 */
#define THRESHOLD_CLAMP_HI        220 /* Otsu 缁撴灉涓婇檺 */

/* ====================== 瀵瑰鍏ㄥ眬鍙橀噺 ====================== */

int16 line_mid[MT9V03X_1_H];         /* 鍚勮鐨勪腑绾垮垪鍧愭爣锛屼緵 control.c 鍓嶈鐢?*/
int16 track_offset = 0;              /* 璧涢亾鍋忕Щ閲忥紙鍍忕礌锛夛紝渚?control.c 杞悜鐢?*/
uint8 junction_type_from_camera = 0; /* 0=鐩撮亾 1=T瀛楄矾鍙?2=鍗佸瓧 3=鐩磋寮?*/
uint8 junction_side = 0;            /* 0=no 1=left 2=right */
uint8 junction_visual_type = 0;     /* 0=--- 1=LT 2=RT 3=ST 4=L90 5=R90 */
uint8 junction_dbg_left_hit = 0;
uint8 junction_dbg_top_hit = 0;
uint8 junction_dbg_right_hit = 0;
static uint8 line_lost_count = 0;    /* 杩炵画涓㈢嚎琛屾暟缁熻 */

/* =================== 鍙岀紦鍐茬紦鍐插尯瀹氫箟 =================== */

/* raw_snapshot锛氫粠 DMA 缂撳啿鍖虹ǔ瀹氭嫹璐濈殑涓€甯у師濮嬬伆搴﹀浘 */
static uint8 raw_snapshot[MT9V03X_1_H][MT9V03X_1_W];
static uint8 binary_buf_0[MT9V03X_1_H][MT9V03X_1_W];
static uint8 binary_buf_1[MT9V03X_1_H][MT9V03X_1_W];

static int16 line_mid_buf_0[MT9V03X_1_H];
static int16 line_mid_buf_1[MT9V03X_1_H];

/* 澶勭悊缂撳啿鍖哄拰鏄剧ず缂撳啿鍖烘寚閽堬紝姣忓抚浜ゆ崲涓€娆★紝閬垮厤 DMA 鍜屾樉绀哄啿绐?*/
static uint8 (*process_image)[MT9V03X_1_W] = binary_buf_0;
static int16 *process_line_mid = line_mid_buf_0;

static uint8 (*display_image)[MT9V03X_1_W] = binary_buf_1;
static int16 *display_line_mid = line_mid_buf_1;

static uint8 image_ready = 0;

/* 鐩磋寮皟璇曞彔鍔犳暟鎹紙process 鍐欏叆锛宒isplay 璇诲彇 */
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

/* ==================== 鍒濆鍖栧嚱鏁?==================== */

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
    /* 璁剧疆鏇濆厜鏃堕棿涓?400锛堥粯璁ゅ€硷紝鍙牴鎹幆澧冭皟鏁?*/
    mt9v03x_set_confing_buffer_1[MT9V03X_DOUBLE_EXP_TIME][1] = 400;
    mt9v03x_double_init(mt9v03x_1);
}

/* ================== 缂撳啿鍖轰氦鎹?================== */

static void image_swap_buffer(void)
{
    /* O(1) 鏃堕棿浜ゆ崲 process/display 鎸囬拡锛岄伩鍏嶅ぇ鍧楀唴瀛樻嫹璐?*/
    uint8 (*temp_image)[MT9V03X_1_W] = process_image;
    process_image = display_image;
    display_image = temp_image;

    int16 *temp_line_mid = process_line_mid;
    process_line_mid = display_line_mid;
    display_line_mid = temp_line_mid;
}

/* ================== 绋冲畾甯ф嫹璐?================== */

static void camera_copy_stable_frame(void)
{
    /* 鍦?DMA 涓柇闂撮殭灏嗘憚鍍忓ご鍥惧儚鎷疯礉鍒?raw_snapshot锛屼繚璇佸鐞嗘椂涓嶄細鎾曡 */
    memcpy(raw_snapshot[0], mt9v03x_image_1[0], MT9V03X_1_W * MT9V03X_1_H);
}

/* =============== Otsu 澶ф触娉曡嚜閫傚簲浜屽€煎寲 =============== */

/* =============== 妗嗘娴嬬洿瑙掑集锛坱ask1 浣跨敤锛?=============== */
static void detect_box_sharp_turn(void)
{
    uint16 x0, y0, x1, y1;
    uint8 left_hit = 0, right_hit = 0;
    uint16 side_scan_start;
    int left_hit_y = -1;
    int right_hit_y = -1;

    get_turn_detect_box(&x0, &y0, &x1, &y1);
    side_scan_start = y0 + 1;
    if (side_scan_start <= MT9V03X_1_H / 2)
        side_scan_start = MT9V03X_1_H / 2 + 1;
    for (uint16 y = side_scan_start; y < y1 && !left_hit; y++)
    {
        if (process_image[y][x0] != 0)
        {
            int chk_top = (int)y - 5;  if (chk_top < 0) chk_top = 0;
            int chk_bot = (int)y + 5;  if (chk_bot >= MT9V03X_1_H) chk_bot = MT9V03X_1_H - 1;
            int pass_rows = 0;
            for (int row = chk_top; row <= chk_bot; row++)
            {
                int run = 0;
                for (int col = (int)x0; col < MT9V03X_1_W; col++)
                {
                    if (process_image[row][col] != 0) run++; else break;
                }
                if (run >= 50) pass_rows++;
            }
            if (pass_rows >= 2) { left_hit = 1; left_hit_y = y; }
        }
    }
    for (uint16 y = side_scan_start; y < y1 && !right_hit; y++)
    {
        if (process_image[y][x1] != 0)
        {
            int chk_top = (int)y - 5;  if (chk_top < 0) chk_top = 0;
            int chk_bot = (int)y + 5;  if (chk_bot >= MT9V03X_1_H) chk_bot = MT9V03X_1_H - 1;
            int pass_rows = 0;
            for (int row = chk_top; row <= chk_bot; row++)
            {
                int run = 0;
                for (int col = (int)x1; col >= 0; col--)
                {
                    if (process_image[row][col] != 0) run++; else break;
                }
                if (run >= 50) pass_rows++;
            }
            if (pass_rows >= 2) { right_hit = 1; right_hit_y = y; }
        }
    }

    int is_left_turn = left_hit;
    int is_right_turn = right_hit;

    if (!is_left_turn && !is_right_turn)
    {
        junction_type_from_camera = 0;
        turn_dbg_active = 0;
        return;
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

/* =============== 中心框边缘检测转弯 + T字/正T路口（task2 使用） =============== */
static void detect_box_edge_turn(void)
{
    uint16 x0, y0, x1, y1;
    uint8 top_hit = 0, left_hit = 0, right_hit = 0;
    uint16 side_scan_start;
    int left_hit_y = -1;
    int right_hit_y = -1;

    get_turn_detect_box(&x0, &y0, &x1, &y1);
    side_scan_start = y0 + 1;

    /* 顶边扫描：行 y0~y0+14，中心向外，向下确认 ≥30 连续白像素，≥2 列通过 */
    for (uint16 y = y0; y < y0 + 15 && y < MT9V03X_1_H && !top_hit; y++)
    {
        uint16 mid = (x0 + x1) / 2;
        for (int span = 0; span <= (int)(x1 - mid) && !top_hit; span++)
        {
            int check_cols[2] = { (int)mid - span, (int)mid + span };
            for (int ci = 0; ci < 2 && !top_hit; ci++)
            {
                int x = check_cols[ci];
                if (span == 0 && ci == 1) break;
                if (x < (int)x0 || x > (int)x1) continue;
                if (process_image[y][x] == 0) continue;

                int chk_left = (int)x - 5;  if (chk_left < 0) chk_left = 0;
                int chk_right = (int)x + 5;  if (chk_right >= MT9V03X_1_W) chk_right = MT9V03X_1_W - 1;
                int pass_cols = 0;
                for (int col = chk_left; col <= chk_right; col++)
                {
                    int run = 0;
                    for (int r = (int)y; r < MT9V03X_1_H; r++)
                    {
                        if (process_image[r][col] != 0) run++; else break;
                    }
                    if (run >= 15) pass_cols++;
                }
                if (pass_cols >= 2) top_hit = 1;
            }
        }
    }

    /* 左边扫描：从 y0+1 到 y1，向下确认 10 行，≥50 连续白像素，≥2 行通过 */
    for (uint16 y = side_scan_start; y < y1 && !left_hit; y++)
    {
        if (process_image[y][x0] != 0)
        {
            int chk_top = (int)y;  if (chk_top < 0) chk_top = 0;
            int chk_bot = (int)y + 10;  if (chk_bot >= MT9V03X_1_H) chk_bot = MT9V03X_1_H - 1;
            int pass_rows = 0;
            for (int row = chk_top; row <= chk_bot; row++)
            {
                int run = 0;
                for (int col = (int)x0; col < MT9V03X_1_W; col++)
                {
                    if (process_image[row][col] != 0) run++; else break;
                }
                if (run >= 30) pass_rows++;
            }
            if (pass_rows >= 2) { left_hit = 1; left_hit_y = y; }
        }
    }

    /* 右边扫描：从 y0+1 到 y1，向下确认 10 行，≥50 连续白像素，≥2 行通过 */
    for (uint16 y = side_scan_start; y < y1 && !right_hit; y++)
    {
        if (process_image[y][x1] != 0)
        {
            int chk_top = (int)y;  if (chk_top < 0) chk_top = 0;
            int chk_bot = (int)y + 10;  if (chk_bot >= MT9V03X_1_H) chk_bot = MT9V03X_1_H - 1;
            int pass_rows = 0;
            for (int row = chk_top; row <= chk_bot; row++)
            {
                int run = 0;
                for (int col = (int)x1; col >= 0; col--)
                {
                    if (process_image[row][col] != 0) run++; else break;
                }
                if (run >= 30) pass_rows++;
            }
            if (pass_rows >= 2) { right_hit = 1; right_hit_y = y; }
        }
    }

    /* 分类：左T、右T、标准T、直角弯 */
    int is_t_left  = left_hit && top_hit && !right_hit;
    int is_t_right = right_hit && top_hit && !left_hit;
    int is_t_std   = left_hit && right_hit;
    int is_t_junc  = is_t_left || is_t_right || is_t_std;
    int is_left_turn  = left_hit  && !right_hit && !top_hit;
    int is_right_turn = right_hit && !left_hit  && !top_hit;

    junction_dbg_left_hit = left_hit;
    junction_dbg_top_hit = top_hit;
    junction_dbg_right_hit = right_hit;

    if (is_t_junc)
    {
        uint8 t_dir = Task2_GetNextTDir();
        junction_type_from_camera = 1;
        junction_side = 0;

        if (is_t_left)       junction_visual_type = 1;
        else if (is_t_right) junction_visual_type = 2;
        else                 junction_visual_type = 3;

        if (t_dir == 0)      { junction_side = 2; is_right_turn = 1; }
        else if (t_dir == 1) { junction_side = 1; is_left_turn  = 1; }
        /* t_dir == 2: junction_side stays 0, neither is_left_turn nor is_right_turn */
    }
    else if (is_left_turn || is_right_turn)
    {
        junction_type_from_camera = 3;
        junction_side = is_left_turn ? 1 : 2;
        junction_visual_type = is_left_turn ? 4 : 5;
    }
    else
    {
        junction_type_from_camera = 0;
        junction_side = 0;
        junction_visual_type = 0;
        turn_dbg_active = 0;
        return;
    }

    if (!is_left_turn && !is_right_turn)
    {
        turn_dbg_active = 0;
        return;
    }

    int start_x = (int)process_line_mid[MT9V03X_1_H - 1];
    int start_y = MT9V03X_1_H - 1;
    int corner_y = is_left_turn ? left_hit_y : right_hit_y;
    int end_x = is_left_turn ? SHARP_TURN_EDGE_MARGIN
                             : MT9V03X_1_W - 1 - SHARP_TURN_EDGE_MARGIN;
    int end_y;

    if (start_x < 0 || start_x >= MT9V03X_1_W) start_x = MT9V03X_1_W / 2;
    if (corner_y < 0)
    {
        junction_type_from_camera = 0;
        junction_side = 0;
        junction_visual_type = 0;
        turn_dbg_active = 0;
        return;
    }

    /* 拐点在图像上半部分时不做补线，保留正常中线 */
    if (corner_y >= MT9V03X_1_H / 2)
    {
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
        turn_dbg_corner_x = is_left_turn ? (int)x0 : (int)x1;
        turn_dbg_corner_y = corner_y;
        turn_dbg_is_left = is_left_turn;
    }
    else
    {
        turn_dbg_active = 0;
    }
}

static uint8 compute_otsu_threshold(void)
{
    /*
     * 澶ф触娉曡嚜鍔ㄩ€夊彇鏈€浼樹簩鍊煎寲闃堝€硷細
     *   閬嶅巻 0-255 鎵€鏈夌伆搴︾骇锛岃绠楃被闂存柟宸紙between-class variance锛夛紝
     *   鍙栨柟宸渶澶х殑鐏板害绾т綔涓洪槇鍊笺€?
     *   浼樼偣鏄嚜閫傚簲鍏夌収鍙樺寲锛屾瘮鍥哄畾闃堝€兼洿椴佹銆?
     */
    int histogram[256] = {0};
    uint16 x0, y0, x1, y1;
    int pixel_count;

    get_turn_detect_box(&x0, &y0, &x1, &y1);

    for (int y = (int)y0; y <= (int)y1; y++)
    {
        uint8 *row = raw_snapshot[y];
        for (int x = (int)x0; x <= (int)x1; x++)
        {
            histogram[row[x]]++;
        }
    }
    pixel_count = ((int)x1 - (int)x0 + 1) * ((int)y1 - (int)y0 + 1);

    /* 璁＄畻鍏ㄥ眬鐏板害鎬诲拰锛岀敤浜庡悗缁绠楃被闂存柟宸?*/
    int sum = 0;
    for(int i = 0; i < 256; i++) {
        sum += i * histogram[i];
    }

    int sumB = 0, wB = 0, wF = 0;
    float varMax = 0.0;
    uint8 threshold = 0;

    /* 閬嶅巻鎵€鏈夌伆搴︾骇锛屾壘浣跨被闂存柟宸渶澶х殑闃堝€?*/
    for(int i = 0; i < 256; i++) {
        wB += histogram[i];            /* 鍓嶆櫙鍍忕礌鏁?*/
        if (wB == 0) continue;

        wF = pixel_count - wB;         /* 鑳屾櫙鍍忕礌鏁?*/
        if (wF == 0) break;

        sumB += i * histogram[i];      /* 鍓嶆櫙鐏板害绱姞 */
        int sumF = sum - sumB;         /* 鑳屾櫙鐏板害绱姞 */

        /* 绫婚棿鏂瑰樊鍏紡锛歏ar = wB * wF * (uB - uF)^2 */
        float varBetween = (float)sumB * sumB / wB + (float)sumF * sumF / wF;

        if (varBetween > varMax) {
            varMax = varBetween;
            threshold = i;
        }
    }

    /* 瀵瑰ぇ娲ユ硶缁撴灉鍋氶檺骞咃紝閬垮厤鏋佺鍊?*/
    if(threshold < THRESHOLD_CLAMP_LO) threshold = THRESHOLD_CLAMP_LO;
    if(threshold > THRESHOLD_CLAMP_HI) threshold = THRESHOLD_CLAMP_HI;

    /* 澧炲姞鏇濆厜鑷€傚簲鍋忕Щ锛堟殫鍦鸿皟楂橀槇鍊煎帇鍣紝浜満璋冧綆淇濈暀缁嗚妭 */
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

/* =============== 涓诲鐞嗕换鍔?=============== */

void image_process_task(void)
{
    /* 妫€鏌ユ憚鍍忓ご涓€甯ф槸鍚﹂噰闆嗗畬姣?*/
    if (mt9v03x_finish_flag_1 == 1)
    {
        mt9v03x_finish_flag_1 = 0;

        /* 姝ラ 1锛氱ǔ瀹氭嫹璐濅竴甯у埌 raw_snapshot */
        camera_copy_stable_frame();

        /* 姝ラ 2锛歄tsu 鑷€傚簲浜屽€煎寲 */
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

        // 3x3 鍘诲櫔锛氬绔嬬櫧鐐癸紙閭诲煙鐧界偣鏁?< 2锛夎涓哄櫔澹版姽鎺?
        {
            static uint8 clean[MT9V03X_1_H][MT9V03X_1_W];  /* 鏀鹃潤鎬佸尯锛岄伩鍏嶆爤婧㈠嚭 */
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
            // 杈圭晫琛?鍒椾繚鎸佷笉鍙?
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

        int lost_line_count = 0;  /* 鏈疆杩炵画涓㈢嚎鐨勮鏁扮粺璁?*/
        int boundary_left[MT9V03X_1_H];   /* 姣忚璧涢亾宸﹁竟鐣岋紝渚涚洿瑙掑集妫€娴嬪鐢?*/
        int boundary_right[MT9V03X_1_H];  /* 姣忚璧涢亾鍙宠竟鐣岋紝渚涚洿瑙掑集妫€娴嬪鐢?*/

        /* 姝ラ 3锛氶€愯鎵弿涓嚎锛堜粠杩戝寰€杩滃鎵?*/
        for (int i = MT9V03X_1_H - 1; i >= 0; i--)
        {
            /* 浠庝笂涓€琛屼腑绾夸綅缃嚭鍙戞悳绱紝鎻愰珮閫熷害骞舵姂鍒跺櫔澹?*/
            int center_seed = (i == MT9V03X_1_H - 1) ? (MT9V03X_1_W / 2) : process_line_mid[i + 1];
            if (center_seed < 0) center_seed = 0;
            if (center_seed >= MT9V03X_1_W) center_seed = MT9V03X_1_W - 1;

            int line_pos = -1;
            int left;
            int right;

            /* 浠庣瀛愮偣鍚戜袱渚ф墿鏁ｆ悳绱㈢櫧鑹插儚绱?*/
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

            /* 璇ヨ瀹屽叏鎵句笉鍒拌禌閬?鈥?涓㈢嚎锛岀敤绉嶅瓙鐐瑰～鍏?*/
            if (line_pos < 0)
            {
                boundary_left[i]  = -1;
                boundary_right[i] = -1;
                process_line_mid[i] = center_seed;
                lost_line_count++;
                continue;
            }

            /* 鎵惧埌璧涢亾鍚庯紝鍚戝乏鍙虫墿灞曟壘鍒板畬鏁磋竟鐣?*/
            left = line_pos;
            right = line_pos;
            while ((left > 0) && (process_image[i][left] != 0)) left--;
            while ((right < MT9V03X_1_W - 1) && (process_image[i][right] != 0)) right++;

            boundary_left[i]  = left;
            boundary_right[i] = right;

            /* 濡傛灉璧涢亾瀹藉害灏忎簬 3 鍍忕礌锛岃涓哄櫔澹帮紝鐢ㄧ瀛愮偣浠ｆ浛 */
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

        // 榛戝尯琛ョ嚎锛氬彇涓㈠け娈靛墠鍚庡悇5琛岀殑鏈夋晥涓嚎锛岀洿绾胯繛鎺?
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

        /* 姝ラ 4锛氭娴嬭浆寮紙task1鐢ㄦ妫€娴嬬洿瑙掑集锛宼ask2鐢ㄤ腑蹇冩杈圭紭+T瀛楄矾鍙ｏ級 */
        if (control_task_mode == 1)
            detect_box_sharp_turn();
        else
            detect_box_edge_turn();

        line_lost_count = (lost_line_count > 255) ? 255 : (uint8)lost_line_count;

        /* 姝ラ 5锛氬姞鏉冨钩鍧囪绠楄禌閬撳亸绉婚噺锛堣繎澶勮鏉冮噸澶э級
         *   鍙彇鐢婚潰涓棿娈碉紙1/3 鍒?5/6锛夛紝蹇界暐椤堕儴澶繙鐨勮鍜屽簳閮ㄥお杩戠殑琛?*/
        {
            int32 sum = 0;
            int32 weight_sum = 0;
            int start = MT9V03X_1_H / 3;
            int end   = MT9V03X_1_H * 5 / 6;

            for (int i = start; i < end; i++)
            {
                int weight = i;  /* 瓒婇潬杩戝簳閮ㄧ殑琛屾潈閲嶈秺澶?*/
                sum += (process_line_mid[i] - (MT9V03X_1_W / 2)) * weight;
                weight_sum += weight;
            }
            track_offset = (weight_sum > 0) ? (int16)(sum / weight_sum) : 0;
        }

        /* 姝ラ 6锛氬皢缁撴灉鍚屾鍒板澶栨暟缁勭粰 motor.c 浣跨敤 */
        for (int i = 0; i < MT9V03X_1_H; i++)
        {
            line_mid[i] = process_line_mid[i];
        }

        /* 姝ラ 7锛氫氦鎹㈠鐞?鏄剧ず缂撳啿鍖?*/
        image_swap_buffer();
        image_ready = 1;
    }
}

/* =============== 鏄剧ず浠诲姟 =============== */

void image_display_task(void)
{
#if ENABLE_DISPLAY
    static uint8 refresh_cnt = 0;
    if (!image_ready) return;

    /* 闄嶄綆鏄剧ず鍒锋柊鐜囷紙姣?3 甯ф樉绀轰竴娆★級锛屼负鍥惧儚澶勭悊鑵惧嚭 CPU */
    if (++refresh_cnt < 3) return;
    refresh_cnt = 0;

    /* 鏄剧ず浜屽€煎寲鍚庣殑鎽勫儚澶村浘鍍?*/
    ips200_show_gray_image(0, 0, display_image[0], MT9V03X_1_W, MT9V03X_1_H, MT9V03X_1_W, MT9V03X_1_H, 0);

    /* 缁胯壊妫€娴嬫 */
    {
        uint16 x0, y0, x1, y1;
        get_turn_detect_box(&x0, &y0, &x1, &y1);
        ips200_draw_line(x0, y0, x1, y0, RGB565_GREEN);
        ips200_draw_line(x1, y0, x1, y1, RGB565_GREEN);
        ips200_draw_line(x1, y1, x0, y1, RGB565_GREEN);
        ips200_draw_line(x0, y1, x0, y0, RGB565_GREEN);

        for (uint16 x = x0; x <= x1; )
        {
            if (display_image[y0][x] != 0)
            {
                uint16 bx = x;
                while (x <= x1 && display_image[y0][x] != 0) x++;
                draw_clamped_box((int)bx, (int)y0, 2, RGB565_BLUE);
            }
            else x++;
        }
        for (uint16 x = x0; x <= x1; )
        {
            if (display_image[y1][x] != 0)
            {
                uint16 bx = x;
                while (x <= x1 && display_image[y1][x] != 0) x++;
                draw_clamped_box((int)bx, (int)y1, 2, RGB565_BLUE);
            }
            else x++;
        }
        for (uint16 y = y0 + 1; y < y1; )
        {
            if (display_image[y][x0] != 0)
            {
                uint16 by = y;
                while (y < y1 && display_image[y][x0] != 0) y++;
                draw_clamped_box((int)x0, (int)by, 2, RGB565_BLUE);
            }
            else y++;
        }
        for (uint16 y = y0 + 1; y < y1; )
        {
            if (display_image[y][x1] != 0)
            {
                uint16 by = y;
                while (y < y1 && display_image[y][x1] != 0) y++;
                draw_clamped_box((int)x1, (int)by, 2, RGB565_BLUE);
            }
            else y++;
        }
    }

    /* 鐢ㄧ孩鐐圭敾涓嚎銆佽摑鐐圭敾宸﹁竟鐣屻€佺豢鐐圭敾鍙宠竟鐣?*/
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

    /* IMU 闄€铻轰华鏁版嵁鏄剧ず鍦ㄦ憚鍍忓ご鍥惧儚涓嬫柟 */
    ips200_set_font(IPS200_6X8_FONT);
    ips200_set_color(RGB565_RED, RGB565_WHITE);
    ips200_show_string(0, 122, "GZ");
    ips200_show_int(18, 122, (int32)(gyro[2] * 57.3f), 4);
    ips200_show_string(54, 122, "GY");
    ips200_show_int(72, 122, (int32)(gyro[1] * 57.3f), 4);
    ips200_show_string(108, 122, "GX");
    ips200_show_int(126, 122, (int32)(gyro[0] * 57.3f), 4);

    /* 璺彛绫诲瀷 */
    ips200_show_string(0, 130, "J:");
    switch (junction_visual_type)
    {
        case 1: ips200_show_string(18, 130, "LT"); break;
        case 2: ips200_show_string(18, 130, "RT"); break;
        case 3: ips200_show_string(18, 130, "ST"); break;
        case 4: ips200_show_string(18, 130, "L90"); break;
        case 5: ips200_show_string(18, 130, "R90"); break;
        default: ips200_show_string(18, 130, "---"); break;
    }

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
        ips200_show_string(60, 140, "SI:");
        ips200_show_int(84, 140, Task2_GetSeqIndex(), 1);
        ips200_show_string(96, 140, "SD:");
        if (junction_side == 1)
            ips200_show_string(120, 140, "L");
        else if (junction_side == 2)
            ips200_show_string(120, 140, "R");
        else
            ips200_show_string(120, 140, "S");

        ips200_show_string(0, 148, "L");
        ips200_show_int(12, 148, junction_dbg_left_hit, 1);
        ips200_show_string(30, 148, "T");
        ips200_show_int(42, 148, junction_dbg_top_hit, 1);
        ips200_show_string(60, 148, "R");
        ips200_show_int(72, 148, junction_dbg_right_hit, 1);
    }

#if ENABLE_TURN_DEBUG
    /* 鐩磋寮瘖鏂彔鍔犲眰 */
    if (turn_dbg_active)
    {
        /* 闈掕壊鏂滅嚎琛ㄧず鐢熸垚鐨勮浆寮腑绾?*/
        ips200_draw_line((uint16)turn_dbg_start_x, (uint16)turn_dbg_start_y,
                         (uint16)turn_dbg_end_x,   (uint16)turn_dbg_end_y,
                         RGB565_CYAN);

        /* 缁胯壊鐐规爣璁拌捣鐐癸紙搴曢儴涓績锛?*/
        ips200_draw_point((uint16)turn_dbg_start_x, (uint16)turn_dbg_start_y, RGB565_GREEN);

        /* 榛勮壊鐐规爣璁扮粓鐐癸紙寮鐬勫噯鐐癸級 */
        ips200_draw_point((uint16)turn_dbg_end_x, (uint16)turn_dbg_end_y, RGB565_YELLOW);

        /* 绱壊灏忔鏍囪璇嗗埆鍒扮殑妗嗚竟鎷愮偣 */
        draw_clamped_box(turn_dbg_corner_x, turn_dbg_corner_y, 3, RGB565_MAGENTA);

        /* 鍙充笂瑙掓樉绀鸿浆寮柟鍚?*/
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

/* ====================== 瀵瑰鎺ュ彛 ====================== */

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
