//  简介:八邻域图像处理

//------------------------------------------------------------------------------------------------------------------
#include "image.h"
#include "common.h"
#include "global.h"
#include "motor.h"
#include "pid.h"
#include "zf_device_tft180.h"
#include "track.h"
// RGB565颜色定义
#define RGB565_RED 0xF800    // 红色
#define RGB565_GREEN 0x07E0  // 绿色
#define RGB565_BLUE 0x001F   // 蓝色
#define RGB565_YELLOW 0xFFE0 // 黄色
#define RGB565_PURPLE 0xF81F // 紫色
#define RGB565_CYAN 0x07FF   // 青色
#define RGB565_WHITE 0xFFFF  // 白色
#define RGB565_BLACK 0x0000  // 黑色

// 十字路口相关全局变量
CROSS_STATE cross_state = CROSS_NONE;
CROSS_TYPE cross_type = CROSS_TYPE_NONE;
uint8 cross_detected = 0;
uint8 cross_timer = 0;
uint8 cross_enter_count = 0;
uint8 cross_exit_count = 0;
uint8 cross_center_x = 0;
uint8 cross_center_y = 0;

// 用于十字补线的边界数组
uint8 cross_l_border[IMAGE_HEIGHT];
uint8 cross_r_border[IMAGE_HEIGHT];

/*
函数名称：int my_abs(int value)
功能说明：求绝对值
参数说明：
函数返回：绝对值
修改时间：2022年9月8日
备    注：
example：  my_abs( x)；
 */
int my_abs(int value)
{
    if (value >= 0)
        return value;
    else
        return -value;
}

int16 limit_a_b(int16 x, int a, int b)
{
    if (x < a)
        x = a;
    if (x > b)
        x = b;
    return x;
}

/*
函数名称：int16 limit(int16 x, int16 y)
功能说明：求x,y中的最小值
参数说明：
函数返回：返回两值中的最小值
修改时间：2022年9月8日
备    注：
example：  limit( x,  y)
 */
int16 limit1(int16 x, int16 y)
{
    if (x > y)
        return y;
    else if (x < -y)
        return -y;
    else
        return x;
}

/*
函数名称：void get_start_point(uint8 start_row)
功能说明：寻找两个边界的边界点作为八邻域循环的起始点
参数说明：输入任意行数
函数返回：无
修改时间：2022年9月8日
备    注：
example：  get_start_point(image_h-2)
 */
uint8 start_point_l[2] = {0}; // 左边起点的x，y值
uint8 start_point_r[2] = {0}; // 右边起点的x，y值
uint8 get_start_point(uint8 start_row)
{
    uint8 i = 0, l_found = 0, r_found = 0;
    // 清零
    start_point_l[0] = 0; // x
    start_point_l[1] = 0; // y

    start_point_r[0] = 0; // x
    start_point_r[1] = 0; // y

    // 从中间往左边，先找起点
    for (i = IMAGE_WIDTH / 2; i > border_min; i--)
    {
        start_point_l[0] = i;         // x
        start_point_l[1] = start_row; // y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
        {
            // printf("找到左边起点image[%d][%d]\n", start_row,i);
            l_found = 1;
            break;
        }
    }

    for (i = IMAGE_WIDTH / 2; i < border_max; i++)
    {
        start_point_r[0] = i;         // x
        start_point_r[1] = start_row; // y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0)
        {
            // printf("找到右边起点image[%d][%d]\n",start_row, i);
            r_found = 1;
            break;
        }
    }

    if (l_found && r_found)
        return 1;
    else
    {
        // printf("未找到起点\n");
        return 0;
    }
}

/*
函数名称：void search_l_r(uint16 break_flag, uint8(*image)[IMAGE_WIDTH],uint16 *l_stastic, uint16 *r_stastic,
                        uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

功能说明：八邻域正式开始找右边点的函数，输入参数有点多，调用的时候不要漏了，这个是左右线一次性找完。
参数说明：
break_flag_r            ：最多需要循环的次数
(*image)[IMAGE_WIDTH]    ：需要进行找点的图像数组，必须是二值图,填入数组名称即可
                       特别注意，不要拿宏定义名字作为输入参数，否则数据可能无法传递过来
*l_stastic                ：统计左边数据，用来输入初始数组成员的序号和取出循环次数
*r_stastic                ：统计右边数据，用来输入初始数组成员的序号和取出循环次数
l_start_x                ：左边起点横坐标
l_start_y                ：左边起点纵坐标
r_start_x                ：右边起点横坐标
r_start_y                ：右边起点纵坐标
hightest                ：循环结束所得到的最高高度
函数返回：无
修改时间：2022年9月25日
备    注：
example：
    search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
                start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
 */
#define USE_num IMAGE_HEIGHT * 3 // 定义找点的数组成员个数按理说300个点能放下，但是有些特殊情况确实难顶，多定义了一点

// 存放点的x，y坐标
uint16 points_l[(uint16)USE_num][2] = {{0}}; // 左线
uint16 points_r[(uint16)USE_num][2] = {{0}}; // 右线
uint16 dir_r[(uint16)USE_num] = {0};         // 用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = {0};         // 用来存储左边生长方向
uint16 data_stastics_l = 0;                  // 统计左边找到点的个数
uint16 data_stastics_r = 0;                  // 统计右边找到点的个数
uint8 hightest = 0;                          // 最高点
void search_l_r(uint16 break_flag, uint8 (*image)[IMAGE_WIDTH], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8 *hightest)
{

    uint8 i = 0, j = 0;

    // 左边变量
    uint8 search_filds_l[8][2] = {{0}};
    uint8 index_l = 0;
    uint8 temp_l[8][2] = {{0}};
    uint8 center_point_l[2] = {0};
    uint16 l_data_statics; // 统计左边

    // 定义八个邻域
    static int8 seeds_l[8][2] = {
        {0, 1},
        {-1, 1},
        {-1, 0},
        {-1, -1},
        {0, -1},
        {1, -1},
        {1, 0},
        {1, 1},
    };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},         {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    // 这个是顺时针

    // 右边变量
    uint8 search_filds_r[8][2] = {{0}};
    uint8 center_point_r[2] = {0}; // 中心坐标点
    uint8 index_r = 0;             // 索引下标
    uint8 temp_r[8][2] = {{0}};
    uint16 r_data_statics; // 统计右边
    // 定义八个邻域
    static int8 seeds_r[8][2] = {
        {0, 1},
        {1, 1},
        {1, 0},
        {1, -1},
        {0, -1},
        {-1, -1},
        {-1, 0},
        {-1, 1},
    };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},         {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    // 这个是逆时针

    l_data_statics = *l_stastic; // 统计找到了多少个点，方便后续把点全部画出来
    r_data_statics = *r_stastic; // 统计找到了多少个点，方便后续把点全部画出来

    // 第一次更新坐标点  将找到的起点值传进来
    center_point_l[0] = l_start_x; // x
    center_point_l[1] = l_start_y; // y
    center_point_r[0] = r_start_x; // x
    center_point_r[1] = r_start_y; // y

    // 开启邻域循环
    while (break_flag--)
    {

        // 左边
        for (i = 0; i < 8; i++) // 传递8F坐标
        {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0]; // x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1]; // y
        }
        // 中心坐标点填充到已经找到的点内
        points_l[l_data_statics][0] = center_point_l[0]; // x
        points_l[l_data_statics][1] = center_point_l[1]; // y
        l_data_statics++;                                // 索引加一

        // 右边
        for (i = 0; i < 8; i++) // 传递8F坐标
        {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0]; // x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1]; // y
        }
        // 中心坐标点填充到已经找到的点内
        points_r[r_data_statics][0] = center_point_r[0]; // x
        points_r[r_data_statics][1] = center_point_r[1]; // y

        index_l = 0; // 先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0; // 先清零，后使用
            temp_l[i][1] = 0; // 先清零，后使用
        }

        // 左边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0 && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
            {
                temp_l[index_l][0] = search_filds_l[(i)][0];
                temp_l[index_l][1] = search_filds_l[(i)][1];
                index_l++;
                dir_l[l_data_statics - 1] = (i); // 记录生长方向
            }

            if (index_l)
            {
                // 更新坐标点
                center_point_l[0] = temp_l[0][0]; // x
                center_point_l[1] = temp_l[0][1]; // y
                for (j = 0; j < index_l; j++)
                {
                    if (center_point_l[1] > temp_l[j][1])
                    {
                        center_point_l[0] = temp_l[j][0]; // x
                        center_point_l[1] = temp_l[j][1]; // y
                    }
                }
            }
        }
        if ((points_r[r_data_statics][0] == points_r[r_data_statics - 1][0] && points_r[r_data_statics][0] == points_r[r_data_statics - 2][0] && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1]) || (points_l[l_data_statics - 1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics - 1][0] == points_l[l_data_statics - 3][0] && points_l[l_data_statics - 1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics - 1][1] == points_l[l_data_statics - 3][1]))
        {
            // printf("三次进入同一个点，退出\n");
            break;
        }
        if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2 && my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2))
        {
            // printf("\n左右相遇退出\n");
            *hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1; // 取出最高点
            // printf("\n在y=%d处退出\n",*hightest);
            break;
        }
        if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
        {
            // printf("\n如果左边比右边高了，左边等待右边\n");
            continue; // 如果左边比右边高了，左边等待右边
        }
        if (dir_l[l_data_statics - 1] == 7 && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1])) // 左边比右边高且已经向下生长了
        {
            // printf("\n左边开始向下了，等待右边，等待中... \n");
            center_point_l[0] = points_l[l_data_statics - 1][0]; // x
            center_point_l[1] = points_l[l_data_statics - 1][1]; // y
            l_data_statics--;
        }
        r_data_statics++; // 索引加一

        index_r = 0; // 先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0; // 先清零，后使用
            temp_r[i][1] = 0; // 先清零，后使用
        }

        // 右边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0 && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
            {
                temp_r[index_r][0] = search_filds_r[(i)][0];
                temp_r[index_r][1] = search_filds_r[(i)][1];
                index_r++;                       // 索引加一
                dir_r[r_data_statics - 1] = (i); // 记录生长方向
                // printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
            }
            if (index_r)
            {

                // 更新坐标点
                center_point_r[0] = temp_r[0][0]; // x
                center_point_r[1] = temp_r[0][1]; // y
                for (j = 0; j < index_r; j++)
                {
                    if (center_point_r[1] > temp_r[j][1])
                    {
                        center_point_r[0] = temp_r[j][0]; // x
                        center_point_r[1] = temp_r[j][1]; // y
                    }
                }
            }
        }
    }

    // 取出循环次数
    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;
}

/*
函数名称：void get_left(uint16 total_L)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_L    ：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example： get_left(data_stastics_l );
 */
uint8 l_border[IMAGE_HEIGHT];    // 左线数组
uint8 r_border[IMAGE_HEIGHT];    // 右线数组
uint8 center_line[IMAGE_HEIGHT]; // 中线数组
void get_left(uint16 total_L)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    // 初始化
    for (i = 0; i < IMAGE_HEIGHT; i++)
    {
        l_border[i] = border_min;
    }
    h = IMAGE_HEIGHT - 2;
    // 左边
    for (j = 0; j < total_L; j++)
    {
        // printf("%d\n", j);
        if (points_l[j][1] == h)
        {
            l_border[h] = points_l[j][0] + 1;
        }
        else
            continue; // 每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)
        {
            break; // 到最后一行退出
        }
    }
}
/*
函数名称：void get_right(uint16 total_R)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_R  ：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example：get_right(data_stastics_r);
 */
void get_right(uint16 total_R)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    for (i = 0; i < IMAGE_HEIGHT; i++)
    {
        r_border[i] = border_max; // 右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
    }
    h = IMAGE_HEIGHT - 2;
    // 右边
    for (j = 0; j < total_R; j++)
    {
        if (points_r[j][1] == h)
        {
            r_border[h] = points_r[j][0] - 1;
        }
        else
            continue; // 每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)
            break; // 到最后一行退出
    }
}

// 定义膨胀和腐蚀的阈值区间
#define threshold_max 255 * 5                      // 此参数可根据自己的需求调节
#define threshold_min 255 * 2                      // 此参数可根据自己的需求调节
void image_filter(uint8 (*bin_image)[IMAGE_WIDTH]) // 形态学滤波，简单来说就是膨胀和腐蚀的思想
{
    uint16 i, j;
    uint32 num = 0;

    for (i = 1; i < IMAGE_HEIGHT - 1; i++)
    {
        for (j = 1; j < (IMAGE_WIDTH - 1); j++)
        {
            // 统计八个方向的像素值
            num =
                bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1] + bin_image[i][j - 1] + bin_image[i][j + 1] + bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];

            if (num >= threshold_max && bin_image[i][j] == 0)
            {
                bin_image[i][j] = 255; // 白  可以搞成宏定义，方便更改
            }
            if (num <= threshold_min && bin_image[i][j] == 255)
            {
                bin_image[i][j] = 0; // 黑
            }
        }
    }
}

/*
函数名称：void image_draw_rectan(uint8(*image)[IMAGE_WIDTH])
功能说明：给图像画一个黑框
参数说明：uint8(*image)[IMAGE_WIDTH]    图像首地址
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_draw_rectan(bin_image);
 */
void image_draw_rectan(uint8 (*image)[IMAGE_WIDTH])
{
    uint8 i = 0;
    for (i = 0; i < IMAGE_HEIGHT; i++)
    {
        image[i][0] = 0;
        image[i][1] = 0;
        image[i][IMAGE_WIDTH - 1] = 0;
        image[i][IMAGE_WIDTH - 2] = 0;
    }
    for (i = 0; i < IMAGE_WIDTH; i++)
    {
        image[0][i] = 0;
        image[1][i] = 0;
        // image[image_h-1][i] = 0;
    }
}

// 路径偏差值
int16 track_deviation = 0;
// 线检测标志
uint8 line_detected = 0;

/*
函数名称：void calculate_deviation(void)
功能说明：计算路径偏差值，用于控制小车行驶
参数说明：无
函数返回：无
 */
void calculate_deviation(void)
{
    //    uint8 i;
    uint16 middle_row = IMAGE_HEIGHT * 3 / 4; // 取图像下半部分的中间行作为控制行
    int16 target_center = IMAGE_WIDTH / 2;    // 目标中点

    // 检查是否检测到有效的路径
    if (l_border[middle_row] > border_min && r_border[middle_row] < border_max)
    {
        // 计算当前中线位置
        center_line[middle_row] = (l_border[middle_row] + r_border[middle_row]) >> 1;

        // 计算偏差值
        track_deviation = center_line[middle_row] - target_center;

        // 限制偏差值范围
        track_deviation = limit_a_b(track_deviation, -30, 30);

        line_detected = 1;
    }
    else
    {
        // 未检测到有效路径
        track_deviation = 0;
        line_detected = 0;
    }
}

// /*
// 函数名称：void track_control(void)
// 功能说明：根据路径偏差值控制小车行驶
// 参数说明：无
// 函数返回：无
//  */
// void track_control(void)
// {
//     if (line_detected)
//     {
//         // 根据偏差值调整左右轮速度
//         int16 left_adjust = track_deviation * 0.8;   // 左轮调整系数
//         int16 right_adjust = -track_deviation * 0.8; // 右轮调整系数

//         // 应用调整并限制范围
//         target_left = limit_a_b(speedleft + left_adjust, 0, 100);
//         target_right = limit_a_b(speedright + right_adjust, 0, 100);
//     }
//     else
//     {
//         // 未检测到路径，减速并尝试寻找
//         target_left = limit_a_b(speedleft * 0.6, 0, 100);
//         target_right = limit_a_b(speedright * 0.6, 0, 100);
//     }
// }

/*
函数名称：void track_process(void)
功能说明：巡线处理主函数
参数说明：无
函数返回：无
 */
void track_process_circle(void)
{
    uint16 i;
    uint8 hightest = 0; // 定义一个最高行，tip：这里的最高指的是y值的最小

    // 二值化图像已经在cpu0_main.c中通过otsuThreshold函数处理

    // 应用逆透视变换（如果项目中已实现）
    // Inv_Perspective_Map_Image(bin_image, map_image);

    // 滤波和预处理
    image_filter(bin_image);
    image_draw_rectan(bin_image);

    // 清零计数
    data_stastics_l = 0;
    data_stastics_r = 0;

    if (get_start_point(IMAGE_HEIGHT - 2)) // 找到起点了，再执行八领域，没找到就一直找
    {
        // printf("正在开始八领域\n");
        search_l_r((uint16)USE_num, bin_image, &data_stastics_l, &data_stastics_r, start_point_l[0],
                   start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
        // printf("八邻域已结束\n");

        // 从爬取的边界线内提取边线，这个才是最终有用的边线
        get_left(data_stastics_l);
        get_right(data_stastics_r);

        // 计算中线
        for (i = hightest; i < IMAGE_HEIGHT - 1; i++)
        {
            if (l_border[i] > border_min && r_border[i] < border_max)
            {
                center_line[i] = (l_border[i] + r_border[i]) >> 1; // 求中线
            }
        }

        // 计算路径偏差
        calculate_deviation();

        // 控制小车行驶
        // track_control();
    }
    else
    {
        // 未找到起点
        line_detected = 0;
        track_deviation = 0;
    }

    // 显示处理后的图像
    tft180_displayimage03x((const uint8 *)bin_image, 160, 128);

    //    // 根据循环次数画出边界点
    //        for (uint16 i = 0; i < data_stastics_l; i++)
    //        {
    //            // 限制x坐标在显示屏范围内(0-159)
    //            uint16 x = limit_a_b(points_l[i][0] + 2, 0, 159);
    //            // 限制y坐标在显示屏范围内(0-127)
    //            uint16 y = limit_a_b(points_l[i][1], 0, 127);
    //            tft180_draw_point(x, y, RGB565_BLUE); // 显示左边起点
    //        }
    //        for (i = 0; i < data_stastics_r; i++)
    //        {
    //            // 限制x坐标在显示屏范围内(0-159)
    //            uint16 x = limit_a_b(points_r[i][0] - 2, 0, 159);
    //            // 限制y坐标在显示屏范围内(0-127)
    //            uint16 y = limit_a_b(points_r[i][1], 0, 127);
    //            tft180_draw_point(x, y, RGB565_RED); // 显示右边起点
    //        }
    //
    //        // 显示中线和左右边界，限制在显示屏高度范围内(0-127)
    //        for (i = hightest; i < 128; i++)
    //        {
    //            center_line[i] = (l_border[i] + r_border[i]) >> 1; // 求中线
    //            // 求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
    //            // 当然也有多组边线的找法，但是个人感觉很繁琐，不建议
    //
    //            // 使用与tft180_show_gray_image相同的坐标缩放逻辑，确保与二值化图像显示对齐
    //            // 注意：原始图像大小为188×120，显示大小为160×128
    //            uint16 center_x = limit_a_b((center_line[i] * 160) / 188, 0, 159);
    //            uint16 l_x = limit_a_b((l_border[i] * 160) / 188, 0, 159);
    //            uint16 r_x = limit_a_b((r_border[i] * 160) / 188, 0, 159);
    //            uint16 y = limit_a_b((i * 128) / 120, 0, 127);
    //            tft180_draw_point(center_x, y, RGB565_CYAN); // 显示中线
    //            tft180_draw_point(l_x, y, RGB565_GREEN);   // 显示左边线
    //            tft180_draw_point(r_x, y, RGB565_GREEN);   // 显示右边线
    //        }
    for (uint16 i = 0; i < data_stastics_l; i++)
    {
        // 使用与tft180_show_gray_image相同的坐标缩放逻辑，确保与二值化图像显示对齐
        uint16 x = limit_a_b(((points_l[i][0] + 2) * 160) / 188, 0, 159);
        uint16 y = limit_a_b((points_l[i][1] * 128) / 120, 0, 127);
        tft180_draw_point(x, y, RGB565_BLUE); // 显示左边起点
    }
    for (i = 0; i < data_stastics_r; i++)
    {
        // 使用与tft180_show_gray_image相同的坐标缩放逻辑，确保与二值化图像显示对齐
        uint16 x = limit_a_b(((points_r[i][0] - 2) * 160) / 188, 0, 159);
        uint16 y = limit_a_b((points_r[i][1] * 128) / 120, 0, 127);
        tft180_draw_point(x, y, RGB565_RED); // 显示右边起点
    }

    // 显示中线和左右边界，限制在显示屏高度范围内(0-127)
    for (i = hightest; i < 128; i++)
    {
        center_line[i] = (l_border[i] + r_border[i]) >> 1; // 求中线
        // 求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
        // 当然也有多组边线的找法，但是个人感觉很繁琐，不建议

        // 使用与tft180_show_gray_image相同的坐标缩放逻辑，确保与二值化图像显示对齐
        // 注意：原始图像大小为188×120，显示大小为160×128
        uint16 center_x = limit_a_b((center_line[i] * 160) / 188, 0, 159);
        uint16 l_x = limit_a_b((l_border[i] * 160) / 188, 0, 159);
        uint16 r_x = limit_a_b((r_border[i] * 160) / 188, 0, 159);
        uint16 y = limit_a_b((i * 128) / 120, 0, 127);
        // 根据环岛状态设置中线颜色
        if (circle_state != CIRCLE_NONE)
        {
            tft180_draw_point(center_x, y, RGB565_PURPLE); // 环岛内显示紫色中线
        }
        else
        {
            tft180_draw_point(center_x, y, RGB565_CYAN); // 非环岛显示青色中线
        }
        tft180_draw_point(l_x, y, RGB565_GREEN); // 显示左边线
        tft180_draw_point(r_x, y, RGB565_GREEN); // 显示右边线
    }
}

/*

这里是起点（0.0）***************——>*************x值最大
************************************************************
************************************************************
************************************************************
************************************************************
******************假如这是一副图像*************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
y值最大*******************************************(188.120)

*/

// 环岛处理相关代码

// 环岛相关全局变量
enum CIRCLE_STATE circle_state = CIRCLE_NONE; // 当前环岛状态
uint8 circle_entry_count = 0;                 // 进入环岛计数
uint8 circle_exit_count = 0;                  // 离开环岛计数
uint8 circle_exit_detected = 0;               // 出口检测标志

/**
 * @brief 检测环岛入口
 * @return uint8 1表示检测到入口，0表示未检测到
 */
uint8 detect_circle_entry(void)
{
    // 检查底部区域是否有宽线特征
    uint8 width_sum = 0;
    for (uint8 y = CIRCLE_BOTTOM_LINE; y < image_h; y++)
    {
        uint8 line_width = r_border[y] - l_border[y];
        if (line_width > CIRCLE_ENTRY_WIDTH_THRESHOLD)
        {
            width_sum++;
        }
    }

    // 检查左右边界是否呈现环岛入口特征
    if (width_sum > (image_h - CIRCLE_BOTTOM_LINE) * 0.7) // 70%的线满足宽度条件
    {
        circle_entry_count++;
        if (circle_entry_count >= CIRCLE_ENTRY_THRESHOLD)
        {
            return 1;
        }
    }
    else
    {
        circle_entry_count = 0;
    }

    return 0;
}

/**
 * @brief 检测环岛出口
 * @return uint8 1表示检测到出口，0表示未检测到
 */
uint8 detect_circle_exit(void)
{
    // 如果已经检测到出口，直接返回
    if (circle_exit_detected)
    {
        return 1;
    }

    // 检查是否有出口特征：底部宽度变窄，顶部保持宽线
    uint8 bottom_narrow_count = 0;
    uint8 top_wide_count = 0;

    // 检查底部区域宽度是否变窄
    for (uint8 y = CIRCLE_BOTTOM_LINE; y < image_h; y++)
    {
        uint8 line_width = r_border[y] - l_border[y];
        if (line_width < CIRCLE_EXIT_WIDTH_THRESHOLD)
        {
            bottom_narrow_count++;
        }
    }

    // 检查顶部区域是否保持宽线
    for (uint8 y = 0; y < CIRCLE_TOP_LINE; y++)
    {
        uint8 line_width = r_border[y] - l_border[y];
        if (line_width > CIRCLE_ENTRY_WIDTH_THRESHOLD)
        {
            top_wide_count++;
        }
    }

    // 判断是否满足出口条件
    if (bottom_narrow_count > (image_h - CIRCLE_BOTTOM_LINE) * 0.7 &&
        top_wide_count > CIRCLE_TOP_LINE * 0.7)
    {
        circle_exit_count++;
        if (circle_exit_count >= CIRCLE_EXIT_THRESHOLD)
        {
            circle_exit_detected = 1;
            return 1;
        }
    }
    else
    {
        circle_exit_count = 0;
    }

    return 0;
}

/**
 * @brief 环岛处理主函数
 */
void circle_process(void)
{
    switch (circle_state)
    {
    case CIRCLE_NONE:
        // 检测是否进入环岛
        if (detect_circle_entry())
        {
            circle_state = CIRCLE_ENTRY;
            // 进入环岛时的处理：降低速度、调整方向等
            target_speed = 40; // 进入环岛减速
        }
        break;

    case CIRCLE_ENTRY:
        // 进入环岛后的稳定行驶
        if (!detect_circle_entry())
        {
            circle_state = CIRCLE_ON_TRACK;
            // 环岛内正常行驶速度
            target_speed = 50;
        }
        break;

    case CIRCLE_ON_TRACK:
        // 检测是否到达出口
        if (detect_circle_exit())
        {
            circle_state = CIRCLE_EXIT_DETECT;
            // 检测到出口时的处理
        }
        break;

    case CIRCLE_EXIT_DETECT:
        // 确认出口并准备离开
        if (detect_circle_exit())
        {
            circle_state = CIRCLE_EXIT;
            // 离开环岛时的处理
        }
        break;

    case CIRCLE_EXIT:
        // 离开环岛后的恢复
        if (!detect_circle_exit())
        {
            // 重置环岛状态
            circle_state = CIRCLE_NONE;
            circle_entry_count = 0;
            circle_exit_count = 0;
            circle_exit_detected = 0;
            // 恢复正常行驶速度
            target_speed = 60;
        }
        break;

    default:
        circle_state = CIRCLE_NONE;
        break;
    }
}

/**
 * @brief 带环岛处理的巡线函数
 */
void track_process_with_circle(void)
{
    // 先执行常规巡线处理
    track_process();

    // 根据环岛状态进行处理
    circle_process();
}

/**
 * @brief 检测十字路口
 * @return uint8 1表示检测到十字路口，0表示未检测到
 */
uint8 detect_crossroad(void)
{
    uint8 i, j;
    uint8 white_pixel_count = 0;
    uint8 cross_feature_count = 0;
    
    // 方法1: 检测图像中上部区域的白色像素密度
    for (i = 5; i < 30; i++) {
        for (j = 10; j < IMAGE_WIDTH - 10; j++) {
            if (bin_image[i][j] == 255) {
                white_pixel_count++;
            }
        }
    }
    
    // 方法2: 检测边界特征 - 左右边界同时向外扩展
    uint8 left_extend = 0, right_extend = 0;
    for (i = IMAGE_HEIGHT - 10; i < IMAGE_HEIGHT - 1; i++) {
        if (l_border[i] < 20 && l_border[i] > border_min) {
            left_extend++;
        }
        if (r_border[i] > IMAGE_WIDTH - 20 && r_border[i] < border_max) {
            right_extend++;
        }
    }
    
    // 方法3: 检测中线在顶部区域的连续性
    uint8 center_line_break = 0;
    for (i = 5; i < 25; i++) {
        if (center_line[i] < IMAGE_WIDTH/2 - 10 || center_line[i] > IMAGE_WIDTH/2 + 10) {
            center_line_break++;
        }
    }
    
    // 综合判断条件
    if ((white_pixel_count > 800) || 
        (left_extend > 5 && right_extend > 5) ||
        (center_line_break < 5)) {
        cross_enter_count++;
        if (cross_enter_count > 3) {
            return 1;
        }
    } else {
        cross_enter_count = 0;
    }
    
    return 0;
}

/**
 * @brief 识别十字路口类型
 * @return CROSS_TYPE 十字路口类型
 */
CROSS_TYPE identify_cross_type(void)
{
    uint8 i;
    uint8 left_break = 0, right_break = 0;
    
    // 检测左右边界的连续性
    for (i = IMAGE_HEIGHT/2; i < IMAGE_HEIGHT - 5; i++) {
        if (l_border[i] <= border_min + 5) {
            left_break++;
        }
        if (r_border[i] >= border_max - 5) {
            right_break++;
        }
    }
    
    // 判断十字类型
    if (left_break > 3 && right_break > 3) {
        return CROSS_TYPE_NORMAL;  // 普通十字，两边都有延伸
    } else if (left_break > 3 || right_break > 3) {
        return CROSS_TYPE_T;       // T型十字，只有一边有延伸
    }
    
    return CROSS_TYPE_NORMAL;      // 默认为普通十字
}

/**
 * @brief 十字路口补线处理
 */
void crossroad_line_complement(void)
{
    uint8 i;
    float left_slope = 0, right_slope = 0;
    float left_intercept = 0, right_intercept = 0;
    
    // 保存原始边界
    memcpy(cross_l_border, l_border, IMAGE_HEIGHT);
    memcpy(cross_r_border, r_border, IMAGE_HEIGHT);
    
    // 根据十字类型采用不同的补线策略
    switch (cross_type) {
        case CROSS_TYPE_NORMAL:
            // 普通十字 - 延长左右边界
            for (i = 0; i < IMAGE_HEIGHT; i++) {
                if (l_border[i] > border_min && l_border[i] < IMAGE_WIDTH/2 - 20) {
                    l_border[i] = border_min + 5;  // 左边界向左扩展
                }
                if (r_border[i] < border_max && r_border[i] > IMAGE_WIDTH/2 + 20) {
                    r_border[i] = border_max - 5;  // 右边界向右扩展
                }
            }
            break;
            
        case CROSS_TYPE_T:
            // T型十字 - 根据缺失的边界进行补线
            if (cross_center_x < IMAGE_WIDTH/2) {
                // 左边缺失，向右补线
                for (i = 0; i < IMAGE_HEIGHT; i++) {
                    if (l_border[i] <= border_min) {
                        l_border[i] = cross_center_x - 30;
                    }
                }
            } else {
                // 右边缺失，向左补线
                for (i = 0; i < IMAGE_HEIGHT; i++) {
                    if (r_border[i] >= border_max) {
                        r_border[i] = cross_center_x + 30;
                    }
                }
            }
            break;
            
        default:
            break;
    }
    
    // 重新计算中线
    for (i = 0; i < IMAGE_HEIGHT; i++) {
        if (l_border[i] > border_min && r_border[i] < border_max) {
            center_line[i] = (l_border[i] + r_border[i]) >> 1;
        }
    }
}

/**
 * @brief 十字路口控制策略
 */
void crossroad_control_strategy(void)
{
    switch (cross_state) {
        case CROSS_DETECTED:
            // 检测到十字路口，减速并准备直行
            Target_Speed_Control(40, 40);
            break;
            
        case CROSS_ON_TRACK:
            // 在十字路口内，保持直行
            if (cross_type == CROSS_TYPE_NORMAL) {
                Target_Speed_Control(50, 50);  // 普通十字稍快
            } else {
                Target_Speed_Control(45, 45);  // T型十字稍慢
            }
            break;
            
        case CROSS_EXIT:
            // 离开十字路口，加速恢复
            Target_Speed_Control(55, 55);
            break;
            
        default:
            // 正常巡线速度
            break;
    }
}

/**
 * @brief 十字路口处理主函数
 */
void crossroad_process(void)
{
    static uint8 cross_process_timer = 0;
    
    switch (cross_state) {
        case CROSS_NONE:
            // 检测十字路口
            if (detect_crossroad()) {
                cross_state = CROSS_DETECTED;
                cross_type = identify_cross_type();
                cross_timer = 0;
                // 计算十字中心点
                cross_center_x = center_line[IMAGE_HEIGHT/2];
                cross_center_y = IMAGE_HEIGHT/2;
            }
            break;
            
        case CROSS_DETECTED:
            cross_timer++;
            if (cross_timer > 20) {  // 确认进入十字路口
                cross_state = CROSS_ON_TRACK;
                cross_timer = 0;
                // 执行补线
                crossroad_line_complement();
            }
            break;
            
        case CROSS_ON_TRACK:
            cross_timer++;
            // 在十字路口内行驶一段时间
            if (cross_timer > 80) {  // 根据实际情况调整时间
                cross_state = CROSS_EXIT;
                cross_timer = 0;
            }
            break;
            
        case CROSS_EXIT:
            cross_timer++;
            if (cross_timer > 50) {  // 确认离开十字路口
                cross_state = CROSS_NONE;
                cross_timer = 0;
                cross_enter_count = 0;
                // 恢复原始边界
                memcpy(l_border, cross_l_border, IMAGE_HEIGHT);
                memcpy(r_border, cross_r_border, IMAGE_HEIGHT);
            }
            break;
    }
    
    // 应用控制策略
    crossroad_control_strategy();
}

/*
函数名称：void track_process(void)
功能说明：巡线处理主函数
参数说明：无
函数返回：无
其他说明：与前面的track_process_circle思想与内容基本一致，主要是对环岛的处理改为了对十字的处理,后续可以尝试把一样的部分写为一个函数
 */
void track_process_cross(void)                     
{
    uint16 i;
    uint8 hightest = 0; // 定义一个最高行，这里的最高指的是y值的最小

    // 滤波和预处理
    image_filter(bin_image);
    image_draw_rectan(bin_image);
    // 清零计数
    data_stastics_l = 0;
    data_stastics_r = 0;

    if (get_start_point(IMAGE_HEIGHT - 2)) // 找到起点，再执行八领域，没找到就一直找
    {
        // printf("正在开始八领域\n");
        search_l_r((uint16)USE_num, bin_image, &data_stastics_l, &data_stastics_r, start_point_l[0],
                   start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
        // printf("八邻域已结束\n");

        // 从爬取的边界线内提取边线，这个才是最终有用的边线
        get_left(data_statics_l);
        get_right(data_statics_r);

        // 执行十字补线处理
        cross_fill(bin_image, l_border, r_border, data_statics_l, data_statics_r, dir_l, dir_r, points_l, points_r);

        // 计算中线
        for (i = hightest; i < IMAGE_HEIGHT - 1; i++)
        {
            if (l_border[i] > border_min && r_border[i] < border_max)
            {
                center_line[i] = (l_border[i] + r_border[i]) >> 1; // 求中线
            }
        }
    }
    else
    {
        // 未找到起点
        line_detected = 0;
        track_deviation = 0;
    }
    // 计算路径偏差
    calculate_deviation();
    
    // 执行十字路口处理
    crossroad_process();
    
    // 应用舵机控制（集成PID控制）
    pid_steer_control();

    // 显示处理后的图像
    tft180_displayimage03x((const uint8 *)bin_image, 160, 128);

    for (uint16 i = 0; i < data_stastics_l; i++)
    {
        // 使用与tft180_show_gray_image相同的坐标缩放逻辑，确保与二值化图像显示对齐
        uint16 x = limit_a_b(((points_l[i][0] + 2) * 160) / 188, 0, 159);
        uint16 y = limit_a_b((points_l[i][1] * 128) / 120, 0, 127);
        tft180_draw_point(x, y, RGB565_BLUE); // 显示左边起点
    }
    for (i = 0; i < data_stastics_r; i++)
    {
        // 使用与tft180_show_gray_image相同的坐标缩放逻辑，确保与二值化图像显示对齐
        uint16 x = limit_a_b(((points_r[i][0] - 2) * 160) / 188, 0, 159);
        uint16 y = limit_a_b((points_r[i][1] * 128) / 120, 0, 127);
        tft180_draw_point(x, y, RGB565_RED); // 显示右边起点
    }

    // 显示中线和左右边界，限制在显示屏高度范围内(0-127)
    for (i = hightest; i < 128; i++)
    {
        center_line[i] = (l_border[i] + r_border[i]) >> 1; // 求中线

        // 使用与tft180_show_gray_image相同的坐标缩放逻辑，确保与二值化图像显示对齐
        // 注意：原始图像大小为188×120，显示大小为160×128
        uint16 center_x = limit_a_b((center_line[i] * 160) / 188, 0, 159);
        uint16 l_x = limit_a_b((l_border[i] * 160) / 188, 0, 159);
        uint16 r_x = limit_a_b((r_border[i] * 160) / 188, 0, 159);
        uint16 y = limit_a_b((i * 128) / 120, 0, 127);
        // 根据十字状态设置中线颜色
        if (cross_state != CROSS_TYPE_NONE)
        {
            tft180_draw_point(center_x, y, RGB565_PURPLE); // 十字内显示紫色中线
        }
        else
        {
            tft180_draw_point(center_x, y, RGB565_CYAN); // 非十字显示青色中线
        }
        tft180_draw_point(l_x, y, RGB565_GREEN); // 显示左边线
        tft180_draw_point(r_x, y, RGB565_GREEN); // 显示右边线
    }
}
