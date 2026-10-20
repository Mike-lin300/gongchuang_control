#include "utils.h"

/**
 * @brief   求 int32_t 绝对值。
 * @param   v   要求绝对值的 int32_t 整数。
 * @retval      返回绝对值。
 * @note    边界：v == INT32_MIN 时结果无法用 int32_t 表示，本函数饱和返回 INT32_MAX。
 */
int32_t abs_i32(int32_t v)
{
    if (v == INT32_MIN) {
        return INT32_MAX;          /* 饱和，见头文件说明 */
    }
    return (v < 0) ? -v : v;
}


/**
 * @brief   把角度 wrap 到 [-18000, 18000)，即 [-180.00°, 180.00°)。
 * @param   angle   要求 wrap 的角度，单位 0.01 度。
 * @retval      返回 wrap 后的角度，单位 0.01 度。
 * @note    例如：-18000 → -18000，-18001 → 17999。
 */
int32_t angle_wrap_180_centideg(int32_t angle)
{
    // angle %= 36000L;
    if (angle >= 18000L)  angle -= 36000L;
    if (angle < -18000L)  angle += 36000L;
    return angle;
}

