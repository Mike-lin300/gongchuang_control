#ifndef __UTILS_H
#define __UTILS_H

#include <stdint.h>

/* ---- 整数数学 ---- */

int32_t abs_i32(int32_t v);

/* ---- 角度（单位统一 0.01 度） ---- */

int32_t angle_wrap_180_centideg(int32_t angle);




/* 其他函数做个登记，本次不实现 */

// clamp_i32
// in_range_i32
// deadband_i32
// abs_f
// in_range_f
// deadband_f
// round_f
// lowpass_filter_f
// pid_*

// 比较杂，如果实现pid的时候可能还需要再开一个 utils/ 然后分分类



#endif /* __UTILS_H */
