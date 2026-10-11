#ifndef __GYRO_H
#define __GYRO_H

#include "stm32f10x.h"

/* JY901/WitMotion 兼容角度帧：55 53 ...，总长 11 字节。 */
#define GYRO_FRAME_LEN       11u
#define GYRO_FRAME_HEADER    0x55u
#define GYRO_TYPE_ANGLE      0x53u
#define GYRO_ANGLE_SCALE_NUM 18000L
#define GYRO_ANGLE_SCALE_DEN 32768L

/* 在 USART2 接收中断中调用一次，传入刚收到的一个字节。 */
void Gyro_ProcessByte(uint8_t byte);

/* 清空解析状态和上一次数据。 */
void Gyro_Init(void);

/* 返回最近一次校验通过的 Z 轴角度，单位为 0.01 度。 */
uint8_t Gyro_GetYawCentiDegrees(int32_t *centi_degrees);

/* 返回最近一次校验通过的 Z 轴原始有符号值。 */
uint8_t Gyro_GetYawRaw(int16_t *raw);

/* 最近一次有效帧的计数，可用于判断传感器是否正在输出。 */
extern volatile uint32_t Gyro_FrameCount;

#endif /* __GYRO_H */
