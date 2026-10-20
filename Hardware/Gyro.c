#include "Gyro.h"

static uint8_t frame[GYRO_FRAME_LEN];
static uint8_t frame_pos;
static volatile int16_t yaw_raw;
static volatile int32_t yaw_centi_degrees;
static volatile uint8_t yaw_valid;
volatile uint32_t Gyro_FrameCount;

static void Gyro_ResetParser(void)
{
    frame_pos = 0;
}

static void Gyro_CommitAngle(void)
{
    uint16_t raw_u16;
    int16_t raw;

    raw_u16 = (uint16_t)frame[6] | ((uint16_t)frame[7] << 8);
    if (raw_u16 & 0x8000u)
        raw = (int16_t)(-((int32_t)(0x10000u - raw_u16)));
    else
        raw = (int16_t)raw_u16;

    yaw_raw = raw;
    yaw_centi_degrees = ((int32_t)raw * GYRO_ANGLE_SCALE_NUM) /
                        GYRO_ANGLE_SCALE_DEN;
    yaw_valid = 1;
    Gyro_FrameCount++;

}

void Gyro_Init(void)
{
    Gyro_ResetParser();
    yaw_raw = 0;
    yaw_centi_degrees = 0;
    yaw_valid = 0;
    Gyro_FrameCount = 0;
}

void Gyro_ProcessByte(uint8_t byte)
{
    uint8_t i;
    uint8_t checksum;

    /* 等待帧头。 */
    if (frame_pos == 0u)
    {
        if (byte == GYRO_FRAME_HEADER)
        {
            frame[0] = byte;
            frame_pos = 1u;
        }
        return;
    }

    /* 第二字节必须是角度帧类型；55 52 等其他 JY901 帧直接丢弃。 */
    if (frame_pos == 1u)
    {
        if (byte == GYRO_TYPE_ANGLE)
        {
            frame[1] = byte;
            frame_pos = 2u;
        }
        else if (byte == GYRO_FRAME_HEADER)
        {
            /* 当前字节可能已经是下一帧的帧头。 */
            frame[0] = byte;
            frame_pos = 1u;
        }
        else
        {
            Gyro_ResetParser();
        }
        return;
    }

    frame[frame_pos++] = byte;
    if (frame_pos < GYRO_FRAME_LEN)
        return;

    checksum = 0u;
    for (i = 0u; i < GYRO_FRAME_LEN - 1u; i++)
        checksum = (uint8_t)(checksum + frame[i]);

    if (checksum == frame[GYRO_FRAME_LEN - 1u])
    {
        Gyro_CommitAngle();
    }

    Gyro_ResetParser();
}

uint8_t Gyro_GetYawCentiDegrees(int32_t *centi_degrees)
{
    if (centi_degrees == 0 || !yaw_valid)
        return 0u;

    *centi_degrees = yaw_centi_degrees;
    return 1u;
}

uint8_t Gyro_GetYawRaw(int16_t *raw)
{
    if (raw == 0 || !yaw_valid)
        return 0u;

    *raw = yaw_raw;
    return 1u;
}


/**
 * @brief   等待一帧新的有效角度，避免用旧数据重复计算 PD。
 * @param   last_frame_count  上次已处理的陀螺仪帧计数，传入地址以便更新。
 * @param   yaw               输出：新的角度，单位 0.01 度。
 * @retval                1 表示成功，0 表示超时。
 * @note        
 */
u8 gyro_wait_new_sample(uint32_t *last_frame_count, int32_t *yaw)
{
    uint16_t timeout;

    for(timeout = 0; timeout < 200u; timeout++){
        if(Gyro_FrameCount != *last_frame_count){
            if(Gyro_GetYawCentiDegrees(yaw)){
                *last_frame_count = Gyro_FrameCount;
                return 1;
            }
        }
        Delay_ms(10);
    }
    return 0;
}
