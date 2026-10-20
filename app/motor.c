#include "motor.h"
#include "Gyro.h"
#include "Serial.h"
#include "Delay.h"

/*
 * 依赖你工程里的串口/延时函数（请确保对应头文件已包含）：
 *   Serial_SendArray(buf, len)   Serial_SendByte(byte)   Delay_ms(ms)
 */

/* —— 标定 —— */
#define PULSE_PER_100CM  13008u   /** 平移标定：13008u 脉冲 = 100 cm（已标定）
                                    * 左右平移会稍短，每 100cm 约少 5 cm
                                    */
#define PULSE_PER_360DEG    18373u     /** 旋转标定：18373u 脉冲 = 360°（10月16日）
                                         * 开环旋转必然打滑，此标定与地面性质有关，经过实测：
                                         * 实验室大理石地面上：18373u 脉冲 = 361.4°
                                         * 实验室场地白布上：18373u 脉冲 = 358.9°
                                         * 结论：场地越粗糙，阻力大，不同轮之间打滑差异越大，旋转角度越小
                                         */

/* 速度 / 加速度（可调） */
#define MOTOR_SPEED      70    /* 速度 0x07D0；上位机默认 200 已较快 */
#define MOTOR_ACCEL      200     /* 加速度 0x64 */
#define MOTOR_SPEED_H    ((uint8_t)(MOTOR_SPEED >> 8))   /* 速度高字节 */
#define MOTOR_SPEED_L    ((uint8_t)(MOTOR_SPEED & 0xFF)) /* 速度低字节 */

/* F6 速度模式参数：速度 0~3000 RPM，加速度 0~255 档。 */
#define SPEED_MODE_FUNCTION 0xF6u
#define TURN_SPEED_ACCEL    150u

/*
 * 移动命令（13 字节）：
 *   [地址][FD][方向][速度H][速度L][加速度][脉冲B3][脉冲B2][脉冲B1][脉冲B0][相对][同步][校验]
 *   方向 0x00=正转/CW，0x01=反转/CCW。
 *   FR(地址2)、RR(地址4) 镜像安装，前进时发反转。
 *   同步 0x01=等待同步信号（直行用，末尾触发帧统一起步），0x00=立即执行（旋转用）。
 *   校验固定 0x6B。
 */

/* 前进：mm 毫米 */
void qianjin_(uint32_t mm)
{
    uint32_t p = mm * PULSE_PER_100CM / 1000u;   /* 脉冲数 */
    uint8_t cmd[13] = {0x00, 0xFD, 0x00, MOTOR_SPEED_H, MOTOR_SPEED_L, (uint8_t)MOTOR_ACCEL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x6B};

    cmd[6] = (uint8_t)(p >> 24);
    cmd[7] = (uint8_t)(p >> 16);
    cmd[8] = (uint8_t)(p >> 8);
    cmd[9] = (uint8_t)p;

    cmd[0] = 0x01; cmd[2] = 0x01;   /* FL 正转 */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x02; cmd[2] = 0x00;   /* FR 反转（镜像） */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x03; cmd[2] = 0x01;   /* RL 正转 */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x04; cmd[2] = 0x00;   /* RR 反转（镜像） */
    Serial_SendArray(cmd, 13); Delay_ms(5);

    /* 同步释放：直行 sync=0x01，此帧让 4 个电机同时起步 */
    Serial_SendByte(0x00); Serial_SendByte(0xFF); Serial_SendByte(0x66); Serial_SendByte(0x6B);
	Delay_ms(5);
}

/* 后退：mm 毫米 */
void houtui_(uint32_t mm)
{
    uint32_t p = mm * PULSE_PER_100CM / 1000u;
    uint8_t cmd[13] = {0x00, 0xFD, 0x00, MOTOR_SPEED_H, MOTOR_SPEED_L, (uint8_t)MOTOR_ACCEL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x6B};

    cmd[6] = (uint8_t)(p >> 24);
    cmd[7] = (uint8_t)(p >> 16);
    cmd[8] = (uint8_t)(p >> 8);
    cmd[9] = (uint8_t)p;

    cmd[0] = 0x01; cmd[2] = 0x00;   /* FL 反转 */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x02; cmd[2] = 0x01;   /* FR 正转（镜像） */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x03; cmd[2] = 0x00;   /* RL 反转 */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x04; cmd[2] = 0x01;   /* RR 正转（镜像） */
    Serial_SendArray(cmd, 13); Delay_ms(5);

    Serial_SendByte(0x00); Serial_SendByte(0xFF); Serial_SendByte(0x66); Serial_SendByte(0x6B);
	Delay_ms(5);
}

/* 左平移 */
void zuo_(uint32_t mm)
{
    uint32_t p = mm * PULSE_PER_100CM / 1000u;
    uint8_t cmd[13] = {0x00, 0xFD, 0x00, MOTOR_SPEED_H, MOTOR_SPEED_L, (uint8_t)MOTOR_ACCEL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x6B};

    cmd[6] = (uint8_t)(p >> 24);
    cmd[7] = (uint8_t)(p >> 16);
    cmd[8] = (uint8_t)(p >> 8);
    cmd[9] = (uint8_t)p;

    cmd[0] = 0x01; cmd[2] = 0x01;   /* FL */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x02; cmd[2] = 0x01;   /* FR */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x03; cmd[2] = 0x00;   /* RL */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x04; cmd[2] = 0x00;   /* RR */
    Serial_SendArray(cmd, 13); Delay_ms(5);

    Serial_SendByte(0x00); Serial_SendByte(0xFF); Serial_SendByte(0x66); Serial_SendByte(0x6B);
	Delay_ms(5);
}

/* 右平移*/
void you_(uint32_t mm)
{
    uint32_t p = mm * PULSE_PER_100CM / 1000u;
    uint8_t cmd[13] = {0x00, 0xFD, 0x00, MOTOR_SPEED_H, MOTOR_SPEED_L, (uint8_t)MOTOR_ACCEL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x6B};

    cmd[6] = (uint8_t)(p >> 24);
    cmd[7] = (uint8_t)(p >> 16);
    cmd[8] = (uint8_t)(p >> 8);
    cmd[9] = (uint8_t)p;

    cmd[0] = 0x01; cmd[2] = 0x00;   /* FL */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x02; cmd[2] = 0x00;   /* FR */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x03; cmd[2] = 0x01;   /* RL */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x04; cmd[2] = 0x01;   /* RR */
    Serial_SendArray(cmd, 13); Delay_ms(5);

    Serial_SendByte(0x00); Serial_SendByte(0xFF); Serial_SendByte(0x66); Serial_SendByte(0x6B);
	Delay_ms(5);
}

static uint32_t motor_centideg_to_pulse(int32_t centideg)
{
    int64_t num = (int64_t)centideg * (int64_t)PULSE_PER_360DEG;

    if (num >= 0)
        return (uint32_t)((num + 18000) / 36000);   /* 四舍五入 */
    else
        return (uint32_t)((-num + 18000) / 36000);
}

// 细化旋转
void motor_rotate_centideg(int32_t centideg)
{
    uint32_t p;
    uint8_t  dir;
    uint8_t  cmd[13] = {0x00, 0xFD, 0x00, MOTOR_SPEED_H, MOTOR_SPEED_L,
                        (uint8_t)MOTOR_ACCEL, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x01, 0x6B};

    if (centideg == 0) return;

    if (centideg >= 0) { dir = 0x01; p = motor_centideg_to_pulse(centideg); }
    else               { dir = 0x00; p = motor_centideg_to_pulse(-centideg); }

    cmd[2] = dir;
    cmd[6] = (uint8_t)(p >> 24);
    cmd[7] = (uint8_t)(p >> 16);
    cmd[8] = (uint8_t)(p >> 8);
    cmd[9] = (uint8_t)p;

    cmd[0] = 0x01; Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x02; Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x03; Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x04; Serial_SendArray(cmd, 13); Delay_ms(5);

    Serial_SendByte(0x00); Serial_SendByte(0xFF);
    Serial_SendByte(0x66); Serial_SendByte(0x6B);
    Delay_ms(5);
}

/**
 * @brief   陀螺仪比例转向：先执行目标增量，再按剩余误差的一半逐次修正。
 * @param   target_delta    使用相对起始角度增量，单位 0.01 度，正（顺时针）负（逆时针）。
 * @retval                  1 表示成功，0 表示失败。
 * @note        
 */
u8 gyro_turn_pos_test(int32_t target_delta)
{
    const int32_t TOLERANCE = 10L;              // 到位容差 0.1 度
    const uint8_t MAX_CORRECTIONS = 5u;         // 最多修正5次，不含首发命令
    const int32_t MAX_CORRECTION_DEG = 30L;     // 修正时最大转 30 度
    // start_yaw 表示进入时陀螺仪 yaw， 作为相对零点
    // yaw 表示每次新样本的绝对yaw
    int32_t start_yaw, yaw, relative_yaw, error, centideg;
    uint32_t last_frame_count;
    uint8_t correction_count = 0u;
    uint8_t stable_count = 0u;                  // 连续进入容差的新样本数，达到 3 才算真正到位
    uint8_t first_feedback = 1u;                // 单次检验标志

    if(!Gyro_GetYawCentiDegrees(&start_yaw)) return 0;      // 如果陀螺仪未接入，直接返回失败
    last_frame_count = Gyro_FrameCount;                     // 记录最新的帧号，后面只接受更新的

    /* 先按目标方向执行一次完整位置命令。 */
    motor_rotate_centideg(target_delta);

    Delay_ms(50);
    if(!judge()) return 0;

    while(1){                                   // 离散的修正循环
        if(!gyro_wait_new_sample(&last_frame_count, &yaw)) return 0;

        relative_yaw = angle_wrap_180_centideg(yaw - start_yaw);          // relative_yaw - start_yaw 并 wrap 到 [-180,180) 表示实际已转相对角度
        error = angle_wrap_180_centideg(target_delta - relative_yaw);     // 剩余误差，若 > 0 继续逆，若 < 0 继续顺
        OLED_ShowString(3, 1, "DY:");
        OLED_ShowSignedNum(3, 4, relative_yaw / 100L, 3);

        if(error <= TOLERANCE && error >= -TOLERANCE){
            if(++stable_count >= 3u) return 1;                  // 如果连续 3 个新样本都在容差内，判为真正到位
            continue;
        }
        stable_count = 0u;

        /* 单次检验，用于排查是否反了，可以做一个保护, 5 度的保护阈值。 */
        if(first_feedback){
            first_feedback = 0u;
            if((target_delta > 0L && relative_yaw < -500L) ||
               (target_delta < 0L && relative_yaw > 500L))
                return 0;
        }

        if(correction_count >= MAX_CORRECTIONS) return 0;       // 修正次数用完仍然未进入容差，失败       

        centideg = error / 2;

        motor_rotate_centideg(centideg);

        Delay_ms(50);
        if(!judge()) return 0;
        correction_count++;
    }
}


/* 左右旋转；旧命令，后续停止使用（薄封装，还能用，但实际执行 motor_rotate_centideg ）*/
void shun_yaw_(uint32_t deg) { motor_rotate_centideg(-(int32_t)(deg * 100u)); }
void ni_yaw_(uint32_t deg)   { motor_rotate_centideg( (int32_t)(deg * 100u)); }

/* 发送 F6 速度模式命令：[地址][F6][方向][速度H][速度L][加速度][同步][6B]。
 * 旋转仍使用 FL(1) 和 FR(2)，两个电机方向按现有旋转逻辑保持一致。
 * 速度模式是持续运行命令，rpm=0 用于停止。 */
static void motor_send_speed_command(uint8_t address, uint8_t direction, uint16_t rpm)
{
    uint8_t cmd[8];

    cmd[0] = address;
    cmd[1] = SPEED_MODE_FUNCTION;
    cmd[2] = direction;
    cmd[3] = (uint8_t)(rpm >> 8);
    cmd[4] = (uint8_t)(rpm & 0xFFu);
    cmd[5] = TURN_SPEED_ACCEL;
    cmd[6] = 0x00;                 /* 立即执行，不等待同步释放 */
    cmd[7] = 0x6B;

    Serial_SendArray(cmd, 8);
    Delay_ms(2);
}

void motor_rotate_speed(uint8_t direction, uint16_t rpm)
{
    motor_send_speed_command(0x01, direction, rpm);
    motor_send_speed_command(0x02, direction, rpm);
}

void motor_rotate_speed_stop(void)
{
    motor_rotate_speed(0x00, 0x0000);
}

/* 使能 4 个电机：[地址][F3][AB][01][同步][校验]，上电默认失能，移动前先调一次 */
void motor_enable_all(void)
{
    uint8_t i;
    for (i = 1; i <= 4; i++) {
        Serial_SendByte(i);
        Serial_SendByte(0xF3);
        Serial_SendByte(0xAB);
        Serial_SendByte(0x01);
        Serial_SendByte(0x00);
        Serial_SendByte(0x6B);
        Delay_ms(5);
    }
}
volatile uint8_t uart_key_num = 0;
int32_t  pos_now  = 0;    // 电机当前位置（有符号）
int32_t  pos_memo = 0;    // 记忆位置（有符号）
uint8_t  pos_dir  = 0;    // 方向位暂存
void motor_read_pos(void)
{
    Serial_SendByte(0x01);
    Serial_SendByte(0x36);
    Serial_SendByte(0x6B);

    Serial_RxFlag = 0;
    uint32_t timeout = 200000;
    while (Serial_RxFlag == 0 && timeout > 0) timeout--;

    if (Serial_RxFlag &&
        Serial_RxPacket[0] == 0x01 &&
        Serial_RxPacket[1] == 0x36 &&
        Serial_RxPacket[7] == 0x6B)
    {
        pos_dir = Serial_RxPacket[2];

        uint32_t p = ((uint32_t)Serial_RxPacket[3] << 24) |
                     ((uint32_t)Serial_RxPacket[4] << 16) |
                     ((uint32_t)Serial_RxPacket[5] << 8)  |
                     ((uint32_t)Serial_RxPacket[6]);

        /* 方向位 0x01 代表负，就转成负数 */
        if (pos_dir == 0x01)
            pos_now = -(int32_t)p;
        else
            pos_now =  (int32_t)p;
    }
    Serial_RxFlag = 0;
}

uint8_t cmd[13] = {
    0x01,                       // [0] 地址（原来是 0x00，改成 0x01）
    0xFD,                       // [1] 命令
    0x00,                       // [2] 方向，运行时覆盖
    0x00, 0x64,                 // [3][4] 速度 = 0x1194 = 4500
    0xC8,                       // [5] 加速度 = 200
    0x00, 0x00, 0x00, 0x00,     // [6..9] 位置，运行时覆盖
    0x01,                       // [10] 绝对位置模式
    0x00,                       // [11] 保留
    0x6B                        // [12] 尾部
};
void motor_set_pos(int32_t pos)
{
    uint32_t absp;
    uint8_t  dir;

    if (pos < 0)
    {
        dir  = 0x01;              // 负数方向
        absp = (uint32_t)(-pos);  // 绝对值
    }
    else
    {
        dir  = 0x00;              // 正数方向
        absp = (uint32_t)pos;
    }

    cmd[2] = dir;
    cmd[6] = (uint8_t)(absp >> 24);
    cmd[7] = (uint8_t)(absp >> 16);
    cmd[8] = (uint8_t)(absp >> 8);
    cmd[9] = (uint8_t)(absp);

    Serial_SendArray(cmd, 13);
}
