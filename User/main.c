#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "Gyro.h"
#include "Key.h"
#include "LED.h"
#include "motor.h"
#include "path_planner.h"
#include "path_config.h"

static u8 dir = PATH_INITIAL_DIR;
static u16 judge_trans_display_count = 0;

#define ABS(x) ((x) < 0 ? -(x) : (x))

static u8 judge(void);

/* 等待四个驱动电机全部到位。
 * 0x3A 查询的回包格式为 [地址][0x3A][状态][0x6B]，其中状态字节 (Serial_RxPacket[2])
 * 为 0x03 时表示该电机已到达。每查一个地址，先清标志再发查询，然后阻塞等回包，
 * 确认回包地址与本电机一致后再判断到位，避免把别的电机的回包误判成自己的。 */
static u8 judge(void)
{
    u8 judge1 = 0, judge2 = 0, judge3 = 0, judge4 = 0;
    u16 t, guard = 0;

    while(1){
        if(judge1 == 0){
            Serial_RxFlag = 0;
            Serial_RxPacket[0] = Serial_RxPacket[1] = Serial_RxPacket[2] = Serial_RxPacket[3] = 0;
            Serial_SendByte(0x01); Serial_SendByte(0x3A); Serial_SendByte(0x6B);
            for(t = 0; t < 50; t++){ if(Serial_GetRxFlag()) break; Delay_ms(1); }
            if(Serial_RxPacket[0] == 0x01 && Serial_RxPacket[1] == 0x3A && Serial_RxPacket[2] == 0x03)
                judge1 = 1;
        }
        Delay_ms(5);
        if(judge2 == 0){
            Serial_RxFlag = 0;
            Serial_RxPacket[0] = Serial_RxPacket[1] = Serial_RxPacket[2] = Serial_RxPacket[3] = 0;
            Serial_SendByte(0x02); Serial_SendByte(0x3A); Serial_SendByte(0x6B);
            for(t = 0; t < 50; t++){ if(Serial_GetRxFlag()) break; Delay_ms(1); }
            if(Serial_RxPacket[0] == 0x02 && Serial_RxPacket[1] == 0x3A && Serial_RxPacket[2] == 0x03)
                judge2 = 1;
        }
        Delay_ms(5);
        if(judge3 == 0){
            Serial_RxFlag = 0;
            Serial_RxPacket[0] = Serial_RxPacket[1] = Serial_RxPacket[2] = Serial_RxPacket[3] = 0;
            Serial_SendByte(0x03); Serial_SendByte(0x3A); Serial_SendByte(0x6B);
            for(t = 0; t < 50; t++){ if(Serial_GetRxFlag()) break; Delay_ms(1); }
            if(Serial_RxPacket[0] == 0x03 && Serial_RxPacket[1] == 0x3A && Serial_RxPacket[2] == 0x03)
                judge3 = 1;
        }
        Delay_ms(5);
        if(judge4 == 0){
            Serial_RxFlag = 0;
            Serial_RxPacket[0] = Serial_RxPacket[1] = Serial_RxPacket[2] = Serial_RxPacket[3] = 0;
            Serial_SendByte(0x04); Serial_SendByte(0x3A); Serial_SendByte(0x6B);
            for(t = 0; t < 50; t++){ if(Serial_GetRxFlag()) break; Delay_ms(1); }
            if(Serial_RxPacket[0] == 0x04 && Serial_RxPacket[1] == 0x3A && Serial_RxPacket[2] == 0x03)
                judge4 = 1;
        }
        Delay_ms(5);

        if(judge1 == 1 && judge2 == 1 && judge3 == 1 && judge4 == 1){
            return 1;
        }

        OLED_ShowNum(2, 1, judge_trans_display_count++, 5);

        if(++guard > 60000u){   /* 安全兜底，仅在电机死机时触发 */
            return 0;
        }
    }
}

static u8 translation(u16 x0, u16 y0, u16 x1, u16 y1)
{
    u16 distance;

    if(x1 == x0 && y1 > y0){
        distance = y1 - y0;
        if(dir == 2){ shun_yaw_(90); Delay_ms(1000); if(!judge()) return 0; dir = 1; }
        if(dir == 4){ ni_yaw_(90);   Delay_ms(1000); if(!judge()) return 0; dir = 1; }
        if(dir == 3) houtui_(distance);
        else         qianjin_(distance);
    }else if(x1 == x0 && y1 < y0){
        distance = y0 - y1;
        if(dir == 2){ ni_yaw_(90);   Delay_ms(1000); if(!judge()) return 0; dir = 3; }
        if(dir == 4){ shun_yaw_(90); Delay_ms(1000); if(!judge()) return 0; dir = 3; }
        if(dir == 3) qianjin_(distance);
        else         houtui_(distance);
    }else if(x1 < x0 && y1 == y0){
        distance = x0 - x1;
        if(dir == 1){ ni_yaw_(90);   Delay_ms(1000); if(!judge()) return 0; dir = 2; }
        else if(dir == 3){ shun_yaw_(90); Delay_ms(1000); if(!judge()) return 0; dir = 2; }
        if(dir == 2) qianjin_(distance);
        else         houtui_(distance);
    }else if(x1 > x0 && y1 == y0){
        distance = x1 - x0;
        if(dir == 1){ shun_yaw_(90); Delay_ms(1000); if(!judge()) return 0; dir = 4; }
        if(dir == 3){ ni_yaw_(90);   Delay_ms(1000); if(!judge()) return 0; dir = 4; }
        if(dir == 4) qianjin_(distance);
        else         houtui_(distance);
    }else{
        return 0;
    }

    return judge();
}

/* 从 (cx,cy) 规划到 (tx,ty) 并逐段执行，每段阻塞等电机到位。
 * 成功返回 1；无路径或某段超时返回 0。 */
static u8 go_to(u16 *cx, u16 *cy, u16 tx, u16 ty)
{
    u16 xpath[MAX_KEY_POINTS], ypath[MAX_KEY_POINTS];
    u8 i;

    if(*cx == tx && *cy == ty) return 1;          /* 已在目标点，无需移动 */

    path_plan(*cx, *cy, tx, ty, MAX_KEY_POINTS, xpath, ypath);
    if(key_point_num < 2) return 0;               /* 规划失败（无路径 / 参数非法） */

    for(i = 1; i < key_point_num; i++){
        if(!translation(xpath[i - 1], ypath[i - 1], xpath[i], ypath[i])) return 0;
    }

    *cx = tx;
    *cy = ty;
    return 1;
}

int main(void)
{
    u16 cur_x = PATH_START_X;
    u16 cur_y = PATH_START_Y;
    u8 step = 0;
    int32_t yaw_centi_degrees;

    LED_Init();
    OLED_Init();
    Key_Init();
    Serial_Init();
    Gyro_Init();
    USART3_Init(115200);
    USART2_Init(115200);
    Delay_ms(200);

    path_planner_init();
    motor_enable_all();

//	go_to(&cur_x, &cur_y, 2250u, 1200u);
//	while(1){
//		
//	}
    /* 路线：每个 case 开头指定「下一个地点」，随后 go_to 完成规划 + 发送电机 + 等到位，
     * 全部到位后才进入下一个 case（不共享 path_ready/path_index 之类的中间状态）。 */
    while(1)
    {
        OLED_ShowNum(1, 1, step, 4);

        /* 显示最近一次校验通过的 Z 轴角度，单位 0.01 度。传感器未接入时
         * 保留空白，解析仍在 USART2 中断中持续进行。 */
        OLED_ShowString(4, 1, "YAW:");
        if (Gyro_GetYawCentiDegrees(&yaw_centi_degrees))
            OLED_ShowSignedNum(4, 5, yaw_centi_degrees, 6);

        switch(step)
        {
            case 0: if(!go_to(&cur_x, &cur_y, 2250u, 1200u)) step = 99; else step = 1;   break; /* -> (2250,1200) */
            case 1: if(!go_to(&cur_x, &cur_y, 1200u, 2250u)) step = 99; else step = 2;   break; /* -> (1200,2250) */
            case 2: if(!go_to(&cur_x, &cur_y, 1200u, 150u))  step = 99; else step = 3;   break; /* -> (1200,150)  */
            case 3: if(!go_to(&cur_x, &cur_y, 150u,  1200u)) step = 99; else step = 4;   break; /* -> (150,1200)  */
            case 4: if(!go_to(&cur_x, &cur_y, 1200u, 2250u)) step = 99; else step = 5;   break; /* -> (1200,2250) */
            case 5: if(!go_to(&cur_x, &cur_y, 1200u, 150u))  step = 99; else step = 6;   break; /* -> (1200,150)  */
            case 6: if(!go_to(&cur_x, &cur_y, 150u,  1200u)) step = 99; else step = 7;   break; /* -> (150,1200)  */
            case 7: if(!go_to(&cur_x, &cur_y, PATH_TARGET_X, PATH_TARGET_Y)) step = 99; else step = 100;   break; /* -> (1200,2250) */
            case 8: if(!go_to(&cur_x, &cur_y, 1200u, 150u))  step = 99; else step = 9;   break; /* -> (1200,150)  */
            case 9: if(!go_to(&cur_x, &cur_y, PATH_TARGET_X, PATH_TARGET_Y)) step = 99; else step = 100; break; /* -> 终点(2250,2250) */

            case 100:
                OLED_ShowString(1, 1, "PATH DONE");
                while(1){}
            case 99:
                OLED_ShowString(1, 1, "PATH ERROR");
                while(1){}
        }
    }
}
