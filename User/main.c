#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "Key.h"
#include "LED.h"
#include "motor.h"
#include "path_planner.h"
#include "path_config.h"

static u8 dir = PATH_INITIAL_DIR;
static u16 judge_trans_display_count = 0;
static u8 OrdinalNum = 0;
static u16 route_current_x = PATH_START_X;
static u16 route_current_y = PATH_START_Y;

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

void Mytest(unsigned char dT_ms)
{
    static u8 initialized = 0;
    static u8 path_ready = 0;
    static u8 path_index = 0;
    static u8 terminal_displayed = 1;
    static u16 xpath[MAX_KEY_POINTS];
    static u16 ypath[MAX_KEY_POINTS];
    u16 target_x, target_y;

    (void)dT_ms;

    if(!initialized){
        path_planner_init();
        motor_enable_all();
        initialized = 1;
    }

    switch(OrdinalNum)
    {
        case 0:  target_x = 2250u; target_y = 1200u; break; /* (2250,2250) -> (2250,1200) */
        case 10: target_x = 1200u; target_y = 2250u; break; /* -> (1200,2250) */
        case 20: target_x = 1200u; target_y = 150u;  break; /* -> (1200,150) */
        case 30: target_x = 150u;  target_y = 1200u; break; /* -> (150,1200) */
        case 40: target_x = 2250u; target_y = 1200u; break; /* -> (2250,1200) */
        case 50: target_x = 1200u; target_y = 2250u; break; /* -> (1200,2250) */
        case 60: target_x = 1200u; target_y = 150u;  break; /* -> (1200,150) */
        case 70: target_x = 150u;  target_y = 1200u; break; /* -> (150,1200) */
        case 80: target_x = PATH_TARGET_X; target_y = PATH_TARGET_Y; break; /* -> (2250,2250) */

        case 90:
            if(!terminal_displayed){
                OLED_ShowString(1, 1, "PATH DONE");
                terminal_displayed = 1;
            }
            return;

        case 99:
            if(!terminal_displayed){
                OLED_ShowString(1, 1, "PATH ERROR");
                terminal_displayed = 1;
            }
            return;

        default:
            OrdinalNum = 99;
            return;
    }

    if(!path_ready){
        path_plan(route_current_x, route_current_y,
                  target_x, target_y,
                  MAX_KEY_POINTS, xpath, ypath);
        if(key_point_num < 2){
            OrdinalNum = 99;
            return;
        }
        path_index = 1;
        path_ready = 1;
    }

    /* Execute one obstacle-safe straight segment on each state-machine pass. */
    if(path_index < key_point_num){
        if(!translation(xpath[path_index - 1], ypath[path_index - 1],
                        xpath[path_index], ypath[path_index])){
            OrdinalNum = 99;
            return;
        }

        route_current_x = xpath[path_index];
        route_current_y = ypath[path_index];
        path_index++;
    }

    if(path_index >= key_point_num){
        path_ready = 0;
        path_index = 0;
        if(OrdinalNum == 80) OrdinalNum = 90;
        else                 OrdinalNum += 10;
    }
}

int main(void)
{
    LED_Init();
    OLED_Init();
    Key_Init();
    Serial_Init();
	USART2_Init(115200);
    Delay_ms(200);

//	while (1)
//	{
//		USART2_SendByte('A');        // 0x41
//		USART2_SendByte('B');        // 0x42
//		USART2_SendByte('\r');
//		USART2_SendByte('\n');
//		Delay_ms(500);
//		PC13_Turn();
//	}


//	uint16_t print_cnt = 0;   // ��ӡ��Ƶ����

//	USART2_SendByte(0x33);
//
//	while (1)
//	{
//		uint8_t key = 0;

//		/* ÿԼ 500ms ͨ�� USART2 ��ӡһ��״̬ */
//		if (++print_cnt >= 500)
//		{
//			/*
//				int32_t pos_now    = 0;   // �����ǰλ�ã��Ӵ��ڶ��أ�
//				int32_t pos_memo   = 0;   // ��ס��λ��
//				uint8_t pos_dir    = 0;   // ����λ�ݴ�
//			*/
//			print_cnt = 0;

//			USART2_SendString(" now=");
//			USART2_SendInt(pos_now);
//			USART2_SendString(" memo=");
//			USART2_SendInt(pos_memo);
//			USART2_SendString(" dir=");
//			USART2_SendDec(pos_dir);
//			USART2_SendString("\r\n");
//		}
//		Delay_ms(1);
//

//		/* ����2ģ�ⰴ�� */
//		if (uart_key_num != 0)
//		{
//			key = uart_key_num;
//			uart_key_num = 0;              // ���ѵ�
//		}

//		if (key != 0)
//		{
//			if (key == 1)
//			{
//				motor_read_pos();       // �ڲ����� pos_now �� pos_dir
//				pos_memo = pos_now;     // �� ���䵱ǰλ��
//			}
//			else if (key == 2)
//			{
//				motor_set_pos(pos_memo);
//			}
//			else if (key == 3)
//			{
//				pos_memo += 160;         // ���ڼ���λ��΢��
//			}
//			else if (key == 4)
//			{
//				pos_memo -= 160;
//			}
//		}
//	}
	
	
//	u8 key_num=0;
//	u32 now_compsition=0;
//	uint8_t cmd[13] = {0x00, 0xFD, 0x00, 0x11, 0x94, 0xc8,0x01, 
//						now_compsition>>24,now_compsition>>16, now_compsition>>8, now_compsition, 0x00, 0x6B};
//	
//	while(1) {
//		uint8_t key_num = Key_GetNum();
//		if (key_num != 0) {
//			if(key_num==1){
//				
//				Serial_SendByte(0x01); Serial_SendByte(0x36); Serial_SendByte(0x6B);
//				while(!uart_get){
//					now_compsition=compsition_buffer[0]<<24||compsition_buffer[1]<<16||\
//					compsition_buffer[0]<<8||compsition_buffer[0];
//					continue;
//				}
//			}else if(key_num==2){
//				Serial_SendArray(cmd, 13);
//			}else if(key_num==3){
//				now_compsition+=1000;
//			}
//		}
//		OLED_ShowNum(1, 1, key_num, 1);
//		OLED_ShowHexNum(2, 1, now_compsition, 1);
//	}
	
//	u16 pulse_600mm=12658u;
//	uint8_t last_key = 0;
//	while(1) {
//		uint8_t key_num = Key_GetNum();
//		if (key_num != 0) {
//			last_key = key_num;
//			if(key_num==1){
//				pulse_600mm+=10;
//			}else if(key_num==2){
//				pulse_600mm-=10;	
//			}else if(key_num==3){
//				uint32_t p = 600 * pulse_600mm / 1000u;   /* ������ */
//				uint8_t cmd[13] = {0x00, 0xFD, 0x00, MOTOR_SPEED_H, MOTOR_SPEED_L, (uint8_t)MOTOR_ACCEL,
//									0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x6B};

//				cmd[6] = (uint8_t)(p >> 24);
//				cmd[7] = (uint8_t)(p >> 16);
//				cmd[8] = (uint8_t)(p >> 8);
//				cmd[9] = (uint8_t)p;

//				cmd[0] = 0x01; cmd[2] = 0x00;   /* FL ��ת */
//				Serial_SendArray(cmd, 13); Delay_ms(5);
//				cmd[0] = 0x02; cmd[2] = 0x01;   /* FR ��ת������ */
//				Serial_SendArray(cmd, 13); Delay_ms(5);
//				cmd[0] = 0x03; cmd[2] = 0x00;   /* RL ��ת */
//				Serial_SendArray(cmd, 13); Delay_ms(5);
//				cmd[0] = 0x04; cmd[2] = 0x01;   /* RR ��ת������ */
//				Serial_SendArray(cmd, 13); Delay_ms(5);

//				/* ͬ���ͷţ�ֱ�� sync=0x01����֡�� 4 �����ͬʱ�� */
//				Serial_SendByte(0x00); Serial_SendByte(0xFF); Serial_SendByte(0x66); Serial_SendByte(0x6B);
//				Delay_ms(5);
//			}
//		}
//		OLED_ShowNum(4, 1, last_key, 1);
//		OLED_ShowNum(4, 3, key_num, 1);
//		OLED_ShowNum(3, 1, pulse_600mm, 5);
//	}

	
    Delay_ms(500);
    while(1){
        Mytest(20);
        Delay_ms(20);
    }
}
