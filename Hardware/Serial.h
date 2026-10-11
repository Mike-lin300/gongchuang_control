#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>

/* USART1_IRQHandler updates the receive buffer and flag asynchronously. */
extern uint8_t Serial_TxPacket[];
extern volatile uint8_t Serial_RxPacket[];
extern volatile uint8_t Serial_RxFlag;

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

void Serial_SendPacket(void);
uint8_t Serial_GetRxFlag(void);

void USART2_SendByte(uint8_t byte);
void USART2_Init(uint32_t bound);
/* Serial2.h */
extern volatile uint8_t uart_key_num;
void USART2_SendString(char *s);
void USART2_SendHex32(uint32_t val);
void USART2_SendDec(uint32_t val);
void USART2_SendInt(int32_t v);

/* USART3：陀螺仪角度文本输出，使用默认映射 PB10(TX)/PB11(RX)。 */
void USART3_Init(uint32_t bound);
void USART3_SendByte(uint8_t byte);
void USART3_SendAngleCentiDegrees(int32_t centi_degrees);


#endif
