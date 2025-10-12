#ifndef _SBUS_H
#define _SBUS_H

#include <stdint.h>
#include "main.h"
#include "oled_kk_simple.h"
#include "usart.h"

#define USART_REC_LEN  			200  	// 定义最大接收字节数 200
#define EN_USART1_RX 			1		// 使能（1）/禁止（0）串口1接收
#define SBUS_MAX_NUM 			25  	// SBUS每次传输最大25字节

extern uint8_t  sbus_rx_buffer[USART_REC_LEN];  // 接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
// extern uint16_t USART_RX_STA;                // 接收状态标记	
extern uint8_t sbus_data_ready;                 // SBUS数据就绪(单次接收完毕) 全局变量

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Save_6CH(uint16_t* channel_buf);
void rxbuf_clear(void);


#endif

