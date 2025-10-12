#include "SBUS.h"  

/*
感谢 LGQWakkk 的开源仓库 https://github.com/LGQWakkk/STM32_SBUS_RX
注意一定要使用三极管硬件信号取反，还有接收机三极管单片机共地之类的问题

1. 三极管基极（B）的连接
接收机 RF209S 的 SBUS 信号线（黄 / 白色）→ 1kΩ 电阻的一端；
1kΩ 电阻的另一端 → SS8050 三极管的基极（B）（中间引脚）。

2. 三极管发射极（E）的连接
SS8050 的发射极（E）（左侧引脚）→ GND 线；
GND 线 同时连接：
接收器的 GND（黑色线）；
F103C8T6 的 GND 引脚（如 PA14 或 PB3 旁的 GND 引脚）。

3. 三极管集电极（C）的连接
SS8050 的集电极（C）（右侧引脚）→ 两根线：
第一根线 → 10kΩ 电阻的一端 → F103C8T6 的 3.3V 引脚（如 PA13 旁的 3.3V 引脚）；
第二根线 → F103C8T6 的 PA10 引脚（USART1_RX）。
*/

uint8_t sbus_rx_buffer[USART_REC_LEN]; 
uint8_t sbus_rx_counter = 0;
uint8_t sbus_start_byte_detected = 0;
uint8_t sbus_data_ready = 0 ;
uint8_t dummyByte = 0; // 用于接收中断的dummy字节，但实际上根本用不上它

// 优化后的回调函数示例
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance == USART1) {
        uint8_t Res = (uint8_t)(huart->Instance->DR & 0x00FF);
        
        // 帧同步与数据接收状态机
        if(sbus_start_byte_detected == 0) {
            // 等待起始字节
            if(Res == 0x0F) {
                sbus_start_byte_detected = 1;
                sbus_rx_counter = 0;
                sbus_rx_buffer[sbus_rx_counter++] = Res;
            }
        } else {
            // 接收数据字节
            if(sbus_rx_counter < SBUS_MAX_NUM) {
                sbus_rx_buffer[sbus_rx_counter++] = Res;
                
                // 检查是否接收完一帧数据
                if(sbus_rx_counter == SBUS_MAX_NUM) {
                    // 可以在这里增加CRC校验或其他数据有效性检查
                    sbus_data_ready = 1;
                    sbus_start_byte_detected = 0; // 准备接收下一帧
                }
            } else {
                // 接收溢出，重置状态
                sbus_start_byte_detected = 0;
                sbus_rx_counter = 0;
            }
        }
        
        // 立即重启接收中断
        HAL_UART_Receive_IT(&huart1, &dummyByte, 1);
    }
}

//数据解析函数
//将串口接收缓冲区的数据读取并转换到通道形式
//通道值数组uint16_t channel_buf[6];
void Save_6CH(uint16_t* channel_buf)
{	
	// 检查数据帧的有效性（检查结束字节）
    if (sbus_rx_buffer[SBUS_MAX_NUM - 1] != 0x00 && sbus_rx_buffer[SBUS_MAX_NUM - 1] != 0x04) {
        // 数据帧无效，清空通道数据
        for (uint8_t i = 0; i < 6; i++) {
            channel_buf[i] = 0;
        }
        return;
    }
	//CH1
	uint16_t tmp = 0;
	tmp=(sbus_rx_buffer[2]&0x07)<<8;
	tmp=tmp|sbus_rx_buffer[1];
	channel_buf[0]=tmp;
	//CH2
	tmp=0;
	tmp=(sbus_rx_buffer[3]&0x3f)<<5;
	tmp=tmp|(sbus_rx_buffer[2]>>3);
	channel_buf[1]=tmp;
	//CH3
	tmp=0;
	tmp=(sbus_rx_buffer[5]&0x01)<<10;
	tmp=tmp|(sbus_rx_buffer[4]<<2);
	tmp=tmp|(sbus_rx_buffer[3]>>6);
	channel_buf[2]=tmp;
	//CH4
	tmp=0;
	tmp=(sbus_rx_buffer[6]&0x0f)<<7;
	tmp=tmp|(sbus_rx_buffer[5]>>1);
	channel_buf[3]=tmp;
	//CH5
	tmp=0;
	tmp=(sbus_rx_buffer[7]&0x7f)<<4;
	tmp=tmp|(sbus_rx_buffer[6]>>4);
	channel_buf[4]=tmp;
	//CH6
	tmp=0;
	tmp=(sbus_rx_buffer[9]&0x03)<<9;
	tmp=tmp|(sbus_rx_buffer[8]<<1);
	tmp=tmp|(sbus_rx_buffer[7]>>7);
	channel_buf[5]=tmp;
}

//清空串口缓冲区
void rxbuf_clear(void)
{
    uint8_t i = 0;
    for(i = 0; i < SBUS_MAX_NUM; i++){
        sbus_rx_buffer[i] = 0;
    }
}
