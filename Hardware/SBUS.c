#include "SBUS.h"  
#include "stm32f10x.h"

/*
感谢 LGQWakkk 的开源仓库 https://github.com/LGQWakkk/STM32_SBUS_RX
注意一定要使用三极管硬件信号取反，还有接收机三极管单片机共地之类的问题

1. 三极管基极（B）的连接
接收器的 SBUS 信号线（黄 / 白色）→ 1kΩ 电阻的一端；
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

uint8_t USART_RX_BUF[USART_REC_LEN]; 
uint8_t uart_rx_num=0;
uint8_t startbyte_detected=0;
uint8_t uart_sbus_ok=0;

void uart_init(uint32_t bound){
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	// //USART1_TX   GPIOA.9
    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 				// PA.9
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			// 复用推挽输出
    // GPIO_Init(GPIOA, &GPIO_InitStructure);					// 初始化GPIOA.9
   
    //USART1_RX	  GPIOA.10初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				// PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	// 浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);					// 初始化GPIOA.10  

    //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		// 这里VSCode报错不用在意，编译没错就行
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;// 抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		// 子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			// IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							// 根据指定的参数初始化VIC寄存器
  
    //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// 数据位8bit
	USART_InitStructure.USART_StopBits = USART_StopBits_2;							// 2个停止位
	USART_InitStructure.USART_Parity = USART_Parity_Even;							// 偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	// 无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure); 				// 初始化串口1
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);			// 开启串口接受中断
    USART_Cmd(USART1, ENABLE);                    			// 使能串口1 

}

void USART1_IRQHandler(void)
{
	uint8_t Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)	//RXNE:数据已经完全移入接收数据寄存器，可以读出。
	{
		Res =USART_ReceiveData(USART1);						//读取接收到的数据
		if(startbyte_detected==0&&Res==0x0f){
			startbyte_detected=1;
			uart_rx_num=0;
			USART_RX_BUF[uart_rx_num]=Res;
			uart_rx_num++;
		}else if(startbyte_detected==1&&(uart_rx_num<(SBUS_MAX_NUM-1))){
			USART_RX_BUF[uart_rx_num]=Res;
			uart_rx_num++;
		}else if(startbyte_detected==1&&(uart_rx_num==(SBUS_MAX_NUM-1))){
			uart_sbus_ok=1;
			uart_rx_num=0;
		}else{
			uart_rx_num=0;
			uart_sbus_ok=0;
		}  		 
	} 
} 

//数据解析函数
//将串口接收缓冲区的数据读取并转换到通道形式
//通道值数组uint16_t chbuf[6];
void Save_6CH(uint16_t* chbuf)
{	//CH1
	uint16_t tmp=0;
	tmp=(USART_RX_BUF[2]&0x07)<<8;
	tmp=tmp|USART_RX_BUF[1];
	chbuf[0]=tmp;
	//CH2
	tmp=0;
	tmp=(USART_RX_BUF[3]&0x3f)<<5;
	tmp=tmp|(USART_RX_BUF[2]>>3);
	chbuf[1]=tmp;
	//CH3
	tmp=0;
	tmp=(USART_RX_BUF[5]&0x01)<<10;
	tmp=tmp|(USART_RX_BUF[4]<<2);
	tmp=tmp|(USART_RX_BUF[3]>>6);
	chbuf[2]=tmp;
	//CH4
	tmp=0;
	tmp=(USART_RX_BUF[6]&0x0f)<<7;
	tmp=tmp|(USART_RX_BUF[5]>>1);
	chbuf[3]=tmp;
	//CH5
	tmp=0;
	tmp=(USART_RX_BUF[7]&0x7f)<<4;
	tmp=tmp|(USART_RX_BUF[6]>>4);
	chbuf[4]=tmp;
	//CH6
	tmp=0;
	tmp=(USART_RX_BUF[9]&0x03)<<9;
	tmp=tmp|(USART_RX_BUF[8]<<1);
	tmp=tmp|(USART_RX_BUF[7]>>7);
	chbuf[5]=tmp;
}

//清空串口缓冲区
void rxbuf_clear(void)
{
	uint8_t i=0;
	for(i=0;i<25;i++){
		USART_RX_BUF[i]=0;
	}
}
