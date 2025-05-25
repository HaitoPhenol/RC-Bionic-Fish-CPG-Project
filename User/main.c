#include "stm32f10x.h"
#include "stm32f10x_tim.h" // Add this for TIM2_IRQn and timer definitions
#include "OLED.h"
#include "Delay.h"
#include "misc.h" // For NVIC and IRQn_Type definitions
#include "stm32f10x_it.h" // Ensure IRQn definitions like TIM2_IRQn are available

#define BEEP_GPIO_PORT GPIOB
#define BEEP_GPIO_PIN  GPIO_Pin_0

volatile uint32_t seconds = 0;
volatile uint32_t count = 0;
volatile uint8_t beep_flag = 0;

void Beep_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = BEEP_GPIO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BEEP_GPIO_PORT, &gpio);
    GPIO_SetBits(BEEP_GPIO_PORT, BEEP_GPIO_PIN); // 默认高电平关闭
}

void Beep_On(void) {
    GPIO_ResetBits(BEEP_GPIO_PORT, BEEP_GPIO_PIN); // 低电平响
}

void Beep_Off(void) {
    GPIO_SetBits(BEEP_GPIO_PORT, BEEP_GPIO_PIN);
}

void Timer2_Init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseInitTypeDef tim;
    tim.TIM_Period = 9999; // 1秒定时 (72MHz/7200/10000=1Hz)
    tim.TIM_Prescaler = 7199;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tim);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    TIM_Cmd(TIM2, ENABLE);
}

void ShowTime(uint32_t sec) {
    uint32_t min = sec / 60;
    uint32_t s = sec % 60;
    OLED_ShowString(1, 1, "Time:");
    OLED_ShowNum(1, 6, min, 2);
    OLED_ShowChar(1, 8, ':');
    OLED_ShowNum(1, 9, s, 2);
}

void ShowCount(uint32_t cnt) {
    OLED_ShowString(2, 1, "Count:");
    OLED_ShowNum(2, 7, cnt, 3);
}

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        seconds++;
        if (seconds % 300 == 0) { // 每5分钟
            beep_flag = 1;
            count++;
        }
    }
}

int main(void) {
    OLED_Init();
    Beep_Init();
    Timer2_Init();
    OLED_Clear();
    ShowTime(0);
    ShowCount(0);

    while (1) {
        static uint32_t last_sec = 0;
        static uint32_t last_count = 0;
        if (last_sec != seconds) {
            last_sec = seconds;
            ShowTime(seconds);
        }
        if (last_count != count) {
            last_count = count;
            ShowCount(count);
        }
        if (beep_flag) {
            Beep_On();
            Delay_ms(800);
            Beep_Off();
            beep_flag = 0;
        }
        // ...existing code...
    }
}