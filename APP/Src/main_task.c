#include "main_task.h"
#include "servo.h"



uint16_t channel_buf[6]={0}; // 6个通道数据存储数组
CPGState state;
double dt = 0.01; // 时间步长
SERVO_ID servo_ids[3] = {SERVO_1, SERVO_2, SERVO_3};

void mainTaskInit(void)
{
    Servo_Init();
    OLED_kk_Init();
    initCPGState(&state);
    HAL_Delay(20);

    // 添加测试代码
    OLED_PrintASCIIString(1, 1, "System Init...", &afont8x6, OLED_COLOR_NORMAL);
    OLED_ShowFrame();
    OLED_NewFrame();

    // 启动USART1中断接收
    uint8_t dummyByte;
    HAL_UART_Receive_IT(&huart1, &dummyByte, 1);
}

void mainTask(void)
{
    OLED_NewFrame();
    if(sbus_data_ready){            // 数据就绪可以读取
        sbus_data_ready = 0;
        Save_6CH(channel_buf);      // 使用Save_6CH函数向channel_buf写入通道数据
        rxbuf_clear();              // 清空串口缓冲区

    
        // 更新CPG状态
        set_cpg_frequency(&state, channel_buf[2]);    // 每次循环都设置频率
        set_cpg_bias(&state, channel_buf[3]);
        // CPG更新舵机角度数据
        cpg_update(&state, dt);

        // 获取舵机角度数据
        double servo_angles[3];
        for (int i = 0; i < 3; i++) {
            servo_angles[i] = map_to_servo(state.y[i]);
        }

        // 显示舵机角度（取整显示），并设置舵机角度
        for (int i = 0; i < 3; i++) {
            // 舵机角度数据
            char servo_label[5];
            sprintf(servo_label, "S%d:", i + 1);
            OLED_PrintASCIIString(1, i + 1, servo_label, &afont8x6, OLED_COLOR_NORMAL);
            OLED_PrintFloat(4, i + 1, servo_angles[i], 0, &afont8x6, OLED_COLOR_NORMAL);
            printf("Servo ID: %d\n", servo_ids[i]);
            Servo_SetAngle(servo_ids[i], servo_angles[i]);             //分别设置3个舵机
        }
        Servo_SetAngle(SERVO_4, map_to_range(55, 145, channel_buf[1]));     // 单独设置侧鳍舵机
        // 速度、偏置、侧鳍 数据
        OLED_PrintASCIIString(8, 1, "a:", &afont8x6, OLED_COLOR_NORMAL);
        OLED_PrintFloat(10, 1, state.omega[0], 1, &afont8x6, OLED_COLOR_NORMAL);            // 显示浮点数，保留1位小数
        OLED_PrintASCIIString(8, 2, "b:", &afont8x6, OLED_COLOR_NORMAL);
        OLED_PrintFloat(10, 2, state.bias[0], 1, &afont8x6, OLED_COLOR_NORMAL);             // 显示浮点数，保留1位小数
        OLED_PrintASCIIString(8, 3, "c:", &afont8x6, OLED_COLOR_NORMAL);
        OLED_PrintFloat(10, 3, map_to_range(55, 145, channel_buf[1]), 1, &afont8x6, OLED_COLOR_NORMAL);           // 转换为浮点数显示
    } else {
        // OLED_PrintASCIIString(1, 1, "SBUS did not ready..", &afont8x6, OLED_COLOR_NORMAL);
    }
    OLED_ShowFrame(); // 显示帧
}
