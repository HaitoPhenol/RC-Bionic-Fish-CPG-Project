#ifndef __MAIN_TASK_H__
#define __MAIN_TASK_H__

#include "main.h"
#include "stm32f1xx_hal.h"
#include "usart.h"

#include "oled_kk_simple.h"
#include "cpg_control.h"
#include "servo.h"
#include "SBUS.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

void mainTaskInit(void);
void mainTask(void);

#endif
