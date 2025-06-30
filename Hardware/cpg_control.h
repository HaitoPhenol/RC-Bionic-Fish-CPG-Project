#ifndef CPG_CONTROL_H
#define CPG_CONTROL_H

#include <stdint.h>
#include <math.h>

typedef struct {
    double x[6]; 
    double t;  
} CPG_State;

typedef struct
{
  uint16_t GPIO_Pin;
  //GPIOSpeed_TypeDef GPIO_Speed;
  //GPIOMode_TypeDef GPIO_Mode;
}TIM_HandleTypeDef;

extern double t1, t2, t3, t4, t5, t6, t7;

void CPG_Init(CPG_State *state);
void CPG_Update(CPG_State *state, double dt);
void CPG_GetOutput(const CPG_State *state, double *left_fin, double *right_fin, double *tail);

#endif
