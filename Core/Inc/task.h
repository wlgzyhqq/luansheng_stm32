#ifndef __TASK_H
#define __TASK_H

#include "main.h"

/* 应用层初始化，主函数中 MX_xxx_Init() 之后调用一次 */
void INIT_ALL(void);

/* 50Hz 任务 */
void Task_Uart1(void);      /* 串口 1 命令解析 */
void Task_Process(void);    /* 工艺流程状态机 */

/* 硬件控制接口 */
void Motor_Control(uint8_t motor, uint8_t status, uint8_t duty);
void Stepper_SetSpeed(int32_t speed);
void Stepper_Rotate(uint16_t angle, int32_t speed);
void Pump_Control(uint8_t enable);

#endif
