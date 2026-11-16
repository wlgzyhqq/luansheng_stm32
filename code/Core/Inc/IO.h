#ifndef __IO_H
#define __IO_H

/*
 * 硬件映射表(逻辑功能 -> 实际引脚/定时器通道)
 * 上层代码只使用本文件的宏，改接线 / 换引脚时只改这里。
 * 端口与引脚号来自 main.h(CubeMX 生成)，定时器来自 tim.h。
 */

#include "main.h"
#include "tim.h"
#include "usart.h"

/* ---------------- 直流电机 1 ---------------- */
#define MOTOR1_IN1_PORT     Motor1_IN1_GPIO_Port        /* PE7  */
#define MOTOR1_IN1_PIN      Motor1_IN1_Pin
#define MOTOR1_IN2_PORT     ((GPIO_TypeDef *)0)         /* 无 IN2 引脚 */
#define MOTOR1_IN2_PIN      0
#define MOTOR1_PWM_TIM      (&htim2)
#define MOTOR1_PWM_CH       TIM_CHANNEL_2               /* PB3  */

/* ---------------- 直流电机 2 ---------------- */
#define MOTOR2_IN1_PORT     Motor2_IN1_GPIO_Port        /* PE15 */
#define MOTOR2_IN1_PIN      Motor2_IN1_Pin
#define MOTOR2_IN2_PORT     Motor2_IN2_GPIO_Port        /* PE8  */
#define MOTOR2_IN2_PIN      Motor2_IN2_Pin
#define MOTOR2_PWM_TIM      (&htim2)
#define MOTOR2_PWM_CH       TIM_CHANNEL_3               /* PB10 */

/* ---------------- 直流电机 3 ---------------- */
#define MOTOR3_IN1_PORT     Motor3_IN1_GPIO_Port        /* PF4  */
#define MOTOR3_IN1_PIN      Motor3_IN1_Pin
#define MOTOR3_IN2_PORT     Motor3_IN2_GPIO_Port        /* PD13 */
#define MOTOR3_IN2_PIN      Motor3_IN2_Pin
#define MOTOR3_PWM_TIM      (&htim2)
#define MOTOR3_PWM_CH       TIM_CHANNEL_4               /* PB11 */

/* ---------------- 步进电机 1: 脉冲输出(定时器 PWM) ---------------- */
#define STEPPER1_STEP_TIM   (&htim5)
#define STEPPER1_STEP_CH    TIM_CHANNEL_2               /* PA1  */
#define STEPPER1_DIR_PORT   Stepper1_DIR_GPIO_Port      /* PF5  */
#define STEPPER1_DIR_PIN    Stepper1_DIR_Pin

/*
 * 步进电机 2/3 为闭环步进，只接串口 2(见 zdt.h)，没有 STEP/DIR 引脚。
 * 脉冲计数用该定时器的更新中断，换定时器时 task.c 里的 TIMx_IRQHandler 和 NVIC 也要一起改。
 */

/* ---------------- 继电器 ---------------- */
#define RELAY1_PORT         Relay1_GPIO_Port            /* PB8  风机 */
#define RELAY1_PIN          Relay1_Pin
#define RELAY2_PORT         Relay2_GPIO_Port            /* PG9  水泵 */
#define RELAY2_PIN          Relay2_Pin
#define RELAY3_PORT         Relay3_GPIO_Port            /* PA15 */
#define RELAY3_PIN          Relay3_Pin
#define RELAY4_PORT         Relay4_GPIO_Port            /* PC3  封膜 */
#define RELAY4_PIN          Relay4_Pin

/* ---------------- 指示灯 / 辅助输出 ---------------- */
#define LED_PORT            LED_GPIO_Port               /* PE5  */
#define LED_PIN             LED_Pin
#define OUT_AUX1_PORT       GPIOG                       /* PG6  功能待确认 */
#define OUT_AUX1_PIN        GPIO_PIN_6
#define OUT_AUX2_PORT       GPIOG                       /* PG8  功能待确认 */
#define OUT_AUX2_PIN        GPIO_PIN_8

#endif
