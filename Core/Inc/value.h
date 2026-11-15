#ifndef __VALUE_H
#define __VALUE_H

#include "main.h"
#include "IO.h"

/* ==================== 宏定义 ==================== */

/* 任务周期 20ms -> 50Hz */
#define TASK_PERIOD_MS      20

/* 直流电机 */
#define Motor_STOP          0
#define Motor_FORWARD       1
#define Motor_REVERSE       2

#define Motor1              0
#define Motor2              1
#define Motor3              2
#define MOTOR_NUM           3
#define MAX_DUTY_CYCLE      100         /* 占空比 0~100 % */

/* 步进电机: 1 号脉冲输出, 2/3 号闭环(串口 2, 地址 2/3) */
#define Stepper1            0
#define Stepper2            1
#define Stepper3            2
#define STEPPER_NUM         3
#define STEP_PER_REV        800         /* 整步数/圈 */
#define STEPPER_MAX_SPEED   100000      /* 最高脉冲频率 Hz */
#define STEPPER1_RATIO      12          /* 1 号步进减速比 */
#define TIM_CLK             72000000    /* 定时器计数时钟 Hz */
#define STEPPER_TIM_PSC     71          /* 脉冲步进定时器预分频, 计数时钟 1MHz */

/* 张大头闭环步进 */
#define ZDT_ADDR2           2
#define ZDT_ADDR3           3
#define ZDT_PULSE_PER_REV   3200        /* 16 细分下 3200 脉冲/圈 */

/* 屏幕预设重量(原版写死的数据) */
#define WEIGHT_PRESET_NUM   3

/* 时序 (ms) */
#define TIME_START_DELAY    8000        /* 一键启动后等待 */
#define TIME_STEP2_FIRST    750         /* 2 号步进第一圈 */
#define TIME_STEP2_M3       1000        /* 启动 3 号直流电机 */
#define TIME_STEP2_SECOND   13000       /* 2 号步进第二圈前等待 */
#define TIME_STEP2_LAST     750         /* 2 号步进第二圈 */
#define TIME_STEP2_AUTO     12000       /* 2 号步进自动循环间隔 */
#define TIME_STEP2_AUTO_RST 500
#define TIME_STEP3_FIRST    600         /* 3 号步进前半圈 */
#define TIME_STEP3_WAIT     1000        /* 3 号步进中间等待 */
#define TIME_STEP3_SECOND   600         /* 3 号步进后半圈 */
#define TIME_TO_STEP3       3000        /* 2 号步进放料后, 料到称重工位的时间, 按现场调 */

/* ==================== 类型定义 ==================== */

/* 直流电机配置 */
typedef struct
{
  GPIO_TypeDef      *in1_port;
  uint16_t           in1_pin;
  GPIO_TypeDef      *in2_port;
  uint16_t           in2_pin;
  TIM_HandleTypeDef *htim;
  uint32_t           channel;
} MotorCfg;

/* 2 号闭环步进流程 */
typedef enum
{
  STEP2_IDLE,
  STEP2_START_DELAY,        /* 一键启动后的延时 */
  STEP2_RUNNING_FIRST,      /* 第一圈转动中 */
  STEP2_WAIT_FOR_SECOND,    /* 等待第二圈 */
  STEP2_RUNNING_SECOND,     /* 第二圈转动中 */
  STEP2_DONE
} Step2State;

/* 3 号闭环步进流程 */
typedef enum
{
  STEP3_IDLE,
  STEP3_RUNNING_FIRST,      /* 前半圈转动中 */
  STEP3_WAIT_BETWEEN,       /* 中间等待 */
  STEP3_RUNNING_SECOND,     /* 后半圈转动中 */
  STEP3_DONE
} Step3State;

/* 封膜流程 */
typedef enum
{
  EXT_IDLE,
  EXT_START_DC1,            /* 启动直流电机 1 */
  EXT_DELAY_DC1,            /* 运输计时 */
  EXT_STEP1_FIRST,          /* 步进 1 第一次 180° */
  EXT_WAIT_FIRST,           /* 等待第一次转完 */
  EXT_DELAY_STEP1,          /* 两次转动之间停顿 */
  EXT_STEP1_SECOND,         /* 步进 1 第二次 180° */
  EXT_WAIT_SECOND           /* 等待第二次转完 */
} ExtState;

/* ==================== 全局变量 ==================== */

/* 电机参数 */
extern uint8_t  motor_speeds[MOTOR_NUM];        /* 直流电机占空比 % */
extern int32_t  stepper_speeds[STEPPER_NUM];    /* 步进电机速度 */
extern uint32_t stepper_times[STEPPER_NUM];     /* 步进电机停顿时长 */
extern uint32_t dc1_transport_time[4];          /* 电机 1 分段运输时长 */
extern uint32_t dc1_loop_time;                  /* 循环模式运输时长 */
extern uint8_t  dc1_time_index;                 /* 当前运输段 */

/* 流程状态 */
extern uint16_t   piece_count;
extern uint8_t    one_key_start_flag;
extern uint8_t    pump_enable;
extern uint8_t    ext_num;
extern uint8_t    ext_loop_mode;
extern uint8_t    step2_auto_run;
extern uint32_t   step2_auto_tick;
extern uint8_t    step2_auto_state;
extern uint8_t    motor3_started_by_step2;

extern Step2State step2_state;
extern uint32_t   step2_tick;
extern Step3State step3_state;
extern uint32_t   step3_tick;
extern ExtState   ext_state;
extern uint32_t   ext_tick;

/* 1 号脉冲步进状态(中断与流程共用) */
extern uint32_t          stepper_target_steps;
extern volatile uint32_t stepper_current_steps;
extern volatile uint8_t  stepper_running;
extern volatile uint8_t  stepper_finished;

/* 硬件配置表 */
extern MotorCfg motor_cfg[MOTOR_NUM];

#endif
