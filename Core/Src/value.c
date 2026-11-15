#include "value.h"

/* ==================== 电机参数 ==================== */

/* 直流电机占空比 % */
uint8_t motor_speeds[MOTOR_NUM] = {40, 50, 50};

/* 步进电机速度：1 号=脉冲频率(Hz)，2/3 号=闭环转速(RPM) */
int32_t stepper_speeds[STEPPER_NUM] = {2000, 100, 120};

/* 步进电机停顿时间 ms */
uint32_t stepper_times[STEPPER_NUM] = {500, 1000, 1000};

/* 直流电机 1 分段运输时长 ms */
uint32_t dc1_transport_time[4] = {8000, 8500, 8000, 8000};
uint32_t dc1_loop_time = 9000;
uint8_t  dc1_time_index = 0;

/* ==================== 流程状态 ==================== */

uint16_t piece_count = 0;
uint8_t  one_key_start_flag = 0;
uint8_t  pump_enable = 0;
uint8_t  ext_num = 0;
uint8_t  ext_loop_mode = 0;
uint8_t  step2_auto_run = 0;
uint32_t step2_auto_tick = 0;
uint8_t  step2_auto_state = 0;
uint8_t  motor3_started_by_step2 = 0;

Step2State step2_state = STEP2_IDLE;
uint32_t   step2_tick = 0;
Step3State step3_state = STEP3_IDLE;
uint32_t   step3_tick = 0;
ExtState   ext_state = EXT_IDLE;
uint32_t   ext_tick = 0;

/* 1 号脉冲步进状态 */
uint32_t          stepper_target_steps = 0;
volatile uint32_t stepper_current_steps = 0;
volatile uint8_t  stepper_running = 0;
volatile uint8_t  stepper_finished = 0;

/* ==================== 硬件配置表 ==================== */

MotorCfg motor_cfg[MOTOR_NUM] =
{
  [Motor1] = { MOTOR1_IN1_PORT, MOTOR1_IN1_PIN, MOTOR1_IN2_PORT, MOTOR1_IN2_PIN,
               MOTOR1_PWM_TIM, MOTOR1_PWM_CH },
  [Motor2] = { MOTOR2_IN1_PORT, MOTOR2_IN1_PIN, MOTOR2_IN2_PORT, MOTOR2_IN2_PIN,
               MOTOR2_PWM_TIM, MOTOR2_PWM_CH },
  [Motor3] = { MOTOR3_IN1_PORT, MOTOR3_IN1_PIN, MOTOR3_IN2_PORT, MOTOR3_IN2_PIN,
               MOTOR3_PWM_TIM, MOTOR3_PWM_CH },
};
