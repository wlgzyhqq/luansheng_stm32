#include "task.h"
#include "value.h"
#include "zdt.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ==================== 内部函数声明 ==================== */
static void Motor_Init(void);
static void Motor_SetDir(uint8_t motor, uint8_t status);
static void Motor_Stop_All(void);
static void Uart1_Send(const char *str);
static void Encoder_Init(void);
static void Encoder_Update(void);
static void Step3_Arm(void);
static void Step3_Start(void);
static void Process_OneKey(void);
static void Process_Step2(void);
static void Process_Step3(void);
static void Process_Step2Auto(void);
static void Process_Ext(void);

/* 放料后等待料到达称重工位 */
static uint8_t  step3_pending = 0;
static uint32_t step3_pending_tick = 0;

/* 编码器上次的计数值(用于算增量并处理 16 位回绕) */
static uint16_t encoder_last1 = 0;
static uint16_t encoder_last2 = 0;
static uint16_t encoder_last3 = 0;

/* ==================== 硬件控制 ==================== */

/* 串口 1(屏幕)发送字符串 */
static void Uart1_Send(const char *str)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
}

/* 直流电机方向 */
static void Motor_SetDir(uint8_t motor, uint8_t status)
{
  const MotorCfg *m = &motor_cfg[motor];

  switch (status)
  {
    case Motor_FORWARD:
      HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
      if (m->in2_port != NULL) HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
      break;

    case Motor_REVERSE:
      HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
      if (m->in2_port != NULL) HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
      break;

    default:   /* Motor_STOP */
      HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
      if (m->in2_port != NULL) HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
      break;
  }
}

/* 直流电机控制: status = Motor_STOP/FORWARD/REVERSE, duty = 0~100 % */
void Motor_Control(uint8_t motor, uint8_t status, uint8_t duty)
{
  const MotorCfg *m = &motor_cfg[motor];
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(m->htim) + 1;

  Motor_SetDir(motor, status);

  if (status == Motor_STOP || duty == 0)
  {
    __HAL_TIM_SET_COMPARE(m->htim, m->channel, 0);
    return;
  }

  if (duty > MAX_DUTY_CYCLE) duty = MAX_DUTY_CYCLE;
  __HAL_TIM_SET_COMPARE(m->htim, m->channel, arr * duty / MAX_DUTY_CYCLE);
}

/* 1 号脉冲步进速度: 正数正转，负数反转，0 停止 */
void Stepper_SetSpeed(int32_t speed)
{
  uint32_t cnt_clk, arr;

  if (speed > 0)
  {
    HAL_GPIO_WritePin(STEPPER1_DIR_PORT, STEPPER1_DIR_PIN, GPIO_PIN_SET);
  }
  else if (speed < 0)
  {
    HAL_GPIO_WritePin(STEPPER1_DIR_PORT, STEPPER1_DIR_PIN, GPIO_PIN_RESET);
    speed = -speed;
  }

  if (speed == 0)
  {
    HAL_TIM_PWM_Stop(STEPPER1_STEP_TIM, STEPPER1_STEP_CH);
    HAL_TIM_Base_Stop_IT(STEPPER1_STEP_TIM);
    __HAL_TIM_SET_COUNTER(STEPPER1_STEP_TIM, 0);
    stepper_running = 0;
    return;
  }

  /* 比较值 = 计数频率 / 脉冲频率 */
  cnt_clk = TIM_CLK / (STEPPER1_STEP_TIM->Instance->PSC + 1);
  if (speed > STEPPER_MAX_SPEED) speed = STEPPER_MAX_SPEED;

  arr = cnt_clk / (uint32_t)speed;
  arr = (arr > 65535) ? 65535 : arr;
  arr = (arr > 1) ? (arr - 1) : 1;

  __HAL_TIM_SET_AUTORELOAD(STEPPER1_STEP_TIM, arr);
  __HAL_TIM_SET_COMPARE(STEPPER1_STEP_TIM, STEPPER1_STEP_CH, arr / 2);

  if (!stepper_running)
  {
    HAL_TIM_PWM_Start(STEPPER1_STEP_TIM, STEPPER1_STEP_CH);
    HAL_TIM_Base_Start_IT(STEPPER1_STEP_TIM);
    stepper_running = 1;
  }
}

/* 1 号脉冲步进转动指定角度，转完 stepper_finished 置 1 */
void Stepper_Rotate(uint16_t angle, int32_t speed)
{
  Stepper_SetSpeed(0);

  stepper_target_steps  = (uint32_t)((float)angle * STEP_PER_REV / 360.0f) * STEPPER1_RATIO;
  stepper_current_steps = 0;
  stepper_finished      = 0;

  Stepper_SetSpeed(speed);
}

/* 风机(继电器 1) + 水泵(继电器 2) */
void Pump_Control(uint8_t enable)
{
  GPIO_PinState state = enable ? GPIO_PIN_SET : GPIO_PIN_RESET;

  HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, state);
  HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN, state);
}

/* 直流电机 PWM 启动并全部停止 */
static void Motor_Init(void)
{
  uint8_t i;

  for (i = 0; i < MOTOR_NUM; i++)
  {
    HAL_TIM_PWM_Start(motor_cfg[i].htim, motor_cfg[i].channel);
    Motor_Control(i, Motor_STOP, 0);
  }

  /* 1 号步进电机: 1MHz 计数发脉冲，更新中断用于数脉冲 */
  __HAL_TIM_SET_PRESCALER(STEPPER1_STEP_TIM, STEPPER_TIM_PSC);
  HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  HAL_TIM_PWM_Start(STEPPER1_STEP_TIM, STEPPER1_STEP_CH);
  __HAL_TIM_SET_COMPARE(STEPPER1_STEP_TIM, STEPPER1_STEP_CH, 0);
}

/* ==================== 编码器 ==================== */

/* 启动三个编码器定时器, 并让定时器6 定时中断来采样 */
static void Encoder_Init(void)
{
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);   /* 1 号编码器 */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);   /* 2 号编码器 */
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);   /* 3 号编码器 */

  __HAL_TIM_SET_COUNTER(&htim1, 0);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  __HAL_TIM_SET_COUNTER(&htim8, 0);

  encoder_last1 = 0;
  encoder_last2 = 0;
  encoder_last3 = 0;

  HAL_TIM_Base_Start_IT(&htim6);                   /* 定时器6 中断里读编码器 */
}

/* 读一次三个编码器: 累加脉冲数, 再换算累计角度
 * 16 位计数器回绕用 (int16_t) 差值处理 */
static void Encoder_Update(void)
{
  uint16_t now;

  now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
  encoder1 += (int16_t)(now - encoder_last1);
  encoder_last1 = now;

  now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  encoder2 += (int16_t)(now - encoder_last2);
  encoder_last2 = now;

  now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim8);
  encoder3 += (int16_t)(now - encoder_last3);
  encoder_last3 = now;

  angle1 = encoder1 * 360.0f / ENCODER_PPR;
  angle2 = encoder2 * 360.0f / ENCODER_PPR;
  angle3 = encoder3 * 360.0f / ENCODER_PPR;
}

/* 定时器更新回调: 定时器6 采编码器, 脉冲步进定时器数脉冲 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* 定时器6: 采样编码器 */
  if (htim->Instance == TIM6)
  {
    Encoder_Update();
    return;
  }

  if (htim->Instance != STEPPER1_STEP_TIM->Instance || !stepper_running) return;

  stepper_current_steps++;
  if (stepper_current_steps >= stepper_target_steps)
  {
    HAL_TIM_PWM_Stop(STEPPER1_STEP_TIM, STEPPER1_STEP_CH);
    HAL_TIM_Base_Stop_IT(STEPPER1_STEP_TIM);
    stepper_running  = 0;
    stepper_finished = 1;
  }
}

/* 1 号步进电机脉冲计数
 * 注意: 与 IO.h 里的 STEPPER1_STEP_TIM 一一对应(现在是 TIM5)。
 * 若在 CubeMX 里勾选了该定时器的全局中断，请删掉本函数，用生成的即可。 */
void TIM5_IRQHandler(void)
{
  HAL_TIM_IRQHandler(STEPPER1_STEP_TIM);
}

/* ==================== 初始化 ==================== */

void INIT_ALL(void)
{
  uart_receive_start();

  Motor_Init();
  Encoder_Init();
  Pump_Control(0);

  Emm_V5_En_Control(ZDT_ADDR2, true, false);
  HAL_Delay(20);
  Emm_V5_En_Control(ZDT_ADDR3, true, false);
  HAL_Delay(20);

  HAL_Delay(2000);
}

/* ==================== 任务: 串口 1 ==================== */

/* 全部停止并复位流程 */
static void Motor_Stop_All(void)
{
  uint8_t i;

  for (i = 0; i < MOTOR_NUM; i++) Motor_Control(i, Motor_STOP, 0);
  Stepper_SetSpeed(0);

  HAL_GPIO_WritePin(OUT_AUX1_PORT, OUT_AUX1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OUT_AUX2_PORT, OUT_AUX2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RELAY4_PORT, RELAY4_PIN, GPIO_PIN_RESET);
  Pump_Control(0);

  Emm_V5_Stop_Now(ZDT_ADDR2, false);
  Emm_V5_Stop_Now(ZDT_ADDR3, false);

  step2_auto_run = 0;
  step2_state    = STEP2_DONE;
  step3_state    = STEP3_IDLE;
  step3_pending  = 0;
  ext_loop_mode  = 0;
  dc1_time_index = 0;
  ext_num        = 0;
  ext_state      = EXT_IDLE;
}

/* 串口 1 命令解析 */
void Task_Uart1(void)
{
  uint8_t start, len;
  char   *p;

  if (!uart_frame_flag) return;

  /* 找帧头，丢掉前面的杂字节 */
  start = 0;
  while (start < uart_rx_len && uart_rx_buf[start] != FRAME_HEADER) start++;
  if (start >= uart_rx_len) goto frame_reset;

  len = uart_rx_len - start;
  memmove(uart_rx_buf, uart_rx_buf + start, len);
  uart_rx_buf[len] = '\0';

  /* 去掉帧尾 */
  if (len >= 2 && uart_rx_buf[len - 2] == '\\' && uart_rx_buf[len - 1] == 'n')
  {
    uart_rx_buf[len - 2] = '\0';
    len -= 2;
  }
  else if (len >= 1 && uart_rx_buf[len - 1] == FRAME_TAIL)
  {
    uart_rx_buf[len - 1] = '\0';
    len -= 1;
  }

  /* 一键启动 / 全部停止 */
  if ((p = strstr((char *)uart_rx_buf, "#ALL:")) != NULL)
  {
    if (atoi(p + 5) == 1)      one_key_start_flag = 1;
    else if (atoi(p + 5) == 0) Motor_Stop_All();
  }
  /* 水泵 */
  else if ((p = strstr((char *)uart_rx_buf, "#PUMP:")) != NULL)
  {
    Pump_Control(atoi(p + 6) == 1);
  }
  /* 封膜流程 */
  else if ((p = strstr((char *)uart_rx_buf, "#PART3:")) != NULL)
  {
    if (atoi(p + 7) == 1)
    {
      ext_state = EXT_START_DC1;
    }
    else if (atoi(p + 7) == 0)
    {
      ext_state = EXT_IDLE;
      Motor_Control(Motor1, Motor_STOP, 0);
      HAL_GPIO_WritePin(RELAY4_PORT, RELAY4_PIN, GPIO_PIN_RESET);
      ext_loop_mode  = 0;
      dc1_time_index = 0;
      ext_num        = 0;
    }
  }
  /* 工位 1: 直流电机 2 */
  else if ((p = strstr((char *)uart_rx_buf, "#PART1:")) != NULL)
  {
    if (atoi(p + 7) == 1)
    {
      Motor_Control(Motor2, Motor_FORWARD, motor_speeds[Motor2]);
      HAL_GPIO_WritePin(OUT_AUX1_PORT, OUT_AUX1_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(OUT_AUX2_PORT, OUT_AUX2_PIN, GPIO_PIN_SET);
    }
    else if (atoi(p + 7) == 0)
    {
      Motor_Control(Motor2, Motor_STOP, 0);
      HAL_GPIO_WritePin(OUT_AUX1_PORT, OUT_AUX1_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(OUT_AUX2_PORT, OUT_AUX2_PIN, GPIO_PIN_RESET);
    }
  }
  /* 工位 2: 直流电机 3 + 风机水泵 */
  else if ((p = strstr((char *)uart_rx_buf, "#PART2:")) != NULL)
  {
    if (atoi(p + 7) == 1)
    {
      Motor_Control(Motor3, Motor_FORWARD, motor_speeds[Motor3]);
      HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN, GPIO_PIN_SET);
    }
    else if (atoi(p + 7) == 0)
    {
      Motor_Control(Motor3, Motor_STOP, 0);
      HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN, GPIO_PIN_RESET);
    }
  }
  /* 设置直流电机速度: #MOTOR:id,speed */
  else if ((p = strstr((char *)uart_rx_buf, "#MOTOR:")) != NULL)
  {
    uint8_t id = atoi(p + 7);
    char   *comma = strchr(p + 7, ',');

    if (comma != NULL && id >= 1 && id <= MOTOR_NUM)
    {
      motor_speeds[id - 1] = atoi(comma + 1);
    }
  }
  /* 启停直流电机: #MOTOR_CTRL:id,status */
  else if ((p = strstr((char *)uart_rx_buf, "#MOTOR_CTRL:")) != NULL)
  {
    uint8_t id = atoi(p + 12);
    char   *comma = strchr(p + 12, ',');

    if (comma != NULL && id >= 1 && id <= MOTOR_NUM)
    {
      if (atoi(comma + 1) == 1) Motor_Control(id - 1, Motor_FORWARD, motor_speeds[id - 1]);
      else                      Motor_Control(id - 1, Motor_STOP, 0);
    }
  }
  /* 设置步进电机速度: #STEPPER:id,speed */
  else if ((p = strstr((char *)uart_rx_buf, "#STEPPER:")) != NULL)
  {
    uint8_t id = atoi(p + 9);
    char   *comma = strchr(p + 9, ',');

    if (comma != NULL && id >= 1 && id <= STEPPER_NUM)
    {
      stepper_speeds[id - 1] = atoi(comma + 1);
    }
  }
  /* 设置步进电机停顿时间(s): #STEPPER_TIME:id,time */
  else if ((p = strstr((char *)uart_rx_buf, "#STEPPER_TIME:")) != NULL)
  {
    uint8_t id = atoi(p + 14);
    char   *comma = strchr(p + 14, ',');

    if (comma != NULL && id >= 1 && id <= STEPPER_NUM)
    {
      stepper_times[id - 1] = atoi(comma + 1) * 1000;
    }
  }
  /* 设置电机 1 运输时间(s): #DC1_TIME:id,time */
  else if ((p = strstr((char *)uart_rx_buf, "#DC1_TIME:")) != NULL)
  {
    uint8_t id = atoi(p + 10);
    char   *comma = strchr(p + 10, ',');

    if (comma != NULL)
    {
      if (id == 1)      dc1_transport_time[2] = atoi(comma + 1) * 1000;
      else if (id == 2) dc1_transport_time[3] = atoi(comma + 1) * 1000;
    }
  }
  /* 启停步进电机: #STEPPER_CTRL:id,status */
  else if ((p = strstr((char *)uart_rx_buf, "#STEPPER_CTRL:")) != NULL)
  {
    uint8_t id = atoi(p + 14);
    char   *comma = strchr(p + 14, ',');

    if (comma != NULL && id >= 1 && id <= STEPPER_NUM)
    {
      if (atoi(comma + 1) == 1)
      {
        if (id == 1)
        {
          Stepper_Rotate(180, stepper_speeds[Stepper1]);
          while (!stepper_finished);
          HAL_Delay(stepper_times[Stepper1]);
          stepper_finished = 0;
          Stepper_Rotate(180, stepper_speeds[Stepper1]);
        }
        else if (id == 2)
        {
          Emm_V5_Pos_Control(ZDT_ADDR2, 1, stepper_speeds[Stepper2], 0, ZDT_PULSE_PER_REV, 2, false);
        }
        else
        {
          Emm_V5_Pos_Control(ZDT_ADDR3, 0, stepper_speeds[Stepper3], 0, ZDT_PULSE_PER_REV, 2, false);
        }
      }
      else
      {
        if (id == 1) Stepper_SetSpeed(0);
        else         Emm_V5_Stop_Now(id, false);
      }
    }
  }
  /* 步进电机转动角度: #STEPPER_ANGLE:id,angle */
  else if ((p = strstr((char *)uart_rx_buf, "#STEPPER_ANGLE:")) != NULL)
  {
    uint8_t id = atoi(p + 16);
    char   *comma = strchr(p + 16, ',');

    if (comma != NULL && id >= 1 && id <= STEPPER_NUM)
    {
      char    *end;
      uint16_t angle = (uint16_t)strtoul(comma + 1, &end, 10);

      if (angle <= 360 && end != comma + 1)
      {
        if (id == 1)
        {
          Stepper_Rotate(angle, stepper_speeds[Stepper1]);
        }
        else
        {
          uint32_t pulse = (uint32_t)((float)angle * ZDT_PULSE_PER_REV / 360.0f);
          Emm_V5_Pos_Control(id, 0, stepper_speeds[id - 1], 10, pulse, 2, false);
        }
      }
    }
  }

frame_reset:
  memset(uart_rx_buf, 0, UART_BUF_SIZE);
  uart_rx_len   = 0;
  uart_frame_flag = 0;
  uart_receive_start();
}

/* ==================== 任务: 工艺流程 ==================== */

void Task_Process(void)
{
  Process_OneKey();
  Process_Step2();
  Process_Step3();
  Process_Step2Auto();
  Process_Ext();
}

/* 一键启动 */
static void Process_OneKey(void)
{
  if (!one_key_start_flag) return;
  one_key_start_flag = 0;

  if (step2_state != STEP2_IDLE && step2_state != STEP2_DONE)
  {
    Uart1_Send("page0.t16.txt=0\xff\xff\xff");   /* 忙 */
    return;
  }

  Motor_Control(Motor2, Motor_FORWARD, motor_speeds[Motor2]);
  HAL_GPIO_WritePin(OUT_AUX1_PORT, OUT_AUX1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OUT_AUX2_PORT, OUT_AUX2_PIN, GPIO_PIN_SET);

  step3_state = STEP3_IDLE;
  step3_pending = 0;
  ext_state   = EXT_IDLE;
  ext_loop_mode  = 0;
  dc1_time_index = 0;
  ext_num     = 0;
  motor3_started_by_step2 = 0;

  step2_tick  = HAL_GetTick();
  step2_state = STEP2_START_DELAY;

  Uart1_Send("page0.t16.txt=1\xff\xff\xff");
}

/* 放料完成: 等料走到称重工位再启动 3 号步进 */
static void Step3_Arm(void)
{
  step3_pending      = 1;
  step3_pending_tick = HAL_GetTick();
}

/* 称重工位(3 号闭环步进): 转半圈, 停 1s, 再转半圈
 * 原版本由 HX711 检测到有料触发, 称重模块已拆, 改为放料后延时触发 */
static void Step3_Start(void)
{
  static const uint16_t preset_weights[WEIGHT_PRESET_NUM] = {207, 212, 215};
  char buf[32];

  if (step3_state != STEP3_IDLE && step3_state != STEP3_DONE) return;

  Emm_V5_Pos_Control(ZDT_ADDR3, 0, stepper_speeds[Stepper3], 0, 1600, 2, false);
  step3_tick  = HAL_GetTick();
  step3_state = STEP3_RUNNING_FIRST;

  /* 屏幕显示预设重量与件数 */
  sprintf(buf, "page0.n4.val=%d\xff\xff\xff",
          (piece_count < WEIGHT_PRESET_NUM) ? preset_weights[piece_count] : 0);
  Uart1_Send(buf);

  piece_count++;
  sprintf(buf, "page0.n5.val=%d\xff\xff\xff", piece_count);
  Uart1_Send(buf);
}

/* 2 号闭环步进流程 */
static void Process_Step2(void)
{
  switch (step2_state)
  {
    case STEP2_START_DELAY:
      if (HAL_GetTick() - step2_tick >= TIME_START_DELAY)
      {
        Emm_V5_Pos_Control(ZDT_ADDR2, 1, stepper_speeds[Stepper2], 0, ZDT_PULSE_PER_REV, 2, false);
        step2_tick  = HAL_GetTick();
        step2_state = STEP2_RUNNING_FIRST;
        Step3_Arm();
      }
      break;

    case STEP2_RUNNING_FIRST:
      if (HAL_GetTick() - step2_tick >= TIME_STEP2_FIRST)
      {
        step2_tick  = HAL_GetTick();
        step2_state = STEP2_WAIT_FOR_SECOND;
      }
      break;

    case STEP2_WAIT_FOR_SECOND:
      if (!motor3_started_by_step2 && HAL_GetTick() - step2_tick >= TIME_STEP2_M3)
      {
        Motor_Control(Motor3, Motor_FORWARD, motor_speeds[Motor3]);
        motor3_started_by_step2 = 1;
        Pump_Control(pump_enable);
        HAL_GPIO_WritePin(OUT_AUX1_PORT, OUT_AUX1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(OUT_AUX2_PORT, OUT_AUX2_PIN, GPIO_PIN_RESET);
      }
      if (HAL_GetTick() - step2_tick >= TIME_STEP2_SECOND)
      {
        Emm_V5_Pos_Control(ZDT_ADDR2, 1, stepper_speeds[Stepper2], 0, ZDT_PULSE_PER_REV, 2, false);
        step2_tick  = HAL_GetTick();
        step2_state = STEP2_RUNNING_SECOND;
        Step3_Arm();
      }
      break;

    case STEP2_RUNNING_SECOND:
      if (HAL_GetTick() - step2_tick >= TIME_STEP2_LAST)
      {
        step2_state     = STEP2_DONE;
        step2_auto_run  = 1;
        step2_auto_tick = HAL_GetTick();
        step2_auto_state = 0;
      }
      break;

    default:
      break;
  }
}

/* 3 号闭环步进流程 */
static void Process_Step3(void)
{
  /* 放料后延时到点, 料已到称重工位 */
  if (step3_pending && HAL_GetTick() - step3_pending_tick >= TIME_TO_STEP3)
  {
    step3_pending = 0;
    Step3_Start();
  }

  switch (step3_state)
  {
    case STEP3_RUNNING_FIRST:
      if (HAL_GetTick() - step3_tick >= TIME_STEP3_FIRST)
      {
        step3_tick  = HAL_GetTick();
        step3_state = STEP3_WAIT_BETWEEN;
      }
      break;

    case STEP3_WAIT_BETWEEN:
      if (HAL_GetTick() - step3_tick >= TIME_STEP3_WAIT)
      {
        Emm_V5_Pos_Control(ZDT_ADDR3, 0, stepper_speeds[Stepper3], 10, 1600, 2, false);
        step3_tick  = HAL_GetTick();
        step3_state = STEP3_RUNNING_SECOND;
      }
      break;

    case STEP3_RUNNING_SECOND:
      if (HAL_GetTick() - step3_tick >= TIME_STEP3_SECOND)
      {
        step3_state = STEP3_DONE;
      }
      break;

    default:
      break;
  }
}

/* 2 号闭环步进自动循环 */
static void Process_Step2Auto(void)
{
  if (!step2_auto_run) return;

  if (step2_auto_state == 0)
  {
    if (HAL_GetTick() - step2_auto_tick >= TIME_STEP2_AUTO)
    {
      Emm_V5_Pos_Control(ZDT_ADDR2, 1, stepper_speeds[Stepper2], 0, ZDT_PULSE_PER_REV, 2, false);
      step2_auto_tick  = HAL_GetTick();
      step2_auto_state = 1;
      Step3_Arm();
    }
  }
  else
  {
    if (HAL_GetTick() - step2_auto_tick >= TIME_STEP2_AUTO_RST)
    {
      step2_auto_state = 0;
    }
  }
}

/* 封膜流程 */
static void Process_Ext(void)
{
  switch (ext_state)
  {
    case EXT_IDLE:
      /* 3 号步进完成且未开始过封膜 -> 进入封膜流程 */
      if (step3_state == STEP3_DONE && ext_num == 0)
      {
        step3_state = STEP3_IDLE;
        ext_state   = EXT_START_DC1;
      }
      break;

    case EXT_START_DC1:
      Motor_Control(Motor1, Motor_FORWARD, motor_speeds[Motor1]);
      ext_tick  = HAL_GetTick();
      ext_state = EXT_DELAY_DC1;
      break;

    case EXT_DELAY_DC1:
      if (!ext_loop_mode)
      {
        /* 分段模式：使用 DC1_TIME 设定 */
        if (HAL_GetTick() - ext_tick >= dc1_transport_time[dc1_time_index])
        {
          Motor_Control(Motor1, Motor_STOP, 0);
          dc1_time_index = (dc1_time_index == 2) ? 0 : (dc1_time_index + 1);
          ext_state = EXT_STEP1_FIRST;
        }
      }
      else
      {
        /* 循环模式：固定 dc1_loop_time */
        if (HAL_GetTick() - ext_tick >= dc1_loop_time)
        {
          Motor_Control(Motor1, Motor_STOP, 0);
          ext_state = EXT_STEP1_FIRST;
        }
      }
      break;

    case EXT_STEP1_FIRST:
      Stepper_Rotate(180, stepper_speeds[Stepper1]);
      ext_state = EXT_WAIT_FIRST;
      break;

    case EXT_WAIT_FIRST:
      if (stepper_finished)
      {
        stepper_finished = 0;
        ext_tick  = HAL_GetTick();
        ext_state = EXT_DELAY_STEP1;
      }
      break;

    case EXT_DELAY_STEP1:
      if (HAL_GetTick() - ext_tick >= stepper_times[Stepper1])
      {
        ext_state = EXT_STEP1_SECOND;
      }
      break;

    case EXT_STEP1_SECOND:
      Stepper_Rotate(180, stepper_speeds[Stepper1]);
      ext_state = EXT_WAIT_SECOND;
      break;

    case EXT_WAIT_SECOND:
      if (stepper_finished)
      {
        stepper_finished = 0;
        ext_num++;
        ext_tick = HAL_GetTick();
        Motor_Control(Motor1, Motor_FORWARD, motor_speeds[Motor1]);

        if (!ext_loop_mode && ext_num > 2)
        {
          ext_loop_mode = 1;     /* 三次后转为循环模式 */
        }
        ext_state = EXT_DELAY_DC1;
      }
      break;

    default:
      break;
  }
}
