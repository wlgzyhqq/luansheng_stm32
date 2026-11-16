#ifndef __EMM_V5_H
#define __EMM_V5_H

/*
 * 张大头 Emm_V5 闭环步进驱动
 * 命令经串口 2 普通发送(不使用 DMA / 中断)
 * 帧格式: 地址 + 功能码 + 数据 + 0x6B
 */

#include "main.h"
#include "usart.h"
#include <stdbool.h>

#define ZDT_UART            huart2
#define ABS(x)              ((x) > 0 ? (x) : -(x))

/* 读取系统参数的类型 */
typedef enum {
  S_VBUS  = 5,    /* 总线电压 */
  S_CBUS  = 6,    /* 总线电流 */
  S_CPHA  = 7,    /* 相电流 */
  S_ENCO  = 8,    /* 编码器原始值 */
  S_CLKC  = 9,    /* 实时脉冲数 */
  S_ENCL  = 10,   /* 线性化校准后的编码器值 */
  S_CLKI  = 11,   /* 输入脉冲数 */
  S_TPOS  = 12,   /* 目标位置 */
  S_SPOS  = 13,   /* 实时设定的目标位置 */
  S_VEL   = 14,   /* 实时转速 */
  S_CPOS  = 15,   /* 实时位置 */
  S_PERR  = 16,   /* 位置误差 */
  S_VBAT  = 17,   /* 多圈编码器电池电压(Y42) */
  S_TEMP  = 18,   /* 实时温度(Y42) */
  S_FLAG  = 19,   /* 状态标志位 */
  S_OFLAG = 20,   /* 回零状态标志位 */
  S_OAF   = 21,   /* 状态标志位 + 回零状态标志位(Y42) */
  S_PIN   = 22,   /* 引脚状态(Y42) */
} SysParams_t;

/* ---------------- 触发动作 ---------------- */
void Emm_V5_Trig_Encoder_Cal(uint8_t addr);
void Emm_V5_Reset_Motor(uint8_t addr);
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr);
void Emm_V5_Reset_Clog_Pro(uint8_t addr);
void Emm_V5_Restore_Motor(uint8_t addr);

/* ---------------- 运动控制 ---------------- */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF);
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, bool snF);
void Emm_V5_Set_QPos_Params(uint8_t addr, uint16_t vel, uint8_t acc, uint8_t raF, bool snF);
void Emm_V5_QPos_Control(uint8_t addr, int32_t clk);
void Emm_V5_Stop_Now(uint8_t addr, bool snF);
void Emm_V5_Synchronous_motion(uint8_t addr);

/* ---------------- 原点回零 ---------------- */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF);
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);
void Emm_V5_Origin_Interrupt(uint8_t addr);
void Emm_V5_Origin_Read_Params(uint8_t addr);
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel,
                                 uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);
void X_V2_Origin_Read_SL_RP(uint8_t addr);
void X_V2_Origin_Modify_SL_RP(uint8_t addr, bool svF, uint16_t sl_rp);

/* ---------------- 读取系统参数 ---------------- */
void Emm_V5_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms);
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);

/* ---------------- 读写驱动参数 ---------------- */
void Emm_V5_Modify_Motor_ID(uint8_t addr, bool svF, uint8_t id);
void Emm_V5_Modify_MicroStep(uint8_t addr, bool svF, uint8_t mstep);
void Emm_V5_Modify_PDFlag(uint8_t addr, bool pdf);
void Emm_V5_Read_Opt_Param_Sta(uint8_t addr);
void Emm_V5_Modify_Motor_Type(uint8_t addr, bool svF, bool mottype);
void Emm_V5_Modify_Firmware_Type(uint8_t addr, bool svF, bool fwtype);
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, bool ctrl_mode);
void Emm_V5_Modify_Motor_Dir(uint8_t addr, bool svF, bool dir);
void Emm_V5_Modify_Lock_Btn(uint8_t addr, bool svF, bool lockbtn);
void Emm_V5_Modify_S_Vel(uint8_t addr, bool svF, bool s_vel);
void Emm_V5_Modify_OM_mA(uint8_t addr, bool svF, uint16_t om_ma);
void Emm_V5_Modify_FOC_mA(uint8_t addr, bool svF, uint16_t foc_mA);
void Emm_V5_Read_PID_Params(uint8_t addr);
void Emm_V5_Modify_PID_Params(uint8_t addr, bool svF, uint32_t kp, uint32_t ki, uint32_t kd);
void Emm_V5_Read_DMX512_Params(uint8_t addr);
void Emm_V5_Modify_DMX512_Params(uint8_t addr, bool svF, uint16_t tch, uint8_t nch, uint8_t mode,
                                 uint16_t vel, uint16_t acc, uint16_t vel_step, uint32_t pos_step);
void Emm_V5_Read_Pos_Window(uint8_t addr);
void Emm_V5_Modify_Pos_Window(uint8_t addr, bool svF, uint16_t prw);
void Emm_V5_Read_Otocp(uint8_t addr);
void Emm_V5_Modify_Otocp(uint8_t addr, bool svF, uint16_t otp, uint16_t ocp, uint16_t time_ms);
void Emm_V5_Read_Heart_Protect(uint8_t addr);
void Emm_V5_Modify_Heart_Protect(uint8_t addr, bool svF, uint32_t hp);
void Emm_V5_Read_Integral_Limit(uint8_t addr);
void Emm_V5_Modify_Integral_Limit(uint8_t addr, bool svF, uint32_t il);

/* ---------------- 读取全部参数 ---------------- */
void Emm_V5_Read_System_State_Params(uint8_t addr);
void Emm_V5_Read_Motor_Conf_Params(uint8_t addr);

#endif
