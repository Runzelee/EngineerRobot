#ifndef MOTOR_SINGLE_PID_H
#define MOTOR_SINGLE_PID_H

#include <stdint.h>
#include <stdbool.h>

/* 使用全局配置头 */
#include "config.h" /* 需在工程中提供，包含 SinglePID_Params_t 定义 */

/* 支持多个电机（默认 4 个），为每个电机维护独立速度和 PID 状态 */
#define MOTOR_SINGLE_PID_NUM 4

/* 每个电机当前速度（rad/s）数组，索引 0..MOTOR_SINGLE_PID_NUM-1 */
extern float velocities[MOTOR_SINGLE_PID_NUM];

/* Debug / PID 输出变量（按电机索引读取） */
extern float errors[MOTOR_SINGLE_PID_NUM];
extern float derivatives[MOTOR_SINGLE_PID_NUM];
extern float pid_outputs[MOTOR_SINGLE_PID_NUM];
extern int16_t send_vols[MOTOR_SINGLE_PID_NUM];

/* 函数原型，新增 motor_index 参数用于区分电机（0..MOTOR_SINGLE_PID_NUM-1） */
void Single_PID_Handler(uint8_t motor_index, uint16_t raw_ticks, float _velocity);
// 使用全局配置 single_pid_cfg：无需外部传入 params
void Single_PID_ToVelocity(float setpoint_velocity, uint32_t CAN_CMD_ID, uint8_t can_index, uint8_t motor_index);
// 仅计算 PID 输出并返回要发送的电压（不发送 CAN），用于批量打包发送以避免多次阻塞
int16_t Single_PID_ComputeSendVol(float setpoint_velocity, uint8_t motor_index);

#endif /* MOTOR_SINGLE_PID_H */
