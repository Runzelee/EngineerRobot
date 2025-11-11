#ifndef __MOTOR_CAN_H
#define __MOTOR_CAN_H

#include <stdint.h>

void Enable_CAN1(uint32_t CAN_CMD_ID);
void Set_Motor_Torque(int16_t vol, uint32_t CAN_CMD_ID, uint8_t index);
// 批量发送 4 路电机扭矩，一次填充完整的 8 字节 CAN 数据并发送，避免多次逐个发送带来的延迟
// vols: 按电机索引排列的扭矩数组（索引 0..3）
// start_index: 与 Set_Motor_Torque 保持一致的低位索引起始值（通常传入 1）
void Set_Motors_Torque(const int16_t vols[4], uint32_t CAN_CMD_ID);

// 增加 motor_index 参数，用于区分不同电机的回调（0..3）
typedef void (*Motor_CAN_AngleVelocityCallback_t)(uint8_t motor_index, uint16_t raw_angle, float velocity);

void Motor_CAN_RegisterAngleVelocityCallback(Motor_CAN_AngleVelocityCallback_t callback);

#endif /* __MOTOR_CAN_H */
