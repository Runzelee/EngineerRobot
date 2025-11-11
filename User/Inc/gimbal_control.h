#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* 云台状态枚举：reset 与 ready */
typedef enum {
    GIMBAL_STATE_RESET = 0,
    GIMBAL_STATE_READY
} Gimbal_State_t;

/* 初始化云台控制模块（注册回调、必要的初始化） */
void Gimbal_Init(void);

/* 阻塞接口：进入 reset 状态并完成全部 reset 步骤（顺序：lift -> pitch -> yaw）
 * 返回 true 表示完成（所有电机已 homing 并记录新零点），false 表示发生错误
 */
bool Gimbal_ResetAndBlock(void);

/* 阻塞接口：进入 ready 状态并使各电机到位（顺序：lift -> pitch -> yaw）
 * - yaw/pitch 通过 Double PID 到预设角度（相对于 reset 时记录的新零点）
 * - lift 可选择通过另一个 homing 来完成（由配置决定）
 * 返回 true 表示到位成功，false 表示超时或错误
 */
bool Gimbal_ReadyAndBlock(void);

/* 非阻塞工具：查询当前云台模块状态 */
Gimbal_State_t Gimbal_GetState(void);

/* 可选：用户可调用以更新 ready 目标角（单位：rad，相对于 reset 后的新零点） */
void Gimbal_SetReadyTargets(float lift_rad, float pitch_rad, float yaw_rad);

#endif /* GIMBAL_CONTROL_H */
