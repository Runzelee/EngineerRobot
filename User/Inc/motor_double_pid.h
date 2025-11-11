#ifndef MOTOR_DOUBLE_PID_H
#define MOTOR_DOUBLE_PID_H

#include <stdint.h>
#include <stdbool.h>

/* 使用全局配置头，提供 DoublePID_Params_t 定义 */
#include "config.h"

/* 支持多个电机（默认 4 个），为每个电机维护独立角度/速度和 PID 状态 */
#define MOTOR_DOUBLE_PID_NUM 8

/* 每个电机当前速度（rad/s）与角度（rad, 多圈展开）数组，索引 0..MOTOR_DOUBLE_PID_NUM-1 */
extern float dm_velocities[MOTOR_DOUBLE_PID_NUM];
extern float dm_angles[MOTOR_DOUBLE_PID_NUM];

/* Debug / PID 输出变量（按电机索引读取） */
extern float dm_errors_ang[MOTOR_DOUBLE_PID_NUM];
extern float dm_derivatives_ang[MOTOR_DOUBLE_PID_NUM];
extern float dm_pid_outputs_ang[MOTOR_DOUBLE_PID_NUM];

extern float dm_errors_vel[MOTOR_DOUBLE_PID_NUM];
extern float dm_derivatives_vel[MOTOR_DOUBLE_PID_NUM];
extern float dm_pid_outputs_vel[MOTOR_DOUBLE_PID_NUM];
extern int16_t dm_send_vols[MOTOR_DOUBLE_PID_NUM];

/* 函数原型（新增 motor_index 参数用于区分电机） */
void Double_PID_Handler(uint8_t motor_index, uint16_t raw_ticks, float rpm);
// 使用全局配置 double_pid_cfg：无需外部传入 params
// 新增参数 `use_velocity_mode`：
//   - true  : 将传入的 target_rad 视为角速度（rad/s），在内部按 SAMPLE_DT 积分到内部目标（用于遥控器/摇杆）
//   - false : 将传入的 target_rad 视为绝对角度（rad，多圈展开），直接设定目标并进入 ready/就绪行为
void Double_PID_ToAngle(float target_rad, uint32_t CAN_CMD_ID, uint8_t can_index, uint8_t motor_index, bool use_velocity_mode);
// 仅计算 PID 输出并返回要发送的电压（不发送 CAN），用于批量打包发送以避免多次阻塞
int16_t Double_PID_ComputeSendVol(float target_rad, uint8_t motor_index);

/* 新增：允许为每个电机指定不同的双环 PID 参数结构体地址
 * 用法示例：
 *   DoublePID_SetParams(0, &double_pid_cfg_variant1);
 * 如果传入 NULL，则回退到全局默认 `double_pid_cfg`。
 */
void DoublePID_SetParams(uint8_t motor_index, const DoublePID_Params_t *params);
const DoublePID_Params_t *DoublePID_GetParams(uint8_t motor_index);

/* 每电机可绑定的 homing 配置接口：若传入 NULL 则回退到全局 homing_cfg */
void DoublePID_SetHomingConfig(uint8_t motor_index, const Homing_Config_t *cfg);
const Homing_Config_t *DoublePID_GetHomingConfig(uint8_t motor_index);

/* ---------- Homing 单独接口（将 homing 逻辑从 Double_PID_ToAngle 中抽离） ----------
 * 说明：
 *  - DoublePID_Homing_Start 为阻塞式接口，会在内部完成 homing 的循环并返回（或超时）。
 *    上游不再需要周期性调用 ProcessTick。
 *  - 也提供 IsDone/Abort 辅助函数以便外部显式查询/中止状态。
 */
/* 启动指定电机的 homing，立即向总线发送 homing 扭矩（start 会发送一次初始扭矩）
 * 参数 CAN_CMD_ID 与 can_index 用于立即发送 CAN
 */
void DoublePID_Homing_Start(uint8_t motor_index, uint32_t CAN_CMD_ID, uint8_t can_index);
// 说明：DoublePID_Homing_Start 为阻塞式接口，会在内部完成 homing 的循环并返回（或超时）。
// 如需查询状态，请使用 DoublePID_Homing_IsDone().
// 查询 homing 是否已完成（true: 已完成或被跳过）
bool DoublePID_Homing_IsDone(uint8_t motor_index);
// 取消/中止当前 homing（会停止输出并标记为已完成）
void DoublePID_Homing_Abort(uint8_t motor_index);

#endif /* MOTOR_DOUBLE_PID_H */
