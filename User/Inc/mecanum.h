/*
 * mecanum.h
 * Mecanum 轮解算头文件
 *
 * 提供从车体速度（线速度 vx, vy，角速度 wz）到每个车轮角速度（rad/s）的解算
 * 以及直接将解算结果传入单环速度 PID 的辅助函数
 *
 * 所有注释为中文
 */

#ifndef __MECANUM_H
#define __MECANUM_H

#include <stdint.h>
#include "config.h" /* 包含 SinglePID_Params_t */

/* 机械参数：用户可根据实际平台修改这些宏 */
/* 车轮半径（米） */
#ifndef MECANUM_WHEEL_RADIUS_M
#define MECANUM_WHEEL_RADIUS_M (0.031f) /* 默认 31 mm */
#endif

/* 车体几何：L 为前后轴中心到质心的距离，W 为左右轴中心到质心的距离（单位：米）
   在常见推导中常使用 (L + W) 一项，默认给出一个估算值，按需调整 */
#ifndef MECANUM_L_PLUS_W
#define MECANUM_L_PLUS_W (1.0f) /* 默认 L+W = 0.18 m */
#endif

/* 轮序说明（索引对应解算输出数组下标）
   0 - 前左 (Front Left)
   1 - 前右 (Front Right)
   2 - 后左 (Rear Left)
   3 - 后右 (Rear Right)
 */

/*
 * 函数：Mecanum_Solve
 * 说明：将车体速度解算为每个轮子的角速度（rad/s）
 * 参数：
 *   vx - 车体前向线速度（米/秒），正方向为车头向前
 *   vy - 车体侧向线速度（米/秒），正方向为向左
 *   wz - 车体绕重心的角速度（rad/s），正方向为逆时针
 *   wheel_omega - 输出数组，长度至少为 4，按上面索引顺序存放每个轮子的角速度（rad/s）
 * 返回：无
 */
void Mecanum_Solve(float vx, float vy, float wz, float wheel_omega[4]);

/*
 * 函数：Mecanum_Apply
 * 说明：解算并将每个轮子的角速度作为 setpoint 传入 Single_PID_ToVelocity
 * 参数：
 *   params - 单环 PID 参数指针（可传 NULL，函数内部会尝试使用已注册的默认参数）
 *   vx, vy, wz - 与 Mecanum_Solve 相同的速度
 *   CAN_CMD_ID - 发送 CAN 的基准 ID（与现有代码约定一致），
 *                Single_PID_ToVelocity 的 index 参数会依次传入 start_index .. start_index+3
 *   start_index - 目标电机在 CAN 帧中的索引（通常为 1 开始）
 * 返回：无
 * 注意：Single_PID_ToVelocity 内部会有延时（SAMPLE_DT），因此该函数会阻塞并逐个发送四次 PID 指令。
 */
void Mecanum_Apply(float vx, float vy, float wz, uint32_t CAN_CMD_ID, uint8_t start_index);

/* 仅计算角速度（不发送） */
void Mecanum_ComputeOnly(float vx, float vy, float wz, float out_omegas[4]);

#endif /* __MECANUM_H */
