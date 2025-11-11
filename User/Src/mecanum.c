/*
 * mecanum.c
 * Mecanum 轮解算实现
 *
 * 将整车速度解算为每个轮子的角速度（rad/s），并可直接调用 Single_PID_ToVelocity 发送
 */

#include "mecanum.h"
#include "unit_conversation.h" /* 提供 PI 与单位换算宏 */
#include "motor_single_pid.h" /* Single_PID_ToVelocity */
#include "motor_can.h"
#include "config.h"

#include <math.h>

/*
 * 解算矩阵（采用常见 4 轮 mecanum，滚轮 45° 配置）
 * 线速度到车轮线速度的关系（矩阵形式）：
 * [v1]   [ 1  -1  -(L+W)] [vx]
 * [v2] = [ 1   1   (L+W)] [vy]
 * [v3]   [ 1   1  -(L+W)] [wz]
 * [v4]   [ 1  -1   (L+W)]
 * 其中 vi 为第 i 个车轮的切向线速度（单位：m/s），wheel_omega_i = vi / r
 * 注：坐标系约定：vx 向前为正，vy 向左为正，wz 逆时针为正。
 */

void Mecanum_Solve(float vx, float vy, float wz, float wheel_omega[4])
{
    /* 车轮线速度（m/s） */
    float r = MECANUM_WHEEL_RADIUS_M;
    float lplusw = MECANUM_L_PLUS_W;

    /* 根据上面矩阵计算每轮线速度 */
    float v1 = 1*(vx - vy + lplusw * wz); //前右
    float v2 = 1*(vx + vy + lplusw * wz); //后右
    float v3 = 1*(- vx + vy + lplusw * wz); //后左
    float v4 = 1*(- vx - vy + lplusw * wz); //前左

    /* 线速度转角速度（rad/s） */
    wheel_omega[0] = v1 / r;
    wheel_omega[1] = v2 / r;
    wheel_omega[2] = v3 / r;
    wheel_omega[3] = v4 / r;
}

/**
 * @brief 解算 mecanum 轮的速度并运用于电机
 * @param vx 前后线速度，单位 m/s（向前为正）
 * @param vy 左右线速度，单位 m/s（向左为正）
 * @param wz 旋转速度，单位 rad/s（逆时针为正）
 * @param CAN_CMD_ID 电机控制命令的 CAN ID，GM6020 为 0x1FF，M3508 为 0x200
 * @param start_index 数据域低位索引，一般与电机 ID 一致
 */
void Mecanum_Apply(float vx, float vy, float wz, uint32_t CAN_CMD_ID, uint8_t start_index)
{
    float omegas[4];
    Mecanum_Solve(vx, vy, wz, omegas);

    /* 批量处理：先计算每路 PID 输出（不发送），然后一次性打包发送，避免多次 HAL_CAN_AddTxMessage 导致的延时/阻塞
       start_index 保持与原 Set_Motor_Torque 的索引语义一致（通常传入 1） */
    int16_t vols[4];
    for (int i = 0; i < 4; ++i) {
        /* Single PID now uses global single_pid_cfg; params argument is ignored */
        vols[i] = Single_PID_ComputeSendVol(omegas[i], (uint8_t)i);
    }
    // 一次性打包发送四个电机的扭矩
    Set_Motors_Torque(vols, CAN_CMD_ID);
}

/**
 * @brief 仅解算 mecanum 轮的速度，不发送控制命令
 * @param vx 前后线速度，单位 m/s（向前为正）
 * @param vy 左右线速度，单位 m/s（向左为正）
 * @param wz 旋转速度，单位 rad/s（逆时针为正）
 * @param out_omegas 输出数组，存放四个轮子的角速度（rad/s）
 */
void Mecanum_ComputeOnly(float vx, float vy, float wz, float out_omegas[4])
{
    Mecanum_Solve(vx, vy, wz, out_omegas);
}
