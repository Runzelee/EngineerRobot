#include "gimbal_control.h"
#include "motor_double_pid.h"
#include "motor_can.h"
#include "config.h"
#include "main.h" /* for HAL_Delay */
#include <math.h>
#include <stdio.h>
#include "unit_conversation.h"

/* 模块内部状态 */
static Gimbal_State_t g_state = GIMBAL_STATE_RESET;
/* 存储 reset 时读取到的角度作为新坐标系零点（多圈 rad） */
static float g_zero_offsets[MOTOR_DOUBLE_PID_NUM] = {0};

/* 容错/超时参数（可根据需要改为配置项） */
#define GIMBAL_STEP_TIMEOUT_MS (15000U) /* 单个动作最大等待 15s */
#define GIMBAL_TICK_MS 10U

static void record_zero(uint8_t motor_index)
{
    /* 记录当前绝对角度为该电机在新坐标系的零点 */
    extern float dm_angles[MOTOR_DOUBLE_PID_NUM];
    g_zero_offsets[motor_index] = dm_angles[motor_index];
}

/* Note: DoublePID_Homing_Start is blocking and will return when homing is done.
 * Therefore no polling helper is required here.
 */

static bool wait_motor_at_angle(uint8_t motor_index, float target_rad, float tolerance_rad, uint32_t can_cmd_id, uint8_t can_index)
{
    extern float dm_angles[MOTOR_DOUBLE_PID_NUM];
    uint32_t waited = 0;
    while (1) {
    /* 调用 PID 控制函数推进一次控制输出（内部会发送 CAN）
     * 在 Ready 阶段我们使用绝对角度模式（use_velocity_mode = false），
     * 以保证电机移动到期望的绝对位置并进入就绪状态。
     */
    Double_PID_ToAngle(target_rad, can_cmd_id, can_index, motor_index, false);
        float cur = dm_angles[motor_index];
        float err = cur - target_rad;
        if (err < 0) err = -err;
        if (err <= tolerance_rad) return true;
        HAL_Delay(GIMBAL_TICK_MS);
        waited += GIMBAL_TICK_MS;
        if (waited >= GIMBAL_STEP_TIMEOUT_MS) return false;
    }
}

void Gimbal_Init(void)
{
    /* 注册 CAN 回调，使 motor 模块能更新角速度数组 */
    Motor_CAN_RegisterAngleVelocityCallback(Double_PID_Handler);
    g_state = GIMBAL_STATE_RESET;
}

Gimbal_State_t Gimbal_GetState(void)
{
    return g_state;
}

void Gimbal_SetReadyTargets(float lift_rad, float pitch_rad, float yaw_rad)
{
    gimbal_cfg.lift_ready_target_rad = lift_rad;
    gimbal_cfg.pitch_ready_target_rad = pitch_rad;
    gimbal_cfg.yaw_ready_target_rad = yaw_rad;
}

bool Gimbal_ResetAndBlock(void)
{
    /* 顺序：lift -> pitch -> yaw，每步 homing 完成后记录零点 */
    uint8_t lift = gimbal_cfg.lift_motor_index;
    uint8_t pitch = gimbal_cfg.pitch_motor_index;
    uint8_t yaw = gimbal_cfg.yaw_motor_index;

    /* lift */
    DoublePID_SetHomingConfig(lift, gimbal_cfg.lift_reset_homing);
    DoublePID_Homing_Start(lift, gimbal_cfg.can_cmd_id, gimbal_cfg.lift_can_index);
    record_zero(lift);

    /* pitch */
    DoublePID_SetHomingConfig(pitch, gimbal_cfg.pitch_reset_homing);
    DoublePID_Homing_Start(pitch, gimbal_cfg.can_cmd_id, gimbal_cfg.pitch_can_index);
    record_zero(pitch);

    /* yaw */
    DoublePID_SetHomingConfig(yaw, gimbal_cfg.yaw_reset_homing);
    DoublePID_Homing_Start(yaw, gimbal_cfg.can_cmd_id, gimbal_cfg.yaw_can_index);
    record_zero(yaw);

    g_state = GIMBAL_STATE_RESET;
    return true;
}

bool Gimbal_ReadyAndBlock(void)
{
    /* 顺序：lift -> pitch -> yaw
     * - lift 可能通过另一个 homing 完成 fine positioning
     * - pitch/yaw 使用 Double_PID_ToAngle 到配置的目标（相对于 reset 记录的新零点）
     */
    uint8_t lift = gimbal_cfg.lift_motor_index;
    uint8_t pitch = gimbal_cfg.pitch_motor_index;
    uint8_t yaw = gimbal_cfg.yaw_motor_index;

    /* 1) lift: 若配置了 lift_ready_homing 则执行一次 homing */
    if (gimbal_cfg.lift_ready_homing != NULL) {
        DoublePID_SetHomingConfig(lift, gimbal_cfg.lift_ready_homing);
        DoublePID_Homing_Start(lift, gimbal_cfg.can_cmd_id, gimbal_cfg.lift_can_index);
        /* homing 完成后可选择更新零点；保持与 reset 期间一致，这里不覆盖原零点，除非需要 */
    }

    /* 2) pitch: 到位（使用记录的新零点） */
    {
        float target_pitch = g_zero_offsets[pitch] + gimbal_cfg.pitch_ready_target_rad;
        float tol = double_pid_cfg.ANG_DEAD_EXIT; /* 使用全局配置作为容差 */
        if (!wait_motor_at_angle(pitch, target_pitch, tol, gimbal_cfg.can_cmd_id, gimbal_cfg.pitch_can_index)) return false;
    }

    /* 3) yaw */
    {
        float target_yaw = g_zero_offsets[yaw] + gimbal_cfg.yaw_ready_target_rad;
        float tol = double_pid_cfg.ANG_DEAD_EXIT;
        if (!wait_motor_at_angle(yaw, target_yaw, tol, gimbal_cfg.can_cmd_id, gimbal_cfg.yaw_can_index)) return false;
    }

    g_state = GIMBAL_STATE_READY;
    return true;
}
