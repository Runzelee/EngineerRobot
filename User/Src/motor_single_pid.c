#include <stdint.h>
#include <stdbool.h>
#include "motor_single_pid.h"
#include "motor_can.h"
#include "usart.h"
#include "math.h"
#include "stdio.h"
#include "string.h"

/* 使用 config.h 中的 SinglePID_Params_t，去掉本地重复定义 */
#include "config.h"

#include "unit_conversation.h"

/* 支持多个电机的速度缓存和 PID 状态数组 */
#define NUM_MOTORS MOTOR_SINGLE_PID_NUM

float velocities[NUM_MOTORS] = {0};

/* 每路 PID 状态 */
typedef struct {
    float pid_integral;
    float pid_prev_error;

    /* Debug */
    float error;
    float derivative;
    float pid_output;
    int16_t send_vol;
} SinglePID_State_t;

static SinglePID_State_t g_states[NUM_MOTORS];

/* 使用来自 config.c 的全局参数实例（single_pid_cfg） */
extern SinglePID_Params_t single_pid_cfg;

/* Debug 变量数组导出（在 header 中声明） */
float errors[NUM_MOTORS];
float derivatives[NUM_MOTORS];
float pid_outputs[NUM_MOTORS];
int16_t send_vols[NUM_MOTORS];

/* 简单限幅 */
static inline float clampf_s(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

/* 更新速度：增加 motor_index 参数以支持多电机 */
void Single_PID_Handler(uint8_t motor_index, uint16_t raw_ticks, float _velocity){
    velocities[motor_index] = RPM_TO_RAD_S(_velocity);
}

/**
 * @brief 单闭环速度 PID 控制函数（支持多电机），新增 motor_index 参数
 * @param setpoint 目标速度（单位：rad/s）
 * @param CAN_CMD_ID CAN 命令 ID
 * @param can_index 数据域低位索引（用于 Set_Motor_Torque）
 * @param motor_index 要控制的电机索引（0..NUM_MOTORS-1）
 */
void Single_PID_ToVelocity(float setpoint, uint32_t CAN_CMD_ID, uint8_t can_index, uint8_t motor_index)
{
    // 不做 motor_index 边界检查（由上层确保合法）

    /* 先计算 PID 输出（将核心计算抽离为可复用函数），然后发送 CAN */
    int16_t vol = Single_PID_ComputeSendVol(setpoint, motor_index);
    /* 按原逻辑发送给单个电机（保持向后兼容） */
    Set_Motor_Torque(vol, CAN_CMD_ID, can_index);
}

/**
 * @brief 仅计算 PID 并返回要发送的电压（不执行 CAN 发送）
 *        便于在上层批量打包后一次性发送，避免多次调用 HAL_CAN_AddTxMessage 导致的延迟/阻塞
 */
int16_t Single_PID_ComputeSendVol(float setpoint, uint8_t motor_index)
{
    /* 不做 motor_index 边界检查（由上层确保合法） */

    /* 使用全局配置实例 */
    const SinglePID_Params_t *params = &single_pid_cfg;

    /* PID 计算基于对应电机的速度和状态 */
    SinglePID_State_t *s = &g_states[motor_index];

    /*
     * 处理“回中仍有惯性”问题：
     * 当目标速度（setpoint）非常接近 0 时（认为用户已回中）
     * 我们希望尽快让底盘停止并避免积分影响导致持续输出。
     * 策略：
     *  - 使用 params->ERROR_DEADZONE 作为 setpoint 死区阈值（延续配置项）
     *  - 清除积分项以防止积分饱和导致持续驱动
     *  - 基于当前速度计算一个短时的 P+D 制动力（即把目标速度视为 0，计算误差和导数）
     * 这样能在手柄回中后给出一个反向制动力，抑制车体继续向前滑行。
     */
    if (fabsf(setpoint) < params->ERROR_DEADZONE) {
        /* 视为回中/停止命令：清除积分项，构造基于速度的制动输出 */
        s->pid_integral = 0.0f;

        /* 以当前速度为基础计算误差（目标 = 0） */
        s->error = 0.0f - velocities[motor_index];

        /* 导数相对于上一次误差（我们把上次误差视为 0，产生一个刹车斜率） */
        s->derivative = (s->error - 0.0f) / params->SAMPLE_DT;

        /* 仅使用 P 和 D 产生制动力，避免积分造成延续动作 */
        s->pid_output = params->Kp * s->error + params->Kd * s->derivative;

        /* 更新上次误差，避免下一次导数突变 */
        s->pid_prev_error = s->error;
    } else {
        /* 正常 PID 流程：计算误差、积分和导数 */
        s->error = setpoint - velocities[motor_index];
        if (fabsf(s->error) < params->ERROR_DEADZONE) {
            s->error = 0.0f;
        }

        s->pid_integral += s->error * params->SAMPLE_DT;
        s->pid_integral = clampf_s(s->pid_integral, -params->INTEGRAL_LIMIT, params->INTEGRAL_LIMIT);

        s->derivative = (s->error - s->pid_prev_error) / params->SAMPLE_DT;

        s->pid_output = params->Kp * s->error + params->Ki * s->pid_integral + params->Kd * s->derivative;

        s->pid_prev_error = s->error;
    }

    /* 可选的 UART 调试输出（包含电机索引） */
    if (params->ENABLE_UART_DEBUG) {
        uint8_t TxData[128];
        sprintf((char *)TxData, "m[%u]: v=%.2f, e=%.2f, out=%.2f\r\n", motor_index, velocities[motor_index], s->error, s->pid_output);
        HAL_UART_Transmit(&huart3, TxData, strlen((char *)TxData), 1000);
    }

    /* 输出限幅并转为 int16_t（驱动期望的电压范围） */
    float out = clampf_s(s->pid_output, -params->MAX_VOLTAGE, params->MAX_VOLTAGE);
    s->send_vol = (int16_t)roundf(out);

    /* 保存调试变量到导出的数组 */
    errors[motor_index] = s->error;
    derivatives[motor_index] = s->derivative;
    pid_outputs[motor_index] = s->pid_output;
    send_vols[motor_index] = s->send_vol;

    return s->send_vol;
}
