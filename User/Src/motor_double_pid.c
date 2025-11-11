/* 双环 PID 多电机实现
 * - 接口兼容 motor_single_pid 的风格（全局 cfg、状态数组、Handler、ComputeSendVol）
 * - 内部算法与提供的 main.c 中的双环逻辑保持一致（位置外环产出速度、速度内环产出电压）
 * - 输入：角度以 ticks(0..8191) 形式更新，速度以 rpm 更新；外部 API 接受角度 target（rad）
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "motor_double_pid.h"
#include "config.h"
#include "unit_conversation.h"
#include "motor_can.h" // Set_Motor_Torque
#include "usart.h"
#include "main.h" /* HAL_Delay */

/* 支持的电机数量由头文件宏定义 MOTOR_DOUBLE_PID_NUM 提供 */
#define NUM_MOTORS MOTOR_DOUBLE_PID_NUM

/* 全局状态：当前速度（rad/s）与展开后的角度（rad，多圈展开）由外部导出 */
float dm_velocities[NUM_MOTORS] = {0};
float dm_angles[NUM_MOTORS] = {0};

/* 调试导出变量数组 */
float dm_errors_ang[NUM_MOTORS] = {0};
float dm_derivatives_ang[NUM_MOTORS] = {0};
float dm_pid_outputs_ang[NUM_MOTORS] = {0};

float dm_errors_vel[NUM_MOTORS] = {0};
float dm_derivatives_vel[NUM_MOTORS] = {0};
float dm_pid_outputs_vel[NUM_MOTORS] = {0};
int16_t dm_send_vols[NUM_MOTORS] = {0};

/* 每路内部状态：角度外环与速度内环的积分与上一次误差 */
/* Homing 状态枚举：使用具名常量避免散落的 0/1/2 magic number */
typedef enum {
    HOMING_NOT_STARTED = 0,
    HOMING_IN_PROGRESS,
    HOMING_DONE,
} HomingState_t;

typedef struct {
    /* 外环（角度） */
    float ang_integral;    /* 单位：rad * s */
    float ang_prev_error;  /* 单位：rad */

    /* 内环（速度） */
    float vel_integral;    /* 单位：rad * s */
    float vel_prev_error;  /* 单位：rad/s */

    /* 便于角度换档：保存上次原始 ticks */
    uint16_t last_ticks_raw;
    int32_t  turn_count; /* 多圈计数 */
    /* 是否存在外部非零速度输入(摇杆在外侧) */
    bool input_active;
    /* homing/limit 检测相关状态（新增） */
    HomingState_t homing_state; /* HOMING_NOT_STARTED/HOMING_IN_PROGRESS/HOMING_DONE */
    float homing_last_angle; /* 用于检测上电时是否继续运动（rad，多圈展开） */
    uint16_t homing_no_motion_samples; /* 连续不动计数（上电找限位） */

    /* 运行时限位检测（当遥控器持续推到物理限位时触发） */
    float limit_last_angle;
    uint16_t limit_no_motion_samples;
} DoublePID_State_t;

static DoublePID_State_t g_states[NUM_MOTORS] = {0};

/* 每路保存的目标角（多圈展开，单位 rad）——当外部把参数作为“角度变化量/速度”时使用 */
static float dm_target_angles[NUM_MOTORS] = {0};
static bool dm_target_inited[NUM_MOTORS] = {false};

/* 使用来自 config.c 的全局参数实例（默认配置） */
extern DoublePID_Params_t double_pid_cfg;

/* 每路可以绑定到不同的 DoublePID_Params_t 实例（允许每电机不同参数）
 * 默认情况下未绑定的通道会回退到 `double_pid_cfg`。
 */
static const DoublePID_Params_t *g_params[NUM_MOTORS] = {0};

/* 为每个电机指定参数实例（可传 NULL 恢复为全局默认） */
void DoublePID_SetParams(uint8_t motor_index, const DoublePID_Params_t *params)
{
    g_params[motor_index] = params;
}

const DoublePID_Params_t *DoublePID_GetParams(uint8_t motor_index)
{
    return (g_params[motor_index] != NULL) ? g_params[motor_index] : &double_pid_cfg;
}

/* 每电机绑定的 homing 配置指针（可为 NULL，表示使用全局 homing_cfg） */
static const Homing_Config_t *g_homing_params[NUM_MOTORS] = {0};

void DoublePID_SetHomingConfig(uint8_t motor_index, const Homing_Config_t *cfg)
{
    g_homing_params[motor_index] = cfg;
}

const Homing_Config_t *DoublePID_GetHomingConfig(uint8_t motor_index)
{
    return (g_homing_params[motor_index] != NULL) ? g_homing_params[motor_index] : &homing_cfg;
}

/* ---------- Homing 状态机函数实现 ----------
 * 把原来在 Double_PID_ToAngle 内的 homing 逻辑抽出，便于外部显式控制或在主循环中单独调用。
 */
void DoublePID_Homing_Start(uint8_t motor_index, uint32_t CAN_CMD_ID, uint8_t can_index)
{
    DoublePID_State_t *s = &g_states[motor_index];
    const Homing_Config_t *hc = DoublePID_GetHomingConfig(motor_index);

    if (!hc->ENABLE_HOMING) {
        /* 配置禁用 homing：直接标记为已完成 */
        s->homing_state = HOMING_DONE;
        return;
    }

    /* 如果当前没有正在进行的 homing，则（重新）初始化并进入 IN_PROGRESS
     * 这样可以支持在 reset 完成后、ready 阶段再次调用 homing（例如用于 fine positioning）。
     * 保持对已有 IN_PROGRESS 状态的保护，避免重复初始化覆盖正在进行的 homing。*/
    if (s->homing_state != HOMING_IN_PROGRESS) {
        s->homing_state = HOMING_IN_PROGRESS;
        s->homing_last_angle = dm_angles[motor_index];
        s->homing_no_motion_samples = 0;
        /* 初始化运行时限位检测参考 */
        s->limit_last_angle = dm_angles[motor_index];
        s->limit_no_motion_samples = 0;
        /* 立即发送一次 homing 扭矩以启动动作（避免上游需要先调用 ProcessTick 才有输出） */
        Set_Motor_Torque(hc->HOMING_TORQUE, CAN_CMD_ID, can_index);
    }

    /* Blocking homing loop: this function will not return until homing is
     * considered complete (no-motion samples reached) or homing is disabled.
     * This encapsulates the previous ProcessTick behavior so upstream callers
     * don't need to poll.
     */
    while (s->homing_state == HOMING_IN_PROGRESS) {
        /* 持续向负方向施力，检测角度是否停止变化 */
        Set_Motor_Torque(hc->HOMING_TORQUE, CAN_CMD_ID, can_index);

        float delta = fabsf(dm_angles[motor_index] - s->homing_last_angle);
        if (delta > hc->HOMING_MOTION_THRESHOLD_RAD) {
            /* 仍然在移动，重置计数并记住最新位置 */
            s->homing_last_angle = dm_angles[motor_index];
            s->homing_no_motion_samples = 0;
        } else {
            /* 无明显移动，累计样本 */
            s->homing_no_motion_samples++;
        }

        if (s->homing_no_motion_samples >= hc->HOMING_NO_MOTION_SAMPLES) {
            /* 认为到达限位：停止输出、把当前位置作为坐标原点，并清除积分项 */
            Set_Motor_Torque(0, CAN_CMD_ID, can_index);
            s->homing_state = HOMING_DONE; /* done */
            dm_target_angles[motor_index] = dm_angles[motor_index];
            dm_target_inited[motor_index] = true;
            /* 清除积分与历史误差，避免残留导致继续输出 */
            s->ang_integral = 0.0f;
            s->vel_integral = 0.0f;
            s->ang_prev_error = 0.0f;
            s->vel_prev_error = 0.0f;
            s->input_active = false;
            /* 初始化运行时限位检测参考 */
            s->limit_last_angle = dm_angles[motor_index];
            s->limit_no_motion_samples = 0;

            break; /* homing 完成，退出循环 */
        }

        /* 等待下一个采样周期再检查（使用全局 SAMPLE_DT，转换为 ms） */
        uint32_t ms = (uint32_t)(double_pid_cfg.SAMPLE_DT * 1000.0f);
        if (ms == 0) ms = 1;
        HAL_Delay(ms);
    }
}

// 返回 true 表示 homing 仍在进行中，false 表示 homing 已完成或被跳过
/* DoublePID_Homing_ProcessTick removed: homing is now a blocking operation
 * performed inside DoublePID_Homing_Start(). Callers should invoke
 * DoublePID_Homing_Start(...) which will only return when homing is done.
 */

bool DoublePID_Homing_IsDone(uint8_t motor_index)
{
    return (g_states[motor_index].homing_state == HOMING_DONE);
}

void DoublePID_Homing_Abort(uint8_t motor_index)
{
    DoublePID_State_t *s = &g_states[motor_index];
    /* 停止输出并标记为已完成 */
    s->homing_state = HOMING_DONE;
    s->input_active = false;
}

/* 简单限幅 */
static inline float clampf_s(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

/* homing/limit 配置统一从 config.c 的 homing_cfg 读取（在 config.h 中声明） */
extern Homing_Config_t homing_cfg;


/* 环绕差值函数已移除（未使用），保留注释以供未来参考 */

/* 更新速度/角度状态（由 CAN 接收回调处调用）
 * raw_ticks: 0..8191
 * rpm: 当前速度（rpm）
 */

void Double_PID_Handler(uint8_t motor_index, uint16_t raw_ticks, float rpm)
{

    DoublePID_State_t *s = &g_states[motor_index];

    /* 更新速度（导出为 rad/s） */
    dm_velocities[motor_index] = RPM_TO_RAD_S(rpm);

    /* 角度多圈展开：根据 raw_ticks 与上次 raw_ticks 差值来累计 turn_count */
    if (s->last_ticks_raw == 0 && s->turn_count == 0) {
        /* 首次调用：直接设置 last_ticks_raw，不产生 jump */
        s->last_ticks_raw = raw_ticks;
        dm_angles[motor_index] = TICKS_TO_RAD(raw_ticks);
        return;
    }

    int16_t diff = (int16_t)raw_ticks - (int16_t)s->last_ticks_raw;
    if (diff > 4096) {
        diff -= 8192;
    } else if (diff < -4096) {
        diff += 8192;
    }

    /* 更新多圈角度：以 ticks 差值转为 rad 并累加 */
    float d_rad = TICKS_TO_RAD((float)diff);
    dm_angles[motor_index] += d_rad;

    /* 更新 last ticks */
    s->last_ticks_raw = raw_ticks;
}

/* 仅计算 PID 输出并返回要发送的电压（不发送 CAN），与 main.c 保持一致的算法：
 * - 外环：以 angle error（ticks 风格）为基础，产生速度目标（与 main.c 中产出单位相同，经配置转换为 rad/s）
 * - 内环：以速度（rad/s）为输入执行 PID，返回 int16 电压
 */
int16_t Double_PID_ComputeSendVol(float target_rad, uint8_t motor_index)
{

    const DoublePID_Params_t *params = DoublePID_GetParams(motor_index);
    DoublePID_State_t *s = &g_states[motor_index];

    /* --- 外环（角度）计算（使用多圈角度直接计算，以支持大于一圈的目标） --- */
    /* 使用 dm_angles（多圈展开的角度，单位 rad）直接计算误差，支持用户传入如 2*pi、4*pi 的多圈目标。
     * 如果你想改为按“最近一圈”移动（短路路径），可以用注释部分把误差映射到 [-PI, PI]
     */
    float err_rad = target_rad - dm_angles[motor_index];

    /* // 可选：将误差映射到 [-PI, PI] 来使用最短路径（取消下面注释启用）
    float two_pi = 2.0f * PI;
    float rem = fmodf(err_rad, two_pi);
    if (rem > PI) rem -= two_pi;
    if (rem < -PI) rem += two_pi;
    err_rad = rem;
    */

    /* 使用配置的角度死区（rad 单位），保持外环死区行为可配置 */
    if (params->ENABLE_ANG_DEAD) {
        if (err_rad > -params->ANG_DEAD_ENTER && err_rad < params->ANG_DEAD_ENTER) {
            err_rad = 0.0f;
        }
    }

    /* 外环积分和导数 */
    s->ang_integral += err_rad * params->SAMPLE_DT;
    s->ang_integral = clampf_s(s->ang_integral, -params->ANG_INTEGRAL_LIMIT, params->ANG_INTEGRAL_LIMIT);

    float ang_derivative = (err_rad - s->ang_prev_error) / params->SAMPLE_DT;
    float ang_out = params->Kp_ang * err_rad + params->Ki_ang * s->ang_integral + params->Kd_ang * ang_derivative;

    s->ang_prev_error = err_rad;

    /* 将外环输出限制为配置的最大目标速度（注意 config 中存的是 rad/s） */
    ang_out = clampf_s(ang_out, -params->MAX_TARGET_VEL, params->MAX_TARGET_VEL);

    /*
     * 当外环计算出的速度目标非常小（例如小于 1 rpm）时，内环的速度死区可能会把它忽略，
     * 导致角度微小但实际不动。这里给一个最小非零速度保护：
     * 如果外环确实要求移动（err_rad != 0），并且 ang_out 非零但幅值太小，则提升到最小速度。
     * 这个值可以按需调小或调大（例如 0.5 rpm 或 2 rpm）。
     */
    float min_ang_out = RPM_TO_RAD_S(1.0f); /* 1 rpm -> rad/s */
    if (ang_out != 0.0f && fabsf(ang_out) < min_ang_out && fabsf(err_rad) > 0.0f) {
        ang_out = (ang_out > 0.0f) ? min_ang_out : -min_ang_out;
    }

    /* 保存外环调试信息 */
    dm_errors_ang[motor_index] = err_rad;
    dm_derivatives_ang[motor_index] = ang_derivative;
    dm_pid_outputs_ang[motor_index] = ang_out;

    /* --- 内环（速度）计算：与 main.c 的 Speed_PID_RPM 行为一致，但在 rad/s 单位下计算 --- */
    float setpoint_rad_s = ang_out; /* 外环直接产生 rad/s 目标 */
    float current_rad_s = dm_velocities[motor_index];

    /* main.c 对速度误差的死区为 2 rpm，但直接把小误差置零会导致小角度目标（如 PI/2、PI）有时被忽略。
     * 改进策略：
     *  - 把速度死区减小到 0.5 rpm，以减少被误杀的慢速命令；
     *  - 不直接把 vel_err 置零（会阻止慢速移动），而是在外环输出阶段对极小的速度目标应用一个最小非零速度，
     *    确保当外环确实要求移动时，内环能收到足够的速度目标以克服死区。
     */
    float vel_err = setpoint_rad_s - current_rad_s;

    /* 内环积分、导数（单位：rad*s / rad/s 等） */
    s->vel_integral += vel_err * params->SAMPLE_DT;
    /* main.c 的 integral clamp 为 10000 (rpm*s)，我们在 config 中已换算为 rad*s 并存到 single/inner config 的 INTEGRAL_LIMIT
     * 这里使用 double_pid_cfg 的 ANG_INTEGRAL_LIMIT 仅用于外环，内环使用 single_pid_cfg 样式的限制。
     */
    /* 但 double_pid_cfg 没有单独的 vel integral limit 字段，使用一个保守的上限：params->MAX_VOLTAGE 相关范围不可直接作为 integral 限幅，
     * 因此我们采用一个较大的限幅以避免积分爆炸（按 main.c 逻辑，内环积分上限为 10000 rpm*s，已在 single_pid_cfg 中设置）。
     */
    /* 这里我们将内环积分限幅为 RPM_TO_RAD_S(10000) 对应的 rad*s（与 single 的 INTEGRAL_LIMIT 保持一致） */
    float vel_integral_limit = RPM_TO_RAD_S(10000.0f);
    if (s->vel_integral > vel_integral_limit) s->vel_integral = vel_integral_limit;
    if (s->vel_integral < -vel_integral_limit) s->vel_integral = -vel_integral_limit;

    float vel_derivative = (vel_err - s->vel_prev_error) / params->SAMPLE_DT;
    s->vel_prev_error = vel_err;

    /* 使用 double_pid_cfg 中已经换算为 rad/s 单位的内环增益 */
    float vel_out = params->Kp * vel_err + params->Ki * s->vel_integral + params->Kd * vel_derivative;

    /* 限幅到驱动期望的电压范围 */
    vel_out = clampf_s(vel_out, -((float)params->MAX_VOLTAGE), (float)params->MAX_VOLTAGE);

    int16_t send = (int16_t)lroundf(vel_out);

    /* 保存内环调试信息 */
    dm_errors_vel[motor_index] = vel_err;
    dm_derivatives_vel[motor_index] = vel_derivative;
    dm_pid_outputs_vel[motor_index] = vel_out;
    dm_send_vols[motor_index] = send;

    return send;
}

/* 外部调用：目标角度以 rad 为单位（建议传入 0..2pi 单圈角），函数内部会计算并发送 CAN 命令 */
void Double_PID_ToAngle(float target_rad, uint32_t CAN_CMD_ID, uint8_t can_index, uint8_t motor_index, bool use_velocity_mode)
{
    /*
     * 改动说明（中文注释）：
     *   为满足“遥控器松手回中就立刻停下，而不是回去”的需求，
     *   把外部传入的参数当作“角速度 (rad/s)”（或角度变化量 / s）来处理，
     *   在函数内按采样周期累加到内部的目标角 `dm_target_angles`：
     *       dm_target += target_rad * SAMPLE_DT;
     *   这样当摇杆回中（传入 0）时，内部目标就停止变化而不是回到 0，电机会停在当前位置。
     *
     * 设计细节：
     * - 初次使用时把 dm_target 初始化为当前多圈角度 `dm_angles`，以避免跳变。
     * - 使用 double_pid_cfg.SAMPLE_DT 作为积分时间基准（与 PID 采样一致）。
     * - 这种方式等价于把摇杆作为速度控制（非绝对位置），更符合遥控操控感受。
     *
     * 注意：如果将来需要恢复“绝对角度目标”的行为，需增加一个显式的 API 或开关。
     */

    

    const DoublePID_Params_t *params = &double_pid_cfg;
    DoublePID_State_t *s = &g_states[motor_index];

    /* 获取当前电机的 homing 配置（可能是全局或每电机绑定的变体） */
    const Homing_Config_t *hc = DoublePID_GetHomingConfig(motor_index);

    /* 如果 homing 尚未完成，跳过闭环控制；注意现在 DoublePID_Homing_Start
     * 为阻塞式接口，因此在正常运行中 homing 应该已经完成。这里保守地
     * 检查状态以避免在特殊情况下进入闭环。 */
    if (!DoublePID_Homing_IsDone(motor_index)) {
        return;
    }

    /* ---------- 正常运行（homing 完成后） ---------- */
    /* 首次调用时用当前展开角初始化目标角（若 homing 已经完成或被跳过） */
    if (!dm_target_inited[motor_index]) {
        dm_target_angles[motor_index] = dm_angles[motor_index];
    dm_target_inited[motor_index] = true;
        /* 初始化限位检测参考点 */
        s->limit_last_angle = dm_angles[motor_index];
        s->limit_no_motion_samples = 0;
    }
    /* 两种输入模式：
     * - use_velocity_mode == true  : 把传入的 target_rad 当作角速度 (rad/s)，按 SAMPLE_DT 积分到内部目标（遥控器/摇杆模式，保持原有行为）
     * - use_velocity_mode == false : 把传入的 target_rad 当作绝对角度 (rad，多圈展开)，直接设置内部目标并清除积分/历史误差（就绪/绝对角度模式）
     */
    int16_t vol = 0;
    if (use_velocity_mode) {
        /* 遥控器/速度模式（兼容既有逻辑） */
        float input_deadzone = RPM_TO_RAD_S(0.5f); /* 0.5 rpm 等效阈值，视需调整 */

        if (fabsf(target_rad) <= input_deadzone) {
            /* 摇杆回中或命令近似为零 */
            if (s->input_active) {
                /* 刚刚从有输入变为无输入：锁定目标并清除积分/历史误差 */
                dm_target_angles[motor_index] = dm_angles[motor_index];
                s->ang_integral = 0.0f;
                s->vel_integral = 0.0f;
                s->ang_prev_error = 0.0f;
                s->vel_prev_error = 0.0f;
                s->input_active = false;
                /* 重置限位检测计数 */
                s->limit_no_motion_samples = 0;
                s->limit_last_angle = dm_angles[motor_index];
            }
            /* 已经处于无输入状态：保持 dm_target 不变（驻点） */
        } else {
            /* 有非零输入：正常按速度积分到目标角 */
            s->input_active = true;
            float delta = target_rad * params->SAMPLE_DT;
            dm_target_angles[motor_index] += delta;
        }

        /* 调用原有的计算发送电压接口（使用累加后的绝对目标角） */
        vol = Double_PID_ComputeSendVol(dm_target_angles[motor_index], motor_index);
    } else {
        /* 绝对角度模式：直接把传入的 target_rad 作为内部目标（多圈 rad），进入 ready/就绪行为 */
        dm_target_angles[motor_index] = target_rad;
        dm_target_inited[motor_index] = true;
        /* 进入就绪模式时清除积分与历史误差，避免残留动作 */
        s->ang_integral = 0.0f;
        s->vel_integral = 0.0f;
        s->ang_prev_error = 0.0f;
        s->vel_prev_error = 0.0f;
        s->input_active = false;
        /* 重置限位检测参考 */
        s->limit_last_angle = dm_angles[motor_index];
        s->limit_no_motion_samples = 0;

        vol = Double_PID_ComputeSendVol(dm_target_angles[motor_index], motor_index);
    }

    /* ---------- 运行时限位检测：当遥控器持续推到限位且角度不变时停输出 ---------- */
    /* 仅在遥控器/速度输入模式下并且有 input_active 且输出量较大（说明在持续用力）时才计数 */
    if (use_velocity_mode && s->input_active && (abs((int)vol) >= hc->MIN_PUSH_VOL)) {
        float d = fabsf(dm_angles[motor_index] - s->limit_last_angle);
        if (d > hc->HOMING_MOTION_THRESHOLD_RAD) {
            /* 仍在移动，重置计数并记住最新位置 */
            s->limit_last_angle = dm_angles[motor_index];
            s->limit_no_motion_samples = 0;
        } else {
            s->limit_no_motion_samples++;
        }

        if (s->limit_no_motion_samples >= hc->LIMIT_NO_MOTION_SAMPLES) {
            /* 认为被推到物理限位：停止输出并锁定目标点 */
            Set_Motor_Torque(0, CAN_CMD_ID, can_index);
            s->input_active = false;
            dm_target_angles[motor_index] = dm_angles[motor_index];
            s->ang_integral = 0.0f;
            s->vel_integral = 0.0f;
            s->ang_prev_error = 0.0f;
            s->vel_prev_error = 0.0f;
            s->limit_no_motion_samples = 0;
            s->limit_last_angle = dm_angles[motor_index];
            return;
        }
    } else {
        /* 若输出很小或无输入，或处于绝对角度模式下，不做限位计数（不应误触发） */
        s->limit_no_motion_samples = 0;
        s->limit_last_angle = dm_angles[motor_index];
    }

    /* 最终发送计算出的电压 */
    Set_Motor_Torque(vol, CAN_CMD_ID, can_index);
}
