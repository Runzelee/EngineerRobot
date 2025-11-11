#include "config.h"

#include "unit_conversation.h"


SinglePID_Params_t single_pid_cfg = {
    .Kp = 20.0,
    .Ki = 10.0f,
    .Kd = 0.0f,
    .SAMPLE_DT = 0.01f,    /* 10 ms 主循环 */
    .MAX_VOLTAGE = 10000,
    .ERROR_DEADZONE = 2.0f, // 误差死区范围
    .INTEGRAL_LIMIT = 50000.0f, // 积分限幅
    .ENABLE_UART_DEBUG = true
};

// SinglePID_Params_t single_pid_cfg = {
//     .Kp = 15.0f,
//     .Ki = 5.0f,
//     .Kd = 0.0f,
//     .SAMPLE_DT = 0.05f,    /* 50 ms 主循环 */
//     .MAX_VOLTAGE = 20000,
//     .ENABLE_UART_DEBUG = true
// };

/* 双环 PID 默认参数（可按需调整） */
/* 双环 PID 参数（基于 main.c 的 Position + Speed PID，已按单位转换为 rad/rad_s）
 * main.c 外环: pp_kp=0.5, pp_ki=0.3, pp_kd=0.0, deadzone 5 ticks, integral clamp 1000 ticks
 * main.c 内环: sp_kp=5.0, sp_ki=1.3, sp_kd=0.0, deadzone 2 rpm, integral clamp 10000
 * 我们把外环增益转换为以 rad 为输入，输出为 rad/s 的增益；内环增益按 RPM->rad/s 换算。
 */
DoublePID_Params_t double_pid_cfg = {
    /* 角度死区（使用 main.c 对应的 5 ticks） */
    .ANG_DEAD_ENTER = TICKS_TO_RAD(10.0f),
    .ANG_DEAD_EXIT = TICKS_TO_RAD(10.0f),
    /* 速度环 PID（内环），将 main 的 sp_* 换算到 rad/s 单位 */
    .Kp = 20.0f, /* 5.0 * 60 / (2*pi) */
    .Ki = 10.0f, /* 1.3 * 60 / (2*pi) */
    .Kd = 0,
    .SAMPLE_DT = 0.01f,      /* 10 ms 主循环 */
    .MAX_VOLTAGE = 16384,
    /* 角度环 PID（外环）: 将 main 的 pp_* (单位: rpm per tick) 换算为 (rad/s) per rad */
    .Kp_ang = 50.0f,
    .Ki_ang = 30.0f,
    .Kd_ang = 0,
    .MAX_TARGET_VEL = RPM_TO_RAD_S(1000.0f), /* main 外环输出限幅 5000 rpm -> rad/s */
    .ANG_INTEGRAL_LIMIT = TICKS_TO_RAD(500.0f), /* main integral clamp 1000 ticks -> rad */
    .ENABLE_ANG_DEAD = true,
    .ENABLE_UART_DEBUG = false,
};

/* 额外的双环 PID 配置示例，供不同电机使用（演示） */
DoublePID_Params_t double_pid_cfg_variant1 = {
    .ANG_DEAD_ENTER = TICKS_TO_RAD(2.0f),
    .ANG_DEAD_EXIT = TICKS_TO_RAD(4.0f),
    .Kp = 20.0f,
    .Ki = 10.0f,
    .Kd = 0,
    .SAMPLE_DT = 0.01f,
    .MAX_VOLTAGE = 2000,
    .Kp_ang = 30.0f,
    .Ki_ang = 20.0f,
    .Kd_ang = 0,
    .MAX_TARGET_VEL = RPM_TO_RAD_S(1000.0f),
    .ANG_INTEGRAL_LIMIT = TICKS_TO_RAD(500.0f),
    .ENABLE_ANG_DEAD = true,
    .ENABLE_UART_DEBUG = false,
};

DoublePID_Params_t double_pid_cfg_variant2 = {
    .ANG_DEAD_ENTER = TICKS_TO_RAD(20.0f),
    .ANG_DEAD_EXIT = TICKS_TO_RAD(20.0f),
    .Kp = 10.0f,
    .Ki = 2.0f,
    .Kd = 0,
    .SAMPLE_DT = 0.01f,
    .MAX_VOLTAGE = 16384,
    .Kp_ang = 80.0f,
    .Ki_ang = 40.0f,
    .Kd_ang = 0,
    .MAX_TARGET_VEL = RPM_TO_RAD_S(200.0f),
    .ANG_INTEGRAL_LIMIT = TICKS_TO_RAD(300.0f),
    .ENABLE_ANG_DEAD = true,
    .ENABLE_UART_DEBUG = false,
};

/* homing / limit 相关默认配置 */
Homing_Config_t homing_cfg = {
    .ENABLE_HOMING = true,
    .HOMING_TORQUE = -500,
    .HOMING_MOTION_THRESHOLD_RAD = TICKS_TO_RAD(10.0f),
    .HOMING_NO_MOTION_SAMPLES = 10,
    .LIMIT_NO_MOTION_SAMPLES = 50,
    .MIN_PUSH_VOL = 200,
};

/* 示例：多个 homing 配置实例（供不同电机使用） */
Homing_Config_t homing_cfg_variant1 = {
    .ENABLE_HOMING = true,
    .HOMING_TORQUE = 500,
    .HOMING_MOTION_THRESHOLD_RAD = TICKS_TO_RAD(10.0f),
    .HOMING_NO_MOTION_SAMPLES = 1000, //10
    .LIMIT_NO_MOTION_SAMPLES = 50,
    .MIN_PUSH_VOL = 200,
};

Homing_Config_t homing_cfg_variant2 = {
    .ENABLE_HOMING = true,
    .HOMING_TORQUE = -5000,
    .HOMING_MOTION_THRESHOLD_RAD = TICKS_TO_RAD(10.0f),
    .HOMING_NO_MOTION_SAMPLES = 10,
    .LIMIT_NO_MOTION_SAMPLES = 60,
    .MIN_PUSH_VOL = 250,
};

Homing_Config_t homing_cfg_variant3 = {
    .ENABLE_HOMING = true,
    .HOMING_TORQUE = 5000,
    .HOMING_MOTION_THRESHOLD_RAD = TICKS_TO_RAD(10.0f),
    .HOMING_NO_MOTION_SAMPLES = 10,
    .LIMIT_NO_MOTION_SAMPLES = 60,
    .MIN_PUSH_VOL = 250,
};

/* 默认的云台配置（按任务说明：pitch、yaw、lift）
 * - motor_index: lift=5, pitch=6, yaw=7
 * - can_index: 对应数据域起始索引（示例值）: lift=5->index 5? 原项目里常用 1/3/5 作为 can data start
 *   这里按用户描述使用 1/3/5 对应 motor_index 5/6/7。can_cmd_id 取 0x1FF。
 * - ready 目标角度以 0.0f 填充，用户会根据需要修改
 */
Gimbal_Config_t gimbal_cfg = {
    /* 按题主说明：pitch, yaw, lift 的 motor_index 分别为 5,6,7；can_index 分别为 1,3,5 */
    .pitch_motor_index = 5,
    .yaw_motor_index = 6,
    .lift_motor_index = 7,

    .pitch_can_index = 1,
    .yaw_can_index = 3,
    .lift_can_index = 5,

    .can_cmd_id = 0x1FFu,

    .lift_ready_target_rad = 0.0f,
    .pitch_ready_target_rad = TICKS_TO_RAD(1000.0f),
    .yaw_ready_target_rad = -TICKS_TO_RAD(2380.0f),

    /* 默认 homing 配置：lift 使用全局 homing_cfg，pitch/yaw 使用示例变体，可由用户在 config.c 内调整 */
    .lift_reset_homing = &homing_cfg,
    .pitch_reset_homing = &homing_cfg_variant2,
    .yaw_reset_homing = &homing_cfg_variant3,

    .lift_ready_homing = &homing_cfg_variant1,
};

// DoublePID_Params_t double_pid_cfg = {
//     /* 角度死区（使用 main.c 对应的 5 ticks） */
//     .ANG_DEAD_ENTER = TICKS_TO_RAD(10.0f),
//     .ANG_DEAD_EXIT = TICKS_TO_RAD(10.0f),
//     /* 速度环 PID（内环），将 main 的 sp_* 换算到 rad/s 单位 */
//     .Kp = 40.0f, /* 5.0 * 60 / (2*pi) */
//     .Ki = 10.0f, /* 1.3 * 60 / (2*pi) */
//     .Kd = 0,
//     .SAMPLE_DT = 0.01f,      /* 10 ms 主循环 */
//     .MAX_VOLTAGE = 16384,
//     /* 角度环 PID（外环）: 将 main 的 pp_* (单位: rpm per tick) 换算为 (rad/s) per rad */
//     .Kp_ang = 60.0f,
//     .Ki_ang = 40.0f,
//     .Kd_ang = 0,
//     .MAX_TARGET_VEL = RPM_TO_RAD_S(5000.0f), /* main 外环输出限幅 5000 rpm -> rad/s */
//     .ANG_INTEGRAL_LIMIT = TICKS_TO_RAD(1000.0f), /* main integral clamp 1000 ticks -> rad */
//     .ENABLE_ANG_DEAD = true,
//     .ENABLE_UART_DEBUG = true,
// };

// DoublePID_Params_t double_pid_cfg = {
//     .SINGLE_TURN_TICKS = 8192u,
//     .ANG_DEAD_ENTER = 500.0f,
//     .ANG_DEAD_EXIT = 600.0f,
//     /* 速度环 PID（内环） */
//     .Kp = 15.0f,
//     .Ki = 5.0f,
//     .Kd = 0.0f,
//     .SAMPLE_DT = 0.05f,      /* 50 ms 主循环 */
//     .MAX_VOLTAGE = 20000,
//     /* 角度环 PID（外环） */
//     .Kp_ang = 15.0f,
//     .Ki_ang = 5.0f,
//     .Kd_ang = 0.0f,
//     .MAX_TARGET_VEL = 200.0f,
//     .ENABLE_UART_DEBUG = true
// };
