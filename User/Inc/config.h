#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* 单环速度 PID 参数结构体（供 Single_PID_ToVelocity 使用） */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float SAMPLE_DT;      // 秒
    int16_t MAX_VOLTAGE;  // 输出上限
    float ERROR_DEADZONE; // 误差死区范围
    float INTEGRAL_LIMIT; // 积分限幅
    bool ENABLE_UART_DEBUG;
} SinglePID_Params_t;

/* 双环角度-速度 PID 参数结构体（供 Double_PID_ToAngle / Handler 使用） */
typedef struct {
    float ANG_DEAD_ENTER;
    float ANG_DEAD_EXIT;
    /* 速度环 PID */
    float Kp;
    float Ki;
    float Kd;
    float SAMPLE_DT;      // 秒
    int16_t MAX_VOLTAGE;  // 输出上限
    /* 角度外环 PID */
    float Kp_ang;
    float Ki_ang;
    float Kd_ang;
    float MAX_TARGET_VEL; /* 角度环输出速度上限（rpm） */
    float ANG_INTEGRAL_LIMIT; /* 角度环积分限幅（单位：rad*s，已由配置转换） */
    bool ENABLE_ANG_DEAD;
    bool ENABLE_UART_DEBUG;
} DoublePID_Params_t;

/* 全局默认配置实例（在 properties.c 中定义并初始化） */
extern SinglePID_Params_t single_pid_cfg;
extern DoublePID_Params_t double_pid_cfg;

/* 可选的额外 PID 配置示例：外部代码可通过地址把不同配置分配给不同电机 */
extern DoublePID_Params_t double_pid_cfg_variant1;
extern DoublePID_Params_t double_pid_cfg_variant2;

/* 上电/限位与 homing 相关的配置统一放到结构体中，便于集中管理与修改 */
typedef struct {
    bool ENABLE_HOMING; /* 是否启用上电 homing 行为（bool 开关） */
    int16_t HOMING_TORQUE; /* 上电向负方向施加的固定扭矩（int16 单位），负值为向负方向） */
    float HOMING_MOTION_THRESHOLD_RAD; /* 认为有运动的阈值（rad） */
    uint16_t HOMING_NO_MOTION_SAMPLES; /* 连续多少个采样无运动则判定为到达物理限位 */

    uint16_t LIMIT_NO_MOTION_SAMPLES; /* 运行时遥控器推动到限位时的计数阈值 */
    int16_t MIN_PUSH_VOL; /* 判断为持续向限位用力的最小输出阈值（int16 单位） */
} Homing_Config_t;

extern Homing_Config_t homing_cfg;
extern Homing_Config_t homing_cfg_variant1;
extern Homing_Config_t homing_cfg_variant2;
extern Homing_Config_t homing_cfg_variant3;

/* Gimbal（云台）统一配置：电机映射与 ready 目标角度（单位：rad）
 * 将具体的 motor_index / can_index / can_cmd_id 放在这里，便于集中管理。
 */
typedef struct {
    uint8_t lift_motor_index;  /* lift 对应的 motor_index（例如 5） */
    uint8_t pitch_motor_index; /* pitch 对应的 motor_index（例如 6） */
    uint8_t yaw_motor_index;   /* yaw 对应的 motor_index（例如 7） */

    uint8_t lift_can_index;    /* lift 在 CAN 数据域中的起始索引（用于 Set_Motor_Torque） */
    uint8_t pitch_can_index;
    uint8_t yaw_can_index;

    uint32_t can_cmd_id;       /* 三路共用的 CAN 命令 ID（通常 0x1FF） */

    /* ready 状态下的目标角度（相对于 reset 后记录的新坐标系零点，单位 rad） */
    float lift_ready_target_rad; /* 若 lift 用 homing 完成定位，可忽略 */
    float pitch_ready_target_rad;
    float yaw_ready_target_rad;

    /* 指定在 reset/ready 时各路使用的 homing 配置指针（可为 NULL -> 使用全局 homing_cfg） */
    const Homing_Config_t *lift_reset_homing;
    const Homing_Config_t *pitch_reset_homing;
    const Homing_Config_t *yaw_reset_homing;

    const Homing_Config_t *lift_ready_homing; /* ready 状态中 lift 可能使用不同 homing */
} Gimbal_Config_t;

extern Gimbal_Config_t gimbal_cfg;

#endif /* CONFIG_H */
