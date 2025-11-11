#define PI 3.14159265358979323846f

/* 速度单位转换：RPM <-> rad/s */
#define RPM_TO_RAD_S(rpm) ((rpm) * 2.0f * PI / 60.0f)
#define RAD_S_TO_RPM(rad_s) ((rad_s) * 60.0f / (2.0f * PI))

/* 角度单位转换：ticks(0-8191) <-> rad */
#define TICKS_TO_RAD(ticks) ((ticks) * 2.0f * PI / 8192.0f)
#define RAD_TO_TICKS(rad) ((rad) * 8192.0f / (2.0f * PI))

/* 将 PID 增益在不同单位间换算的辅助函数声明
 *  - Speed/PID (rpm <-> rad/s) 的增益转换: 当原始增益以 rpm 为输入单位、输出为驱动量时，
 *    要用于 rad/s 输入的 PID 增益需乘以 60/(2*pi)。
 *  - Position/PID (ticks <-> rad) 的增益转换: 当原始外环增益以 (rpm / tick) 为单位时，
 *    要用于 rad -> rad/s 输出的增益需乘以 (TICKS_PER_TURN / 60).
 */
#define Gain_RPM_to_RADs(gain_rpm_unit) ((gain_rpm_unit) * (60.0f / (2.0f * PI)))
#define Gain_TICKS_to_RAD(gain_ticks_unit) ((gain_ticks_unit) * (8192.0f / 60.0f))



