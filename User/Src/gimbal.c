#ifndef __DISABLE_GIMBAL_TASK__

#include "cmsis_os.h"
#include "motor_double_pid.h"
#include "motor_single_pid.h"
#include "motor_can.h"
#include "config.h"
#include "unit_conversation.h"
#include "bsp_Dbus.h"
#include "stdio.h"
#include "mecanum.h"

/**
* @brief Function implementing the Gimbal_Task thread.
* @param argument: Not used
* @retval None
*/
void GimbalTask(void *argument)
{

    Motor_CAN_RegisterAngleVelocityCallback(Double_PID_Handler);
    extern RC_ctrl_t rc_ctrl;

    /* Infinite loop */
    for(;;)
    {
        if(rc_ctrl.s[1] == 1){
            // 停止电机
            Set_Motor_Torque(0, 0x1FF, 1);
        } else {
            /* 这里把传入值视为摇杆角速度（rad/s），使用速度模式（true）以保持遥控操控感受 */
            Double_PID_ToAngle(rc_ctrl.ch[0]/150.0f * 2 * PI, 0x1FF, 1, 5, true);
        }
        osDelay(10);
    }

}

#endif /* __DISABLE_GIMBAL_TASK__ */
