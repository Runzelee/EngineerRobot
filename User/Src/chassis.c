#ifndef __DISABLE_CHASSIS_TASK__

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
  * @brief  Function implementing the Chassis_Task thread.
  * @param  argument: Not used
  * @retval None
  */
void ChassisTask(void *argument)
{

    Motor_CAN_RegisterAngleVelocityCallback(Single_PID_Handler);
    extern RC_ctrl_t rc_ctrl;

    /* Infinite loop */
    for(;;)
    {
        if(rc_ctrl.s[1] == 1){
            // 停止电机
            for (uint8_t i = 0; i < MOTOR_SINGLE_PID_NUM; ++i) {
                Set_Motor_Torque(0, 0x200, i + 1);
            }
        }else{
            //Single_PID_ToVelocity(100, 0x1FF, 3, 2);
            // int16_t vols[4];
            // vols[0] = Single_PID_ComputeSendVol(rc_ctrl.ch[0]/5, 1);
            // vols[1] = Single_PID_ComputeSendVol(rc_ctrl.ch[3]/2, 2);
            // 一次性打包发送四个电机的扭矩
            // Set_Motors_Torque(vols, 0x1FF);
            Mecanum_Apply(rc_ctrl.ch[1]/100.0f, rc_ctrl.ch[0]/100.0f, rc_ctrl.ch[2]/100.0f, 0x200, 1);
        }

    // 示例（已改为使用 rad 单位，motor_index 从 0 开始）：
    // Double_PID_ToAngle(TICKS_TO_RAD(8192), 0x200, 1, 0);

        //HAL_Delay(50);
        //Set_Motor_Torque(8000, 0x200, 1);
        osDelay(10);
    }
}

#endif /* __DISABLE_CHASSIS_TASK__ */
