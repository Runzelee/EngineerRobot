#include "motor_can.h"
#include "can.h"

//#include "cmsis_os.h"

static Motor_CAN_AngleVelocityCallback_t Motor_CAN_AngleVelocityCallback = NULL;

/**
 * @brief 注册回调函数以接收电机的角度和速度数据
 * @param callback 回调函数指针，Single_PID_Handler 或 Double_PID_Handler 二选一
 */
void Motor_CAN_RegisterAngleVelocityCallback(Motor_CAN_AngleVelocityCallback_t callback)
{
    Motor_CAN_AngleVelocityCallback = callback;
}

/**
 * @brief 配置并启动 CAN1，设置接收过滤器以接标准帧消息
 * @param CAN_CMD_ID 期望接收的 CAN ID
 */
void Enable_CAN1(uint32_t CAN_CMD_ID)
{
    CAN_FilterTypeDef CAN_Filter;
    CAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    CAN_Filter.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_Filter.FilterBank = 0;
    CAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_Filter.SlaveStartFilterBank = 10;
    CAN_Filter.FilterActivation = CAN_FILTER_ENABLE;
    CAN_Filter.FilterIdHigh = (uint32_t)(CAN_CMD_ID<<5) >> 16;
    CAN_Filter.FilterIdLow = 0x0000;
    CAN_Filter.FilterMaskIdHigh= (uint32_t)(0x7FF<<5) >> 16;  // 只匹配标准ID的前11位
    CAN_Filter.FilterMaskIdLow = 0x0000;
    if (HAL_CAN_ConfigFilter(&hcan1,&CAN_Filter)!= HAL_OK){
        Error_Handler();
    }
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);  // CAN1 -> FIFO0
}

/**
 * @brief 通过 CAN 总线发送电机控制命令
 * @param vol 期望扭矩值，M3508与GM6020范围均为-16384到+16384
 * @param CAN_CMD_ID 电机控制命令的 CAN ID，GM6020为0x1FF，M3508为0x200
 * @param index 数据域低位索引，一般与电机ID一致，发送给第 index 和 index-1 字节
 */
void Set_Motor_Torque(int16_t vol, uint32_t CAN_CMD_ID, uint8_t index)
{
    uint8_t TxData[8] = {0}; // 清零
    TxData[index - 1] = (uint8_t)(vol>>8);
    TxData[index] = (uint8_t)vol;
    CAN_TxHeaderTypeDef TxHeader = {
            .DLC = 8,
            .IDE = CAN_ID_STD,    // 标准帧
            .RTR = CAN_RTR_DATA,  // 数据帧
            .StdId = CAN_CMD_ID
    };
    uint32_t TxBox = CAN_TX_MAILBOX0;
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxBox) != HAL_OK){
        //HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);//错误处理
    }
}

// 新增：一次性发送 4 个电机的扭矩，避免轮询/阻塞多次调用 HAL_CAN_AddTxMessage
// 按照原 Set_Motor_Torque 的索引语义进行填充：每个电机使用两个字节，低位索引由 start_index 开始
void Set_Motors_Torque(const int16_t vols[4], uint32_t CAN_CMD_ID)
{
    uint8_t TxData[8] = {0};
    // 使用与 Set_Motor_Torque 相同的映射规则填充数据域
    // 按固定映射：bytes 0-1 -> motor1, 2-3 -> motor2, 4-5 -> motor3, 6-7 -> motor4
    for (int i = 0; i < 4; ++i) {
        int base = 2 * i;
        TxData[base]     = (uint8_t)((vols[i] >> 8) & 0xFF); // 高字节
        TxData[base + 1] = (uint8_t)(vols[i] & 0xFF);        // 低字节
    }

    CAN_TxHeaderTypeDef TxHeader = {
            .DLC = 8,
            .IDE = CAN_ID_STD,
            .RTR = CAN_RTR_DATA,
            .StdId = CAN_CMD_ID
    };
    uint32_t TxBox = CAN_TX_MAILBOX0;
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxBox) != HAL_OK){
        // 错误处理：这里保持与原实现一致（暂不阻塞或重试），可根据需要扩展重试或队列
    }
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
  if (hcan == &hcan1)
  {
      CAN_RxHeaderTypeDef RxHeader;
      uint8_t RxData[8];
      if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)  // 获得接收到的数据头和数据
      {
          // 角度为无符号 0..8191（13-bit），高字节在 RxData[0], 低字节在 RxData[1]
          uint16_t raw_angle = (uint16_t)(((uint16_t)RxData[0] << 8) | (uint16_t)RxData[1]);
          raw_angle &= 0x1FFF; // 保证 13-bit（0..8191）

          // 速度为有符号 int16，高字节放在 RxData[2], 低字节在 RxData[3]
          int16_t raw_vel = (int16_t)(((uint16_t)RxData[2] << 8) | (uint16_t)RxData[3]);
          float velocity = (float)raw_vel;

          /*
           * 按 CAN 标准 ID 分流到对应电机（常见电机反馈 ID 为 0x201..0x204）
           * 如果收到的 ID 在该范围内，则计算 motor_index（0..3）并回调。
           * 若收到其它 ID，则忽略（或者可以根据需求扩展映射表）。
           */
          /*
           * 将不同厂商/固件的电机反馈 StdId 映射为 motor_index（0..3）
           * 常见映射：
           *  - GM6020 等返回 ID 0x205..0x208 -> 对应电机 0..3
           *  - M3508 等返回 ID 0x201..0x204 -> 对应电机 0..3
           * 之前的实现中存在错误比较（把 CAN_ID_STD 与具体 ID 比较），导致回调从未被触发。
           */
          uint8_t motor_index = 0xFF;
          if (RxHeader.IDE == CAN_ID_STD) {
                motor_index = (uint8_t)(RxHeader.StdId - 0x200);
                //motor_index = (uint8_t)(RxHeader.StdId - 0x201); // 底盘
                //motor_index = (uint8_t)(RxHeader.StdId - 0x205);
						
								if(RxHeader.StdId == 0x205){
									raw_angle &= 0xFFFF;
								}
          }
					
					

          if (motor_index != 0xFF && Motor_CAN_AngleVelocityCallback != NULL) {
              Motor_CAN_AngleVelocityCallback(motor_index, raw_angle, velocity);
          }
      }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);  // 再次使能FIFO0接收中断
}
