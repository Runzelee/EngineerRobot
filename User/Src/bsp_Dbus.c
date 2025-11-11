#include  "bsp_Dbus.h"
/**
  ***************************COPYRIGHT 2023 Architect***************************
  * @file       bsp_Dbus.c/h
  * @brief      D-Bus文件,负责D-Bus接收和解析
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ***************************COPYRIGHT 2024 Architect***************************
  */
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

RC_ctrl_t rc_ctrl ={
        .heart_beat = &rc_heart_beat,
        .clear_cnt = &rc_clear_cnt,
        .offline_max_cnt = Rc_Offline_Max,
};


void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);

}


/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//remote control data 
//遥控器控制变量


//receive data, 18 bytes one frame, but set 36 bytes 
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
  


  
void DBus_Control_init(void)
{
     
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
//	rc=get_remote_control_point();
	//rc->rc.ch[0]
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
 RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}


//串口中断
void USART3_user_IRQHandler(void)
{
	
	rc_ctrl.clear_cnt(&rc_ctrl);
    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
    
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
    }
}
static void rc_heart_beat(RC_ctrl_t *rc_data)
{
	rc_data->offline_cnt++;      //正常连接时会在can中断中清零
	if(rc_data->offline_cnt > rc_data->offline_max_cnt) {
		rc_data->offline_cnt = rc_data->offline_max_cnt;
		
		rc_data->init_flag = 1;
	}

}
static void rc_clear_cnt(RC_ctrl_t *rc_data)
{
    rc_data->offline_cnt = 0;
	rc_data->init_flag = 0;
}

/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        	//!< Channel 0
    rc_ctrl->ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; 	//!< Channel 1
    rc_ctrl->ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          	//!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; 	//!< Channel 3
    rc_ctrl->s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  		//!< Switch left
    rc_ctrl->s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                     //!< Switch right
    rc_ctrl->x = sbus_buf[6] | (sbus_buf[7] << 8);                    		//!< Mouse X axis
    rc_ctrl->y = sbus_buf[8] | (sbus_buf[9] << 8);                    		//!< Mouse Y axis
    rc_ctrl->z = sbus_buf[10] | (sbus_buf[11] << 8);                 		//!< Mouse Z axis
    rc_ctrl->press_l = sbus_buf[12];                                		//!< Mouse Left Is Press ?
    rc_ctrl->press_r = sbus_buf[13];                                  		//!< Mouse Right Is Press ?
    rc_ctrl->v = sbus_buf[14] | (sbus_buf[15] << 8);                    	//!< KeyBoard value
    rc_ctrl->ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 	//!< Channel 4

    rc_ctrl->ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->ch[4] -= RC_CH_VALUE_OFFSET;
	
	if( rc_ctrl->ch[0]>-20&&rc_ctrl->ch[0]<20)	rc_ctrl->ch[0]=0;
	if( rc_ctrl->ch[1]>-20&&rc_ctrl->ch[1]<20)  rc_ctrl->ch[1]=0;
	if( rc_ctrl->ch[2]>-20&&rc_ctrl->ch[2]<20)  rc_ctrl->ch[2]=0;
	if( rc_ctrl->ch[3]>-20&&rc_ctrl->ch[3]<20)  rc_ctrl->ch[3]=0;
}

/**
  * @brief  判断一个键是否被按下
  * @param  要进行判断的按键，字符大写
  * @retval 1 按下        0 未按下
  */
uint8_t Key_Check(uint16_t Key)
{
	if(rc_ctrl.v & Key)	return 1;
	else				return 0;
}
