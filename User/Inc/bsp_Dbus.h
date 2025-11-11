#ifndef _BSP_DBUS_H_
#define _BSP_DBUS_H_
#include "struct_typedef.h"
#include "main.h"

extern void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);


#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)



/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)


/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)

#define Rc_Offline_Max     3
/* ----------------------- Data Struct ------------------------------------- */
typedef struct control_struct
{
	int16_t ch[5];
    uint8_t s[2];        
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t press_l;
    uint8_t press_r;
    uint8_t press_l_hop;//鼠标左键跳变
	uint8_t press_r_hop;//鼠标右键跳变	
     
    uint16_t v;
    uint8_t		init_flag;

	uint8_t		offline_cnt;
	uint8_t		offline_max_cnt;
	
	void    (*heart_beat)(struct control_struct *self);	// 设备心跳包
    void    (*clear_cnt )(struct control_struct *self);//清除计数值
} RC_ctrl_t;

/* ----------------------- Internal Data ----------------------------------- */

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
extern void DBus_Control_init(void);
void USART3_user_IRQHandler(void);

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
  
 RC_ctrl_t *get_remote_control_point(void);
extern RC_ctrl_t rc_ctrl ;

static void rc_heart_beat(RC_ctrl_t *rc_data);
static void rc_clear_cnt(RC_ctrl_t *rc_data);

extern void USART3_user_IRQHandler(void);
extern uint8_t Key_Check(uint16_t Key);
#endif
