#ifndef	FENG_GPIOTE_H
#define	FENG_GPIOTE_H

#define TP_SCL_PIN             (5)    // SCL signal pin 5
#define TP_SDA_PIN             (6)    // SDA signal pin 6

#define TP_EINT_PIN             (9)    //interupt signal pin
#define TP_RSET_PIN             (10)   //reset signal pin
 
#define TP_I2C_ADDRESS	 	0x15
#define TP_CHIP_ID		  	0xB4

#define TP_REG_GESTURE				0x01
#define TP_REG_FINGER_NUM			0x02
#define TP_REG_XPOS_H				0x03
#define TP_REG_XPOS_L				0x04
#define TP_REG_YPOS_H				0x05
#define TP_REG_YPOS_L				0x06
#define TP_REG_BPC0_H				0xB0
#define TP_REG_BPC0_L				0xB1
#define TP_REG_BPC1_H				0xB2
#define TP_REG_BPC1_L				0xB3
#define TP_REG_CHIPID				0xA7
#define TP_REG_PROJID				0xA8
#define TP_REG_FW_VER				0xA9
#define TP_REG_SLEEP_MODE			0xE5
#define TP_REG_ERR_RESET			0xEA
#define TP_REG_LONG_PRESS_TICK		0xEB
#define TP_REG_MOTION_MASK			0xEC
#define TP_REG_IRQ_PLUSE_WIDTH		0xED
#define TP_REG_NOR_SCAN_PER			0xEE
#define TP_REG_MOTION_SL_ANGLE		0xEF
#define TP_REG_LP_SCAN_RAW1_H		0xF0
#define TP_REG_LP_SCAN_RAW1_L		0xF1
#define TP_REG_LP_SCAN_RAW2_H		0xF2
#define TP_REG_LP_SCAN_RAW2_L		0xF3
#define TP_REG_LP_AUTO_WAKE_TIME	0xF4
#define TP_REG_LP_SCAN_TH			0xF5
#define TP_REG_LP_SCAN_WIN			0xF6
#define TP_REG_LP_SCAN_FREQ			0xF7
#define TP_REG_LP_SCAN_IDAC			0xF8
#define TP_REG_AUTO_SLEEP_TIME		0xF9
#define TP_REG_IRQ_CTL				0xFA
#define TP_REG_AUTO_RESET			0xFB
#define TP_REG_LONG_PRESS_TIME		0xFC
#define TP_REG_IO_CTL				0xFD
#define TP_REG_DIS_AUTO_SLEEP		0xFE

#define GESTURE_NONE			0x00
#define GESTURE_MOVING_DOWN		0x01
#define GESTURE_MOVING_UP		0x02
#define GESTURE_MOVING_LEFT		0x03
#define GESTURE_MOVING_RIGHT	0x04
#define GESTURE_SINGLE_CLICK	0x05
#define GESTURE_DOUBLE_CLICK	0x0B
#define GESTURE_LONG_PRESS		0x0C

typedef enum
{
	TP_EVENT_MOVING_UP,
	TP_EVENT_MOVING_DOWN,
	TP_EVENT_MOVING_LEFT,
	TP_EVENT_MOVING_RIGHT,
	TP_EVENT_SINGLE_CLICK,
	TP_EVENT_DOUBLE_CLICK,
	TP_EVENT_LONG_PRESS,
	TP_EVENT_MAX
}tp_event;

void tp_interrupt_handler(void);
void Nrf52810_Uart_Send_Data_Test(void);

#endif
