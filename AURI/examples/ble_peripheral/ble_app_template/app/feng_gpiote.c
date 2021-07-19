#include "feng_gpiote.h"

#include <stdio.h>

#include <stdint.h>
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_gpio.h"

#include "nrf_delay.h"
#include "nrf_uart.h"
#include "app_uart.h"

void touch_panel_event_handle(tp_event tp_type, uint16_t x_pos, uint16_t y_pos)
{
	switch(tp_type)
	{
	case TP_EVENT_MOVING_UP:
		NRF_LOG_INFO("tp moving up!\n");
		break;
	case TP_EVENT_MOVING_DOWN:
		NRF_LOG_INFO("tp moving down!\n");
		break;
	case TP_EVENT_MOVING_LEFT:
		NRF_LOG_INFO("tp moving left!\n");
		break;
	case TP_EVENT_MOVING_RIGHT:
		NRF_LOG_INFO("tp moving right!\n");
		break;
	case TP_EVENT_SINGLE_CLICK:
		NRF_LOG_INFO("tp single click! x:%d, y:%d\n", x_pos,y_pos);
		break;
	case TP_EVENT_DOUBLE_CLICK:
		NRF_LOG_INFO("tp double click! x:%d, y:%d\n", x_pos,y_pos);
		break;
	case TP_EVENT_LONG_PRESS:
		NRF_LOG_INFO("tp long press! x:%d, y:%d\n", x_pos,y_pos);
		break;
	case TP_EVENT_MAX:
		break;
	}
}


//nrf_drv_gpiote_in_init
void tp_interrupt_handler(void)
{
static	uint32_t error;
	uint32_t tp_counter;
	uint8_t tmpbuf[128] = {0};
	uint8_t TP_type = TP_EVENT_MAX;
	uint8_t tp_temp[10]={0};
 	
	nrf_twi_rx(TP_REG_GESTURE,&tp_temp[0],1);
	nrf_twi_rx(TP_REG_FINGER_NUM, &tp_temp[1], 1);
	
	nrf_twi_rx(TP_REG_XPOS_H, &tp_temp[2], 1);
	nrf_twi_rx(TP_REG_XPOS_L, &tp_temp[3], 1);
	
	nrf_twi_rx(TP_REG_YPOS_H, &tp_temp[4], 1);
	nrf_twi_rx(TP_REG_YPOS_L, &tp_temp[5], 1);
#if 1	
	tmpbuf[0]=0xAB ;//头
	tmpbuf[1]=0x00 ;//长度
	tmpbuf[2]=0x0A ;//长度
	
	tmpbuf[3] = 0xFF;//ID 校验开始 到校验和之前
	tmpbuf[4] = 0xB1;	//ID
	
	tmpbuf[5] = tp_temp[0] ;	//data  方向，上下，左右
	tmpbuf[6] = tp_temp[1] ;	//data
	
	tmpbuf[7] = tp_temp[2] ;	//data
	tmpbuf[8] = tp_temp[3] ;	//data 
	
	tmpbuf[9] = tp_temp[4] ;	//data
	tmpbuf[10] = tp_temp[5] ;	//data
	
	tmpbuf[11] = tmpbuf[3]+tmpbuf[4]+tmpbuf[5]+tmpbuf[6]+tmpbuf[7]+tmpbuf[8]+
							 tmpbuf[9]+tmpbuf[10];//校验和

	tmpbuf[12] = 0x88; //	
	
	for(uint8_t i=0;i<13;i++)
	{
	  error = app_uart_put(tmpbuf[i]);		
	 NRF_LOG_INFO("i=%x,error=%d",i,error);
	}
	#endif
	NRF_LOG_INFO("tp_temp=%x,%x,%x,%x,%x,%x,\n",tp_temp[0],tp_temp[1],tp_temp[2],tp_temp[3],tp_temp[4],tp_temp[5]);

	
	#if 0
	nrf_twi_rx(TP_REG_FINGER_NUM, &tp_temp[1], 1);//
	nrf_twi_rx(TP_REG_XPOS_L, &tp_temp[2], 1);//
	nrf_twi_rx(TP_REG_YPOS_L, &tp_temp[3], 1);//
#endif	
	
//	NRF_LOG_INFO("tp_temp=%x,%x,%x,%x\n",tp_temp[0],tp_temp[1],tp_temp[2],tp_temp[3]);
	switch(tp_temp[0])
	{
		case GESTURE_NONE:
			break;
		
		case GESTURE_MOVING_UP: //1 向上
					TP_type = TP_EVENT_MOVING_UP;
			break;
		
		case GESTURE_MOVING_DOWN://1 向下
					TP_type = TP_EVENT_MOVING_DOWN;
			break;
		
		case GESTURE_MOVING_LEFT://1 向昨
				 TP_type = TP_EVENT_MOVING_LEFT;
			break;
		
		case GESTURE_MOVING_RIGHT://1 向右
				 TP_type = TP_EVENT_MOVING_RIGHT;
			break;
		
		case GESTURE_SINGLE_CLICK://1 向上
				 TP_type = TP_EVENT_SINGLE_CLICK;
		break;
		
		case GESTURE_DOUBLE_CLICK://1 双击
				 TP_type = TP_EVENT_DOUBLE_CLICK;
		break;
		
		case GESTURE_LONG_PRESS://1 长按
				  TP_type = TP_EVENT_LONG_PRESS;
		break;
		
		default:
			
			break;
	}
 
	sprintf((char *)&tmpbuf, "TP_type:%d, x:%03d, y:%03d", TP_type, tp_temp[2], tp_temp[3]);

	if(TP_type != TP_EVENT_MAX)
	{
		touch_panel_event_handle(TP_type, tp_temp[2], tp_temp[3]);
	}	
  
}



void Nrf52810_Uart_Send_Data_Test(void)
{
  static	uint32_t error;
	uint8_t i;
//	uint32_t tp_counter;
	uint8_t tmpbuf[64] = {0};
  //uint8_t TP_type = TP_EVENT_MAX;
	//uint8_t tp_temp[10]={0};
	 uint8_t tp_temp[]={0x11,0x12,0x13,0x14,0x15,0x16};
	 
 	
//	nrf_twi_rx(TP_REG_GESTURE,&tp_temp[0],1);
//	nrf_twi_rx(TP_REG_FINGER_NUM, &tp_temp[1], 1);
//	
//	nrf_twi_rx(TP_REG_XPOS_H, &tp_temp[2], 1);
//	nrf_twi_rx(TP_REG_XPOS_L, &tp_temp[3], 1);
//	
//	nrf_twi_rx(TP_REG_YPOS_H, &tp_temp[4], 1);
//	nrf_twi_rx(TP_REG_YPOS_L, &tp_temp[5], 1);
	
	tmpbuf[0]=0xAB ;//头
	tmpbuf[1]=0x00 ;//长度
	tmpbuf[2]=0x0A ;//长度
	
	tmpbuf[3] = 0xFF;//ID 校验开始 到校验和之前
	tmpbuf[4] = 0xB1;	//ID
	
	tmpbuf[5] = tp_temp[0] ;	//data  
	tmpbuf[6] = tp_temp[1] ;	//data

	tmpbuf[7] = tp_temp[2] ;	//data
	tmpbuf[8] = tp_temp[3] ;	//data 

	tmpbuf[9] = tp_temp[4] ;	//data
	tmpbuf[10] = tp_temp[5] ;	//data
	
	tmpbuf[11] = tmpbuf[3]+tmpbuf[4]+tmpbuf[5]+tmpbuf[6]+tmpbuf[7]+tmpbuf[8]+
							 tmpbuf[9]+tmpbuf[10];//校验和
 
	tmpbuf[12] = 0x88; //	
	
	for(i=0;i<13;i++)
	{
		
		error = app_uart_put(tmpbuf[i]);
    
//		NRF_LOG_INFO("i=%x,error=%d",i,error);
		NRF_LOG_INFO(" i= %d,-52810_send_NRF9160_data---- =%x\n",i,tmpbuf[i]);		
	}

}


	


	