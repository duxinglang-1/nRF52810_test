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

uint8_t systemStartFlag = 0;	//nrf52810 enter normal work mode
uint8_t bootloaderMode = 0;		//nrf52810 enter bootloader mode

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

void tp_interrupt_handler(void)
{
	uint32_t i,len = 0;
	uint8_t tp_temp[10] = {0};
	uint8_t tmpbuf[128] = {0};
	uint8_t TP_type = TP_EVENT_MAX;

	nrf_twi_rx(TP_REG_GESTURE,&tp_temp[0],1);
	nrf_twi_rx(TP_REG_FINGER_NUM, &tp_temp[1], 1);
	
	nrf_twi_rx(TP_REG_XPOS_H, &tp_temp[2], 1);
	nrf_twi_rx(TP_REG_XPOS_L, &tp_temp[3], 1);
	
	nrf_twi_rx(TP_REG_YPOS_H, &tp_temp[4], 1);
	nrf_twi_rx(TP_REG_YPOS_L, &tp_temp[5], 1);

	NRF_LOG_INFO("tp_temp=%x,%x,%x,%x,%x,%x,\n",tp_temp[0],tp_temp[1],tp_temp[2],tp_temp[3],tp_temp[4],tp_temp[5]);
	
	//packet head
	tmpbuf[len++] = 0xAB;
	//data_len
	tmpbuf[len++] = 0x00;	
	tmpbuf[len++] = 0x0A;
	//data ID
	tmpbuf[len++] = 0xFF;
	tmpbuf[len++] = 0xB1;
	//resture
	tmpbuf[len++] = tp_temp[0];
	//finger_num
	tmpbuf[len++] = tp_temp[1];
	//pos_x
	tmpbuf[len++] = tp_temp[2];
	tmpbuf[len++] = tp_temp[3];
	//pos_y
	tmpbuf[len++] = tp_temp[4];
	tmpbuf[len++] = tp_temp[5];
	//crc
	tmpbuf[len++] = 0x00;
	//end
	tmpbuf[len++] = 0x88;
	
	for(i=0;i<(len-2);i++)
		tmpbuf[len-2] += tmpbuf[i];
	
	for(i=0;i<len;i++)
		app_uart_put(tmpbuf[i]);		

	switch(tp_temp[0])
	{
	case GESTURE_NONE:
		break;
	case GESTURE_MOVING_UP:
		TP_type = TP_EVENT_MOVING_UP;
		break;
	case GESTURE_MOVING_DOWN:
		TP_type = TP_EVENT_MOVING_DOWN;
		break;
	case GESTURE_MOVING_LEFT:
		TP_type = TP_EVENT_MOVING_LEFT;
		break;
	case GESTURE_MOVING_RIGHT:
		TP_type = TP_EVENT_MOVING_RIGHT;
		break;
	case GESTURE_SINGLE_CLICK:
		TP_type = TP_EVENT_SINGLE_CLICK;
		break;
	case GESTURE_DOUBLE_CLICK:
		TP_type = TP_EVENT_DOUBLE_CLICK;
		break;
	case GESTURE_LONG_PRESS:
		TP_type = TP_EVENT_LONG_PRESS;
		break;
	default:
		break;
	}

	if(TP_type != TP_EVENT_MAX)
	{
		touch_panel_event_handle(TP_type, tp_temp[3], tp_temp[5]);
	}
}

void Nrf52810_Uart_Send_Data_Test(void)
{
	static uint32_t error;
	uint8_t i,len=0;
	uint8_t tmpbuf[64] = {0};
	uint8_t tp_temp[] = {0x11,0x12,0x13,0x14,0x15,0x16};

	//packet head
	tmpbuf[len++] = 0xAB;
	//data_len
	tmpbuf[len++] = 0x00;	
	tmpbuf[len++] = 0x0A;
	//data ID
	tmpbuf[len++] = 0xFF;
	tmpbuf[len++] = 0xB1;
	//resture
	tmpbuf[len++] = tp_temp[0];
	//finger_num
	tmpbuf[len++] = tp_temp[1];
	//pos_x
	tmpbuf[len++] = tp_temp[2];
	tmpbuf[len++] = tp_temp[3];
	//pos_y
	tmpbuf[len++] = tp_temp[4];
	tmpbuf[len++] = tp_temp[5];
	//crc
	tmpbuf[len++] = 0x00;
 	//packet end
	tmpbuf[len++] = 0x88;

	for(i=0;i<(len-2);i++)
		tmpbuf[len-2] += tmpbuf[i];
	
	for(i=0;i<len;i++)
	{
		error = app_uart_put(tmpbuf[i]);
		NRF_LOG_INFO("i=%d,-52810_send_NRF9160_data---- =%x\n",i,tmpbuf[i]);	
	}
}

void system_upgrade_info_send(void)
{
	uint16_t i,len = 0;
	uint8_t tmpbuf[64] = {0};

	//packet head
	tmpbuf[len++] = 0xAB;
	//data_len
	tmpbuf[len++] = 0x00;
	tmpbuf[len++] = 0x05;
	//data id
	tmpbuf[len++] = 0xFF;
	tmpbuf[len++] = 0x10;
	//work mode
	if(bootloaderMode)
		tmpbuf[len++] = 0x00; 	//nrf52810 enter bootloader mode
	else if(systemStartFlag)
		tmpbuf[len++] = 0x01;	//nrf52810 enter normal work mode
	//crc
	tmpbuf[len++] = 0x00;
	//packet end
	tmpbuf[len++] = 0x88;
	
	for(i=0;i<(len-2);i++)
		tmpbuf[len-2] += tmpbuf[i];
	
	for(i=0;i<len;i++)
		app_uart_put(tmpbuf[i]);
}	
