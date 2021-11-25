#ifndef	APP_H
#define	APP_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define RANDOM_BUFF_SIZE	16
#define	DEVICE_ADDRESS_LEN	(6)

extern bool	judg_app_flag;
extern uint32_t	GUARD_TIME_SECONDS;
extern uint8_t device_address[DEVICE_ADDRESS_LEN];
extern uint8_t ble_work_status;

/*************************************************************************************************************************
* ID 从 0xFFB0 开始
* 0xFFB0: app和BLE设备连接断开的ID
* 0xFFB1: 获取MAC地址
* 0xFFB2: 获取版本
*
* #define GET_NRF52810_VER_ID		0xFFB2			//获取52810版本号
* #define GET_BLE_MAC_ADDR_ID		0xFFB3			//获取BLE MAC地址
* #define GET_BLE_STATUS_ID			0xFFB4			//获取BLE当前工作状态	0:关闭 1:休眠 2:广播 3:连接
* #define SET_BEL_WORK_MODE_ID		0xFFB5			//设置BLE工作模式		0:关闭 1:打开 2:唤醒 3:休眠
*************************************************************************************************************************/
extern uint8_t g_aes_in[16];	//全局加密解密输入buff
extern uint8_t g_aes_out[16];	//解密全局buff
extern uint8_t aes_key[];		//16字节,128的key即密码						
extern uint8_t aes_iv[];		//16字节,128的IV即偏移量

extern bool connect_flag;

uint32_t whitelist_response(char pbuff[], uint16_t cmd_id);
void test_random(void);
void send_data_app(uint8_t *pbuff, uint16_t pdata_len);
void send_random_app(void);
void Whitelist_exists(void);
void init_sec(void);
void init_keyset(void);
void disconnect_app(void);

uint32_t guard_time_init(void);
uint32_t guard_time_manger(bool ON_OFF_CMD);

void send_ble_device_mac(void);
void send_ble_version(void);

void get_ble_work_stutes(void);
void set_ble_work_mode(void);
	
#endif
