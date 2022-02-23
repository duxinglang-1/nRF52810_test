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
extern uint8_t ble_work_status;	//0:off 1:sleep 2:advertising 3:connected

/*************************************************************************************************************************
* ID �� 0xFFB0 ��ʼ
* 0xFFB0: app��BLE�豸���ӶϿ���ID
* 0xFFB1: ��ȡMAC��ַ
* 0xFFB2: ��ȡ�汾
*
* #define GET_NRF52810_VER_ID		0xFFB2			//��ȡ52810�汾��
* #define GET_BLE_MAC_ADDR_ID		0xFFB3			//��ȡBLE MAC��ַ
* #define GET_BLE_STATUS_ID			0xFFB4			//��ȡBLE��ǰ����״̬	0:�ر� 1:���� 2:�㲥 3:����
* #define SET_BEL_WORK_MODE_ID		0xFFB5			//����BLE����ģʽ		0:�ر� 1:�� 2:���� 3:����
*************************************************************************************************************************/
extern uint8_t g_aes_in[16];	//ȫ�ּ��ܽ�������buff
extern uint8_t g_aes_out[16];	//����ȫ��buff
extern uint8_t aes_key[];		//16�ֽ�,128��key������						
extern uint8_t aes_iv[];		//16�ֽ�,128��IV��ƫ����

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
