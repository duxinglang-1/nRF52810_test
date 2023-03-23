#include "app.h"
#include "ble_gap.h"
#include "ble_advertising.h"
#include "nrf_queue.h"  
#include <stdio.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "app_error.h"
#include "nrf_drv_rng.h"
#include "nrf_assert.h"
#include "aes.h" 
#include "ble_nus.h"
#include "app_timer.h"
#include "app_uart.h"

#define FW_VERSION	"E2_NRF52810_FW_V1.0.3_20230322"

bool connect_flag;
bool judg_app_flag;
uint8_t	ble_work_status;	//0:off 1:sleep 2:advertising 3:connected
uint32_t GUARD_TIME_SECONDS=0;
uint8_t device_address[DEVICE_ADDRESS_LEN];
uint8_t	g_aes_in[16]; //全局加密解密输入buff
uint8_t	g_aes_out[16];//解密全局buff
//16字节，128的key 即密码
uint8_t aes_key[] = {0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30};
//16字节，128的IV 即偏移量
uint8_t aes_iv[] = {0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30};

uint8_t random_vector_generate(uint8_t * p_buff, uint8_t size)
{
    uint32_t err_code;
    uint8_t  available;

    nrf_drv_rng_bytes_available(&available);
    uint8_t length = MIN(size, available);

    err_code = nrf_drv_rng_rand(p_buff, length);
    APP_ERROR_CHECK(err_code);

    return length;
}

void test_random(void)
{
	uint8_t p_buff[RANDOM_BUFF_SIZE] = {0};
	uint8_t length = random_vector_generate(p_buff,RANDOM_BUFF_SIZE);
	send_data_app(p_buff, RANDOM_BUFF_SIZE);
	NRF_LOG_INFO("Random Vector:");
	NRF_LOG_HEXDUMP_INFO(p_buff, length);
	NRF_LOG_INFO("");
	NRF_LOG_FLUSH();
}
         
//=====================================================
void send_random_app(void)
{
	static uint8_t p_buff[RANDOM_BUFF_SIZE] = {0};

	memset(g_aes_in,0,sizeof(g_aes_in));
	memset(g_aes_out,0,sizeof(g_aes_out));

	uint8_t length = random_vector_generate(p_buff,RANDOM_BUFF_SIZE);
	NRF_LOG_INFO("length:%d ",length);
	//memcpy(p_buff,aes_key,sizeof(p_buff));
	send_data_app(p_buff,RANDOM_BUFF_SIZE);

	memcpy(g_aes_in,p_buff,sizeof(p_buff));
	AES128_CBC_encrypt_buffer(g_aes_out, g_aes_in, RANDOM_BUFF_SIZE, aes_key, aes_iv);//

	NRF_LOG_INFO(" *** Random  encrypt complete *** ");
}

//应答
void Whitelist_exists(void)
{	
	uint8_t buff[128] = {0};
	uint8_t i,len = 0;

	//head
	buff[len++] = 0xAB;
	//data len
	buff[len++] = 0x00;
	buff[len++] = 0x07;
	//ID
	buff[len++] = 0xFF;
	buff[len++] = 0x58;
	//status
	buff[len++] = 0x80;
	//control
	buff[len++] = 0x00;
	//data
	buff[len++] = 0x01;
	//crc
	buff[len++] = 0x00;
	//end
	buff[len++] = 0x88;
	
	for(i=0;i<len-2;i++)
		buff[len-2] += buff[i];

	send_data_app(buff, len); 			
}

/* ******************************************************************
为了不让非法的APP连接，一段时间后会自动断开
**********************************************************************/
APP_TIMER_DEF(GUARD_TIME_id);

static void UTC_timer_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	
	GUARD_TIME_SECONDS++; 
	if(GUARD_TIME_SECONDS >= 10)
	{
		NRF_LOG_INFO("[%s] no auth connected, auto disconnect!", __func__);
		
		app_timer_stop(GUARD_TIME_id);
		disconnect_app();
		GUARD_TIME_SECONDS = 0;
	}
	
	NRF_LOG_INFO("[%s] GUARD_TIME_SECONDS:%d", __func__, GUARD_TIME_SECONDS);
}

uint32_t guard_time_init(void)
{
	uint32_t err_code;
	
	err_code = app_timer_create(&GUARD_TIME_id,APP_TIMER_MODE_REPEATED,UTC_timer_handler); //UTC_timer_handler
	APP_ERROR_CHECK(err_code); 
	
	if(err_code!= NRF_SUCCESS)
		return NRF_SUCCESS;	
	
	return err_code;	
}

/*********************************************************************************
* 守卫非 APP  连接的管理函数
*********************************************************************************/
uint32_t guard_time_manger(bool ON_OFF_CMD)
{
	uint32_t err_code=0;
		
	if(ON_OFF_CMD == true)  
	{
		err_code = app_timer_start(GUARD_TIME_id, APP_TIMER_TICKS(1000), NULL); //1000
	}
	if(ON_OFF_CMD == false)
	{
		err_code = app_timer_stop(GUARD_TIME_id);
		GUARD_TIME_SECONDS = 0;
	}
	
	return err_code;
}

void send_ble_device_mac(void)
{
	uint8_t	i,len=0;
	uint8_t buff[128] = {0};

	//head
	buff[len++] = 0xAB;
	//len
	buff[len++] = 0x00;
	buff[len++] = 0x0C;
	//id
	buff[len++] = 0xFF;
	buff[len++] = 0xB3;		
	//status
	buff[len++] = 0x80;
	//control
	buff[len++] = 0x00;
	//mac addr data
	buff[len++] = device_address[0];
	buff[len++] = device_address[1];
	buff[len++] = device_address[2];
	buff[len++] = device_address[3];
	buff[len++] = device_address[4];
	buff[len++] = device_address[5];
	//crc
	buff[len++] = 0x00;
	//end
	buff[len++] = 0x88; 
	
	for(i=0;i<len-2;i++)
		buff[len-2] += buff[i];

	for(i=0;i<len;i++)
		app_uart_put(buff[i]);		
}

void send_ble_version(void)
{
	uint8_t mac_buff[128] = {0};
	uint8_t i,len=0;

	//head
	mac_buff[len++] = 0xAB;
	//data len
	mac_buff[len++] = 0x00;
	mac_buff[len++] = 0x24;
	//ID
	mac_buff[len++] = 0xFF;
	mac_buff[len++] = 0xB2;
	//status
	mac_buff[len++] = 0x80;
	//control
	mac_buff[len++] = 0x00; 
	//version
	memcpy(&mac_buff[len], FW_VERSION, strlen(FW_VERSION));
	len += strlen(FW_VERSION);
	//crc
	mac_buff[len++] = 0x00;
	//end
	mac_buff[len++] = 0x88;

	for(i=0;i<len-2;i++)
		mac_buff[len-2] += mac_buff[i];

	for(i=0;i<len;i++)
		app_uart_put(mac_buff[i]);
}

void get_ble_work_stutes(void)
{
	uint8_t ble_work_buff[128] = {0};
	uint8_t i,len=0;

	//head
	ble_work_buff[len++]=0xAB;
	//len
	ble_work_buff[len++]=0x00;
	ble_work_buff[len++]=0x07;
	//ID
	ble_work_buff[len++]=0xFF;
	ble_work_buff[len++]=0xB4;
	//status
	ble_work_buff[len++]=0x80;
	//control
	ble_work_buff[len++]=0x00;
	//ble work status
	ble_work_buff[len++]=ble_work_status;
	//crc
	ble_work_buff[len++]=0x00;
	//end
	ble_work_buff[len++]=0x88;

	for(i=0;i<len-2;i++)
		ble_work_buff[len-2] += ble_work_buff[i];

	for(i=0;i<len;i++)
		app_uart_put(ble_work_buff[i]);		
}
