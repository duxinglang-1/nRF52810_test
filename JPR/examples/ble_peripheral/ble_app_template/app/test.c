#include "test.h"
#include "app.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "nrf_delay.h"
#include "aes.h"
#include "feng_fstorage.h"

#define CBC 1
#define ECB 1
 
static void phex(uint8_t* str);
static void test_encrypt_ecb(void);
static void test_decrypt_ecb(void);
static void test_encrypt_ecb_verbose(void);
static void test_encrypt_cbc(void);
static void test_decrypt_cbc(void);

//==============================================
APP_TIMER_DEF(UTC_TIME_id);

uint32_t UTC_TIME_SECONDS=0; //
uint8_t SECONDE_BUFF[4];
uint16_t SECONDE_LEN=4;

static void UTC_timer_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	UTC_TIME_SECONDS++; 

	if(UTC_TIME_SECONDS%5 ==0) //write
	{
		//send_ble_version();
	}

	//NRF_LOG_INFO("UTC_TIME_SECONDS:%d", UTC_TIME_SECONDS);
}

uint32_t system_time_init(void)
{
	uint32_t err_code;
	
	err_code = app_timer_create(&UTC_TIME_id,APP_TIMER_MODE_REPEATED,UTC_timer_handler);
	APP_ERROR_CHECK(err_code); 
	
	if(err_code!= NRF_SUCCESS)
		return NRF_SUCCESS;	
	
	return err_code;	
}

/*********************************************************************************
* APP 同步设备系统的UTC时间的管理函数
*********************************************************************************/
uint32_t system_time_manger(bool ON_OFF_CMD)
{
	uint32_t err_code=0;
		
	if(ON_OFF_CMD)  
	{
		err_code = app_timer_start(UTC_TIME_id, APP_TIMER_TICKS(1000), NULL); //1000
		if (err_code != NRF_SUCCESS)
		{
				return err_code;
		}		
	}
	else
	{
		app_timer_stop(UTC_TIME_id);
		if (err_code != NRF_SUCCESS)
		{
				return err_code;
		}		
	}
	return err_code;
}

//===========================================================================
// prints string as hex
static void phex(uint8_t* str)
{
	unsigned char i;
	
	for(i = 0; i < 16; ++i)
		NRF_LOG_INFO("%.2x", str[i]);
}

static void test_decrypt_cbc(void)//feng
{
	//Example "simulating" a smaller buffer...
	uint8_t buffer[64];
	//0123456789012345
	uint8_t key[] = {0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x30,0x31,0x32,0x33,0x34,0x35}; 
	//0123456776543210
	uint8_t iv[]  = {0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x37,0x36,0x35,0x34,0x33,0x32,0x31,0x30};
	uint8_t in[]  = {0x45,0x21,0x47,0xE1,0xA7,0x9A,0xE9,0x12,0x30,0x5D,0xDB,0x28,0x30,0xEC,0x3F,0x0C};
	uint8_t out[] = {0x30,0x31,0x32,0x33,0x30,0x31,0x32,0x33,0x30,0x31,0x32,0x33,0x30,0x31,0x32,0x33};

	AES128_CBC_decrypt_buffer(buffer+0, in+0,  16, key, iv);
	//AES128_CBC_decrypt_buffer(buffer+16, in+16, 16, 0, 0);

	NRF_LOG_INFO("CBC decrypt: ");

	if(0 == memcmp((char*) out, (char*) buffer, 16))
	{
		NRF_LOG_INFO(" ------ SUCCESS!\n");
	}
	else
	{
		NRF_LOG_INFO("------ FAILURE!\n");
	}
}

static void test_encrypt_cbc(void)
{
	//0123456789012345 密码 也可以全是0
	//0000000000000000
	static uint8_t key[] = {0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30};
	//0123456776543210  可以为0
	//0000000000000000									
	static uint8_t iv[]  = {0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30};
	//0123012301230123  0123012301230123 输入的数据
	//0123000000000000	0000000000000000								
	static uint8_t in[]  = {0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30};							

	uint8_t out[] = {0x45,0x21,0x47,0xE1,0xA7,0x9A,0xE9,0x12,0x30,0x5D,0xDB,0x28,0x30,0xEC,0x3F,0x0C};
	uint8_t buffer[16];
	
	memset(buffer,0,16);

	AES128_CBC_encrypt_buffer(buffer, in, 16, key, iv);//

	NRF_LOG_INFO("CBC encrypt: ");

	if(0 == memcmp((char*) out, (char*) buffer, 16))
	{
		NRF_LOG_INFO(" +++++ SUCCESS!\n");
	}
	else
	{
		NRF_LOG_INFO(" +++++ FAILURE!\n");
	}
}
