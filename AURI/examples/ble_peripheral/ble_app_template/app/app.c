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
//==================================================================
/*

0					1						3				5				6				7			8
StarFrame Datalength  ID 		  Status	Control	CRC8	EndFrame
0XAB			0X00~0XFF		0xFF52	0X80		


包的总长度：从ID ~包尾
包的校验和：从包头~CRC 之前
示例
		buff[0]=0xAB ;
		buff[1]=0x00 ;
		buff[2]=0x05 ;
		
		buff[3]=0xFF ;
		buff[4]=0x59 ;
		
		buff[5]=0x80 ;
		buff[6]=0x00 ;
		for(uint8_t i=0;i<7;i++)
			buff[7]=buff[7]+buff[i] ;//crc
		
		buff[8]=0x88 ;
		sendlength = 9 ;
		error = ble_nus_data_send(&m_nus, buff, &sendlength, m_conn_handle);
		NRF_LOG_INFO("error:%x \r\n",error);

*/

//================================================================
bool connect_flag;
bool	judg_app_flag;
uint8_t	ble_work_status;
uint32_t	GUARD_TIME_SECONDS=0;
uint8_t device_address[DEVICE_ADDRESS_LEN];

uint8_t	g_aes_in[16]; //全局加密解密输入buff
uint8_t	g_aes_out[16];//解密全局buff
//16字节，128的key 即密码
uint8_t aes_key[] =  { 0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30};
//16字节，128的IV 即偏移量									
uint8_t aes_iv[]  = { 0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30};

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
	 uint8_t p_buff[RANDOM_BUFF_SIZE];
		uint8_t length = random_vector_generate(p_buff,RANDOM_BUFF_SIZE);
		send_data_app(p_buff,RANDOM_BUFF_SIZE);
		NRF_LOG_INFO("Random Vector:");
		NRF_LOG_HEXDUMP_INFO(p_buff, length);
		NRF_LOG_INFO("");
		NRF_LOG_FLUSH();

}
    
		 
     
//=====================================================

void send_random_app(void)
{
		static		uint8_t p_buff[RANDOM_BUFF_SIZE];
		
		memset(p_buff,0,sizeof(p_buff));
		memset(g_aes_in,0,sizeof(g_aes_in));
		memset(g_aes_out,0,sizeof(g_aes_out));
	
		uint8_t length = random_vector_generate(p_buff,RANDOM_BUFF_SIZE);
		NRF_LOG_INFO(" length:%d ",length);
		//memcpy(p_buff,aes_key,sizeof(p_buff));
		send_data_app(p_buff,RANDOM_BUFF_SIZE);
		
		memcpy(g_aes_in,p_buff,sizeof(p_buff));
		AES128_CBC_encrypt_buffer(g_aes_out, g_aes_in, RANDOM_BUFF_SIZE, aes_key, aes_iv);//
	 
		NRF_LOG_INFO(" *** Random  encrypt complete *** ");
	 
	
}





//应答
void Whitelist_exists(void)
{	
	uint8_t buff[10];
	uint16_t sendlength=0;
	static	uint16_t error;
	memset(buff,0,sizeof(buff));
	
		buff[0]=0xAB ;
		buff[1]=0x00 ;
		buff[2]=0x08 ;
		
		buff[3]=0xFF ;
		buff[4]=0x58 ;
		
		buff[5]=0x80 ;
		buff[6]=0x00 ;
		
		buff[7]=0x01 ;//
		
		for(uint8_t i=0;i<7;i++)
			buff[8]=buff[8]+buff[i] ;//crc
		  
		buff[9]=0x58 ;
		sendlength = 10;
		send_data_app(buff,10);//
 			
}

/* ******************************************************************
为了不让非自己的APP 连接，一定时间内会断开

**********************************************************************/
 
APP_TIMER_DEF(GUARD_TIME_id);


 
static void UTC_timer_handler(void * p_context)
{
  UNUSED_PARAMETER(p_context);
	GUARD_TIME_SECONDS++; 
	if(GUARD_TIME_SECONDS>=10)
	{
		app_timer_stop(GUARD_TIME_id);
		disconnect_app();
		GUARD_TIME_SECONDS = 0 ;
	}
	
	NRF_LOG_INFO(" +++ GUARD_TIME_SECONDS: %d +++\n",GUARD_TIME_SECONDS);
 
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
** 守卫非 APP  连接的管理函数
*/
uint32_t guard_time_manger(bool ON_OFF_CMD)
{
	uint32_t err_code=0;
		
	if(ON_OFF_CMD == true)  
	{
		err_code = app_timer_start(GUARD_TIME_id, APP_TIMER_TICKS(1000), NULL); //1000
		if (err_code != NRF_SUCCESS)
		{
				return err_code;
		}		
	}
	if(ON_OFF_CMD == false) 
	{
		app_timer_stop(GUARD_TIME_id);
		GUARD_TIME_SECONDS =  0 ;
		
		if (err_code != NRF_SUCCESS)
		{
				return err_code;
		}		
	}
	return err_code;
}


void send_ble_device_mac(void)
{
	uint8_t mac_buff[15];
	uint8_t	mac_index=0;
	uint8_t	mac_crc=0;
	
	  mac_buff[mac_index++]=0xAB ;
	
		mac_buff[mac_index++]=0x00 ;
		mac_buff[mac_index++]=0x0C ;
		
		mac_buff[mac_index++]=0xFF ;
		mac_buff[mac_index++]=0xB3 ;		
	
		mac_buff[mac_index++]=0x80 ;
		mac_buff[mac_index++]=0x00 ;
	
		mac_buff[mac_index++]=device_address[0] ;
		mac_buff[mac_index++]=device_address[1] ;
		mac_buff[mac_index++]=device_address[2] ;
		mac_buff[mac_index++]=device_address[3] ;
		mac_buff[mac_index++]=device_address[4] ;
		mac_buff[mac_index++]=device_address[5] ;
	
		mac_buff[mac_index++]=0x00 ;
		
		mac_buff[mac_index]=0x88 ; 
		
	//  memcpy(mac_buff+7,device_address,DEVICE_ADDRESS_LEN);
	
		for(uint8_t i=0;i<mac_index-1;i++)
			mac_buff[mac_index-1]=mac_buff[mac_index-1]+mac_buff[i] ;//crc
 
		for(uint8_t i=0;i<mac_index+1;i++)
		{
			app_uart_put(mac_buff[i]);		
			NRF_LOG_INFO("MAC %d:%4x\r\n",i,mac_buff[i]);
		}
}




//RM_SW001_NRF52810_FW_V2.1_20201130.hex

void send_ble_version(void)
{
	uint8_t mac_buff[43];
	uint8_t	ver_crc=0;
	uint8_t	index=0;
	
	  mac_buff[index++]=0xAB ;
	
		mac_buff[index++]=0x00 ;
		mac_buff[index++]=0x28 ;
		
		mac_buff[index++]=0xFF ;
		mac_buff[index++]=0xB2 ;
		
		mac_buff[index++]=0x80 ;
		mac_buff[index++]=0x00 ; 
	
		mac_buff[index++]='R' ; //DATA
		mac_buff[index++]='M' ; //DATA
		mac_buff[index++]='_' ; //DATA
		mac_buff[index++]='S' ; //DATA
		mac_buff[index++]='W' ; //DATA
		mac_buff[index++]='0' ; //DATA
		mac_buff[index++]='0' ; //DATA
		mac_buff[index++]='1' ; //DATA		
		mac_buff[index++]='_' ; //DATA
		mac_buff[index++]='N' ; //DATA
		mac_buff[index++]='R' ; //DATA
		mac_buff[index++]='F' ; //DATA
		mac_buff[index++]='5' ; //DATA
		mac_buff[index++]='2' ; //DATA
		mac_buff[index++]='8' ; //DATA
		mac_buff[index++]='1' ; //DATA
		mac_buff[index++]='0' ; //DATA
		mac_buff[index++]='_' ; //DATA		
		mac_buff[index++]='F' ; //DATA
		mac_buff[index++]='W' ; //DATA
		mac_buff[index++]='_' ; //DATA		
		mac_buff[index++]='V' ; //DATA
		mac_buff[index++]='2' ; //DATA
		mac_buff[index++]='.' ; //DATA
		mac_buff[index++]='2' ; //DATA
		mac_buff[index++]='_' ; //DATA
		mac_buff[index++]='2' ; //DATA
		mac_buff[index++]='0' ; //DATA
		mac_buff[index++]='2' ; //DATA
		mac_buff[index++]='0' ; //DATA
		mac_buff[index++]='1' ; //DATA
		mac_buff[index++]='2' ; //DATA
		
		mac_buff[index++]='0' ; //DATA
		mac_buff[index++]='8' ; //DATA
		mac_buff[index++]=0x00 ; //CRC
		mac_buff[index]=0x88 ;
		 
		 
		for(uint8_t i=0;i<index-1;i++)
			mac_buff[index-1]=mac_buff[index-1]+mac_buff[i] ;//crc
	 
		//send_data_app(mac_buff,43);
	
		for(uint8_t i=0;i<43;i++)
		{
			app_uart_put(mac_buff[i]);		
			NRF_LOG_INFO("Version %d:%4x\r\n",i,mac_buff[i]);
		}
}


void get_ble_work_stutes(void)
{
	uint8_t ble_work_buff[43];
	
	  ble_work_buff[0]=0xAB ;
	
		ble_work_buff[1]=0x00 ;
		ble_work_buff[2]=0x07 ;
		
		ble_work_buff[3]=0xFF ;
		ble_work_buff[4]=0xB4 ;
		
		ble_work_buff[5]=0x80 ;
		ble_work_buff[6]=0x00 ;
		ble_work_buff[7]=ble_work_status ; //添加各种状态
	 
	
		for(uint8_t i=0;i<8;i++)
			ble_work_buff[8]=ble_work_buff[8]+ble_work_buff[i] ;//crc
		
		ble_work_buff[9]=0x88 ;
 
	
		for(uint8_t i=0;i<10;i++)
		{
			app_uart_put(ble_work_buff[i]);		
			NRF_LOG_INFO("MAC %d:%4x\r\n",i,ble_work_buff[i]);
		}
}


 
















