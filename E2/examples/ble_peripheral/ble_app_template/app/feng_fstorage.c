#include "feng_fstorage.h"
#include "nrf_fstorage_sd.h"
#include <string.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_gpio.h"

#include "nrf_delay.h"

#include "test.h"

bool flash_read_flag;
bool write_flag;
bool find_whilt_flag;
bool is_find_flag; //是否找到

uint8_t	read_cmp[10][16];	//
static	uint8_t wait_inset_buff[16];
static	uint8_t zero_buff[16];
static	uint8_t temp_buff[16];
uint8_t write_buff[16];
uint8_t test_buff[16];


////////////////////////////////////////////////////////////////////
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);
 NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = 0x27000,
    .end_addr   = 0x28000,
};

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x",
                         p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
}


static void print_flash_info(nrf_fstorage_t * p_fstorage)
{
    NRF_LOG_INFO("========| flash info |========");
    NRF_LOG_INFO("erase unit: \t%d bytes",      p_fstorage->p_flash_info->erase_unit);
    NRF_LOG_INFO("program unit: \t%d bytes",    p_fstorage->p_flash_info->program_unit);
    NRF_LOG_INFO("==============================");
}

void fs_init(void)
{
	uint32_t	error;
	nrf_fstorage_api_t * p_fs_api;
	
	
	p_fs_api = &nrf_fstorage_sd;
	error = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
  print_flash_info(&fstorage);
     
}


//一次从falsh 读取160个字节
void bath_read()
{
		nrf_fstorage_read(&fstorage,0x27000,read_cmp,160); 
}
//擦除一页flash
void test_rease_flash(void)
{
static	uint32_t error;
		error = nrf_fstorage_erase(&fstorage,0x27000,1,NULL);
	NRF_LOG_INFO("========test_rease_flash:%x",error);
}


bool add_whilt_list(uint8_t *find_buff)
{
	uint8_t find_buf[16];
	uint8_t i;
	bool  flag;
	uint32_t cmp_rlt;
	memset(find_buf,0,sizeof(find_buf));
	
	memcpy(find_buf,find_buff,16);
	
	for(i=0;i<10;i++)
	{
		cmp_rlt = memcmp(&read_cmp[i][0],find_buf,16);
		if(!cmp_rlt)
		{ 
			NRF_LOG_INFO("======== find");
			flag = true;
			break;
		}		
	}
	if(i == 10) 
	{ 
			NRF_LOG_INFO("======== not find");
			flag = false;
	}

	return flag;
}
//NRF_SUCCESS
void updata_write(void)
{
	uint32_t	err_code;
	bool busy_stats=false;
	
	static	uint32_t test_addr=0x27000;
	busy_stats = nrf_fstorage_is_busy(&fstorage);
 	if(busy_stats)
	{
		test_rease_flash();
		nrf_delay_ms(50);
		NRF_LOG_INFO(" ***** ======== *****");
		NRF_LOG_INFO(" ***** Flash erase *****");
	}
	
	if(busy_stats) //不忙，为true
	{
			err_code = nrf_fstorage_write(&fstorage,test_addr,read_cmp,sizeof(read_cmp),NULL); //向flash 以字 单位写16字节
		nrf_delay_ms(50);
		if(NRF_SUCCESS == err_code)
		{
			NRF_LOG_INFO(" ***** Flash write  NRF_SUCCESS*****");
		}
	}
}



/* ***********************************************************
添加新的记录的时候，在ram 中进行操作完成后，再擦除和写入flash
即完成更新
*/

//bool updata_records(uint8_t *insert_buff)
bool updata_records(void)
{
	uint8_t i;
	uint8_t j;
	
	uint32_t cmp_rlt;
	memset(wait_inset_buff,0,sizeof(wait_inset_buff));	
	memset(zero_buff,0,sizeof(zero_buff));
	
	memcpy(wait_inset_buff,test_buff,16);
	
//	memcpy(wait_inset_buff,insert_buff,16);//sizeof(wait_inset_buff)
	
		for( i=0;i<10;i++)
		{
			cmp_rlt = memcmp(&read_cmp[i][0],zero_buff,16);//
			if(cmp_rlt == 0)//为0,进行储存
			{
				memcpy(&read_cmp[i][0],wait_inset_buff,sizeof(wait_inset_buff));
				updata_write();//更新到flash
				break;
			}
		}
		if(i == 10)
		{
			for(j=0;j<9;j++)
			{
				memset(&read_cmp[j][0],0,sizeof(read_cmp[j][0]));
				memcpy(&read_cmp[j][0],&read_cmp[j+1][0],sizeof(read_cmp[j][0]));
				
				memcpy(temp_buff,&read_cmp[j][0],sizeof(temp_buff));
			}
			memcpy(&read_cmp[j][0],wait_inset_buff,sizeof(wait_inset_buff));
			updata_write();
		}
		i=0;j=0;
		NRF_LOG_INFO(" --------------complete----------------- \r\n");
		Whitelist_exists();
 
	return 0;	
}






















