/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "app.h"
#include "app_uart.h"
#include "app_timer.h"
#include "app_error.h"
#include "ble_dfu.h"
#include "ble_conn_state.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_nus.h"
#include "ble_hids.h"
#include "ble_gap.h"
#include "ble_conn_state.h"
#include "nrf.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_rng.h"
#include "nrf_drv_wdt.h"
#include "nrf_delay.h"
#include "nrf_power.h"
#include "nrf_bootloader_info.h"
#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "test.h"
#include "feng_fstorage.h"
#include "feng_gpiote.h"
#include "feng_twi.h"

//=======================================================
#define DEVICE_NAME                     "E2_V3_BLE"	//"Nordic_Template"     /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL              	1600*1 //3 sec                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_DURATION                0 //18000         						/**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0     									/**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */
//===============================================================
#define BASE_USB_HID_SPEC_VERSION           0x0101   

#define OUTPUT_REPORT_INDEX                 0                                          /**< Index of Output Report. */
#define OUTPUT_REPORT_MAX_LEN               1                                          /**< Maximum length of Output Report. */
#define INPUT_REPORT_KEYS_INDEX             0                                          /**< Index of Input Report. */
#define OUTPUT_REPORT_BIT_MASK_CAPS_LOCK    0x02                                       /**< CAPS LOCK bit in Output Report (based on 'LED Page (0x08)' of the Universal Serial Bus HID Usage Tables). */
#define INPUT_REP_REF_ID                    0                                          /**< Id of reference to Keyboard Input Report. */
#define OUTPUT_REP_REF_ID                   0                                          /**< Id of reference to Keyboard Output Report. */
#define FEATURE_REP_REF_ID                  0                                          /**< ID of reference to Keyboard Feature Report. */
#define FEATURE_REPORT_MAX_LEN              2                                          /**< Maximum length of Feature Report. */
#define FEATURE_REPORT_INDEX                0                                          /**< Index of Feature Report. */

#define MAX_BUFFER_ENTRIES                  5   
#define INPUT_REPORT_KEYS_MAX_LEN           8   

#define MCU_WAKE_PIN	16
#define MCU_INT_PIN		18

bool uartflag = false;
bool connectflag = false;
bool disconnectflag =false;
bool senddataflag = false;
bool uart_is_wake = true;

static void uart_init(void);

uint32_t DFU_timer_init(void);
uint32_t UART_timer_init(void);

void irqio_init(void);
void nrf52810_outio_irq(void);
void senddatato_9160(void);
void inio_irq_init(void);
void in_pin_handler(nrf_drv_gpiote_pin_t pin,nrf_gpiote_polarity_t action);

NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);   
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT); 								/**< Advertising module instance. */

BLE_HIDS_DEF(m_hids,                                                			/**< Structure used to identify the HID service. */
             NRF_SDH_BLE_TOTAL_LINK_COUNT,
             INPUT_REPORT_KEYS_MAX_LEN,
             OUTPUT_REPORT_MAX_LEN,
             FEATURE_REPORT_MAX_LEN);

static bool m_in_boot_mode = false; 

nrf_drv_wdt_channel_id m_channel_id;
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;        	/**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

//YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{
	//{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
	{BLE_UUID_NUS_SERVICE, BLE_UUID_TYPE_BLE}
};


static void advertising_start(bool erase_bonds);


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
	//Initialize timer module.
	ret_code_t err_code = app_timer_init();
	APP_ERROR_CHECK(err_code);
	system_time_init();

	DFU_timer_init();
	UART_timer_init();
	//Create timers.
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("[%s] Data len is set to:%d", __func__, m_ble_nus_max_data_len);
    }
	
    NRF_LOG_DEBUG("[%s] ATT MTU exchange completed. central 0x%x peripheral 0x%x", 
					__func__,
                  	p_gatt->att_mtu_desired_central,
                  	p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
	ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
	APP_ERROR_CHECK(err_code);

	err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

void disconnect_app(void)
{
	sd_ble_gap_disconnect(m_conn_handle,BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
}

static void enter_dfu_mode(int8_t CMD)
{
	if(CMD == 0X01) //received the DFU command from master
	{
		uint32_t err_code = sd_power_gpregret_set(0, 0xB1); 
		APP_ERROR_CHECK(err_code);
		(void)sd_nvic_SystemReset();
	}
}

uint32_t whitelist_response(char pbuff[], uint16_t cmd_id)
{
	uint8_t buff[32]={0};
	uint16_t i,len=0;
	uint16_t error;
	
	memset(buff,0,sizeof(buff));
	memcpy(buff,pbuff,strlen(pbuff));
	
	if(cmd_id == 0xFF59)  
	{  
		//packet head
		buff[len++] = 0xAB;
		//data_len
		buff[len++] = 0x00;
		buff[len++] = 0x06;
		//data ID
		buff[len++] = 0xFF;
		buff[len++] = 0x59;
		//status
		buff[len++] = 0x80;
		//control
		buff[len++] = 0x00;
		//crc
		buff[len++] = 0x00;
		//packet end
		buff[len++] = 0x88;
		
		for(i=0;i<(len-2);i++)
			buff[len-2] += buff[i];

		error = ble_nus_data_send(&m_nus, buff, &len, m_conn_handle);
		NRF_LOG_INFO("error:%x \r\n",error);
	}
}

void ack_find(bool is_find_flag)
{	
	uint8_t buff[32]={0};
	uint16_t i,len=0;
	uint16_t error;

	//packet head
	buff[len++] = 0xAB;
	//data_len
	buff[len++] = 0x00;
	buff[len++] = 0x08;
	//data ID
	buff[len++] = 0xFF;
	buff[len++] = 0x58;
	//status
	buff[len++] = 0x80;
	//control
	buff[len++] = 0x00;
	//flag
	buff[len++] = is_find_flag;
	//crc
	buff[len++] = 0x00;
	//packet end
	buff[len++] = 0x88;
		
	for(i=0;i<(len-2);i++)
		buff[len-2] += buff[i];
		
	error = ble_nus_data_send(&m_nus, buff, &len, m_conn_handle);
	NRF_LOG_INFO("error:%x \r\n",error);
	if(is_find_flag == false)
	{
		connect_flag  = true;	
		NRF_LOG_INFO(" XXXXXX not found \r\n");
	}			
}


APP_TIMER_DEF(DFU_TIME_id);
void DFU_timer_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	enter_dfu_mode(0x01);
}

uint32_t DFU_timer_init(void)
{
	uint32_t err_code;
	
	err_code = app_timer_create(&DFU_TIME_id,APP_TIMER_MODE_SINGLE_SHOT,DFU_timer_handler);
	APP_ERROR_CHECK(err_code); 
 	return err_code;
}


APP_TIMER_DEF(UART_TIME_id);
void UART_sleep_timer_handler(void * p_context)
{
	uint32_t err_code;
	
	UNUSED_PARAMETER(p_context);
	
	nrf_uart_task_trigger(NRF_UART0, NRF_UART_TASK_STOPRX);
	nrf_uart_event_clear(NRF_UART0, NRF_UART_EVENT_RXDRDY);
	err_code = app_uart_close();
	APP_ERROR_CHECK(err_code);

	uart_is_wake = false;
}

uint32_t UART_timer_init(void)
{
	uint32_t err_code;
	
	err_code = app_timer_create(&UART_TIME_id, APP_TIMER_MODE_SINGLE_SHOT, UART_sleep_timer_handler);
	APP_ERROR_CHECK(err_code); 
	return err_code;
}

void UART_WakeUp(void)
{
	app_timer_stop(UART_TIME_id);
	app_timer_start(UART_TIME_id, APP_TIMER_TICKS(5000), NULL);

	if(uart_is_wake)
		return;
	
	uart_init();
	uart_is_wake = true;
}

/**@brief Function for handling the YYY Service events.
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 */
static uint8_t temp[40];
static uint8_t buff[40];
uint8_t actual_length=0;
uint16_t command_id=0;
uint16_t data_length=2;
uint32_t error;

static uint32_t err_code;
static uint16_t packet_len=0;
static uint16_t data_len=0;				
static uint8_t crc=0;

static void nus_data_handler(ble_nus_evt_t * p_evt)
{
	if(p_evt->type == BLE_NUS_EVT_RX_DATA)   //RX receive event
	{  
		NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
		packet_len = p_evt->params.rx_data.length;
		memset(buff,0, packet_len);	
		memcpy(buff,p_evt->params.rx_data.p_data,packet_len);

		NRF_LOG_INFO("packet_len:%d\r\n",packet_len);

		senddataflag = true;
	}
}

/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling HID events.
 *
 * @details This function will be called for all HID events which are passed to the application.
 *
 * @param[in]   p_hids  HID service structure.
 * @param[in]   p_evt   Event received from the HID service.
 */
static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_HIDS_EVT_BOOT_MODE_ENTERED:
            m_in_boot_mode = true;
            break;

        case BLE_HIDS_EVT_REPORT_MODE_ENTERED:
            m_in_boot_mode = false;
            break;

        case BLE_HIDS_EVT_REP_CHAR_WRITE:
            //on_hid_rep_char_write(p_evt);
            break;

        case BLE_HIDS_EVT_NOTIF_ENABLED:
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for initializing HID Service.
 */
static void hids_init(void)
{
    ret_code_t                    err_code;
    ble_hids_init_t               hids_init_obj;
    ble_hids_inp_rep_init_t     * p_input_report;
    ble_hids_outp_rep_init_t    * p_output_report;
    ble_hids_feature_rep_init_t * p_feature_report;
    uint8_t                       hid_info_flags;

    static ble_hids_inp_rep_init_t     input_report_array[1];
    static ble_hids_outp_rep_init_t    output_report_array[1];
    static ble_hids_feature_rep_init_t feature_report_array[1];
    static uint8_t                     report_map_data[] =
    {
        0x05, 0x01,       // Usage Page (Generic Desktop)
        0x09, 0x06,       // Usage (Keyboard)
        0xA1, 0x01,       // Collection (Application)
        0x05, 0x07,       // Usage Page (Key Codes)
        0x19, 0xe0,       // Usage Minimum (224)
        0x29, 0xe7,       // Usage Maximum (231)
        0x15, 0x00,       // Logical Minimum (0)
        0x25, 0x01,       // Logical Maximum (1)
        0x75, 0x01,       // Report Size (1)
        0x95, 0x08,       // Report Count (8)
        0x81, 0x02,       // Input (Data, Variable, Absolute)

        0x95, 0x01,       // Report Count (1)
        0x75, 0x08,       // Report Size (8)
        0x81, 0x01,       // Input (Constant) reserved byte(1)

        0x95, 0x05,       // Report Count (5)
        0x75, 0x01,       // Report Size (1)
        0x05, 0x08,       // Usage Page (Page# for LEDs)
        0x19, 0x01,       // Usage Minimum (1)
        0x29, 0x05,       // Usage Maximum (5)
        0x91, 0x02,       // Output (Data, Variable, Absolute), Led report
        0x95, 0x01,       // Report Count (1)
        0x75, 0x03,       // Report Size (3)
        0x91, 0x01,       // Output (Data, Variable, Absolute), Led report padding

        0x95, 0x06,       // Report Count (6)
        0x75, 0x08,       // Report Size (8)
        0x15, 0x00,       // Logical Minimum (0)
        0x25, 0x65,       // Logical Maximum (101)
        0x05, 0x07,       // Usage Page (Key codes)
        0x19, 0x00,       // Usage Minimum (0)
        0x29, 0x65,       // Usage Maximum (101)
        0x81, 0x00,       // Input (Data, Array) Key array(6 bytes)

        0x09, 0x05,       // Usage (Vendor Defined)
        0x15, 0x00,       // Logical Minimum (0)
        0x26, 0xFF, 0x00, // Logical Maximum (255)
        0x75, 0x08,       // Report Size (8 bit)
        0x95, 0x02,       // Report Count (2)
        0xB1, 0x02,       // Feature (Data, Variable, Absolute)

        0xC0              // End Collection (Application)
    };

    memset((void *)input_report_array, 0, sizeof(ble_hids_inp_rep_init_t));
    memset((void *)output_report_array, 0, sizeof(ble_hids_outp_rep_init_t));
    memset((void *)feature_report_array, 0, sizeof(ble_hids_feature_rep_init_t));

    // Initialize HID Service
    p_input_report                      = &input_report_array[INPUT_REPORT_KEYS_INDEX];
    p_input_report->max_len             = INPUT_REPORT_KEYS_MAX_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    p_input_report->sec.cccd_wr = SEC_JUST_WORKS;
    p_input_report->sec.wr      = SEC_JUST_WORKS;
    p_input_report->sec.rd      = SEC_JUST_WORKS;

    p_output_report                      = &output_report_array[OUTPUT_REPORT_INDEX];
    p_output_report->max_len             = OUTPUT_REPORT_MAX_LEN;
    p_output_report->rep_ref.report_id   = OUTPUT_REP_REF_ID;
    p_output_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_OUTPUT;

    p_output_report->sec.wr = SEC_JUST_WORKS;
    p_output_report->sec.rd = SEC_JUST_WORKS;

    p_feature_report                      = &feature_report_array[FEATURE_REPORT_INDEX];
    p_feature_report->max_len             = FEATURE_REPORT_MAX_LEN;
    p_feature_report->rep_ref.report_id   = FEATURE_REP_REF_ID;
    p_feature_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_FEATURE;

    p_feature_report->sec.rd              = SEC_JUST_WORKS;
    p_feature_report->sec.wr              = SEC_JUST_WORKS;

    hid_info_flags = HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;
		 
    memset(&hids_init_obj, 0, sizeof(hids_init_obj));

    hids_init_obj.evt_handler                    = on_hids_evt;
    hids_init_obj.error_handler                  = service_error_handler;
    hids_init_obj.is_kb                          = true;
    hids_init_obj.is_mouse                       = false;
    hids_init_obj.inp_rep_count                  = 1;
    hids_init_obj.p_inp_rep_array                = input_report_array;
    hids_init_obj.outp_rep_count                 = 1;
    hids_init_obj.p_outp_rep_array               = output_report_array;
    hids_init_obj.feature_rep_count              = 1;
    hids_init_obj.p_feature_rep_array            = feature_report_array;
    hids_init_obj.rep_map.data_len               = sizeof(report_map_data);
    hids_init_obj.rep_map.p_data                 = report_map_data;
    hids_init_obj.hid_information.bcd_hid        = BASE_USB_HID_SPEC_VERSION;
    hids_init_obj.hid_information.b_country_code = 0;
    hids_init_obj.hid_information.flags          = hid_info_flags;
    hids_init_obj.included_services_count        = 0;
    hids_init_obj.p_included_services_array      = NULL;

    hids_init_obj.rep_map.rd_sec         = SEC_JUST_WORKS;
    hids_init_obj.hid_information.rd_sec = SEC_JUST_WORKS;

    hids_init_obj.boot_kb_inp_rep_sec.cccd_wr = SEC_JUST_WORKS;
    hids_init_obj.boot_kb_inp_rep_sec.rd      = SEC_JUST_WORKS;

    hids_init_obj.boot_kb_outp_rep_sec.rd = SEC_JUST_WORKS;
    hids_init_obj.boot_kb_outp_rep_sec.wr = SEC_JUST_WORKS;

    hids_init_obj.protocol_mode_rd_sec = SEC_JUST_WORKS;
    hids_init_obj.protocol_mode_wr_sec = SEC_JUST_WORKS;
    hids_init_obj.ctrl_point_wr_sec    = SEC_JUST_WORKS;

    err_code = ble_hids_init(&m_hids, &hids_init_obj);
    APP_ERROR_CHECK(err_code);
}

//dfu 9
//关机准备处理程序。在关闭过程中，将以1秒的间隔调用此函数，直到函数返回true。当函数返回true时，表示应用程序已准备好复位为DFU模式
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
			// NRF_LOG_INFO("Power management wants to reset to DFU mode.");
			// YOUR_JOB: Get ready to reset into DFU mode
			//
			// If you aren't finished with any ongoing tasks, return "false" to
			// signal to the system that reset is impossible at this stage.
			//
			// Here is an example using a variable to delay resetting the device.
			//
			// if (!m_ready_for_reset)
			// {
			//      return false;
			// }
			// else
			// {
			//
			//    // Device ready to enter
			//    uint32_t err_code;
			//    err_code = sd_softdevice_disable();
			//    APP_ERROR_CHECK(err_code);
			//    err_code = app_timer_stop_all();
			//    APP_ERROR_CHECK(err_code);
			// }
            break;

        default:
            // YOUR_JOB: Implement any of the other events available from the power management module:
            //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
            //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
            //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
            return true;
    }

	//NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

//dfu 8
//注册优先级为0的应用程序关闭处理程序
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

//dfu 7
//SoftDevice状态监视者
static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context)
{
    if(state == NRF_SDH_EVT_STATE_DISABLED)
    {
        //表明Softdevice在复位之前已经禁用，告之bootloader启动时应跳过CRC
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);
        //进入system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

//dfu 6
//注册SoftDevice状态监视者，用于SoftDevice状态改变或者即将改变时接收SoftDevice事件
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
{
    .handler = buttonless_dfu_sdh_state_observer,
};

//dfu 5
//断开当前连接，设备准备进入bootloader之前，需要先断开连接
static void disconnect(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);
    //断开当前连接
    ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    }
    else
    {
        NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
    }
}

//dfu 4
//获取广播模式、间隔和超时时间
static void advertising_config_get(ble_adv_modes_config_t * p_config)
{
    memset(p_config, 0, sizeof(ble_adv_modes_config_t));

    p_config->ble_adv_fast_enabled  = true;
    p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
    p_config->ble_adv_fast_timeout  = APP_ADV_DURATION;
}

//dfu 3
//DFU事件处理函数。如果需要在DFU事件中执行操作，可以在相应的事件里面加入处理代码
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        //该事件指示设备正在准备进入bootloader
		case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
        {
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");

            //防止设备在断开连接时广播
            ble_adv_modes_config_t config;
            advertising_config_get(&config);
			//连接断开后设备不自动进行广播
            config.ble_adv_on_disconnect_disabled = true;
			//修改广播配置
            ble_advertising_modes_config_set(&m_advertising, &config);
            //断开当前已经连接的所有其他绑定设备。在设备固件更新成功（或中止）后，需要在启动时接收服务更改指示
            uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
            NRF_LOG_INFO("Disconnected %d links.", conn_count);
            break;
        }
        //该事件指示函数返回后设备即进入bootloader
        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            //如果应用程序有数据需要保存到Flash，通过app_shutdown_handler返回flase以延迟复位，从而保证数据正确写入到Flash
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;
        //该事件指示进入bootloader失败
        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            //进入bootloader失败，应用程序需要采取纠正措施来处理问题
			NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            break;
        //该事件指示发送响应失败
        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            //发送响应失败，应用程序需要采取纠正措施来处理问题
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};
		
	//dfu 2		
	//定义DFU服务初始化结构体变量
	ble_dfu_buttonless_init_t dfus_init = {0};
	//DFU服务事件句柄
	dfus_init.evt_handler = ble_dfu_evt_handler;
    //初始化DFU服务
    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);

    //Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    //Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
		
	hids_init();
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
  system_time_manger(true);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
			//sleep_mode_enter();     //无效广播后进入睡眠模式。
            break;

        default:
            break;
    }
}

//======================================================
ble_gap_sec_params_t g_pair_params;

void init_sec(void)
{
	g_pair_params.bond = 1 ;
	g_pair_params.io_caps = BLE_GAP_IO_CAPS_DISPLAY_ONLY; //0
	g_pair_params.oob = 0;
	g_pair_params.mitm = 1 ;
	g_pair_params.min_key_size = 7 ;
	g_pair_params.max_key_size = 16;
	
	g_pair_params.kdist_own.enc =1 ;
	g_pair_params.kdist_own.id = 0;
	g_pair_params.kdist_own.sign = 0;
	
	g_pair_params.kdist_peer.enc = 1 ;
	g_pair_params.kdist_peer.id = 0 ;
	g_pair_params.kdist_peer.sign = 0 ;
}

ble_gap_enc_key_t  my_enc_key;
ble_gap_enc_key_t  my_enc_key_center;
ble_gap_sec_keyset_t keyset;
void init_keyset(void)
{
	//keyset.keys_periph.p_enc_key
	keyset.keys_peer.p_enc_key = &my_enc_key;
	keyset.keys_peer.p_id_key = NULL;
	keyset.keys_peer.p_sign_key = NULL;
	
	keyset.keys_own.p_enc_key = &my_enc_key_center ;
	keyset.keys_own.p_id_key = NULL;
	keyset.keys_own.p_sign_key = NULL;
}

//======================================================
/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void	app_connect(void)
{ 
	uint8_t Push_conne[32]={0};
	uint16_t i,len=0;

	//packet head
	Push_conne[len++] = 0xAB;
	//data len
	Push_conne[len++] = 0x00;
	Push_conne[len++] = 0x06;
	//data id
	Push_conne[len++] = 0xFF;
	Push_conne[len++] = 0xB0;
	//status
	Push_conne[len++] = 0x80;
	//control
	Push_conne[len++] = 0x01;
	//crc
	Push_conne[len++] = 0x00;
	//packet end
	Push_conne[len++] = 0x88;
	
	for(i=0;i<(len-2);i++)
		Push_conne[len-2] += Push_conne[i];

	for(i=0;i<len;i++)
	{
		app_uart_put(Push_conne[i]);		
		NRF_LOG_INFO("connected %d:%4x\r\n",i,Push_conne[i]);
	}
}

static void	app_disconnect(void)
{
	uint8_t Push_conne[9]={0}; 
	uint16_t i,len=0;

	//packet head
	Push_conne[len++] = 0xAB;
	//data len
	Push_conne[len++] = 0x00;
	Push_conne[len++] = 0x06;
	//data id
	Push_conne[len++] = 0xFF;
	Push_conne[len++] = 0xB0;
	//status
	Push_conne[len++] = 0x80;
	//control
	Push_conne[len++] = 0x00;
	//crc
	Push_conne[len++] = 0x00;
	//packet end
	Push_conne[len++] = 0x88;

	for(i=0;i<(len-2);i++)
		Push_conne[len-2] += Push_conne[i];

	for(i=0;i<len;i++)
	{
		app_uart_put(Push_conne[i]);		
		NRF_LOG_INFO("disconnected %d:%4x\r\n",i,Push_conne[i]);
	}
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.   

 */
uint8_t step_counter;
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch(p_ble_evt->header.evt_id)
	{
	case BLE_GAP_EVT_CONNECTED:
		NRF_LOG_INFO("Connected.");
		connectflag = true;
	#if PM_BOND_SWITCH
		init_keyset();
	#endif
		err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
		APP_ERROR_CHECK(err_code);
		m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
		err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
		APP_ERROR_CHECK(err_code);
		ble_work_status = 0x03;
		break;
		
	case BLE_GAP_EVT_DISCONNECTED:
		switch(p_ble_evt->evt.gap_evt.params.disconnected.reason)
		{
		case BLE_HCI_CONNECTION_TIMEOUT:
			NRF_LOG_INFO("[%s] Disconnected: connection timeout.", __func__);
			break;
		case BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION:
			NRF_LOG_INFO("[%s] Disconnected: user terminated connection.", __func__);
			break;
		default:
			NRF_LOG_INFO("[%s] Disconnected: other reason(%02X)", __func__, p_ble_evt->evt.gap_evt.params.disconnected.reason);
			break;
		}
		NRF_LOG_INFO("Disconnected.");
		disconnectflag = true;
		break;

	case BLE_GAP_EVT_CONN_PARAM_UPDATE:
		NRF_LOG_INFO("[%s] conn param update.", __func__);
		sd_ble_gap_conn_param_update(m_conn_handle, &p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params);
		break;
		
	case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
		NRF_LOG_INFO("[%s] src param request.", __func__);
		
	#if PM_BOND_SWITCH
		NRF_LOG_INFO("receive pair req."); 
		NRF_LOG_INFO("step_counter:%d",++step_counter);
		init_sec();
		sd_ble_gap_sec_params_reply(m_conn_handle,BLE_GAP_SEC_STATUS_SUCCESS,&g_pair_params,&keyset);
	#endif
		break;
	
	case BLE_GAP_EVT_SEC_INFO_REQUEST:
		NRF_LOG_INFO("[%s] sec info update.", __func__);
		
	#if PM_BOND_SWITCH
		NRF_LOG_INFO("step:%d", ++step_counter);
		NRF_LOG_INFO("enc_need:%d, id_need:%d, sign need:%d",
						p_ble_evt->evt.gap_evt.params.sec_info_request.enc_info,
						p_ble_evt->evt.gap_evt.params.sec_info_request.id_info,
						p_ble_evt->evt.gap_evt.params.sec_info_request.sign_info);

		for(int i=0;i<my_enc_key.enc_info.ltk_len;i++)
		{
			NRF_LOG_INFO("LTK[%d]:%x", i, my_enc_key.enc_info.ltk[i]); 
		}

		NRF_LOG_INFO("EDIV:%x", p_ble_evt->evt.gap_evt.params.sec_info_request.master_id.ediv); 

		for(int i=0;i<8;i++)
		{
			NRF_LOG_INFO("rand[%d]:%x", i, p_ble_evt->evt.gap_evt.params.sec_info_request.master_id.rand[i]);
		}
		sd_ble_gap_sec_info_reply(m_conn_handle,&my_enc_key.enc_info, NULL, NULL);
	#endif
		break;

	case BLE_GAP_EVT_PASSKEY_DISPLAY:
		NRF_LOG_INFO("[%s] passkey display.", __func__);
		
	#if PM_BOND_SWITCH
		NRF_LOG_INFO("step:%d",++step_counter );
		NRF_LOG_INFO("feng PASSKEY:"); 
		for(uint8_t i=0;i<6;i++)
		{
			NRF_LOG_INFO("%c",p_ble_evt->evt.gap_evt.params.passkey_display.passkey[i]);			 
		}
	#endif
		break;
	
	case BLE_GAP_EVT_KEY_PRESSED:
		NRF_LOG_INFO("[%s] key pressed.", __func__);
		break;
		
	case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
		NRF_LOG_INFO("[%s] dhkey request.", __func__);
		break;
		
	case BLE_GAP_EVT_AUTH_STATUS:
		NRF_LOG_INFO("[%s] auth status.", __func__);
		
	#if PM_BOND_SWITCH
		NRF_LOG_INFO(" step:%d",++step_counter ); 
		NRF_LOG_INFO(" keyset dispatch done");  
		NRF_LOG_INFO("LTK"); 
		for(int i=0;i<my_enc_key.enc_info.ltk_len;i++)
		{
			NRF_LOG_INFO("%x",my_enc_key.enc_info.ltk[i]); 
		}

		NRF_LOG_INFO("AUTH:%d",my_enc_key.enc_info.auth); 				
		NRF_LOG_INFO("LTK length:%d",my_enc_key.enc_info.ltk_len); 				
		NRF_LOG_INFO("EDIV:%x ",my_enc_key.master_id.ediv); 
		for(int i=0;i<8;i++)
		{
			NRF_LOG_INFO("%x",my_enc_key.master_id.rand[i]); 
		}
	#endif
		break;

	case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
		{
			NRF_LOG_INFO("[%s] PHY update request.", __func__);
			ble_gap_phys_t const phys =
			{
				.rx_phys = BLE_GAP_PHY_AUTO,
				.tx_phys = BLE_GAP_PHY_AUTO,
			};
			err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
			APP_ERROR_CHECK(err_code);
		}
		break;
		
	case BLE_GAP_EVT_PHY_UPDATE:
		NRF_LOG_INFO("[%s] PHY update.", __func__);
		break;
		
	case BLE_GAP_EVT_ADV_SET_TERMINATED:
		NRF_LOG_INFO("[%s] adv set terminated.", __func__);
		ble_gap_phys_t const phys =
		{
			.rx_phys = BLE_GAP_PHY_AUTO,
			.tx_phys = BLE_GAP_PHY_AUTO,
		};
		err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GATTC_EVT_EXCHANGE_MTU_RSP:
		NRF_LOG_INFO("[%s] gattc evt exchange mtu rsp.", __func__);
		break;
		
	case BLE_GATTC_EVT_TIMEOUT:
		NRF_LOG_INFO("[%s] gattc evt timeout.", __func__);
		err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
		NRF_LOG_INFO("[%s] gatts evt exchange mtu request.", __func__);
		break;

	case BLE_GATTS_EVT_TIMEOUT:
		NRF_LOG_INFO("[%s] gatts evt timeout.", __func__);
		err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		APP_ERROR_CHECK(err_code);
		break;

	default:
		//No implementation needed.
		break;
	}
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM. nrf_sdh_ble_default_cfg_set
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event. 
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt); //配对绑定的提示信息
    pm_handler_flash_clean(p_evt);

    switch(p_evt->evt_id)
	{	
	case PM_EVT_BONDED_PEER_CONNECTED:
		NRF_LOG_INFO("[%s] PM_EVT_BONDED_PEER_CONNECTED", __func__);
		break;
		
	case PM_EVT_CONN_SEC_START:
		NRF_LOG_INFO("[%s] PM_EVT_CONN_SEC_START", __func__);
		break;

	case PM_EVT_CONN_SEC_SUCCEEDED:
		NRF_LOG_INFO("[%s] PM_EVT_CONN_SEC_SUCCEEDED", __func__);
		break;

	case PM_EVT_CONN_SEC_FAILED:
		NRF_LOG_INFO("[%s] PM_EVT_CONN_SEC_FAILED", __func__);
		break;

	case PM_EVT_CONN_SEC_CONFIG_REQ:
		{		
			//this case is optional. comment the following lines if not needed
			pm_conn_sec_config_t cfg;
			cfg.allow_repairing = true;   //true to permit a second paring with the same host when the bonding info is removed
			pm_conn_sec_config_reply(p_evt->conn_handle, &cfg);
		}
		break; 

	case PM_EVT_CONN_SEC_PARAMS_REQ:
		NRF_LOG_INFO("[%s] PM_EVT_CONN_SEC_PARAMS_REQ", __func__);
		break;

	case PM_EVT_STORAGE_FULL:
		NRF_LOG_INFO("[%s] PM_EVT_STORAGE_FULL", __func__);
		break;

	case PM_EVT_ERROR_UNEXPECTED:
		NRF_LOG_INFO("[%s] PM_EVT_CONN_SEC_SUCCEEDED", __func__);
		break;

	case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
		//配对绑定后，会输出该提示信息
		NRF_LOG_INFO("[%s] PM_EVT_CONN_SEC_SUCCEEDED", __func__);
		break; 

	case PM_EVT_PEER_DATA_UPDATE_FAILED:
		NRF_LOG_INFO("[%s] PM_EVT_PEER_DATA_UPDATE_FAILED", __func__);
		break;  

	case PM_EVT_PEER_DELETE_SUCCEEDED:
		NRF_LOG_INFO("[%s] PM_EVT_PEER_DELETE_SUCCEEDED", __func__);
		break;  

	case PM_EVT_PEER_DELETE_FAILED:
		NRF_LOG_INFO("[%s] PM_EVT_PEER_DELETE_FAILED", __func__);
		break;

	case PM_EVT_PEERS_DELETE_SUCCEEDED:
		NRF_LOG_INFO("[%s] PM_EVT_PEERS_DELETE_SUCCEEDED", __func__);
		advertising_start(false);
		break; 

	case PM_EVT_PEERS_DELETE_FAILED:
		NRF_LOG_INFO("[%s] PM_EVT_PEERS_DELETE_FAILED", __func__);
		break;  

	case PM_EVT_LOCAL_DB_CACHE_APPLIED:
		NRF_LOG_INFO("[%s] PM_EVT_LOCAL_DB_CACHE_APPLIED", __func__);
		break;  

	case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
		NRF_LOG_INFO("[%s] PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED", __func__);
		break;  

	case PM_EVT_SERVICE_CHANGED_IND_SENT:
		NRF_LOG_INFO("[%s] PM_EVT_SERVICE_CHANGED_IND_SENT", __func__);
		break;  

	case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
		NRF_LOG_INFO("[%s] PM_EVT_SERVICE_CHANGED_IND_CONFIRMED", __func__);
		break;  

	case PM_EVT_SLAVE_SECURITY_REQ:
		NRF_LOG_INFO("[%s] PM_EVT_SLAVE_SECURITY_REQ", __func__);
		break;  

	case PM_EVT_FLASH_GARBAGE_COLLECTED:
		NRF_LOG_INFO("[%s] PM_EVT_FLASH_GARBAGE_COLLECTED", __func__);
		break;  

	case PM_EVT_FLASH_GARBAGE_COLLECTION_FAILED:
		NRF_LOG_INFO("[%s] PM_EVT_FLASH_GARBAGE_COLLECTION_FAILED", __func__);
		break;

	default:
		NRF_LOG_INFO("[%s] default evt_id:%d", __func__, p_evt->evt_id);
		break;
	}
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM; //
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
	ret_code_t err_code;

	switch(event)
	{
	case BSP_EVENT_SLEEP:
		NRF_LOG_INFO("[%s] BSP_EVENT_SLEEP", __func__);
		//sleep_mode_enter(); //按钮触发进入睡眠模式。
		break;

	case BSP_EVENT_DISCONNECT:
		NRF_LOG_INFO("[%s] BSP_EVENT_DISCONNECT", __func__);
		err_code = sd_ble_gap_disconnect(m_conn_handle,
	                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		if (err_code != NRF_ERROR_INVALID_STATE)
		{
			APP_ERROR_CHECK(err_code);
		}
		break;

	case BSP_EVENT_WHITELIST_OFF:
		NRF_LOG_INFO("[%s] BSP_EVENT_WHITELIST_OFF", __func__);
		if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
		{
			err_code = ble_advertising_restart_without_whitelist(&m_advertising);
			if (err_code != NRF_ERROR_INVALID_STATE)
			{
				APP_ERROR_CHECK(err_code);
			}
		}
		break;

	default:
		NRF_LOG_INFO("[%s] default event:%d", __func__, event);
		break;
	}
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t err_code;
    ble_advertising_init_t init={0};
	ble_advdata_manuf_data_t manuf_data;
	ble_gap_addr_t device_addr;
	uint8_t m_adv_data[6]={0};

	err_code = sd_ble_gap_addr_get(&device_addr); 
	device_address[0]= device_addr.addr[5];
	device_address[1]= device_addr.addr[4];
	device_address[2]= device_addr.addr[3];
	device_address[3]= device_addr.addr[2];
	device_address[4]= device_addr.addr[1];
	device_address[5]= device_addr.addr[0];
	memcpy(m_adv_data,device_address,DEVICE_ADDRESS_LEN);

	manuf_data.data.p_data = m_adv_data;			//添加到广播包中的数据
	manuf_data.data.size = sizeof(m_adv_data);  	//添加到广播包中的数据长度
	manuf_data.company_identifier = 0x0001;			//2个字节，广播包从这个地方开始
	
	init.advdata.name_type               = BLE_ADVDATA_FULL_NAME; //BLE_ADVDATA_FULL_NAME BLE_ADVDATA_SHORT_NAME
	init.advdata.include_appearance      = false;//true
	init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

	init.advdata.p_manuf_specific_data= &manuf_data ; //add	    

	//init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
	//init.advdata.uuids_complete.p_uuids  = m_adv_uuids;	//不注释掉，广播名字不能显示完整

	init.config.ble_adv_fast_enabled  = true;
	init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
	init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

	init.evt_handler = on_adv_evt;

	err_code = ble_advertising_init(&m_advertising, &init);
	APP_ERROR_CHECK(err_code);

	ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}
/* 
ble_advertising_t * const p_advertising
advertising_stop(ble_advertising_t * const p_advertising)
m_advertising
*/
										 
void advertising_stop(ble_advertising_t * const p_advertising)
{
	sd_ble_gap_adv_stop(p_advertising->adv_handle);
	ble_work_status = 0x00;
}

/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if(erase_bonds == true)
    {
    	advertising_stop(&m_advertising);
		//delete_bonds();
        //Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
		ble_work_status = 0x02;
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 BLE 从串口获取数据，然后通过ble 发送出去AB0007FF468000017888
*/
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
	static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
	static uint16_t rec_len = 0;
	uint8_t	cr=0,i=0;
	uint16_t data_len=0; 
	uint8_t	rx_crc=0;
	uint16_t rx_cmd_id=0;
	uint32_t err_code;
	
    switch(p_event->evt_type)
	{
	case APP_UART_DATA_READY:
		while(app_uart_get(&cr) == NRF_SUCCESS)
		{
			data_array[rec_len++] = cr;
			NRF_LOG_INFO("[%s] rec_len:%d, cr:%x", __func__, rec_len, cr);
			
			if(data_array[0] != 0xab)
			{
				NRF_LOG_INFO("[%s] receive data is valid!", __func__);
				rec_len = 0;
			}
		}

		if((data_array[0] == 0xab)&&(data_array[rec_len-1] == 0x88))
		{
			data_len = ((data_array[1]<<8)|data_array[2]);
			rx_cmd_id = (data_array[3]<<8|data_array[4]);
			switch(rx_cmd_id)
			{
			case 0xFFB1:
				break;

			case 0xFFB2://send 52810 fw version to 9160
				send_ble_version();
				break;

			case 0xFFB3://send 52810 ble mac addr to 9160
				send_ble_device_mac();
				break;

			case 0xFFB4://send ble work status to 9160 0:off,1:sleep,2:advertising,3:connected
				get_ble_work_stutes();
				break;

			case 0xFFB5://set ble work status from 9160 0:off,1:on,2:wake,3:sleep
				advertising_stop(&m_advertising);
				advertising_start(false);	
				break;

			default:
				break;
			}

			if((rx_cmd_id != 0xFFB0)&&(rx_cmd_id != 0xFFB1)&&(rx_cmd_id != 0xFFB2)
				&&(rx_cmd_id != 0xFFB3)&&(rx_cmd_id != 0xFFB4)&&(rx_cmd_id != 0xFFB5))
			{
				for(i=0;i<rec_len-2;i++)
				{
					rx_crc = rx_crc + data_array[i] ;
				}
				
				if(rx_crc == data_array[rec_len-2])
				{
					NRF_LOG_INFO("uart receive length:%d", rec_len);		
					ble_nus_data_send(&m_nus, data_array, &rec_len, m_conn_handle);
				}
			}

			rec_len = 0;
			memset(data_array, 0, sizeof(data_array));
		}
		
		err_code = app_timer_stop(UART_TIME_id);
		err_code = app_timer_start(UART_TIME_id, APP_TIMER_TICKS(5000), NULL);
		APP_ERROR_CHECK(err_code);
		break;

    case APP_UART_DATA: 
        break;

    case APP_UART_TX_EMPTY:
        break;

    case APP_UART_FIFO_ERROR:
        APP_ERROR_HANDLER(p_event->data.error_code);
        break;

    default:
        break;
    }
}


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
	#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
	#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
	#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */

void wdt_event_handler(void)
{
    //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}

/**@brief Function for application main entry.
 */
int main(void)
{
	bool erase_bonds;
	uint32_t err_code;
	
	//dfu 10
	//使能任一中断之前，将异步SVCI接口初始化为引导加载程序
	//err_code = ble_dfu_buttonless_async_svci_init();
	//APP_ERROR_CHECK(err_code);

	//Initialize.
	log_init();
	NRF_LOG_INFO("main started.");
	uart_init();
	twi_init();//i2c
	fs_init();

	irqio_init();
	inio_irq_init();

	bath_read();
	timers_init();
	//buttons_leds_init(&erase_bonds);
	power_management_init();
	ble_stack_init();
	gap_params_init();
	gatt_init();
	advertising_init();
	services_init();
	conn_params_init();
	//peer_manager_init();
	guard_time_init();

	//Start execution.
	application_timers_start();

	advertising_start(false);	 
	err_code = nrf_drv_rng_init(NULL);
	APP_ERROR_CHECK(err_code);

	//Configure WDT.
	nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
	err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
	APP_ERROR_CHECK(err_code);
	nrf_drv_wdt_enable();

	//Touch
	tp_init();

	/****系统初始化完成，NRF52810通过串口向NRF9160发送远成标志数据**********/
	nrf52810_outio_irq();
	systemStartFlag = 1;  //系统启动完成标志
	system_upgrade_info_send();
	systemStartFlag = 0;	

	err_code = app_timer_start(UART_TIME_id, APP_TIMER_TICKS(10), NULL);
	APP_ERROR_CHECK(err_code);

	//Enter main loop.

	while(1)
	{
		nrf_drv_wdt_channel_feed(m_channel_id);

		TpMsgProc();
		
		if(connect_flag)
		{  
			connect_flag = false; 
			send_random_app();			
		}

		if(write_flag)
		{
			write_flag = false;
			updata_records();
		}

		if(find_whilt_flag)
		{
			find_whilt_flag = false;
			is_find_flag = add_whilt_list(test_buff);
			ack_find(is_find_flag); //没有找到 发送 随机数
		}
		
		if(connectflag)
			senddatato_9160();
		
		if(disconnectflag)
			senddatato_9160();
		
		if(senddataflag)
			senddatato_9160();
		
		idle_state_handle();
	}
}

void send_data(void)
{
	uint32_t error;
	uint16_t data_len = 32;
	uint8_t out[] = {0x28, 0x86, 0x6A, 0x9D, 0x94, 0xE8, 0x1C, 0x50, 0x40, 0x61, 0x01, 0xBC, 0x1A, 0x22, 0xD5, 0x04, 
					 0x95, 0xFE, 0xA3, 0xD6, 0x3D, 0x24, 0xEE, 0xFC, 0x2E, 0x03, 0xDC, 0xA3, 0x3B, 0x95, 0xCF, 0xF1};
	error = ble_nus_data_send(&m_nus, out, &data_len, m_conn_handle);
	NRF_LOG_INFO("[%s] error:%x", __func__, error);
}

void send_data_app(uint8_t *pbuff, uint16_t	pdata_len)
{
	uint32_t error;
 
	error = ble_nus_data_send(&m_nus, pbuff, &pdata_len, m_conn_handle);
	NRF_LOG_INFO(" Ble sned to app error:%x \r\n",error);
}

/**
 * @}
 */

void irqio_init(void)
{
	nrf_gpio_cfg_output(MCU_WAKE_PIN);
	nrf_gpio_pin_clear(MCU_WAKE_PIN);
}	

void nrf52810_outio_irq(void)
{
	nrf_gpio_pin_clear(MCU_WAKE_PIN);
	nrf_delay_ms(10);
	nrf_gpio_pin_set(MCU_WAKE_PIN);
	
	//xb add 2023-03-22 wait mcu(9160) for wakeup
	nrf_delay_ms(10);
}	

void in_pin_handler(nrf_drv_gpiote_pin_t pin,nrf_gpiote_polarity_t action)
{
	UART_WakeUp();
	
	NRF_LOG_INFO("============= ble io in irq ================ \r\n");
}	

void inio_irq_init(void)
{
	ret_code_t err_code;
	
	err_code = nrf_drv_gpiote_init();
	APP_ERROR_CHECK(err_code);
	
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(1);
	in_config.pull = NRF_GPIO_PIN_PULLDOWN;
	
	err_code = nrf_drv_gpiote_in_init(MCU_INT_PIN,&in_config,in_pin_handler);
	APP_ERROR_CHECK(err_code);
	
	nrf_drv_gpiote_in_event_enable(MCU_INT_PIN,true);
}	

void senddatato_9160(void)
{
	ret_code_t err_code = NRF_SUCCESS;
	
	if(connectflag)
	{
		connectflag = false;

		UART_WakeUp();
		
		nrf52810_outio_irq();
		app_connect();
		judg_app_flag = true;
	}

	if(disconnectflag)	
	{
		disconnectflag = false;

		UART_WakeUp();

		nrf52810_outio_irq();
		app_disconnect();
		GUARD_TIME_SECONDS = 0;
		judg_app_flag = false;
	}

	if(senddataflag)
	{
		senddataflag = false;
		if(memcmp(buff,g_aes_out,RANDOM_BUFF_SIZE) == 0) //验证随机数
		{
			write_flag = true;
			NRF_LOG_INFO("  ******************** \r\n");
			NRF_LOG_INFO("   OK OK OK OK OK  \r\n");
		}	
		else if((buff[0] == 0xAB)&&(buff[packet_len-1] != 0x88 ))
		{
			memcpy(temp,buff,packet_len);
		}
		else if((buff[0] != 0xAB)&&(buff[packet_len-1]==0x88))
		{
			memcpy(temp+20,buff,packet_len);
			actual_length = packet_len+20;
			memset(buff,0,sizeof(buff));
			memcpy(buff,temp+20,actual_length);

			NRF_LOG_INFO(" actual_length:%d\r\n",actual_length);
			for(uint8_t i=0;i<actual_length;i++)
			{				
				NRF_LOG_INFO(" buff[%d]:%x\r\n",i,buff[i]);
			}
		}
		else if((buff[0] == 0xAB)&&(buff[packet_len-1]==0x88 ))
		{
			NRF_LOG_INFO(" packet_len:%d\r\n",packet_len);
			actual_length = packet_len;
			for(uint8_t i=0;i<actual_length;i++)
			{				
				NRF_LOG_INFO(" buff[%d]:%x\r\n",i,buff[i]);
			}
		}
		else
		{
			disconnect_app(); //随机数验证，APP 加密后的结果不相等，断开
		} 
		NRF_LOG_INFO(" ++++++++++++++++++++++++ \r\n");

		if((buff[0] == 0xAB)&&(buff[actual_length-1] == 0x88))
		{
			data_len = ((buff[1]<<8)|buff[2]);
			command_id = (buff[3]<<8|buff[4]);
			crc=0;

			for(int8_t i=0;i<packet_len-2;i++)
			{
				crc = crc + buff[i];
			}
			NRF_LOG_INFO(" crc  is ok \r\n");

			if(crc == buff[actual_length-2])//crc
			{
				if(command_id == 0xFF30) //升级指令
				{
					bootloaderMode=1;
					system_upgrade_info_send();
					bootloaderMode=0;

					err_code = app_timer_start(DFU_TIME_id, APP_TIMER_TICKS(10), NULL);
					APP_ERROR_CHECK(err_code);
				}		
				if(command_id == 0xFF58) //find，APP发送手机的ID给BLE 手表
				{
					memset(test_buff,0,sizeof(test_buff));
					memcpy(test_buff,buff+7,data_len-6);
					find_whilt_flag = true;				
					judg_app_flag = true; 							
					guard_time_manger(false);								
					NRF_LOG_INFO(" Rec data comm:%4x,data_len:%d \n",command_id,data_len);
					NRF_LOG_INFO(" ++++++ 0xFF58 0xFF58 0xFF58 \n");
				}
				//0x ab 00 07 ff 21 80 00 01 53 88
				if(command_id == 0xFF21) //苹果手机连接后，发送过来的,苹果先发这个，后发0xff58
				{  	
					ble_gap_sec_params_t params;

					params.bond = 0;
					params.mitm = 1;

					sd_ble_gap_authenticate(m_conn_handle, &params);				

					NRF_LOG_INFO(" ++++++ 0xFF21 0xFF21 0xFF21 \n");	 
					NRF_LOG_INFO(" ++++++ 0xFF21 0xFF21 0xFF21 \n");
				}					
				if((command_id != 0xFF30)&&(command_id != 0xFF58)&&(command_id != 0xFF21)&&(judg_app_flag == true))
				{
					UART_WakeUp();

					nrf52810_outio_irq();
					
					for(uint32_t i = 0; i < actual_length; i++)
					{
						do
						{										
							err_code = app_uart_put(buff[i]);//to 9160
							NRF_LOG_INFO(" 1+++err_code:%d \r\n",err_code);
							if((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
							{
								NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
								APP_ERROR_CHECK(err_code);
							}
						}while(err_code == NRF_ERROR_BUSY);
					}
					NRF_LOG_INFO("Tx uart over!");
				}

				//清零准备接收下一条
				crc = 0; 
				memset(buff,0,sizeof(buff));
			}
		}
	}
}	
