#ifndef	FENG_TWI_H
#define	FENG_TWI_H
 
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nrf_drv_twi.h"

#define TP_SCL_PIN             (5)    // SCL signal pin 5
#define TP_SDA_PIN             (6)    // SDA signal pin 6

#define TP_EINT_PIN             (9)    // 中断 signal pin
#define TP_RSET_PIN             (10)   // 复位 signal pin
 
#define	TP_I2C_ADD					(0x15)


void nrf_twi_rx(uint8_t reg_addr,uint8_t *p_data,uint8_t length);
void nrf_twi_tx(uint8_t reg_addr,uint8_t reg_value);
void read_tp_id(void);
void twi_init (void);



#endif
