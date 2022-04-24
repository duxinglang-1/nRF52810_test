#ifndef	FENG_TWI_H
#define	FENG_TWI_H
 
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nrf_drv_twi.h"

void nrf_twi_rx(uint8_t reg_addr,uint8_t *p_data,uint8_t length);
void nrf_twi_tx(uint8_t reg_addr,uint8_t reg_value);
void twi_init(void);

#endif
