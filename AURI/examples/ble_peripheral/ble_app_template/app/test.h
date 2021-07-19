#ifndef	TEST_H
#define	TEST_H
#include "app.h"

extern uint8_t test_buff[16];

#define	PM_BOND_SWITCH	0

uint32_t system_time_manger(bool ON_OFF_CMD);
uint32_t system_time_init(void);
void send_data(void);

#endif
