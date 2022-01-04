#ifndef	FENG_FSTORAGE_H
#define	FENG_FSTORAGE_H
#include "app.h"

extern	bool flash_read_flag;
extern 	bool	write_flag;
extern 	bool find_whilt_flag;
extern	bool	is_find_flag; //是否找到
extern	uint8_t write_buff[16];

void fs_init(void);
void	bath_read(void);
//bool updata_records(uint8_t *insert_buff);
bool updata_records();
bool add_whilt_list(uint8_t *find_buff);
 

#endif
