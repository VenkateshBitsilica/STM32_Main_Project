

#ifndef INC_FLASH_PAGE_H_
#define INC_FLASH_PAGE_H_

#include "main.h"
#include "wifi.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

extern FLASH_EraseInitTypeDef erase;
extern HAL_StatusTypeDef status;


#define PAGE_SIZE          0x800                // 2KB
#define PAGE1_BASE_ADDR    0x08080000           // Start of Bank 2

void sample_main(void);
void page_read(uint8_t page);
void page_write(char *str, uint8_t page);
void page_erase(uint8_t page);
extern void uart_print(char *str);
extern uint32_t sensor_data(void);

void read_data(void);
void write_data(uint16_t Sensor_data,uint32_t time);

#endif /* INC_FLASH_PAGE_H_ */
