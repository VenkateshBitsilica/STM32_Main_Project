
#ifndef INC_WIFI_H_
#define INC_WIFI_H_

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

uint8_t ESP_WaitForResponse(char *expected, uint32_t timeout_ms);
void WiFi_Connect(void);
uint32_t get_time(void);

#endif /* INC_WIFI_H_ */
