
#ifndef INC_GSM_H_
#define INC_GSM_H_

#include "main.h"

extern UART_HandleTypeDef huart1;

void gps_init(void);
void send_cmd(const char *cmd);
void send_sms(const char *number, const char *message);


#endif /* INC_GSM_H_ */
