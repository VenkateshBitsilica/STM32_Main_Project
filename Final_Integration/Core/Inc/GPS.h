
#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "main.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

float NMEA_to_Decimal(float coordinate);
void Parse_GPS_Data(char *strParse);
int Validate_GPS_data(char *nmea);

#endif /* INC_GPS_H_ */
