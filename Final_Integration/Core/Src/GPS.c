
#include "GPS.h"

uint8_t rxBuffer[128] = {0};
uint8_t rxIndex = 0;
extern uint8_t rxData;
float nmeaLong;
float nmeaLat;
float utcTime;
char northsouth;
char eastwest;
char posStatus;

extern float Latitude;
extern float Longitude;

float NMEA_to_Decimal(float coordinate) {
    int degree = (int)(coordinate/100);
    float minutes = coordinate - degree * 100;
    float decimalDegree = minutes / 60;
    float decimal = degree + decimalDegree;
    return decimal;
}

void Parse_GPS_Data(char *strParse){
  if(!strncmp(strParse, "$GPGGA", 6)){
    sscanf(strParse, "$GPGGA,%f,%f,%c,%f,%c",
      &utcTime, &nmeaLat, &northsouth, &nmeaLong, &eastwest);
    Latitude = NMEA_to_Decimal(nmeaLat);
    Longitude = NMEA_to_Decimal(nmeaLong);
  }
  else if (!strncmp(strParse, "$GPGLL", 6)){
    sscanf(strParse, "$GPGLL,%f,%c,%f,%c,%f",
      &nmeaLat, &northsouth, &nmeaLong, &eastwest, &utcTime);
    Latitude = NMEA_to_Decimal(nmeaLat);
    Longitude = NMEA_to_Decimal(nmeaLong);
  }
  else if (!strncmp(strParse, "$GPRMC", 6)){
    sscanf(strParse, "$GPRMC,%f,%c,%f,%c,%f,%c",
      &utcTime, &posStatus, &nmeaLat, &northsouth, &nmeaLong, &eastwest);
    Latitude = NMEA_to_Decimal(nmeaLat);
    Longitude = NMEA_to_Decimal(nmeaLong);
  }
}

int Validate_GPS_data(char *nmea){
    char check[3];
    char checksum_str[3];
    int index;
    int checkSum;

    index=0;
    checkSum=0;

    // Ensure that the string starts with a "$"
    if(nmea[index] == '$')
        index++;
    else
        return 0;

    /*
     * No NULL reached, 75 char largest possible NMEA message, no '*' reached
     * Calculate checkSum for characters between $ and *
     * Checksum should be equal to Characters after * in rxBuffer string
     */

    while((nmea[index] != 0) && (nmea[index] != '*') && (index < 75)){
    	checkSum ^= nmea[index];
        index++;
    }

    if(index >= 75){
        return 0;
    }

    if (nmea[index] == '*'){
        check[0] = nmea[index+1];
        check[1] = nmea[index+2];
        check[2] = 0;
    }
    else
        return 0;

    sprintf(checksum_str,"%02X",checkSum);
    return((checksum_str[0] == check[0])
        && (checksum_str[1] == check[1])) ? 1 : 0 ;
}
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==USART3)
  {
    // if the character received is other than 'enter' ascii13, save the data in buffer
    if(rxData!='\n' && rxIndex < sizeof(rxBuffer))
    {
      rxBuffer[rxIndex++]=rxData;
    }
    else
    {
      if(Validate_GPS_data((char*) rxBuffer)) Parse_GPS_Data((char*) rxBuffer);
      rxIndex=0;
      memset(rxBuffer,0,sizeof(rxBuffer));
    }
    HAL_UART_Receive_IT(&huart3,&rxData,1); // Enabling interrupt receive again
  }
}
*/


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART3)
  {
    if(rxData != '\n' && rxIndex < sizeof(rxBuffer) - 1)
    {
      rxBuffer[rxIndex++] = rxData;
    }
    else
    {
      rxBuffer[rxIndex] = '\0';

      if(Validate_GPS_data((char*)rxBuffer))
      {
    	  Parse_GPS_Data((char*)rxBuffer);

        // Print only after full sentence is parsed
        //printf("Buffer : %s\n",rxBuffer);
        //printf("Latitude : %.6f %c, Longitude : %.6f %c\r\n",Latitude, northsouth, Longitude, eastwest);
      }
      else
      {
        printf("Invalid NMEA: %s\r\n", rxBuffer);
      }

      rxIndex = 0;
      memset(rxBuffer, 0, sizeof(rxBuffer));
    }

    // Re-enable interrupt
    HAL_UART_Receive_IT(&huart3, &rxData, 1);
  }
}
