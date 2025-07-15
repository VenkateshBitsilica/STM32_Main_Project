
#include "wifi.h"

uint8_t ESP_WaitForResponse(char *expected, uint32_t timeout_ms)
{
    char buffer[300] = {0};
    uint32_t timeStart = HAL_GetTick();
    uint16_t i = 0;
    uint8_t ch;

    while ((HAL_GetTick() - timeStart) < timeout_ms && i < sizeof(buffer) - 1)
    {
        if (HAL_UART_Receive(&huart1, &ch, 1, 1) == HAL_OK)
        {
            buffer[i++] = ch;
            buffer[i] = '\0';
            if (strstr(buffer, expected))
            {
                printf("ESP8266: %s\r\n", buffer);
                return 1;
            }
        }
    }

    printf("ESP8266 TIMEOUT OR FAILED: %s\r\n", buffer);
    return 0;
}

void WiFi_Connect(void)
{
  	char ATcommand[50];
    uint8_t rxBuffer[200] = {0};

	sprintf(ATcommand,"AT+RST\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
	HAL_UART_Receive (&huart1, rxBuffer, 512, 100);
	printf("RESET Response:\r\n%s\r\n", rxBuffer);
	HAL_Delay(500);

	// Set Wi-Fi mode to Station
	  printf("Setting WiFi Mode = Station...\r\n");
	  sprintf(ATcommand, "AT+CWMODE_CUR=1\r\n");
	  memset(rxBuffer, 0, sizeof(rxBuffer));
	  HAL_UART_Transmit(&huart1, (uint8_t *)ATcommand, strlen(ATcommand), 1000);
	  HAL_UART_Receive(&huart1, rxBuffer, sizeof(rxBuffer), 1000);
	  printf("CWMODE Response:\r\n%s\r\n", rxBuffer);

	  printf("Connecting to WiFi: venky...\r\n");
	  sprintf(ATcommand, "AT+CWJAP_CUR=\"venky\",\"11223344\"\r\n");
	  HAL_UART_Transmit(&huart1, (uint8_t *)ATcommand, strlen(ATcommand), 1000);

	  // Use the new function to wait for full response
	  if (ESP_WaitForResponse("OK", 15000) || ESP_WaitForResponse("WIFI GOT IP", 15000))
	  {
	      printf("WiFi Connected Successfully!\r\n");
	  }
	  else
	  {
	      printf("WiFi Connection Failed!\r\n");
	  }

}

uint32_t get_time(void)
{
    char ATcmd[128];
    char buffer[300] = {0};      // Big enough to hold +IPD response
    uint8_t ntpPacket[48] = {0};
    ntpPacket[0] = 0x1B;         // NTP request header (LI=0, VN=3, Mode=3)

    // Step 1: Start UDP connection
    sprintf(ATcmd, "AT+CIPSTART=\"UDP\",\"pool.ntp.org\",123\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)ATcmd, strlen(ATcmd), 1000);
    if (!ESP_WaitForResponse("OK", 5000))
    {
        printf("CIPSTART failed.\r\n");
        return 0;
    }

    // Step 2: Send CIPSEND command
    sprintf(ATcmd, "AT+CIPSEND=48\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)ATcmd, strlen(ATcmd), 1000);
    if (!ESP_WaitForResponse(">", 3000))
    {
        printf("CIPSEND prompt not received.\r\n");
        return 0;
    }

    // Step 3: Transmit 48-byte NTP request packet
    HAL_UART_Transmit(&huart1, ntpPacket, 48, 1000);
    if (!ESP_WaitForResponse("SEND OK", 3000))
    {
        printf("NTP packet send failed.\r\n");
        return 0;
    }

    // Step 4: Wait for response and accumulate data
    uint32_t tickStart = HAL_GetTick();
    uint16_t idx = 0;
    uint8_t byte;

    while ((HAL_GetTick() - tickStart) < 5000 && idx < sizeof(buffer) - 1)
    {
        if (HAL_UART_Receive(&huart1, &byte, 1, 10) == HAL_OK)
        {
            buffer[idx++] = byte;
            buffer[idx] = '\0';  // Null-terminate for strstr
            if (strstr(buffer, "+IPD,48:"))
                break;
        }
    }


    // Step 5: Look for +IPD,48: response
    char *payload_start = strstr(buffer, "+IPD,48:");
    if (payload_start)
    {
        payload_start += strlen("+IPD,48:");

        // Check if full 48-byte NTP payload is available
        if ((payload_start - buffer + 48) > idx)
        {
            // Not enough data received yet – continue receiving
            while ((HAL_GetTick() - tickStart) < 5000 && idx < sizeof(buffer))
            {
                if (HAL_UART_Receive(&huart1, &byte, 1, 10) == HAL_OK)
                {
                    buffer[idx++] = byte;
                    buffer[idx] = '\0';
                    if ((payload_start - buffer + 48) <= idx)
                        break;
                }
            }
        }

        // Now extract time from payload_start[40] to payload_start[43]
        uint8_t *ntp_data = (uint8_t *)payload_start;
        uint32_t ntp_seconds = (ntp_data[40] << 24) | (ntp_data[41] << 16) |
                               (ntp_data[42] << 8) | ntp_data[43];
        uint32_t unix_time = ntp_seconds - 2208988800UL;

        printf("NTP Time (UTC): %lu\r\n", unix_time);
        return unix_time;
    }
    else
    {
        printf("NTP response invalid: +IPD not found.\r\n");
        return 0;
    }

}

