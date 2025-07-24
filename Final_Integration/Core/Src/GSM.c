
#include <GSM.h>

void gps_init(void)
{
  	send_cmd("AT");			//To check GSM module response
  	send_cmd("ATE0");		//Disable Echo command
  	send_cmd("AT+CFUN=1");	//Full Functionality mode
  	send_cmd("AT+CPIN?");	//to check SIM status (Response should be "READY")
  	send_cmd("AT+CSQ");		//to check signal strength
  	send_cmd("AT+CREG?");	//Network registration checking

  	send_sms("+919642593997", "Live Tracking Activated...");
}

void send_cmd(const char *cmd)
{
    char tx_buf[100] = {0};
    char rx_buf[200] = {0};
    uint8_t ch;
    int idx = 0;

    snprintf(tx_buf, sizeof(tx_buf), "%s\r\n", cmd);
    HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, strlen(tx_buf), HAL_MAX_DELAY);

    uint32_t start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < 1000)
    {
        if (HAL_UART_Receive(&huart1, &ch, 1, 10) == HAL_OK)
        {
            if (idx < sizeof(rx_buf) - 1)
                rx_buf[idx++] = ch;

            if (strstr(rx_buf, "OK\r\n") || strstr(rx_buf, "ERROR\r\n"))
                break;
        }
    }

    rx_buf[idx] = '\0';
    printf("%s Response:\r\n%s\r\n", cmd, rx_buf);
    HAL_Delay(500);
}

void send_sms(const char *number, const char *message)
{
    char cmd[50];
    uint8_t ch;
    char resp[200] = {0};
    int idx = 0;

    // Set text mode
    send_cmd("AT+CMGF=1");
    send_cmd("AT+CSCS=\"GSM\"");

    // Compose CMGS command
    snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"", number);
    HAL_UART_Transmit(&huart1, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t *)"\r", 1, HAL_MAX_DELAY);

    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 3000)
    {
        if (HAL_UART_Receive(&huart1, &ch, 1, 100) == HAL_OK)
        {
            resp[idx++] = ch;
            if (ch == '>') break;
        }
    }

    if (!strchr(resp, '>')) {
        printf("Didn't receive '>' prompt. SMS not sent.\r\n");
        return;
    }

    printf("Sending SMS to %s...\r\n", number);
    HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);

    // CTRL+Z(command value 26) to end SMS
    uint8_t end_char = 0x1A;
    HAL_UART_Transmit(&huart1, &end_char, 1, HAL_MAX_DELAY);

    idx = 0;
    memset(resp, 0, sizeof(resp));
    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 5000)
    {
        if (HAL_UART_Receive(&huart1, &ch, 1, 100) == HAL_OK)
        {
            resp[idx++] = ch;
            if (strstr(resp, "OK\r\n") || strstr(resp, "ERROR\r\n"))
                break;
        }
    }

    printf("SMS Send Response:\r\n%s\r\n", resp);
}
