/*
 * This file is used to perform read and write operations on the internal flash memory
 * of the STM32L476RG microcontroller.
 *
 * Pages 1 to 10 (each 2 KB in size) are used in a circular buffer fashion
 * to store sensor data persistently across resets.
 *
 * Although only a small amount of data is stored per write, entire 2 KB pages are used
 * to simplify page-based erase and write operations.
 *
 * Each page holds one sensor record, and when page 10 is reached,
 * the buffer wraps around to page 1, overwriting old data.
 */

#include "Flash_Page.h"

uint32_t Pageerror;
uint8_t count = 0;

void write_data(uint16_t Sensor_data,uint32_t time)
{
	char buf[100];
	char buffer[50];
	time_t rawtime = time/1000;
	struct tm *timeinfo = gmtime(&rawtime);
	strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);


	sprintf(buf,"%s --> Distance : %d",buffer,Sensor_data);
	page_write(buf, count);
	count++;
	if(count > 10)
		count = 0;
}

void read_data(void)
{
	uint8_t i = count;

	do
	{
		i = (i+1) % 11;
		page_read(i);
	}while(i != count);
}

void sample_main(void)
{
	int count = 0;
	//uint32_t Sensor_data;
	char buf[64];
	while(1)
	{
		//Sensor_data = sensor_data();
		sprintf(buf,"Count : %d",count);
		page_write(buf, count);
		count++;
		if(count > 10)
		{
			while(count >0)
			{
				page_read(count);
				count--;
			}
		}
		HAL_Delay(500);
	}
}

void page_erase(uint8_t page)
{
	HAL_FLASH_Unlock();

	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.Banks = FLASH_BANK_2;
	erase.Page = page;
	erase.NbPages = 1;

	if(HAL_FLASHEx_Erase(&erase, &Pageerror) != HAL_OK)
	{
		printf("Page:%d Erase Failed\n",page);
	}
	HAL_FLASH_Lock();
}

void page_write(char *str, uint8_t page)
{
	uint32_t addr = PAGE1_BASE_ADDR + (page * PAGE_SIZE);
	uint8_t i =0;
	uint64_t data;

	page_erase(page);

	HAL_FLASH_Unlock();

	while(str[i] != '\0')
	{
		data =0;

		for(int j=0; j<8; j++)
		{
			if(str[i] != '\0')
				((uint8_t *)&data)[j] = str[i++];
			else
				((uint8_t *)&data)[j] = '\0';
		}

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, data) != HAL_OK)
        {
        	printf("Flash write failed at addr 0x%08lX!\r\n", addr);
            break;
        }
       addr += 8;
	}
	HAL_FLASH_Lock();
}

void page_read(uint8_t page)
{
	char *addr = (char *)(PAGE1_BASE_ADDR + (page * PAGE_SIZE));
	char str[100];

    int i = 0;

    while (1)
    {
        str[i] = *addr;
        if (str[i] == '\0')
            break;
        i++;
        addr++;
    }
    str[i] = '\0';
    printf("%s",str);
    uart_print(str);

}
