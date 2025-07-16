
#include "WQ25128J_Flash.h"

extern SPI_HandleTypeDef hspi2;

#define csLOW()		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET)
#define csHIGH()	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET)

#define BLOCKS		256

void SPI_Write(uint8_t *data, uint32_t len)
{
	HAL_SPI_Transmit(&hspi2, data, len, 1000);
}

void SPI_Read(uint8_t *data, uint32_t len)
{
	HAL_SPI_Receive(&hspi2, data, len, 1000);
}

void Flash_WriteStatus(void)
{
    uint8_t cmd = 0x05;  // Read Status Register
    uint8_t status = 0;

    csLOW();
    SPI_Write(&cmd, 1);

    do {
        SPI_Read(&status, 1);
    } while (status & 0x01);  // Wait while BUSY bit is set

    csHIGH();
}

uint8_t Flash_ReadStatus(void)
{
    uint8_t cmd = 0x05; // Read Status Register
    uint8_t status = 0;

    csLOW();
    SPI_Write(&cmd, 1);
    SPI_Read(&status, 1);
    csHIGH();

    return status;
}

void Flash_Reset(void)
{
	uint8_t data[2];
	data[0] = 0x66;		//Enable Reset
	data[1] = 0x99;		//Reset
	csLOW();
	HAL_SPI_Transmit(&hspi2, data, 2, 100);
	csHIGH();
}

uint32_t Read_ID(void)
{
	uint8_t data = 0x9F;
	uint8_t read_data[3];

	csLOW();
	HAL_SPI_Transmit(&hspi2, &data, 1, 100);
	HAL_SPI_Receive(&hspi2, read_data, 3, 300);
	csHIGH();

	return ((read_data[0] << 16) | (read_data[1] << 8) | (read_data[2]));
}

void Flash_Read(uint32_t startPage, uint16_t offset, uint32_t size, uint8_t *data)
{
	uint8_t read_addr[4];
	uint32_t memAddr = (startPage*256) + offset;

	read_addr[0] = 0x03;		//Enables the Read data in Flash
	read_addr[1] = (memAddr >> 16) & 0xFF;
	read_addr[2] = (memAddr >> 8) & 0xFF;
	read_addr[3] = memAddr & 0xFF;

	csLOW();

	SPI_Write(read_addr, 4);

	SPI_Read(data, size);

	csHIGH();
}

void Write_enable(void)
{
	uint8_t data = 0x06; 	//Enable Write

	csLOW();
	SPI_Write(&data, 1);
	csHIGH();

	HAL_Delay(5);
}

void Write_disable(void)
{
	uint8_t data = 0x04; 	//Enable Write

	csLOW();
	SPI_Write(&data, 1);
	csHIGH();

	HAL_Delay(5);
}

/*
 * Byte that are remaining in sector to write data into it
 */
uint32_t bytes_to_modify(uint32_t size, uint16_t offset)
{
	if((size+offset)<4096) return size;
	else return (4096-offset);
}

void Flash_Erase_sector(uint16_t sector)
{
	uint8_t Erase_addr[4];
	uint32_t memAddr = sector*16*256;	//Each sector contains 16 pages * 256 bytes

	Write_enable();

	Erase_addr[0] = 0x20;
	Erase_addr[1] = (memAddr >> 16) & 0xFF;
	Erase_addr[2] = (memAddr >> 8) & 0xFF;
	Erase_addr[3] = memAddr & 0xFF;

	csLOW();

	SPI_Write(Erase_addr, 4);
	csHIGH();

	HAL_Delay(450);	//for erasing it need 400ms delay as per datasheet

	Write_disable();
}


/*
 * This will clears the entire sector and write new data page by page
 */

#if 1
void Flash_Write(uint32_t page, uint16_t offset, uint32_t size, uint8_t *tdata)
{
    uint8_t data[266];  		// 4 bytes cmd + 256 bytes max
    uint32_t dataPosition = 0;
    uint32_t startPage = page;

    while (size > 0)
    {
        uint16_t pageOffset = offset;
        uint16_t bytesToWrite = (size > (256 - pageOffset)) ? (256 - pageOffset) : size;
        uint32_t memAddr = (startPage * 256) + pageOffset;

        Write_enable();

        data[0] = 0x02; // Write command
        data[1] = (memAddr >> 16) & 0xFF;
        data[2] = (memAddr >> 8) & 0xFF;
        data[3] = memAddr & 0xFF;

        for (int i = 0; i < bytesToWrite; i++)
            data[4 + i] = tdata[dataPosition + i];

        csLOW();
        SPI_Write(data, 4 + bytesToWrite);
        csHIGH();

        //HAL_Delay(5);  	// Wait for write completion
        Flash_WriteStatus();
        Write_disable();

        size -= bytesToWrite;
        dataPosition += bytesToWrite;
        offset = 0;
        startPage++;
    }
}
#endif

#if 0
void Flash_Write_Update(uint32_t page, uint16_t offset, uint32_t size, uint8_t *data)
{

	uint16_t startSector = page/16;
	uint16_t endSector = (page + ((size+offset-1)/256))/16;
	uint16_t numSector = endSector-startSector+1;

	uint8_t previousData[4096];	//To store complete data present in Flash sector
	/*
	 * Consider i want to update data at page 20 and 100th byte of that page
	 * Page = 20; offset = 100
	 * 20%16 = 4
	 * 4*256 + 100 = 1124
	 */
	uint32_t sectorOffset = ((page%16)*256)+offset;
	uint32_t dataindx = 0;


	for(uint16_t i=0; i<numSector; i++)
	{
		uint32_t startPage = startSector * 16;
		Flash_Read(startPage, 0, 4096, previousData);

		uint16_t bytesRemaining = bytes_to_modify(size, sectorOffset);

		for(uint16_t i=0; i<bytesRemaining; i++)
		{
			previousData[i+sectorOffset] = data[i+dataindx];
		}

		Flash_Erase_sector(startSector);
		Flash_Write(startPage, 0, 4096, previousData);

		startSector++;
		sectorOffset = 0;
		dataindx = dataindx + bytesRemaining;
		size = size-bytesRemaining;

	}
}
#endif

/*
 * This will update the data in Pages without clearing the entire sector
 * Read data from the sector of 4kb(4096 bytes) and store it in buffer
 * Update buffer with new data and write buffer back to sector again
 */
#if 1
void Flash_Write_Update(uint32_t page, uint16_t offset, uint32_t size, uint8_t *data)
{
    uint16_t startSector = page / 16;
    uint16_t endSector = (page + ((size + offset - 1) / 256)) / 16;
    uint16_t numSectors = endSector - startSector + 1;

    uint8_t previousData[4096];
	/*
	 * Consider i want to update data at page 20 and 100th byte of that page
	 * Page = 20; offset = 100
	 * 20%16 = 4
	 * 4*256 + 100 = 1124
	 */
    uint32_t sectorOffset = (page % 16) * 256 + offset;
    uint32_t dataIndex = 0;

    for (uint16_t s = 0; s < numSectors; s++)
    {
        uint32_t startPage = startSector * 16;

        Flash_Read(startPage, 0, 4096, previousData);
        uint16_t bytesRemaining = bytes_to_modify(size, sectorOffset);

        bool erase = false;
        for (uint16_t i = 0; i < bytesRemaining; i++)
        {
            if (previousData[sectorOffset + i] != 0xFF)
            {
            	erase = true;
                break;
            }
        }

        for (uint16_t i = 0; i < bytesRemaining; i++)
        {
            if ((sectorOffset + i) < 4096 && (dataIndex + i) < size)
            {
                previousData[sectorOffset + i] = data[dataIndex + i];
            }
        }

        if (erase)
        {
            Flash_Erase_sector(startSector);
        }

        Flash_Write(startPage, 0, 4096, previousData);

        startSector++;
        sectorOffset = 0;
        dataIndex += bytesRemaining;
        size -= bytesRemaining;
    }
}

#endif





