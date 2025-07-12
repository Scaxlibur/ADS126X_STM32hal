#include "myspi.h"

//      CS    <------  PA4     	 SPI_CS
//      DIN   <------  PA7       SPI_MOSI
//      DOUT  ------>  PA6       SPI_MISO
//      SCLK  <------  PA5       SPI clock

void MySPI_W_SS(uint8_t BitValue)
{
	if(BitValue) 
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
}

void MySPI_W_SCK(uint8_t BitValue)
{
	if(BitValue) 
		HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_RESET);
}

void MySPI_W_MOSI(uint8_t BitValue)
{
	if(BitValue) 
		HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_RESET);
}

uint8_t MySPI_R_MISO(void)
{
	return HAL_GPIO_ReadPin(MISO_GPIO_Port, MISO_Pin);
}

void MySPI_Init(void)
{
	MySPI_W_SS(1);
	MySPI_W_SCK(0);
}

void MySPI_Start(void)
{
	MySPI_W_SS(0);
}

void MySPI_Stop(void)
{
	MySPI_W_SS(1);
}

uint8_t MySPI_SwapByte(uint8_t ByteSend)
{
	uint8_t i, ByteReceive = 0x00;
	
	for (i = 0; i < 8; i ++)
	{
		MySPI_W_MOSI(ByteSend & (0x80 >> i));
		MySPI_W_SCK(1);
		if (MySPI_R_MISO() == 1){ByteReceive |= (0x80 >> i);}
		MySPI_W_SCK(0);
	}
	
	return ByteReceive;
}
