#include "init.h"
GPIO_InitTypeDef gpio,GPIO_InitStructure;

SPI_InitTypeDef SPI_InitStructure;
uint8_t spi(uint8_t data)
{
	 uint8_t stat;
	SPI1->DR = data;
	stat=data;
    while((SPI1->SR & SPI_FLAG_BSY) >0);
	return stat;
}




