#include "DS18B20.h"
void delay(uint32_t del)
{
	for(volatile uint32_t i = 0; i<del; i++); 
} 
void send_presence() 
{ 	
	ds_port->ODR = ds_pin;
	delay(100);
	ds_port->ODR = 0;
	delay(3500); //420us
	ds_port->ODR = ds_pin;
}
	/*
void one_wire_write_bit(uint8_t bit)
{
	GPIOA->ODR = 0;
	delay(bit ? 150 : 500);
	GPIOA->ODR = GPIO_Pin_3;
	delay(bit ? 650 : 200);
}

uint8_t one_wire_read_bit()
{
	uint8_t bit = 0;
	GPIOA->ODR = 0;
	delay(80);
	GPIOA->ODR = GPIO_Pin_3;
	delay(50);
	GPIOA->CRL &= ~GPIO_CRL_MODE3;
	GPIOA->CRL &= ~GPIO_CRL_CNF3;
	GPIOA->CRL |=  GPIO_CRL_CNF3_0;
	bit = (GPIOA->IDR&GPIO_Pin_3?1:0);
	GPIOA->CRL |=  GPIO_CRL_MODE3;
	GPIOA->CRL |=  GPIO_CRL_CNF3_0;
	delay(600);
	return bit;
}

void one_wire_write_byte(uint8_t data)
{
	for(uint8_t i = 0; i<8; i++) 		one_wire_write_bit(data>>i & 1);
}
*/