#include "DS18B20.h"
/*void delay(uint32_t del)
{
	for(volatile uint32_t i = 0; i<del; i++); 
} */
uint8_t one_wire_reset() 
{ uint8_t bit = 0;	
	ds_port->BSRR = ds_pin; 
	delay_us(10);
 	ds_port->BRR = ds_pin;  
	delay_us(500); 	
	ds_port->BSRR = ds_pin;
	ds_port->CRx &= ~GPIO_CRx_MODEx;
	ds_port->CRx &= ~GPIO_CRx_CNFx;
	ds_port->CRx |=  GPIO_CRx_CNFx_0;
	delay_us(70);
	bit = (ds_port->IDR&ds_pin?0:1);
	while (ds_port->IDR&ds_pin?0:1);
	ds_port->CRx |=  GPIO_CRx_MODEx;
	ds_port->CRx |=  GPIO_CRx_CNFx_0;
	delay_us(450);
	return bit;
}
void one_wire_read_mode(void)
{
	 HAL_GPIO_Init(ds_port, &ds_pin_read);
}
void one_wire_write_mode(void)
{
	 HAL_GPIO_Init(ds_port, &ds_pin_write);
}	
void one_wire_write_bit(uint8_t bit)
{
	ds_port->BRR = ds_pin;
	delay_us(bit ? 5 : 70);
	ds_port->BSRR = ds_pin;
	delay_us(bit ? 70 : 5);
}

uint8_t one_wire_read_bit()
{
	uint8_t bit = 0;
	ds_port->BRR = ds_pin;
	delay_us(6);
	ds_port->BSRR = ds_pin;
	delay_us(10);
	ds_port->CRx &= ~GPIO_CRx_MODEx;
	ds_port->CRx &= ~GPIO_CRx_CNFx;
	ds_port->CRx |=  GPIO_CRx_CNFx_0;
	bit = (ds_port->IDR&ds_pin?1:0);
	while (ds_port->IDR&ds_pin?0:1);
	ds_port->CRx |=  GPIO_CRx_MODEx;
	ds_port->CRx |=  GPIO_CRx_CNFx_0;
	delay_us(10);
	return bit;
}
uint8_t one_wire_read_byte(void)
{
	uint8_t b=0,i;
	for(i=0;i<8;i++) b+=(uint8_t) one_wire_read_bit()<<i;
	return b;
}
void one_wire_write_byte(uint8_t data)
{
	for(uint8_t i = 0; i<8; i++) 		one_wire_write_bit(data>>i & 1);
}
uint8_t DS18B20_Start(void)
{
	uint8_t state=0;
	if(DS_State!=DS_Undefine)
	{
	state=one_wire_reset();
	one_wire_write_byte(0xCC);
	one_wire_write_byte(0x44);
	DS_State=DS_Run;
	}
	return state;
}
uint16_t DS18B20_ReadTemp(void)
{
	 uint16_t temp=0;
	if(DS_State==DS_Run)
	{
	 if(one_wire_reset())
	 {
		 one_wire_write_byte(0xCC);
		 one_wire_write_byte(0xBE);
		 for(uint8_t i = 0; i<16; i++) temp += (uint16_t)one_wire_read_bit()<<i;
		 DS_State=DS_Ready;
	 }
	 else DS_State=DS_Undefine;
 }
	 return temp;
}
float DS18B20_GetFloatTemp(uint16_t temp)
{
	 float tempf;
	if((temp & 0x8000)==0)
	{
	  tempf=temp;
	  tempf/=16;
	}
	else 
	{
		temp=~temp;
		temp++;
		tempf=temp;
		tempf/=16;
		tempf=-tempf;
	}
	return tempf;
}
uint8_t DS18B20_Init()
{
	uint8_t state=0;
	if(DS_State==DS_Undefine)
	{
	ds_pin_read.Pin = ds_pin;
	ds_pin_read.Mode = GPIO_MODE_INPUT;
	ds_pin_read.Pull = GPIO_NOPULL;
	
	ds_pin_write.Pin = ds_pin;
	ds_pin_write.Mode = GPIO_MODE_OUTPUT_OD;
	ds_pin_write.Pull = GPIO_NOPULL;
	ds_pin_write.Speed = GPIO_SPEED_FREQ_LOW;
	one_wire_write_mode();
	if(one_wire_reset())
	{
	one_wire_write_byte(0xCC);
	one_wire_write_byte(0x4E);
	one_wire_write_byte(0x4B);
	one_wire_write_byte(0x46);
	one_wire_write_byte(0x7F);
		DS_State=DS_Ready;
	}
  }
	else return true;
	return state;
}

