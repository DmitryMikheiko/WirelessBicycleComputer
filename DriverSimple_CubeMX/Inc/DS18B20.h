/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#define ds_port GPIOC
#define ds_pin  GPIO_PIN_13
#define CRx CRH
#define GPIO_CRx_MODEx GPIO_CRH_MODE13
#define GPIO_CRx_CNFx GPIO_CRH_CNF13
#define GPIO_CRx_CNFx_0 GPIO_CRH_CNF13_0
GPIO_InitTypeDef ds_pin_read;
GPIO_InitTypeDef ds_pin_write;
uint8_t DS18B20_init=0;
enum DS18B20_State{DS_Undefine,DS_Ready,DS_Run}DS_State=DS_Undefine;

extern void delay_us(uint16_t time_);

uint8_t one_wire_reset(void);
uint8_t DS18B20_Init(void);
void    one_wire_write_byte(uint8_t data);
uint8_t one_wire_read_byte(void);
float DS18B20_GetFloatTemp(uint16_t temp);
uint8_t DS18B20_Start(void);
uint16_t DS18B20_ReadTemp(void);
