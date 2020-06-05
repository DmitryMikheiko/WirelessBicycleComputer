    
/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "LS020.c"
#include "string.h"
#include "fon.h"
#include "fon2.h"
#include "0S.h"
#include "1S.h"
#include "2S.h"
#include "3S.h"
#include "4S.h"
#include "5S.h"
#include "6S.h"
#include "7S.h"
#include "8S.h"
#include "9S.h"
#include "SS.h"

#include "l0.h"
#include "l1.h"
#include "l2.h"
#include "l3.h"
#include "l4.h"
#include "l5.h"
#include "l6.h"
#include "l7.h"
#include "l8.h"
#include "l9.h"
#include "ll.h"

#include "A1.h"
#include "A2.h"
#include "A3.h"
#include "A4.h"
#include "A5.h"
#include "A6.h"
#include "A7.h"
#include "A8.h"
#include "AA.h"

#include "Battery_low.h"
#include "Battery_0.h"
#include "Battery_1.h"
#include "Battery_2.h"
#include "Battery_3.h"
#include "Battery_4.h"

#include "Menu_Icon_22.h"
#include "Menu_Icon_16.h"

#include "Backlight_Icon_16.h"
#include "Backlight_Icon_22.h"

#include "Power_Icon_16.h"
#include "Power_Icon_22.h"

#include "Light_Icon_16.h"
#include "Light_Icon_22.h"

#include "Connect_Icon_16.h"
#include "Connect_Icon_22.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
#define HEX2BCD(v)	((v) % 10 + (v) / 10 * 16)
#define BCD2HEX(v)	((v) % 16 + (v) / 16 * 10)
RTC_TimeTypeDef Time;
  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
/* Private variables ---------------------------------------------------------*/
#define NRF24L01_Power_on HAL_GPIO_WritePin(NRF24L01_POWER_GPIO_Port, NRF24L01_POWER_Pin, GPIO_PIN_SET);
#define NRF24L01_Power_off HAL_GPIO_WritePin(NRF24L01_POWER_GPIO_Port, NRF24L01_POWER_Pin, GPIO_PIN_RESET);
//#define led_inv HAL_GPIO_TogglePin(led_GPIO_Port,led_Pin);

#define SysFreq 24
#define SYS_FCLK SysFreq

#define InU_Koef 2.0*3.03*1.02667/4095.0

#define SpeedArcMin 226.0
#define SpeedArcAngel 241.0
#define RpmArcMin 280.0 
#define RpmArcAngel 210.0
#define speed_digits_color   Red
#define  fon_color_0_index    1
#define  fon_color_1_index    0
#define  fon_color_2_index    2
//#define  fon_color_3_index    3
u8 ColorBar[sizeof(fon2_ColorBar)];
const u8 ColorBarSize=sizeof(fon2_ColorBar);
const u8 Colors[20]={fon1,fon2,Black,Grey,Red,Orange,Yellow,Yellow2,color1,color2,Blue,Blue2,Blue3,Magenta,Lime,Green,Green2,Green3,Cyan,White};


uint8_t TimeOut=255;
uint8_t TimeOutTimer=0;
uint8_t fps=0;
char s[]="   ";
bool Ready=false;
volatile bool PowerDownState=true;

#define csn_1 HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port,NRF24L01_CSN_Pin,GPIO_PIN_SET); 
#define csn_0 HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port,NRF24L01_CSN_Pin,GPIO_PIN_RESET);
#define ce_1 HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port,NRF24L01_CE_Pin,GPIO_PIN_SET);
#define ce_0 HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port,NRF24L01_CE_Pin,GPIO_PIN_RESET);
#define R_REGISTER  0X00      //Read registers directly address bitwise
#define W_REGISTER  0X20      //write registers      Bitwise OR with address
#define R_RX_PAYLOAD  0X61    //read data 1-32 byte  From the beginning of 0 bytes
#define W_TX_PAYLOAD  0XA0    //write data 1-32 byte  From the beginning of 0 bytes
#define FLUSH_TX  0xE1        //clear TX FIFO regsiters
#define FLUSH_RX  0XE2        //clear RX FIFO regsiters  This command should not be used when the transfer acknowledge signal
#define RESUSE_TX_PL 0XE3     //Re-use on a packet of valid data when CE is high, the data packets continually re-launch
#define NOP  0XFF             //Empty command is used to retrieve data
#define R_RX_PL_WID  0x60
uint8_t state = 0x00;
uint8_t NRF24L01_ID[5]={0xFF,0xFF,0xFF,0xFF,0x02};
uint8_t buffer[33];
uint8_t info_buffer[33];
uint8_t length;
uint8_t TransmitRepeats=0;
char *msg;
uint8_t data;
uint32_t i;
uint8_t count;
bool ConnectError=false;
bool NRF24L01_POWER=false;
bool NRF24L01_Transmit=false;
bool NRF24L01_RX=false;
bool NRF24L01_TX=false;

bool PackageReceived=false;

uint16_t InUmV=0;
float last_speed_percent=225.0;
float last_rpm_percent=225.0;
float speed=0.0;
float rpm=0.0;
u16 wheel_diameter=680;
u16 gears[9]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
double distance=0.0;
double distance_2=0.0;


const uint16_t good_volt=4000;
const uint16_t bad_volt=3100;

float wheel_length=2.13;
struct _MainValues
{
	u16 U_int;
	u16 U_ext;
	u16 I_ext;
	u16 temperature;
	u16 rpm;
	u16 speed;
	double distance;
	u8 gear;
	u8 on_off_channels;
}MainValues;
struct Package
{
	u8 pid;
	u16 Uin;
	u16 Iin;
	u32 rev_count_1;
	u32 rev_count_2;
	u16 rpm_1;
	u16 rpm_2;
	u8  on_off_channels;
	u16 temperature;
	u16 geears;
}package;
union U32U8
{
	uint32_t u32;
	uint8_t u8[4];
}u32u8;
union U16U8
{
	uint16_t u16;
	uint8_t u8[2];
}u16u8;
union U8bits
{
	uint8_t u8;
	struct
	{
		unsigned char bit0:1;
		unsigned char bit1:1;
		unsigned char bit2:1;
		unsigned char bit3:1;
		unsigned char bit4:1;
		unsigned char bit5:1;
		unsigned char bit6:1;
		unsigned char bit7:1;
	}bits;
}u8bits;
uint8_t test_buffer[22]={0x01,0x1,0x02,0x1,0x2,0xAA,0xBB,0xCC,0xDD,0xAA,0xBB,0xCC,0xDD,0x00,0x00,0xFF,0x00,0xFF};
u8 encodermax=0;
u8 encoder[10];
u8 encoder_cnt_pos=0;
#define Encoder htim4.Instance->CNT
#define EncoderMaxVal htim4.Init.Period
#define EncoderMax(max) EncoderMaxVal=max;HAL_TIM_Encoder_Init(&htim4, &sConfig);
#define PushEncoderMax encodermax=EncoderMaxVal;
#define PopEncoderMax EncoderMax(encodermax);
#define PushEncoder encoder[encoder_cnt_pos++]=Encoder;
#define PopEncoder Encoder=encoder[--encoder_cnt_pos];
#define options_max_index 4
u8 menu_color=Blue;
volatile bool Enter=false;
bool menu_status=false;
char str[5]="Hello";
unsigned char volt_string[5];
char options[8][22]=
{"         Back         ",
 "       Set Time       ",
 "    Wheel Diameter    ",
 "  Gears  Calibration  ",
 "        Colors        ",
 "                      ",
 "                      ",
 "                      "};	

#define MenuCount 5
uint8_t SelectMenu = 2;
const unsigned char* MenuImages_16[5]={Power_Icon_16,Backlight_Icon_16,Menu_Icon_16,Light_Icon_16,Connect_Icon_16};
const unsigned char* MenuImages_22[5]={Power_Icon_22,Backlight_Icon_22,Menu_Icon_22,Light_Icon_22,Connect_Icon_22};
const unsigned int  MenuImages_22_Sizes[5]={sizeof(Power_Icon_22),sizeof(Backlight_Icon_22),sizeof(Menu_Icon_22),sizeof(Light_Icon_22),sizeof(Connect_Icon_22)};
const unsigned int  MenuImages_16_Sizes[5]={sizeof(Power_Icon_16),sizeof(Backlight_Icon_16),sizeof(Menu_Icon_16),sizeof(Light_Icon_16),sizeof(Connect_Icon_16)};

uint8_t BacklightMode=1;

bool MenuDisplayI =false;

//Prototypes

unsigned char read_irq(void);
void clr_irq(void);
void NRF24L01_IRQ(void);
void tx_mode(void);
void rx_mode(void);
//void send_data(char *data);
uint8_t read_rx(uint8_t *data);
void NRF24L01_init(void);
void uart_received(void);
void send_data(uint8_t *data,uint8_t size);
void send_msg(char *msg);
void send_u32(uint32_t u32);
void send_hex(uint32_t u32);
void send_float(float f,char presition);
void SendInfo(void);
void SetID(uint8_t *data);
void SetTxAddr(uint8_t *addr);
void SetRxAddr(uint8_t *addr);
void NRF24L01_power_down(void);
void ChangeBacklight(void);
void OpenMenu(u8 color,u8 bcColor);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void LCD_SPI(uint8_t data);
void Core0(void);
void PowerDown(void);
void InitAll(void);
void SystemClock_UserConfig(uint8_t Freq);
void WriteSpeed(float speed);
void WriteSpeedOnly(float speed);
void WriteRpm(float rpm);
void WriteGear(void);
void WriteDistance(u32 distance);
void WriteTime(void);
void WriteOuterBattery(void);
void WriteVoltage(void);
void ButtonPressed(void);
void SetBacklight(uint8_t BacklightMode_);
void ChangeBacklight(void);
void EncoderInit(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t spi(uint8_t address)
{
	HAL_SPI_TransmitReceive(&hspi1, &address, &data, sizeof(data), 0x1000);
	return data;
}
void LCD_SPI(uint8_t data)
{
	lcd_cs_0
	//HAL_SPI_TransmitReceive(&hspi1, &data, &data, sizeof(data), 0x1000);
  SPI2->DR=data;
	while((SPI2->SR &SPI_SR_BSY)>0); 
	lcd_cs_1
}
void delay_us(uint16_t time_)
{
	uint32_t time;
	time=time_;
	time*=SYS_FCLK;
	time/=3;
  while(time--);
}
void soft_delay_ms(uint16_t ms)
{
	while(ms--) delay_us(1000);
}
uint16_t GetInUmV(void)
{
	InUmV=(uint16_t)(((float)(hadc1.Instance->DR))*InU_Koef*1000.0);
	return InUmV;
}
void Core0(void)
{
	TimeOutTimer++;
	if(TimeOutTimer>=TimeOut)
	{
		TimeOutTimer=0;
		PowerDownState=true;
	}
	MenuDisplayI=!MenuDisplayI;
	/*if(Ready)
	{
	s[0]='0'+fps/100;
	fps%=100;
	s[1]='0'+fps/10;
	fps%=10;
	s[2]='0'+fps;
	lcd_put_s(0,0,s,3,Red,White);
	htim1.Instance->CNT=0;
	fps=0;
	}*/
}
void WriteCNT(uint16_t c)
{
	uint16_t mask=10000;
	uint8_t p=0;
	char buffer[5]="00000";
	while(mask>0){buffer[p++]+=c/mask; c%=mask; mask/=10;}
  lcd_put_s_xy(0,108,buffer,5,Red,Green);	
}
void PowerDown(void)
{
	uint8_t d;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	lcd_off();
	NRF24L01_power_down();
	lcd_backlight_off
	PowerDownState=true;
	 HAL_GPIO_WritePin(NRF24L01_POWER_GPIO_Port, NRF24L01_POWER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, backlight_1_Pin|backlight_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NRF24L01_CSN_Pin|NRF24L01_CE_Pin|LCD_RS_Pin|LCD_RESET_Pin 
                          |LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : NRF24L01_POWER_Pin */
 /* GPIO_InitStruct.Pin = NRF24L01_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(NRF24L01_POWER_GPIO_Port, &GPIO_InitStruct);*/

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : backlight_1_Pin backlight_2_Pin LCD_POWER_Pin */
 /* GPIO_InitStruct.Pin = backlight_1_Pin|backlight_2_Pin|LCD_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);*/

  /*Configure GPIO pins : PA3 PA9 PA10 PA11 
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



  /*Configure GPIO pins : NRF24L01_CSN_Pin NRF24L01_CE_Pin LCD_RS_Pin LCD_RESET_Pin 
                           LCD_CS_Pin */
  GPIO_InitStruct.Pin = NRF24L01_CSN_Pin|NRF24L01_CE_Pin|LCD_RS_Pin|LCD_RESET_Pin 
                          |LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB3 PB4 PB5 
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  for(d=0;d<100;d++)
  {
  delay_ms(100);
	if(!PowerDownState) break;
  }
  if(PowerDownState)
  {
	lcd_power_off 
	NRF24L01_Power_off
	HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON,PWR_SLEEPENTRY_WFI);	
  }
}
void WriteSpeed(float speed)
{
	float e;
	float percent;
	u16 speed_i;
	u8 color;
	const uint8_t *dig;
	u16 size;
	if(speed>99.0)
	{
		speed=99.0;
		percent=100.0;
	}
	else if(speed>50.0)
	{
		percent=100.0;
	}
	else percent=speed*2.0;
	if(speed<18.0) color=Lime;
	else if(speed<28.0) color=Yellow;
	else color=Red;
	speed_i=speed;
	e=(SpeedArcAngel*percent)/100.0;
	if(e<last_speed_percent) GUI_Arc(120,70,50,16,SpeedArcMin-last_speed_percent,SpeedArcMin-e,White);
	GUI_Arc(120,70,50,16,SpeedArcMin-e,SpeedArcMin,color);
	last_speed_percent=e;
			
	switch(speed_i/10)
	{
		case 0: dig=SS;size=sizeof(SS);break;
		case 1: dig=S1;size=sizeof(S1);break;
		case 2: dig=S2;size=sizeof(S2);break;
		case 3: dig=S3;size=sizeof(S3);break;
		case 4: dig=S4;size=sizeof(S4);break;
		case 5: dig=S5;size=sizeof(S5);break;
		case 6: dig=S6;size=sizeof(S6);break;
		case 7: dig=S7;size=sizeof(S7);break;
		case 8: dig=S8;size=sizeof(S8);break;
		case 9: dig=S9;size=sizeof(S9);break;
		default: dig=SS;size=sizeof(SS);break;
	}
	lcd_draw_rle_digit_24(95,49,dig,size,speed_digits_color,ColorBar[fon_color_1_index]);
	switch(speed_i%10)
	{
		case 0: dig=S0;size=sizeof(S0);break;
		case 1: dig=S1;size=sizeof(S1);break;
		case 2: dig=S2;size=sizeof(S2);break;
		case 3: dig=S3;size=sizeof(S3);break;
		case 4: dig=S4;size=sizeof(S4);break;
		case 5: dig=S5;size=sizeof(S5);break;
		case 6: dig=S6;size=sizeof(S6);break;
		case 7: dig=S7;size=sizeof(S7);break;
		case 8: dig=S8;size=sizeof(S8);break;
		case 9: dig=S9;size=sizeof(S9);break;
		default: dig=S0;size=sizeof(S0);break;
	}
	lcd_draw_rle_digit_24(122,49,dig,size,speed_digits_color,ColorBar[fon_color_1_index]);	
}
void WriteRpm(float rpm)
{
	float e;
	float percent;
	u16 rpm_i;
	u8 color;
	char str[3]="000";
	if(rpm>300.0) rpm=300.0;
	if(rpm>150.0)
	{		
		percent=100.0;
	}
	else percent=rpm/1.5;
	if(rpm<60.0) color=Lime;
	else if(rpm<90.0) color=Yellow;
	else color=Red;
	rpm_i=rpm;
	e=(RpmArcAngel*percent)/100.0;
	if(e<last_rpm_percent) GUI_Arc(70,63,39,11,RpmArcMin-last_rpm_percent,RpmArcMin-e,White);
	GUI_Arc(70,63,39,11,RpmArcMin-e,RpmArcMin,color);
	last_rpm_percent=e;
	str[0]+=rpm_i/100;
	str[1]+=(rpm_i/10)%10;
	str[2]+=rpm_i%10;
	if(str[0]=='0') str[0]=' ';
	if((str[0]==' ') && (str[1]=='0')) str[1]=' '; 
	lcd_put_s_xy(44,59,str,3,Red,ColorBar[fon_color_1_index]);
}
void WriteSpeedOnly(float speed)
{
	float e;
	float percent;
	u16 speed_i;
	u8 color;
	const uint8_t *dig;
	u16 size;
	
	speed_i=speed;

	switch(speed_i/10)
	{
		case 0: dig=SS;size=sizeof(SS);break;
		case 1: dig=S1;size=sizeof(S1);break;
		case 2: dig=S2;size=sizeof(S2);break;
		case 3: dig=S3;size=sizeof(S3);break;
		case 4: dig=S4;size=sizeof(S4);break;
		case 5: dig=S5;size=sizeof(S5);break;
		case 6: dig=S6;size=sizeof(S6);break;
		case 7: dig=S7;size=sizeof(S7);break;
		case 8: dig=S8;size=sizeof(S8);break;
		case 9: dig=S9;size=sizeof(S9);break;
		default: dig=SS;size=sizeof(SS);break;
	}
	lcd_draw_rle_digit_24(62,35,dig,size,speed_digits_color,ColorBar[fon_color_1_index]); //95 49
	switch(speed_i%10)
	{
		case 0: dig=S0;size=sizeof(S0);break;
		case 1: dig=S1;size=sizeof(S1);break;
		case 2: dig=S2;size=sizeof(S2);break;
		case 3: dig=S3;size=sizeof(S3);break;
		case 4: dig=S4;size=sizeof(S4);break;
		case 5: dig=S5;size=sizeof(S5);break;
		case 6: dig=S6;size=sizeof(S6);break;
		case 7: dig=S7;size=sizeof(S7);break;
		case 8: dig=S8;size=sizeof(S8);break;
		case 9: dig=S9;size=sizeof(S9);break;
		default: dig=S0;size=sizeof(S0);break;
	}
	lcd_draw_rle_digit_24(90,35,dig,size,speed_digits_color,ColorBar[fon_color_1_index]);	//122 49
}
void WriteGear(void)
{ 
	const uint8_t *dig,*dig_ColorBar;
	u16 size;
	u8 gear;
	for(gear=1;gear<8;gear++) if ((package.geears<((gears[gear-1]+gears[gear])/2)) &&(package.geears>((gears[gear+1]+gears[gear])/2))) {break;}		
	switch(gear)
	{
		case 1: dig=A1;dig_ColorBar=A1_ColorBar;size=sizeof(A1);break;
		case 2: dig=A2;dig_ColorBar=A2_ColorBar;size=sizeof(A2);break;
		case 3: dig=A3;dig_ColorBar=A3_ColorBar;size=sizeof(A3);break;
		case 4: dig=A4;dig_ColorBar=A4_ColorBar;size=sizeof(A4);break;
		case 5: dig=A5;dig_ColorBar=A5_ColorBar;size=sizeof(A5);break;
		case 6: dig=A6;dig_ColorBar=A6_ColorBar;size=sizeof(A6);break;
		case 7: dig=A7;dig_ColorBar=A7_ColorBar;size=sizeof(A7);break;
		case 8: dig=A8;dig_ColorBar=A8_ColorBar;size=sizeof(A8);break;
		default: dig=AA;dig_ColorBar=AA_ColorBar;size=sizeof(AA);break;
	}

lcd_draw_rle_digit_16(150,25,dig,dig_ColorBar,size,Black,ColorBar[fon_color_1_index]);

}
void WriteDistance(u32 distance)
{ 
	char ds[9];
	u8 i;
	
	ds[0]='0'+distance/100000;
	distance=distance % 100000;
	ds[1]='0'+distance/10000;
  distance=distance % 10000;
	ds[2]='0'+distance/1000;
	distance=distance % 1000;
	ds[3]='1';
	ds[4]='0'+distance/100;
	distance=distance % 100;
	ds[5]='0'+distance/10;
	distance=distance % 10;
	ds[6]='0'+distance;
	for(i=0;i<7;i++)if (ds[i]>'0') break;
	else ds[i]=' ';
	ds[3]='.';
	ds[7]=' ';
	ds[8]='m';
	lcd_put_s_xy(52,88,(char*)ds,9,Blue,ColorBar[fon_color_1_index]);//99 93

}
void ReadTime(unsigned char *time)
{
	HAL_RTC_GetTime(&hrtc,&Time,RTC_FORMAT_BCD);
	time[0]=BCD2HEX(Time.Hours);
	time[1]=BCD2HEX(Time.Minutes);
}
void WriteTime(void)
{ 
	unsigned char time[2];
  unsigned char time_string[5];

	  ReadTime(time);
		time_string[0]='0'+((time[0]/10) %10);
	  time_string[1]='0'+(time[0] %10);
	  time_string[2]=':';
		time_string[3]='0'+((time[1]/10) %10);
	  time_string[4]='0'+(time[1] %10);
	  lcd_put_s_xy(132,1,(char*)time_string,5,Blue,ColorBar[fon_color_2_index]);
}
void WriteTemp(void)
{
	char ds[4];
	int16_t Temperature;
	Temperature=package.temperature;
		if(PackageReceived)
	{
	if(Temperature & 0x8000) {ds[0]='-'; Temperature&=0x7FFF;}
			else ds[0]=Temperature/100+'0';
		ds[1]=(Temperature%100)/10+'0';
		ds[2]='.';
		ds[3]=Temperature%10+'0';
		if(ds[0]=='0')ds[0]=' ';

	lcd_put_s_xy(91,1,(char*)ds,4,Red,ColorBar[fon_color_2_index]);
  }
}
void WriteOuterBattery(void)
{
	int volt;
  unsigned char volt_string[5];
	uint16_t OuterBatteryUmV=package.Uin;
		if(PackageReceived)
	{
	if(OuterBatteryUmV>3900)
	{
		lcd_draw_rle_picture_fon(1,22,28,88,Battery_4,sizeof(Battery_4),0xFE,ColorBar[fon_color_1_index]);
	}
	else if(OuterBatteryUmV>3800)
	{
		lcd_draw_rle_picture_fon(1,22,28,88,Battery_3,sizeof(Battery_3),0xFE,ColorBar[fon_color_1_index]);
	}
	else if(OuterBatteryUmV>3700)
	{
		lcd_draw_rle_picture_fon(1,22,28,88,Battery_2,sizeof(Battery_2),0xFE,ColorBar[fon_color_1_index]);
	}
	else if(OuterBatteryUmV>3600)
	{
		lcd_draw_rle_picture_fon(1,22,28,88,Battery_1,sizeof(Battery_1),0xFE,ColorBar[fon_color_1_index]);
	}
	else if(OuterBatteryUmV>3500)
	{
		lcd_draw_rle_picture_fon(1,22,28,88,Battery_0,sizeof(Battery_0),0xFE,ColorBar[fon_color_1_index]);
	}
	else
	{
		lcd_draw_rle_picture_fon(1,22,28,88,Battery_low,sizeof(Battery_low),0xFE,ColorBar[fon_color_1_index]);
	}
	if(MenuDisplayI) 
		{
		volt=package.Iin;
		volt_string[4]='A';
		}
	else 
	  {
		volt=OuterBatteryUmV;
		volt_string[4]='V';
  	}
	volt_string[0]=volt/1000+'0';
	volt%=1000;
	volt_string[1]='.';
	volt_string[2]=volt/100+'0';
	volt%=100;
	volt_string[3]=volt/10+'0';
		
		lcd_put_s_xy(3,92,(char*)volt_string,5,Red,ColorBar[fon_color_1_index]);	 
  }
}
void WriteVoltage(void)
{ 
	u8 x0,y0,x1,y1;
	int volt,volt2,part;
   unsigned char volt_string[5];
	u8 color;
	volt=GetInUmV();
	delay_us(300);
	volt+=GetInUmV();
	delay_us(300);
	volt+=GetInUmV();
	volt/=3;
	volt2=volt;
	if(volt<bad_volt)
	{  
	 
	  
	}
	volt_string[0]=volt/1000+'0';
	volt%=1000;
	volt_string[1]='.';
	volt_string[2]=volt/100+'0';
	volt%=100;
	volt_string[3]=volt/10+'0';
	volt_string[4]=volt%10+'0';
	lcd_put_s_xy(43,1,(char*)volt_string,4,Red,ColorBar[fon_color_2_index]);
	
	x0=3;
	y0=4;
	y1=12;
	if(volt2>bad_volt){
	volt2-=bad_volt;
  part=(35*volt2)/(good_volt-bad_volt);
	if(part>35)part=35;}
	else part=0;
	x1=(u8)part+x0;
	if(part>23)color=Lime;
	else if(x1>12)color=Yellow;
	else color=Red;
	GUI_Rectangle(x0,y0,x1,y1,color,1);
	if(x1<38) GUI_Rectangle(x1+1,y0,35+3,y1,White,1);
	

}
void WriteMenu(void)
{
	uint8_t number;
	uint8_t x=25;

	int index=htim4.Instance->CNT;
	if(index-2>=0) index-=2;
	else index=3+index;
	for(number=0;number<MenuCount;number++)
	{
		if(number==2)
		{
			lcd_draw_rle_picture_fon(x,108,x+21,129,MenuImages_22[index],MenuImages_22_Sizes[index],0xFC,ColorBar[fon_color_1_index]);
			x+=32;
		}
		else 
		{
			lcd_draw_rle_picture_fon(x,111,x+21,126,MenuImages_16[index],MenuImages_16_Sizes[index],0xFC,ColorBar[fon_color_1_index]);
			x+=26;
		}
		if(++index>=5)index=0;
		//WriteCNT(htim4.Instance->CNT);
	}

}
void SetBacklight(uint8_t BacklightMode_)
{
	BacklightMode=BacklightMode_;
	switch(BacklightMode)
	{
		case 0:
			lcd_backlight_off
			break;
		case 1:
			lcd_backlight_off
	  	lcd_backlight_1_on
			break;
		case 2:
			lcd_backlight_off
	  	lcd_backlight_2_on 
			break;
		case 3:
			lcd_backlight_1_on
		  lcd_backlight_2_on
			break;
		default: 
			break;
	}

}
void ChangeBacklight(void)
{
	if(BacklightMode<2)BacklightMode++;
	else BacklightMode=0;
	SetBacklight(BacklightMode);
}
void ButtonPressed(void)
{
	uint8_t number=htim4.Instance->CNT;
	if(!PowerDownState)
	{
		if(!menu_status)
	{
	  switch(number)
	  {
		case 0:
			PowerDownState=true;
			break;
		case 1:
			ChangeBacklight();
			break;
		case 2:
			OpenMenu(White,menu_color);
		lcd_draw_rle_picture_CT(0,0,175,131,fon2_image,sizeof(fon2_image),ColorBar);
			break;
		case 3:
			break;
		case 4:
			break;
		default: 
			break;
	  }
		Enter=false;
   }
  }
}
void I_set_time_menu(unsigned char *time_string,u8 opt_pos,bool chFlag,u8 color,u8 bcColor)
{ 
	u8 sbcColor=bcColor;
	u8 scolor=color;
	if(chFlag) {scolor=Red; sbcColor=color;}
	if(opt_pos==0) 
	{
  lcd_put_char(8,3,time_string[0],sbcColor,scolor);
	lcd_put_char(9,3,time_string[1],sbcColor,scolor);
	lcd_put_char(10,3,':',color,bcColor);	
	lcd_put_char(11,3,time_string[2],color,bcColor);
	lcd_put_char(12,3,time_string[3],color,bcColor);
		lcd_put_s(9,5,"Back",4,color,bcColor);
  }
	else if (opt_pos==1)
	{
  lcd_put_char(8,3,time_string[0],color,bcColor);
	lcd_put_char(9,3,time_string[1],color,bcColor);
	lcd_put_char(10,3,':',color,bcColor);	
	lcd_put_char(11,3,time_string[2],sbcColor,scolor);
	lcd_put_char(12,3,time_string[3],sbcColor,scolor);
		lcd_put_s(9,5,"Back",4,color,bcColor);
  }
	else {
	lcd_put_s(9,5,"Back",4,bcColor,color);
	lcd_put_char(8,3,time_string[0],color,bcColor);
	lcd_put_char(9,3,time_string[1],color,bcColor);
	lcd_put_char(10,3,':',color,bcColor);	
	lcd_put_char(11,3,time_string[2],color,bcColor);
	lcd_put_char(12,3,time_string[3],color,bcColor);
	}
}
void SaveTime(u8 *time)
{
	Time.Hours=HEX2BCD(time[0]);
	Time.Minutes=HEX2BCD(time[1]);
	HAL_RTC_SetTime(&hrtc,&Time,RTC_FORMAT_BCD);
}
void time_to_string(unsigned char *time,unsigned char *time_string)
{
    time_string[0]='0'+((time[0]/10) %10);
	  time_string[1]='0'+(time[0] %10);
		time_string[2]='0'+((time[1]/10) %10);
	  time_string[3]='0'+(time[1] %10);
}
void I_set_time(char *str,u8 color,u8 bcColor)
{  
	unsigned char time[2];
	 unsigned char time_string[4];
	u8 opt_pos=0;
	PushEncoderMax
	PushEncoder
	bool chFlag=false;
	Enter=false;
	lcd_clear(bcColor);
	lcd_put_s(0,0,str,22,color,bcColor);
	lcd_put_s(9,5,"Back",4,color,bcColor);
	  ReadTime(time);
		time_to_string(time,time_string);
    I_set_time_menu(time_string,opt_pos,chFlag,color,bcColor);
	while(true)
{
		EncoderMax(2);
		Encoder=opt_pos;
	while(!Enter)
	{
		if(opt_pos!=Encoder)
		{
			opt_pos=Encoder;
			I_set_time_menu(time_string,opt_pos,chFlag,color,bcColor);
		}
    
  }
   Enter=false;
	if(opt_pos==2){SaveTime(time);PopEncoderMax PopEncoder break;}
		else if(opt_pos==0) //**************** HOURS ***********************//
			{ 
				EncoderMax(23);
				Encoder=time[0];
        chFlag=true;
				time_to_string(time,time_string);
				I_set_time_menu(time_string,opt_pos,chFlag,color,bcColor);
	while(!Enter)
	{   
     if(Encoder!=time[0])
		 {
			 time[0]=Encoder;
			 time_to_string(time,time_string);
			 I_set_time_menu(time_string,opt_pos,chFlag,color,bcColor);
		 }
    
  }
	  Enter=false;
			chFlag=false;
	I_set_time_menu(time_string,opt_pos,chFlag,color,bcColor);
      }
		else if(opt_pos==1) //**************** MINUTES ***********************//
      {
				EncoderMax(59);
				Encoder=time[1];
				chFlag=true;
				time_to_string(time,time_string);
				I_set_time_menu(time_string,opt_pos,chFlag,color,bcColor);
	while(!Enter)
	{
    if(Encoder!=time[1])
		 {
			 time[1]=Encoder;
			 time_to_string(time,time_string);
			 I_set_time_menu(time_string,opt_pos,chFlag,color,bcColor);
		 }
  }
	  Enter=false;
			chFlag=false;
	I_set_time_menu(time_string,opt_pos,chFlag,color,bcColor);
      }
}
}
void save_wheel_diameter(void)
{ 
	u8 i;
	union
	{
		u16 ww;
		u8 bw[4];
	}ftob;
	ftob.ww=wheel_diameter;
//	for(i=0;i<2;i++) DS1302_WriteByte(i*2+244,ftob.bw[i]);
}
void whell_menu(unsigned char *wheel_string,u8 opt_pos,bool chFlag,u8 color,u8 bcColor)
{ 
	u8 sbcColor=bcColor;
	u8 scolor=color;
	if(chFlag) {scolor=Red; sbcColor=color;}
	if(opt_pos==0)  lcd_put_char(9,3,wheel_string[0],sbcColor,scolor);
	else            lcd_put_char(9,3,wheel_string[0],color,bcColor);
	if(opt_pos==1)  lcd_put_char(10,3,wheel_string[1],sbcColor,scolor);
	else            lcd_put_char(10,3,wheel_string[1],color,bcColor);
	if(opt_pos==2)  lcd_put_char(11,3,wheel_string[2],sbcColor,scolor);
	else            lcd_put_char(11,3,wheel_string[2],color,bcColor);
  if(opt_pos==3)  lcd_put_s(9,5,"Back",4,bcColor,color);
	else          	lcd_put_s(9,5,"Back",4,color,bcColor);
}
void wheel_set(char *str,u8 color,u8 bcColor)
{  
	u8 opt_pos=0;
	u8 val;
	bool chFlag=false;
	Enter=false;
	PushEncoder
	PushEncoderMax
	EncoderMax(3);
	Encoder=opt_pos;
	 unsigned char wheel_string[3];
	wheel_string[0]=wheel_diameter/100+'0';
	wheel_string[1]=(wheel_diameter%100)/10+'0';
	wheel_string[2]=wheel_diameter%10+'0';
  lcd_clear(bcColor);
	lcd_put_s(0,0,str,22,color,bcColor);
	lcd_put_s(9,5,"Back",4,color,bcColor);
	whell_menu(wheel_string,opt_pos,chFlag,color,bcColor);
	while(true)
		{
			
	while(!Enter)
	{ 
		if(Encoder!=opt_pos)
		{
			opt_pos=Encoder;
			whell_menu(wheel_string,opt_pos,chFlag,color,bcColor);
		}
  }
	Enter=false;
	if(opt_pos==3){ wheel_length=(wheel_diameter*3.14)/1000;save_wheel_diameter();PopEncoderMax PopEncoder break;}
		else  
			{ 
        chFlag=true;whell_menu(wheel_string,opt_pos,chFlag,color,bcColor);
				EncoderMax(9);
				Encoder=val=wheel_string[opt_pos]-'0';
				while(!Enter)
	{
		if(val!=Encoder)
		{
			val=Encoder;
			wheel_string[opt_pos]=val+'0';
			whell_menu(wheel_string,opt_pos,chFlag,color,bcColor);
		}

  }
	Enter=false;
	EncoderMax(3);
  Encoder=opt_pos;
				
				chFlag=false;
	      whell_menu(wheel_string,opt_pos,chFlag,color,bcColor);
	wheel_diameter=100*(wheel_string[0]-'0')+10*(wheel_string[1]-'0')+(wheel_string[2]-'0');
      }
		
}
}
void gear_menu(u8 opt_pos,u8 color,u8 bcColor)
{ 
	
	if(opt_pos==0) 
	{
    lcd_put_s(10,4,"OK",2,bcColor,color);
		lcd_put_s(9,5,"Back",4,color,bcColor);
  }
	else 
		{
	lcd_put_s(10,4,"OK",2,color,bcColor);
	lcd_put_s(9,5,"Back",4,bcColor,color);
	  }
}
void write_gear_U(u8 gp,u8 color,u8 bcColor)
{
	u8 i;
	u16 volt;
	for(i=1;i<8;i++)
	{
	volt=gears[i];
	volt_string[0]=volt/1000+'0';
	volt%=1000;
	volt_string[1]='.';
	volt_string[2]=volt/100+'0';
	volt%=100;
	volt_string[3]=volt/10+'0';
	volt_string[4]=volt%10+'0';
		if(gp!=i)lcd_put_s(0,i,(char*)volt_string,5,color,bcColor);
		else lcd_put_s(0,i,(char*)volt_string,5,Lime,bcColor);
	}
}
void save_gears(u16 *gears)
{ 
	u8 i;
	union{
		u16 g16[9];
		u8  g8[18];
	}g168;
	for(i=0;i<9;i++)g168.g16[i]=gears[i];
	//for(i=0;i<18;i++) DS1302_WriteByte(i*2+208,g168.g8[i]);
}
void gear_set(char *str,u8 color,u8 bcColor)
{  
	u8 opt_pos=0;
	u8 cur_gear=1;
	Enter=false;
	PushEncoder
	PushEncoderMax
	EncoderMax(1);
	Encoder=0;
  lcd_clear(bcColor);
	lcd_put_s(0,0,str,22,color,bcColor);
	lcd_put_s(6,2,"Set ",4,color,bcColor);
	gear_menu(opt_pos,color,bcColor);
	lcd_put_char(10,2,'0'+cur_gear,Red,bcColor);
	lcd_put_s(12,2,"gear",4,color,bcColor);
	write_gear_U(cur_gear,color,bcColor);
	while(true){
	while(!Enter)
	{
    if(Encoder!=opt_pos) 
			{
				opt_pos=Encoder;
				gear_menu(opt_pos,color,bcColor);
      }

  }
	Enter=false;
	if(opt_pos==1){ break;}
		else if(opt_pos==0) 
			{ 
   gears[cur_gear]=package.geears;				
		if(cur_gear<7){ cur_gear++;
			write_gear_U(cur_gear,color,bcColor);
			lcd_put_char(10,2,'0'+cur_gear,Red,bcColor);
		}
		else {save_gears(gears);break;}
	 
				}
		
}
	PopEncoderMax
  PopEncoder
}
void save_colors(void)
{
	/*DS1302_WriteByte(248,ColorBar[fon_color_0_index]);
	DS1302_WriteByte(250,ColorBar[fon_color_1_index]);
	DS1302_WriteByte(252,menu_color);*/
}
void color_menu(u8 opt_pos,u8 color,u8 bcColor,bool selected,u8 color1_index,u8 color2_index,u8 color3_index)
{
	if(opt_pos==0)if(selected)lcd_put_s(0,2,"Color1: ",8,Red,color);
		else lcd_put_s(0,2,"Color1: ",8,bcColor,color);
	else lcd_put_s(0,2,"Color1: ",8,color,bcColor);
	 lcd_put_s(12,2,"|     |",7,color,Colors[color1_index]);
	if(opt_pos==1)if(selected)lcd_put_s(0,4,"Color2: ",8,Red,color); 
	  else lcd_put_s(0,3,"Color2: ",8,bcColor,color);
	else lcd_put_s(0,3,"Color2: ",8,color,bcColor);
	  lcd_put_s(12,3,"|     |",7,color,Colors[color2_index]);
	if(opt_pos==2)if(selected)lcd_put_s(0,4,"mColor: ",8,Red,color); 
	  else lcd_put_s(0,4,"mColor: ",8,bcColor,color);
	else lcd_put_s(0,4,"mColor: ",8,color,bcColor);
	  lcd_put_s(12,4,"|     |",7,color,Colors[color3_index]);
	if(opt_pos==3) lcd_put_s(9,6,"Back",4,bcColor,color);
	else lcd_put_s(9,6,"Back",4,color,bcColor);
}
void colors_set(char *str,u8 color,u8 bcColor)
{ 
	u8 opt_pos=0;
	u8 color1_index=0;
	u8 color2_index=0;
	u8 color3_index=0;
	u8 i;
	Enter=false;
	PushEncoder
	PushEncoderMax
	EncoderMax(3);
	Encoder=0;
	bool selected=false;
	for(i=0;i<sizeof(Colors);i++)if(Colors[i]==ColorBar[fon_color_1_index])break;
	color1_index=i;
	for(i=0;i<sizeof(Colors);i++)if(Colors[i]==ColorBar[fon_color_2_index])break;
	color2_index=i;
	for(i=0;i<sizeof(Colors);i++)if(Colors[i]==menu_color)break;
	color3_index=i;
	lcd_clear(bcColor);
	lcd_put_s(0,0,str,22,color,bcColor);
	color_menu(opt_pos,color,bcColor,selected,color1_index,color2_index,color3_index);
	while(true){
	while(!Enter)
	{
		if(Encoder!=opt_pos)
		{
			opt_pos=Encoder;
			color_menu(opt_pos,color,bcColor,selected,color1_index,color2_index,color3_index);
		}
 
  }
	Enter=false;
	if(opt_pos==3){
	ColorBar[fon_color_1_index]=Colors[color1_index];
	ColorBar[fon_color_2_index]=Colors[color2_index];
	menu_color=Colors[color3_index];
		save_colors();
	break;}
		else if(opt_pos==0) { 
			if(color1_index<sizeof(Colors)) color1_index++;
		else color1_index=0;}
		else if(opt_pos==1) {
		if(color2_index<sizeof(Colors))color2_index++;
		else color2_index=0;}
		else if(opt_pos==2) {
		if(color3_index<sizeof(Colors))color3_index++;
		else color3_index=0;}
		color_menu(opt_pos,color,bcColor,selected,color1_index,color2_index,color3_index);
				}
		PopEncoderMax
		PopEncoder	
}
void out_menu(u8 pos,u8 color,u8 bcColor)
{
	u8 opt_pos=0;
	char ds[9];
	u8 i;
	u32 dis32=distance;
	for(opt_pos=0;opt_pos<8;opt_pos++) 
	if (pos==opt_pos) lcd_put_s(0,opt_pos,options[opt_pos],22,bcColor,color);
	 else lcd_put_s(0,opt_pos,options[opt_pos],22,color,bcColor);    
	ds[0]='0'+dis32/100000;
	dis32=dis32 % 100000;
	ds[1]='0'+dis32/10000;
  dis32=dis32 % 10000;
	ds[2]='0'+dis32/1000;
	dis32=dis32 % 1000;
	ds[3]='1';
	ds[4]='0'+dis32/100;
	dis32=dis32 % 100;
	ds[5]='0'+dis32/10;
	dis32=dis32 % 10;
	ds[6]='0'+dis32;
	for(i=0;i<7;i++)if (ds[i]>'0') break;
	else ds[i]=' ';
	ds[3]='.';
	ds[7]=' ';
	ds[8]='m';
	lcd_put_s(6,7,(char*)ds,9,Lime,bcColor);
}
void OpenMenu(u8 color,u8 bcColor)
{ 
	u8 opt_pos=0;
	menu_status=true;
	Enter=false;
	PushEncoder
	Encoder=0;
	EncoderMax(options_max_index);
		while(true)
		{
	lcd_clear(bcColor);
	out_menu(opt_pos,color,bcColor);

	while(!Enter)
	{
		if(Encoder!=opt_pos)
    {		
    	opt_pos=Encoder;
	    out_menu(opt_pos,color,bcColor);
    }
  }
	
	if(opt_pos==0) {menu_status=false;PopEncoder break;}
		else if(opt_pos==1) I_set_time(options[opt_pos],color,bcColor);
	   else if(opt_pos==2) wheel_set(options[opt_pos],color,bcColor);
	    else if(opt_pos==3) gear_set(options[opt_pos],color,bcColor);
	     else if(opt_pos==4) colors_set(options[opt_pos],color,bcColor);
			 
		}
}
void InitAll(void)
{
	if(PowerDownState)
	{
		HAL_Init();
		SystemClock_UserConfig(SysFreq);
		MX_GPIO_Init();
		EncoderInit();
	  lcd_init();
	  SetBacklight(1);
	  //lcd_clear(Green);
		for(i=0;i<ColorBarSize;i++){ColorBar[i]=fon2_ColorBar[i];}
		lcd_draw_rle_picture_CT(0,0,175,131,fon2_image,sizeof(fon2_image),ColorBar);
		
	  NRF24L01_init();
	  rx_mode();
		HAL_ADC_Start(&hadc1);
		htim4.Instance->CNT=SelectMenu;
	  PowerDownState=false;		
		Enter=false;
	}
}
/* USER CODE END 0 */
void ReadPackage(u8 *buffer)
{
	u8 p=0;
	if(buffer[p++]==1)
	{
	package.pid=buffer[p++];
	u16u8.u8[1]=buffer[p++];
	u16u8.u8[0]=buffer[p++];
	package.Uin=u16u8.u16;
	u16u8.u8[1]=buffer[p++];
	u16u8.u8[0]=buffer[p++];
	package.Iin=u16u8.u16;
	u32u8.u8[3]=buffer[p++];
	u32u8.u8[2]=buffer[p++];
	u32u8.u8[1]=buffer[p++];
	u32u8.u8[0]=buffer[p++];
	package.rev_count_1=u32u8.u32;
	u32u8.u8[3]=buffer[p++];
	u32u8.u8[2]=buffer[p++];
	u32u8.u8[1]=buffer[p++];
	u32u8.u8[0]=buffer[p++];
	package.rev_count_2=u32u8.u32;
	u16u8.u8[1]=buffer[p++];
	u16u8.u8[0]=buffer[p++];
	package.rpm_1=u16u8.u16;
	u16u8.u8[1]=buffer[p++];
	u16u8.u8[0]=buffer[p++];
	package.rpm_2=u16u8.u16;
	package.on_off_channels=buffer[p++];
	u16u8.u8[1]=buffer[p++];
	u16u8.u8[0]=buffer[p++];
	package.temperature=u16u8.u16;
	u16u8.u8[1]=buffer[p++];
	u16u8.u8[0]=buffer[p++];
	package.geears=u16u8.u16;
}
}
int main(void)
{

  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
	SPI2->CR1|=SPI_CR1_SPE ;
	HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Encoder_Start_IT(&htim4,TIM_CHANNEL_1);
	
	InitAll();
	
	
	__enable_irq();
	Ready=true;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if(Enter)ButtonPressed();
		if(PowerDownState) PowerDown();
	//	if(PowerDownState) {lcd_power_off HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON,PWR_SLEEPENTRY_WFI);}
		

			if(PackageReceived)
	{
		speed=(float)package.rpm_2*(float)(wheel_diameter)*0.113097/1000.0;
		WriteSpeedOnly(speed);
		//rpm=(float)package.rpm_1*0.6;
		//WriteRpm(rpm);
	}
	  
    WriteGear();
    distance=(float)package.rev_count_2*(float)(wheel_diameter)/1000.0*3.141592653;
		WriteDistance((u32)distance);
		WriteTime();
		WriteTemp();
	  WriteOuterBattery();
		WriteVoltage();
		
		WriteMenu();
		if(package.on_off_channels & 0x10) 
		{
			package.on_off_channels &=0xEF;
			ChangeBacklight();
		}
		
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  HAL_ADC_Init(&hadc1);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* RTC init function */
void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;

    /**Initialize RTC and set the Time and Date 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  HAL_RTC_Init(&hrtc);

  sTime.Hours = 0x1;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);

  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);

}

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi2);

}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

}

/* TIM4 init function */
void MX_TIM4_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 2;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 2;
  HAL_TIM_Encoder_Init(&htim4, &sConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

}
void EncoderInit(void)
{
	

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 2;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 2;
  HAL_TIM_Encoder_Init(&htim4, &sConfig);
/*
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);*/
}
/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF24L01_POWER_GPIO_Port, NRF24L01_POWER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, backlight_1_Pin|backlight_2_Pin|LCD_POWER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NRF24L01_CSN_Pin|NRF24L01_CE_Pin|LCD_RS_Pin|LCD_RESET_Pin 
                          |LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : NRF24L01_POWER_Pin */
  GPIO_InitStruct.Pin = NRF24L01_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRF24L01_POWER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : backlight_1_Pin backlight_2_Pin LCD_POWER_Pin */
  GPIO_InitStruct.Pin = backlight_1_Pin|backlight_2_Pin|LCD_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA9 PA10 PA11 
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF24L01_IRQ_Pin */
  GPIO_InitStruct.Pin = NRF24L01_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(NRF24L01_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF24L01_CSN_Pin NRF24L01_CE_Pin LCD_RS_Pin LCD_RESET_Pin 
                           LCD_CS_Pin */
  GPIO_InitStruct.Pin = NRF24L01_CSN_Pin|NRF24L01_CE_Pin|LCD_RS_Pin|LCD_RESET_Pin 
                          |LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB3 PB4 PB5 
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER_Button_Pin */
  GPIO_InitStruct.Pin = ENCODER_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENCODER_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void SystemClock_UserConfig(uint8_t Freq)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
	
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
	
	switch(Freq)
	{
		case 4:
		  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
	  break;
		case 8:
			
		break;
		case 24:
			RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
      RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
      RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
		  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
		  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		break;
		default: break;
	}
	

  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
 
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	switch(Freq)
	{
		case 4:
		  htim1.Init.Prescaler = 3999;
	  break;
		case 8:
			htim1.Init.Prescaler = 7999;
		break;
		case 24:
		  htim1.Init.Prescaler = 23999;
		break;
		default: break;
	}
	HAL_TIM_Base_Init(&htim1);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
//*****************************************************************************
//(((((((((((((((((((((((((    NRF24L01    ))))))))))))))))))))))))))))))))))))
//*****************************************************************************
unsigned char read_irq()
{
	unsigned char stat;
 csn_0
 spi(0x07);
 stat=spi(0);
 csn_1
 return stat; 
}
void clr_irq()
{
 unsigned char stat;
 stat=read_irq();
 csn_0
 spi(0x27);
 spi(stat); 
 csn_1
}
void NRF24L01_IRQ(void)
{
	state=read_irq();
	if(state&0x40)
	{
		i=read_rx(buffer);
		ReadPackage(buffer);
		PackageReceived=true;
		clr_irq();
		TimeOutTimer=0;
	}
	if(state&0x10)
	{
		if(TransmitRepeats==3)
		{
		ConnectError=true;
		TransmitRepeats=0;
		clr_irq();
		NRF24L01_power_down();
		}
		else
		{			
			TransmitRepeats++;
			csn_0
      spi(FLUSH_TX);
      csn_1
		}
	}
	if(state & 0x1)
	{
		csn_0
		spi(FLUSH_TX);
		csn_1
	}
	if(state & 0x20)
	{
		NRF24L01_Transmit=false;
		TransmitRepeats=0;
		clr_irq();
	//	if(NRF24L01_POWER) rx_mode();
	}
  clr_irq();
}
void tx_mode(void)
{ 
	__disable_irq();
 ce_0
 csn_0
 spi(0x20);
 spi(0xE); 
 csn_1
 ce_1
 soft_delay_ms(1);  
	NRF24L01_POWER=true;
	NRF24L01_TX=true;
	NRF24L01_RX=false;
	__enable_irq();
}
void NRF24L01_power_down(void)
{	
	__disable_irq();
 ce_0
 csn_0
 spi(0x20);
 spi(0x8);
 csn_1
	NRF24L01_POWER=false;
	NRF24L01_TX=false;
	NRF24L01_RX=false;
	__enable_irq();
}
void rx_mode(void)
{ 
	__disable_irq();
 ce_1
 csn_0
 spi(0x20);
 spi(0xF);
 csn_1
 soft_delay_ms(2); 
	NRF24L01_POWER=true;
	NRF24L01_TX=false;
	NRF24L01_RX=true;
__enable_irq();	
}

void send_msg(char *msg)
{
	uint8_t a;
	if(NRF24L01_POWER)
{
	__disable_irq();
while(read_irq() & 0x1){}
	clr_irq();
 csn_0
 spi(0xE1);
 csn_1
 delay_us(10);
 csn_0
 spi(0xA0);
 for (a=0;a<strlen(msg);a++)spi(msg[a]); 
 spi('\n');
 csn_1 
	NRF24L01_Transmit=true;
__enable_irq();
}
}
void send_data(uint8_t *data,uint8_t size)
{
	uint8_t a=0,c;
if(NRF24L01_POWER)
{
	__disable_irq();
	length=size;
while(a<size)
{ 
while(read_irq() & 0x1){}
	clr_irq();
 csn_0
 spi(0xE1);
 csn_1
 delay_us(10);
 csn_0
 spi(0xA0);
 for (c=0;(a<size) && (c<31);a++,c++)spi(data[a]); 
 csn_1
	NRF24L01_Transmit=true;
 __enable_irq(); 
}
}
}
uint8_t read_rx(uint8_t *data)
{
	unsigned char a;
 unsigned char n;
	__disable_irq();
 csn_0
 spi(R_RX_PL_WID);
 n=spi(0);
 csn_1  
 csn_0
 spi(0x61);
 for (a=0;a<n;a++) data[a]=spi(0);
 csn_1
 csn_0
 spi(0xE2);
 csn_1
 buffer[a]='\0';
 clr_irq(); 
	length=n;
	__enable_irq();
return n;
}
/*void SetID(uint8_t *data)
{
char i;
__disable_irq();
	soft_delay_ms(30);
	clr_irq();
csn_0
spi(0x2A);   
for(i=0;i<5;i++) spi(data[i]);
csn_1
csn_0
spi(0x30);   
for(i=0;i<5;i++) spi(data[i]);
csn_1
soft_delay_ms(30);
	clr_irq();
__enable_irq();
}*/
void NRF24L01_init(void)
{
	__disable_irq();
	
NRF24L01_Power_on
soft_delay_ms(10);
	
clr_irq();
csn_1
ce_0	
	
/*csn_0
spi(0x50);
spi(0x73);   //Activate
csn_1
	
csn_0
spi(0x20);      
spi(0x0F);  // Enable CRC | 2-bytes | POWER UP | PRX
csn_1 
	
csn_0
spi(0x24);
spi(0x23);  // 750us 3 Re-Transmit
csn_1
	
csn_0
spi(0x3D);
spi(0x06);   //Enables Dynamic Payload Length with ACK
csn_1
	
csn_0
spi(0x3C);   //Enable dyn. payload length for all data pipes
spi(0x3F);
csn_1

csn_0
spi(0x26);
spi(0x0E);    // 2Mbps |  0dBm | LNA gain disable
csn_1

soft_delay_ms(10);*/
csn_0
spi(0x50);
spi(0x73);   //Activate
csn_1

csn_0
spi(0x3D);
spi(0x06);   // 0x04-Enables Dynamic Payload Length  | 0x2  - Enables Payload with ACK
csn_1           
csn_0
spi(0x3C);   //Enable dyn. payload length for all data pipes
spi(0x3F);
csn_1
delay_us(10);
csn_0
spi(0x26);    // Disable LNA Gain  
spi(0x0E);
csn_1
csn_0
spi(0x20);
spi(0xE);   //Config(0x0E) - CRC-2bytes | EN_CRC | PWR_UP
csn_1
csn_0
spi(0x24);
spi(0x13);   //Setup_Retr   500us 3-Re-Transmit
csn_1

csn_0
spi(FLUSH_RX);
csn_1

csn_0
spi(FLUSH_TX);
csn_1

clr_irq();
SetID(NRF24L01_ID);
NRF24L01_POWER=true;

__enable_irq();
}
void SetID(uint8_t *data)
{
  SetRxAddr(data);
  SetTxAddr(data);
}
void SetRxAddr(uint8_t *data)
{
	char i;
	__disable_irq();
  csn_0
  spi(0x2A);   
  for(i=0;i<5;i++) spi(data[i]);
  csn_1
  __enable_irq();
}
void SetTxAddr(uint8_t *data)
{
	char i;
	__disable_irq();
  csn_0
  spi(0x30);   
  for(i=0;i<5;i++) spi(data[i]);
  csn_1
  __enable_irq();
}
void DisplayMainWindow(struct _MainValues *main_values)
{
	
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
