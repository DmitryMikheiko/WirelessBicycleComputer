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
//#include "init.c"
#include "string.h"
#include "math.h"
#include "stdbool.h"
#include "DS18B20.c"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define delay HAL_Delay
// ADC constants===============================================================
#define Vref 3.3
#define ADC_Koef Vref/4095.0
#define CH0_Koef 2.0*ADC_Koef           //gears
#define CH1_Koef 1.0*ADC_Koef           //control
#define CH4_Koef 1.090*5.034*ADC_Koef   //Step UP 1
#define CH5_Koef 1.038*5.05*ADC_Koef    //Step UP 2
#define CH6_Koef 1.22*ADC_Koef/Koy1/R1  //I LED
#define CH7_Koef 1.1*2.0*ADC_Koef       //U LED
#define CH8_Koef 1.015*2.0*ADC_Koef           //U POWER
#define CH9_Koef 1.068*1.0*ADC_Koef/Koy2/R2   //I POWER
#define Koy1 10.1
#define Koy2 10.5
#define R1 0.1
#define R2 0.09556
float CHx_Koef[]={CH0_Koef,CH1_Koef,CH4_Koef,CH5_Koef,CH6_Koef,CH7_Koef,CH8_Koef,CH9_Koef};
#define ADC_Gears    0
#define ADC_Control  1
#define ADC_Stab1U   2
#define ADC_Stab2U   3
#define ADC_Stab3I   4
#define ADC_Stab3U   5
#define ADC_InU      6
#define ADC_InI      7
//=============================================================================
#define green_led_pin GPIO_PIN_9
#define green_led_port GPIOB
#define green_led_on  HAL_GPIO_WritePin(green_led_port,green_led_pin,GPIO_PIN_SET);
#define green_led_off HAL_GPIO_WritePin(green_led_port,green_led_pin,GPIO_PIN_RESET);
uint8_t green_led_state=0;

#define red_light_pin GPIO_PIN_9
#define red_light_port GPIOA
#define red_light_on  HAL_GPIO_WritePin(red_light_port,red_light_pin,GPIO_PIN_SET);
#define red_light_off HAL_GPIO_WritePin(red_light_port,red_light_pin,GPIO_PIN_RESET);
uint8_t red_light_state=0;

#define red_led_pin GPIO_PIN_11
#define red_led_port GPIOA
#define red_led_on  HAL_GPIO_WritePin(red_led_port,red_led_pin,GPIO_PIN_SET);
#define red_led_off HAL_GPIO_WritePin(red_led_port,red_led_pin,GPIO_PIN_RESET);
uint8_t red_led_state=0;

#define ce GPIO_PIN_12
#define csn GPIO_PIN_15
#define csn_1 HAL_GPIO_WritePin(GPIOA,csn,GPIO_PIN_SET); 
#define csn_0 HAL_GPIO_WritePin(GPIOA,csn,GPIO_PIN_RESET);
#define ce_1 HAL_GPIO_WritePin(GPIOA,ce,GPIO_PIN_SET);
#define ce_0 HAL_GPIO_WritePin(GPIOA,ce,GPIO_PIN_RESET);
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

bool Core2C_flag=false;

bool LcdChangeBacklight=false;

#define uart_buffer_size 30
uint8_t uart_buffer[uart_buffer_size];
uint8_t uart_buffer_pointer=0;

uint8_t adc_channel = 0;
uint16_t adc_value[8]={0,0,0,0,0,0,0,0};

#define MaxButtonCount 10
#define ButtonsADCRange 100
uint16_t buttons[MaxButtonCount]={1965,2170,2383,2703,3060};
#define button_count 5
uint8_t button_action[button_count]={1,2,3,5,4};
uint8_t button_pressed=false;

#define gear_count 7
uint16_t gears[gear_count];
uint16_t gear_ranges[gear_count+2];
uint8_t Gear=0;
uint16_t GearU=0;

float Uglobal;
float Iglobal;
HAL_StatusTypeDef s,s2;


bool Ready=false;
//RTC_TimeTypeDef Time;
//RTC_DateTypeDef Date;

#define HEX2BCD(v)	((v) % 10 + (v) / 10 * 16)
#define BCD2HEX(v)	((v) % 16 + (v) / 16 * 10)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_RTC_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
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
void NRF24L01_power_down(void);

float GetU(char ch);
void SaveConfig(void);
void LoadConfig(void);
void ChangeConfig(uint16_t I,uint16_t U1,uint16_t U2);
void ProcessRequest(uint8_t *data);
//****************************************
#define StepUp1_max_value 1000
#define StepUp2_max_value 1000
#define StepUp3_max_value   1000
#define StepDown3_max_value 1000

struct StabU
{
	uint16_t U;
	uint16_t Unow;
	uint16_t StepUpMaxValue;
	uint16_t StepUpValue;
	uint8_t ADC_CH;
	bool run;
	bool error;
	bool state;
  TIM_HandleTypeDef *TIM;
	uint32_t TIM_CH;
	__IO uint32_t *TIM_Value;
}StabU1,StabU2;

struct StabI
{
	uint16_t Umax;
	uint16_t Unow;
	uint16_t I;
	uint16_t Inow;
	uint16_t StepUpMaxValue;
	uint16_t StepUpValue;
	uint16_t StepDownValue;
	uint16_t StepDownMaxValue;
	uint8_t ADC_I_CH;
	uint8_t ADC_U_CH;
	bool run;
	bool error;
	bool state;
	bool StepUP_state;
	bool StepDown_state;
  TIM_HandleTypeDef *TIM_UP;
	TIM_HandleTypeDef *TIM_DOWN;
	uint32_t TIM_UP_CH;
	uint32_t TIM_DOWN_CH;
	__IO uint32_t *TIM_UP_Value;
	__IO uint32_t *TIM_DOWN_Value;
}StabI1;
struct Power
{
	uint16_t I;
	uint16_t U;
	uint16_t Imax;
	uint16_t Umin;
	uint16_t Uwarning;
	uint8_t warning;
	uint8_t run;
}power;
void StepUpState (struct StabU *);
void StepUpDownState (struct StabI *);

void StepUpValue(struct StabU *,uint16_t);

void StepUpValue_inc(struct StabU *);
void StepUpValue_dec(struct StabU *);

void StepUpDownValue_inc(struct StabI *stab);
void StepUpDownValue_dec(struct StabI *stab);

void Core0(void);
void Core1(void);
void Core2A(void);
void Core2B(void);
void Core2C(void);

void GPS_Procc(void);

void StabU_Procc(struct StabU *stab);
void StabI_Procc(struct StabI *stab);

uint8_t CheckButtons(void);
uint8_t CheckGears(void);
//******** Core0 temps ****************
bool StepUp1_state=false;
bool StepUp2_state=false;
bool StepUp3_state=false;
bool StepDown3_state=false;
bool UIStab_state=false;

bool StepUp1_state_now=false;
bool StepUp2_state_now=false;
bool StepUp3_state_now=false;
bool StepDown3_state_now=false;
bool UIStab_state_now=false;

uint16_t Stab1Value=0;
uint16_t Stab2Value=0;
uint16_t Stab3UpValue=0;
uint16_t Stab3DownValue=0;

uint16_t Stab1_U;
uint16_t Stab2_U;
uint16_t Stab3_U;
uint16_t Stab3_I;

float I;

uint8_t SYS_FCLK;

float rpm_1=0.0;
float rpm_2=0.0;
uint32_t rev_count_1;
uint32_t rev_count_2;

bool Saved=false;

float Temperature;
#define GPS_buffer_size 16
uint8_t GPS_buffer[GPS_buffer_size];
struct 
{
	float latitude;
	char  latitude_NS;
	float longitude;
	char  longitude_WE;
	float time;
	float speed;
}GPS_data;

union U32U16
{
	uint32_t u32;
  uint16_t u16[2];
} u32u16;
union U32U8
{
	uint32_t u32;
  uint8_t  u8[4];
} u32u8;
union U16U8
{
	uint16_t u16;
	uint8_t u8[2];
}u16u8;
union U8Bits
{
	uint8_t u8;
	struct
  {
    unsigned char bit0 : 1;
    unsigned char bit1 : 1;
    unsigned char bit2 : 1;
    unsigned char bit3 : 1;
    unsigned char bit4 : 1;
    unsigned char bit5 : 1;
    unsigned char bit6 : 1;
    unsigned char bit7 : 1;
  }bits;
} u8bits;

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void delay_us(uint16_t time_)
{
	uint32_t time;
	time=time_;
	time*=SYS_FCLK;
	time/=3;
  while(time--);
}
void delay_ms(uint16_t ms)
{
	
	/*htim7.Instance->ARR=ms;
	htim7.Instance->CNT=0;
	htim7.Instance->CR1|=TIM_CR1_CEN;
	while(!(htim7.Instance->SR & TIM_SR_UIF) );
	htim7.Instance->SR &= ~TIM_SR_UIF;*/
	  /*htim7.Instance->ARR = ms;                      
    htim7.Instance->EGR = TIM_EGR_UG;                 
    htim7.Instance->CR1 = TIM_CR1_CEN|TIM_CR1_OPM;    
    while ((htim7.Instance->CR1 & TIM_CR1_CEN)!=0);  */
	while(ms--) delay_us(1000);
}
uint8_t spi(uint8_t address)
{
	HAL_SPI_TransmitReceive(&hspi1, &address, &data, sizeof(data), 0x1000);
	return data;
}
uint16_t GetImA(char ch)
{
	//uint16_t I;
	I=adc_value[ch];
	I*=(CHx_Koef[ch] * 1000.0);
	return I;
}
uint16_t GetR1dU(void)
{
  float dU;
	dU=GetImA(ADC_InI);
	dU*=R1;
	Uglobal=(uint16_t)dU;
	return (uint16_t)dU;
}
uint16_t GetUmv(char ch)
{
	uint16_t U;
	U=adc_value[ch];
	U*=(CHx_Koef[ch] * 1000.0);
	if(ch!=ADC_InU)
	{
		U-=GetR1dU();
	}
	return U;
}
float GetU(char ch)
{
	float U;
	U=adc_value[ch];
	U*=CHx_Koef[ch];
	return U;
}
void StabU_Procc(struct StabU *stab)
{
	uint16_t U,Unow;
	if(stab->run!=stab->state)
	{
		StepUpState(stab);
	}
	else if(stab->run)
	{ 
		U=stab->U;
		Unow=GetUmv(stab->ADC_CH);
		stab->Unow=Unow;
		if(Unow>U)
		{
			StepUpValue_dec(stab);
		}
		else if(Unow<U)
		{
			StepUpValue_inc(stab);
		}
	}
}
void StabI_Procc(struct StabI *stab)
{
	uint16_t I,Inow,Unow;
	if(stab->run!=stab->state)
	{
		StepUpDownState(stab);
	}
	else if(stab->run)
	{ 
		I=stab->I;
		Inow=GetImA(stab->ADC_I_CH);
		Unow=GetUmv(stab->ADC_U_CH);
		stab->Unow=Unow;
		stab->Inow=Inow;
		if(Inow>I )//|| Unow>stab->Umax
		{
			StepUpDownValue_dec(stab);
		}
		else if(Inow<I)
		{
			if((*(stab->TIM_UP_Value))==0) StepUpDownValue_inc(stab);
			else if(Unow<stab->Umax)
			{
				StepUpDownValue_inc(stab);
			}
			else 
			{
				StepUpDownValue_dec(stab);
				stab->error=true;
			}
		}		
	}
}
uint8_t RedLightState(void)
{
	return red_light_state;
}
void RedLightSetState(uint8_t state)
{
	red_light_state=state;
	if(state) red_light_on
		else red_light_off
}
void RedLightInvState(void)
{
RedLightSetState(!RedLightState());
}
uint8_t GreenLedState(void)
{
	return green_led_state;
}
void GreenLedSetState(uint8_t state)
{
	green_led_state=state;
	if(state) green_led_on
		else green_led_off
}
void GreenLedInvState(void)
{
GreenLedSetState(!GreenLedState());
}
uint8_t RedLedState(void)
{
	return red_led_state;
}
void RedLedSetState(uint8_t state)
{
	red_led_state=state;
	if(state) red_led_on
	else red_led_off
}
void RedLedInvState(void)
{
RedLedSetState(!RedLedState());
}
//***************************************************
void Core0(void)
{
	StabU_Procc(&StabU1);
	StabU_Procc(&StabU2);
	StabI_Procc(&StabI1);      
  //delay_us(10);
}
//***************************************************
void Core1(void)
{
	uint16_t temp;
	if(Ready)
	{
	if(DS18B20_Init())
	{
		if(DS_State==DS_Run)
			{
	      temp=DS18B20_ReadTemp();
	      Temperature=DS18B20_GetFloatTemp(temp);
	    }
	DS18B20_Start();
	}
	if(power.warning)
	{
		GreenLedSetState(false);
		RedLedInvState();
	}
	else
	{
		if(NRF24L01_POWER)GreenLedInvState();
		else GreenLedSetState(true);
		RedLedSetState(false);
	}
 	
	SendInfo();
}
}
//**************************************************
void Core2A(void)
{
	uint8_t number;
	if(Ready)
	{
	number=CheckButtons();
	if(number)
	{
	if(!button_pressed)
 	 {
		button_pressed=true;
	  switch(button_action[number-1])
	  	{
			case 1: 
			  StabI1.run=!StabI1.run;	
      break;	
			case 2:
				RedLightInvState();
      break;	
			case 3: 
	   	  StabU1.run=!StabU1.run;
      break;
			case 4:
			  StabU2.run=!StabU2.run;
			break;
			case 5:
				LcdChangeBacklight=true;
			break;
      default : break;			
		  }			
	  }	
   } 
	else button_pressed=false;
 }
}
//---------------------------------------------------
void Core2B(void)
{
	if(Ready)
	{
	GearU=GetUmv(ADC_Gears);
			delay_us(10);
	GearU+=GetUmv(ADC_Gears);
			delay_us(10);
GearU+=GetUmv(ADC_Gears);		
		GearU/=3;
	power.I=GetImA(ADC_InI);
	delay_us(10);
	power.I+=GetImA(ADC_InI);
	delay_us(10);
	power.I+=GetImA(ADC_InI);
	power.I/=3;
	
	power.U=GetUmv(ADC_InU);
	delay_us(10);
	power.U+=GetUmv(ADC_InU);
	delay_us(10);
	power.U+=GetUmv(ADC_InU);
	power.U/=3;
	
	if(power.U<power.Umin) power.run=false;
	if(power.I>power.Imax) power.run=false;
	if(power.U<=power.Uwarning) 
	{
		power.warning=true;
	}
   else 
	 {
		 power.warning=false;
	 }		
  if(!power.run)
   {
		 StabI1.run=false;
		 StepUpDownState(&StabI1);
		 StabU1.run=false;
		 StepUpState(&StabU1);
		 StabU2.run=false;
		 StepUpState(&StabU2);
		 RedLightSetState(false);
     green_led_off
		 red_led_on
		 NRF24L01_power_down();
		 HAL_PWR_EnterSTANDBYMode();
		 while(true);
   }	
 }
}
void Core2C(void)
{
	if(Core2C_flag)
	{
		Core2C_flag=false;
		if(NRF24L01_POWER) tx_mode();
	}
	else if(NRF24L01_RX) Core2C_flag=true;
}
//*************************************************
/* USER CODE END 0 */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_RTC_Init();

  /* USER CODE BEGIN 2 */
	SYS_FCLK=HAL_RCC_GetHCLKFreq()/1e+6;

	HAL_TIM_Base_Start_IT(&htim1);          // timers interrupt
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim6);
	
	power.Imax=2500;
	power.Umin=2800;
	power.Uwarning=3000;
	power.run=true;	
	delay(500);
	clr_irq();
	NRF24L01_init();
	tx_mode();
  
	__enable_irq();
	
	//HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc_value,8);
	hdma_adc1.Instance->CCR^=DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE;
	//HAL_UART_Receive_DMA(&huart1,GPS_buffer,GPS_buffer_size);
	
  // Stabs Init*********************
  StabU1.ADC_CH=ADC_Stab1U;
	StabU1.state=false;
	StabU1.StepUpMaxValue=300;
	StabU1.StepUpValue=0;
	StabU1.TIM=&htim15;
	StabU1.TIM_CH=TIM_CHANNEL_1;
	StabU1.TIM_Value=&(TIM15->CCR1);
	StabU1.U=10000;
	StabU1.run=false;
	
  StabU2.ADC_CH=ADC_Stab2U;
	StabU2.state=false;
	StabU2.StepUpMaxValue=250;
	StabU2.StepUpValue=0;
	StabU2.TIM=&htim15;
	StabU2.TIM_CH=TIM_CHANNEL_2;
	StabU2.TIM_Value=&(TIM15->CCR2);
	StabU2.U=6425;
	StabU2.run=false;
	
	StabI1.ADC_I_CH=ADC_Stab3I;
	StabI1.ADC_U_CH=ADC_Stab3U;
	StabI1.I=300;
	StabI1.Umax=6000;
	StabI1.StepDownMaxValue=400;
	StabI1.StepUpMaxValue=250;
	StabI1.TIM_DOWN=&htim2;
	StabI1.TIM_DOWN_CH=TIM_CHANNEL_3;
	StabI1.TIM_DOWN_Value=&(TIM2->CCR3);
	StabI1.TIM_UP=&htim2;
	StabI1.TIM_UP_CH=TIM_CHANNEL_4;
	StabI1.TIM_UP_Value=&(TIM2->CCR4);
	StabI1.state=false;
  StabI1.run=false;
	Ready=true;
	LoadConfig();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
     Core0();
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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV4;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  HAL_ADC_Init(&hadc1);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 4;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 6;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 7;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 8;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);

}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 23999;
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

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 400;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 28000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

/* TIM4 init function */
void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 2399;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 32000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

}

/* TIM6 init function */
void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 23999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 50;
  HAL_TIM_Base_Init(&htim6);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);

}

/* TIM7 init function */
void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 23999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 0;
  HAL_TIM_Base_Init(&htim7);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig);

}

/* TIM15 init function */
void MX_TIM15_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 400;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  HAL_TIM_PWM_Init(&htim15);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_MspPostInit(&htim15);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB13 PB14 
                           PB15 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 PA11 PA12 
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
unsigned char read_irq()
{unsigned char stat;
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
	__disable_irq();
//state=read_irq();
	csn_0
  spi(0x07);
  state=spi(0);
  csn_1
	if(state&0x40)
	{
		i=read_rx(buffer);
		clr_irq();
		csn_0
    spi(0x27);
    spi(state); 
    csn_1
		ProcessRequest(buffer);
	}
	if(state&0x10)
	{
		csn_0
    spi(0x27);
    spi(state); 
    csn_1
	/*	if(TransmitRepeats==5)
		{
		ConnectError=true;
		TransmitRepeats=0;
		NRF24L01_power_down();
		}
		else*/
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
		csn_0
    spi(0x27);
    spi(state); 
    csn_1
		if(NRF24L01_POWER) rx_mode();
	}
    csn_0
    spi(0x27);
    spi(state); 
    csn_1
	__enable_irq();
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
 delay_ms(1);  
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
 delay_ms(2); 
	NRF24L01_POWER=true;
	NRF24L01_TX=false;
	NRF24L01_RX=true;
__enable_irq();	
}

void send_msg(char *msg)
{ uint8_t a;
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
{ uint8_t a=0,c;
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
{unsigned char a;
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
void SetID(uint8_t *data)
{
char i;
__disable_irq();
	delay_ms(30);
	clr_irq();
csn_0
spi(0x2A);   
for(i=0;i<5;i++) spi(data[i]);
csn_1
csn_0
spi(0x30);   
for(i=0;i<5;i++) spi(data[i]);
csn_1
delay_ms(30);
	clr_irq();
__enable_irq();
}
void NRF24L01_init(void)
{
	__disable_irq();
	
csn_1
ce_0	
	
csn_0
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

delay_ms(10);

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
}/*
void uart_received(void)
{	
	
	while(huart1.Instance->SR & USART_SR_RXNE)
	{
	uart_buffer[uart_buffer_pointer++]=huart1.Instance->DR;	
	}
	
	//uart_buffer[0]+=uart_buffer_pointer;
//	HAL_UART_Transmit(&huart1, buffer, i, 1000);
  if(uart_buffer_pointer==uart_buffer_size)	
	{
	send_data(uart_buffer,uart_buffer_pointer);
	uart_buffer_pointer=0;
  }
}*/

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//***************************************************************
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void StepUpValue_inc(struct StabU *stab)
{
	if((*(stab->TIM_Value))<(stab->StepUpMaxValue))
	{
    (*stab->TIM_Value)++;
	}
 else stab->error=true;
}

void StepUpValue_dec(struct StabU *stab)
{
	if((*(stab->TIM_Value))>0)
	{
		(*stab->TIM_Value)--;
		
	}
	else stab->error=true;
}
void StepUpDownValue_inc(struct StabI *stab)
{
	if((*(stab->TIM_DOWN_Value))<(stab->StepDownMaxValue))
	{
		 (*stab->TIM_DOWN_Value)++;
	}
	else if ((*(stab->TIM_UP_Value))<(stab->StepUpMaxValue))
	{
		 (*stab->TIM_UP_Value)++;
	}
	else stab->error=true;
}
void StepUpDownValue_dec(struct StabI *stab)
{
	if((*(stab->TIM_UP_Value))>0)
	{
		 (*stab->TIM_UP_Value)--;
	}
	else if ((*(stab->TIM_DOWN_Value))>0)
	{
		 (*stab->TIM_DOWN_Value)--;
	}
	else stab->error=true;
}

void StepUpState(struct StabU *stab)
{
	if(stab->run)
	{
	HAL_TIM_PWM_Start(stab->TIM,stab->TIM_CH);
	stab->error=false;
	}
	else 
  {
	HAL_TIM_PWM_Stop(stab->TIM,stab->TIM_CH);
	*stab->TIM_Value=0;		
	}
	stab->state=stab->run;
}
void StepUpDownState(struct StabI *stab)
{
	if(stab->run)
	{
  HAL_TIM_PWM_Start(stab->TIM_DOWN,stab->TIM_DOWN_CH);
	HAL_TIM_PWM_Start(stab->TIM_UP,stab->TIM_UP_CH);
	stab->error=false;
	}
	else 
  {
	HAL_TIM_PWM_Stop(stab->TIM_DOWN,stab->TIM_DOWN_CH);
	HAL_TIM_PWM_Stop(stab->TIM_UP,stab->TIM_UP_CH);
	*stab->TIM_DOWN_Value=0;
  *stab->TIM_UP_Value=0;		
	}
	stab->state=stab->run;
}

void StepUpValue(struct StabU *stab,uint16_t value)
{
	(*(stab->TIM_Value))=value;
	//TIM15->CCR1=value;
	//Stab1Value=value;
}

uint8_t CheckButtons(void)  //return number of presses button 1..button_count
{
	uint8_t number;
	for(number=0;number<button_count;number++)
	{
		if(((buttons[number]-ButtonsADCRange)<adc_value[ADC_Control]) && (adc_value[ADC_Control]<(buttons[number]+ButtonsADCRange)))
		{			
			break;
		}
	}
	if(number==button_count) number=0;
	else number++;
	return number;
}
uint8_t CheckGears(void)
{
	uint8_t gear;
	uint16_t U;
	U=adc_value[ADC_Gears];
	for(gear=0;gear<(gear_count+1);gear++)
	{
		if((U>gear_ranges[gear]) && (U<gear_ranges[gear+1]))
		{
			
			break;
		}
	}
	if(gear==(gear_count+1)) gear=0;
	else gear++;
	Gear=gear;
	return gear;
}
void GPS_Procc(void)
{
	
}

void SendInfo(void)
{
	uint8_t p=0;
	info_buffer[p++]=0x1;
	info_buffer[p++]=0x1;
	
	u16u8.u16=power.U;
	info_buffer[p++]=u16u8.u8[1];
	info_buffer[p++]=u16u8.u8[0];
	
	u16u8.u16=power.I;
	info_buffer[p++]=u16u8.u8[1];
	info_buffer[p++]=u16u8.u8[0];
	
	u32u8.u32=rev_count_1;
	info_buffer[p++]=u32u8.u8[3];
	info_buffer[p++]=u32u8.u8[2];
	info_buffer[p++]=u32u8.u8[1];
	info_buffer[p++]=u32u8.u8[0];
	
	u32u8.u32=rev_count_2;
	info_buffer[p++]=u32u8.u8[3];
	info_buffer[p++]=u32u8.u8[2];
	info_buffer[p++]=u32u8.u8[1];
	info_buffer[p++]=u32u8.u8[0];
	
	u16u8.u16=(rpm_1 *100.0);
	info_buffer[p++]=u16u8.u8[1];
	info_buffer[p++]=u16u8.u8[0];
	
	u16u8.u16=(rpm_2 *100.0);
	info_buffer[p++]=u16u8.u8[1];
	info_buffer[p++]=u16u8.u8[0];
	
	u8bits.u8=0;
	u8bits.bits.bit0=StabI1.state;
	u8bits.bits.bit1=StabU1.state;
	u8bits.bits.bit2=StabU2.state;
	u8bits.bits.bit3=red_light_state;
	u8bits.bits.bit4=LcdChangeBacklight;
	LcdChangeBacklight=false;
	info_buffer[p++]=u8bits.u8;
	
	if(Temperature>0)
	{
		u16u8.u16=(uint16_t)(Temperature*10.0);
	}
	else 
	{
		u16u8.u16=(uint16_t)(Temperature*10.0)+0x8000;
	}
	info_buffer[p++]=u16u8.u8[1];
	info_buffer[p++]=u16u8.u8[0];
	
	u16u8.u16=GearU;
	info_buffer[p++]=u16u8.u8[1];
	info_buffer[p++]=u16u8.u8[0];
  
	send_data(info_buffer,p);
	/*float f;
	uint16_t w;*/
	/*f=power.I;
	send_msg("\r\npower.I:");
	f/=1000;
	send_float(f,3);
	f=power.U;
	f/=1000;
	send_msg("\r\npower.U:");
	send_float(f,3);
	
	w=StabI1.state;
	send_msg("\r\nStabI1.state:");
	send_u32(w);
	f=StabI1.Inow;
	send_msg("\r\nStabI1.Inow:");
	send_float(f,3);
	f=StabI1.Unow;
	send_msg("\r\nStabI1.Unow:");
	send_float(f,3);
	
	w=StabU1.state;
	send_msg("\r\nStabU1.state:");
	send_u32(w);
	f=StabU1.Unow;
	send_msg("\r\nStabU1.Unow:");
	send_float(f,3);
	f=StabU1.U;
	send_msg("\r\nStabU1.U:");
	send_float(f,3);
	
	w=StabU2.state;
	send_msg("\r\nStabU2.state:");
	send_u32(w);
	f=StabU2.Unow;
	send_msg("\r\nStabU2.Unow:");
	send_float(f,3);
	f=StabU2.U;
	send_msg("\r\nStabU2.U:");
	send_float(f,3);
	
	w=LedState();
	send_msg("\r\nLedState:");
	send_u32(w);
	
	w=RedLightState();
	send_msg("\r\nRedLightState:");
	send_u32(w);

  send_msg("\r\nTemperature:");
	send_float(Temperature,1);
  */
/*	tx_mode();
	delay_ms(10);
	send_msg("\r\nrpm_1: ");
	f=rpm_1;
	delay_ms(5);
	send_float(f,3);
	delay_ms(10);
	rx_mode();*/
	/*
	send_msg("\r\n DR1 ");
	send_hex(HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR1));
	send_msg("\r\n DR2 ");
	send_hex(HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR2));*/
	/*
	HAL_RTC_GetTime(&hrtc,&Time,RTC_FORMAT_BCD);
	send_msg("\r\nTime: ");
	send_u32(BCD2HEX(Time.Hours));
	send_msg(": ");
	send_u32(BCD2HEX(Time.Minutes));
	send_msg(": ");
	send_u32(BCD2HEX(Time.Seconds));*/
	/*
	send_msg("\r\nrpm_2:");
	f=rpm_2;
	send_float(f,3);*/
	
	/*
	send_msg("\r\nGear:");
	send_u32(Gear);*/
	/*
	send_msg("\r\nControl:");
	send_u32(adc_value[ADC_Control]);*/
	
	
	
	
}
void SendFullInfo()
{
	uint8_t p=1;
	info_buffer[0]=0x02;
	info_buffer[p++]=1;
	tx_mode();
	delay_ms(1);
	 
	send_data(info_buffer,p);
	
}
void ProcessRequest(uint8_t *data)
{
	uint8_t p=0;
	switch(data[p++])
	{
		case 0x81 :
		StabI1.run=data[p++];
    StabU1.run=data[p++];
    StabU2.run=data[p++];
    RedLightSetState(data[p++]);		
		break;
		case 0x82 :
		u16u8.u8[0]=data[p++];
		u16u8.u8[1]=data[p++];
		StabI1.I=u16u8.u16;
		u16u8.u8[0]=data[p++];
		u16u8.u8[1]=data[p++];
		StabU1.U=u16u8.u16;
		u16u8.u8[0]=data[p++];
		u16u8.u8[1]=data[p++];
		StabU2.U=u16u8.u16;
		break;
		case 0x83 :
		rev_count_1=0;
		break;
		case 0x84 :
		rev_count_2=0;	
		break;
		case 0x85 :
			SendFullInfo();
		break;
		default:
			break;
	}
	SaveConfig();
}
bool CheckBKUP(void)
{
	uint16_t crc=0;
	crc^=HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR1);
	crc^=HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR2);
	crc^=HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR3);
	crc^=HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR4);
	crc^=HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR5);
	crc^=HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR6);
	crc^=HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR7);
	return ((crc==HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR10))&&(HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR9)==0xAA));
}
void SaveConfig(void)
{	
	uint16_t data;
	uint16_t crc=0;
	if(Ready)
	{
	do
	{
		data=StabI1.I;
		crc^=data;
	HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1,data);
		data=StabU1.U;
		crc^=data;
	HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR2,data);
		data=StabU2.U;
		crc^=data;
	HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR3,data);
	u32u16.u32=rev_count_1;
		data=u32u16.u16[0];
		crc^=data;
	HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR4,data);
		data=u32u16.u16[1];
		crc^=data;
	HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR5,data);
	u32u16.u32=rev_count_2;
		data=u32u16.u16[0];
		crc^=data;
	HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR6,data);
		data=u32u16.u16[1];
		crc^=data;
	HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR7,data);
	HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR9,0xAA);
	HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR10,crc);
	}while(!CheckBKUP());
	Saved=true;
}
}
void LoadConfig(void)
{
	if(CheckBKUP())
	{
	StabI1.I=HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR1);
	StabU1.U=HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR2);
	StabU2.U=HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR3);
	u32u16.u16[0]=HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR4);
	u32u16.u16[1]=HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR5);
	rev_count_1=u32u16.u32;
	u32u16.u16[0]=HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR6);
	u32u16.u16[1]=HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR7);
	rev_count_2=u32u16.u32;
	}
	else
	{
		rev_count_1=0;
		rev_count_2=0;
	}
}
void ChangeConfig(uint16_t I,uint16_t U1,uint16_t U2)
{
	StabI1.I=I;
	StabU1.U=U1;
	StabU2.U=U2;
	SaveConfig();
}
void send_float(float f,char presition)
{
	uint32_t mask=1,d;
  uint8_t count=0,i=0;
	uint8_t buffer[50];
	if(f<0.0)
	{
		f=-f;
		buffer[i++]='-';
		count++;
	}
	d=(uint32_t)f;
	while(d>=mask)
	{
		mask*=10;
		count++;
	}
	if((count+presition)<50)
	{
		mask/=10;
		for(;i<count;i++)
		{
			buffer[i]='0'+(uint8_t)(d/mask);
			d%=mask;
			mask/=10;
		}
		buffer[i++]='.';
		mask=pow(10.0,(double)presition);
		
		d=f*(float)mask;
		d%=mask;
		mask/=10;
		for(;i<(count+1+presition);i++)
		{
			buffer[i]='0'+d/mask;
			d%=mask;
			mask/=10;
		}
	}
	count=(count+1+presition);
	buffer[count++]='\r';
	buffer[count++]='\n';
	send_data(buffer,count);
}
void ByteToHex(uint8_t b,uint8_t *buffer,uint8_t pos)
{
	uint8_t c1,c2;
	c1=b/16;
	c2=b%16;
	if(c1>9)c1='A'+c1-10;
	else c1+='0';
	if(c2>9)c2='A'+c2-10;
  else c2+='0';
  buffer[pos++]=c1;
  buffer[pos]=c2;	
}
void send_hex(uint32_t u32)
{
	uint8_t u;
	uint8_t buffer[]="0x00000000";
	u=u32>>24;
	ByteToHex(u,buffer,2);
	u=u32>>16;
	ByteToHex(u,buffer,4);
	u=u32>>8;
	ByteToHex(u,buffer,6);
	u=u32;
	ByteToHex(u,buffer,8);
	send_data(buffer,10);
}
void send_u32(uint32_t u32)
{
	uint8_t buffer[]="000000000";
	uint8_t c,p,pe;
	i=100000000;
	for(c=0;c<9;c++)
	{
		buffer[c]+=u32 / i;
		u32%=i;
		i/=10;
	}
	for(c=0;c<8;c++) if(buffer[c]=='0')buffer[c]=' ';
	                   else break;
	pe=9-c;
	for(p=0;p<pe;p++,c++) buffer[p]=buffer[c];
	
	send_data(buffer,p);
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
