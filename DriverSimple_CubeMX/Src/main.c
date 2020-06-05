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
#include "stdbool.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define delay HAL_Delay
// ADC constants===============================================================
#define Vref 3.283
#define ADC_Koef Vref/4095.0
#define CH0_Koef 1.0*ADC_Koef             //gears
#define CH1_Koef 1.0*ADC_Koef             //control
#define CH4_Koef 1.063*5.0*ADC_Koef       //Step UP 1
#define CH5_Koef 1.054*5.0*ADC_Koef        //Step UP 2
#define CH6_Koef 1.048*ADC_Koef/Koy1/R1   //I LED
#define CH7_Koef 1.0*2.0*ADC_Koef         //U LED
#define CH8_Koef 1.033*2.0*ADC_Koef       //U POWER
#define Koy1 10.1
#define R1 0.1
float CHx_Koef[]={CH0_Koef,CH1_Koef,CH4_Koef,CH5_Koef,CH6_Koef,CH7_Koef,CH8_Koef};
#define ADC_Gears    0
#define ADC_Control  1
#define ADC_Stab1U   2
#define ADC_Stab2U   3
#define ADC_Stab3I   4
#define ADC_Stab3U   5
#define ADC_InU      6
//=============================================================================
#define green_led_pin GPIO_PIN_7
#define green_led_port GPIOB
#define green_led_on  HAL_GPIO_WritePin(green_led_port,green_led_pin,GPIO_PIN_SET);
#define green_led_off HAL_GPIO_WritePin(green_led_port,green_led_pin,GPIO_PIN_RESET);
uint8_t green_led_state=0;

#define red_led_pin GPIO_PIN_6
#define red_led_port GPIOB
#define red_led_on  HAL_GPIO_WritePin(red_led_port,red_led_pin,GPIO_PIN_SET);
#define red_led_off HAL_GPIO_WritePin(red_led_port,red_led_pin,GPIO_PIN_RESET);
uint8_t red_led_state=0;

#define red_light_pin GPIO_PIN_14
#define red_light_port GPIOC
#define red_light_on  HAL_GPIO_WritePin(red_light_port,red_light_pin,GPIO_PIN_SET);
#define red_light_off HAL_GPIO_WritePin(red_light_port,red_light_pin,GPIO_PIN_RESET);
uint8_t red_light_state=0;

#define red_strip_pin GPIO_PIN_13
#define red_strip_port GPIOC
#define red_strip_on  HAL_GPIO_WritePin(red_strip_port,red_strip_pin,GPIO_PIN_SET);
#define red_strip_off HAL_GPIO_WritePin(red_strip_port,red_strip_pin,GPIO_PIN_RESET);
uint8_t red_strip_state=0;

#define green_strip_pin GPIO_PIN_9
#define green_strip_port GPIOB
#define green_strip_on  HAL_GPIO_WritePin(green_strip_port,green_strip_pin,GPIO_PIN_SET);
#define green_strip_off HAL_GPIO_WritePin(green_strip_port,green_strip_pin,GPIO_PIN_RESET);
uint8_t green_strip_state=0;

#define blue_strip_pin GPIO_PIN_8
#define blue_strip_port GPIOB
#define blue_strip_on  HAL_GPIO_WritePin(blue_strip_port,blue_strip_pin,GPIO_PIN_SET);
#define blue_strip_off HAL_GPIO_WritePin(blue_strip_port,blue_strip_pin,GPIO_PIN_RESET);
uint8_t blue_strip_state=0;

uint8_t adc_channel = 0;
uint16_t adc_value[8]={0,0,0,0,0,0,0,0};

#define MaxButtonCount 10
#define ButtonsADCRange 100
uint16_t buttons[MaxButtonCount]={1965,2170,2383,2703,3060};
#define button_count 5
uint8_t button_action[button_count]={1,2,3,4,5};
uint8_t button_pressed=false;

HAL_StatusTypeDef s,s2;

bool Ready=false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM6_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

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
	uint16_t U;
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
void Core2A(void);
void Core2B(void);

void StabU_Procc(struct StabU *stab);
void StabI_Procc(struct StabI *stab);

uint8_t CheckButtons(void);

//******** Core0 temps ****************
float I;

uint8_t SYS_FCLK;

uint8_t color=0;

void ChangeColor(void);
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
	while(ms--) delay_us(1000);
}

uint16_t GetImA(char ch)
{
	//uint16_t I;
	I=adc_value[ch];
	I*=(CHx_Koef[ch] * 1000.0);
	return I;
}

uint16_t GetUmv(char ch)
{
	uint16_t U;
	U=adc_value[ch];
	U*=(CHx_Koef[ch] * 1000.0);
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
}
//***************************************************

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
				 ChangeColor();
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
	power.U=GetUmv(ADC_InU);
	delay_us(10);
	power.U+=GetUmv(ADC_InU);
	delay_us(10);
	power.U+=GetUmv(ADC_InU);
	power.U/=3;	
		
	if(power.U<power.Umin) power.run=false;
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
		 HAL_PWR_EnterSTANDBYMode();
		 while(true);
   }	
	 if(power.warning)
	{
		GreenLedSetState(false);
		RedLedInvState();
	}
	else
	{
		GreenLedSetState(true);
		RedLedSetState(false);
	}
 }
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
  MX_TIM2_Init();
  MX_TIM15_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */
	SYS_FCLK=HAL_RCC_GetHCLKFreq()/1e+6;

	//HAL_TIM_Base_Start_IT(&htim1);          // timers interrupt
	HAL_TIM_Base_Start_IT(&htim6);
	
	power.Umin=2850;
	power.Uwarning=3200;
	power.run=true;	
	delay(500);
  
	__enable_irq();
	
	//HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc_value,7);
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
	StabU1.U=7000;
	StabU1.run=false;
	
  StabU2.ADC_CH=ADC_Stab2U;
	StabU2.state=false;
	StabU2.StepUpMaxValue=250;
	StabU2.StepUpValue=0;
	StabU2.TIM=&htim15;
	StabU2.TIM_CH=TIM_CHANNEL_2;
	StabU2.TIM_Value=&(TIM15->CCR2);
	StabU2.U=10500;
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
	
	green_strip_on
	
	Ready=true;
	


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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV4;
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
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
  hadc1.Init.NbrOfConversion = 7;
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB12 PB13 
                           PB14 PB15 PB3 PB4 
                           PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11 
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

void ChangeColor(void)
{
	green_strip_off
	red_strip_off
	blue_strip_off
	if(color<6) color++;
	else color=0;
	switch(color)
	{
		case 0:
		green_strip_on
		break;
		case 1:
		red_strip_on
		break;
		case 2:
		blue_strip_on
		break;
		case 3:
		red_strip_on
		green_strip_on
		break;
		case 4:
		red_strip_on
		blue_strip_on
		break;
		case 5:
		green_strip_on
		blue_strip_on
		break;
		case 6:
		green_strip_on
		blue_strip_on
		red_strip_on
		break;
	}
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
