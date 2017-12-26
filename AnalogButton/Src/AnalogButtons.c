/**
  ******************************************************************************
  * @file    ADC AnalogButton.c
  * @author  z.m.dadaev
  * @version V1.1.0
  * @date    08-April-2017
  * @brief   Описание здесь будет
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 z.m.dadaev </center></h2>
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "analogbuttons.h"

/* Private define ------------------------------------------------------------*/
#define ADC_BUFFER_SIZE 							 ((uint32_t) 4) /* Size of array containing ADC converted values */

#define TIMER_FREQUENCY                ((uint32_t)10) /* Timer frequency (unit: Hz). With a timer 16 bits and time base freq min 1Hz, range is min=1Hz, max=32kHz. */
#define TIMER_FREQUENCY_RANGE_MIN      ((uint32_t) 1) /* Timer minimum frequency (unit: Hz). With a timer 16 bits, maximum frequency will be 32000 times this value. */
#define TIMER_PRESCALER_MAX_VALUE      (0xFFFF-1)     /* Timer prescaler maximum value (0xFFFF for a timer 16 bits) */


/* Private variables ---------------------------------------------------------*/
__IO uint32_t button_event_state 	= 0;
uint16_t BUTTON_MASK 							= 32-1;
AB_ButtonStateTypeDef buttonInfo;

/* ADC handler declaration */
ADC_HandleTypeDef   AdcHandle;
/* Timer handler declaration */
TIM_HandleTypeDef   TimHandle;
DMA_HandleTypeDef  	DmaHandle;


/* Variable containing ADC results */
__IO uint32_t   aADCxValue;

#define ANY_BUTTON_HOLDED 	((button_event_state & ~BUTTON_MASK) != 0)
#define ANY_BUTTON_CLICKED 	((button_event_state & BUTTON_MASK) != 0)
#define BTN_HOLDED(btncode) (button_event_state & (btncode + (BUTTONS_COUNT<<1) - 1))


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
static void ADC_Config(void);
static void TIM_Config(void);


void ButtonsPressCallback(AB_ButtonStateTypeDef *ButtonState);
void Button_1_Pressed(void);
void Button_2_Pressed(void);
void Button_3_Pressed(void);
void Button_4_Pressed(void);
void Button_5_Pressed(void);
	



/*################# ADC INITIALIZATION	######################################*/
/**
  * @brief  ADC configuration
  * @param  None
  * @retval None
  */
static void ADC_Config(void)
{
  ADC_ChannelConfTypeDef   sConfig;
  
  /* Configuration of ADCx init structure: ADC parameters and regular group */
  AdcHandle.Instance 									 = ADCx;
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.ScanConvMode          = ADC_SCAN_DISABLE;              /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
  AdcHandle.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
  AdcHandle.Init.NbrOfConversion       = 1;                             /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_Tx_TRGO;  /* Trig of conversion start done by external event */

  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    /* ADC initialization error */
    Error_Handler();
  }

  sConfig.Channel      = ADCx_CHANNELa;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }

}


/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief ADC MSP initialization
  *        This function configures the hardware resources used in this example:
  *          - Enable clock of ADC peripheral
  *          - Configure the GPIO associated to the peripheral channels
  *          - Configure the DMA associated to the peripheral
  *          - Configure the NVIC associated to the peripheral interruptions
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef          GPIO_InitStruct;
  
  RCC_PeriphCLKInitTypeDef  PeriphClkInit;
  
  /* Enable clock of GPIO associated to the peripheral channels */
  ADCx_CHANNELa_GPIO_CLK_ENABLE();
  
  /* Enable clock of ADCx peripheral */
  ADCx_CLK_ENABLE();

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /* Enable clock of DMA associated to the peripheral */
  ADCx_DMA_CLK_ENABLE();
  
  /* Configure GPIO pin of the selected ADC channel */
  GPIO_InitStruct.Pin 	= ADCx_CHANNELa_PIN;
  GPIO_InitStruct.Mode 	= GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull 	= GPIO_NOPULL;
  HAL_GPIO_Init(ADCx_CHANNELa_GPIO_PORT, &GPIO_InitStruct);
  
  /* Configure DMA parameters */
  DmaHandle.Instance = ADCx_DMA;

  DmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  DmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
  DmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
  DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;   /* Transfer from ADC by half-word to match with ADC configuration: ADC resolution 10 or 12 bits */
  DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;   /* Transfer to memory by half-word to match with buffer variable type: half-word */
  DmaHandle.Init.Mode                = DMA_CIRCULAR;              /* DMA in circular mode to match with ADC configuration: DMA continuous requests */
  DmaHandle.Init.Priority            = DMA_PRIORITY_HIGH;
  
  /* Deinitialize  & Initialize the DMA for new transfer */
  HAL_DMA_DeInit(&DmaHandle);
  HAL_DMA_Init(&DmaHandle);

  /* Associate the initialized DMA handle to the ADC handle */
  __HAL_LINKDMA(hadc, DMA_Handle, DmaHandle);

  HAL_NVIC_SetPriority(ADCx_DMA_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(ADCx_DMA_IRQn);
  
  HAL_NVIC_SetPriority(ADCx_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADCx_IRQn);
}


void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  //ADCx_FORCE_RESET();
  //ADCx_RELEASE_RESET();

  /* De-initialize GPIO pin of the selected ADC channel */
  HAL_GPIO_DeInit(ADCx_CHANNELa_GPIO_PORT, ADCx_CHANNELa_PIN);

  /* De-Initialize the DMA associated to the peripheral */
  if(hadc->DMA_Handle != NULL)
  {
    HAL_DMA_DeInit(hadc->DMA_Handle);
  }


  /* Disable the NVIC configuration for DMA interrupt */
  HAL_NVIC_DisableIRQ(ADCx_DMA_IRQn);
  
  /* Disable the NVIC configuration for ADC interrupt */
  HAL_NVIC_DisableIRQ(ADCx_IRQn);
}



btnSatets GetBtnState(uint8_t btnNum){
	uint32_t shift = 0;
	
	if (button_event_state & (1 << (btnNum-1))){
		shift = (1UL << (btnNum + (BUTTONS_COUNT<<2) - 1));
		if (button_event_state & shift){ 
			button_event_state &= ~shift; 
			return btn_holded_l;
		}
			
		shift = (1UL << (btnNum + (BUTTONS_COUNT<<1) - 1));
		if (button_event_state & shift){ 
			button_event_state &= ~shift; 
			return btn_holded;		
		}
		
		shift = (1UL << (btnNum + BUTTONS_COUNT - 1));
		if (button_event_state & shift){ 
			button_event_state &= ~shift; 
			return btn_down;				
		}
		
		if ((button_event_state & ~(BUTTON_MASK)) == 0){
			return btn_click;						
		}
	}
	return btn_none;
}

void ADCx_DMA_IRQHandler(void)
{
	uint8_t btnNum;
	static uint8_t holded_cycles = 0;	/* btn pressed cycles	*/
	
  HAL_DMA_IRQHandler(&DmaHandle);
	
	/* devide 256	*/
	btnNum = (aADCxValue >> 8);																

	
	/* if button pressed	*/
	if (btnNum){	
		
		if (holded_cycles < BUTTON_HOLDING_LONG_CYCLES) holded_cycles++;
		
		button_event_state |= 1 << (btnNum-1); 				
		 
		buttonInfo.code = btnNum;
		buttonInfo.state = btn_click;
		
		
		/* #########################	BTN PUSHED HANDLING	########################*/
		if ( (holded_cycles >= BUTTON_PUSHED_CYCLES) & \
			(holded_cycles < BUTTON_HOLDING_CYCLES) ) {
			button_event_state |= 1UL << (btnNum + BUTTONS_COUNT - 1);
			buttonInfo.state = btn_down;
			ButtonsPressCallback(&buttonInfo);
		}
		
		/* #########################	BTN HOLD PROCESSING	 #######################*/
		if ( (holded_cycles >= BUTTON_HOLDING_CYCLES) & \
			(holded_cycles < BUTTON_HOLDING_LONG_CYCLES) ){
			button_event_state |= 1UL << (btnNum + (BUTTONS_COUNT<<1) - 1);
			buttonInfo.state = btn_holded;
			ButtonsPressCallback(&buttonInfo);
		}
		
		/* #########################	BTN LHOLD PROCESSING #######################*/
		if ( holded_cycles == BUTTON_HOLDING_LONG_CYCLES ){
			button_event_state |= 1UL << (btnNum + (BUTTONS_COUNT<<2) - 1); 
			buttonInfo.state = btn_holded_l;
			ButtonsPressCallback(&buttonInfo);
		}		
	}else{
		/* #########################	BTN CLICK PROCESSING #######################*/
		if ((buttonInfo.code) && (buttonInfo.state == btn_click)){
			buttonInfo.state = btn_click;
			ButtonsPressCallback(&buttonInfo);
		}
		
		holded_cycles 		= 0;
		
		/* reset button info state	*/
		buttonInfo.code 	= 0;
		buttonInfo.state 	= btn_none;
	
	} //if (btnNum){	
	
	
	
	
}



/*####################	TIMER CONFIGURATION	##################################*/	

void TIM_Config(void)
{
	
  TIM_MasterConfigTypeDef master_timer_config;
  RCC_ClkInitTypeDef clk_init_struct = {0};       /* Temporary variable to retrieve RCC clock configuration */
  uint32_t latency;                               /* Temporary variable to retrieve Flash Latency */
  uint32_t timer_clock_frequency = 0;             /* Timer clock frequency */
  uint32_t timer_prescaler = 0;                   /* Time base prescaler to have timebase aligned on minimum frequency possible */
  
  /* Retrieve timer clock source frequency */
  HAL_RCC_GetClockConfig(&clk_init_struct, &latency);

  if (clk_init_struct.APB1CLKDivider == RCC_HCLK_DIV1)
  {
    timer_clock_frequency = HAL_RCC_GetPCLK1Freq();
  }
  else
  {
    timer_clock_frequency = HAL_RCC_GetPCLK1Freq() * 2;
  }
  
  /* Timer prescaler calculation */
  /* (computation for timer 16 bits, additional + 1 to round the prescaler up) */
  timer_prescaler = (timer_clock_frequency / (TIMER_PRESCALER_MAX_VALUE * TIMER_FREQUENCY_RANGE_MIN)) +1;
  
  /* Set timer instance */
  TimHandle.Instance = TIMx;
  
  /* Configure timer parameters */
  TimHandle.Init.Period            = ((timer_clock_frequency / (timer_prescaler * TIMER_FREQUENCY)) - 1);
  TimHandle.Init.Prescaler         = (timer_prescaler - 1);
  TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0x0;
  
  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Timer initialization Error */
    Error_Handler();
  }

  /* Timer TRGO selection */
  master_timer_config.MasterOutputTrigger = TIM_TRGO_UPDATE;
  master_timer_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &master_timer_config) != HAL_OK)
  {
    /* Timer TRGO selection Error */
    Error_Handler();
  }
	
}


void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  /* TIM peripheral clock enable */
  TIMx_CLK_ENABLE();
}

/**
  * @brief TIM MSP de-initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable clock of peripheral
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim)
{
  TIMx_FORCE_RESET();
  TIMx_RELEASE_RESET();
}







/*################### ANALOG BUTTON INITIALIZATION	##########################*/	
/**
  * @brief  Init functions.
  * @param  None
  * @retval None
  */
void init_analog_buttons(void)
{
  
  //HAL_Init();
  /* Configure the system clock to 64 MHz */
  //SystemClock_Config();
  /* Configure the ADC peripheral */
  ADC_Config();
  /* Run the ADC calibration */  
  if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }

  TIM_Config();
	if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    /* Counter Enable Error */
    Error_Handler();
  }
	
	/* debug construction	*/
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);	


  /* Start ADC conversions */
  if (HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)&aADCxValue, 1) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
}
/*################### ANALOG BUTTON INITIALIZATION	##########################*/






/* Private functions ---------------------------------------------------------*/
__weak void ButtonsPressCallback(AB_ButtonStateTypeDef *ButtonState){
		(void)(ButtonState);
}








/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : AdcHandle handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
	
	
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{


}


