/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ANALOG_BUTTONS_H
#define __ANALOG_BUTTONS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"


/* Exported types ------------------------------------------------------------*/
typedef enum{
	btn_none,
	btn_click,
	btn_down,
	btn_holded,
	btn_holded_l,
	btn_up
} btnSatets;

typedef struct{
	uint8_t					code;
	btnSatets		state;
} AB_ButtonStateTypeDef;

/* Exported constants --------------------------------------------------------*/


/* ## Definition of ADC resources */
/* Definition of ADCx clock resources */
#define ADCx                            ADC1
#define ADCx_CLK_ENABLE()               __HAL_RCC_ADC1_CLK_ENABLE()

/* Definition of ADCx channels */
#define ADCx_CHANNELa                   ADC_CHANNEL_0

/* Definition of ADCx channels pins */
#define ADCx_CHANNELa_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define ADCx_CHANNELa_GPIO_PORT         GPIOA
#define ADCx_CHANNELa_PIN               GPIO_PIN_0

/* Definition of ADCx DMA resources */
#define ADCx_DMA_CLK_ENABLE()           __HAL_RCC_DMA1_CLK_ENABLE()
#define ADCx_DMA                        DMA1_Channel1

#define ADCx_DMA_IRQn                   DMA1_Channel1_IRQn
#define ADCx_DMA_IRQHandler             DMA1_Channel1_IRQHandler

/* Definition of ADCx NVIC resources */
#define ADCx_IRQn                       ADC1_2_IRQn
#define ADCx_IRQHandler                 ADC1_2_IRQHandler

/* Definition if Timer NVIC resources */
#define TIMx_IRQn						TIM3_IRQn
#define TIMx_IRQHandler					TIM3_IRQHandler

/* Definition of TIMx clock resources */
#define TIMx                            TIM3    /* Caution: Timer instance must be on APB1 (clocked by PCLK1) due to frequency computation in function "TIM_Config()" */
#define TIMx_CLK_ENABLE()               __HAL_RCC_TIM3_CLK_ENABLE()

#define TIMx_FORCE_RESET()              __HAL_RCC_TIM3_FORCE_RESET()
#define TIMx_RELEASE_RESET()            __HAL_RCC_TIM3_RELEASE_RESET()

#define ADC_EXTERNALTRIGCONV_Tx_TRGO    ADC_EXTERNALTRIGCONV_T3_TRGO


#define BUTTONS_COUNT					5
#define BUTTON_PUSHED_CYCLES			4
#define BUTTON_HOLDING_CYCLES			10
#define BUTTON_HOLDING_LONG_CYCLES		30



/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void init_analog_buttons(void);					/* initialization buttons     */	
btnSatets GetBtnState(uint8_t btnNum);			/* Get button state on btnNum */


#endif /* __ANALOG_BUTTONS_H */

