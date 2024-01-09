#ifndef __STM32F1xx_HAL_H
#define __STM32F1xx_HAL_H

#define HAL_MODULE_ENABLED
#define HAL_PCD_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
//#define HAL_GPIO_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED

#define HSE_VALUE    			8000000U 	/*!< Value of the External oscillator in Hz */
#define HSE_STARTUP_TIMEOUT    	100U   		/*!< Time out for HSE start up, in ms */
#define HSI_VALUE    			8000000U 	/*!< Value of the Internal oscillator in Hz*/
#define LSI_VALUE               40000U    	/*!< LSI Typical Value in Hz */
#define LSE_VALUE    			32768U 		/*!< Value of the External oscillator in Hz*/
#define LSE_STARTUP_TIMEOUT    	5000U   	/*!< Time out for LSE start up, in ms */
#define TICK_INT_PRIORITY 		0U    		/*!< tick interrupt priority (lowest by default)  */

#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32f1xx_hal_rcc.h"
#endif /* HAL_RCC_MODULE_ENABLED */

//#ifdef HAL_GPIO_MODULE_ENABLED
//#include "stm32f1xx_hal_gpio.h"
//#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_CORTEX_MODULE_ENABLED
#define NVIC_PRIORITYGROUP_0         0x00000007U
#define NVIC_PRIORITYGROUP_1         0x00000006U
#define NVIC_PRIORITYGROUP_2         0x00000005U
#define NVIC_PRIORITYGROUP_3         0x00000004U
#define NVIC_PRIORITYGROUP_4         0x00000003U
#endif /* HAL_CORTEX_MODULE_ENABLED */

#ifdef HAL_FLASH_MODULE_ENABLED
#define __HAL_FLASH_SET_LATENCY(__LATENCY__)    	(FLASH->ACR = (FLASH->ACR&(~FLASH_ACR_LATENCY)) | (__LATENCY__))
#define __HAL_FLASH_GET_LATENCY()     				(READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY))
#define __HAL_FLASH_PREFETCH_BUFFER_ENABLE()    	(FLASH->ACR |= FLASH_ACR_PRFTBE)
#endif /* HAL_FLASH_MODULE_ENABLED */

#ifdef HAL_PCD_MODULE_ENABLED
#include "stm32f1xx_hal_pcd.h"
#endif /* HAL_PCD_MODULE_ENABLED */

#define assert_param(expr) ((void)0U)

typedef enum{
  HAL_TICK_FREQ_10HZ         = 100U,
  HAL_TICK_FREQ_100HZ        = 10U,
  HAL_TICK_FREQ_1KHZ         = 1U,
  HAL_TICK_FREQ_DEFAULT      = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;

extern __IO uint32_t uwTick;
extern uint32_t uwTickPrio;
extern HAL_TickFreqTypeDef uwTickFreq;

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);


#endif /* __STM32F1xx_HAL_H */
