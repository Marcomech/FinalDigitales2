

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32F1xx_HAL_GPIO_EX_H
#define STM32F1xx_HAL_GPIO_EX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal_def.h"




/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/







#define AFIO_EVENTOUT_PIN_0  AFIO_EVCR_PIN_PX0 /*!< EVENTOUT on pin 0 */
#define AFIO_EVENTOUT_PIN_1  AFIO_EVCR_PIN_PX1 /*!< EVENTOUT on pin 1 */
#define AFIO_EVENTOUT_PIN_2  AFIO_EVCR_PIN_PX2 /*!< EVENTOUT on pin 2 */
#define AFIO_EVENTOUT_PIN_3  AFIO_EVCR_PIN_PX3 /*!< EVENTOUT on pin 3 */
#define AFIO_EVENTOUT_PIN_4  AFIO_EVCR_PIN_PX4 /*!< EVENTOUT on pin 4 */
#define AFIO_EVENTOUT_PIN_5  AFIO_EVCR_PIN_PX5 /*!< EVENTOUT on pin 5 */
#define AFIO_EVENTOUT_PIN_6  AFIO_EVCR_PIN_PX6 /*!< EVENTOUT on pin 6 */
#define AFIO_EVENTOUT_PIN_7  AFIO_EVCR_PIN_PX7 /*!< EVENTOUT on pin 7 */
#define AFIO_EVENTOUT_PIN_8  AFIO_EVCR_PIN_PX8 /*!< EVENTOUT on pin 8 */
#define AFIO_EVENTOUT_PIN_9  AFIO_EVCR_PIN_PX9 /*!< EVENTOUT on pin 9 */
#define AFIO_EVENTOUT_PIN_10 AFIO_EVCR_PIN_PX10 /*!< EVENTOUT on pin 10 */
#define AFIO_EVENTOUT_PIN_11 AFIO_EVCR_PIN_PX11 /*!< EVENTOUT on pin 11 */
#define AFIO_EVENTOUT_PIN_12 AFIO_EVCR_PIN_PX12 /*!< EVENTOUT on pin 12 */
#define AFIO_EVENTOUT_PIN_13 AFIO_EVCR_PIN_PX13 /*!< EVENTOUT on pin 13 */
#define AFIO_EVENTOUT_PIN_14 AFIO_EVCR_PIN_PX14 /*!< EVENTOUT on pin 14 */
#define AFIO_EVENTOUT_PIN_15 AFIO_EVCR_PIN_PX15 /*!< EVENTOUT on pin 15 */

#define IS_AFIO_EVENTOUT_PIN(__PIN__) (((__PIN__) == AFIO_EVENTOUT_PIN_0) || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_1) || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_2) || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_3) || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_4) || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_5) || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_6) || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_7) || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_8) || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_9) || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_10) || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_11) || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_12) || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_13) || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_14) || \
                                       ((__PIN__) == AFIO_EVENTOUT_PIN_15))




#define AFIO_EVENTOUT_PORT_A AFIO_EVCR_PORT_PA /*!< EVENTOUT on port A */
#define AFIO_EVENTOUT_PORT_B AFIO_EVCR_PORT_PB /*!< EVENTOUT on port B */
#define AFIO_EVENTOUT_PORT_C AFIO_EVCR_PORT_PC /*!< EVENTOUT on port C */
#define AFIO_EVENTOUT_PORT_D AFIO_EVCR_PORT_PD /*!< EVENTOUT on port D */
#define AFIO_EVENTOUT_PORT_E AFIO_EVCR_PORT_PE /*!< EVENTOUT on port E */

#define IS_AFIO_EVENTOUT_PORT(__PORT__) (((__PORT__) == AFIO_EVENTOUT_PORT_A) || \
                                         ((__PORT__) == AFIO_EVENTOUT_PORT_B) || \
                                         ((__PORT__) == AFIO_EVENTOUT_PORT_C) || \
                                         ((__PORT__) == AFIO_EVENTOUT_PORT_D) || \
                                         ((__PORT__) == AFIO_EVENTOUT_PORT_E))







#define __HAL_AFIO_REMAP_SPI1_ENABLE()  AFIO_REMAP_ENABLE(AFIO_MAPR_SPI1_REMAP)


#define __HAL_AFIO_REMAP_SPI1_DISABLE()  AFIO_REMAP_DISABLE(AFIO_MAPR_SPI1_REMAP)


#define __HAL_AFIO_REMAP_I2C1_ENABLE()  AFIO_REMAP_ENABLE(AFIO_MAPR_I2C1_REMAP)


#define __HAL_AFIO_REMAP_I2C1_DISABLE() AFIO_REMAP_DISABLE(AFIO_MAPR_I2C1_REMAP)


#define __HAL_AFIO_REMAP_USART1_ENABLE()  AFIO_REMAP_ENABLE(AFIO_MAPR_USART1_REMAP)


#define __HAL_AFIO_REMAP_USART1_DISABLE() AFIO_REMAP_DISABLE(AFIO_MAPR_USART1_REMAP)


#define __HAL_AFIO_REMAP_USART2_ENABLE()  AFIO_REMAP_ENABLE(AFIO_MAPR_USART2_REMAP)


#define __HAL_AFIO_REMAP_USART2_DISABLE() AFIO_REMAP_DISABLE(AFIO_MAPR_USART2_REMAP)


#define __HAL_AFIO_REMAP_USART3_ENABLE()  AFIO_REMAP_PARTIAL(AFIO_MAPR_USART3_REMAP_FULLREMAP, AFIO_MAPR_USART3_REMAP_FULLREMAP)


#define __HAL_AFIO_REMAP_USART3_PARTIAL()  AFIO_REMAP_PARTIAL(AFIO_MAPR_USART3_REMAP_PARTIALREMAP, AFIO_MAPR_USART3_REMAP_FULLREMAP)


#define __HAL_AFIO_REMAP_USART3_DISABLE()  AFIO_REMAP_PARTIAL(AFIO_MAPR_USART3_REMAP_NOREMAP, AFIO_MAPR_USART3_REMAP_FULLREMAP)


#define __HAL_AFIO_REMAP_TIM1_ENABLE()  AFIO_REMAP_PARTIAL(AFIO_MAPR_TIM1_REMAP_FULLREMAP, AFIO_MAPR_TIM1_REMAP_FULLREMAP)


#define __HAL_AFIO_REMAP_TIM1_PARTIAL()  AFIO_REMAP_PARTIAL(AFIO_MAPR_TIM1_REMAP_PARTIALREMAP, AFIO_MAPR_TIM1_REMAP_FULLREMAP)


#define __HAL_AFIO_REMAP_TIM1_DISABLE()  AFIO_REMAP_PARTIAL(AFIO_MAPR_TIM1_REMAP_NOREMAP, AFIO_MAPR_TIM1_REMAP_FULLREMAP)


#define __HAL_AFIO_REMAP_TIM2_ENABLE()  AFIO_REMAP_PARTIAL(AFIO_MAPR_TIM2_REMAP_FULLREMAP, AFIO_MAPR_TIM2_REMAP_FULLREMAP)


#define __HAL_AFIO_REMAP_TIM2_PARTIAL_2()  AFIO_REMAP_PARTIAL(AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2, AFIO_MAPR_TIM2_REMAP_FULLREMAP)


#define __HAL_AFIO_REMAP_TIM2_PARTIAL_1()  AFIO_REMAP_PARTIAL(AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1, AFIO_MAPR_TIM2_REMAP_FULLREMAP)


#define __HAL_AFIO_REMAP_TIM2_DISABLE()  AFIO_REMAP_PARTIAL(AFIO_MAPR_TIM2_REMAP_NOREMAP, AFIO_MAPR_TIM2_REMAP_FULLREMAP)


#define __HAL_AFIO_REMAP_TIM3_ENABLE()  AFIO_REMAP_PARTIAL(AFIO_MAPR_TIM3_REMAP_FULLREMAP, AFIO_MAPR_TIM3_REMAP_FULLREMAP)


#define __HAL_AFIO_REMAP_TIM3_PARTIAL()  AFIO_REMAP_PARTIAL(AFIO_MAPR_TIM3_REMAP_PARTIALREMAP, AFIO_MAPR_TIM3_REMAP_FULLREMAP)


#define __HAL_AFIO_REMAP_TIM3_DISABLE()  AFIO_REMAP_PARTIAL(AFIO_MAPR_TIM3_REMAP_NOREMAP, AFIO_MAPR_TIM3_REMAP_FULLREMAP)


#define __HAL_AFIO_REMAP_TIM4_ENABLE()  AFIO_REMAP_ENABLE(AFIO_MAPR_TIM4_REMAP)


#define __HAL_AFIO_REMAP_TIM4_DISABLE() AFIO_REMAP_DISABLE(AFIO_MAPR_TIM4_REMAP)

#if defined(AFIO_MAPR_CAN_REMAP_REMAP1)


#define __HAL_AFIO_REMAP_CAN1_1()  AFIO_REMAP_PARTIAL(AFIO_MAPR_CAN_REMAP_REMAP1, AFIO_MAPR_CAN_REMAP)


#define __HAL_AFIO_REMAP_CAN1_2()  AFIO_REMAP_PARTIAL(AFIO_MAPR_CAN_REMAP_REMAP2, AFIO_MAPR_CAN_REMAP)


#define __HAL_AFIO_REMAP_CAN1_3()  AFIO_REMAP_PARTIAL(AFIO_MAPR_CAN_REMAP_REMAP3, AFIO_MAPR_CAN_REMAP)

#endif


#define __HAL_AFIO_REMAP_PD01_ENABLE()  AFIO_REMAP_ENABLE(AFIO_MAPR_PD01_REMAP)


#define __HAL_AFIO_REMAP_PD01_DISABLE() AFIO_REMAP_DISABLE(AFIO_MAPR_PD01_REMAP)

#if defined(AFIO_MAPR_TIM5CH4_IREMAP)

#define __HAL_AFIO_REMAP_TIM5CH4_ENABLE()  AFIO_REMAP_ENABLE(AFIO_MAPR_TIM5CH4_IREMAP)


#define __HAL_AFIO_REMAP_TIM5CH4_DISABLE() AFIO_REMAP_DISABLE(AFIO_MAPR_TIM5CH4_IREMAP)
#endif

#if defined(AFIO_MAPR_ETH_REMAP)

#define __HAL_AFIO_REMAP_ETH_ENABLE()  AFIO_REMAP_ENABLE(AFIO_MAPR_ETH_REMAP)


#define __HAL_AFIO_REMAP_ETH_DISABLE() AFIO_REMAP_DISABLE(AFIO_MAPR_ETH_REMAP)
#endif

#if defined(AFIO_MAPR_CAN2_REMAP)


#define __HAL_AFIO_REMAP_CAN2_ENABLE()  AFIO_REMAP_ENABLE(AFIO_MAPR_CAN2_REMAP)


#define __HAL_AFIO_REMAP_CAN2_DISABLE() AFIO_REMAP_DISABLE(AFIO_MAPR_CAN2_REMAP)
#endif

#if defined(AFIO_MAPR_MII_RMII_SEL)

#define __HAL_AFIO_ETH_RMII() AFIO_REMAP_ENABLE(AFIO_MAPR_MII_RMII_SEL)


#define __HAL_AFIO_ETH_MII()  AFIO_REMAP_DISABLE(AFIO_MAPR_MII_RMII_SEL)
#endif


#define __HAL_AFIO_REMAP_ADC1_ETRGINJ_ENABLE()  AFIO_REMAP_ENABLE(AFIO_MAPR_ADC1_ETRGINJ_REMAP)


#define __HAL_AFIO_REMAP_ADC1_ETRGINJ_DISABLE() AFIO_REMAP_DISABLE(AFIO_MAPR_ADC1_ETRGINJ_REMAP)


#define __HAL_AFIO_REMAP_ADC1_ETRGREG_ENABLE()  AFIO_REMAP_ENABLE(AFIO_MAPR_ADC1_ETRGREG_REMAP)


#define __HAL_AFIO_REMAP_ADC1_ETRGREG_DISABLE() AFIO_REMAP_DISABLE(AFIO_MAPR_ADC1_ETRGREG_REMAP)

#if defined(AFIO_MAPR_ADC2_ETRGINJ_REMAP)


#define __HAL_AFIO_REMAP_ADC2_ETRGINJ_ENABLE()  AFIO_REMAP_ENABLE(AFIO_MAPR_ADC2_ETRGINJ_REMAP)


#define __HAL_AFIO_REMAP_ADC2_ETRGINJ_DISABLE() AFIO_REMAP_DISABLE(AFIO_MAPR_ADC2_ETRGINJ_REMAP)
#endif

#if defined (AFIO_MAPR_ADC2_ETRGREG_REMAP)


#define __HAL_AFIO_REMAP_ADC2_ETRGREG_ENABLE()  AFIO_REMAP_ENABLE(AFIO_MAPR_ADC2_ETRGREG_REMAP)


#define __HAL_AFIO_REMAP_ADC2_ETRGREG_DISABLE() AFIO_REMAP_DISABLE(AFIO_MAPR_ADC2_ETRGREG_REMAP)
#endif


#define __HAL_AFIO_REMAP_SWJ_ENABLE()  AFIO_DBGAFR_CONFIG(AFIO_MAPR_SWJ_CFG_RESET)


#define __HAL_AFIO_REMAP_SWJ_NONJTRST()  AFIO_DBGAFR_CONFIG(AFIO_MAPR_SWJ_CFG_NOJNTRST)



#define __HAL_AFIO_REMAP_SWJ_NOJTAG()  AFIO_DBGAFR_CONFIG(AFIO_MAPR_SWJ_CFG_JTAGDISABLE)


#define __HAL_AFIO_REMAP_SWJ_DISABLE()  AFIO_DBGAFR_CONFIG(AFIO_MAPR_SWJ_CFG_DISABLE)

#if defined(AFIO_MAPR_SPI3_REMAP)


#define __HAL_AFIO_REMAP_SPI3_ENABLE()  AFIO_REMAP_ENABLE(AFIO_MAPR_SPI3_REMAP)


#define __HAL_AFIO_REMAP_SPI3_DISABLE() AFIO_REMAP_DISABLE(AFIO_MAPR_SPI3_REMAP)
#endif

#if defined(AFIO_MAPR_TIM2ITR1_IREMAP)


#define __HAL_AFIO_TIM2ITR1_TO_USB() AFIO_REMAP_ENABLE(AFIO_MAPR_TIM2ITR1_IREMAP)


#define __HAL_AFIO_TIM2ITR1_TO_ETH() AFIO_REMAP_DISABLE(AFIO_MAPR_TIM2ITR1_IREMAP)
#endif

#if defined(AFIO_MAPR_PTP_PPS_REMAP)


#define __HAL_AFIO_ETH_PTP_PPS_ENABLE()  AFIO_REMAP_ENABLE(AFIO_MAPR_PTP_PPS_REMAP)


#define __HAL_AFIO_ETH_PTP_PPS_DISABLE() AFIO_REMAP_DISABLE(AFIO_MAPR_PTP_PPS_REMAP)
#endif

#if defined(AFIO_MAPR2_TIM9_REMAP)


#define __HAL_AFIO_REMAP_TIM9_ENABLE()  SET_BIT(AFIO->MAPR2, AFIO_MAPR2_TIM9_REMAP)


#define __HAL_AFIO_REMAP_TIM9_DISABLE() CLEAR_BIT(AFIO->MAPR2, AFIO_MAPR2_TIM9_REMAP)
#endif

#if defined(AFIO_MAPR2_TIM10_REMAP)


#define __HAL_AFIO_REMAP_TIM10_ENABLE()  SET_BIT(AFIO->MAPR2, AFIO_MAPR2_TIM10_REMAP)


#define __HAL_AFIO_REMAP_TIM10_DISABLE() CLEAR_BIT(AFIO->MAPR2, AFIO_MAPR2_TIM10_REMAP)
#endif

#if defined(AFIO_MAPR2_TIM11_REMAP)

#define __HAL_AFIO_REMAP_TIM11_ENABLE()  SET_BIT(AFIO->MAPR2, AFIO_MAPR2_TIM11_REMAP)


#define __HAL_AFIO_REMAP_TIM11_DISABLE() CLEAR_BIT(AFIO->MAPR2, AFIO_MAPR2_TIM11_REMAP)
#endif

#if defined(AFIO_MAPR2_TIM13_REMAP)


#define __HAL_AFIO_REMAP_TIM13_ENABLE()  SET_BIT(AFIO->MAPR2, AFIO_MAPR2_TIM13_REMAP)


#define __HAL_AFIO_REMAP_TIM13_DISABLE() CLEAR_BIT(AFIO->MAPR2, AFIO_MAPR2_TIM13_REMAP)
#endif

#if defined(AFIO_MAPR2_TIM14_REMAP)


#define __HAL_AFIO_REMAP_TIM14_ENABLE()  SET_BIT(AFIO->MAPR2, AFIO_MAPR2_TIM14_REMAP)


#define __HAL_AFIO_REMAP_TIM14_DISABLE() CLEAR_BIT(AFIO->MAPR2, AFIO_MAPR2_TIM14_REMAP)
#endif

#if defined(AFIO_MAPR2_FSMC_NADV_REMAP)


#define __HAL_AFIO_FSMCNADV_DISCONNECTED() SET_BIT(AFIO->MAPR2, AFIO_MAPR2_FSMC_NADV_REMAP)


#define __HAL_AFIO_FSMCNADV_CONNECTED()    CLEAR_BIT(AFIO->MAPR2, AFIO_MAPR2_FSMC_NADV_REMAP)
#endif

#if defined(AFIO_MAPR2_TIM15_REMAP)


#define __HAL_AFIO_REMAP_TIM15_ENABLE()  SET_BIT(AFIO->MAPR2, AFIO_MAPR2_TIM15_REMAP)


#define __HAL_AFIO_REMAP_TIM15_DISABLE() CLEAR_BIT(AFIO->MAPR2, AFIO_MAPR2_TIM15_REMAP)
#endif

#if defined(AFIO_MAPR2_TIM16_REMAP)


#define __HAL_AFIO_REMAP_TIM16_ENABLE()  SET_BIT(AFIO->MAPR2, AFIO_MAPR2_TIM16_REMAP)


#define __HAL_AFIO_REMAP_TIM16_DISABLE() CLEAR_BIT(AFIO->MAPR2, AFIO_MAPR2_TIM16_REMAP)
#endif

#if defined(AFIO_MAPR2_TIM17_REMAP)


#define __HAL_AFIO_REMAP_TIM17_ENABLE()  SET_BIT(AFIO->MAPR2, AFIO_MAPR2_TIM17_REMAP)


#define __HAL_AFIO_REMAP_TIM17_DISABLE() CLEAR_BIT(AFIO->MAPR2, AFIO_MAPR2_TIM17_REMAP)
#endif

#if defined(AFIO_MAPR2_CEC_REMAP)


#define __HAL_AFIO_REMAP_CEC_ENABLE()  SET_BIT(AFIO->MAPR2, AFIO_MAPR2_CEC_REMAP)


#define __HAL_AFIO_REMAP_CEC_DISABLE() CLEAR_BIT(AFIO->MAPR2, AFIO_MAPR2_CEC_REMAP)
#endif

#if defined(AFIO_MAPR2_TIM1_DMA_REMAP)


#define __HAL_AFIO_REMAP_TIM1DMA_ENABLE()  SET_BIT(AFIO->MAPR2, AFIO_MAPR2_TIM1_DMA_REMAP)


#define __HAL_AFIO_REMAP_TIM1DMA_DISABLE() CLEAR_BIT(AFIO->MAPR2, AFIO_MAPR2_TIM1_DMA_REMAP)
#endif

#if defined(AFIO_MAPR2_TIM67_DAC_DMA_REMAP)


#define __HAL_AFIO_REMAP_TIM67DACDMA_ENABLE()  SET_BIT(AFIO->MAPR2, AFIO_MAPR2_TIM67_DAC_DMA_REMAP)


#define __HAL_AFIO_REMAP_TIM67DACDMA_DISABLE() CLEAR_BIT(AFIO->MAPR2, AFIO_MAPR2_TIM67_DAC_DMA_REMAP)
#endif

#if defined(AFIO_MAPR2_TIM12_REMAP)


#define __HAL_AFIO_REMAP_TIM12_ENABLE()  SET_BIT(AFIO->MAPR2, AFIO_MAPR2_TIM12_REMAP)


#define __HAL_AFIO_REMAP_TIM12_DISABLE() CLEAR_BIT(AFIO->MAPR2, AFIO_MAPR2_TIM12_REMAP)
#endif

#if defined(AFIO_MAPR2_MISC_REMAP)


#define __HAL_AFIO_REMAP_MISC_ENABLE()  SET_BIT(AFIO->MAPR2, AFIO_MAPR2_MISC_REMAP)


#define __HAL_AFIO_REMAP_MISC_DISABLE() CLEAR_BIT(AFIO->MAPR2, AFIO_MAPR2_MISC_REMAP)
#endif






#if defined(STM32F101x6) || defined(STM32F102x6) || defined(STM32F102xB) || defined(STM32F103x6)
#define GPIO_GET_INDEX(__GPIOx__) (((__GPIOx__) == (GPIOA))? 0uL :\
                                   ((__GPIOx__) == (GPIOB))? 1uL :\
                                   ((__GPIOx__) == (GPIOC))? 2uL :3uL)
#elif defined(STM32F100xB) || defined(STM32F101xB) || defined(STM32F103xB) || defined(STM32F105xC) || defined(STM32F107xC)
#define GPIO_GET_INDEX(__GPIOx__) (((__GPIOx__) == (GPIOA))? 0uL :\
                                   ((__GPIOx__) == (GPIOB))? 1uL :\
                                   ((__GPIOx__) == (GPIOC))? 2uL :\
                                   ((__GPIOx__) == (GPIOD))? 3uL :4uL)
#elif defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F103xE) || defined(STM32F103xG)
#define GPIO_GET_INDEX(__GPIOx__) (((__GPIOx__) == (GPIOA))? 0uL :\
                                   ((__GPIOx__) == (GPIOB))? 1uL :\
                                   ((__GPIOx__) == (GPIOC))? 2uL :\
                                   ((__GPIOx__) == (GPIOD))? 3uL :\
                                   ((__GPIOx__) == (GPIOE))? 4uL :\
                                   ((__GPIOx__) == (GPIOF))? 5uL :6uL)
#endif

#define AFIO_REMAP_ENABLE(REMAP_PIN)       do{ uint32_t tmpreg = AFIO->MAPR; \
                                               tmpreg |= AFIO_MAPR_SWJ_CFG;  \
                                               tmpreg |= REMAP_PIN;          \
                                               AFIO->MAPR = tmpreg;          \
                                               }while(0u)

#define AFIO_REMAP_DISABLE(REMAP_PIN)      do{ uint32_t tmpreg = AFIO->MAPR;  \
                                               tmpreg |= AFIO_MAPR_SWJ_CFG;   \
                                               tmpreg &= ~REMAP_PIN;          \
                                               AFIO->MAPR = tmpreg;           \
                                               }while(0u)

#define AFIO_REMAP_PARTIAL(REMAP_PIN, REMAP_PIN_MASK) do{ uint32_t tmpreg = AFIO->MAPR; \
                                                          tmpreg &= ~REMAP_PIN_MASK;    \
                                                          tmpreg |= AFIO_MAPR_SWJ_CFG;  \
                                                          tmpreg |= REMAP_PIN;          \
                                                          AFIO->MAPR = tmpreg;          \
                                                          }while(0u)

#define AFIO_DBGAFR_CONFIG(DBGAFR_SWJCFG)  do{ uint32_t tmpreg = AFIO->MAPR;     \
                                               tmpreg &= ~AFIO_MAPR_SWJ_CFG_Msk; \
                                               tmpreg |= DBGAFR_SWJCFG;          \
                                               AFIO->MAPR = tmpreg;              \
                                               }while(0u)



/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/




void HAL_GPIOEx_ConfigEventout(uint32_t GPIO_PortSource, uint32_t GPIO_PinSource);
void HAL_GPIOEx_EnableEventout(void);
void HAL_GPIOEx_DisableEventout(void);











#endif /* STM32F1xx_HAL_GPIO_EX_H */

