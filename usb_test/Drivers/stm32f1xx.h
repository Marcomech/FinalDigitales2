#ifndef __STM32F1xx_HAL_H
#define __STM32F1xx_HAL_H

#define HAL_MODULE_ENABLED
#define HAL_PCD_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED


#if !defined  (HSE_VALUE)
  #define HSE_VALUE    8000000U /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (HSE_STARTUP_TIMEOUT)
  #define HSE_STARTUP_TIMEOUT    100U   /*!< Time out for HSE start up, in ms */
#endif /* HSE_STARTUP_TIMEOUT */


#if !defined  (HSI_VALUE)
  #define HSI_VALUE    8000000U /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/**
  * @brief Internal Low Speed oscillator (LSI) value.
  */
#if !defined  (LSI_VALUE)
 #define LSI_VALUE               40000U    /*!< LSI Typical Value in Hz */
#endif /* LSI_VALUE */                     /*!< Value of the Internal Low Speed oscillator in Hz
                                                The real value may vary depending on the variations
                                                in voltage and temperature. */

#if !defined  (LSE_VALUE)
  #define LSE_VALUE    32768U /*!< Value of the External oscillator in Hz*/
#endif /* LSE_VALUE */

#if !defined  (LSE_STARTUP_TIMEOUT)
  #define LSE_STARTUP_TIMEOUT    5000U   /*!< Time out for LSE start up, in ms */
#endif /* LSE_STARTUP_TIMEOUT */

/* Tip: To avoid modifying this file each time you need to use different HSE,
   ===  you can define the HSE value in your toolchain compiler preprocessor. */

/* ########################### System Configuration ######################### */
/**
  * @brief This is the HAL system configuration section
  */
#define  VDD_VALUE                    3300U /*!< Value of VDD in mv */
#define  TICK_INT_PRIORITY            0U    /*!< tick interrupt priority (lowest by default)  */
#define  USE_RTOS                     0U
#define  PREFETCH_ENABLE              1U

#define  USE_HAL_ADC_REGISTER_CALLBACKS         0U /* ADC register callback disabled       */
#define  USE_HAL_CAN_REGISTER_CALLBACKS         0U /* CAN register callback disabled       */
#define  USE_HAL_CEC_REGISTER_CALLBACKS         0U /* CEC register callback disabled       */
#define  USE_HAL_DAC_REGISTER_CALLBACKS         0U /* DAC register callback disabled       */
#define  USE_HAL_ETH_REGISTER_CALLBACKS         0U /* ETH register callback disabled       */
#define  USE_HAL_HCD_REGISTER_CALLBACKS         0U /* HCD register callback disabled       */
#define  USE_HAL_I2C_REGISTER_CALLBACKS         0U /* I2C register callback disabled       */
#define  USE_HAL_I2S_REGISTER_CALLBACKS         0U /* I2S register callback disabled       */
#define  USE_HAL_MMC_REGISTER_CALLBACKS         0U /* MMC register callback disabled       */
#define  USE_HAL_NAND_REGISTER_CALLBACKS        0U /* NAND register callback disabled      */
#define  USE_HAL_NOR_REGISTER_CALLBACKS         0U /* NOR register callback disabled       */
#define  USE_HAL_PCCARD_REGISTER_CALLBACKS      0U /* PCCARD register callback disabled    */
#define  USE_HAL_PCD_REGISTER_CALLBACKS         0U /* PCD register callback disabled       */
#define  USE_HAL_RTC_REGISTER_CALLBACKS         0U /* RTC register callback disabled       */
#define  USE_HAL_SD_REGISTER_CALLBACKS          0U /* SD register callback disabled        */
#define  USE_HAL_SMARTCARD_REGISTER_CALLBACKS   0U /* SMARTCARD register callback disabled */
#define  USE_HAL_IRDA_REGISTER_CALLBACKS        0U /* IRDA register callback disabled      */
#define  USE_HAL_SRAM_REGISTER_CALLBACKS        0U /* SRAM register callback disabled      */
#define  USE_HAL_SPI_REGISTER_CALLBACKS         0U /* SPI register callback disabled       */
#define  USE_HAL_TIM_REGISTER_CALLBACKS         0U /* TIM register callback disabled       */
#define  USE_HAL_UART_REGISTER_CALLBACKS        0U /* UART register callback disabled      */
#define  USE_HAL_USART_REGISTER_CALLBACKS       0U /* USART register callback disabled     */
#define  USE_HAL_WWDG_REGISTER_CALLBACKS        0U /* WWDG register callback disabled      */



#define MAC_ADDR0   2U
#define MAC_ADDR1   0U
#define MAC_ADDR2   0U
#define MAC_ADDR3   0U
#define MAC_ADDR4   0U
#define MAC_ADDR5   0U

/* Definition of the Ethernet driver buffers size and count */
#define ETH_RX_BUF_SIZE                ETH_MAX_PACKET_SIZE /* buffer size for receive               */
#define ETH_TX_BUF_SIZE                ETH_MAX_PACKET_SIZE /* buffer size for transmit              */
#define ETH_RXBUFNB                    8U       /* 4 Rx buffers of size ETH_RX_BUF_SIZE  */
#define ETH_TXBUFNB                    4U       /* 4 Tx buffers of size ETH_TX_BUF_SIZE  */

/* Section 2: PHY configuration section */

/* DP83848_PHY_ADDRESS Address*/
#define DP83848_PHY_ADDRESS           0x01U
/* PHY Reset delay these values are based on a 1 ms Systick interrupt*/
#define PHY_RESET_DELAY                 0x000000FFU
/* PHY Configuration delay */
#define PHY_CONFIG_DELAY                0x00000FFFU

#define PHY_READ_TO                     0x0000FFFFU
#define PHY_WRITE_TO                    0x0000FFFFU

/* Section 3: Common PHY Registers */

#define PHY_BCR                         ((uint16_t)0x00)    /*!< Transceiver Basic Control Register   */
#define PHY_BSR                         ((uint16_t)0x01)    /*!< Transceiver Basic Status Register    */

#define PHY_RESET                       ((uint16_t)0x8000)  /*!< PHY Reset */
#define PHY_LOOPBACK                    ((uint16_t)0x4000)  /*!< Select loop-back mode */
#define PHY_FULLDUPLEX_100M             ((uint16_t)0x2100)  /*!< Set the full-duplex mode at 100 Mb/s */
#define PHY_HALFDUPLEX_100M             ((uint16_t)0x2000)  /*!< Set the half-duplex mode at 100 Mb/s */
#define PHY_FULLDUPLEX_10M              ((uint16_t)0x0100)  /*!< Set the full-duplex mode at 10 Mb/s  */
#define PHY_HALFDUPLEX_10M              ((uint16_t)0x0000)  /*!< Set the half-duplex mode at 10 Mb/s  */
#define PHY_AUTONEGOTIATION             ((uint16_t)0x1000)  /*!< Enable auto-negotiation function     */
#define PHY_RESTART_AUTONEGOTIATION     ((uint16_t)0x0200)  /*!< Restart auto-negotiation function    */
#define PHY_POWERDOWN                   ((uint16_t)0x0800)  /*!< Select the power down mode           */
#define PHY_ISOLATE                     ((uint16_t)0x0400)  /*!< Isolate PHY from MII                 */

#define PHY_AUTONEGO_COMPLETE           ((uint16_t)0x0020)  /*!< Auto-Negotiation process completed   */
#define PHY_LINKED_STATUS               ((uint16_t)0x0004)  /*!< Valid link established               */
#define PHY_JABBER_DETECTION            ((uint16_t)0x0002)  /*!< Jabber condition detected            */

/* Section 4: Extended PHY Registers */
#define PHY_SR                          ((uint16_t)0x10U)    /*!< PHY status register Offset                      */

#define PHY_SPEED_STATUS                ((uint16_t)0x0002U)  /*!< PHY Speed mask                                  */
#define PHY_DUPLEX_STATUS               ((uint16_t)0x0004U)  /*!< PHY Duplex mask                                 */

#define USE_SPI_CRC                     0U


#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32f1xx_hal_rcc.h"
#endif /* HAL_RCC_MODULE_ENABLED */

#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32f1xx_hal_gpio.h"
#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_EXTI_MODULE_ENABLED
#include "stm32f1xx_hal_exti.h"
#endif /* HAL_EXTI_MODULE_ENABLED */

#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32f1xx_hal_dma.h"
#endif /* HAL_DMA_MODULE_ENABLED */

#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32f1xx_hal_cortex.h"
#endif /* HAL_CORTEX_MODULE_ENABLED */

#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32f1xx_hal_flash.h"
#endif /* HAL_FLASH_MODULE_ENABLED */

#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32f1xx_hal_pwr.h"
#endif /* HAL_PWR_MODULE_ENABLED */

#ifdef HAL_PCD_MODULE_ENABLED
#include "stm32f1xx_hal_pcd.h"
#endif /* HAL_PCD_MODULE_ENABLED */


/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT


#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
void assert_failed(uint8_t* file, uint32_t line);
#else
#define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */




typedef enum{
  HAL_TICK_FREQ_10HZ         = 100U,
  HAL_TICK_FREQ_100HZ        = 10U,
  HAL_TICK_FREQ_1KHZ         = 1U,
  HAL_TICK_FREQ_DEFAULT      = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;

extern __IO uint32_t uwTick;
extern uint32_t uwTickPrio;
extern HAL_TickFreqTypeDef uwTickFreq;


#define __HAL_DBGMCU_FREEZE_TIM2()            SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM2_STOP)
#define __HAL_DBGMCU_UNFREEZE_TIM2()          CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM2_STOP)


#define __HAL_DBGMCU_FREEZE_TIM3()            SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM3_STOP)
#define __HAL_DBGMCU_UNFREEZE_TIM3()          CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM3_STOP)

#if defined (DBGMCU_CR_DBG_TIM4_STOP)


#define __HAL_DBGMCU_FREEZE_TIM4()            SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM4_STOP)
#define __HAL_DBGMCU_UNFREEZE_TIM4()          CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM4_STOP)
#endif

#if defined (DBGMCU_CR_DBG_TIM5_STOP)


#define __HAL_DBGMCU_FREEZE_TIM5()            SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM5_STOP)
#define __HAL_DBGMCU_UNFREEZE_TIM5()          CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM5_STOP)
#endif

#if defined (DBGMCU_CR_DBG_TIM6_STOP)


#define __HAL_DBGMCU_FREEZE_TIM6()            SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM6_STOP)
#define __HAL_DBGMCU_UNFREEZE_TIM6()          CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM6_STOP)
#endif

#if defined (DBGMCU_CR_DBG_TIM7_STOP)


#define __HAL_DBGMCU_FREEZE_TIM7()            SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM7_STOP)
#define __HAL_DBGMCU_UNFREEZE_TIM7()          CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM7_STOP)
#endif

#if defined (DBGMCU_CR_DBG_TIM12_STOP)


#define __HAL_DBGMCU_FREEZE_TIM12()            SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM12_STOP)
#define __HAL_DBGMCU_UNFREEZE_TIM12()          CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM12_STOP)
#endif

#if defined (DBGMCU_CR_DBG_TIM13_STOP)


#define __HAL_DBGMCU_FREEZE_TIM13()            SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM13_STOP)
#define __HAL_DBGMCU_UNFREEZE_TIM13()          CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM13_STOP)
#endif

#if defined (DBGMCU_CR_DBG_TIM14_STOP)


#define __HAL_DBGMCU_FREEZE_TIM14()            SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM14_STOP)
#define __HAL_DBGMCU_UNFREEZE_TIM14()          CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM14_STOP)
#endif


#define __HAL_DBGMCU_FREEZE_WWDG()            SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_WWDG_STOP)
#define __HAL_DBGMCU_UNFREEZE_WWDG()          CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_WWDG_STOP)

/**
  * @brief  IWDG Peripherals Debug mode
  */
#define __HAL_DBGMCU_FREEZE_IWDG()            SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_IWDG_STOP)
#define __HAL_DBGMCU_UNFREEZE_IWDG()          CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_IWDG_STOP)

/**
  * @brief  I2C1 Peripherals Debug mode
  */
#define __HAL_DBGMCU_FREEZE_I2C1_TIMEOUT()    SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_I2C1_SMBUS_TIMEOUT)
#define __HAL_DBGMCU_UNFREEZE_I2C1_TIMEOUT()  CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_I2C1_SMBUS_TIMEOUT)

#if defined (DBGMCU_CR_DBG_I2C2_SMBUS_TIMEOUT)
/**
  * @brief  I2C2 Peripherals Debug mode
  */
#define __HAL_DBGMCU_FREEZE_I2C2_TIMEOUT()    SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_I2C2_SMBUS_TIMEOUT)
#define __HAL_DBGMCU_UNFREEZE_I2C2_TIMEOUT()  CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_I2C2_SMBUS_TIMEOUT)
#endif

#if defined (DBGMCU_CR_DBG_CAN1_STOP)
/**
  * @brief  CAN1 Peripherals Debug mode
  */
#define __HAL_DBGMCU_FREEZE_CAN1()            SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_CAN1_STOP)
#define __HAL_DBGMCU_UNFREEZE_CAN1()          CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_CAN1_STOP)
#endif

#if defined (DBGMCU_CR_DBG_CAN2_STOP)
/**
  * @brief  CAN2 Peripherals Debug mode
  */
#define __HAL_DBGMCU_FREEZE_CAN2()            SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_CAN2_STOP)
#define __HAL_DBGMCU_UNFREEZE_CAN2()          CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_CAN2_STOP)
#endif

/* Peripherals on APB2 */
#if defined (DBGMCU_CR_DBG_TIM1_STOP)
/**
  * @brief  TIM1 Peripherals Debug mode
  */
#define __HAL_DBGMCU_FREEZE_TIM1()            SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM1_STOP)
#define __HAL_DBGMCU_UNFREEZE_TIM1()          CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM1_STOP)
#endif

#if defined (DBGMCU_CR_DBG_TIM8_STOP)
/**
  * @brief  TIM8 Peripherals Debug mode
  */
#define __HAL_DBGMCU_FREEZE_TIM8()            SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM8_STOP)
#define __HAL_DBGMCU_UNFREEZE_TIM8()          CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM8_STOP)
#endif

#if defined (DBGMCU_CR_DBG_TIM9_STOP)
/**
  * @brief  TIM9 Peripherals Debug mode
  */
#define __HAL_DBGMCU_FREEZE_TIM9()            SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM9_STOP)
#define __HAL_DBGMCU_UNFREEZE_TIM9()          CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM9_STOP)
#endif

#if defined (DBGMCU_CR_DBG_TIM10_STOP)
/**
  * @brief  TIM10 Peripherals Debug mode
  */
#define __HAL_DBGMCU_FREEZE_TIM10()            SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM10_STOP)
#define __HAL_DBGMCU_UNFREEZE_TIM10()          CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM10_STOP)
#endif

#if defined (DBGMCU_CR_DBG_TIM11_STOP)
/**
  * @brief  TIM11 Peripherals Debug mode
  */
#define __HAL_DBGMCU_FREEZE_TIM11()            SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM11_STOP)
#define __HAL_DBGMCU_UNFREEZE_TIM11()          CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM11_STOP)
#endif


#if defined (DBGMCU_CR_DBG_TIM15_STOP)
/**
  * @brief  TIM15 Peripherals Debug mode
  */
#define __HAL_DBGMCU_FREEZE_TIM15()            SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM15_STOP)
#define __HAL_DBGMCU_UNFREEZE_TIM15()          CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM15_STOP)
#endif

#if defined (DBGMCU_CR_DBG_TIM16_STOP)


#define __HAL_DBGMCU_FREEZE_TIM16()            SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM16_STOP)
#define __HAL_DBGMCU_UNFREEZE_TIM16()          CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM16_STOP)
#endif

#if defined (DBGMCU_CR_DBG_TIM17_STOP)


#define __HAL_DBGMCU_FREEZE_TIM17()            SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM17_STOP)
#define __HAL_DBGMCU_UNFREEZE_TIM17()          CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM17_STOP)
#endif


#define IS_TICKFREQ(FREQ) (((FREQ) == HAL_TICK_FREQ_10HZ)  || \
                           ((FREQ) == HAL_TICK_FREQ_100HZ) || \
                           ((FREQ) == HAL_TICK_FREQ_1KHZ))


HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_DeInit(void);
void HAL_MspInit(void);
void HAL_MspDeInit(void);
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);

void HAL_IncTick(void);
void HAL_Delay(uint32_t Delay);
uint32_t HAL_GetTick(void);
uint32_t HAL_GetTickPrio(void);
HAL_StatusTypeDef HAL_SetTickFreq(HAL_TickFreqTypeDef Freq);
HAL_TickFreqTypeDef HAL_GetTickFreq(void);
void HAL_SuspendTick(void);
void HAL_ResumeTick(void);
uint32_t HAL_GetHalVersion(void);
uint32_t HAL_GetREVID(void);
uint32_t HAL_GetDEVID(void);
uint32_t HAL_GetUIDw0(void);
uint32_t HAL_GetUIDw1(void);
uint32_t HAL_GetUIDw2(void);
void HAL_DBGMCU_EnableDBGSleepMode(void);
void HAL_DBGMCU_DisableDBGSleepMode(void);
void HAL_DBGMCU_EnableDBGStopMode(void);
void HAL_DBGMCU_DisableDBGStopMode(void);
void HAL_DBGMCU_EnableDBGStandbyMode(void);
void HAL_DBGMCU_DisableDBGStandbyMode(void);


#endif /* __STM32F1xx_HAL_H */




#ifndef __STM32F1XX_H
#define __STM32F1XX_H

#if !defined (STM32F1)
#define STM32F1
#endif /* STM32F1 */


#if !defined (STM32F100xB) && !defined (STM32F100xE) && !defined (STM32F101x6) && \
    !defined (STM32F101xB) && !defined (STM32F101xE) && !defined (STM32F101xG) && !defined (STM32F102x6) && !defined (STM32F102xB) && !defined (STM32F103x6) && \
    !defined (STM32F103xB) && !defined (STM32F103xE) && !defined (STM32F103xG) && !defined (STM32F105xC) && !defined (STM32F107xC)
#endif

  
#if !defined  (USE_HAL_DRIVER)

#endif /* USE_HAL_DRIVER */

#define __STM32F1_CMSIS_VERSION_MAIN   (0x04) /*!< [31:24] main version */
#define __STM32F1_CMSIS_VERSION_SUB1   (0x03) /*!< [23:16] sub1 version */
#define __STM32F1_CMSIS_VERSION_SUB2   (0x04) /*!< [15:8]  sub2 version */
#define __STM32F1_CMSIS_VERSION_RC     (0x00) /*!< [7:0]  release candidate */ 
#define __STM32F1_CMSIS_VERSION        ((__STM32F1_CMSIS_VERSION_MAIN << 24)\
                                       |(__STM32F1_CMSIS_VERSION_SUB1 << 16)\
                                       |(__STM32F1_CMSIS_VERSION_SUB2 << 8 )\
                                       |(__STM32F1_CMSIS_VERSION_RC))


#if defined(STM32F103xB)
  #include "stm32f103xb.h"
#else
 #error "Please select first the target STM32F1xx device used in your application (in stm32f1xx.h file)"
#endif

typedef enum {
  RESET = 0, 
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum {
  DISABLE = 0, 
  ENABLE = !DISABLE
} FunctionalState;

#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum{
  SUCCESS = 0U,
  ERROR = !SUCCESS
} ErrorStatus;


#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL))) 

/* Use of CMSIS compiler intrinsics for register exclusive access */
/* Atomic 32-bit register access macro to set one or several bits */
#define ATOMIC_SET_BIT(REG, BIT)                             \
  do {                                                       \
    uint32_t val;                                            \
    do {                                                     \
      val = __LDREXW((__IO uint32_t *)&(REG)) | (BIT);       \
    } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U); \
  } while(0)

/* Atomic 32-bit register access macro to clear one or several bits */
#define ATOMIC_CLEAR_BIT(REG, BIT)                           \
  do {                                                       \
    uint32_t val;                                            \
    do {                                                     \
      val = __LDREXW((__IO uint32_t *)&(REG)) & ~(BIT);      \
    } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U); \
  } while(0)

/* Atomic 32-bit register access macro to clear and set one or several bits */
#define ATOMIC_MODIFY_REG(REG, CLEARMSK, SETMASK)                          \
  do {                                                                     \
    uint32_t val;                                                          \
    do {                                                                   \
      val = (__LDREXW((__IO uint32_t *)&(REG)) & ~(CLEARMSK)) | (SETMASK); \
    } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U);               \
  } while(0)

/* Atomic 16-bit register access macro to set one or several bits */
#define ATOMIC_SETH_BIT(REG, BIT)                            \
  do {                                                       \
    uint16_t val;                                            \
    do {                                                     \
      val = __LDREXH((__IO uint16_t *)&(REG)) | (BIT);       \
    } while ((__STREXH(val,(__IO uint16_t *)&(REG))) != 0U); \
  } while(0)

/* Atomic 16-bit register access macro to clear one or several bits */
#define ATOMIC_CLEARH_BIT(REG, BIT)                          \
  do {                                                       \
    uint16_t val;                                            \
    do {                                                     \
      val = __LDREXH((__IO uint16_t *)&(REG)) & ~(BIT);      \
    } while ((__STREXH(val,(__IO uint16_t *)&(REG))) != 0U); \
  } while(0)

/* Atomic 16-bit register access macro to clear and set one or several bits */
#define ATOMIC_MODIFYH_REG(REG, CLEARMSK, SETMASK)                         \
  do {                                                                     \
    uint16_t val;                                                          \
    do {                                                                   \
      val = (__LDREXH((__IO uint16_t *)&(REG)) & ~(CLEARMSK)) | (SETMASK); \
    } while ((__STREXH(val,(__IO uint16_t *)&(REG))) != 0U);               \
  } while(0)


#endif /* __STM32F1xx_H */

