#ifndef __STM32F1xx_HAL_H
#define __STM32F1xx_HAL_H

typedef __INT8_TYPE__ int8_t;
typedef __INT16_TYPE__ int16_t;
typedef __INT32_TYPE__ int32_t;
typedef __INT64_TYPE__ int64_t;
typedef __UINT8_TYPE__ uint8_t;
typedef __UINT16_TYPE__ uint16_t;
typedef __UINT32_TYPE__ uint32_t;
typedef __UINT64_TYPE__ uint64_t;

#define __I volatile const  /*!< Defines 'read only' permissions */
#define __O volatile        /*!< Defines 'write only' permissions */
#define __IO volatile       /*!< Defines 'read / write' permissions */
#define __IM volatile const /*! Defines 'read only' structure member permissions */
#define __OM volatile       /*! Defines 'write only' structure member permissions */
#define __IOM volatile      /*! Defines 'read / write' structure member permissions */

#define HAL_MODULE_ENABLED
#define HAL_PCD_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED

















#ifndef Clock_Config
#define Clock_Config

#define RCC_HSE_PREDIV_DIV1             0x00000000U
#define RCC_PLL_MUL9                    (0x7UL << 18U)
#define RCC_PERIPHCLK_USB               0x00000010U
#define RCC_USBCLKSOURCE_PLL_DIV1_5     0x00000000U

#define FLASH_LATENCY_0            0x00000000U               /*!< FLASH Zero Latency cycle */
#define FLASH_LATENCY_1            0x00000001                /*!< FLASH One Latency cycle */
#define FLASH_LATENCY_2            0x00000002                /*!< FLASH Two Latency cycles */

typedef struct{
  uint32_t PLLState;
  uint32_t PLLSource;
  uint32_t PLLMUL;
} RCC_PLLInitTypeDef;

typedef struct{
  uint32_t OscillatorType;
  uint32_t HSEState;
  uint32_t HSEPredivValue;
  uint32_t LSEState;
  uint32_t HSIState;
  uint32_t HSICalibrationValue;
  uint32_t LSIState;
  RCC_PLLInitTypeDef PLL;
} RCC_OscInitTypeDef;

typedef struct{
  uint32_t PeriphClockSelection;
  uint32_t RTCClockSelection;
  uint32_t AdcClockSelection;
  uint32_t UsbClockSelection;
} RCC_PeriphCLKInitTypeDef;

typedef struct{
  uint32_t ClockType;
  uint32_t SYSCLKSource;
  uint32_t AHBCLKDivider;
  uint32_t APB1CLKDivider;
  uint32_t APB2CLKDivider;
} RCC_ClkInitTypeDef;

#define RCC_HSI_ON                       RCC_CR_HSION           /*!< HSI clock activation */
#define RCC_OSCILLATORTYPE_HSE           0x00000001U
#define RCC_HSE_ON                       RCC_CR_HSEON                /*!< HSE clock activation */
#define RCC_PLL_ON                       0x00000002U /*!< PLL activation */
#define RCC_PLLSOURCE_HSE                RCC_CFGR_PLLSRC      /*!< HSE clock selected as PLL entry clock source */
#define RCC_CLOCKTYPE_SYSCLK             0x00000001U /*!< SYSCLK to configure */
#define RCC_CLOCKTYPE_HCLK               0x00000002U /*!< HCLK to configure */
#define RCC_CLOCKTYPE_PCLK1              0x00000004U /*!< PCLK1 to configure */
#define RCC_CLOCKTYPE_PCLK2              0x00000008U /*!< PCLK2 to configure */
#define RCC_SYSCLKSOURCE_PLLCLK          0x00000002U /*!< PLL selected as system clock */
#define RCC_HCLK_DIV1                    0x00000000U  /*!< HCLK not divided */
#define RCC_HCLK_DIV2                    0x00000400U  /*!< HCLK divided by 2 */
#define RCC_SYSCLK_DIV1                  0x00000000U  /*!< SYSCLK not divided */




#endif //Clock_Config











#define HSE_VALUE 8000000U        /*!< Value of the External oscillator in Hz */
#define HSE_STARTUP_TIMEOUT 100U  /*!< Time out for HSE start up, in ms */
#define HSI_VALUE 8000000U        /*!< Value of the Internal oscillator in Hz*/
#define LSI_VALUE 40000U          /*!< LSI Typical Value in Hz */
#define LSE_VALUE 32768U          /*!< Value of the External oscillator in Hz*/
#define LSE_STARTUP_TIMEOUT 5000U /*!< Time out for LSE start up, in ms */

/* ########################### System Configuration ######################### */

#define VDD_VALUE 3300U      /*!< Value of VDD in mv */
#define TICK_INT_PRIORITY 0U /*!< tick interrupt priority (lowest by default)  */
#define USE_RTOS 0U
#define PREFETCH_ENABLE 1U

#define USE_HAL_ADC_REGISTER_CALLBACKS 0U       /* ADC register callback disabled       */
#define USE_HAL_CAN_REGISTER_CALLBACKS 0U       /* CAN register callback disabled       */
#define USE_HAL_CEC_REGISTER_CALLBACKS 0U       /* CEC register callback disabled       */
#define USE_HAL_DAC_REGISTER_CALLBACKS 0U       /* DAC register callback disabled       */
#define USE_HAL_ETH_REGISTER_CALLBACKS 0U       /* ETH register callback disabled       */
#define USE_HAL_HCD_REGISTER_CALLBACKS 0U       /* HCD register callback disabled       */
#define USE_HAL_I2C_REGISTER_CALLBACKS 0U       /* I2C register callback disabled       */
#define USE_HAL_I2S_REGISTER_CALLBACKS 0U       /* I2S register callback disabled       */
#define USE_HAL_MMC_REGISTER_CALLBACKS 0U       /* MMC register callback disabled       */
#define USE_HAL_NAND_REGISTER_CALLBACKS 0U      /* NAND register callback disabled      */
#define USE_HAL_NOR_REGISTER_CALLBACKS 0U       /* NOR register callback disabled       */
#define USE_HAL_PCCARD_REGISTER_CALLBACKS 0U    /* PCCARD register callback disabled    */
#define USE_HAL_PCD_REGISTER_CALLBACKS 0U       /* PCD register callback disabled       */
#define USE_HAL_RTC_REGISTER_CALLBACKS 0U       /* RTC register callback disabled       */
#define USE_HAL_SD_REGISTER_CALLBACKS 0U        /* SD register callback disabled        */
#define USE_HAL_SMARTCARD_REGISTER_CALLBACKS 0U /* SMARTCARD register callback disabled */
#define USE_HAL_IRDA_REGISTER_CALLBACKS 0U      /* IRDA register callback disabled      */
#define USE_HAL_SRAM_REGISTER_CALLBACKS 0U      /* SRAM register callback disabled      */
#define USE_HAL_SPI_REGISTER_CALLBACKS 0U       /* SPI register callback disabled       */
#define USE_HAL_TIM_REGISTER_CALLBACKS 0U       /* TIM register callback disabled       */
#define USE_HAL_UART_REGISTER_CALLBACKS 0U      /* UART register callback disabled      */
#define USE_HAL_USART_REGISTER_CALLBACKS 0U     /* USART register callback disabled     */
#define USE_HAL_WWDG_REGISTER_CALLBACKS 0U      /* WWDG register callback disabled      */

#define MAC_ADDR0 2U
#define MAC_ADDR1 0U
#define MAC_ADDR2 0U
#define MAC_ADDR3 0U
#define MAC_ADDR4 0U
#define MAC_ADDR5 0U

/* Definition of the Ethernet driver buffers size and count */
#define ETH_RX_BUF_SIZE ETH_MAX_PACKET_SIZE /* buffer size for receive               */
#define ETH_TX_BUF_SIZE ETH_MAX_PACKET_SIZE /* buffer size for transmit              */
#define ETH_RXBUFNB 8U                      /* 4 Rx buffers of size ETH_RX_BUF_SIZE  */
#define ETH_TXBUFNB 4U                      /* 4 Tx buffers of size ETH_TX_BUF_SIZE  */

/* Section 2: PHY configuration section */

/* DP83848_PHY_ADDRESS Address*/
#define DP83848_PHY_ADDRESS 0x01U
/* PHY Reset delay these values are based on a 1 ms Systick interrupt*/
#define PHY_RESET_DELAY 0x000000FFU
/* PHY Configuration delay */
#define PHY_CONFIG_DELAY 0x00000FFFU

#define PHY_READ_TO 0x0000FFFFU
#define PHY_WRITE_TO 0x0000FFFFU

/* Section 3: Common PHY Registers */

#define PHY_BCR ((uint16_t)0x00) /*!< Transceiver Basic Control Register   */
#define PHY_BSR ((uint16_t)0x01) /*!< Transceiver Basic Status Register    */

#define PHY_RESET ((uint16_t)0x8000)                   /*!< PHY Reset */
#define PHY_LOOPBACK ((uint16_t)0x4000)                /*!< Select loop-back mode */
#define PHY_FULLDUPLEX_100M ((uint16_t)0x2100)         /*!< Set the full-duplex mode at 100 Mb/s */
#define PHY_HALFDUPLEX_100M ((uint16_t)0x2000)         /*!< Set the half-duplex mode at 100 Mb/s */
#define PHY_FULLDUPLEX_10M ((uint16_t)0x0100)          /*!< Set the full-duplex mode at 10 Mb/s  */
#define PHY_HALFDUPLEX_10M ((uint16_t)0x0000)          /*!< Set the half-duplex mode at 10 Mb/s  */
#define PHY_AUTONEGOTIATION ((uint16_t)0x1000)         /*!< Enable auto-negotiation function     */
#define PHY_RESTART_AUTONEGOTIATION ((uint16_t)0x0200) /*!< Restart auto-negotiation function    */
#define PHY_POWERDOWN ((uint16_t)0x0800)               /*!< Select the power down mode           */
#define PHY_ISOLATE ((uint16_t)0x0400)                 /*!< Isolate PHY from MII                 */

#define PHY_AUTONEGO_COMPLETE ((uint16_t)0x0020) /*!< Auto-Negotiation process completed   */
#define PHY_LINKED_STATUS ((uint16_t)0x0004)     /*!< Valid link established               */
#define PHY_JABBER_DETECTION ((uint16_t)0x0002)  /*!< Jabber condition detected            */

/* Section 4: Extended PHY Registers */
#define PHY_SR ((uint16_t)0x10U) /*!< PHY status register Offset                      */

#define PHY_SPEED_STATUS ((uint16_t)0x0002U)  /*!< PHY Speed mask                                  */
#define PHY_DUPLEX_STATUS ((uint16_t)0x0004U) /*!< PHY Duplex mask                                 */

#define USE_SPI_CRC 0U

//#include "../Inc_Clock_Config.h"

typedef enum
{
  HAL_OK = 0x00U,
  HAL_ERROR = 0x01U,
  HAL_BUSY = 0x02U,
  HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

typedef enum
{
  HAL_UNLOCKED = 0x00U,
  HAL_LOCKED = 0x01U
} HAL_LockTypeDef;

#define HAL_MAX_DELAY 0xFFFFFFFFU
#define HAL_IS_BIT_SET(REG, BIT) (((REG) & (BIT)) != 0U)
#define HAL_IS_BIT_CLR(REG, BIT) (((REG) & (BIT)) == 0U)

#define UNUSED(X) (void)X /* To avoid gcc/g++ warnings */

#define __HAL_LOCK(__HANDLE__)            \
  do                                      \
  {                                       \
    if ((__HANDLE__)->Lock == HAL_LOCKED) \
    {                                     \
      return HAL_BUSY;                    \
    }                                     \
    else                                  \
    {                                     \
      (__HANDLE__)->Lock = HAL_LOCKED;    \
    }                                     \
  } while (0U)

#define __HAL_UNLOCK(__HANDLE__)       \
  do                                   \
  {                                    \
    (__HANDLE__)->Lock = HAL_UNLOCKED; \
  } while (0U)

#define __weak __attribute__((weak))
#define __packed __attribute__((__packed__))
#define __ALIGN_BEGIN
#define __ALIGN_END __attribute__((aligned(4)))

#define RCC_PLLSOURCE_HSI_DIV2 0x00000000U /*!< HSI clock divided by 2 selected as PLL entry clock source */

#define RCC_OSCILLATORTYPE_HSI 0x00000002U
#define RCC_OSCILLATORTYPE_LSE 0x00000004U
#define RCC_OSCILLATORTYPE_LSI 0x00000008U
#define RCC_HSE_OFF 0x00000000U                                       /*!< HSE clock deactivation */
#define RCC_HSE_BYPASS ((uint32_t)(RCC_CR_HSEBYP | RCC_CR_HSEON))     /*!< External clock source for HSE clock */
#define RCC_LSE_OFF 0x00000000U                                       /*!< LSE clock deactivation */
#define RCC_LSE_ON RCC_BDCR_LSEON                                     /*!< LSE clock activation */
#define RCC_LSE_BYPASS ((uint32_t)(RCC_BDCR_LSEBYP | RCC_BDCR_LSEON)) /*!< External clock source for LSE clock */
#define RCC_HSI_OFF 0x00000000U                                       /*!< HSI clock deactivation */

#define RCC_LSI_OFF 0x00000000U /*!< LSI clock deactivation */

#define RCC_PLL_NONE 0x00000000U /*!< PLL is not configured */
#define RCC_PLL_OFF 0x00000001U  /*!< PLL deactivation */

#define RCC_SYSCLKSOURCE_HSE RCC_CFGR_SW_HSE /*!< HSE selected as system clock */

#define RCC_SYSCLKSOURCE_STATUS_HSI RCC_CFGR_SWS_HSI                          /*!< HSI used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_HSE RCC_CFGR_SWS_HSE                          /*!< HSE used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_PLLCLK RCC_CFGR_SWS_PLL                       /*!< PLL used as system clock */
#define RCC_HCLK_DIV16 RCC_CFGR_PPRE1_DIV16                                   /*!< HCLK divided by 16 */
#define RCC_FLAG_HSIRDY ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_HSIRDY_Pos)) /*!< Internal High Speed clock ready flag */
#define RCC_FLAG_HSERDY ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_HSERDY_Pos)) /*!< External High Speed clock ready flag */
#define RCC_FLAG_PLLRDY ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_PLLRDY_Pos)) /*!< PLL clock ready flag */

#define RCC_FLAG_LSIRDY ((uint8_t)((CSR_REG_INDEX << 5U) | RCC_CSR_LSIRDY_Pos)) /*!< Internal Low Speed oscillator Ready */

#define RCC_FLAG_LSERDY ((uint8_t)((BDCR_REG_INDEX << 5U) | RCC_BDCR_LSERDY_Pos)) /*!< External Low Speed oscillator Ready */

#define __HAL_RCC_PWR_CLK_ENABLE()                                                               \
  do                                                                                             \
  {                                                                                              \
    __IO uint32_t tmpreg;                                                                        \
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN); /* Delay after an RCC peripheral clock enabling */ \
    tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);                                          \
    UNUSED(tmpreg);                                                                              \
  } while (0U)

#define __HAL_RCC_PWR_CLK_DISABLE() (RCC->APB1ENR &= ~(RCC_APB1ENR_PWREN))

#define __HAL_RCC_PWR_IS_CLK_DISABLED() ((RCC->APB1ENR & (RCC_APB1ENR_PWREN)) == RESET)
#define __HAL_RCC_HSI_ENABLE() (*(__IO uint32_t *)RCC_CR_HSION_BB = ENABLE)
#define __HAL_RCC_HSI_DISABLE() (*(__IO uint32_t *)RCC_CR_HSION_BB = DISABLE)

#define __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(_HSICALIBRATIONVALUE_) (MODIFY_REG(RCC->CR, RCC_CR_HSITRIM, (uint32_t)(_HSICALIBRATIONVALUE_) << RCC_CR_HSITRIM_Pos))

#define __HAL_RCC_LSI_ENABLE() (*(__IO uint32_t *)RCC_CSR_LSION_BB = ENABLE)

#define __HAL_RCC_LSI_DISABLE() (*(__IO uint32_t *)RCC_CSR_LSION_BB = DISABLE)

#define __HAL_RCC_HSE_CONFIG(__STATE__)     \
  do                                        \
  {                                         \
    if ((__STATE__) == RCC_HSE_ON)          \
    {                                       \
      SET_BIT(RCC->CR, RCC_CR_HSEON);       \
    }                                       \
    else if ((__STATE__) == RCC_HSE_OFF)    \
    {                                       \
      CLEAR_BIT(RCC->CR, RCC_CR_HSEON);     \
      CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);    \
    }                                       \
    else if ((__STATE__) == RCC_HSE_BYPASS) \
    {                                       \
      SET_BIT(RCC->CR, RCC_CR_HSEBYP);      \
      SET_BIT(RCC->CR, RCC_CR_HSEON);       \
    }                                       \
    else                                    \
    {                                       \
      CLEAR_BIT(RCC->CR, RCC_CR_HSEON);     \
      CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);    \
    }                                       \
  } while (0U)

#define __HAL_RCC_LSE_CONFIG(__STATE__)      \
  do                                         \
  {                                          \
    if ((__STATE__) == RCC_LSE_ON)           \
    {                                        \
      SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);    \
    }                                        \
    else if ((__STATE__) == RCC_LSE_OFF)     \
    {                                        \
      CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEON);  \
      CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEBYP); \
    }                                        \
    else if ((__STATE__) == RCC_LSE_BYPASS)  \
    {                                        \
      SET_BIT(RCC->BDCR, RCC_BDCR_LSEBYP);   \
      SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);    \
    }                                        \
    else                                     \
    {                                        \
      CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEON);  \
      CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEBYP); \
    }                                        \
  } while (0U)

#define __HAL_RCC_PLL_ENABLE() (*(__IO uint32_t *)RCC_CR_PLLON_BB = ENABLE)

#define __HAL_RCC_PLL_DISABLE() (*(__IO uint32_t *)RCC_CR_PLLON_BB = DISABLE)

#define __HAL_RCC_PLL_CONFIG(__RCC_PLLSOURCE__, __PLLMUL__) MODIFY_REG(RCC->CFGR, (RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL), ((__RCC_PLLSOURCE__) | (__PLLMUL__)))

#define __HAL_RCC_GET_PLL_OSCSOURCE() ((uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_PLLSRC)))

#define __HAL_RCC_SYSCLK_CONFIG(__SYSCLKSOURCE__) MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, (__SYSCLKSOURCE__))

#define __HAL_RCC_GET_SYSCLK_SOURCE() ((uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_SWS)))

#define __HAL_RCC_RTC_CONFIG(__RTC_CLKSOURCE__) MODIFY_REG(RCC->BDCR, RCC_BDCR_RTCSEL, (__RTC_CLKSOURCE__))
#define __HAL_RCC_BACKUPRESET_FORCE() (*(__IO uint32_t *)RCC_BDCR_BDRST_BB = ENABLE)

#define __HAL_RCC_BACKUPRESET_RELEASE() (*(__IO uint32_t *)RCC_BDCR_BDRST_BB = DISABLE)
#define __HAL_RCC_GET_FLAG(__FLAG__) (((((__FLAG__) >> 5U) == CR_REG_INDEX) ? RCC->CR : ((((__FLAG__) >> 5U) == BDCR_REG_INDEX) ? RCC->BDCR : RCC->CSR)) & (1U << ((__FLAG__) & RCC_FLAG_MASK)))
#define RCC_PERIPHCLK_RTC 0x00000001U
#define RCC_PERIPHCLK_ADC 0x00000002U
#define __HAL_RCC_USB_CLK_ENABLE()                                                               \
  do                                                                                             \
  {                                                                                              \
    __IO uint32_t tmpreg;                                                                        \
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USBEN); /* Delay after an RCC peripheral clock enabling */ \
    tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_USBEN);                                          \
    UNUSED(tmpreg);                                                                              \
  } while (0U)

#define __HAL_RCC_HSE_PREDIV_CONFIG(__HSE_PREDIV_VALUE__) MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLXTPRE, (uint32_t)(__HSE_PREDIV_VALUE__))
#define __HAL_RCC_USB_CONFIG(__USBCLKSOURCE__) MODIFY_REG(RCC->CFGR, RCC_CFGR_USBPRE, (uint32_t)(__USBCLKSOURCE__))
#define __HAL_RCC_ADC_CONFIG(__ADCCLKSOURCE__) MODIFY_REG(RCC->CFGR, RCC_CFGR_ADCPRE, (uint32_t)(__ADCCLKSOURCE__))

HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);

uint32_t HAL_RCC_GetSysClockFreq(void);
void HAL_RCC_GetOscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
#define RCC_DBP_TIMEOUT_VALUE 100U /* 100 ms */
#define RCC_LSE_TIMEOUT_VALUE LSE_STARTUP_TIMEOUT
#define CLOCKSWITCH_TIMEOUT_VALUE 5000 /* 5 s    */
#define HSE_TIMEOUT_VALUE HSE_STARTUP_TIMEOUT
#define HSI_TIMEOUT_VALUE 2U /* 2 ms (minimum Tick + 1) */
#define LSI_TIMEOUT_VALUE 2U /* 2 ms (minimum Tick + 1) */
#define PLL_TIMEOUT_VALUE 2U /* 2 ms (minimum Tick + 1) */

#define RCC_OFFSET (RCC_BASE - PERIPH_BASE)
#define RCC_CR_OFFSET 0x00U

#define RCC_BDCR_OFFSET 0x20U
#define RCC_CSR_OFFSET 0x24U

#define RCC_CR_OFFSET_BB (RCC_OFFSET + RCC_CR_OFFSET)

#define RCC_BDCR_OFFSET_BB (RCC_OFFSET + RCC_BDCR_OFFSET)
#define RCC_CSR_OFFSET_BB (RCC_OFFSET + RCC_CSR_OFFSET)
#define RCC_HSION_BIT_NUMBER RCC_CR_HSION_Pos
#define RCC_CR_HSION_BB ((uint32_t)(PERIPH_BB_BASE + (RCC_CR_OFFSET_BB * 32U) + (RCC_HSION_BIT_NUMBER * 4U)))
#define RCC_PLLON_BIT_NUMBER RCC_CR_PLLON_Pos
#define RCC_CR_PLLON_BB ((uint32_t)(PERIPH_BB_BASE + (RCC_CR_OFFSET_BB * 32U) + (RCC_PLLON_BIT_NUMBER * 4U)))
#define RCC_LSION_BIT_NUMBER RCC_CSR_LSION_Pos
#define RCC_CSR_LSION_BB ((uint32_t)(PERIPH_BB_BASE + (RCC_CSR_OFFSET_BB * 32U) + (RCC_LSION_BIT_NUMBER * 4U)))
#define RCC_BDRST_BIT_NUMBER RCC_BDCR_BDRST_Pos
#define RCC_BDCR_BDRST_BB ((uint32_t)(PERIPH_BB_BASE + (RCC_BDCR_OFFSET_BB * 32U) + (RCC_BDRST_BIT_NUMBER * 4U)))
#define CR_REG_INDEX ((uint8_t)1)
#define BDCR_REG_INDEX ((uint8_t)2)
#define CSR_REG_INDEX ((uint8_t)3)
#define RCC_FLAG_MASK ((uint8_t)0x1F)



#define NVIC_PRIORITYGROUP_0 0x00000007U
#define NVIC_PRIORITYGROUP_1 0x00000006U
#define NVIC_PRIORITYGROUP_2 0x00000005U
#define NVIC_PRIORITYGROUP_3 0x00000004U
#define NVIC_PRIORITYGROUP_4 0x00000003U


#define __HAL_FLASH_SET_LATENCY(__LATENCY__) (FLASH->ACR = (FLASH->ACR & (~FLASH_ACR_LATENCY)) | (__LATENCY__))
#define __HAL_FLASH_GET_LATENCY() (READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY))
#define __HAL_FLASH_PREFETCH_BUFFER_ENABLE() (FLASH->ACR |= FLASH_ACR_PRFTBE)

#ifdef HAL_PCD_MODULE_ENABLED

typedef enum
{
  USB_DEVICE_MODE = 0,
  USB_HOST_MODE = 1,
  USB_DRD_MODE = 2
} USB_ModeTypeDef;

typedef enum
{
  URB_IDLE = 0,
  URB_DONE,
  URB_NOTREADY,
  URB_NYET,
  URB_ERROR,
  URB_STALL
} USB_URBStateTypeDef;

typedef enum
{
  HC_IDLE = 0,
  HC_XFRC,
  HC_HALTED,
  HC_ACK,
  HC_NAK,
  HC_NYET,
  HC_STALL,
  HC_XACTERR,
  HC_BBLERR,
  HC_DATATGLERR
} USB_HCStateTypeDef;

typedef struct
{
  uint32_t dev_endpoints;           /*!< Device Endpoints number.										*/
  uint32_t speed;                   /*!< USB Core speed.										*/
  uint32_t ep0_mps;                 /*!< Set the Endpoint 0 Max Packet size.                                    */
  uint32_t phy_itface;              /*!< Select the used PHY interface.							 */
  uint32_t Sof_enable;              /*!< Enable or disable the output of the SOF signal.                        */
  uint32_t low_power_enable;        /*!< Enable or disable the low Power Mode.                                  */
  uint32_t lpm_enable;              /*!< Enable or disable Link Power Management.                               */
  uint32_t battery_charging_enable; /*!< Enable or disable Battery charging.                                    */
} USB_CfgTypeDef;

typedef struct
{
  uint8_t num;            /*!< Endpoint number */
  uint8_t is_in;          /*!< Endpoint direction  */
  uint8_t is_stall;       /*!< Endpoint stall condition   */
  uint8_t type;           /*!< Endpoint type     */
  uint8_t data_pid_start; /*!< Initial data PID    */
  uint16_t pmaadress;     /*!< PMA Address */
  uint16_t pmaaddr0;      /*!< PMA Address0   */
  uint16_t pmaaddr1;      /*!< PMA Address1 */
  uint8_t doublebuffer;   /*!< Double buffer enable                                      */
  uint32_t maxpacket;     /*!< Endpoint Max packet size */
  uint8_t *xfer_buff;     /*!< Pointer to transfer buffer                                               */
  uint32_t xfer_len;      /*!< Current transfer length                                                  */
  uint32_t xfer_count;    /*!< Partial transfer length in case of multi packet transfer                 */
  uint32_t xfer_len_db;   /*!< double buffer transfer length used with bulk double buffer in            */
  uint8_t xfer_fill_db;   /*!< double buffer Need to Fill new buffer  used with bulk_in                 */
} USB_EPTypeDef;

typedef struct
{
  uint8_t dev_addr;              /*!< USB device address.  */
  uint8_t ch_num;                /*!< Host channel number.   */
  uint8_t ep_num;                /*!< Endpoint number.    */
  uint8_t ep_is_in;              /*!< Endpoint direction    */
  uint8_t speed;                 /*!< USB Host Channel speed.           */
  uint8_t do_ping;               /*!< Enable or disable the use of the PING protocol for HS mode.                */
  uint8_t hub_port_nbr;          /*!< USB HUB port number                                                        */
  uint8_t hub_addr;              /*!< USB HUB address                                                            */
  uint8_t ep_type;               /*!< Endpoint Type.                   */
  uint16_t max_packet;           /*!< Endpoint Max packet size.  */
  uint8_t data_pid;              /*!< Initial data PID. */
  uint8_t *xfer_buff;            /*!< Pointer to transfer buffer.                                                */
  uint32_t XferSize;             /*!< OTG Channel transfer size.                                                 */
  uint32_t xfer_len;             /*!< Current transfer length.                                                   */
  uint32_t xfer_count;           /*!< Partial transfer length in case of multi packet transfer.                  */
  uint8_t toggle_in;             /*!< IN transfer current toggle flag.   */
  uint8_t toggle_out;            /*!< OUT transfer current toggle flag    */
  uint32_t dma_addr;             /*!< 32 bits aligned transfer buffer address.                                   */
  uint32_t ErrCnt;               /*!< Host channel error count.                                                  */
  USB_URBStateTypeDef urb_state; /*!< URB state.            */
  USB_HCStateTypeDef state;      /*!< Host Channel state.          */
} USB_HCTypeDef;

#define EP_MPS_64 0U
#define EP_MPS_32 1U
#define EP_MPS_16 2U
#define EP_MPS_8 3U

#define EP_TYPE_CTRL 0U
#define EP_TYPE_ISOC 1U
#define EP_TYPE_BULK 2U
#define EP_TYPE_INTR 3U
#define EP_TYPE_MSK 3U

#define EP_SPEED_LOW 0U
#define EP_SPEED_FULL 1U
#define EP_SPEED_HIGH 2U

#define HC_PID_DATA0 0U
#define HC_PID_DATA2 1U
#define HC_PID_DATA1 2U
#define HC_PID_SETUP 3U

#define USBD_FS_SPEED 2U
#define USBH_FSLS_SPEED 1U

#define BTABLE_ADDRESS 0x000U
#define PMA_ACCESS 2U

#define USB_EP_RX_STRX (0x3U << 12)

#define EP_ADDR_MSK 0x7U

#define USE_USB_DOUBLE_BUFFER 1U

typedef struct
{
  __IO uint16_t EP0R;          /*!< USB Endpoint 0 register,                   Address offset: 0x00 */
  __IO uint16_t RESERVED0;     /*!< Reserved */
  __IO uint16_t EP1R;          /*!< USB Endpoint 1 register,                   Address offset: 0x04 */
  __IO uint16_t RESERVED1;     /*!< Reserved */
  __IO uint16_t EP2R;          /*!< USB Endpoint 2 register,                   Address offset: 0x08 */
  __IO uint16_t RESERVED2;     /*!< Reserved */
  __IO uint16_t EP3R;          /*!< USB Endpoint 3 register,                   Address offset: 0x0C */
  __IO uint16_t RESERVED3;     /*!< Reserved */
  __IO uint16_t EP4R;          /*!< USB Endpoint 4 register,                   Address offset: 0x10 */
  __IO uint16_t RESERVED4;     /*!< Reserved */
  __IO uint16_t EP5R;          /*!< USB Endpoint 5 register,                   Address offset: 0x14 */
  __IO uint16_t RESERVED5;     /*!< Reserved */
  __IO uint16_t EP6R;          /*!< USB Endpoint 6 register,                   Address offset: 0x18 */
  __IO uint16_t RESERVED6;     /*!< Reserved */
  __IO uint16_t EP7R;          /*!< USB Endpoint 7 register,                   Address offset: 0x1C */
  __IO uint16_t RESERVED7[17]; /*!< Reserved */
  __IO uint16_t CNTR;          /*!< Control register,                          Address offset: 0x40 */
  __IO uint16_t RESERVED8;     /*!< Reserved */
  __IO uint16_t ISTR;          /*!< Interrupt status register,                 Address offset: 0x44 */
  __IO uint16_t RESERVED9;     /*!< Reserved */
  __IO uint16_t FNR;           /*!< Frame number register,                     Address offset: 0x48 */
  __IO uint16_t RESERVEDA;     /*!< Reserved */
  __IO uint16_t DADDR;         /*!< Device address register,                   Address offset: 0x4C */
  __IO uint16_t RESERVEDB;     /*!< Reserved */
  __IO uint16_t BTABLE;        /*!< Buffer Table address register,             Address offset: 0x50 */
  __IO uint16_t RESERVEDC;     /*!< Reserved */
} USB_TypeDef;

HAL_StatusTypeDef USB_CoreInit(USB_TypeDef *USBx, USB_CfgTypeDef cfg);
HAL_StatusTypeDef USB_DevInit(USB_TypeDef *USBx, USB_CfgTypeDef cfg);
HAL_StatusTypeDef USB_EnableGlobalInt(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_DisableGlobalInt(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_SetCurrentMode(USB_TypeDef *USBx, USB_ModeTypeDef mode);
HAL_StatusTypeDef USB_SetDevSpeed(USB_TypeDef *USBx, uint8_t speed);

HAL_StatusTypeDef USB_FlushRxFifo(USB_TypeDef const *USBx);
HAL_StatusTypeDef USB_FlushTxFifo(USB_TypeDef const *USBx, uint32_t num);

HAL_StatusTypeDef USB_ActivateEndpoint(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_DeactivateEndpoint(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPStartXfer(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPSetStall(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPClearStall(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPStopXfer(USB_TypeDef *USBx, USB_EPTypeDef *ep);

HAL_StatusTypeDef USB_SetDevAddress(USB_TypeDef *USBx, uint8_t address);
HAL_StatusTypeDef USB_DevConnect(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_DevDisconnect(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_StopDevice(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_EP0_OutStart(USB_TypeDef *USBx, uint8_t *psetup);
HAL_StatusTypeDef USB_WritePacket(USB_TypeDef *USBx, uint8_t *src, uint8_t ch_ep_num, uint16_t len);

void *USB_ReadPacket(USB_TypeDef *USBx, uint8_t *dest, uint16_t len);

uint32_t USB_ReadInterrupts(USB_TypeDef const *USBx);
uint32_t USB_ReadDevAllOutEpInterrupt(USB_TypeDef *USBx);
uint32_t USB_ReadDevOutEPInterrupt(USB_TypeDef *USBx, uint8_t epnum);
uint32_t USB_ReadDevAllInEpInterrupt(USB_TypeDef *USBx);
uint32_t USB_ReadDevInEPInterrupt(USB_TypeDef *USBx, uint8_t epnum);
void USB_ClearInterrupts(USB_TypeDef *USBx, uint32_t interrupt);
HAL_StatusTypeDef USB_ActivateRemoteWakeup(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_DeActivateRemoteWakeup(USB_TypeDef *USBx);

void USB_WritePMA(USB_TypeDef const *USBx, uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes);
void USB_ReadPMA(USB_TypeDef const *USBx, uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes);

typedef enum
{
  HAL_PCD_STATE_RESET = 0x00,
  HAL_PCD_STATE_READY = 0x01,
  HAL_PCD_STATE_ERROR = 0x02,
  HAL_PCD_STATE_BUSY = 0x03,
  HAL_PCD_STATE_TIMEOUT = 0x04
} PCD_StateTypeDef;

typedef enum
{
  LPM_L0 = 0x00, /* on */
  LPM_L1 = 0x01, /* LPM L1 sleep */
  LPM_L2 = 0x02, /* suspend */
  LPM_L3 = 0x03, /* off */
} PCD_LPM_StateTypeDef;

typedef USB_TypeDef PCD_TypeDef;
typedef USB_CfgTypeDef PCD_InitTypeDef;
typedef USB_EPTypeDef PCD_EPTypeDef;

typedef struct
{
  PCD_TypeDef *Instance;          /*!< Register base address             */
  PCD_InitTypeDef Init;           /*!< PCD required parameters           */
  __IO uint8_t USB_Address;       /*!< USB Address                       */
  PCD_EPTypeDef IN_ep[8];         /*!< IN endpoint parameters            */
  PCD_EPTypeDef OUT_ep[8];        /*!< OUT endpoint parameters           */
  HAL_LockTypeDef Lock;           /*!< PCD peripheral status             */
  __IO PCD_StateTypeDef State;    /*!< PCD communication state           */
  __IO uint32_t ErrorCode;        /*!< PCD Error code                    */
  uint32_t Setup[12];             /*!< Setup packet buffer               */
  PCD_LPM_StateTypeDef LPM_State; /*!< LPM State                         */
  uint32_t BESL;
  uint32_t FrameNumber; /*!< Store Current Frame number        */
  void *pData;          /*!< Pointer to upper stack Handler    */
} PCD_HandleTypeDef;

#define PCD_SPEED_FULL USBD_FS_SPEED

#define __HAL_PCD_ENABLE(__HANDLE__) (void)USB_EnableGlobalInt((__HANDLE__)->Instance)
#define __HAL_PCD_DISABLE(__HANDLE__) (void)USB_DisableGlobalInt((__HANDLE__)->Instance)
#define __HAL_PCD_CLEAR_FLAG(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->ISTR) &= (uint16_t)(~(__INTERRUPT__)))

HAL_StatusTypeDef HAL_PCD_SetAddress(PCD_HandleTypeDef *hpcd, uint8_t address);

#define PCD_ENDP0 0U
#define PCD_SNG_BUF 0U
#define USB_CNTRX_BLSIZE (0x1U << 15)

#define PCD_SET_ENDPOINT(USBx, bEpNum, wRegValue) (*(__IO uint16_t *)(&(USBx)->EP0R + ((bEpNum) * 2U)) = (uint16_t)(wRegValue))

#define PCD_GET_ENDPOINT(USBx, bEpNum) (*(__IO uint16_t *)(&(USBx)->EP0R + ((bEpNum) * 2U)))

#define PCD_FREE_USER_BUFFER(USBx, bEpNum, bDir) \
  do                                             \
  {                                              \
    if ((bDir) == 0U)                            \
    { /* OUT double buffered endpoint */         \
      PCD_TX_DTOG((USBx), (bEpNum));             \
    }                                            \
    else if ((bDir) == 1U)                       \
    { /* IN double buffered endpoint */          \
      PCD_RX_DTOG((USBx), (bEpNum));             \
    }                                            \
  } while (0)

#define PCD_SET_EP_TX_STATUS(USBx, bEpNum, wState)                                              \
  do                                                                                            \
  {                                                                                             \
    uint16_t _wRegVal;                                                                          \
    _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPTX_DTOGMASK; /* toggle first bit ? */ \
    if ((USB_EPTX_DTOG1 & (wState)) != 0U)                                                      \
    {                                                                                           \
      _wRegVal ^= USB_EPTX_DTOG1;                                                               \
    } /* toggle second bit ?  */                                                                \
    if ((USB_EPTX_DTOG2 & (wState)) != 0U)                                                      \
    {                                                                                           \
      _wRegVal ^= USB_EPTX_DTOG2;                                                               \
    }                                                                                           \
    PCD_SET_ENDPOINT((USBx), (bEpNum), (_wRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX));             \
  } while (0) /* PCD_SET_EP_TX_STATUS */

#define PCD_SET_EP_RX_STATUS(USBx, bEpNum, wState)                                              \
  do                                                                                            \
  {                                                                                             \
    uint16_t _wRegVal;                                                                          \
    _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPRX_DTOGMASK; /* toggle first bit ? */ \
    if ((USB_EPRX_DTOG1 & (wState)) != 0U)                                                      \
    {                                                                                           \
      _wRegVal ^= USB_EPRX_DTOG1;                                                               \
    } /* toggle second bit ? */                                                                 \
    if ((USB_EPRX_DTOG2 & (wState)) != 0U)                                                      \
    {                                                                                           \
      _wRegVal ^= USB_EPRX_DTOG2;                                                               \
    }                                                                                           \
    PCD_SET_ENDPOINT((USBx), (bEpNum), (_wRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX));             \
  } while (0) /* PCD_SET_EP_RX_STATUS */

#define PCD_SET_EP_KIND(USBx, bEpNum)                                                             \
  do                                                                                              \
  {                                                                                               \
    uint16_t _wRegVal;                                                                            \
    _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPREG_MASK;                               \
    PCD_SET_ENDPOINT((USBx), (bEpNum), (_wRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_KIND)); \
  } while (0) /* PCD_SET_EP_KIND */

#define PCD_CLEAR_EP_KIND(USBx, bEpNum)                                             \
  do                                                                                \
  {                                                                                 \
    uint16_t _wRegVal;                                                              \
    _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPKIND_MASK;                \
    PCD_SET_ENDPOINT((USBx), (bEpNum), (_wRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX)); \
  } while (0) /* PCD_CLEAR_EP_KIND */

#define PCD_SET_BULK_EP_DBUF(USBx, bEpNum) PCD_SET_EP_KIND((USBx), (bEpNum))
#define PCD_CLEAR_BULK_EP_DBUF(USBx, bEpNum) PCD_CLEAR_EP_KIND((USBx), (bEpNum))

#define PCD_CLEAR_RX_EP_CTR(USBx, bEpNum)                                       \
  do                                                                            \
  {                                                                             \
    uint16_t _wRegVal;                                                          \
    _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & (0x7FFFU & USB_EPREG_MASK); \
    PCD_SET_ENDPOINT((USBx), (bEpNum), (_wRegVal | USB_EP_CTR_TX));             \
  } while (0) /* PCD_CLEAR_RX_EP_CTR */

#define PCD_CLEAR_TX_EP_CTR(USBx, bEpNum)                                       \
  do                                                                            \
  {                                                                             \
    uint16_t _wRegVal;                                                          \
    _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & (0xFF7FU & USB_EPREG_MASK); \
    PCD_SET_ENDPOINT((USBx), (bEpNum), (_wRegVal | USB_EP_CTR_RX));             \
  } while (0) /* PCD_CLEAR_TX_EP_CTR */

#define PCD_RX_DTOG(USBx, bEpNum)                                                                   \
  do                                                                                                \
  {                                                                                                 \
    uint16_t _wEPVal;                                                                               \
    _wEPVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPREG_MASK;                                  \
    PCD_SET_ENDPOINT((USBx), (bEpNum), (_wEPVal | USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_DTOG_RX)); \
  } while (0) /* PCD_RX_DTOG */

#define PCD_TX_DTOG(USBx, bEpNum)                                                                   \
  do                                                                                                \
  {                                                                                                 \
    uint16_t _wEPVal;                                                                               \
    _wEPVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPREG_MASK;                                  \
    PCD_SET_ENDPOINT((USBx), (bEpNum), (_wEPVal | USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_DTOG_TX)); \
  } while (0) /* PCD_TX_DTOG */

#define PCD_CLEAR_RX_DTOG(USBx, bEpNum)            \
  do                                               \
  {                                                \
    uint16_t _wRegVal;                             \
    _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum)); \
    if ((_wRegVal & USB_EP_DTOG_RX) != 0U)         \
    {                                              \
      PCD_RX_DTOG((USBx), (bEpNum));               \
    }                                              \
  } while (0) /* PCD_CLEAR_RX_DTOG */

#define PCD_CLEAR_TX_DTOG(USBx, bEpNum)            \
  do                                               \
  {                                                \
    uint16_t _wRegVal;                             \
    _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum)); \
    if ((_wRegVal & USB_EP_DTOG_TX) != 0U)         \
    {                                              \
      PCD_TX_DTOG((USBx), (bEpNum));               \
    }                                              \
  } while (0) /* PCD_CLEAR_TX_DTOG */

#define PCD_SET_EP_ADDRESS(USBx, bEpNum, bAddr)                                     \
  do                                                                                \
  {                                                                                 \
    uint16_t _wRegVal;                                                              \
    _wRegVal = (PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPREG_MASK) | (bAddr);     \
    PCD_SET_ENDPOINT((USBx), (bEpNum), (_wRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX)); \
  } while (0) /* PCD_SET_EP_ADDRESS */

#define PCD_EP_TX_CNT(USBx, bEpNum) ((uint16_t *)((((uint32_t)(USBx)->BTABLE + ((uint32_t)(bEpNum) * 8U) + 2U) * PMA_ACCESS) + ((uint32_t)(USBx) + 0x400U)))

#define PCD_EP_RX_CNT(USBx, bEpNum) ((uint16_t *)((((uint32_t)(USBx)->BTABLE + ((uint32_t)(bEpNum) * 8U) + 6U) * PMA_ACCESS) + ((uint32_t)(USBx) + 0x400U)))

#define PCD_SET_EP_TX_ADDRESS(USBx, bEpNum, wAddr)                                               \
  do                                                                                             \
  {                                                                                              \
    __IO uint16_t *_wRegVal;                                                                     \
    uint32_t _wRegBase = (uint32_t)USBx;                                                         \
    _wRegBase += (uint32_t)(USBx)->BTABLE;                                                       \
    _wRegVal = (__IO uint16_t *)(_wRegBase + 0x400U + (((uint32_t)(bEpNum) * 8U) * PMA_ACCESS)); \
    *_wRegVal = ((wAddr) >> 1) << 1;                                                             \
  } while (0) /* PCD_SET_EP_TX_ADDRESS */

#define PCD_SET_EP_RX_ADDRESS(USBx, bEpNum, wAddr)                                                      \
  do                                                                                                    \
  {                                                                                                     \
    __IO uint16_t *_wRegVal;                                                                            \
    uint32_t _wRegBase = (uint32_t)USBx;                                                                \
    _wRegBase += (uint32_t)(USBx)->BTABLE;                                                              \
    _wRegVal = (__IO uint16_t *)(_wRegBase + 0x400U + ((((uint32_t)(bEpNum) * 8U) + 4U) * PMA_ACCESS)); \
    *_wRegVal = ((wAddr) >> 1) << 1;                                                                    \
  } while (0) /* PCD_SET_EP_RX_ADDRESS */

#define PCD_CALC_BLK32(pdwReg, wCount, wNBlocks)                    \
  do                                                                \
  {                                                                 \
    (wNBlocks) = (wCount) >> 5;                                     \
    if (((wCount) & 0x1fU) == 0U)                                   \
    {                                                               \
      (wNBlocks)--;                                                 \
    }                                                               \
    *(pdwReg) |= (uint16_t)(((wNBlocks) << 10) | USB_CNTRX_BLSIZE); \
  } while (0) /* PCD_CALC_BLK32 */

#define PCD_CALC_BLK2(pdwReg, wCount, wNBlocks) \
  do                                            \
  {                                             \
    (wNBlocks) = (wCount) >> 1;                 \
    if (((wCount) & 0x1U) != 0U)                \
    {                                           \
      (wNBlocks)++;                             \
    }                                           \
    *(pdwReg) |= (uint16_t)((wNBlocks) << 10);  \
  } while (0) /* PCD_CALC_BLK2 */

#define PCD_SET_EP_CNT_RX_REG(pdwReg, wCount)        \
  do                                                 \
  {                                                  \
    uint32_t wNBlocks;                               \
    *(pdwReg) &= 0x3FFU;                             \
    if ((wCount) > 62U)                              \
    {                                                \
      PCD_CALC_BLK32((pdwReg), (wCount), wNBlocks);  \
    }                                                \
    else                                             \
    {                                                \
      if ((wCount) == 0U)                            \
      {                                              \
        *(pdwReg) |= USB_CNTRX_BLSIZE;               \
      }                                              \
      else                                           \
      {                                              \
        PCD_CALC_BLK2((pdwReg), (wCount), wNBlocks); \
      }                                              \
    }                                                \
  } while (0) /* PCD_SET_EP_CNT_RX_REG */

#define PCD_SET_EP_RX_DBUF0_CNT(USBx, bEpNum, wCount)                                                 \
  do                                                                                                  \
  {                                                                                                   \
    uint32_t _wRegBase = (uint32_t)(USBx);                                                            \
    __IO uint16_t *pdwReg;                                                                            \
    _wRegBase += (uint32_t)(USBx)->BTABLE;                                                            \
    pdwReg = (__IO uint16_t *)(_wRegBase + 0x400U + ((((uint32_t)(bEpNum) * 8U) + 2U) * PMA_ACCESS)); \
    PCD_SET_EP_CNT_RX_REG(pdwReg, (wCount));                                                          \
  } while (0)

#define PCD_SET_EP_TX_CNT(USBx, bEpNum, wCount)                                                         \
  do                                                                                                    \
  {                                                                                                     \
    uint32_t _wRegBase = (uint32_t)(USBx);                                                              \
    __IO uint16_t *_wRegVal;                                                                            \
    _wRegBase += (uint32_t)(USBx)->BTABLE;                                                              \
    _wRegVal = (__IO uint16_t *)(_wRegBase + 0x400U + ((((uint32_t)(bEpNum) * 8U) + 2U) * PMA_ACCESS)); \
    *_wRegVal = (uint16_t)(wCount);                                                                     \
  } while (0)

#define PCD_SET_EP_RX_CNT(USBx, bEpNum, wCount)                                                         \
  do                                                                                                    \
  {                                                                                                     \
    uint32_t _wRegBase = (uint32_t)(USBx);                                                              \
    __IO uint16_t *_wRegVal;                                                                            \
    _wRegBase += (uint32_t)(USBx)->BTABLE;                                                              \
    _wRegVal = (__IO uint16_t *)(_wRegBase + 0x400U + ((((uint32_t)(bEpNum) * 8U) + 6U) * PMA_ACCESS)); \
    PCD_SET_EP_CNT_RX_REG(_wRegVal, (wCount));                                                          \
  } while (0)

#define PCD_GET_EP_TX_CNT(USBx, bEpNum) ((uint32_t)(*PCD_EP_TX_CNT((USBx), (bEpNum))) & 0x3ffU)
#define PCD_GET_EP_RX_CNT(USBx, bEpNum) ((uint32_t)(*PCD_EP_RX_CNT((USBx), (bEpNum))) & 0x3ffU)

#define PCD_SET_EP_DBUF0_ADDR(USBx, bEpNum, wBuf0Addr)    \
  do                                                      \
  {                                                       \
    PCD_SET_EP_TX_ADDRESS((USBx), (bEpNum), (wBuf0Addr)); \
  } while (0) /* PCD_SET_EP_DBUF0_ADDR */

#define PCD_SET_EP_DBUF1_ADDR(USBx, bEpNum, wBuf1Addr)    \
  do                                                      \
  {                                                       \
    PCD_SET_EP_RX_ADDRESS((USBx), (bEpNum), (wBuf1Addr)); \
  } while (0) /* PCD_SET_EP_DBUF1_ADDR */

#define PCD_SET_EP_DBUF_ADDR(USBx, bEpNum, wBuf0Addr, wBuf1Addr) \
  do                                                             \
  {                                                              \
    PCD_SET_EP_DBUF0_ADDR((USBx), (bEpNum), (wBuf0Addr));        \
    PCD_SET_EP_DBUF1_ADDR((USBx), (bEpNum), (wBuf1Addr));        \
  } while (0) /* PCD_SET_EP_DBUF_ADDR */

#define PCD_GET_EP_DBUF1_ADDR(USBx, bEpNum) (PCD_GET_EP_RX_ADDRESS((USBx), (bEpNum)))

#define PCD_SET_EP_DBUF0_CNT(USBx, bEpNum, bDir, wCount)   \
  do                                                       \
  {                                                        \
    if ((bDir) == 0U) /* OUT endpoint */                   \
    {                                                      \
      PCD_SET_EP_RX_DBUF0_CNT((USBx), (bEpNum), (wCount)); \
    }                                                      \
    else                                                   \
    {                                                      \
      if ((bDir) == 1U)                                    \
      { /* IN endpoint */                                  \
        PCD_SET_EP_TX_CNT((USBx), (bEpNum), (wCount));     \
      }                                                    \
    }                                                      \
  } while (0) /* SetEPDblBuf0Count*/

#define PCD_SET_EP_DBUF1_CNT(USBx, bEpNum, bDir, wCount)                                                   \
  do                                                                                                       \
  {                                                                                                        \
    uint32_t _wBase = (uint32_t)(USBx);                                                                    \
    __IO uint16_t *_wEPRegVal;                                                                             \
    if ((bDir) == 0U)                                                                                      \
    { /* OUT endpoint */                                                                                   \
      PCD_SET_EP_RX_CNT((USBx), (bEpNum), (wCount));                                                       \
    }                                                                                                      \
    else                                                                                                   \
    {                                                                                                      \
      if ((bDir) == 1U)                                                                                    \
      { /* IN endpoint */                                                                                  \
        _wBase += (uint32_t)(USBx)->BTABLE;                                                                \
        _wEPRegVal = (__IO uint16_t *)(_wBase + 0x400U + ((((uint32_t)(bEpNum) * 8U) + 6U) * PMA_ACCESS)); \
        *_wEPRegVal = (uint16_t)(wCount);                                                                  \
      }                                                                                                    \
    }                                                                                                      \
  } while (0) /* SetEPDblBuf1Count */

#define PCD_SET_EP_DBUF_CNT(USBx, bEpNum, bDir, wCount)       \
  do                                                          \
  {                                                           \
    PCD_SET_EP_DBUF0_CNT((USBx), (bEpNum), (bDir), (wCount)); \
    PCD_SET_EP_DBUF1_CNT((USBx), (bEpNum), (bDir), (wCount)); \
  } while (0) /* PCD_SET_EP_DBUF_CNT */

#define PCD_GET_EP_DBUF0_CNT(USBx, bEpNum) (PCD_GET_EP_TX_CNT((USBx), (bEpNum)))
#define PCD_GET_EP_DBUF1_CNT(USBx, bEpNum) (PCD_GET_EP_RX_CNT((USBx), (bEpNum)))


#endif /* HAL_PCD_MODULE_ENABLED */

#define assert_param(expr) ((void)0U)

typedef enum
{
  HAL_TICK_FREQ_10HZ = 100U,
  HAL_TICK_FREQ_100HZ = 10U,
  HAL_TICK_FREQ_1KHZ = 1U,
  HAL_TICK_FREQ_DEFAULT = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;

extern __IO uint32_t uwTick;
extern uint32_t uwTickPrio;
extern HAL_TickFreqTypeDef uwTickFreq;

#define __HAL_DBGMCU_FREEZE_TIM2() SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM2_STOP)
#define __HAL_DBGMCU_UNFREEZE_TIM2() CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM2_STOP)

#define __HAL_DBGMCU_FREEZE_TIM3() SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM3_STOP)
#define __HAL_DBGMCU_UNFREEZE_TIM3() CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM3_STOP)
#define IS_TICKFREQ(FREQ) (((FREQ) == HAL_TICK_FREQ_10HZ) || ((FREQ) == HAL_TICK_FREQ_100HZ) || ((FREQ) == HAL_TICK_FREQ_1KHZ))

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

#ifndef __STM32F1XX_H
#define __STM32F1XX_H

#define STM32F1

#define __STM32F1_CMSIS_VERSION_MAIN (0x04) /*!< [31:24] main version */
#define __STM32F1_CMSIS_VERSION_SUB1 (0x03) /*!< [23:16] sub1 version */
#define __STM32F1_CMSIS_VERSION_SUB2 (0x04) /*!< [15:8]  sub2 version */
#define __STM32F1_CMSIS_VERSION_RC (0x00)   /*!< [7:0]  release candidate */
#define __STM32F1_CMSIS_VERSION ((__STM32F1_CMSIS_VERSION_MAIN << 24) | (__STM32F1_CMSIS_VERSION_SUB1 << 16) | (__STM32F1_CMSIS_VERSION_SUB2 << 8) | (__STM32F1_CMSIS_VERSION_RC))

#ifndef __STM32F103xB_H
#define __STM32F103xB_H

#define __CM3_REV 0x0200U         /*!< Core Revision r2p0                           */
#define __MPU_PRESENT 0U          /*!< Other STM32 devices does not provide an MPU  */
#define __NVIC_PRIO_BITS 4U       /*!< STM32 uses 4 Bits for the Priority Levels    */
#define __Vendor_SysTickConfig 0U /*!< Set to 1 if different SysTick Config is used */

typedef enum
{

  NonMaskableInt_IRQn = -14,   /*!< 2 Non Maskable Interrupt                             */
  HardFault_IRQn = -13,        /*!< 3 Cortex-M3 Hard Fault Interrupt                     */
  MemoryManagement_IRQn = -12, /*!< 4 Cortex-M3 Memory Management Interrupt              */
  BusFault_IRQn = -11,         /*!< 5 Cortex-M3 Bus Fault Interrupt                      */
  UsageFault_IRQn = -10,       /*!< 6 Cortex-M3 Usage Fault Interrupt                    */
  SVCall_IRQn = -5,            /*!< 11 Cortex-M3 SV Call Interrupt                       */
  DebugMonitor_IRQn = -4,      /*!< 12 Cortex-M3 Debug Monitor Interrupt                 */
  PendSV_IRQn = -2,            /*!< 14 Cortex-M3 Pend SV Interrupt                       */
  SysTick_IRQn = -1,           /*!< 15 Cortex-M3 System Tick Interrupt                   */

  WWDG_IRQn = 0,             /*!< Window WatchDog Interrupt                            */
  PVD_IRQn = 1,              /*!< PVD through EXTI Line detection Interrupt            */
  TAMPER_IRQn = 2,           /*!< Tamper Interrupt                                     */
  RTC_IRQn = 3,              /*!< RTC global Interrupt                                 */
  FLASH_IRQn = 4,            /*!< FLASH global Interrupt                               */
  RCC_IRQn = 5,              /*!< RCC global Interrupt                                 */
  EXTI0_IRQn = 6,            /*!< EXTI Line0 Interrupt                                 */
  EXTI1_IRQn = 7,            /*!< EXTI Line1 Interrupt                                 */
  EXTI2_IRQn = 8,            /*!< EXTI Line2 Interrupt                                 */
  EXTI3_IRQn = 9,            /*!< EXTI Line3 Interrupt                                 */
  EXTI4_IRQn = 10,           /*!< EXTI Line4 Interrupt                                 */
  DMA1_Channel1_IRQn = 11,   /*!< DMA1 Channel 1 global Interrupt                      */
  DMA1_Channel2_IRQn = 12,   /*!< DMA1 Channel 2 global Interrupt                      */
  DMA1_Channel3_IRQn = 13,   /*!< DMA1 Channel 3 global Interrupt                      */
  DMA1_Channel4_IRQn = 14,   /*!< DMA1 Channel 4 global Interrupt                      */
  DMA1_Channel5_IRQn = 15,   /*!< DMA1 Channel 5 global Interrupt                      */
  DMA1_Channel6_IRQn = 16,   /*!< DMA1 Channel 6 global Interrupt                      */
  DMA1_Channel7_IRQn = 17,   /*!< DMA1 Channel 7 global Interrupt                      */
  ADC1_2_IRQn = 18,          /*!< ADC1 and ADC2 global Interrupt                       */
  USB_HP_CAN1_TX_IRQn = 19,  /*!< USB Device High Priority or CAN1 TX Interrupts       */
  USB_LP_CAN1_RX0_IRQn = 20, /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
  CAN1_RX1_IRQn = 21,        /*!< CAN1 RX1 Interrupt                                   */
  CAN1_SCE_IRQn = 22,        /*!< CAN1 SCE Interrupt                                   */
  EXTI9_5_IRQn = 23,         /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_IRQn = 24,        /*!< TIM1 Break Interrupt                                 */
  TIM1_UP_IRQn = 25,         /*!< TIM1 Update Interrupt                                */
  TIM1_TRG_COM_IRQn = 26,    /*!< TIM1 Trigger and Commutation Interrupt               */
  TIM1_CC_IRQn = 27,         /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn = 28,            /*!< TIM2 global Interrupt                                */
  TIM3_IRQn = 29,            /*!< TIM3 global Interrupt                                */
  TIM4_IRQn = 30,            /*!< TIM4 global Interrupt                                */
  I2C1_EV_IRQn = 31,         /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn = 32,         /*!< I2C1 Error Interrupt                                 */
  I2C2_EV_IRQn = 33,         /*!< I2C2 Event Interrupt                                 */
  I2C2_ER_IRQn = 34,         /*!< I2C2 Error Interrupt                                 */
  SPI1_IRQn = 35,            /*!< SPI1 global Interrupt                                */
  SPI2_IRQn = 36,            /*!< SPI2 global Interrupt                                */
  USART1_IRQn = 37,          /*!< USART1 global Interrupt                              */
  USART2_IRQn = 38,          /*!< USART2 global Interrupt                              */
  USART3_IRQn = 39,          /*!< USART3 global Interrupt                              */
  EXTI15_10_IRQn = 40,       /*!< External Line[15:10] Interrupts                      */
  RTC_Alarm_IRQn = 41,       /*!< RTC Alarm through EXTI Line Interrupt                */
  USBWakeUp_IRQn = 42,       /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
} IRQn_Type;

#ifndef __CMSIS_GCC_H
#define __CMSIS_GCC_H

#define __CM_CMSIS_VERSION_MAIN (5U)
#define __CM_CMSIS_VERSION_SUB (1U)
#define __CM_CMSIS_VERSION ((__CM_CMSIS_VERSION_MAIN << 16U) | __CM_CMSIS_VERSION_SUB)

/* ignore some GCC warnings */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wunused-parameter"

/* Fallback for __has_builtin */
#define __has_builtin(x) (0)

/* CMSIS compiler specific defines */
#define __ASM __asm
#define __INLINE inline
#define __STATIC_INLINE static inline
#define __STATIC_FORCEINLINE __attribute__((always_inline)) static inline
#define __NO_RETURN __attribute__((__noreturn__))
#define __USED __attribute__((used))
#define __WEAK __attribute__((weak))
#define __PACKED __attribute__((packed, aligned(1)))
#define __PACKED_STRUCT struct __attribute__((packed, aligned(1)))
#define __PACKED_UNION union __attribute__((packed, aligned(1)))

#ifndef __UNALIGNED_UINT32 /* deprecated */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
#pragma GCC diagnostic ignored "-Wattributes"
struct __attribute__((packed)) T_UINT32
{
  uint32_t v;
};
#pragma GCC diagnostic pop
#define __UNALIGNED_UINT32(x) (((struct T_UINT32 *)(x))->v)
#endif
#ifndef __UNALIGNED_UINT16_WRITE
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
#pragma GCC diagnostic ignored "-Wattributes"
__PACKED_STRUCT T_UINT16_WRITE { uint16_t v; };
#pragma GCC diagnostic pop
#define __UNALIGNED_UINT16_WRITE(addr, val) (void)((((struct T_UINT16_WRITE *)(void *)(addr))->v) = (val))
#endif
#ifndef __UNALIGNED_UINT16_READ
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
#pragma GCC diagnostic ignored "-Wattributes"
__PACKED_STRUCT T_UINT16_READ { uint16_t v; };
#pragma GCC diagnostic pop
#define __UNALIGNED_UINT16_READ(addr) (((const struct T_UINT16_READ *)(const void *)(addr))->v)
#endif
#ifndef __UNALIGNED_UINT32_WRITE
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
#pragma GCC diagnostic ignored "-Wattributes"
__PACKED_STRUCT T_UINT32_WRITE { uint32_t v; };
#pragma GCC diagnostic pop
#define __UNALIGNED_UINT32_WRITE(addr, val) (void)((((struct T_UINT32_WRITE *)(void *)(addr))->v) = (val))
#endif
#ifndef __UNALIGNED_UINT32_READ
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
#pragma GCC diagnostic ignored "-Wattributes"
__PACKED_STRUCT T_UINT32_READ { uint32_t v; };
#pragma GCC diagnostic pop
#define __UNALIGNED_UINT32_READ(addr) (((const struct T_UINT32_READ *)(const void *)(addr))->v)
#endif
#ifndef __ALIGNED
#define __ALIGNED(x) __attribute__((aligned(x)))
#endif
#ifndef __RESTRICT
#define __RESTRICT __restrict
#endif

#if defined(__thumb__) && !defined(__thumb2__)
#define __CMSIS_GCC_OUT_REG(r) "=l"(r)
#define __CMSIS_GCC_RW_REG(r) "+l"(r)
#define __CMSIS_GCC_USE_REG(r) "l"(r)
#else
#define __CMSIS_GCC_OUT_REG(r) "=r"(r)
#define __CMSIS_GCC_RW_REG(r) "+r"(r)
#define __CMSIS_GCC_USE_REG(r) "r"(r)
#endif

#define __NOP() __ASM volatile("nop")

#define __WFI() __ASM volatile("wfi")

#define __WFE() __ASM volatile("wfe")

#define __SEV() __ASM volatile("sev")

typedef union
{
  struct
  {
    uint32_t _reserved0 : 27; /*!< bit:  0..26  Reserved */
    uint32_t Q : 1;           /*!< bit:     27  Saturation condition flag */
    uint32_t V : 1;           /*!< bit:     28  Overflow condition code flag */
    uint32_t C : 1;           /*!< bit:     29  Carry condition code flag */
    uint32_t Z : 1;           /*!< bit:     30  Zero condition code flag */
    uint32_t N : 1;           /*!< bit:     31  Negative condition code flag */
  } b;                        /*!< Structure used for bit  access */
  uint32_t w;                 /*!< Type      used for word access */
} APSR_Type;

/* APSR Register Definitions */
#define APSR_N_Pos 31U                 /*!< APSR: N Position */
#define APSR_N_Msk (1UL << APSR_N_Pos) /*!< APSR: N Mask */

#define APSR_Z_Pos 30U                 /*!< APSR: Z Position */
#define APSR_Z_Msk (1UL << APSR_Z_Pos) /*!< APSR: Z Mask */

#define APSR_C_Pos 29U                 /*!< APSR: C Position */
#define APSR_C_Msk (1UL << APSR_C_Pos) /*!< APSR: C Mask */

#define APSR_V_Pos 28U                 /*!< APSR: V Position */
#define APSR_V_Msk (1UL << APSR_V_Pos) /*!< APSR: V Mask */

#define APSR_Q_Pos 27U                 /*!< APSR: Q Position */
#define APSR_Q_Msk (1UL << APSR_Q_Pos) /*!< APSR: Q Mask */

typedef union
{
  struct
  {
    uint32_t ISR : 9;         /*!< bit:  0.. 8  Exception number */
    uint32_t _reserved0 : 23; /*!< bit:  9..31  Reserved */
  } b;                        /*!< Structure used for bit  access */
  uint32_t w;                 /*!< Type      used for word access */
} IPSR_Type;

/* IPSR Register Definitions */
#define IPSR_ISR_Pos 0U                            /*!< IPSR: ISR Position */
#define IPSR_ISR_Msk (0x1FFUL /*<< IPSR_ISR_Pos*/) /*!< IPSR: ISR Mask */

typedef union
{
  struct
  {
    uint32_t ISR : 9;        /*!< bit:  0.. 8  Exception number */
    uint32_t _reserved0 : 1; /*!< bit:      9  Reserved */
    uint32_t ICI_IT_1 : 6;   /*!< bit: 10..15  ICI/IT part 1 */
    uint32_t _reserved1 : 8; /*!< bit: 16..23  Reserved */
    uint32_t T : 1;          /*!< bit:     24  Thumb bit */
    uint32_t ICI_IT_2 : 2;   /*!< bit: 25..26  ICI/IT part 2 */
    uint32_t Q : 1;          /*!< bit:     27  Saturation condition flag */
    uint32_t V : 1;          /*!< bit:     28  Overflow condition code flag */
    uint32_t C : 1;          /*!< bit:     29  Carry condition code flag */
    uint32_t Z : 1;          /*!< bit:     30  Zero condition code flag */
    uint32_t N : 1;          /*!< bit:     31  Negative condition code flag */
  } b;                       /*!< Structure used for bit  access */
  uint32_t w;                /*!< Type      used for word access */
} xPSR_Type;

/* xPSR Register Definitions */
#define xPSR_N_Pos 31U                 /*!< xPSR: N Position */
#define xPSR_N_Msk (1UL << xPSR_N_Pos) /*!< xPSR: N Mask */

#define xPSR_Z_Pos 30U                 /*!< xPSR: Z Position */
#define xPSR_Z_Msk (1UL << xPSR_Z_Pos) /*!< xPSR: Z Mask */

#define xPSR_C_Pos 29U                 /*!< xPSR: C Position */
#define xPSR_C_Msk (1UL << xPSR_C_Pos) /*!< xPSR: C Mask */

#define xPSR_V_Pos 28U                 /*!< xPSR: V Position */
#define xPSR_V_Msk (1UL << xPSR_V_Pos) /*!< xPSR: V Mask */

#define xPSR_Q_Pos 27U                 /*!< xPSR: Q Position */
#define xPSR_Q_Msk (1UL << xPSR_Q_Pos) /*!< xPSR: Q Mask */

#define xPSR_ICI_IT_2_Pos 25U                        /*!< xPSR: ICI/IT part 2 Position */
#define xPSR_ICI_IT_2_Msk (3UL << xPSR_ICI_IT_2_Pos) /*!< xPSR: ICI/IT part 2 Mask */

#define xPSR_T_Pos 24U                 /*!< xPSR: T Position */
#define xPSR_T_Msk (1UL << xPSR_T_Pos) /*!< xPSR: T Mask */

#define xPSR_ICI_IT_1_Pos 10U                           /*!< xPSR: ICI/IT part 1 Position */
#define xPSR_ICI_IT_1_Msk (0x3FUL << xPSR_ICI_IT_1_Pos) /*!< xPSR: ICI/IT part 1 Mask */

#define xPSR_ISR_Pos 0U                            /*!< xPSR: ISR Position */
#define xPSR_ISR_Msk (0x1FFUL /*<< xPSR_ISR_Pos*/) /*!< xPSR: ISR Mask */

typedef union
{
  struct
  {
    uint32_t nPRIV : 1;       /*!< bit:      0  Execution privilege in Thread mode */
    uint32_t SPSEL : 1;       /*!< bit:      1  Stack to be used */
    uint32_t _reserved1 : 30; /*!< bit:  2..31  Reserved */
  } b;                        /*!< Structure used for bit  access */
  uint32_t w;                 /*!< Type      used for word access */
} CONTROL_Type;

/* CONTROL Register Definitions */
#define CONTROL_SPSEL_Pos 1U                         /*!< CONTROL: SPSEL Position */
#define CONTROL_SPSEL_Msk (1UL << CONTROL_SPSEL_Pos) /*!< CONTROL: SPSEL Mask */

#define CONTROL_nPRIV_Pos 0U                             /*!< CONTROL: nPRIV Position */
#define CONTROL_nPRIV_Msk (1UL /*<< CONTROL_nPRIV_Pos*/) /*!< CONTROL: nPRIV Mask */

/*@} end of group CMSIS_CORE */

typedef struct
{
  __IOM uint32_t ISER[8U]; /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
  uint32_t RESERVED0[24U];
  __IOM uint32_t ICER[8U]; /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
  uint32_t RSERVED1[24U];
  __IOM uint32_t ISPR[8U]; /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
  uint32_t RESERVED2[24U];
  __IOM uint32_t ICPR[8U]; /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
  uint32_t RESERVED3[24U];
  __IOM uint32_t IABR[8U]; /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
  uint32_t RESERVED4[56U];
  __IOM uint8_t IP[240U]; /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
  uint32_t RESERVED5[644U];
  __OM uint32_t STIR; /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
} NVIC_Type;

/* Software Triggered Interrupt Register Definitions */
#define NVIC_STIR_INTID_Pos 0U                                   /*!< STIR: INTLINESNUM Position */
#define NVIC_STIR_INTID_Msk (0x1FFUL /*<< NVIC_STIR_INTID_Pos*/) /*!< STIR: INTLINESNUM Mask */

/*@} end of group CMSIS_NVIC */

typedef struct
{
  __IM uint32_t CPUID;    /*!< Offset: 0x000 (R/ )  CPUID Base Register */
  __IOM uint32_t ICSR;    /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
  __IOM uint32_t VTOR;    /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
  __IOM uint32_t AIRCR;   /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  __IOM uint32_t SCR;     /*!< Offset: 0x010 (R/W)  System Control Register */
  __IOM uint32_t CCR;     /*!< Offset: 0x014 (R/W)  Configuration Control Register */
  __IOM uint8_t SHP[12U]; /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
  __IOM uint32_t SHCSR;   /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
  __IOM uint32_t CFSR;    /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
  __IOM uint32_t HFSR;    /*!< Offset: 0x02C (R/W)  HardFault Status Register */
  __IOM uint32_t DFSR;    /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
  __IOM uint32_t MMFAR;   /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
  __IOM uint32_t BFAR;    /*!< Offset: 0x038 (R/W)  BusFault Address Register */
  __IOM uint32_t AFSR;    /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
  __IM uint32_t PFR[2U];  /*!< Offset: 0x040 (R/ )  Processor Feature Register */
  __IM uint32_t DFR;      /*!< Offset: 0x048 (R/ )  Debug Feature Register */
  __IM uint32_t ADR;      /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
  __IM uint32_t MMFR[4U]; /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
  __IM uint32_t ISAR[5U]; /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
  uint32_t RESERVED0[5U];
  __IOM uint32_t CPACR; /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_Type;

/* SCB CPUID Register Definitions */
#define SCB_CPUID_IMPLEMENTER_Pos 24U                                   /*!< SCB CPUID: IMPLEMENTER Position */
#define SCB_CPUID_IMPLEMENTER_Msk (0xFFUL << SCB_CPUID_IMPLEMENTER_Pos) /*!< SCB CPUID: IMPLEMENTER Mask */

#define SCB_CPUID_VARIANT_Pos 20U                              /*!< SCB CPUID: VARIANT Position */
#define SCB_CPUID_VARIANT_Msk (0xFUL << SCB_CPUID_VARIANT_Pos) /*!< SCB CPUID: VARIANT Mask */

#define SCB_CPUID_ARCHITECTURE_Pos 16U                                   /*!< SCB CPUID: ARCHITECTURE Position */
#define SCB_CPUID_ARCHITECTURE_Msk (0xFUL << SCB_CPUID_ARCHITECTURE_Pos) /*!< SCB CPUID: ARCHITECTURE Mask */

#define SCB_CPUID_PARTNO_Pos 4U                                /*!< SCB CPUID: PARTNO Position */
#define SCB_CPUID_PARTNO_Msk (0xFFFUL << SCB_CPUID_PARTNO_Pos) /*!< SCB CPUID: PARTNO Mask */

#define SCB_CPUID_REVISION_Pos 0U                                    /*!< SCB CPUID: REVISION Position */
#define SCB_CPUID_REVISION_Msk (0xFUL /*<< SCB_CPUID_REVISION_Pos*/) /*!< SCB CPUID: REVISION Mask */

/* SCB Interrupt Control State Register Definitions */
#define SCB_ICSR_NMIPENDSET_Pos 31U                              /*!< SCB ICSR: NMIPENDSET Position */
#define SCB_ICSR_NMIPENDSET_Msk (1UL << SCB_ICSR_NMIPENDSET_Pos) /*!< SCB ICSR: NMIPENDSET Mask */

#define SCB_ICSR_PENDSVSET_Pos 28U                             /*!< SCB ICSR: PENDSVSET Position */
#define SCB_ICSR_PENDSVSET_Msk (1UL << SCB_ICSR_PENDSVSET_Pos) /*!< SCB ICSR: PENDSVSET Mask */

#define SCB_ICSR_PENDSVCLR_Pos 27U                             /*!< SCB ICSR: PENDSVCLR Position */
#define SCB_ICSR_PENDSVCLR_Msk (1UL << SCB_ICSR_PENDSVCLR_Pos) /*!< SCB ICSR: PENDSVCLR Mask */

#define SCB_ICSR_PENDSTSET_Pos 26U                             /*!< SCB ICSR: PENDSTSET Position */
#define SCB_ICSR_PENDSTSET_Msk (1UL << SCB_ICSR_PENDSTSET_Pos) /*!< SCB ICSR: PENDSTSET Mask */

#define SCB_ICSR_PENDSTCLR_Pos 25U                             /*!< SCB ICSR: PENDSTCLR Position */
#define SCB_ICSR_PENDSTCLR_Msk (1UL << SCB_ICSR_PENDSTCLR_Pos) /*!< SCB ICSR: PENDSTCLR Mask */

#define SCB_ICSR_ISRPREEMPT_Pos 23U                              /*!< SCB ICSR: ISRPREEMPT Position */
#define SCB_ICSR_ISRPREEMPT_Msk (1UL << SCB_ICSR_ISRPREEMPT_Pos) /*!< SCB ICSR: ISRPREEMPT Mask */

#define SCB_ICSR_ISRPENDING_Pos 22U                              /*!< SCB ICSR: ISRPENDING Position */
#define SCB_ICSR_ISRPENDING_Msk (1UL << SCB_ICSR_ISRPENDING_Pos) /*!< SCB ICSR: ISRPENDING Mask */

#define SCB_ICSR_VECTPENDING_Pos 12U                                   /*!< SCB ICSR: VECTPENDING Position */
#define SCB_ICSR_VECTPENDING_Msk (0x1FFUL << SCB_ICSR_VECTPENDING_Pos) /*!< SCB ICSR: VECTPENDING Mask */

#define SCB_ICSR_RETTOBASE_Pos 11U                             /*!< SCB ICSR: RETTOBASE Position */
#define SCB_ICSR_RETTOBASE_Msk (1UL << SCB_ICSR_RETTOBASE_Pos) /*!< SCB ICSR: RETTOBASE Mask */

#define SCB_ICSR_VECTACTIVE_Pos 0U                                       /*!< SCB ICSR: VECTACTIVE Position */
#define SCB_ICSR_VECTACTIVE_Msk (0x1FFUL /*<< SCB_ICSR_VECTACTIVE_Pos*/) /*!< SCB ICSR: VECTACTIVE Mask */

/* SCB Vector Table Offset Register Definitions */
#if defined(__CM3_REV) && (__CM3_REV < 0x0201U)            /* core r2p1 */
#define SCB_VTOR_TBLBASE_Pos 29U                           /*!< SCB VTOR: TBLBASE Position */
#define SCB_VTOR_TBLBASE_Msk (1UL << SCB_VTOR_TBLBASE_Pos) /*!< SCB VTOR: TBLBASE Mask */

#define SCB_VTOR_TBLOFF_Pos 7U                                  /*!< SCB VTOR: TBLOFF Position */
#define SCB_VTOR_TBLOFF_Msk (0x3FFFFFUL << SCB_VTOR_TBLOFF_Pos) /*!< SCB VTOR: TBLOFF Mask */
#else
#define SCB_VTOR_TBLOFF_Pos 7U                                   /*!< SCB VTOR: TBLOFF Position */
#define SCB_VTOR_TBLOFF_Msk (0x1FFFFFFUL << SCB_VTOR_TBLOFF_Pos) /*!< SCB VTOR: TBLOFF Mask */
#endif

/* SCB Application Interrupt and Reset Control Register Definitions */
#define SCB_AIRCR_VECTKEY_Pos 16U                                 /*!< SCB AIRCR: VECTKEY Position */
#define SCB_AIRCR_VECTKEY_Msk (0xFFFFUL << SCB_AIRCR_VECTKEY_Pos) /*!< SCB AIRCR: VECTKEY Mask */

#define SCB_AIRCR_VECTKEYSTAT_Pos 16U                                     /*!< SCB AIRCR: VECTKEYSTAT Position */
#define SCB_AIRCR_VECTKEYSTAT_Msk (0xFFFFUL << SCB_AIRCR_VECTKEYSTAT_Pos) /*!< SCB AIRCR: VECTKEYSTAT Mask */

#define SCB_AIRCR_ENDIANESS_Pos 15U                              /*!< SCB AIRCR: ENDIANESS Position */
#define SCB_AIRCR_ENDIANESS_Msk (1UL << SCB_AIRCR_ENDIANESS_Pos) /*!< SCB AIRCR: ENDIANESS Mask */

#define SCB_AIRCR_PRIGROUP_Pos 8U                              /*!< SCB AIRCR: PRIGROUP Position */
#define SCB_AIRCR_PRIGROUP_Msk (7UL << SCB_AIRCR_PRIGROUP_Pos) /*!< SCB AIRCR: PRIGROUP Mask */

#define SCB_AIRCR_SYSRESETREQ_Pos 2U                                 /*!< SCB AIRCR: SYSRESETREQ Position */
#define SCB_AIRCR_SYSRESETREQ_Msk (1UL << SCB_AIRCR_SYSRESETREQ_Pos) /*!< SCB AIRCR: SYSRESETREQ Mask */

#define SCB_AIRCR_VECTCLRACTIVE_Pos 1U                                   /*!< SCB AIRCR: VECTCLRACTIVE Position */
#define SCB_AIRCR_VECTCLRACTIVE_Msk (1UL << SCB_AIRCR_VECTCLRACTIVE_Pos) /*!< SCB AIRCR: VECTCLRACTIVE Mask */

#define SCB_AIRCR_VECTRESET_Pos 0U                                   /*!< SCB AIRCR: VECTRESET Position */
#define SCB_AIRCR_VECTRESET_Msk (1UL /*<< SCB_AIRCR_VECTRESET_Pos*/) /*!< SCB AIRCR: VECTRESET Mask */

/* SCB System Control Register Definitions */
#define SCB_SCR_SEVONPEND_Pos 4U                             /*!< SCB SCR: SEVONPEND Position */
#define SCB_SCR_SEVONPEND_Msk (1UL << SCB_SCR_SEVONPEND_Pos) /*!< SCB SCR: SEVONPEND Mask */

#define SCB_SCR_SLEEPDEEP_Pos 2U                             /*!< SCB SCR: SLEEPDEEP Position */
#define SCB_SCR_SLEEPDEEP_Msk (1UL << SCB_SCR_SLEEPDEEP_Pos) /*!< SCB SCR: SLEEPDEEP Mask */

#define SCB_SCR_SLEEPONEXIT_Pos 1U                               /*!< SCB SCR: SLEEPONEXIT Position */
#define SCB_SCR_SLEEPONEXIT_Msk (1UL << SCB_SCR_SLEEPONEXIT_Pos) /*!< SCB SCR: SLEEPONEXIT Mask */

/* SCB Configuration Control Register Definitions */
#define SCB_CCR_STKALIGN_Pos 9U                            /*!< SCB CCR: STKALIGN Position */
#define SCB_CCR_STKALIGN_Msk (1UL << SCB_CCR_STKALIGN_Pos) /*!< SCB CCR: STKALIGN Mask */

#define SCB_CCR_BFHFNMIGN_Pos 8U                             /*!< SCB CCR: BFHFNMIGN Position */
#define SCB_CCR_BFHFNMIGN_Msk (1UL << SCB_CCR_BFHFNMIGN_Pos) /*!< SCB CCR: BFHFNMIGN Mask */

#define SCB_CCR_DIV_0_TRP_Pos 4U                             /*!< SCB CCR: DIV_0_TRP Position */
#define SCB_CCR_DIV_0_TRP_Msk (1UL << SCB_CCR_DIV_0_TRP_Pos) /*!< SCB CCR: DIV_0_TRP Mask */

#define SCB_CCR_UNALIGN_TRP_Pos 3U                               /*!< SCB CCR: UNALIGN_TRP Position */
#define SCB_CCR_UNALIGN_TRP_Msk (1UL << SCB_CCR_UNALIGN_TRP_Pos) /*!< SCB CCR: UNALIGN_TRP Mask */

#define SCB_CCR_USERSETMPEND_Pos 1U                                /*!< SCB CCR: USERSETMPEND Position */
#define SCB_CCR_USERSETMPEND_Msk (1UL << SCB_CCR_USERSETMPEND_Pos) /*!< SCB CCR: USERSETMPEND Mask */

#define SCB_CCR_NONBASETHRDENA_Pos 0U                                      /*!< SCB CCR: NONBASETHRDENA Position */
#define SCB_CCR_NONBASETHRDENA_Msk (1UL /*<< SCB_CCR_NONBASETHRDENA_Pos*/) /*!< SCB CCR: NONBASETHRDENA Mask */

/* SCB System Handler Control and State Register Definitions */
#define SCB_SHCSR_USGFAULTENA_Pos 18U                                /*!< SCB SHCSR: USGFAULTENA Position */
#define SCB_SHCSR_USGFAULTENA_Msk (1UL << SCB_SHCSR_USGFAULTENA_Pos) /*!< SCB SHCSR: USGFAULTENA Mask */

#define SCB_SHCSR_BUSFAULTENA_Pos 17U                                /*!< SCB SHCSR: BUSFAULTENA Position */
#define SCB_SHCSR_BUSFAULTENA_Msk (1UL << SCB_SHCSR_BUSFAULTENA_Pos) /*!< SCB SHCSR: BUSFAULTENA Mask */

#define SCB_SHCSR_MEMFAULTENA_Pos 16U                                /*!< SCB SHCSR: MEMFAULTENA Position */
#define SCB_SHCSR_MEMFAULTENA_Msk (1UL << SCB_SHCSR_MEMFAULTENA_Pos) /*!< SCB SHCSR: MEMFAULTENA Mask */

#define SCB_SHCSR_SVCALLPENDED_Pos 15U                                 /*!< SCB SHCSR: SVCALLPENDED Position */
#define SCB_SHCSR_SVCALLPENDED_Msk (1UL << SCB_SHCSR_SVCALLPENDED_Pos) /*!< SCB SHCSR: SVCALLPENDED Mask */

#define SCB_SHCSR_BUSFAULTPENDED_Pos 14U                                   /*!< SCB SHCSR: BUSFAULTPENDED Position */
#define SCB_SHCSR_BUSFAULTPENDED_Msk (1UL << SCB_SHCSR_BUSFAULTPENDED_Pos) /*!< SCB SHCSR: BUSFAULTPENDED Mask */

#define SCB_SHCSR_MEMFAULTPENDED_Pos 13U                                   /*!< SCB SHCSR: MEMFAULTPENDED Position */
#define SCB_SHCSR_MEMFAULTPENDED_Msk (1UL << SCB_SHCSR_MEMFAULTPENDED_Pos) /*!< SCB SHCSR: MEMFAULTPENDED Mask */

#define SCB_SHCSR_USGFAULTPENDED_Pos 12U                                   /*!< SCB SHCSR: USGFAULTPENDED Position */
#define SCB_SHCSR_USGFAULTPENDED_Msk (1UL << SCB_SHCSR_USGFAULTPENDED_Pos) /*!< SCB SHCSR: USGFAULTPENDED Mask */

#define SCB_SHCSR_SYSTICKACT_Pos 11U                               /*!< SCB SHCSR: SYSTICKACT Position */
#define SCB_SHCSR_SYSTICKACT_Msk (1UL << SCB_SHCSR_SYSTICKACT_Pos) /*!< SCB SHCSR: SYSTICKACT Mask */

#define SCB_SHCSR_PENDSVACT_Pos 10U                              /*!< SCB SHCSR: PENDSVACT Position */
#define SCB_SHCSR_PENDSVACT_Msk (1UL << SCB_SHCSR_PENDSVACT_Pos) /*!< SCB SHCSR: PENDSVACT Mask */

#define SCB_SHCSR_MONITORACT_Pos 8U                                /*!< SCB SHCSR: MONITORACT Position */
#define SCB_SHCSR_MONITORACT_Msk (1UL << SCB_SHCSR_MONITORACT_Pos) /*!< SCB SHCSR: MONITORACT Mask */

#define SCB_SHCSR_SVCALLACT_Pos 7U                               /*!< SCB SHCSR: SVCALLACT Position */
#define SCB_SHCSR_SVCALLACT_Msk (1UL << SCB_SHCSR_SVCALLACT_Pos) /*!< SCB SHCSR: SVCALLACT Mask */

#define SCB_SHCSR_USGFAULTACT_Pos 3U                                 /*!< SCB SHCSR: USGFAULTACT Position */
#define SCB_SHCSR_USGFAULTACT_Msk (1UL << SCB_SHCSR_USGFAULTACT_Pos) /*!< SCB SHCSR: USGFAULTACT Mask */

#define SCB_SHCSR_BUSFAULTACT_Pos 1U                                 /*!< SCB SHCSR: BUSFAULTACT Position */
#define SCB_SHCSR_BUSFAULTACT_Msk (1UL << SCB_SHCSR_BUSFAULTACT_Pos) /*!< SCB SHCSR: BUSFAULTACT Mask */

#define SCB_SHCSR_MEMFAULTACT_Pos 0U                                     /*!< SCB SHCSR: MEMFAULTACT Position */
#define SCB_SHCSR_MEMFAULTACT_Msk (1UL /*<< SCB_SHCSR_MEMFAULTACT_Pos*/) /*!< SCB SHCSR: MEMFAULTACT Mask */

/* SCB Configurable Fault Status Register Definitions */
#define SCB_CFSR_USGFAULTSR_Pos 16U                                   /*!< SCB CFSR: Usage Fault Status Register Position */
#define SCB_CFSR_USGFAULTSR_Msk (0xFFFFUL << SCB_CFSR_USGFAULTSR_Pos) /*!< SCB CFSR: Usage Fault Status Register Mask */

#define SCB_CFSR_BUSFAULTSR_Pos 8U                                  /*!< SCB CFSR: Bus Fault Status Register Position */
#define SCB_CFSR_BUSFAULTSR_Msk (0xFFUL << SCB_CFSR_BUSFAULTSR_Pos) /*!< SCB CFSR: Bus Fault Status Register Mask */

#define SCB_CFSR_MEMFAULTSR_Pos 0U                                      /*!< SCB CFSR: Memory Manage Fault Status Register Position */
#define SCB_CFSR_MEMFAULTSR_Msk (0xFFUL /*<< SCB_CFSR_MEMFAULTSR_Pos*/) /*!< SCB CFSR: Memory Manage Fault Status Register Mask */

/* MemManage Fault Status Register (part of SCB Configurable Fault Status Register) */
#define SCB_CFSR_MMARVALID_Pos (SCB_SHCSR_MEMFAULTACT_Pos + 7U) /*!< SCB CFSR (MMFSR): MMARVALID Position */
#define SCB_CFSR_MMARVALID_Msk (1UL << SCB_CFSR_MMARVALID_Pos)  /*!< SCB CFSR (MMFSR): MMARVALID Mask */

#define SCB_CFSR_MSTKERR_Pos (SCB_SHCSR_MEMFAULTACT_Pos + 4U) /*!< SCB CFSR (MMFSR): MSTKERR Position */
#define SCB_CFSR_MSTKERR_Msk (1UL << SCB_CFSR_MSTKERR_Pos)    /*!< SCB CFSR (MMFSR): MSTKERR Mask */

#define SCB_CFSR_MUNSTKERR_Pos (SCB_SHCSR_MEMFAULTACT_Pos + 3U) /*!< SCB CFSR (MMFSR): MUNSTKERR Position */
#define SCB_CFSR_MUNSTKERR_Msk (1UL << SCB_CFSR_MUNSTKERR_Pos)  /*!< SCB CFSR (MMFSR): MUNSTKERR Mask */

#define SCB_CFSR_DACCVIOL_Pos (SCB_SHCSR_MEMFAULTACT_Pos + 1U) /*!< SCB CFSR (MMFSR): DACCVIOL Position */
#define SCB_CFSR_DACCVIOL_Msk (1UL << SCB_CFSR_DACCVIOL_Pos)   /*!< SCB CFSR (MMFSR): DACCVIOL Mask */

#define SCB_CFSR_IACCVIOL_Pos (SCB_SHCSR_MEMFAULTACT_Pos + 0U)   /*!< SCB CFSR (MMFSR): IACCVIOL Position */
#define SCB_CFSR_IACCVIOL_Msk (1UL /*<< SCB_CFSR_IACCVIOL_Pos*/) /*!< SCB CFSR (MMFSR): IACCVIOL Mask */

/* BusFault Status Register (part of SCB Configurable Fault Status Register) */
#define SCB_CFSR_BFARVALID_Pos (SCB_CFSR_BUSFAULTSR_Pos + 7U)  /*!< SCB CFSR (BFSR): BFARVALID Position */
#define SCB_CFSR_BFARVALID_Msk (1UL << SCB_CFSR_BFARVALID_Pos) /*!< SCB CFSR (BFSR): BFARVALID Mask */

#define SCB_CFSR_STKERR_Pos (SCB_CFSR_BUSFAULTSR_Pos + 4U) /*!< SCB CFSR (BFSR): STKERR Position */
#define SCB_CFSR_STKERR_Msk (1UL << SCB_CFSR_STKERR_Pos)   /*!< SCB CFSR (BFSR): STKERR Mask */

#define SCB_CFSR_UNSTKERR_Pos (SCB_CFSR_BUSFAULTSR_Pos + 3U) /*!< SCB CFSR (BFSR): UNSTKERR Position */
#define SCB_CFSR_UNSTKERR_Msk (1UL << SCB_CFSR_UNSTKERR_Pos) /*!< SCB CFSR (BFSR): UNSTKERR Mask */

#define SCB_CFSR_IMPRECISERR_Pos (SCB_CFSR_BUSFAULTSR_Pos + 2U)    /*!< SCB CFSR (BFSR): IMPRECISERR Position */
#define SCB_CFSR_IMPRECISERR_Msk (1UL << SCB_CFSR_IMPRECISERR_Pos) /*!< SCB CFSR (BFSR): IMPRECISERR Mask */

#define SCB_CFSR_PRECISERR_Pos (SCB_CFSR_BUSFAULTSR_Pos + 1U)  /*!< SCB CFSR (BFSR): PRECISERR Position */
#define SCB_CFSR_PRECISERR_Msk (1UL << SCB_CFSR_PRECISERR_Pos) /*!< SCB CFSR (BFSR): PRECISERR Mask */

#define SCB_CFSR_IBUSERR_Pos (SCB_CFSR_BUSFAULTSR_Pos + 0U) /*!< SCB CFSR (BFSR): IBUSERR Position */
#define SCB_CFSR_IBUSERR_Msk (1UL << SCB_CFSR_IBUSERR_Pos)  /*!< SCB CFSR (BFSR): IBUSERR Mask */

/* UsageFault Status Register (part of SCB Configurable Fault Status Register) */
#define SCB_CFSR_DIVBYZERO_Pos (SCB_CFSR_USGFAULTSR_Pos + 9U)  /*!< SCB CFSR (UFSR): DIVBYZERO Position */
#define SCB_CFSR_DIVBYZERO_Msk (1UL << SCB_CFSR_DIVBYZERO_Pos) /*!< SCB CFSR (UFSR): DIVBYZERO Mask */

#define SCB_CFSR_UNALIGNED_Pos (SCB_CFSR_USGFAULTSR_Pos + 8U)  /*!< SCB CFSR (UFSR): UNALIGNED Position */
#define SCB_CFSR_UNALIGNED_Msk (1UL << SCB_CFSR_UNALIGNED_Pos) /*!< SCB CFSR (UFSR): UNALIGNED Mask */

#define SCB_CFSR_NOCP_Pos (SCB_CFSR_USGFAULTSR_Pos + 3U) /*!< SCB CFSR (UFSR): NOCP Position */
#define SCB_CFSR_NOCP_Msk (1UL << SCB_CFSR_NOCP_Pos)     /*!< SCB CFSR (UFSR): NOCP Mask */

#define SCB_CFSR_INVPC_Pos (SCB_CFSR_USGFAULTSR_Pos + 2U) /*!< SCB CFSR (UFSR): INVPC Position */
#define SCB_CFSR_INVPC_Msk (1UL << SCB_CFSR_INVPC_Pos)    /*!< SCB CFSR (UFSR): INVPC Mask */

#define SCB_CFSR_INVSTATE_Pos (SCB_CFSR_USGFAULTSR_Pos + 1U) /*!< SCB CFSR (UFSR): INVSTATE Position */
#define SCB_CFSR_INVSTATE_Msk (1UL << SCB_CFSR_INVSTATE_Pos) /*!< SCB CFSR (UFSR): INVSTATE Mask */

#define SCB_CFSR_UNDEFINSTR_Pos (SCB_CFSR_USGFAULTSR_Pos + 0U)   /*!< SCB CFSR (UFSR): UNDEFINSTR Position */
#define SCB_CFSR_UNDEFINSTR_Msk (1UL << SCB_CFSR_UNDEFINSTR_Pos) /*!< SCB CFSR (UFSR): UNDEFINSTR Mask */

/* SCB Hard Fault Status Register Definitions */
#define SCB_HFSR_DEBUGEVT_Pos 31U                            /*!< SCB HFSR: DEBUGEVT Position */
#define SCB_HFSR_DEBUGEVT_Msk (1UL << SCB_HFSR_DEBUGEVT_Pos) /*!< SCB HFSR: DEBUGEVT Mask */

#define SCB_HFSR_FORCED_Pos 30U                          /*!< SCB HFSR: FORCED Position */
#define SCB_HFSR_FORCED_Msk (1UL << SCB_HFSR_FORCED_Pos) /*!< SCB HFSR: FORCED Mask */

#define SCB_HFSR_VECTTBL_Pos 1U                            /*!< SCB HFSR: VECTTBL Position */
#define SCB_HFSR_VECTTBL_Msk (1UL << SCB_HFSR_VECTTBL_Pos) /*!< SCB HFSR: VECTTBL Mask */

/* SCB Debug Fault Status Register Definitions */
#define SCB_DFSR_EXTERNAL_Pos 4U                             /*!< SCB DFSR: EXTERNAL Position */
#define SCB_DFSR_EXTERNAL_Msk (1UL << SCB_DFSR_EXTERNAL_Pos) /*!< SCB DFSR: EXTERNAL Mask */

#define SCB_DFSR_VCATCH_Pos 3U                           /*!< SCB DFSR: VCATCH Position */
#define SCB_DFSR_VCATCH_Msk (1UL << SCB_DFSR_VCATCH_Pos) /*!< SCB DFSR: VCATCH Mask */

#define SCB_DFSR_DWTTRAP_Pos 2U                            /*!< SCB DFSR: DWTTRAP Position */
#define SCB_DFSR_DWTTRAP_Msk (1UL << SCB_DFSR_DWTTRAP_Pos) /*!< SCB DFSR: DWTTRAP Mask */

#define SCB_DFSR_BKPT_Pos 1U                         /*!< SCB DFSR: BKPT Position */
#define SCB_DFSR_BKPT_Msk (1UL << SCB_DFSR_BKPT_Pos) /*!< SCB DFSR: BKPT Mask */

#define SCB_DFSR_HALTED_Pos 0U                               /*!< SCB DFSR: HALTED Position */
#define SCB_DFSR_HALTED_Msk (1UL /*<< SCB_DFSR_HALTED_Pos*/) /*!< SCB DFSR: HALTED Mask */

/*@} end of group CMSIS_SCB */
//
typedef struct
{
  uint32_t RESERVED0[1U];
  __IM uint32_t ICTR; /*!< Offset: 0x004 (R/ )  Interrupt Controller Type Register */
#if defined(__CM3_REV) && (__CM3_REV >= 0x200U)
  __IOM uint32_t ACTLR; /*!< Offset: 0x008 (R/W)  Auxiliary Control Register */
#else
  uint32_t RESERVED1[1U];
#endif
} SCnSCB_Type;

/* Interrupt Controller Type Register Definitions */
#define SCnSCB_ICTR_INTLINESNUM_Pos 0U                                         /*!< ICTR: INTLINESNUM Position */
#define SCnSCB_ICTR_INTLINESNUM_Msk (0xFUL /*<< SCnSCB_ICTR_INTLINESNUM_Pos*/) /*!< ICTR: INTLINESNUM Mask */

/* Auxiliary Control Register Definitions */

#define SCnSCB_ACTLR_DISFOLD_Pos 2U                                /*!< ACTLR: DISFOLD Position */
#define SCnSCB_ACTLR_DISFOLD_Msk (1UL << SCnSCB_ACTLR_DISFOLD_Pos) /*!< ACTLR: DISFOLD Mask */

#define SCnSCB_ACTLR_DISDEFWBUF_Pos 1U                                   /*!< ACTLR: DISDEFWBUF Position */
#define SCnSCB_ACTLR_DISDEFWBUF_Msk (1UL << SCnSCB_ACTLR_DISDEFWBUF_Pos) /*!< ACTLR: DISDEFWBUF Mask */

#define SCnSCB_ACTLR_DISMCYCINT_Pos 0U                                       /*!< ACTLR: DISMCYCINT Position */
#define SCnSCB_ACTLR_DISMCYCINT_Msk (1UL /*<< SCnSCB_ACTLR_DISMCYCINT_Pos*/) /*!< ACTLR: DISMCYCINT Mask */

/*@} end of group CMSIS_SCnotSCB */

typedef struct
{
  __IOM uint32_t CTRL; /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
  __IOM uint32_t LOAD; /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register */
  __IOM uint32_t VAL;  /*!< Offset: 0x008 (R/W)  SysTick Current Value Register */
  __IM uint32_t CALIB; /*!< Offset: 0x00C (R/ )  SysTick Calibration Register */
} SysTick_Type;

/* SysTick Control / Status Register Definitions */
#define SysTick_CTRL_COUNTFLAG_Pos 16U                                 /*!< SysTick CTRL: COUNTFLAG Position */
#define SysTick_CTRL_COUNTFLAG_Msk (1UL << SysTick_CTRL_COUNTFLAG_Pos) /*!< SysTick CTRL: COUNTFLAG Mask */

#define SysTick_CTRL_CLKSOURCE_Pos 2U                                  /*!< SysTick CTRL: CLKSOURCE Position */
#define SysTick_CTRL_CLKSOURCE_Msk (1UL << SysTick_CTRL_CLKSOURCE_Pos) /*!< SysTick CTRL: CLKSOURCE Mask */

#define SysTick_CTRL_TICKINT_Pos 1U                                /*!< SysTick CTRL: TICKINT Position */
#define SysTick_CTRL_TICKINT_Msk (1UL << SysTick_CTRL_TICKINT_Pos) /*!< SysTick CTRL: TICKINT Mask */

#define SysTick_CTRL_ENABLE_Pos 0U                                   /*!< SysTick CTRL: ENABLE Position */
#define SysTick_CTRL_ENABLE_Msk (1UL /*<< SysTick_CTRL_ENABLE_Pos*/) /*!< SysTick CTRL: ENABLE Mask */

/* SysTick Reload Register Definitions */
#define SysTick_LOAD_RELOAD_Pos 0U                                          /*!< SysTick LOAD: RELOAD Position */
#define SysTick_LOAD_RELOAD_Msk (0xFFFFFFUL /*<< SysTick_LOAD_RELOAD_Pos*/) /*!< SysTick LOAD: RELOAD Mask */

/* SysTick Current Register Definitions */
#define SysTick_VAL_CURRENT_Pos 0U                                          /*!< SysTick VAL: CURRENT Position */
#define SysTick_VAL_CURRENT_Msk (0xFFFFFFUL /*<< SysTick_VAL_CURRENT_Pos*/) /*!< SysTick VAL: CURRENT Mask */

/* SysTick Calibration Register Definitions */
#define SysTick_CALIB_NOREF_Pos 31U                              /*!< SysTick CALIB: NOREF Position */
#define SysTick_CALIB_NOREF_Msk (1UL << SysTick_CALIB_NOREF_Pos) /*!< SysTick CALIB: NOREF Mask */

#define SysTick_CALIB_SKEW_Pos 30U                             /*!< SysTick CALIB: SKEW Position */
#define SysTick_CALIB_SKEW_Msk (1UL << SysTick_CALIB_SKEW_Pos) /*!< SysTick CALIB: SKEW Mask */

#define SysTick_CALIB_TENMS_Pos 0U                                          /*!< SysTick CALIB: TENMS Position */
#define SysTick_CALIB_TENMS_Msk (0xFFFFFFUL /*<< SysTick_CALIB_TENMS_Pos*/) /*!< SysTick CALIB: TENMS Mask */

/*@} end of group CMSIS_SysTick */

typedef struct
{
  __OM union
  {
    __OM uint8_t u8;   /*!< Offset: 0x000 ( /W)  ITM Stimulus Port 8-bit */
    __OM uint16_t u16; /*!< Offset: 0x000 ( /W)  ITM Stimulus Port 16-bit */
    __OM uint32_t u32; /*!< Offset: 0x000 ( /W)  ITM Stimulus Port 32-bit */
  } PORT[32U];         /*!< Offset: 0x000 ( /W)  ITM Stimulus Port Registers */
  uint32_t RESERVED0[864U];
  __IOM uint32_t TER; /*!< Offset: 0xE00 (R/W)  ITM Trace Enable Register */
  uint32_t RESERVED1[15U];
  __IOM uint32_t TPR; /*!< Offset: 0xE40 (R/W)  ITM Trace Privilege Register */
  uint32_t RESERVED2[15U];
  __IOM uint32_t TCR; /*!< Offset: 0xE80 (R/W)  ITM Trace Control Register */
  uint32_t RESERVED3[29U];
  __OM uint32_t IWR;   /*!< Offset: 0xEF8 ( /W)  ITM Integration Write Register */
  __IM uint32_t IRR;   /*!< Offset: 0xEFC (R/ )  ITM Integration Read Register */
  __IOM uint32_t IMCR; /*!< Offset: 0xF00 (R/W)  ITM Integration Mode Control Register */
  uint32_t RESERVED4[43U];
  __OM uint32_t LAR; /*!< Offset: 0xFB0 ( /W)  ITM Lock Access Register */
  __IM uint32_t LSR; /*!< Offset: 0xFB4 (R/ )  ITM Lock Status Register */
  uint32_t RESERVED5[6U];
  __IM uint32_t PID4; /*!< Offset: 0xFD0 (R/ )  ITM Peripheral Identification Register #4 */
  __IM uint32_t PID5; /*!< Offset: 0xFD4 (R/ )  ITM Peripheral Identification Register #5 */
  __IM uint32_t PID6; /*!< Offset: 0xFD8 (R/ )  ITM Peripheral Identification Register #6 */
  __IM uint32_t PID7; /*!< Offset: 0xFDC (R/ )  ITM Peripheral Identification Register #7 */
  __IM uint32_t PID0; /*!< Offset: 0xFE0 (R/ )  ITM Peripheral Identification Register #0 */
  __IM uint32_t PID1; /*!< Offset: 0xFE4 (R/ )  ITM Peripheral Identification Register #1 */
  __IM uint32_t PID2; /*!< Offset: 0xFE8 (R/ )  ITM Peripheral Identification Register #2 */
  __IM uint32_t PID3; /*!< Offset: 0xFEC (R/ )  ITM Peripheral Identification Register #3 */
  __IM uint32_t CID0; /*!< Offset: 0xFF0 (R/ )  ITM Component  Identification Register #0 */
  __IM uint32_t CID1; /*!< Offset: 0xFF4 (R/ )  ITM Component  Identification Register #1 */
  __IM uint32_t CID2; /*!< Offset: 0xFF8 (R/ )  ITM Component  Identification Register #2 */
  __IM uint32_t CID3; /*!< Offset: 0xFFC (R/ )  ITM Component  Identification Register #3 */
} ITM_Type;

/* ITM Trace Privilege Register Definitions */
#define ITM_TPR_PRIVMASK_Pos 0U                                         /*!< ITM TPR: PRIVMASK Position */
#define ITM_TPR_PRIVMASK_Msk (0xFFFFFFFFUL /*<< ITM_TPR_PRIVMASK_Pos*/) /*!< ITM TPR: PRIVMASK Mask */

/* ITM Trace Control Register Definitions */
#define ITM_TCR_BUSY_Pos 23U                       /*!< ITM TCR: BUSY Position */
#define ITM_TCR_BUSY_Msk (1UL << ITM_TCR_BUSY_Pos) /*!< ITM TCR: BUSY Mask */

#define ITM_TCR_TraceBusID_Pos 16U                                /*!< ITM TCR: ATBID Position */
#define ITM_TCR_TraceBusID_Msk (0x7FUL << ITM_TCR_TraceBusID_Pos) /*!< ITM TCR: ATBID Mask */

#define ITM_TCR_GTSFREQ_Pos 10U                          /*!< ITM TCR: Global timestamp frequency Position */
#define ITM_TCR_GTSFREQ_Msk (3UL << ITM_TCR_GTSFREQ_Pos) /*!< ITM TCR: Global timestamp frequency Mask */

#define ITM_TCR_TSPrescale_Pos 8U                              /*!< ITM TCR: TSPrescale Position */
#define ITM_TCR_TSPrescale_Msk (3UL << ITM_TCR_TSPrescale_Pos) /*!< ITM TCR: TSPrescale Mask */

#define ITM_TCR_SWOENA_Pos 4U                          /*!< ITM TCR: SWOENA Position */
#define ITM_TCR_SWOENA_Msk (1UL << ITM_TCR_SWOENA_Pos) /*!< ITM TCR: SWOENA Mask */

#define ITM_TCR_DWTENA_Pos 3U                          /*!< ITM TCR: DWTENA Position */
#define ITM_TCR_DWTENA_Msk (1UL << ITM_TCR_DWTENA_Pos) /*!< ITM TCR: DWTENA Mask */

#define ITM_TCR_SYNCENA_Pos 2U                           /*!< ITM TCR: SYNCENA Position */
#define ITM_TCR_SYNCENA_Msk (1UL << ITM_TCR_SYNCENA_Pos) /*!< ITM TCR: SYNCENA Mask */

#define ITM_TCR_TSENA_Pos 1U                         /*!< ITM TCR: TSENA Position */
#define ITM_TCR_TSENA_Msk (1UL << ITM_TCR_TSENA_Pos) /*!< ITM TCR: TSENA Mask */

#define ITM_TCR_ITMENA_Pos 0U                              /*!< ITM TCR: ITM Enable bit Position */
#define ITM_TCR_ITMENA_Msk (1UL /*<< ITM_TCR_ITMENA_Pos*/) /*!< ITM TCR: ITM Enable bit Mask */

/* ITM Integration Write Register Definitions */
#define ITM_IWR_ATVALIDM_Pos 0U                                /*!< ITM IWR: ATVALIDM Position */
#define ITM_IWR_ATVALIDM_Msk (1UL /*<< ITM_IWR_ATVALIDM_Pos*/) /*!< ITM IWR: ATVALIDM Mask */

/* ITM Integration Read Register Definitions */
#define ITM_IRR_ATREADYM_Pos 0U                                /*!< ITM IRR: ATREADYM Position */
#define ITM_IRR_ATREADYM_Msk (1UL /*<< ITM_IRR_ATREADYM_Pos*/) /*!< ITM IRR: ATREADYM Mask */

/* ITM Integration Mode Control Register Definitions */
#define ITM_IMCR_INTEGRATION_Pos 0U                                    /*!< ITM IMCR: INTEGRATION Position */
#define ITM_IMCR_INTEGRATION_Msk (1UL /*<< ITM_IMCR_INTEGRATION_Pos*/) /*!< ITM IMCR: INTEGRATION Mask */

/* ITM Lock Status Register Definitions */
#define ITM_LSR_ByteAcc_Pos 2U                           /*!< ITM LSR: ByteAcc Position */
#define ITM_LSR_ByteAcc_Msk (1UL << ITM_LSR_ByteAcc_Pos) /*!< ITM LSR: ByteAcc Mask */

#define ITM_LSR_Access_Pos 1U                          /*!< ITM LSR: Access Position */
#define ITM_LSR_Access_Msk (1UL << ITM_LSR_Access_Pos) /*!< ITM LSR: Access Mask */

#define ITM_LSR_Present_Pos 0U                               /*!< ITM LSR: Present Position */
#define ITM_LSR_Present_Msk (1UL /*<< ITM_LSR_Present_Pos*/) /*!< ITM LSR: Present Mask */

/*@}*/ /* end of group CMSIS_ITM */

typedef struct
{
  __IOM uint32_t CTRL;      /*!< Offset: 0x000 (R/W)  Control Register */
  __IOM uint32_t CYCCNT;    /*!< Offset: 0x004 (R/W)  Cycle Count Register */
  __IOM uint32_t CPICNT;    /*!< Offset: 0x008 (R/W)  CPI Count Register */
  __IOM uint32_t EXCCNT;    /*!< Offset: 0x00C (R/W)  Exception Overhead Count Register */
  __IOM uint32_t SLEEPCNT;  /*!< Offset: 0x010 (R/W)  Sleep Count Register */
  __IOM uint32_t LSUCNT;    /*!< Offset: 0x014 (R/W)  LSU Count Register */
  __IOM uint32_t FOLDCNT;   /*!< Offset: 0x018 (R/W)  Folded-instruction Count Register */
  __IM uint32_t PCSR;       /*!< Offset: 0x01C (R/ )  Program Counter Sample Register */
  __IOM uint32_t COMP0;     /*!< Offset: 0x020 (R/W)  Comparator Register 0 */
  __IOM uint32_t MASK0;     /*!< Offset: 0x024 (R/W)  Mask Register 0 */
  __IOM uint32_t FUNCTION0; /*!< Offset: 0x028 (R/W)  Function Register 0 */
  uint32_t RESERVED0[1U];
  __IOM uint32_t COMP1;     /*!< Offset: 0x030 (R/W)  Comparator Register 1 */
  __IOM uint32_t MASK1;     /*!< Offset: 0x034 (R/W)  Mask Register 1 */
  __IOM uint32_t FUNCTION1; /*!< Offset: 0x038 (R/W)  Function Register 1 */
  uint32_t RESERVED1[1U];
  __IOM uint32_t COMP2;     /*!< Offset: 0x040 (R/W)  Comparator Register 2 */
  __IOM uint32_t MASK2;     /*!< Offset: 0x044 (R/W)  Mask Register 2 */
  __IOM uint32_t FUNCTION2; /*!< Offset: 0x048 (R/W)  Function Register 2 */
  uint32_t RESERVED2[1U];
  __IOM uint32_t COMP3;     /*!< Offset: 0x050 (R/W)  Comparator Register 3 */
  __IOM uint32_t MASK3;     /*!< Offset: 0x054 (R/W)  Mask Register 3 */
  __IOM uint32_t FUNCTION3; /*!< Offset: 0x058 (R/W)  Function Register 3 */
} DWT_Type;

/* DWT Control Register Definitions */
#define DWT_CTRL_NUMCOMP_Pos 28U                             /*!< DWT CTRL: NUMCOMP Position */
#define DWT_CTRL_NUMCOMP_Msk (0xFUL << DWT_CTRL_NUMCOMP_Pos) /*!< DWT CTRL: NUMCOMP Mask */

#define DWT_CTRL_NOTRCPKT_Pos 27U                              /*!< DWT CTRL: NOTRCPKT Position */
#define DWT_CTRL_NOTRCPKT_Msk (0x1UL << DWT_CTRL_NOTRCPKT_Pos) /*!< DWT CTRL: NOTRCPKT Mask */

#define DWT_CTRL_NOEXTTRIG_Pos 26U                               /*!< DWT CTRL: NOEXTTRIG Position */
#define DWT_CTRL_NOEXTTRIG_Msk (0x1UL << DWT_CTRL_NOEXTTRIG_Pos) /*!< DWT CTRL: NOEXTTRIG Mask */

#define DWT_CTRL_NOCYCCNT_Pos 25U                              /*!< DWT CTRL: NOCYCCNT Position */
#define DWT_CTRL_NOCYCCNT_Msk (0x1UL << DWT_CTRL_NOCYCCNT_Pos) /*!< DWT CTRL: NOCYCCNT Mask */

#define DWT_CTRL_NOPRFCNT_Pos 24U                              /*!< DWT CTRL: NOPRFCNT Position */
#define DWT_CTRL_NOPRFCNT_Msk (0x1UL << DWT_CTRL_NOPRFCNT_Pos) /*!< DWT CTRL: NOPRFCNT Mask */

#define DWT_CTRL_CYCEVTENA_Pos 22U                               /*!< DWT CTRL: CYCEVTENA Position */
#define DWT_CTRL_CYCEVTENA_Msk (0x1UL << DWT_CTRL_CYCEVTENA_Pos) /*!< DWT CTRL: CYCEVTENA Mask */

#define DWT_CTRL_FOLDEVTENA_Pos 21U                                /*!< DWT CTRL: FOLDEVTENA Position */
#define DWT_CTRL_FOLDEVTENA_Msk (0x1UL << DWT_CTRL_FOLDEVTENA_Pos) /*!< DWT CTRL: FOLDEVTENA Mask */

#define DWT_CTRL_LSUEVTENA_Pos 20U                               /*!< DWT CTRL: LSUEVTENA Position */
#define DWT_CTRL_LSUEVTENA_Msk (0x1UL << DWT_CTRL_LSUEVTENA_Pos) /*!< DWT CTRL: LSUEVTENA Mask */

#define DWT_CTRL_SLEEPEVTENA_Pos 19U                                 /*!< DWT CTRL: SLEEPEVTENA Position */
#define DWT_CTRL_SLEEPEVTENA_Msk (0x1UL << DWT_CTRL_SLEEPEVTENA_Pos) /*!< DWT CTRL: SLEEPEVTENA Mask */

#define DWT_CTRL_EXCEVTENA_Pos 18U                               /*!< DWT CTRL: EXCEVTENA Position */
#define DWT_CTRL_EXCEVTENA_Msk (0x1UL << DWT_CTRL_EXCEVTENA_Pos) /*!< DWT CTRL: EXCEVTENA Mask */

#define DWT_CTRL_CPIEVTENA_Pos 17U                               /*!< DWT CTRL: CPIEVTENA Position */
#define DWT_CTRL_CPIEVTENA_Msk (0x1UL << DWT_CTRL_CPIEVTENA_Pos) /*!< DWT CTRL: CPIEVTENA Mask */

#define DWT_CTRL_EXCTRCENA_Pos 16U                               /*!< DWT CTRL: EXCTRCENA Position */
#define DWT_CTRL_EXCTRCENA_Msk (0x1UL << DWT_CTRL_EXCTRCENA_Pos) /*!< DWT CTRL: EXCTRCENA Mask */

#define DWT_CTRL_PCSAMPLENA_Pos 12U                                /*!< DWT CTRL: PCSAMPLENA Position */
#define DWT_CTRL_PCSAMPLENA_Msk (0x1UL << DWT_CTRL_PCSAMPLENA_Pos) /*!< DWT CTRL: PCSAMPLENA Mask */

#define DWT_CTRL_SYNCTAP_Pos 10U                             /*!< DWT CTRL: SYNCTAP Position */
#define DWT_CTRL_SYNCTAP_Msk (0x3UL << DWT_CTRL_SYNCTAP_Pos) /*!< DWT CTRL: SYNCTAP Mask */

#define DWT_CTRL_CYCTAP_Pos 9U                             /*!< DWT CTRL: CYCTAP Position */
#define DWT_CTRL_CYCTAP_Msk (0x1UL << DWT_CTRL_CYCTAP_Pos) /*!< DWT CTRL: CYCTAP Mask */

#define DWT_CTRL_POSTINIT_Pos 5U                               /*!< DWT CTRL: POSTINIT Position */
#define DWT_CTRL_POSTINIT_Msk (0xFUL << DWT_CTRL_POSTINIT_Pos) /*!< DWT CTRL: POSTINIT Mask */

#define DWT_CTRL_POSTPRESET_Pos 1U                                 /*!< DWT CTRL: POSTPRESET Position */
#define DWT_CTRL_POSTPRESET_Msk (0xFUL << DWT_CTRL_POSTPRESET_Pos) /*!< DWT CTRL: POSTPRESET Mask */

#define DWT_CTRL_CYCCNTENA_Pos 0U                                    /*!< DWT CTRL: CYCCNTENA Position */
#define DWT_CTRL_CYCCNTENA_Msk (0x1UL /*<< DWT_CTRL_CYCCNTENA_Pos*/) /*!< DWT CTRL: CYCCNTENA Mask */

/* DWT CPI Count Register Definitions */
#define DWT_CPICNT_CPICNT_Pos 0U                                    /*!< DWT CPICNT: CPICNT Position */
#define DWT_CPICNT_CPICNT_Msk (0xFFUL /*<< DWT_CPICNT_CPICNT_Pos*/) /*!< DWT CPICNT: CPICNT Mask */

/* DWT Exception Overhead Count Register Definitions */
#define DWT_EXCCNT_EXCCNT_Pos 0U                                    /*!< DWT EXCCNT: EXCCNT Position */
#define DWT_EXCCNT_EXCCNT_Msk (0xFFUL /*<< DWT_EXCCNT_EXCCNT_Pos*/) /*!< DWT EXCCNT: EXCCNT Mask */

/* DWT Sleep Count Register Definitions */
#define DWT_SLEEPCNT_SLEEPCNT_Pos 0U                                        /*!< DWT SLEEPCNT: SLEEPCNT Position */
#define DWT_SLEEPCNT_SLEEPCNT_Msk (0xFFUL /*<< DWT_SLEEPCNT_SLEEPCNT_Pos*/) /*!< DWT SLEEPCNT: SLEEPCNT Mask */

/* DWT LSU Count Register Definitions */
#define DWT_LSUCNT_LSUCNT_Pos 0U                                    /*!< DWT LSUCNT: LSUCNT Position */
#define DWT_LSUCNT_LSUCNT_Msk (0xFFUL /*<< DWT_LSUCNT_LSUCNT_Pos*/) /*!< DWT LSUCNT: LSUCNT Mask */

/* DWT Folded-instruction Count Register Definitions */
#define DWT_FOLDCNT_FOLDCNT_Pos 0U                                      /*!< DWT FOLDCNT: FOLDCNT Position */
#define DWT_FOLDCNT_FOLDCNT_Msk (0xFFUL /*<< DWT_FOLDCNT_FOLDCNT_Pos*/) /*!< DWT FOLDCNT: FOLDCNT Mask */

/* DWT Comparator Mask Register Definitions */
#define DWT_MASK_MASK_Pos 0U                                /*!< DWT MASK: MASK Position */
#define DWT_MASK_MASK_Msk (0x1FUL /*<< DWT_MASK_MASK_Pos*/) /*!< DWT MASK: MASK Mask */

/* DWT Comparator Function Register Definitions */
#define DWT_FUNCTION_MATCHED_Pos 24U                                 /*!< DWT FUNCTION: MATCHED Position */
#define DWT_FUNCTION_MATCHED_Msk (0x1UL << DWT_FUNCTION_MATCHED_Pos) /*!< DWT FUNCTION: MATCHED Mask */

#define DWT_FUNCTION_DATAVADDR1_Pos 16U                                    /*!< DWT FUNCTION: DATAVADDR1 Position */
#define DWT_FUNCTION_DATAVADDR1_Msk (0xFUL << DWT_FUNCTION_DATAVADDR1_Pos) /*!< DWT FUNCTION: DATAVADDR1 Mask */

#define DWT_FUNCTION_DATAVADDR0_Pos 12U                                    /*!< DWT FUNCTION: DATAVADDR0 Position */
#define DWT_FUNCTION_DATAVADDR0_Msk (0xFUL << DWT_FUNCTION_DATAVADDR0_Pos) /*!< DWT FUNCTION: DATAVADDR0 Mask */

#define DWT_FUNCTION_DATAVSIZE_Pos 10U                                   /*!< DWT FUNCTION: DATAVSIZE Position */
#define DWT_FUNCTION_DATAVSIZE_Msk (0x3UL << DWT_FUNCTION_DATAVSIZE_Pos) /*!< DWT FUNCTION: DATAVSIZE Mask */

#define DWT_FUNCTION_LNK1ENA_Pos 9U                                  /*!< DWT FUNCTION: LNK1ENA Position */
#define DWT_FUNCTION_LNK1ENA_Msk (0x1UL << DWT_FUNCTION_LNK1ENA_Pos) /*!< DWT FUNCTION: LNK1ENA Mask */

#define DWT_FUNCTION_DATAVMATCH_Pos 8U                                     /*!< DWT FUNCTION: DATAVMATCH Position */
#define DWT_FUNCTION_DATAVMATCH_Msk (0x1UL << DWT_FUNCTION_DATAVMATCH_Pos) /*!< DWT FUNCTION: DATAVMATCH Mask */

#define DWT_FUNCTION_CYCMATCH_Pos 7U                                   /*!< DWT FUNCTION: CYCMATCH Position */
#define DWT_FUNCTION_CYCMATCH_Msk (0x1UL << DWT_FUNCTION_CYCMATCH_Pos) /*!< DWT FUNCTION: CYCMATCH Mask */

#define DWT_FUNCTION_EMITRANGE_Pos 5U                                    /*!< DWT FUNCTION: EMITRANGE Position */
#define DWT_FUNCTION_EMITRANGE_Msk (0x1UL << DWT_FUNCTION_EMITRANGE_Pos) /*!< DWT FUNCTION: EMITRANGE Mask */

#define DWT_FUNCTION_FUNCTION_Pos 0U                                       /*!< DWT FUNCTION: FUNCTION Position */
#define DWT_FUNCTION_FUNCTION_Msk (0xFUL /*<< DWT_FUNCTION_FUNCTION_Pos*/) /*!< DWT FUNCTION: FUNCTION Mask */

/*@}*/ /* end of group CMSIS_DWT */

typedef struct
{
  __IM uint32_t SSPSR;  /*!< Offset: 0x000 (R/ )  Supported Parallel Port Size Register */
  __IOM uint32_t CSPSR; /*!< Offset: 0x004 (R/W)  Current Parallel Port Size Register */
  uint32_t RESERVED0[2U];
  __IOM uint32_t ACPR; /*!< Offset: 0x010 (R/W)  Asynchronous Clock Prescaler Register */
  uint32_t RESERVED1[55U];
  __IOM uint32_t SPPR; /*!< Offset: 0x0F0 (R/W)  Selected Pin Protocol Register */
  uint32_t RESERVED2[131U];
  __IM uint32_t FFSR;  /*!< Offset: 0x300 (R/ )  Formatter and Flush Status Register */
  __IOM uint32_t FFCR; /*!< Offset: 0x304 (R/W)  Formatter and Flush Control Register */
  __IM uint32_t FSCR;  /*!< Offset: 0x308 (R/ )  Formatter Synchronization Counter Register */
  uint32_t RESERVED3[759U];
  __IM uint32_t TRIGGER;   /*!< Offset: 0xEE8 (R/ )  TRIGGER Register */
  __IM uint32_t FIFO0;     /*!< Offset: 0xEEC (R/ )  Integration ETM Data */
  __IM uint32_t ITATBCTR2; /*!< Offset: 0xEF0 (R/ )  ITATBCTR2 */
  uint32_t RESERVED4[1U];
  __IM uint32_t ITATBCTR0; /*!< Offset: 0xEF8 (R/ )  ITATBCTR0 */
  __IM uint32_t FIFO1;     /*!< Offset: 0xEFC (R/ )  Integration ITM Data */
  __IOM uint32_t ITCTRL;   /*!< Offset: 0xF00 (R/W)  Integration Mode Control */
  uint32_t RESERVED5[39U];
  __IOM uint32_t CLAIMSET; /*!< Offset: 0xFA0 (R/W)  Claim tag set */
  __IOM uint32_t CLAIMCLR; /*!< Offset: 0xFA4 (R/W)  Claim tag clear */
  uint32_t RESERVED7[8U];
  __IM uint32_t DEVID;   /*!< Offset: 0xFC8 (R/ )  TPIU_DEVID */
  __IM uint32_t DEVTYPE; /*!< Offset: 0xFCC (R/ )  TPIU_DEVTYPE */
} TPI_Type;

/* TPI Asynchronous Clock Prescaler Register Definitions */
#define TPI_ACPR_PRESCALER_Pos 0U                                       /*!< TPI ACPR: PRESCALER Position */
#define TPI_ACPR_PRESCALER_Msk (0x1FFFUL /*<< TPI_ACPR_PRESCALER_Pos*/) /*!< TPI ACPR: PRESCALER Mask */

/* TPI Selected Pin Protocol Register Definitions */
#define TPI_SPPR_TXMODE_Pos 0U                                 /*!< TPI SPPR: TXMODE Position */
#define TPI_SPPR_TXMODE_Msk (0x3UL /*<< TPI_SPPR_TXMODE_Pos*/) /*!< TPI SPPR: TXMODE Mask */

/* TPI Formatter and Flush Status Register Definitions */
#define TPI_FFSR_FtNonStop_Pos 3U                                /*!< TPI FFSR: FtNonStop Position */
#define TPI_FFSR_FtNonStop_Msk (0x1UL << TPI_FFSR_FtNonStop_Pos) /*!< TPI FFSR: FtNonStop Mask */

#define TPI_FFSR_TCPresent_Pos 2U                                /*!< TPI FFSR: TCPresent Position */
#define TPI_FFSR_TCPresent_Msk (0x1UL << TPI_FFSR_TCPresent_Pos) /*!< TPI FFSR: TCPresent Mask */

#define TPI_FFSR_FtStopped_Pos 1U                                /*!< TPI FFSR: FtStopped Position */
#define TPI_FFSR_FtStopped_Msk (0x1UL << TPI_FFSR_FtStopped_Pos) /*!< TPI FFSR: FtStopped Mask */

#define TPI_FFSR_FlInProg_Pos 0U                                   /*!< TPI FFSR: FlInProg Position */
#define TPI_FFSR_FlInProg_Msk (0x1UL /*<< TPI_FFSR_FlInProg_Pos*/) /*!< TPI FFSR: FlInProg Mask */

/* TPI Formatter and Flush Control Register Definitions */
#define TPI_FFCR_TrigIn_Pos 8U                             /*!< TPI FFCR: TrigIn Position */
#define TPI_FFCR_TrigIn_Msk (0x1UL << TPI_FFCR_TrigIn_Pos) /*!< TPI FFCR: TrigIn Mask */

#define TPI_FFCR_EnFCont_Pos 1U                              /*!< TPI FFCR: EnFCont Position */
#define TPI_FFCR_EnFCont_Msk (0x1UL << TPI_FFCR_EnFCont_Pos) /*!< TPI FFCR: EnFCont Mask */

/* TPI TRIGGER Register Definitions */
#define TPI_TRIGGER_TRIGGER_Pos 0U                                     /*!< TPI TRIGGER: TRIGGER Position */
#define TPI_TRIGGER_TRIGGER_Msk (0x1UL /*<< TPI_TRIGGER_TRIGGER_Pos*/) /*!< TPI TRIGGER: TRIGGER Mask */

/* TPI Integration ETM Data Register Definitions (FIFO0) */
#define TPI_FIFO0_ITM_ATVALID_Pos 29U                                  /*!< TPI FIFO0: ITM_ATVALID Position */
#define TPI_FIFO0_ITM_ATVALID_Msk (0x3UL << TPI_FIFO0_ITM_ATVALID_Pos) /*!< TPI FIFO0: ITM_ATVALID Mask */

#define TPI_FIFO0_ITM_bytecount_Pos 27U                                    /*!< TPI FIFO0: ITM_bytecount Position */
#define TPI_FIFO0_ITM_bytecount_Msk (0x3UL << TPI_FIFO0_ITM_bytecount_Pos) /*!< TPI FIFO0: ITM_bytecount Mask */

#define TPI_FIFO0_ETM_ATVALID_Pos 26U                                  /*!< TPI FIFO0: ETM_ATVALID Position */
#define TPI_FIFO0_ETM_ATVALID_Msk (0x3UL << TPI_FIFO0_ETM_ATVALID_Pos) /*!< TPI FIFO0: ETM_ATVALID Mask */

#define TPI_FIFO0_ETM_bytecount_Pos 24U                                    /*!< TPI FIFO0: ETM_bytecount Position */
#define TPI_FIFO0_ETM_bytecount_Msk (0x3UL << TPI_FIFO0_ETM_bytecount_Pos) /*!< TPI FIFO0: ETM_bytecount Mask */

#define TPI_FIFO0_ETM2_Pos 16U                            /*!< TPI FIFO0: ETM2 Position */
#define TPI_FIFO0_ETM2_Msk (0xFFUL << TPI_FIFO0_ETM2_Pos) /*!< TPI FIFO0: ETM2 Mask */

#define TPI_FIFO0_ETM1_Pos 8U                             /*!< TPI FIFO0: ETM1 Position */
#define TPI_FIFO0_ETM1_Msk (0xFFUL << TPI_FIFO0_ETM1_Pos) /*!< TPI FIFO0: ETM1 Mask */

#define TPI_FIFO0_ETM0_Pos 0U                                 /*!< TPI FIFO0: ETM0 Position */
#define TPI_FIFO0_ETM0_Msk (0xFFUL /*<< TPI_FIFO0_ETM0_Pos*/) /*!< TPI FIFO0: ETM0 Mask */

/* TPI ITATBCTR2 Register Definitions */
#define TPI_ITATBCTR2_ATREADY2_Pos 0U                                        /*!< TPI ITATBCTR2: ATREADY2 Position */
#define TPI_ITATBCTR2_ATREADY2_Msk (0x1UL /*<< TPI_ITATBCTR2_ATREADY2_Pos*/) /*!< TPI ITATBCTR2: ATREADY2 Mask */

#define TPI_ITATBCTR2_ATREADY1_Pos 0U                                        /*!< TPI ITATBCTR2: ATREADY1 Position */
#define TPI_ITATBCTR2_ATREADY1_Msk (0x1UL /*<< TPI_ITATBCTR2_ATREADY1_Pos*/) /*!< TPI ITATBCTR2: ATREADY1 Mask */

/* TPI Integration ITM Data Register Definitions (FIFO1) */
#define TPI_FIFO1_ITM_ATVALID_Pos 29U                                  /*!< TPI FIFO1: ITM_ATVALID Position */
#define TPI_FIFO1_ITM_ATVALID_Msk (0x3UL << TPI_FIFO1_ITM_ATVALID_Pos) /*!< TPI FIFO1: ITM_ATVALID Mask */

#define TPI_FIFO1_ITM_bytecount_Pos 27U                                    /*!< TPI FIFO1: ITM_bytecount Position */
#define TPI_FIFO1_ITM_bytecount_Msk (0x3UL << TPI_FIFO1_ITM_bytecount_Pos) /*!< TPI FIFO1: ITM_bytecount Mask */

#define TPI_FIFO1_ETM_ATVALID_Pos 26U                                  /*!< TPI FIFO1: ETM_ATVALID Position */
#define TPI_FIFO1_ETM_ATVALID_Msk (0x3UL << TPI_FIFO1_ETM_ATVALID_Pos) /*!< TPI FIFO1: ETM_ATVALID Mask */

#define TPI_FIFO1_ETM_bytecount_Pos 24U                                    /*!< TPI FIFO1: ETM_bytecount Position */
#define TPI_FIFO1_ETM_bytecount_Msk (0x3UL << TPI_FIFO1_ETM_bytecount_Pos) /*!< TPI FIFO1: ETM_bytecount Mask */

#define TPI_FIFO1_ITM2_Pos 16U                            /*!< TPI FIFO1: ITM2 Position */
#define TPI_FIFO1_ITM2_Msk (0xFFUL << TPI_FIFO1_ITM2_Pos) /*!< TPI FIFO1: ITM2 Mask */

#define TPI_FIFO1_ITM1_Pos 8U                             /*!< TPI FIFO1: ITM1 Position */
#define TPI_FIFO1_ITM1_Msk (0xFFUL << TPI_FIFO1_ITM1_Pos) /*!< TPI FIFO1: ITM1 Mask */

#define TPI_FIFO1_ITM0_Pos 0U                                 /*!< TPI FIFO1: ITM0 Position */
#define TPI_FIFO1_ITM0_Msk (0xFFUL /*<< TPI_FIFO1_ITM0_Pos*/) /*!< TPI FIFO1: ITM0 Mask */

/* TPI ITATBCTR0 Register Definitions */
#define TPI_ITATBCTR0_ATREADY2_Pos 0U                                        /*!< TPI ITATBCTR0: ATREADY2 Position */
#define TPI_ITATBCTR0_ATREADY2_Msk (0x1UL /*<< TPI_ITATBCTR0_ATREADY2_Pos*/) /*!< TPI ITATBCTR0: ATREADY2 Mask */

#define TPI_ITATBCTR0_ATREADY1_Pos 0U                                        /*!< TPI ITATBCTR0: ATREADY1 Position */
#define TPI_ITATBCTR0_ATREADY1_Msk (0x1UL /*<< TPI_ITATBCTR0_ATREADY1_Pos*/) /*!< TPI ITATBCTR0: ATREADY1 Mask */

/* TPI Integration Mode Control Register Definitions */
#define TPI_ITCTRL_Mode_Pos 0U                                 /*!< TPI ITCTRL: Mode Position */
#define TPI_ITCTRL_Mode_Msk (0x3UL /*<< TPI_ITCTRL_Mode_Pos*/) /*!< TPI ITCTRL: Mode Mask */

/* TPI DEVID Register Definitions */
#define TPI_DEVID_NRZVALID_Pos 11U                               /*!< TPI DEVID: NRZVALID Position */
#define TPI_DEVID_NRZVALID_Msk (0x1UL << TPI_DEVID_NRZVALID_Pos) /*!< TPI DEVID: NRZVALID Mask */

#define TPI_DEVID_MANCVALID_Pos 10U                                /*!< TPI DEVID: MANCVALID Position */
#define TPI_DEVID_MANCVALID_Msk (0x1UL << TPI_DEVID_MANCVALID_Pos) /*!< TPI DEVID: MANCVALID Mask */

#define TPI_DEVID_PTINVALID_Pos 9U                                 /*!< TPI DEVID: PTINVALID Position */
#define TPI_DEVID_PTINVALID_Msk (0x1UL << TPI_DEVID_PTINVALID_Pos) /*!< TPI DEVID: PTINVALID Mask */

#define TPI_DEVID_MinBufSz_Pos 6U                                /*!< TPI DEVID: MinBufSz Position */
#define TPI_DEVID_MinBufSz_Msk (0x7UL << TPI_DEVID_MinBufSz_Pos) /*!< TPI DEVID: MinBufSz Mask */

#define TPI_DEVID_AsynClkIn_Pos 5U                                 /*!< TPI DEVID: AsynClkIn Position */
#define TPI_DEVID_AsynClkIn_Msk (0x1UL << TPI_DEVID_AsynClkIn_Pos) /*!< TPI DEVID: AsynClkIn Mask */

#define TPI_DEVID_NrTraceInput_Pos 0U                                         /*!< TPI DEVID: NrTraceInput Position */
#define TPI_DEVID_NrTraceInput_Msk (0x1FUL /*<< TPI_DEVID_NrTraceInput_Pos*/) /*!< TPI DEVID: NrTraceInput Mask */

/* TPI DEVTYPE Register Definitions */
#define TPI_DEVTYPE_SubType_Pos 4U                                     /*!< TPI DEVTYPE: SubType Position */
#define TPI_DEVTYPE_SubType_Msk (0xFUL /*<< TPI_DEVTYPE_SubType_Pos*/) /*!< TPI DEVTYPE: SubType Mask */

#define TPI_DEVTYPE_MajorType_Pos 0U                                   /*!< TPI DEVTYPE: MajorType Position */
#define TPI_DEVTYPE_MajorType_Msk (0xFUL << TPI_DEVTYPE_MajorType_Pos) /*!< TPI DEVTYPE: MajorType Mask */

/*@}*/ /* end of group CMSIS_TPI */

typedef struct
{
  __IOM uint32_t DHCSR; /*!< Offset: 0x000 (R/W)  Debug Halting Control and Status Register */
  __OM uint32_t DCRSR;  /*!< Offset: 0x004 ( /W)  Debug Core Register Selector Register */
  __IOM uint32_t DCRDR; /*!< Offset: 0x008 (R/W)  Debug Core Register Data Register */
  __IOM uint32_t DEMCR; /*!< Offset: 0x00C (R/W)  Debug Exception and Monitor Control Register */
} CoreDebug_Type;

/* Debug Halting Control and Status Register Definitions */
#define CoreDebug_DHCSR_DBGKEY_Pos 16U                                      /*!< CoreDebug DHCSR: DBGKEY Position */
#define CoreDebug_DHCSR_DBGKEY_Msk (0xFFFFUL << CoreDebug_DHCSR_DBGKEY_Pos) /*!< CoreDebug DHCSR: DBGKEY Mask */

#define CoreDebug_DHCSR_S_RESET_ST_Pos 25U                                     /*!< CoreDebug DHCSR: S_RESET_ST Position */
#define CoreDebug_DHCSR_S_RESET_ST_Msk (1UL << CoreDebug_DHCSR_S_RESET_ST_Pos) /*!< CoreDebug DHCSR: S_RESET_ST Mask */

#define CoreDebug_DHCSR_S_RETIRE_ST_Pos 24U                                      /*!< CoreDebug DHCSR: S_RETIRE_ST Position */
#define CoreDebug_DHCSR_S_RETIRE_ST_Msk (1UL << CoreDebug_DHCSR_S_RETIRE_ST_Pos) /*!< CoreDebug DHCSR: S_RETIRE_ST Mask */

#define CoreDebug_DHCSR_S_LOCKUP_Pos 19U                                   /*!< CoreDebug DHCSR: S_LOCKUP Position */
#define CoreDebug_DHCSR_S_LOCKUP_Msk (1UL << CoreDebug_DHCSR_S_LOCKUP_Pos) /*!< CoreDebug DHCSR: S_LOCKUP Mask */

#define CoreDebug_DHCSR_S_SLEEP_Pos 18U                                  /*!< CoreDebug DHCSR: S_SLEEP Position */
#define CoreDebug_DHCSR_S_SLEEP_Msk (1UL << CoreDebug_DHCSR_S_SLEEP_Pos) /*!< CoreDebug DHCSR: S_SLEEP Mask */

#define CoreDebug_DHCSR_S_HALT_Pos 17U                                 /*!< CoreDebug DHCSR: S_HALT Position */
#define CoreDebug_DHCSR_S_HALT_Msk (1UL << CoreDebug_DHCSR_S_HALT_Pos) /*!< CoreDebug DHCSR: S_HALT Mask */

#define CoreDebug_DHCSR_S_REGRDY_Pos 16U                                   /*!< CoreDebug DHCSR: S_REGRDY Position */
#define CoreDebug_DHCSR_S_REGRDY_Msk (1UL << CoreDebug_DHCSR_S_REGRDY_Pos) /*!< CoreDebug DHCSR: S_REGRDY Mask */

#define CoreDebug_DHCSR_C_SNAPSTALL_Pos 5U                                       /*!< CoreDebug DHCSR: C_SNAPSTALL Position */
#define CoreDebug_DHCSR_C_SNAPSTALL_Msk (1UL << CoreDebug_DHCSR_C_SNAPSTALL_Pos) /*!< CoreDebug DHCSR: C_SNAPSTALL Mask */

#define CoreDebug_DHCSR_C_MASKINTS_Pos 3U                                      /*!< CoreDebug DHCSR: C_MASKINTS Position */
#define CoreDebug_DHCSR_C_MASKINTS_Msk (1UL << CoreDebug_DHCSR_C_MASKINTS_Pos) /*!< CoreDebug DHCSR: C_MASKINTS Mask */

#define CoreDebug_DHCSR_C_STEP_Pos 2U                                  /*!< CoreDebug DHCSR: C_STEP Position */
#define CoreDebug_DHCSR_C_STEP_Msk (1UL << CoreDebug_DHCSR_C_STEP_Pos) /*!< CoreDebug DHCSR: C_STEP Mask */

#define CoreDebug_DHCSR_C_HALT_Pos 1U                                  /*!< CoreDebug DHCSR: C_HALT Position */
#define CoreDebug_DHCSR_C_HALT_Msk (1UL << CoreDebug_DHCSR_C_HALT_Pos) /*!< CoreDebug DHCSR: C_HALT Mask */

#define CoreDebug_DHCSR_C_DEBUGEN_Pos 0U                                         /*!< CoreDebug DHCSR: C_DEBUGEN Position */
#define CoreDebug_DHCSR_C_DEBUGEN_Msk (1UL /*<< CoreDebug_DHCSR_C_DEBUGEN_Pos*/) /*!< CoreDebug DHCSR: C_DEBUGEN Mask */

/* Debug Core Register Selector Register Definitions */
#define CoreDebug_DCRSR_REGWnR_Pos 16U                                 /*!< CoreDebug DCRSR: REGWnR Position */
#define CoreDebug_DCRSR_REGWnR_Msk (1UL << CoreDebug_DCRSR_REGWnR_Pos) /*!< CoreDebug DCRSR: REGWnR Mask */

#define CoreDebug_DCRSR_REGSEL_Pos 0U                                         /*!< CoreDebug DCRSR: REGSEL Position */
#define CoreDebug_DCRSR_REGSEL_Msk (0x1FUL /*<< CoreDebug_DCRSR_REGSEL_Pos*/) /*!< CoreDebug DCRSR: REGSEL Mask */

/* Debug Exception and Monitor Control Register Definitions */
#define CoreDebug_DEMCR_TRCENA_Pos 24U                                 /*!< CoreDebug DEMCR: TRCENA Position */
#define CoreDebug_DEMCR_TRCENA_Msk (1UL << CoreDebug_DEMCR_TRCENA_Pos) /*!< CoreDebug DEMCR: TRCENA Mask */

#define CoreDebug_DEMCR_MON_REQ_Pos 19U                                  /*!< CoreDebug DEMCR: MON_REQ Position */
#define CoreDebug_DEMCR_MON_REQ_Msk (1UL << CoreDebug_DEMCR_MON_REQ_Pos) /*!< CoreDebug DEMCR: MON_REQ Mask */

#define CoreDebug_DEMCR_MON_STEP_Pos 18U                                   /*!< CoreDebug DEMCR: MON_STEP Position */
#define CoreDebug_DEMCR_MON_STEP_Msk (1UL << CoreDebug_DEMCR_MON_STEP_Pos) /*!< CoreDebug DEMCR: MON_STEP Mask */

#define CoreDebug_DEMCR_MON_PEND_Pos 17U                                   /*!< CoreDebug DEMCR: MON_PEND Position */
#define CoreDebug_DEMCR_MON_PEND_Msk (1UL << CoreDebug_DEMCR_MON_PEND_Pos) /*!< CoreDebug DEMCR: MON_PEND Mask */

#define CoreDebug_DEMCR_MON_EN_Pos 16U                                 /*!< CoreDebug DEMCR: MON_EN Position */
#define CoreDebug_DEMCR_MON_EN_Msk (1UL << CoreDebug_DEMCR_MON_EN_Pos) /*!< CoreDebug DEMCR: MON_EN Mask */

#define CoreDebug_DEMCR_VC_HARDERR_Pos 10U                                     /*!< CoreDebug DEMCR: VC_HARDERR Position */
#define CoreDebug_DEMCR_VC_HARDERR_Msk (1UL << CoreDebug_DEMCR_VC_HARDERR_Pos) /*!< CoreDebug DEMCR: VC_HARDERR Mask */

#define CoreDebug_DEMCR_VC_INTERR_Pos 9U                                     /*!< CoreDebug DEMCR: VC_INTERR Position */
#define CoreDebug_DEMCR_VC_INTERR_Msk (1UL << CoreDebug_DEMCR_VC_INTERR_Pos) /*!< CoreDebug DEMCR: VC_INTERR Mask */

#define CoreDebug_DEMCR_VC_BUSERR_Pos 8U                                     /*!< CoreDebug DEMCR: VC_BUSERR Position */
#define CoreDebug_DEMCR_VC_BUSERR_Msk (1UL << CoreDebug_DEMCR_VC_BUSERR_Pos) /*!< CoreDebug DEMCR: VC_BUSERR Mask */

#define CoreDebug_DEMCR_VC_STATERR_Pos 7U                                      /*!< CoreDebug DEMCR: VC_STATERR Position */
#define CoreDebug_DEMCR_VC_STATERR_Msk (1UL << CoreDebug_DEMCR_VC_STATERR_Pos) /*!< CoreDebug DEMCR: VC_STATERR Mask */

#define CoreDebug_DEMCR_VC_CHKERR_Pos 6U                                     /*!< CoreDebug DEMCR: VC_CHKERR Position */
#define CoreDebug_DEMCR_VC_CHKERR_Msk (1UL << CoreDebug_DEMCR_VC_CHKERR_Pos) /*!< CoreDebug DEMCR: VC_CHKERR Mask */

#define CoreDebug_DEMCR_VC_NOCPERR_Pos 5U                                      /*!< CoreDebug DEMCR: VC_NOCPERR Position */
#define CoreDebug_DEMCR_VC_NOCPERR_Msk (1UL << CoreDebug_DEMCR_VC_NOCPERR_Pos) /*!< CoreDebug DEMCR: VC_NOCPERR Mask */

#define CoreDebug_DEMCR_VC_MMERR_Pos 4U                                    /*!< CoreDebug DEMCR: VC_MMERR Position */
#define CoreDebug_DEMCR_VC_MMERR_Msk (1UL << CoreDebug_DEMCR_VC_MMERR_Pos) /*!< CoreDebug DEMCR: VC_MMERR Mask */

#define CoreDebug_DEMCR_VC_CORERESET_Pos 0U                                            /*!< CoreDebug DEMCR: VC_CORERESET Position */
#define CoreDebug_DEMCR_VC_CORERESET_Msk (1UL /*<< CoreDebug_DEMCR_VC_CORERESET_Pos*/) /*!< CoreDebug DEMCR: VC_CORERESET Mask */

/*@} end of group CMSIS_CoreDebug */

#define _VAL2FLD(field, value) (((uint32_t)(value) << field##_Pos) & field##_Msk)

#define _FLD2VAL(field, value) (((uint32_t)(value) & field##_Msk) >> field##_Pos)

/*@} end of group CMSIS_core_bitfield */

/* Memory mapping of Core Hardware */
#define SCS_BASE (0xE000E000UL)            /*!< System Control Space Base Address */
#define ITM_BASE (0xE0000000UL)            /*!< ITM Base Address */
#define DWT_BASE (0xE0001000UL)            /*!< DWT Base Address */
#define TPI_BASE (0xE0040000UL)            /*!< TPI Base Address */
#define CoreDebug_BASE (0xE000EDF0UL)      /*!< Core Debug Base Address */
#define SysTick_BASE (SCS_BASE + 0x0010UL) /*!< SysTick Base Address */
#define NVIC_BASE (SCS_BASE + 0x0100UL)    /*!< NVIC Base Address */
#define SCB_BASE (SCS_BASE + 0x0D00UL)     /*!< System Control Block Base Address */

#define SCnSCB ((SCnSCB_Type *)SCS_BASE)             /*!< System control Register not in SCB */
#define SCB ((SCB_Type *)SCB_BASE)                   /*!< SCB configuration struct */
#define SysTick ((SysTick_Type *)SysTick_BASE)       /*!< SysTick configuration struct */
#define NVIC ((NVIC_Type *)NVIC_BASE)                /*!< NVIC configuration struct */
#define ITM ((ITM_Type *)ITM_BASE)                   /*!< ITM configuration struct */
#define DWT ((DWT_Type *)DWT_BASE)                   /*!< DWT configuration struct */
#define TPI ((TPI_Type *)TPI_BASE)                   /*!< TPI configuration struct */
#define CoreDebug ((CoreDebug_Type *)CoreDebug_BASE) /*!< Core Debug configuration struct */

/*@} */

/* ##########################   NVIC functions  #################################### */

#define NVIC_SetPriorityGrouping __NVIC_SetPriorityGrouping
#define NVIC_GetPriorityGrouping __NVIC_GetPriorityGrouping
#define NVIC_EnableIRQ __NVIC_EnableIRQ
#define NVIC_GetEnableIRQ __NVIC_GetEnableIRQ
#define NVIC_DisableIRQ __NVIC_DisableIRQ
#define NVIC_GetPendingIRQ __NVIC_GetPendingIRQ
#define NVIC_SetPendingIRQ __NVIC_SetPendingIRQ
#define NVIC_ClearPendingIRQ __NVIC_ClearPendingIRQ
#define NVIC_GetActive __NVIC_GetActive
#define NVIC_SetPriority __NVIC_SetPriority
#define NVIC_GetPriority __NVIC_GetPriority
#define NVIC_SystemReset __NVIC_SystemReset
#define NVIC_SetVector __NVIC_SetVector
#define NVIC_GetVector __NVIC_GetVector

#define NVIC_USER_IRQ_OFFSET 16

/* The following EXC_RETURN values are saved the LR on exception entry */
#define EXC_RETURN_HANDLER (0xFFFFFFF1UL)    /* return to Handler mode, uses MSP after return                               */
#define EXC_RETURN_THREAD_MSP (0xFFFFFFF9UL) /* return to Thread mode, uses MSP after return                                */
#define EXC_RETURN_THREAD_PSP (0xFFFFFFFDUL) /* return to Thread mode, uses PSP after return                                */

__STATIC_INLINE void __NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL); /* only values 0..7 are used          */

  reg_value = SCB->AIRCR;                                                     /* read old register configuration    */
  reg_value &= ~((uint32_t)(SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIGROUP_Msk)); /* clear bits to change               */
  reg_value = (reg_value |
               ((uint32_t)0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
               (PriorityGroupTmp << SCB_AIRCR_PRIGROUP_Pos)); /* Insert write key and priority group */
  SCB->AIRCR = reg_value;
}

__STATIC_INLINE uint32_t __NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) >> SCB_AIRCR_PRIGROUP_Pos));
}

__STATIC_INLINE void __NVIC_EnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}

__STATIC_INLINE uint32_t __NVIC_GetEnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return ((uint32_t)(((NVIC->ISER[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return (0U);
  }
}

__STATIC_INLINE void __NVIC_DisableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->ICER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    __DSB();
    __ISB();
  }
}

__STATIC_INLINE uint32_t __NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return ((uint32_t)(((NVIC->ISPR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return (0U);
  }
}

__STATIC_INLINE void __NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->ISPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}

__STATIC_INLINE void __NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->ICPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}

__STATIC_INLINE uint32_t __NVIC_GetActive(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return ((uint32_t)(((NVIC->IABR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return (0U);
  }
}

__STATIC_INLINE void __NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->IP[((uint32_t)IRQn)] = (uint8_t)((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
  }
  else
  {
    SCB->SHP[(((uint32_t)IRQn) & 0xFUL) - 4UL] = (uint8_t)((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
  }
}

__STATIC_INLINE uint32_t __NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) >= 0)
  {
    return (((uint32_t)NVIC->IP[((uint32_t)IRQn)] >> (8U - __NVIC_PRIO_BITS)));
  }
  else
  {
    return (((uint32_t)SCB->SHP[(((uint32_t)IRQn) & 0xFUL) - 4UL] >> (8U - __NVIC_PRIO_BITS)));
  }
}

__STATIC_INLINE uint32_t NVIC_EncodePriority(uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL); /* only values 0..7 are used          */
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(__NVIC_PRIO_BITS)) ? (uint32_t)(__NVIC_PRIO_BITS) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits = ((PriorityGroupTmp + (uint32_t)(__NVIC_PRIO_BITS)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(__NVIC_PRIO_BITS));

  return (
      ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
      ((SubPriority & (uint32_t)((1UL << (SubPriorityBits)) - 1UL))));
}

__STATIC_INLINE void NVIC_DecodePriority(uint32_t Priority, uint32_t PriorityGroup, uint32_t *const pPreemptPriority, uint32_t *const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL); /* only values 0..7 are used          */
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(__NVIC_PRIO_BITS)) ? (uint32_t)(__NVIC_PRIO_BITS) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits = ((PriorityGroupTmp + (uint32_t)(__NVIC_PRIO_BITS)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(__NVIC_PRIO_BITS));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority = (Priority) & (uint32_t)((1UL << (SubPriorityBits)) - 1UL);
}

__STATIC_INLINE void __NVIC_SetVector(IRQn_Type IRQn, uint32_t vector)
{
  uint32_t *vectors = (uint32_t *)SCB->VTOR;
  vectors[(int32_t)IRQn + NVIC_USER_IRQ_OFFSET] = vector;
}

__STATIC_INLINE uint32_t __NVIC_GetVector(IRQn_Type IRQn)
{
  uint32_t *vectors = (uint32_t *)SCB->VTOR;
  return vectors[(int32_t)IRQn + NVIC_USER_IRQ_OFFSET];
}

__NO_RETURN __STATIC_INLINE void __NVIC_SystemReset(void)
{
  __DSB(); /* Ensure all outstanding memory accesses included
              buffered write are completed before reset */
  SCB->AIRCR = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
                          (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
                          SCB_AIRCR_SYSRESETREQ_Msk); /* Keep priority group unchanged */
  __DSB();                                            /* Ensure completion of memory access */

  for (;;) /* wait until reset */
  {
    __NOP();
  }
}

/* ##########################  FPU functions  #################################### */

__STATIC_INLINE uint32_t SCB_GetFPUType(void)
{
  return 0U; /* No FPU */
}

/* ##################################    SysTick function  ############################################ */

#if defined(__Vendor_SysTickConfig) && (__Vendor_SysTickConfig == 0U)

__STATIC_INLINE uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk)
  {
    return (1UL); /* Reload value impossible */
  }

  SysTick->LOAD = (uint32_t)(ticks - 1UL);                         /* set reload register */
  NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL); /* set Priority for Systick Interrupt */
  SysTick->VAL = 0UL;                                              /* Load the SysTick Counter Value */
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                  SysTick_CTRL_TICKINT_Msk |
                  SysTick_CTRL_ENABLE_Msk; /* Enable SysTick IRQ and SysTick Timer */
  return (0UL);                            /* Function successful */
}

#endif

/*@} end of CMSIS_Core_SysTickFunctions */

#endif /* __CORE_CM3_H_DEPENDANT */

#endif /* __CMSIS_GENERIC */

typedef struct
{
  __IO uint32_t SR;
  __IO uint32_t CR1;
  __IO uint32_t CR2;
  __IO uint32_t SMPR1;
  __IO uint32_t SMPR2;
  __IO uint32_t JOFR1;
  __IO uint32_t JOFR2;
  __IO uint32_t JOFR3;
  __IO uint32_t JOFR4;
  __IO uint32_t HTR;
  __IO uint32_t LTR;
  __IO uint32_t SQR1;
  __IO uint32_t SQR2;
  __IO uint32_t SQR3;
  __IO uint32_t JSQR;
  __IO uint32_t JDR1;
  __IO uint32_t JDR2;
  __IO uint32_t JDR3;
  __IO uint32_t JDR4;
  __IO uint32_t DR;
} ADC_TypeDef;

typedef struct
{
  __IO uint32_t SR;  /*!< ADC status register,    used for ADC multimode (bits common to several ADC instances). Address offset: ADC1 base address         */
  __IO uint32_t CR1; /*!< ADC control register 1, used for ADC multimode (bits common to several ADC instances). Address offset: ADC1 base address + 0x04  */
  __IO uint32_t CR2; /*!< ADC control register 2, used for ADC multimode (bits common to several ADC instances). Address offset: ADC1 base address + 0x08  */
  uint32_t RESERVED[16];
  __IO uint32_t DR; /*!< ADC data register,      used for ADC multimode (bits common to several ADC instances). Address offset: ADC1 base address + 0x4C  */
} ADC_Common_TypeDef;

typedef struct
{
  uint32_t RESERVED0;
  __IO uint32_t DR1;
  __IO uint32_t DR2;
  __IO uint32_t DR3;
  __IO uint32_t DR4;
  __IO uint32_t DR5;
  __IO uint32_t DR6;
  __IO uint32_t DR7;
  __IO uint32_t DR8;
  __IO uint32_t DR9;
  __IO uint32_t DR10;
  __IO uint32_t RTCCR;
  __IO uint32_t CR;
  __IO uint32_t CSR;
} BKP_TypeDef;

typedef struct
{
  __IO uint32_t TIR;
  __IO uint32_t TDTR;
  __IO uint32_t TDLR;
  __IO uint32_t TDHR;
} CAN_TxMailBox_TypeDef;

typedef struct
{
  __IO uint32_t RIR;
  __IO uint32_t RDTR;
  __IO uint32_t RDLR;
  __IO uint32_t RDHR;
} CAN_FIFOMailBox_TypeDef;

typedef struct
{
  __IO uint32_t FR1;
  __IO uint32_t FR2;
} CAN_FilterRegister_TypeDef;

typedef struct
{
  __IO uint32_t MCR;
  __IO uint32_t MSR;
  __IO uint32_t TSR;
  __IO uint32_t RF0R;
  __IO uint32_t RF1R;
  __IO uint32_t IER;
  __IO uint32_t ESR;
  __IO uint32_t BTR;
  uint32_t RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  uint32_t RESERVED1[12];
  __IO uint32_t FMR;
  __IO uint32_t FM1R;
  uint32_t RESERVED2;
  __IO uint32_t FS1R;
  uint32_t RESERVED3;
  __IO uint32_t FFA1R;
  uint32_t RESERVED4;
  __IO uint32_t FA1R;
  uint32_t RESERVED5[8];
  CAN_FilterRegister_TypeDef sFilterRegister[14];
} CAN_TypeDef;

typedef struct
{
  __IO uint32_t DR;   /*!< CRC Data register,                           Address offset: 0x00 */
  __IO uint8_t IDR;   /*!< CRC Independent data register,               Address offset: 0x04 */
  uint8_t RESERVED0;  /*!< Reserved,                                    Address offset: 0x05 */
  uint16_t RESERVED1; /*!< Reserved,                                    Address offset: 0x06 */
  __IO uint32_t CR;   /*!< CRC Control register,                        Address offset: 0x08 */
} CRC_TypeDef;

typedef struct
{
  __IO uint32_t IDCODE;
  __IO uint32_t CR;
} DBGMCU_TypeDef;

typedef struct
{
  __IO uint32_t CCR;
  __IO uint32_t CNDTR;
  __IO uint32_t CPAR;
  __IO uint32_t CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  __IO uint32_t ISR;
  __IO uint32_t IFCR;
} DMA_TypeDef;

typedef struct
{
  __IO uint32_t IMR;
  __IO uint32_t EMR;
  __IO uint32_t RTSR;
  __IO uint32_t FTSR;
  __IO uint32_t SWIER;
  __IO uint32_t PR;
} EXTI_TypeDef;

typedef struct
{
  __IO uint32_t ACR;
  __IO uint32_t KEYR;
  __IO uint32_t OPTKEYR;
  __IO uint32_t SR;
  __IO uint32_t CR;
  __IO uint32_t AR;
  __IO uint32_t RESERVED;
  __IO uint32_t OBR;
  __IO uint32_t WRPR;
} FLASH_TypeDef;

typedef struct
{
  __IO uint16_t RDP;
  __IO uint16_t USER;
  __IO uint16_t Data0;
  __IO uint16_t Data1;
  __IO uint16_t WRP0;
  __IO uint16_t WRP1;
  __IO uint16_t WRP2;
  __IO uint16_t WRP3;
} OB_TypeDef;

typedef struct
{
  __IO uint32_t CRL;
  __IO uint32_t CRH;
  __IO uint32_t IDR;
  __IO uint32_t ODR;
  __IO uint32_t BSRR;
  __IO uint32_t BRR;
  __IO uint32_t LCKR;
} GPIO_TypeDef;

typedef struct
{
  __IO uint32_t EVCR;
  __IO uint32_t MAPR;
  __IO uint32_t EXTICR[4];
  uint32_t RESERVED0;
  __IO uint32_t MAPR2;
} AFIO_TypeDef;

typedef struct
{
  __IO uint32_t CR1;
  __IO uint32_t CR2;
  __IO uint32_t OAR1;
  __IO uint32_t OAR2;
  __IO uint32_t DR;
  __IO uint32_t SR1;
  __IO uint32_t SR2;
  __IO uint32_t CCR;
  __IO uint32_t TRISE;
} I2C_TypeDef;

typedef struct
{
  __IO uint32_t KR;  /*!< Key register,                                Address offset: 0x00 */
  __IO uint32_t PR;  /*!< Prescaler register,                          Address offset: 0x04 */
  __IO uint32_t RLR; /*!< Reload register,                             Address offset: 0x08 */
  __IO uint32_t SR;  /*!< Status register,                             Address offset: 0x0C */
} IWDG_TypeDef;

typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CSR;
} PWR_TypeDef;

typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CFGR;
  __IO uint32_t CIR;
  __IO uint32_t APB2RSTR;
  __IO uint32_t APB1RSTR;
  __IO uint32_t AHBENR;
  __IO uint32_t APB2ENR;
  __IO uint32_t APB1ENR;
  __IO uint32_t BDCR;
  __IO uint32_t CSR;

} RCC_TypeDef;

typedef struct
{
  __IO uint32_t CRH;
  __IO uint32_t CRL;
  __IO uint32_t PRLH;
  __IO uint32_t PRLL;
  __IO uint32_t DIVH;
  __IO uint32_t DIVL;
  __IO uint32_t CNTH;
  __IO uint32_t CNTL;
  __IO uint32_t ALRH;
  __IO uint32_t ALRL;
} RTC_TypeDef;

typedef struct
{
  __IO uint32_t CR1;
  __IO uint32_t CR2;
  __IO uint32_t SR;
  __IO uint32_t DR;
  __IO uint32_t CRCPR;
  __IO uint32_t RXCRCR;
  __IO uint32_t TXCRCR;
  __IO uint32_t I2SCFGR;
} SPI_TypeDef;

typedef struct
{
  __IO uint32_t CR1;   /*!< TIM control register 1,                      Address offset: 0x00 */
  __IO uint32_t CR2;   /*!< TIM control register 2,                      Address offset: 0x04 */
  __IO uint32_t SMCR;  /*!< TIM slave Mode Control register,             Address offset: 0x08 */
  __IO uint32_t DIER;  /*!< TIM DMA/interrupt enable register,           Address offset: 0x0C */
  __IO uint32_t SR;    /*!< TIM status register,                         Address offset: 0x10 */
  __IO uint32_t EGR;   /*!< TIM event generation register,               Address offset: 0x14 */
  __IO uint32_t CCMR1; /*!< TIM  capture/compare mode register 1,        Address offset: 0x18 */
  __IO uint32_t CCMR2; /*!< TIM  capture/compare mode register 2,        Address offset: 0x1C */
  __IO uint32_t CCER;  /*!< TIM capture/compare enable register,         Address offset: 0x20 */
  __IO uint32_t CNT;   /*!< TIM counter register,                        Address offset: 0x24 */
  __IO uint32_t PSC;   /*!< TIM prescaler register,                      Address offset: 0x28 */
  __IO uint32_t ARR;   /*!< TIM auto-reload register,                    Address offset: 0x2C */
  __IO uint32_t RCR;   /*!< TIM  repetition counter register,            Address offset: 0x30 */
  __IO uint32_t CCR1;  /*!< TIM capture/compare register 1,              Address offset: 0x34 */
  __IO uint32_t CCR2;  /*!< TIM capture/compare register 2,              Address offset: 0x38 */
  __IO uint32_t CCR3;  /*!< TIM capture/compare register 3,              Address offset: 0x3C */
  __IO uint32_t CCR4;  /*!< TIM capture/compare register 4,              Address offset: 0x40 */
  __IO uint32_t BDTR;  /*!< TIM break and dead-time register,            Address offset: 0x44 */
  __IO uint32_t DCR;   /*!< TIM DMA control register,                    Address offset: 0x48 */
  __IO uint32_t DMAR;  /*!< TIM DMA address for full transfer register,  Address offset: 0x4C */
  __IO uint32_t OR;    /*!< TIM option register,                         Address offset: 0x50 */
} TIM_TypeDef;

typedef struct
{
  __IO uint32_t SR;   /*!< USART Status register,                   Address offset: 0x00 */
  __IO uint32_t DR;   /*!< USART Data register,                     Address offset: 0x04 */
  __IO uint32_t BRR;  /*!< USART Baud rate register,                Address offset: 0x08 */
  __IO uint32_t CR1;  /*!< USART Control register 1,                Address offset: 0x0C */
  __IO uint32_t CR2;  /*!< USART Control register 2,                Address offset: 0x10 */
  __IO uint32_t CR3;  /*!< USART Control register 3,                Address offset: 0x14 */
  __IO uint32_t GTPR; /*!< USART Guard time and prescaler register, Address offset: 0x18 */
} USART_TypeDef;

typedef struct
{
  __IO uint32_t CR;  /*!< WWDG Control register,       Address offset: 0x00 */
  __IO uint32_t CFR; /*!< WWDG Configuration register, Address offset: 0x04 */
  __IO uint32_t SR;  /*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_TypeDef;

#define FLASH_BASE 0x08000000UL      /*!< FLASH base address in the alias region */
#define FLASH_BANK1_END 0x0801FFFFUL /*!< FLASH END address of bank1 */
#define SRAM_BASE 0x20000000UL       /*!< SRAM base address in the alias region */
#define PERIPH_BASE 0x40000000UL     /*!< Peripheral base address in the alias region */

#define SRAM_BB_BASE 0x22000000UL   /*!< SRAM base address in the bit-band region */
#define PERIPH_BB_BASE 0x42000000UL /*!< Peripheral base address in the bit-band region */

/*!< Peripheral memory map */
#define APB1PERIPH_BASE PERIPH_BASE
#define APB2PERIPH_BASE (PERIPH_BASE + 0x00010000UL)
#define AHBPERIPH_BASE (PERIPH_BASE + 0x00020000UL)

#define TIM2_BASE (APB1PERIPH_BASE + 0x00000000UL)
#define TIM3_BASE (APB1PERIPH_BASE + 0x00000400UL)
#define TIM4_BASE (APB1PERIPH_BASE + 0x00000800UL)
#define RTC_BASE (APB1PERIPH_BASE + 0x00002800UL)
#define WWDG_BASE (APB1PERIPH_BASE + 0x00002C00UL)
#define IWDG_BASE (APB1PERIPH_BASE + 0x00003000UL)
#define SPI2_BASE (APB1PERIPH_BASE + 0x00003800UL)
#define I2C1_BASE (APB1PERIPH_BASE + 0x00005400UL)
#define I2C2_BASE (APB1PERIPH_BASE + 0x00005800UL)
#define CAN1_BASE (APB1PERIPH_BASE + 0x00006400UL)
#define PWR_BASE (APB1PERIPH_BASE + 0x00007000UL)
#define GPIOA_BASE (APB2PERIPH_BASE + 0x00000800UL)
#define GPIOB_BASE (APB2PERIPH_BASE + 0x00000C00UL)
#define GPIOC_BASE (APB2PERIPH_BASE + 0x00001000UL)
#define GPIOD_BASE (APB2PERIPH_BASE + 0x00001400UL)
#define GPIOE_BASE (APB2PERIPH_BASE + 0x00001800UL)
#define ADC1_BASE (APB2PERIPH_BASE + 0x00002400UL)
#define ADC2_BASE (APB2PERIPH_BASE + 0x00002800UL)
#define TIM1_BASE (APB2PERIPH_BASE + 0x00002C00UL)
#define SPI1_BASE (APB2PERIPH_BASE + 0x00003000UL)

// #define DMA1_BASE (AHBPERIPH_BASE + 0x00000000UL)
// #define DMA1_Channel1_BASE (AHBPERIPH_BASE + 0x00000008UL)
// #define DMA1_Channel2_BASE (AHBPERIPH_BASE + 0x0000001CUL)
// #define DMA1_Channel3_BASE (AHBPERIPH_BASE + 0x00000030UL)
// #define DMA1_Channel4_BASE (AHBPERIPH_BASE + 0x00000044UL)
// #define DMA1_Channel5_BASE (AHBPERIPH_BASE + 0x00000058UL)
// #define DMA1_Channel6_BASE (AHBPERIPH_BASE + 0x0000006CUL)
// #define DMA1_Channel7_BASE (AHBPERIPH_BASE + 0x00000080UL)
#define RCC_BASE (AHBPERIPH_BASE + 0x00001000UL)
#define CRC_BASE (AHBPERIPH_BASE + 0x00003000UL)

#define FLASH_R_BASE (AHBPERIPH_BASE + 0x00002000UL) /*!< Flash registers base address */
#define FLASHSIZE_BASE 0x1FFFF7E0UL                  /*!< FLASH Size register base address */
#define OB_BASE 0x1FFFF800UL                         /*!< Flash Option Bytes base address */

// #define DBGMCU_BASE 0xE0042000UL /*!< Debug MCU registers base address */

/* USB device FS */
#define USB_BASE (APB1PERIPH_BASE + 0x00005C00UL)    /*!< USB_IP Peripheral Registers base address */
#define USB_PMAADDR (APB1PERIPH_BASE + 0x00006000UL) /*!< USB_IP Packet Memory Area base address */

#define TIM2 ((TIM_TypeDef *)TIM2_BASE)
#define TIM3 ((TIM_TypeDef *)TIM3_BASE)
#define TIM4 ((TIM_TypeDef *)TIM4_BASE)
#define RTC ((RTC_TypeDef *)RTC_BASE)
#define WWDG ((WWDG_TypeDef *)WWDG_BASE)
#define IWDG ((IWDG_TypeDef *)IWDG_BASE)
#define SPI2 ((SPI_TypeDef *)SPI2_BASE)
#define I2C1 ((I2C_TypeDef *)I2C1_BASE)
#define I2C2 ((I2C_TypeDef *)I2C2_BASE)
#define USB ((USB_TypeDef *)USB_BASE)
#define CAN1 ((CAN_TypeDef *)CAN1_BASE)
#define PWR ((PWR_TypeDef *)PWR_BASE)
#define GPIOA ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD ((GPIO_TypeDef *)GPIOD_BASE)
#define GPIOE ((GPIO_TypeDef *)GPIOE_BASE)
#define ADC1 ((ADC_TypeDef *)ADC1_BASE)
#define ADC2 ((ADC_TypeDef *)ADC2_BASE)
#define ADC12_COMMON ((ADC_Common_TypeDef *)ADC1_BASE)
#define TIM1 ((TIM_TypeDef *)TIM1_BASE)
#define SPI1 ((SPI_TypeDef *)SPI1_BASE)

#define RCC ((RCC_TypeDef *)RCC_BASE)
#define CRC ((CRC_TypeDef *)CRC_BASE)
#define FLASH ((FLASH_TypeDef *)FLASH_R_BASE)
#define OB ((OB_TypeDef *)OB_BASE)
// #define DBGMCU ((DBGMCU_TypeDef *)DBGMCU_BASE)

#define LSI_STARTUP_TIME 85U /*!< LSI Maximum startup time in us */

/*                         Peripheral Registers_Bits_Definition               */

/*                                                                            */
/*                       CRC calculation unit (CRC)                           */
/*                                                                            */

#define CRC_DR_DR_Pos (0U)
#define CRC_DR_DR_Msk (0xFFFFFFFFUL << CRC_DR_DR_Pos) /*!< 0xFFFFFFFF */
#define CRC_DR_DR CRC_DR_DR_Msk                       /*!< Data register bits */

#define CRC_IDR_IDR_Pos (0U)
#define CRC_IDR_IDR_Msk (0xFFUL << CRC_IDR_IDR_Pos) /*!< 0x000000FF */
#define CRC_IDR_IDR CRC_IDR_IDR_Msk                 /*!< General-purpose 8-bit data register bits */

#define CRC_CR_RESET_Pos (0U)
#define CRC_CR_RESET_Msk (0x1UL << CRC_CR_RESET_Pos) /*!< 0x00000001 */
#define CRC_CR_RESET CRC_CR_RESET_Msk                /*!< RESET bit */

/*                                                                            */
/*                             Power Control                                  */
/*                                                                            */

#define PWR_CR_LPDS_Pos (0U)
#define PWR_CR_LPDS_Msk (0x1UL << PWR_CR_LPDS_Pos) /*!< 0x00000001 */
#define PWR_CR_LPDS PWR_CR_LPDS_Msk                /*!< Low-Power Deepsleep */
#define PWR_CR_PDDS_Pos (1U)
#define PWR_CR_PDDS_Msk (0x1UL << PWR_CR_PDDS_Pos) /*!< 0x00000002 */
#define PWR_CR_PDDS PWR_CR_PDDS_Msk                /*!< Power Down Deepsleep */
#define PWR_CR_CWUF_Pos (2U)
#define PWR_CR_CWUF_Msk (0x1UL << PWR_CR_CWUF_Pos) /*!< 0x00000004 */
#define PWR_CR_CWUF PWR_CR_CWUF_Msk                /*!< Clear Wakeup Flag */
#define PWR_CR_CSBF_Pos (3U)
#define PWR_CR_CSBF_Msk (0x1UL << PWR_CR_CSBF_Pos) /*!< 0x00000008 */
#define PWR_CR_CSBF PWR_CR_CSBF_Msk                /*!< Clear Standby Flag */
#define PWR_CR_PVDE_Pos (4U)
#define PWR_CR_PVDE_Msk (0x1UL << PWR_CR_PVDE_Pos) /*!< 0x00000010 */
#define PWR_CR_PVDE PWR_CR_PVDE_Msk                /*!< Power Voltage Detector Enable */

#define PWR_CR_PLS_Pos (5U)
#define PWR_CR_PLS_Msk (0x7UL << PWR_CR_PLS_Pos) /*!< 0x000000E0 */
#define PWR_CR_PLS PWR_CR_PLS_Msk                /*!< PLS[2:0] bits (PVD Level Selection) */
#define PWR_CR_PLS_0 (0x1UL << PWR_CR_PLS_Pos)   /*!< 0x00000020 */
#define PWR_CR_PLS_1 (0x2UL << PWR_CR_PLS_Pos)   /*!< 0x00000040 */
#define PWR_CR_PLS_2 (0x4UL << PWR_CR_PLS_Pos)   /*!< 0x00000080 */

/*!< PVD level configuration */
#define PWR_CR_PLS_LEV0 0x00000000U /*!< PVD level 2.2V */
#define PWR_CR_PLS_LEV1 0x00000020U /*!< PVD level 2.3V */
#define PWR_CR_PLS_LEV2 0x00000040U /*!< PVD level 2.4V */
#define PWR_CR_PLS_LEV3 0x00000060U /*!< PVD level 2.5V */
#define PWR_CR_PLS_LEV4 0x00000080U /*!< PVD level 2.6V */
#define PWR_CR_PLS_LEV5 0x000000A0U /*!< PVD level 2.7V */
#define PWR_CR_PLS_LEV6 0x000000C0U /*!< PVD level 2.8V */
#define PWR_CR_PLS_LEV7 0x000000E0U /*!< PVD level 2.9V */

/* Legacy defines */
#define PWR_CR_PLS_2V2 PWR_CR_PLS_LEV0
#define PWR_CR_PLS_2V3 PWR_CR_PLS_LEV1
#define PWR_CR_PLS_2V4 PWR_CR_PLS_LEV2
#define PWR_CR_PLS_2V5 PWR_CR_PLS_LEV3
#define PWR_CR_PLS_2V6 PWR_CR_PLS_LEV4
#define PWR_CR_PLS_2V7 PWR_CR_PLS_LEV5
#define PWR_CR_PLS_2V8 PWR_CR_PLS_LEV6
#define PWR_CR_PLS_2V9 PWR_CR_PLS_LEV7

#define PWR_CR_DBP_Pos (8U)
#define PWR_CR_DBP_Msk (0x1UL << PWR_CR_DBP_Pos) /*!< 0x00000100 */
#define PWR_CR_DBP PWR_CR_DBP_Msk                /*!< Disable Backup Domain write protection */

#define PWR_CSR_WUF_Pos (0U)
#define PWR_CSR_WUF_Msk (0x1UL << PWR_CSR_WUF_Pos) /*!< 0x00000001 */
#define PWR_CSR_WUF PWR_CSR_WUF_Msk                /*!< Wakeup Flag */
#define PWR_CSR_SBF_Pos (1U)
#define PWR_CSR_SBF_Msk (0x1UL << PWR_CSR_SBF_Pos) /*!< 0x00000002 */
#define PWR_CSR_SBF PWR_CSR_SBF_Msk                /*!< Standby Flag */
#define PWR_CSR_PVDO_Pos (2U)
#define PWR_CSR_PVDO_Msk (0x1UL << PWR_CSR_PVDO_Pos) /*!< 0x00000004 */
#define PWR_CSR_PVDO PWR_CSR_PVDO_Msk                /*!< PVD Output */
#define PWR_CSR_EWUP_Pos (8U)
#define PWR_CSR_EWUP_Msk (0x1UL << PWR_CSR_EWUP_Pos) /*!< 0x00000100 */
#define PWR_CSR_EWUP PWR_CSR_EWUP_Msk                /*!< Enable WKUP pin */

/*                                                                            */
/*                            Backup registers                                */
/*                                                                            */

#define RTC_BKP_NUMBER 10

/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */

#define RCC_CR_HSION_Pos (0U)
#define RCC_CR_HSION_Msk (0x1UL << RCC_CR_HSION_Pos) /*!< 0x00000001 */
#define RCC_CR_HSION RCC_CR_HSION_Msk                /*!< Internal High Speed clock enable */
#define RCC_CR_HSIRDY_Pos (1U)
#define RCC_CR_HSIRDY_Msk (0x1UL << RCC_CR_HSIRDY_Pos) /*!< 0x00000002 */
#define RCC_CR_HSIRDY RCC_CR_HSIRDY_Msk                /*!< Internal High Speed clock ready flag */
#define RCC_CR_HSITRIM_Pos (3U)
#define RCC_CR_HSITRIM_Msk (0x1FUL << RCC_CR_HSITRIM_Pos) /*!< 0x000000F8 */
#define RCC_CR_HSITRIM RCC_CR_HSITRIM_Msk                 /*!< Internal High Speed clock trimming */
#define RCC_CR_HSICAL_Pos (8U)
#define RCC_CR_HSICAL_Msk (0xFFUL << RCC_CR_HSICAL_Pos) /*!< 0x0000FF00 */
#define RCC_CR_HSICAL RCC_CR_HSICAL_Msk                 /*!< Internal High Speed clock Calibration */
#define RCC_CR_HSEON_Pos (16U)
#define RCC_CR_HSEON_Msk (0x1UL << RCC_CR_HSEON_Pos) /*!< 0x00010000 */
#define RCC_CR_HSEON RCC_CR_HSEON_Msk                /*!< External High Speed clock enable */
#define RCC_CR_HSERDY_Pos (17U)
#define RCC_CR_HSERDY_Msk (0x1UL << RCC_CR_HSERDY_Pos) /*!< 0x00020000 */
#define RCC_CR_HSERDY RCC_CR_HSERDY_Msk                /*!< External High Speed clock ready flag */
#define RCC_CR_HSEBYP_Pos (18U)
#define RCC_CR_HSEBYP_Msk (0x1UL << RCC_CR_HSEBYP_Pos) /*!< 0x00040000 */
#define RCC_CR_HSEBYP RCC_CR_HSEBYP_Msk                /*!< External High Speed clock Bypass */
#define RCC_CR_CSSON_Pos (19U)
#define RCC_CR_CSSON_Msk (0x1UL << RCC_CR_CSSON_Pos) /*!< 0x00080000 */
#define RCC_CR_CSSON RCC_CR_CSSON_Msk                /*!< Clock Security System enable */
#define RCC_CR_PLLON_Pos (24U)
#define RCC_CR_PLLON_Msk (0x1UL << RCC_CR_PLLON_Pos) /*!< 0x01000000 */
#define RCC_CR_PLLON RCC_CR_PLLON_Msk                /*!< PLL enable */
#define RCC_CR_PLLRDY_Pos (25U)
#define RCC_CR_PLLRDY_Msk (0x1UL << RCC_CR_PLLRDY_Pos) /*!< 0x02000000 */
#define RCC_CR_PLLRDY RCC_CR_PLLRDY_Msk                /*!< PLL clock ready flag */

/*!< SW configuration */
#define RCC_CFGR_SW_Pos (0U)
#define RCC_CFGR_SW_Msk (0x3UL << RCC_CFGR_SW_Pos) /*!< 0x00000003 */
#define RCC_CFGR_SW RCC_CFGR_SW_Msk                /*!< SW[1:0] bits (System clock Switch) */
#define RCC_CFGR_SW_0 (0x1UL << RCC_CFGR_SW_Pos)   /*!< 0x00000001 */
#define RCC_CFGR_SW_1 (0x2UL << RCC_CFGR_SW_Pos)   /*!< 0x00000002 */

#define RCC_CFGR_SW_HSI 0x00000000U /*!< HSI selected as system clock */
#define RCC_CFGR_SW_HSE 0x00000001U /*!< HSE selected as system clock */

/*!< SWS configuration */
#define RCC_CFGR_SWS_Pos (2U)
#define RCC_CFGR_SWS_Msk (0x3UL << RCC_CFGR_SWS_Pos) /*!< 0x0000000C */
#define RCC_CFGR_SWS RCC_CFGR_SWS_Msk                /*!< SWS[1:0] bits (System Clock Switch Status) */
#define RCC_CFGR_SWS_0 (0x1UL << RCC_CFGR_SWS_Pos)   /*!< 0x00000004 */
#define RCC_CFGR_SWS_1 (0x2UL << RCC_CFGR_SWS_Pos)   /*!< 0x00000008 */

#define RCC_CFGR_SWS_HSI 0x00000000U /*!< HSI oscillator used as system clock */
#define RCC_CFGR_SWS_HSE 0x00000004U /*!< HSE oscillator used as system clock */
#define RCC_CFGR_SWS_PLL 0x00000008U /*!< PLL used as system clock */

/*!< HPRE configuration */
#define RCC_CFGR_HPRE_Pos (4U)
#define RCC_CFGR_HPRE_Msk (0xFUL << RCC_CFGR_HPRE_Pos) /*!< 0x000000F0 */
#define RCC_CFGR_HPRE RCC_CFGR_HPRE_Msk                /*!< HPRE[3:0] bits (AHB prescaler) */
#define RCC_CFGR_HPRE_0 (0x1UL << RCC_CFGR_HPRE_Pos)   /*!< 0x00000010 */
#define RCC_CFGR_HPRE_1 (0x2UL << RCC_CFGR_HPRE_Pos)   /*!< 0x00000020 */
#define RCC_CFGR_HPRE_2 (0x4UL << RCC_CFGR_HPRE_Pos)   /*!< 0x00000040 */
#define RCC_CFGR_HPRE_3 (0x8UL << RCC_CFGR_HPRE_Pos)   /*!< 0x00000080 */

#define RCC_CFGR_HPRE_DIV2 0x00000080U   /*!< SYSCLK divided by 2 */
#define RCC_CFGR_HPRE_DIV4 0x00000090U   /*!< SYSCLK divided by 4 */
#define RCC_CFGR_HPRE_DIV8 0x000000A0U   /*!< SYSCLK divided by 8 */
#define RCC_CFGR_HPRE_DIV16 0x000000B0U  /*!< SYSCLK divided by 16 */
#define RCC_CFGR_HPRE_DIV64 0x000000C0U  /*!< SYSCLK divided by 64 */
#define RCC_CFGR_HPRE_DIV128 0x000000D0U /*!< SYSCLK divided by 128 */
#define RCC_CFGR_HPRE_DIV256 0x000000E0U /*!< SYSCLK divided by 256 */
#define RCC_CFGR_HPRE_DIV512 0x000000F0U /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
#define RCC_CFGR_PPRE1_Pos (8U)
#define RCC_CFGR_PPRE1_Msk (0x7UL << RCC_CFGR_PPRE1_Pos) /*!< 0x00000700 */
#define RCC_CFGR_PPRE1 RCC_CFGR_PPRE1_Msk                /*!< PRE1[2:0] bits (APB1 prescaler) */
#define RCC_CFGR_PPRE1_0 (0x1UL << RCC_CFGR_PPRE1_Pos)   /*!< 0x00000100 */
#define RCC_CFGR_PPRE1_1 (0x2UL << RCC_CFGR_PPRE1_Pos)   /*!< 0x00000200 */
#define RCC_CFGR_PPRE1_2 (0x4UL << RCC_CFGR_PPRE1_Pos)   /*!< 0x00000400 */

#define RCC_CFGR_PPRE1_DIV4 0x00000500U  /*!< HCLK divided by 4 */
#define RCC_CFGR_PPRE1_DIV8 0x00000600U  /*!< HCLK divided by 8 */
#define RCC_CFGR_PPRE1_DIV16 0x00000700U /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define RCC_CFGR_PPRE2_Pos (11U)
#define RCC_CFGR_PPRE2_Msk (0x7UL << RCC_CFGR_PPRE2_Pos) /*!< 0x00003800 */
#define RCC_CFGR_PPRE2 RCC_CFGR_PPRE2_Msk                /*!< PRE2[2:0] bits (APB2 prescaler) */
#define RCC_CFGR_PPRE2_0 (0x1UL << RCC_CFGR_PPRE2_Pos)   /*!< 0x00000800 */
#define RCC_CFGR_PPRE2_1 (0x2UL << RCC_CFGR_PPRE2_Pos)   /*!< 0x00001000 */
#define RCC_CFGR_PPRE2_2 (0x4UL << RCC_CFGR_PPRE2_Pos)   /*!< 0x00002000 */

#define RCC_CFGR_PPRE2_DIV1 0x00000000U  /*!< HCLK not divided */
#define RCC_CFGR_PPRE2_DIV2 0x00002000U  /*!< HCLK divided by 2 */
#define RCC_CFGR_PPRE2_DIV4 0x00002800U  /*!< HCLK divided by 4 */
#define RCC_CFGR_PPRE2_DIV8 0x00003000U  /*!< HCLK divided by 8 */
#define RCC_CFGR_PPRE2_DIV16 0x00003800U /*!< HCLK divided by 16 */

/*!< ADCPPRE configuration */
#define RCC_CFGR_ADCPRE_Pos (14U)
#define RCC_CFGR_ADCPRE_Msk (0x3UL << RCC_CFGR_ADCPRE_Pos) /*!< 0x0000C000 */
#define RCC_CFGR_ADCPRE RCC_CFGR_ADCPRE_Msk                /*!< ADCPRE[1:0] bits (ADC prescaler) */
#define RCC_CFGR_ADCPRE_0 (0x1UL << RCC_CFGR_ADCPRE_Pos)   /*!< 0x00004000 */
#define RCC_CFGR_ADCPRE_1 (0x2UL << RCC_CFGR_ADCPRE_Pos)   /*!< 0x00008000 */

#define RCC_CFGR_ADCPRE_DIV2 0x00000000U /*!< PCLK2 divided by 2 */
#define RCC_CFGR_ADCPRE_DIV4 0x00004000U /*!< PCLK2 divided by 4 */
#define RCC_CFGR_ADCPRE_DIV6 0x00008000U /*!< PCLK2 divided by 6 */
#define RCC_CFGR_ADCPRE_DIV8 0x0000C000U /*!< PCLK2 divided by 8 */

#define RCC_CFGR_PLLSRC_Pos (16U)
#define RCC_CFGR_PLLSRC_Msk (0x1UL << RCC_CFGR_PLLSRC_Pos) /*!< 0x00010000 */
#define RCC_CFGR_PLLSRC RCC_CFGR_PLLSRC_Msk                /*!< PLL entry clock source */

#define RCC_CFGR_PLLXTPRE_Pos (17U)
#define RCC_CFGR_PLLXTPRE_Msk (0x1UL << RCC_CFGR_PLLXTPRE_Pos) /*!< 0x00020000 */
#define RCC_CFGR_PLLXTPRE RCC_CFGR_PLLXTPRE_Msk                /*!< HSE divider for PLL entry */

/*!< PLLMUL configuration */
#define RCC_CFGR_PLLMULL_Pos (18U)
#define RCC_CFGR_PLLMULL_Msk (0xFUL << RCC_CFGR_PLLMULL_Pos) /*!< 0x003C0000 */
#define RCC_CFGR_PLLMULL RCC_CFGR_PLLMULL_Msk                /*!< PLLMUL[3:0] bits (PLL multiplication factor) */
#define RCC_CFGR_PLLMULL_0 (0x1UL << RCC_CFGR_PLLMULL_Pos)   /*!< 0x00040000 */
#define RCC_CFGR_PLLMULL_1 (0x2UL << RCC_CFGR_PLLMULL_Pos)   /*!< 0x00080000 */
#define RCC_CFGR_PLLMULL_2 (0x4UL << RCC_CFGR_PLLMULL_Pos)   /*!< 0x00100000 */
#define RCC_CFGR_PLLMULL_3 (0x8UL << RCC_CFGR_PLLMULL_Pos)   /*!< 0x00200000 */

#define RCC_CFGR_PLLXTPRE_HSE 0x00000000U      /*!< HSE clock not divided for PLL entry */
#define RCC_CFGR_PLLXTPRE_HSE_DIV2 0x00020000U /*!< HSE clock divided by 2 for PLL entry */

#define RCC_CFGR_PLLMULL2 0x00000000U /*!< PLL input clock*2 */
#define RCC_CFGR_PLLMULL3_Pos (18U)
#define RCC_CFGR_PLLMULL3_Msk (0x1UL << RCC_CFGR_PLLMULL3_Pos) /*!< 0x00040000 */
#define RCC_CFGR_PLLMULL3 RCC_CFGR_PLLMULL3_Msk                /*!< PLL input clock*3 */
#define RCC_CFGR_PLLMULL4_Pos (19U)
#define RCC_CFGR_PLLMULL4_Msk (0x1UL << RCC_CFGR_PLLMULL4_Pos) /*!< 0x00080000 */
#define RCC_CFGR_PLLMULL4 RCC_CFGR_PLLMULL4_Msk                /*!< PLL input clock*4 */
#define RCC_CFGR_PLLMULL5_Pos (18U)
#define RCC_CFGR_PLLMULL5_Msk (0x3UL << RCC_CFGR_PLLMULL5_Pos) /*!< 0x000C0000 */
#define RCC_CFGR_PLLMULL5 RCC_CFGR_PLLMULL5_Msk                /*!< PLL input clock*5 */
#define RCC_CFGR_PLLMULL6_Pos (20U)
#define RCC_CFGR_PLLMULL6_Msk (0x1UL << RCC_CFGR_PLLMULL6_Pos) /*!< 0x00100000 */
#define RCC_CFGR_PLLMULL6 RCC_CFGR_PLLMULL6_Msk                /*!< PLL input clock*6 */
#define RCC_CFGR_PLLMULL7_Pos (18U)
#define RCC_CFGR_PLLMULL7_Msk (0x5UL << RCC_CFGR_PLLMULL7_Pos) /*!< 0x00140000 */
#define RCC_CFGR_PLLMULL7 RCC_CFGR_PLLMULL7_Msk                /*!< PLL input clock*7 */
#define RCC_CFGR_PLLMULL8_Pos (19U)
#define RCC_CFGR_PLLMULL8_Msk (0x3UL << RCC_CFGR_PLLMULL8_Pos) /*!< 0x00180000 */
#define RCC_CFGR_PLLMULL8 RCC_CFGR_PLLMULL8_Msk                /*!< PLL input clock*8 */
#define RCC_CFGR_PLLMULL9_Pos (18U)
#define RCC_CFGR_PLLMULL9_Msk (0x7UL << RCC_CFGR_PLLMULL9_Pos) /*!< 0x001C0000 */
#define RCC_CFGR_PLLMULL9 RCC_CFGR_PLLMULL9_Msk                /*!< PLL input clock*9 */
#define RCC_CFGR_PLLMULL10_Pos (21U)
#define RCC_CFGR_PLLMULL10_Msk (0x1UL << RCC_CFGR_PLLMULL10_Pos) /*!< 0x00200000 */
#define RCC_CFGR_PLLMULL10 RCC_CFGR_PLLMULL10_Msk                /*!< PLL input clock10 */
#define RCC_CFGR_PLLMULL11_Pos (18U)
#define RCC_CFGR_PLLMULL11_Msk (0x9UL << RCC_CFGR_PLLMULL11_Pos) /*!< 0x00240000 */
#define RCC_CFGR_PLLMULL11 RCC_CFGR_PLLMULL11_Msk                /*!< PLL input clock*11 */
#define RCC_CFGR_PLLMULL12_Pos (19U)
#define RCC_CFGR_PLLMULL12_Msk (0x5UL << RCC_CFGR_PLLMULL12_Pos) /*!< 0x00280000 */
#define RCC_CFGR_PLLMULL12 RCC_CFGR_PLLMULL12_Msk                /*!< PLL input clock*12 */
#define RCC_CFGR_PLLMULL13_Pos (18U)
#define RCC_CFGR_PLLMULL13_Msk (0xBUL << RCC_CFGR_PLLMULL13_Pos) /*!< 0x002C0000 */
#define RCC_CFGR_PLLMULL13 RCC_CFGR_PLLMULL13_Msk                /*!< PLL input clock*13 */
#define RCC_CFGR_PLLMULL14_Pos (20U)
#define RCC_CFGR_PLLMULL14_Msk (0x3UL << RCC_CFGR_PLLMULL14_Pos) /*!< 0x00300000 */
#define RCC_CFGR_PLLMULL14 RCC_CFGR_PLLMULL14_Msk                /*!< PLL input clock*14 */
#define RCC_CFGR_PLLMULL15_Pos (18U)
#define RCC_CFGR_PLLMULL15_Msk (0xDUL << RCC_CFGR_PLLMULL15_Pos) /*!< 0x00340000 */
#define RCC_CFGR_PLLMULL15 RCC_CFGR_PLLMULL15_Msk                /*!< PLL input clock*15 */
#define RCC_CFGR_PLLMULL16_Pos (19U)
#define RCC_CFGR_PLLMULL16_Msk (0x7UL << RCC_CFGR_PLLMULL16_Pos) /*!< 0x00380000 */
#define RCC_CFGR_PLLMULL16 RCC_CFGR_PLLMULL16_Msk                /*!< PLL input clock*16 */
#define RCC_CFGR_USBPRE_Pos (22U)
#define RCC_CFGR_USBPRE_Msk (0x1UL << RCC_CFGR_USBPRE_Pos) /*!< 0x00400000 */
#define RCC_CFGR_USBPRE RCC_CFGR_USBPRE_Msk                /*!< USB Device prescaler */

/*!< MCO configuration */
#define RCC_CFGR_MCO_Pos (24U)
#define RCC_CFGR_MCO_Msk (0x7UL << RCC_CFGR_MCO_Pos) /*!< 0x07000000 */
#define RCC_CFGR_MCO RCC_CFGR_MCO_Msk                /*!< MCO[2:0] bits (Microcontroller Clock Output) */
#define RCC_CFGR_MCO_0 (0x1UL << RCC_CFGR_MCO_Pos)   /*!< 0x01000000 */
#define RCC_CFGR_MCO_1 (0x2UL << RCC_CFGR_MCO_Pos)   /*!< 0x02000000 */
#define RCC_CFGR_MCO_2 (0x4UL << RCC_CFGR_MCO_Pos)   /*!< 0x04000000 */

#define RCC_CFGR_MCO_NOCLOCK 0x00000000U     /*!< No clock */
#define RCC_CFGR_MCO_SYSCLK 0x04000000U      /*!< System clock selected as MCO source */
#define RCC_CFGR_MCO_HSI 0x05000000U         /*!< HSI clock selected as MCO source */
#define RCC_CFGR_MCO_HSE 0x06000000U         /*!< HSE clock selected as MCO source  */
#define RCC_CFGR_MCO_PLLCLK_DIV2 0x07000000U /*!< PLL clock divided by 2 selected as MCO source */

/* Reference defines */
#define RCC_CFGR_MCOSEL RCC_CFGR_MCO
#define RCC_CFGR_MCOSEL_0 RCC_CFGR_MCO_0
#define RCC_CFGR_MCOSEL_1 RCC_CFGR_MCO_1
#define RCC_CFGR_MCOSEL_2 RCC_CFGR_MCO_2
#define RCC_CFGR_MCOSEL_NOCLOCK RCC_CFGR_MCO_NOCLOCK
#define RCC_CFGR_MCOSEL_SYSCLK RCC_CFGR_MCO_SYSCLK
#define RCC_CFGR_MCOSEL_HSI RCC_CFGR_MCO_HSI
#define RCC_CFGR_MCOSEL_HSE RCC_CFGR_MCO_HSE
#define RCC_CFGR_MCOSEL_PLL_DIV2 RCC_CFGR_MCO_PLLCLK_DIV2

/*!<******************  Bit definition for RCC_CIR register  ********************/
#define RCC_CIR_LSIRDYF_Pos (0U)
#define RCC_CIR_LSIRDYF_Msk (0x1UL << RCC_CIR_LSIRDYF_Pos) /*!< 0x00000001 */
#define RCC_CIR_LSIRDYF RCC_CIR_LSIRDYF_Msk                /*!< LSI Ready Interrupt flag */
#define RCC_CIR_LSERDYF_Pos (1U)
#define RCC_CIR_LSERDYF_Msk (0x1UL << RCC_CIR_LSERDYF_Pos) /*!< 0x00000002 */
#define RCC_CIR_LSERDYF RCC_CIR_LSERDYF_Msk                /*!< LSE Ready Interrupt flag */
#define RCC_CIR_HSIRDYF_Pos (2U)
#define RCC_CIR_HSIRDYF_Msk (0x1UL << RCC_CIR_HSIRDYF_Pos) /*!< 0x00000004 */
#define RCC_CIR_HSIRDYF RCC_CIR_HSIRDYF_Msk                /*!< HSI Ready Interrupt flag */
#define RCC_CIR_HSERDYF_Pos (3U)
#define RCC_CIR_HSERDYF_Msk (0x1UL << RCC_CIR_HSERDYF_Pos) /*!< 0x00000008 */
#define RCC_CIR_HSERDYF RCC_CIR_HSERDYF_Msk                /*!< HSE Ready Interrupt flag */
#define RCC_CIR_PLLRDYF_Pos (4U)
#define RCC_CIR_PLLRDYF_Msk (0x1UL << RCC_CIR_PLLRDYF_Pos) /*!< 0x00000010 */
#define RCC_CIR_PLLRDYF RCC_CIR_PLLRDYF_Msk                /*!< PLL Ready Interrupt flag */
#define RCC_CIR_CSSF_Pos (7U)
#define RCC_CIR_CSSF_Msk (0x1UL << RCC_CIR_CSSF_Pos) /*!< 0x00000080 */
#define RCC_CIR_CSSF RCC_CIR_CSSF_Msk                /*!< Clock Security System Interrupt flag */
#define RCC_CIR_LSIRDYIE_Pos (8U)
#define RCC_CIR_LSIRDYIE_Msk (0x1UL << RCC_CIR_LSIRDYIE_Pos) /*!< 0x00000100 */
#define RCC_CIR_LSIRDYIE RCC_CIR_LSIRDYIE_Msk                /*!< LSI Ready Interrupt Enable */
#define RCC_CIR_LSERDYIE_Pos (9U)
#define RCC_CIR_LSERDYIE_Msk (0x1UL << RCC_CIR_LSERDYIE_Pos) /*!< 0x00000200 */
#define RCC_CIR_LSERDYIE RCC_CIR_LSERDYIE_Msk                /*!< LSE Ready Interrupt Enable */
#define RCC_CIR_HSIRDYIE_Pos (10U)
#define RCC_CIR_HSIRDYIE_Msk (0x1UL << RCC_CIR_HSIRDYIE_Pos) /*!< 0x00000400 */
#define RCC_CIR_HSIRDYIE RCC_CIR_HSIRDYIE_Msk                /*!< HSI Ready Interrupt Enable */
#define RCC_CIR_HSERDYIE_Pos (11U)
#define RCC_CIR_HSERDYIE_Msk (0x1UL << RCC_CIR_HSERDYIE_Pos) /*!< 0x00000800 */
#define RCC_CIR_HSERDYIE RCC_CIR_HSERDYIE_Msk                /*!< HSE Ready Interrupt Enable */
#define RCC_CIR_PLLRDYIE_Pos (12U)
#define RCC_CIR_PLLRDYIE_Msk (0x1UL << RCC_CIR_PLLRDYIE_Pos) /*!< 0x00001000 */
#define RCC_CIR_PLLRDYIE RCC_CIR_PLLRDYIE_Msk                /*!< PLL Ready Interrupt Enable */
#define RCC_CIR_LSIRDYC_Pos (16U)
#define RCC_CIR_LSIRDYC_Msk (0x1UL << RCC_CIR_LSIRDYC_Pos) /*!< 0x00010000 */
#define RCC_CIR_LSIRDYC RCC_CIR_LSIRDYC_Msk                /*!< LSI Ready Interrupt Clear */
#define RCC_CIR_LSERDYC_Pos (17U)
#define RCC_CIR_LSERDYC_Msk (0x1UL << RCC_CIR_LSERDYC_Pos) /*!< 0x00020000 */
#define RCC_CIR_LSERDYC RCC_CIR_LSERDYC_Msk                /*!< LSE Ready Interrupt Clear */
#define RCC_CIR_HSIRDYC_Pos (18U)
#define RCC_CIR_HSIRDYC_Msk (0x1UL << RCC_CIR_HSIRDYC_Pos) /*!< 0x00040000 */
#define RCC_CIR_HSIRDYC RCC_CIR_HSIRDYC_Msk                /*!< HSI Ready Interrupt Clear */
#define RCC_CIR_HSERDYC_Pos (19U)
#define RCC_CIR_HSERDYC_Msk (0x1UL << RCC_CIR_HSERDYC_Pos) /*!< 0x00080000 */
#define RCC_CIR_HSERDYC RCC_CIR_HSERDYC_Msk                /*!< HSE Ready Interrupt Clear */
#define RCC_CIR_PLLRDYC_Pos (20U)
#define RCC_CIR_PLLRDYC_Msk (0x1UL << RCC_CIR_PLLRDYC_Pos) /*!< 0x00100000 */
#define RCC_CIR_PLLRDYC RCC_CIR_PLLRDYC_Msk                /*!< PLL Ready Interrupt Clear */
#define RCC_CIR_CSSC_Pos (23U)
#define RCC_CIR_CSSC_Msk (0x1UL << RCC_CIR_CSSC_Pos) /*!< 0x00800000 */
#define RCC_CIR_CSSC RCC_CIR_CSSC_Msk                /*!< Clock Security System Interrupt Clear */

#define RCC_APB2RSTR_AFIORST_Pos (0U)
#define RCC_APB2RSTR_AFIORST_Msk (0x1UL << RCC_APB2RSTR_AFIORST_Pos) /*!< 0x00000001 */
#define RCC_APB2RSTR_AFIORST RCC_APB2RSTR_AFIORST_Msk                /*!< Alternate Function I/O reset */
#define RCC_APB2RSTR_IOPARST_Pos (2U)
#define RCC_APB2RSTR_IOPARST_Msk (0x1UL << RCC_APB2RSTR_IOPARST_Pos) /*!< 0x00000004 */
#define RCC_APB2RSTR_IOPARST RCC_APB2RSTR_IOPARST_Msk                /*!< I/O port A reset */
#define RCC_APB2RSTR_IOPBRST_Pos (3U)
#define RCC_APB2RSTR_IOPBRST_Msk (0x1UL << RCC_APB2RSTR_IOPBRST_Pos) /*!< 0x00000008 */
#define RCC_APB2RSTR_IOPBRST RCC_APB2RSTR_IOPBRST_Msk                /*!< I/O port B reset */
#define RCC_APB2RSTR_IOPCRST_Pos (4U)
#define RCC_APB2RSTR_IOPCRST_Msk (0x1UL << RCC_APB2RSTR_IOPCRST_Pos) /*!< 0x00000010 */
#define RCC_APB2RSTR_IOPCRST RCC_APB2RSTR_IOPCRST_Msk                /*!< I/O port C reset */
#define RCC_APB2RSTR_IOPDRST_Pos (5U)
#define RCC_APB2RSTR_IOPDRST_Msk (0x1UL << RCC_APB2RSTR_IOPDRST_Pos) /*!< 0x00000020 */
#define RCC_APB2RSTR_IOPDRST RCC_APB2RSTR_IOPDRST_Msk                /*!< I/O port D reset */
#define RCC_APB2RSTR_ADC1RST_Pos (9U)
#define RCC_APB2RSTR_ADC1RST_Msk (0x1UL << RCC_APB2RSTR_ADC1RST_Pos) /*!< 0x00000200 */
#define RCC_APB2RSTR_ADC1RST RCC_APB2RSTR_ADC1RST_Msk                /*!< ADC 1 interface reset */

#define RCC_APB2RSTR_ADC2RST_Pos (10U)
#define RCC_APB2RSTR_ADC2RST_Msk (0x1UL << RCC_APB2RSTR_ADC2RST_Pos) /*!< 0x00000400 */
#define RCC_APB2RSTR_ADC2RST RCC_APB2RSTR_ADC2RST_Msk                /*!< ADC 2 interface reset */

#define RCC_APB2RSTR_TIM1RST_Pos (11U)
#define RCC_APB2RSTR_TIM1RST_Msk (0x1UL << RCC_APB2RSTR_TIM1RST_Pos) /*!< 0x00000800 */
#define RCC_APB2RSTR_TIM1RST RCC_APB2RSTR_TIM1RST_Msk                /*!< TIM1 Timer reset */
#define RCC_APB2RSTR_SPI1RST_Pos (12U)
#define RCC_APB2RSTR_SPI1RST_Msk (0x1UL << RCC_APB2RSTR_SPI1RST_Pos) /*!< 0x00001000 */
#define RCC_APB2RSTR_SPI1RST RCC_APB2RSTR_SPI1RST_Msk                /*!< SPI 1 reset */
#define RCC_APB2RSTR_USART1RST_Pos (14U)
#define RCC_APB2RSTR_USART1RST_Msk (0x1UL << RCC_APB2RSTR_USART1RST_Pos) /*!< 0x00004000 */
#define RCC_APB2RSTR_USART1RST RCC_APB2RSTR_USART1RST_Msk                /*!< USART1 reset */

#define RCC_APB2RSTR_IOPERST_Pos (6U)
#define RCC_APB2RSTR_IOPERST_Msk (0x1UL << RCC_APB2RSTR_IOPERST_Pos) /*!< 0x00000040 */
#define RCC_APB2RSTR_IOPERST RCC_APB2RSTR_IOPERST_Msk                /*!< I/O port E reset */

#define RCC_APB1RSTR_TIM2RST_Pos (0U)
#define RCC_APB1RSTR_TIM2RST_Msk (0x1UL << RCC_APB1RSTR_TIM2RST_Pos) /*!< 0x00000001 */
#define RCC_APB1RSTR_TIM2RST RCC_APB1RSTR_TIM2RST_Msk                /*!< Timer 2 reset */
#define RCC_APB1RSTR_TIM3RST_Pos (1U)
#define RCC_APB1RSTR_TIM3RST_Msk (0x1UL << RCC_APB1RSTR_TIM3RST_Pos) /*!< 0x00000002 */
#define RCC_APB1RSTR_TIM3RST RCC_APB1RSTR_TIM3RST_Msk                /*!< Timer 3 reset */
#define RCC_APB1RSTR_WWDGRST_Pos (11U)
#define RCC_APB1RSTR_WWDGRST_Msk (0x1UL << RCC_APB1RSTR_WWDGRST_Pos) /*!< 0x00000800 */
#define RCC_APB1RSTR_WWDGRST RCC_APB1RSTR_WWDGRST_Msk                /*!< Window Watchdog reset */
#define RCC_APB1RSTR_USART2RST_Pos (17U)
#define RCC_APB1RSTR_USART2RST_Msk (0x1UL << RCC_APB1RSTR_USART2RST_Pos) /*!< 0x00020000 */
#define RCC_APB1RSTR_USART2RST RCC_APB1RSTR_USART2RST_Msk                /*!< USART 2 reset */
#define RCC_APB1RSTR_I2C1RST_Pos (21U)
#define RCC_APB1RSTR_I2C1RST_Msk (0x1UL << RCC_APB1RSTR_I2C1RST_Pos) /*!< 0x00200000 */
#define RCC_APB1RSTR_I2C1RST RCC_APB1RSTR_I2C1RST_Msk                /*!< I2C 1 reset */

#define RCC_APB1RSTR_CAN1RST_Pos (25U)
#define RCC_APB1RSTR_CAN1RST_Msk (0x1UL << RCC_APB1RSTR_CAN1RST_Pos) /*!< 0x02000000 */
#define RCC_APB1RSTR_CAN1RST RCC_APB1RSTR_CAN1RST_Msk                /*!< CAN1 reset */

#define RCC_APB1RSTR_BKPRST_Pos (27U)
#define RCC_APB1RSTR_BKPRST_Msk (0x1UL << RCC_APB1RSTR_BKPRST_Pos) /*!< 0x08000000 */
#define RCC_APB1RSTR_BKPRST RCC_APB1RSTR_BKPRST_Msk                /*!< Backup interface reset */
#define RCC_APB1RSTR_PWRRST_Pos (28U)
#define RCC_APB1RSTR_PWRRST_Msk (0x1UL << RCC_APB1RSTR_PWRRST_Pos) /*!< 0x10000000 */
#define RCC_APB1RSTR_PWRRST RCC_APB1RSTR_PWRRST_Msk                /*!< Power interface reset */

#define RCC_APB1RSTR_TIM4RST_Pos (2U)
#define RCC_APB1RSTR_TIM4RST_Msk (0x1UL << RCC_APB1RSTR_TIM4RST_Pos) /*!< 0x00000004 */
#define RCC_APB1RSTR_TIM4RST RCC_APB1RSTR_TIM4RST_Msk                /*!< Timer 4 reset */
#define RCC_APB1RSTR_SPI2RST_Pos (14U)
#define RCC_APB1RSTR_SPI2RST_Msk (0x1UL << RCC_APB1RSTR_SPI2RST_Pos) /*!< 0x00004000 */
#define RCC_APB1RSTR_SPI2RST RCC_APB1RSTR_SPI2RST_Msk                /*!< SPI 2 reset */
#define RCC_APB1RSTR_USART3RST_Pos (18U)
#define RCC_APB1RSTR_USART3RST_Msk (0x1UL << RCC_APB1RSTR_USART3RST_Pos) /*!< 0x00040000 */
#define RCC_APB1RSTR_USART3RST RCC_APB1RSTR_USART3RST_Msk                /*!< USART 3 reset */
#define RCC_APB1RSTR_I2C2RST_Pos (22U)
#define RCC_APB1RSTR_I2C2RST_Msk (0x1UL << RCC_APB1RSTR_I2C2RST_Pos) /*!< 0x00400000 */
#define RCC_APB1RSTR_I2C2RST RCC_APB1RSTR_I2C2RST_Msk                /*!< I2C 2 reset */

#define RCC_APB1RSTR_USBRST_Pos (23U)
#define RCC_APB1RSTR_USBRST_Msk (0x1UL << RCC_APB1RSTR_USBRST_Pos) /*!< 0x00800000 */
#define RCC_APB1RSTR_USBRST RCC_APB1RSTR_USBRST_Msk                /*!< USB Device reset */

#define RCC_AHBENR_DMA1EN_Pos (0U)
#define RCC_AHBENR_DMA1EN_Msk (0x1UL << RCC_AHBENR_DMA1EN_Pos) /*!< 0x00000001 */
#define RCC_AHBENR_DMA1EN RCC_AHBENR_DMA1EN_Msk                /*!< DMA1 clock enable */
#define RCC_AHBENR_SRAMEN_Pos (2U)
#define RCC_AHBENR_SRAMEN_Msk (0x1UL << RCC_AHBENR_SRAMEN_Pos) /*!< 0x00000004 */
#define RCC_AHBENR_SRAMEN RCC_AHBENR_SRAMEN_Msk                /*!< SRAM interface clock enable */
#define RCC_AHBENR_FLITFEN_Pos (4U)
#define RCC_AHBENR_FLITFEN_Msk (0x1UL << RCC_AHBENR_FLITFEN_Pos) /*!< 0x00000010 */
#define RCC_AHBENR_FLITFEN RCC_AHBENR_FLITFEN_Msk                /*!< FLITF clock enable */
#define RCC_AHBENR_CRCEN_Pos (6U)
#define RCC_AHBENR_CRCEN_Msk (0x1UL << RCC_AHBENR_CRCEN_Pos) /*!< 0x00000040 */
#define RCC_AHBENR_CRCEN RCC_AHBENR_CRCEN_Msk                /*!< CRC clock enable */

#define RCC_APB2ENR_AFIOEN_Pos (0U)
#define RCC_APB2ENR_AFIOEN_Msk (0x1UL << RCC_APB2ENR_AFIOEN_Pos) /*!< 0x00000001 */
#define RCC_APB2ENR_AFIOEN RCC_APB2ENR_AFIOEN_Msk                /*!< Alternate Function I/O clock enable */
#define RCC_APB2ENR_IOPAEN_Pos (2U)
#define RCC_APB2ENR_IOPAEN_Msk (0x1UL << RCC_APB2ENR_IOPAEN_Pos) /*!< 0x00000004 */
#define RCC_APB2ENR_IOPAEN RCC_APB2ENR_IOPAEN_Msk                /*!< I/O port A clock enable */
#define RCC_APB2ENR_IOPBEN_Pos (3U)
#define RCC_APB2ENR_IOPBEN_Msk (0x1UL << RCC_APB2ENR_IOPBEN_Pos) /*!< 0x00000008 */
#define RCC_APB2ENR_IOPBEN RCC_APB2ENR_IOPBEN_Msk                /*!< I/O port B clock enable */
#define RCC_APB2ENR_IOPCEN_Pos (4U)
#define RCC_APB2ENR_IOPCEN_Msk (0x1UL << RCC_APB2ENR_IOPCEN_Pos) /*!< 0x00000010 */
#define RCC_APB2ENR_IOPCEN RCC_APB2ENR_IOPCEN_Msk                /*!< I/O port C clock enable */
#define RCC_APB2ENR_IOPDEN_Pos (5U)
#define RCC_APB2ENR_IOPDEN_Msk (0x1UL << RCC_APB2ENR_IOPDEN_Pos) /*!< 0x00000020 */
#define RCC_APB2ENR_IOPDEN RCC_APB2ENR_IOPDEN_Msk                /*!< I/O port D clock enable */
#define RCC_APB2ENR_ADC1EN_Pos (9U)
#define RCC_APB2ENR_ADC1EN_Msk (0x1UL << RCC_APB2ENR_ADC1EN_Pos) /*!< 0x00000200 */
#define RCC_APB2ENR_ADC1EN RCC_APB2ENR_ADC1EN_Msk                /*!< ADC 1 interface clock enable */

#define RCC_APB2ENR_ADC2EN_Pos (10U)
#define RCC_APB2ENR_ADC2EN_Msk (0x1UL << RCC_APB2ENR_ADC2EN_Pos) /*!< 0x00000400 */
#define RCC_APB2ENR_ADC2EN RCC_APB2ENR_ADC2EN_Msk                /*!< ADC 2 interface clock enable */

#define RCC_APB2ENR_TIM1EN_Pos (11U)
#define RCC_APB2ENR_TIM1EN_Msk (0x1UL << RCC_APB2ENR_TIM1EN_Pos) /*!< 0x00000800 */
#define RCC_APB2ENR_TIM1EN RCC_APB2ENR_TIM1EN_Msk                /*!< TIM1 Timer clock enable */
#define RCC_APB2ENR_SPI1EN_Pos (12U)
#define RCC_APB2ENR_SPI1EN_Msk (0x1UL << RCC_APB2ENR_SPI1EN_Pos) /*!< 0x00001000 */
#define RCC_APB2ENR_SPI1EN RCC_APB2ENR_SPI1EN_Msk                /*!< SPI 1 clock enable */
#define RCC_APB2ENR_USART1EN_Pos (14U)
#define RCC_APB2ENR_USART1EN_Msk (0x1UL << RCC_APB2ENR_USART1EN_Pos) /*!< 0x00004000 */
#define RCC_APB2ENR_USART1EN RCC_APB2ENR_USART1EN_Msk                /*!< USART1 clock enable */

#define RCC_APB2ENR_IOPEEN_Pos (6U)
#define RCC_APB2ENR_IOPEEN_Msk (0x1UL << RCC_APB2ENR_IOPEEN_Pos) /*!< 0x00000040 */
#define RCC_APB2ENR_IOPEEN RCC_APB2ENR_IOPEEN_Msk                /*!< I/O port E clock enable */

#define RCC_APB1ENR_TIM2EN_Pos (0U)
#define RCC_APB1ENR_TIM2EN_Msk (0x1UL << RCC_APB1ENR_TIM2EN_Pos) /*!< 0x00000001 */
#define RCC_APB1ENR_TIM2EN RCC_APB1ENR_TIM2EN_Msk                /*!< Timer 2 clock enabled*/
#define RCC_APB1ENR_TIM3EN_Pos (1U)
#define RCC_APB1ENR_TIM3EN_Msk (0x1UL << RCC_APB1ENR_TIM3EN_Pos) /*!< 0x00000002 */
#define RCC_APB1ENR_TIM3EN RCC_APB1ENR_TIM3EN_Msk                /*!< Timer 3 clock enable */
#define RCC_APB1ENR_WWDGEN_Pos (11U)
#define RCC_APB1ENR_WWDGEN_Msk (0x1UL << RCC_APB1ENR_WWDGEN_Pos) /*!< 0x00000800 */
#define RCC_APB1ENR_WWDGEN RCC_APB1ENR_WWDGEN_Msk                /*!< Window Watchdog clock enable */
#define RCC_APB1ENR_USART2EN_Pos (17U)
#define RCC_APB1ENR_USART2EN_Msk (0x1UL << RCC_APB1ENR_USART2EN_Pos) /*!< 0x00020000 */
#define RCC_APB1ENR_USART2EN RCC_APB1ENR_USART2EN_Msk                /*!< USART 2 clock enable */
#define RCC_APB1ENR_I2C1EN_Pos (21U)
#define RCC_APB1ENR_I2C1EN_Msk (0x1UL << RCC_APB1ENR_I2C1EN_Pos) /*!< 0x00200000 */
#define RCC_APB1ENR_I2C1EN RCC_APB1ENR_I2C1EN_Msk                /*!< I2C 1 clock enable */

#define RCC_APB1ENR_CAN1EN_Pos (25U)
#define RCC_APB1ENR_CAN1EN_Msk (0x1UL << RCC_APB1ENR_CAN1EN_Pos) /*!< 0x02000000 */
#define RCC_APB1ENR_CAN1EN RCC_APB1ENR_CAN1EN_Msk                /*!< CAN1 clock enable */

#define RCC_APB1ENR_BKPEN_Pos (27U)
#define RCC_APB1ENR_BKPEN_Msk (0x1UL << RCC_APB1ENR_BKPEN_Pos) /*!< 0x08000000 */
#define RCC_APB1ENR_BKPEN RCC_APB1ENR_BKPEN_Msk                /*!< Backup interface clock enable */
#define RCC_APB1ENR_PWREN_Pos (28U)
#define RCC_APB1ENR_PWREN_Msk (0x1UL << RCC_APB1ENR_PWREN_Pos) /*!< 0x10000000 */
#define RCC_APB1ENR_PWREN RCC_APB1ENR_PWREN_Msk                /*!< Power interface clock enable */

#define RCC_APB1ENR_TIM4EN_Pos (2U)
#define RCC_APB1ENR_TIM4EN_Msk (0x1UL << RCC_APB1ENR_TIM4EN_Pos) /*!< 0x00000004 */
#define RCC_APB1ENR_TIM4EN RCC_APB1ENR_TIM4EN_Msk                /*!< Timer 4 clock enable */
#define RCC_APB1ENR_SPI2EN_Pos (14U)
#define RCC_APB1ENR_SPI2EN_Msk (0x1UL << RCC_APB1ENR_SPI2EN_Pos) /*!< 0x00004000 */
#define RCC_APB1ENR_SPI2EN RCC_APB1ENR_SPI2EN_Msk                /*!< SPI 2 clock enable */
#define RCC_APB1ENR_USART3EN_Pos (18U)
#define RCC_APB1ENR_USART3EN_Msk (0x1UL << RCC_APB1ENR_USART3EN_Pos) /*!< 0x00040000 */
#define RCC_APB1ENR_USART3EN RCC_APB1ENR_USART3EN_Msk                /*!< USART 3 clock enable */
#define RCC_APB1ENR_I2C2EN_Pos (22U)
#define RCC_APB1ENR_I2C2EN_Msk (0x1UL << RCC_APB1ENR_I2C2EN_Pos) /*!< 0x00400000 */
#define RCC_APB1ENR_I2C2EN RCC_APB1ENR_I2C2EN_Msk                /*!< I2C 2 clock enable */

#define RCC_APB1ENR_USBEN_Pos (23U)
#define RCC_APB1ENR_USBEN_Msk (0x1UL << RCC_APB1ENR_USBEN_Pos) /*!< 0x00800000 */
#define RCC_APB1ENR_USBEN RCC_APB1ENR_USBEN_Msk                /*!< USB Device clock enable */

#define RCC_BDCR_LSEON_Pos (0U)
#define RCC_BDCR_LSEON_Msk (0x1UL << RCC_BDCR_LSEON_Pos) /*!< 0x00000001 */
#define RCC_BDCR_LSEON RCC_BDCR_LSEON_Msk                /*!< External Low Speed oscillator enable */
#define RCC_BDCR_LSERDY_Pos (1U)
#define RCC_BDCR_LSERDY_Msk (0x1UL << RCC_BDCR_LSERDY_Pos) /*!< 0x00000002 */
#define RCC_BDCR_LSERDY RCC_BDCR_LSERDY_Msk                /*!< External Low Speed oscillator Ready */
#define RCC_BDCR_LSEBYP_Pos (2U)
#define RCC_BDCR_LSEBYP_Msk (0x1UL << RCC_BDCR_LSEBYP_Pos) /*!< 0x00000004 */
#define RCC_BDCR_LSEBYP RCC_BDCR_LSEBYP_Msk                /*!< External Low Speed oscillator Bypass */

#define RCC_BDCR_RTCSEL_Pos (8U)
#define RCC_BDCR_RTCSEL_Msk (0x3UL << RCC_BDCR_RTCSEL_Pos) /*!< 0x00000300 */
#define RCC_BDCR_RTCSEL RCC_BDCR_RTCSEL_Msk                /*!< RTCSEL[1:0] bits (RTC clock source selection) */
#define RCC_BDCR_RTCSEL_0 (0x1UL << RCC_BDCR_RTCSEL_Pos)   /*!< 0x00000100 */
#define RCC_BDCR_RTCSEL_1 (0x2UL << RCC_BDCR_RTCSEL_Pos)   /*!< 0x00000200 */

/*!< RTC configuration */
#define RCC_BDCR_RTCSEL_NOCLOCK 0x00000000U /*!< No clock */
#define RCC_BDCR_RTCSEL_LSE 0x00000100U     /*!< LSE oscillator clock used as RTC clock */
#define RCC_BDCR_RTCSEL_LSI 0x00000200U     /*!< LSI oscillator clock used as RTC clock */
#define RCC_BDCR_RTCSEL_HSE 0x00000300U     /*!< HSE oscillator clock divided by 128 used as RTC clock */

#define RCC_BDCR_RTCEN_Pos (15U)
#define RCC_BDCR_RTCEN_Msk (0x1UL << RCC_BDCR_RTCEN_Pos) /*!< 0x00008000 */
#define RCC_BDCR_RTCEN RCC_BDCR_RTCEN_Msk                /*!< RTC clock enable */
#define RCC_BDCR_BDRST_Pos (16U)
#define RCC_BDCR_BDRST_Msk (0x1UL << RCC_BDCR_BDRST_Pos) /*!< 0x00010000 */
#define RCC_BDCR_BDRST RCC_BDCR_BDRST_Msk                /*!< Backup domain software reset  */

#define RCC_CSR_LSION_Pos (0U)
#define RCC_CSR_LSION_Msk (0x1UL << RCC_CSR_LSION_Pos) /*!< 0x00000001 */
#define RCC_CSR_LSION RCC_CSR_LSION_Msk                /*!< Internal Low Speed oscillator enable */
#define RCC_CSR_LSIRDY_Pos (1U)
#define RCC_CSR_LSIRDY_Msk (0x1UL << RCC_CSR_LSIRDY_Pos) /*!< 0x00000002 */
#define RCC_CSR_LSIRDY RCC_CSR_LSIRDY_Msk                /*!< Internal Low Speed oscillator Ready */
#define RCC_CSR_RMVF_Pos (24U)
#define RCC_CSR_RMVF_Msk (0x1UL << RCC_CSR_RMVF_Pos) /*!< 0x01000000 */
#define RCC_CSR_RMVF RCC_CSR_RMVF_Msk                /*!< Remove reset flag */
#define RCC_CSR_PINRSTF_Pos (26U)
#define RCC_CSR_PINRSTF_Msk (0x1UL << RCC_CSR_PINRSTF_Pos) /*!< 0x04000000 */
#define RCC_CSR_PINRSTF RCC_CSR_PINRSTF_Msk                /*!< PIN reset flag */
#define RCC_CSR_PORRSTF_Pos (27U)
#define RCC_CSR_PORRSTF_Msk (0x1UL << RCC_CSR_PORRSTF_Pos) /*!< 0x08000000 */
#define RCC_CSR_PORRSTF RCC_CSR_PORRSTF_Msk                /*!< POR/PDR reset flag */
#define RCC_CSR_SFTRSTF_Pos (28U)
#define RCC_CSR_SFTRSTF_Msk (0x1UL << RCC_CSR_SFTRSTF_Pos) /*!< 0x10000000 */
#define RCC_CSR_SFTRSTF RCC_CSR_SFTRSTF_Msk                /*!< Software Reset flag */
#define RCC_CSR_IWDGRSTF_Pos (29U)
#define RCC_CSR_IWDGRSTF_Msk (0x1UL << RCC_CSR_IWDGRSTF_Pos) /*!< 0x20000000 */
#define RCC_CSR_IWDGRSTF RCC_CSR_IWDGRSTF_Msk                /*!< Independent Watchdog reset flag */
#define RCC_CSR_WWDGRSTF_Pos (30U)
#define RCC_CSR_WWDGRSTF_Msk (0x1UL << RCC_CSR_WWDGRSTF_Pos) /*!< 0x40000000 */
#define RCC_CSR_WWDGRSTF RCC_CSR_WWDGRSTF_Msk                /*!< Window watchdog reset flag */
#define RCC_CSR_LPWRRSTF_Pos (31U)
#define RCC_CSR_LPWRRSTF_Msk (0x1UL << RCC_CSR_LPWRRSTF_Pos) /*!< 0x80000000 */
#define RCC_CSR_LPWRRSTF RCC_CSR_LPWRRSTF_Msk                /*!< Low-Power reset flag */

/*                                                                           */
/*                               Timers (TIM)                                */
/*                                                                           */

#define TIM_CR1_CEN_Pos (0U)
#define TIM_CR1_CEN_Msk (0x1UL << TIM_CR1_CEN_Pos) /*!< 0x00000001 */
#define TIM_CR1_CEN TIM_CR1_CEN_Msk                /*!<Counter enable */
#define TIM_CR1_UDIS_Pos (1U)
#define TIM_CR1_UDIS_Msk (0x1UL << TIM_CR1_UDIS_Pos) /*!< 0x00000002 */
#define TIM_CR1_UDIS TIM_CR1_UDIS_Msk                /*!<Update disable */
#define TIM_CR1_URS_Pos (2U)
#define TIM_CR1_URS_Msk (0x1UL << TIM_CR1_URS_Pos) /*!< 0x00000004 */
#define TIM_CR1_URS TIM_CR1_URS_Msk                /*!<Update request source */
#define TIM_CR1_OPM_Pos (3U)
#define TIM_CR1_OPM_Msk (0x1UL << TIM_CR1_OPM_Pos) /*!< 0x00000008 */
#define TIM_CR1_OPM TIM_CR1_OPM_Msk                /*!<One pulse mode */
#define TIM_CR1_DIR_Pos (4U)
#define TIM_CR1_DIR_Msk (0x1UL << TIM_CR1_DIR_Pos) /*!< 0x00000010 */
#define TIM_CR1_DIR TIM_CR1_DIR_Msk                /*!<Direction */

#define TIM_CR1_CMS_Pos (5U)
#define TIM_CR1_CMS_Msk (0x3UL << TIM_CR1_CMS_Pos) /*!< 0x00000060 */
#define TIM_CR1_CMS TIM_CR1_CMS_Msk                /*!<CMS[1:0] bits (Center-aligned mode selection) */
#define TIM_CR1_CMS_0 (0x1UL << TIM_CR1_CMS_Pos)   /*!< 0x00000020 */
#define TIM_CR1_CMS_1 (0x2UL << TIM_CR1_CMS_Pos)   /*!< 0x00000040 */

#define TIM_CR1_ARPE_Pos (7U)
#define TIM_CR1_ARPE_Msk (0x1UL << TIM_CR1_ARPE_Pos) /*!< 0x00000080 */
#define TIM_CR1_ARPE TIM_CR1_ARPE_Msk                /*!<Auto-reload preload enable */

#define TIM_CR1_CKD_Pos (8U)
#define TIM_CR1_CKD_Msk (0x3UL << TIM_CR1_CKD_Pos) /*!< 0x00000300 */
#define TIM_CR1_CKD TIM_CR1_CKD_Msk                /*!<CKD[1:0] bits (clock division) */
#define TIM_CR1_CKD_0 (0x1UL << TIM_CR1_CKD_Pos)   /*!< 0x00000100 */
#define TIM_CR1_CKD_1 (0x2UL << TIM_CR1_CKD_Pos)   /*!< 0x00000200 */

#define TIM_CR2_CCPC_Pos (0U)
#define TIM_CR2_CCPC_Msk (0x1UL << TIM_CR2_CCPC_Pos) /*!< 0x00000001 */
#define TIM_CR2_CCPC TIM_CR2_CCPC_Msk                /*!<Capture/Compare Preloaded Control */
#define TIM_CR2_CCUS_Pos (2U)
#define TIM_CR2_CCUS_Msk (0x1UL << TIM_CR2_CCUS_Pos) /*!< 0x00000004 */
#define TIM_CR2_CCUS TIM_CR2_CCUS_Msk                /*!<Capture/Compare Control Update Selection */
#define TIM_CR2_CCDS_Pos (3U)
#define TIM_CR2_CCDS_Msk (0x1UL << TIM_CR2_CCDS_Pos) /*!< 0x00000008 */
#define TIM_CR2_CCDS TIM_CR2_CCDS_Msk                /*!<Capture/Compare DMA Selection */

#define TIM_CR2_MMS_Pos (4U)
#define TIM_CR2_MMS_Msk (0x7UL << TIM_CR2_MMS_Pos) /*!< 0x00000070 */
#define TIM_CR2_MMS TIM_CR2_MMS_Msk                /*!<MMS[2:0] bits (Master Mode Selection) */
#define TIM_CR2_MMS_0 (0x1UL << TIM_CR2_MMS_Pos)   /*!< 0x00000010 */
#define TIM_CR2_MMS_1 (0x2UL << TIM_CR2_MMS_Pos)   /*!< 0x00000020 */
#define TIM_CR2_MMS_2 (0x4UL << TIM_CR2_MMS_Pos)   /*!< 0x00000040 */

#define TIM_CR2_TI1S_Pos (7U)
#define TIM_CR2_TI1S_Msk (0x1UL << TIM_CR2_TI1S_Pos) /*!< 0x00000080 */
#define TIM_CR2_TI1S TIM_CR2_TI1S_Msk                /*!<TI1 Selection */
#define TIM_CR2_OIS1_Pos (8U)
#define TIM_CR2_OIS1_Msk (0x1UL << TIM_CR2_OIS1_Pos) /*!< 0x00000100 */
#define TIM_CR2_OIS1 TIM_CR2_OIS1_Msk                /*!<Output Idle state 1 (OC1 output) */
#define TIM_CR2_OIS1N_Pos (9U)
#define TIM_CR2_OIS1N_Msk (0x1UL << TIM_CR2_OIS1N_Pos) /*!< 0x00000200 */
#define TIM_CR2_OIS1N TIM_CR2_OIS1N_Msk                /*!<Output Idle state 1 (OC1N output) */
#define TIM_CR2_OIS2_Pos (10U)
#define TIM_CR2_OIS2_Msk (0x1UL << TIM_CR2_OIS2_Pos) /*!< 0x00000400 */
#define TIM_CR2_OIS2 TIM_CR2_OIS2_Msk                /*!<Output Idle state 2 (OC2 output) */
#define TIM_CR2_OIS2N_Pos (11U)
#define TIM_CR2_OIS2N_Msk (0x1UL << TIM_CR2_OIS2N_Pos) /*!< 0x00000800 */
#define TIM_CR2_OIS2N TIM_CR2_OIS2N_Msk                /*!<Output Idle state 2 (OC2N output) */
#define TIM_CR2_OIS3_Pos (12U)
#define TIM_CR2_OIS3_Msk (0x1UL << TIM_CR2_OIS3_Pos) /*!< 0x00001000 */
#define TIM_CR2_OIS3 TIM_CR2_OIS3_Msk                /*!<Output Idle state 3 (OC3 output) */
#define TIM_CR2_OIS3N_Pos (13U)
#define TIM_CR2_OIS3N_Msk (0x1UL << TIM_CR2_OIS3N_Pos) /*!< 0x00002000 */
#define TIM_CR2_OIS3N TIM_CR2_OIS3N_Msk                /*!<Output Idle state 3 (OC3N output) */
#define TIM_CR2_OIS4_Pos (14U)
#define TIM_CR2_OIS4_Msk (0x1UL << TIM_CR2_OIS4_Pos) /*!< 0x00004000 */
#define TIM_CR2_OIS4 TIM_CR2_OIS4_Msk                /*!<Output Idle state 4 (OC4 output) */

#define TIM_SMCR_SMS_Pos (0U)
#define TIM_SMCR_SMS_Msk (0x7UL << TIM_SMCR_SMS_Pos) /*!< 0x00000007 */
#define TIM_SMCR_SMS TIM_SMCR_SMS_Msk                /*!<SMS[2:0] bits (Slave mode selection) */
#define TIM_SMCR_SMS_0 (0x1UL << TIM_SMCR_SMS_Pos)   /*!< 0x00000001 */
#define TIM_SMCR_SMS_1 (0x2UL << TIM_SMCR_SMS_Pos)   /*!< 0x00000002 */
#define TIM_SMCR_SMS_2 (0x4UL << TIM_SMCR_SMS_Pos)   /*!< 0x00000004 */

#define TIM_SMCR_TS_Pos (4U)
#define TIM_SMCR_TS_Msk (0x7UL << TIM_SMCR_TS_Pos) /*!< 0x00000070 */
#define TIM_SMCR_TS TIM_SMCR_TS_Msk                /*!<TS[2:0] bits (Trigger selection) */
#define TIM_SMCR_TS_0 (0x1UL << TIM_SMCR_TS_Pos)   /*!< 0x00000010 */
#define TIM_SMCR_TS_1 (0x2UL << TIM_SMCR_TS_Pos)   /*!< 0x00000020 */
#define TIM_SMCR_TS_2 (0x4UL << TIM_SMCR_TS_Pos)   /*!< 0x00000040 */

#define TIM_SMCR_MSM_Pos (7U)
#define TIM_SMCR_MSM_Msk (0x1UL << TIM_SMCR_MSM_Pos) /*!< 0x00000080 */
#define TIM_SMCR_MSM TIM_SMCR_MSM_Msk                /*!<Master/slave mode */

#define TIM_SMCR_ETF_Pos (8U)
#define TIM_SMCR_ETF_Msk (0xFUL << TIM_SMCR_ETF_Pos) /*!< 0x00000F00 */
#define TIM_SMCR_ETF TIM_SMCR_ETF_Msk                /*!<ETF[3:0] bits (External trigger filter) */
#define TIM_SMCR_ETF_0 (0x1UL << TIM_SMCR_ETF_Pos)   /*!< 0x00000100 */
#define TIM_SMCR_ETF_1 (0x2UL << TIM_SMCR_ETF_Pos)   /*!< 0x00000200 */
#define TIM_SMCR_ETF_2 (0x4UL << TIM_SMCR_ETF_Pos)   /*!< 0x00000400 */
#define TIM_SMCR_ETF_3 (0x8UL << TIM_SMCR_ETF_Pos)   /*!< 0x00000800 */

#define TIM_SMCR_ETPS_Pos (12U)
#define TIM_SMCR_ETPS_Msk (0x3UL << TIM_SMCR_ETPS_Pos) /*!< 0x00003000 */
#define TIM_SMCR_ETPS TIM_SMCR_ETPS_Msk                /*!<ETPS[1:0] bits (External trigger prescaler) */
#define TIM_SMCR_ETPS_0 (0x1UL << TIM_SMCR_ETPS_Pos)   /*!< 0x00001000 */
#define TIM_SMCR_ETPS_1 (0x2UL << TIM_SMCR_ETPS_Pos)   /*!< 0x00002000 */

#define TIM_SMCR_ECE_Pos (14U)
#define TIM_SMCR_ECE_Msk (0x1UL << TIM_SMCR_ECE_Pos) /*!< 0x00004000 */
#define TIM_SMCR_ECE TIM_SMCR_ECE_Msk                /*!<External clock enable */
#define TIM_SMCR_ETP_Pos (15U)
#define TIM_SMCR_ETP_Msk (0x1UL << TIM_SMCR_ETP_Pos) /*!< 0x00008000 */
#define TIM_SMCR_ETP TIM_SMCR_ETP_Msk                /*!<External trigger polarity */

#define TIM_DIER_UIE_Pos (0U)
#define TIM_DIER_UIE_Msk (0x1UL << TIM_DIER_UIE_Pos) /*!< 0x00000001 */
#define TIM_DIER_UIE TIM_DIER_UIE_Msk                /*!<Update interrupt enable */
#define TIM_DIER_CC1IE_Pos (1U)
#define TIM_DIER_CC1IE_Msk (0x1UL << TIM_DIER_CC1IE_Pos) /*!< 0x00000002 */
#define TIM_DIER_CC1IE TIM_DIER_CC1IE_Msk                /*!<Capture/Compare 1 interrupt enable */
#define TIM_DIER_CC2IE_Pos (2U)
#define TIM_DIER_CC2IE_Msk (0x1UL << TIM_DIER_CC2IE_Pos) /*!< 0x00000004 */
#define TIM_DIER_CC2IE TIM_DIER_CC2IE_Msk                /*!<Capture/Compare 2 interrupt enable */
#define TIM_DIER_CC3IE_Pos (3U)
#define TIM_DIER_CC3IE_Msk (0x1UL << TIM_DIER_CC3IE_Pos) /*!< 0x00000008 */
#define TIM_DIER_CC3IE TIM_DIER_CC3IE_Msk                /*!<Capture/Compare 3 interrupt enable */
#define TIM_DIER_CC4IE_Pos (4U)
#define TIM_DIER_CC4IE_Msk (0x1UL << TIM_DIER_CC4IE_Pos) /*!< 0x00000010 */
#define TIM_DIER_CC4IE TIM_DIER_CC4IE_Msk                /*!<Capture/Compare 4 interrupt enable */
#define TIM_DIER_COMIE_Pos (5U)
#define TIM_DIER_COMIE_Msk (0x1UL << TIM_DIER_COMIE_Pos) /*!< 0x00000020 */
#define TIM_DIER_COMIE TIM_DIER_COMIE_Msk                /*!<COM interrupt enable */
#define TIM_DIER_TIE_Pos (6U)
#define TIM_DIER_TIE_Msk (0x1UL << TIM_DIER_TIE_Pos) /*!< 0x00000040 */
#define TIM_DIER_TIE TIM_DIER_TIE_Msk                /*!<Trigger interrupt enable */
#define TIM_DIER_BIE_Pos (7U)
#define TIM_DIER_BIE_Msk (0x1UL << TIM_DIER_BIE_Pos) /*!< 0x00000080 */
#define TIM_DIER_BIE TIM_DIER_BIE_Msk                /*!<Break interrupt enable */
#define TIM_DIER_UDE_Pos (8U)
#define TIM_DIER_UDE_Msk (0x1UL << TIM_DIER_UDE_Pos) /*!< 0x00000100 */
#define TIM_DIER_UDE TIM_DIER_UDE_Msk                /*!<Update DMA request enable */
#define TIM_DIER_CC1DE_Pos (9U)
#define TIM_DIER_CC1DE_Msk (0x1UL << TIM_DIER_CC1DE_Pos) /*!< 0x00000200 */
#define TIM_DIER_CC1DE TIM_DIER_CC1DE_Msk                /*!<Capture/Compare 1 DMA request enable */
#define TIM_DIER_CC2DE_Pos (10U)
#define TIM_DIER_CC2DE_Msk (0x1UL << TIM_DIER_CC2DE_Pos) /*!< 0x00000400 */
#define TIM_DIER_CC2DE TIM_DIER_CC2DE_Msk                /*!<Capture/Compare 2 DMA request enable */
#define TIM_DIER_CC3DE_Pos (11U)
#define TIM_DIER_CC3DE_Msk (0x1UL << TIM_DIER_CC3DE_Pos) /*!< 0x00000800 */
#define TIM_DIER_CC3DE TIM_DIER_CC3DE_Msk                /*!<Capture/Compare 3 DMA request enable */
#define TIM_DIER_CC4DE_Pos (12U)
#define TIM_DIER_CC4DE_Msk (0x1UL << TIM_DIER_CC4DE_Pos) /*!< 0x00001000 */
#define TIM_DIER_CC4DE TIM_DIER_CC4DE_Msk                /*!<Capture/Compare 4 DMA request enable */
#define TIM_DIER_COMDE_Pos (13U)
#define TIM_DIER_COMDE_Msk (0x1UL << TIM_DIER_COMDE_Pos) /*!< 0x00002000 */
#define TIM_DIER_COMDE TIM_DIER_COMDE_Msk                /*!<COM DMA request enable */
#define TIM_DIER_TDE_Pos (14U)
#define TIM_DIER_TDE_Msk (0x1UL << TIM_DIER_TDE_Pos) /*!< 0x00004000 */
#define TIM_DIER_TDE TIM_DIER_TDE_Msk                /*!<Trigger DMA request enable */

#define TIM_SR_UIF_Pos (0U)
#define TIM_SR_UIF_Msk (0x1UL << TIM_SR_UIF_Pos) /*!< 0x00000001 */
#define TIM_SR_UIF TIM_SR_UIF_Msk                /*!<Update interrupt Flag */
#define TIM_SR_CC1IF_Pos (1U)
#define TIM_SR_CC1IF_Msk (0x1UL << TIM_SR_CC1IF_Pos) /*!< 0x00000002 */
#define TIM_SR_CC1IF TIM_SR_CC1IF_Msk                /*!<Capture/Compare 1 interrupt Flag */
#define TIM_SR_CC2IF_Pos (2U)
#define TIM_SR_CC2IF_Msk (0x1UL << TIM_SR_CC2IF_Pos) /*!< 0x00000004 */
#define TIM_SR_CC2IF TIM_SR_CC2IF_Msk                /*!<Capture/Compare 2 interrupt Flag */
#define TIM_SR_CC3IF_Pos (3U)
#define TIM_SR_CC3IF_Msk (0x1UL << TIM_SR_CC3IF_Pos) /*!< 0x00000008 */
#define TIM_SR_CC3IF TIM_SR_CC3IF_Msk                /*!<Capture/Compare 3 interrupt Flag */
#define TIM_SR_CC4IF_Pos (4U)
#define TIM_SR_CC4IF_Msk (0x1UL << TIM_SR_CC4IF_Pos) /*!< 0x00000010 */
#define TIM_SR_CC4IF TIM_SR_CC4IF_Msk                /*!<Capture/Compare 4 interrupt Flag */
#define TIM_SR_COMIF_Pos (5U)
#define TIM_SR_COMIF_Msk (0x1UL << TIM_SR_COMIF_Pos) /*!< 0x00000020 */
#define TIM_SR_COMIF TIM_SR_COMIF_Msk                /*!<COM interrupt Flag */
#define TIM_SR_TIF_Pos (6U)
#define TIM_SR_TIF_Msk (0x1UL << TIM_SR_TIF_Pos) /*!< 0x00000040 */
#define TIM_SR_TIF TIM_SR_TIF_Msk                /*!<Trigger interrupt Flag */
#define TIM_SR_BIF_Pos (7U)
#define TIM_SR_BIF_Msk (0x1UL << TIM_SR_BIF_Pos) /*!< 0x00000080 */
#define TIM_SR_BIF TIM_SR_BIF_Msk                /*!<Break interrupt Flag */
#define TIM_SR_CC1OF_Pos (9U)
#define TIM_SR_CC1OF_Msk (0x1UL << TIM_SR_CC1OF_Pos) /*!< 0x00000200 */
#define TIM_SR_CC1OF TIM_SR_CC1OF_Msk                /*!<Capture/Compare 1 Overcapture Flag */
#define TIM_SR_CC2OF_Pos (10U)
#define TIM_SR_CC2OF_Msk (0x1UL << TIM_SR_CC2OF_Pos) /*!< 0x00000400 */
#define TIM_SR_CC2OF TIM_SR_CC2OF_Msk                /*!<Capture/Compare 2 Overcapture Flag */
#define TIM_SR_CC3OF_Pos (11U)
#define TIM_SR_CC3OF_Msk (0x1UL << TIM_SR_CC3OF_Pos) /*!< 0x00000800 */
#define TIM_SR_CC3OF TIM_SR_CC3OF_Msk                /*!<Capture/Compare 3 Overcapture Flag */
#define TIM_SR_CC4OF_Pos (12U)
#define TIM_SR_CC4OF_Msk (0x1UL << TIM_SR_CC4OF_Pos) /*!< 0x00001000 */
#define TIM_SR_CC4OF TIM_SR_CC4OF_Msk                /*!<Capture/Compare 4 Overcapture Flag */

#define TIM_EGR_UG_Pos (0U)
#define TIM_EGR_UG_Msk (0x1UL << TIM_EGR_UG_Pos) /*!< 0x00000001 */
#define TIM_EGR_UG TIM_EGR_UG_Msk                /*!<Update Generation */
#define TIM_EGR_CC1G_Pos (1U)
#define TIM_EGR_CC1G_Msk (0x1UL << TIM_EGR_CC1G_Pos) /*!< 0x00000002 */
#define TIM_EGR_CC1G TIM_EGR_CC1G_Msk                /*!<Capture/Compare 1 Generation */
#define TIM_EGR_CC2G_Pos (2U)
#define TIM_EGR_CC2G_Msk (0x1UL << TIM_EGR_CC2G_Pos) /*!< 0x00000004 */
#define TIM_EGR_CC2G TIM_EGR_CC2G_Msk                /*!<Capture/Compare 2 Generation */
#define TIM_EGR_CC3G_Pos (3U)
#define TIM_EGR_CC3G_Msk (0x1UL << TIM_EGR_CC3G_Pos) /*!< 0x00000008 */
#define TIM_EGR_CC3G TIM_EGR_CC3G_Msk                /*!<Capture/Compare 3 Generation */
#define TIM_EGR_CC4G_Pos (4U)
#define TIM_EGR_CC4G_Msk (0x1UL << TIM_EGR_CC4G_Pos) /*!< 0x00000010 */
#define TIM_EGR_CC4G TIM_EGR_CC4G_Msk                /*!<Capture/Compare 4 Generation */
#define TIM_EGR_COMG_Pos (5U)
#define TIM_EGR_COMG_Msk (0x1UL << TIM_EGR_COMG_Pos) /*!< 0x00000020 */
#define TIM_EGR_COMG TIM_EGR_COMG_Msk                /*!<Capture/Compare Control Update Generation */
#define TIM_EGR_TG_Pos (6U)
#define TIM_EGR_TG_Msk (0x1UL << TIM_EGR_TG_Pos) /*!< 0x00000040 */
#define TIM_EGR_TG TIM_EGR_TG_Msk                /*!<Trigger Generation */
#define TIM_EGR_BG_Pos (7U)
#define TIM_EGR_BG_Msk (0x1UL << TIM_EGR_BG_Pos) /*!< 0x00000080 */
#define TIM_EGR_BG TIM_EGR_BG_Msk                /*!<Break Generation */

#define TIM_CCMR1_CC1S_Pos (0U)
#define TIM_CCMR1_CC1S_Msk (0x3UL << TIM_CCMR1_CC1S_Pos) /*!< 0x00000003 */
#define TIM_CCMR1_CC1S TIM_CCMR1_CC1S_Msk                /*!<CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define TIM_CCMR1_CC1S_0 (0x1UL << TIM_CCMR1_CC1S_Pos)   /*!< 0x00000001 */
#define TIM_CCMR1_CC1S_1 (0x2UL << TIM_CCMR1_CC1S_Pos)   /*!< 0x00000002 */

#define TIM_CCMR1_OC1FE_Pos (2U)
#define TIM_CCMR1_OC1FE_Msk (0x1UL << TIM_CCMR1_OC1FE_Pos) /*!< 0x00000004 */
#define TIM_CCMR1_OC1FE TIM_CCMR1_OC1FE_Msk                /*!<Output Compare 1 Fast enable */
#define TIM_CCMR1_OC1PE_Pos (3U)
#define TIM_CCMR1_OC1PE_Msk (0x1UL << TIM_CCMR1_OC1PE_Pos) /*!< 0x00000008 */
#define TIM_CCMR1_OC1PE TIM_CCMR1_OC1PE_Msk                /*!<Output Compare 1 Preload enable */

#define TIM_CCMR1_OC1M_Pos (4U)
#define TIM_CCMR1_OC1M_Msk (0x7UL << TIM_CCMR1_OC1M_Pos) /*!< 0x00000070 */
#define TIM_CCMR1_OC1M TIM_CCMR1_OC1M_Msk                /*!<OC1M[2:0] bits (Output Compare 1 Mode) */
#define TIM_CCMR1_OC1M_0 (0x1UL << TIM_CCMR1_OC1M_Pos)   /*!< 0x00000010 */
#define TIM_CCMR1_OC1M_1 (0x2UL << TIM_CCMR1_OC1M_Pos)   /*!< 0x00000020 */
#define TIM_CCMR1_OC1M_2 (0x4UL << TIM_CCMR1_OC1M_Pos)   /*!< 0x00000040 */

#define TIM_CCMR1_OC1CE_Pos (7U)
#define TIM_CCMR1_OC1CE_Msk (0x1UL << TIM_CCMR1_OC1CE_Pos) /*!< 0x00000080 */
#define TIM_CCMR1_OC1CE TIM_CCMR1_OC1CE_Msk                /*!<Output Compare 1Clear Enable */

#define TIM_CCMR1_CC2S_Pos (8U)
#define TIM_CCMR1_CC2S_Msk (0x3UL << TIM_CCMR1_CC2S_Pos) /*!< 0x00000300 */
#define TIM_CCMR1_CC2S TIM_CCMR1_CC2S_Msk                /*!<CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define TIM_CCMR1_CC2S_0 (0x1UL << TIM_CCMR1_CC2S_Pos)   /*!< 0x00000100 */
#define TIM_CCMR1_CC2S_1 (0x2UL << TIM_CCMR1_CC2S_Pos)   /*!< 0x00000200 */

#define TIM_CCMR1_OC2FE_Pos (10U)
#define TIM_CCMR1_OC2FE_Msk (0x1UL << TIM_CCMR1_OC2FE_Pos) /*!< 0x00000400 */
#define TIM_CCMR1_OC2FE TIM_CCMR1_OC2FE_Msk                /*!<Output Compare 2 Fast enable */
#define TIM_CCMR1_OC2PE_Pos (11U)
#define TIM_CCMR1_OC2PE_Msk (0x1UL << TIM_CCMR1_OC2PE_Pos) /*!< 0x00000800 */
#define TIM_CCMR1_OC2PE TIM_CCMR1_OC2PE_Msk                /*!<Output Compare 2 Preload enable */

#define TIM_CCMR1_OC2M_Pos (12U)
#define TIM_CCMR1_OC2M_Msk (0x7UL << TIM_CCMR1_OC2M_Pos) /*!< 0x00007000 */
#define TIM_CCMR1_OC2M TIM_CCMR1_OC2M_Msk                /*!<OC2M[2:0] bits (Output Compare 2 Mode) */
#define TIM_CCMR1_OC2M_0 (0x1UL << TIM_CCMR1_OC2M_Pos)   /*!< 0x00001000 */
#define TIM_CCMR1_OC2M_1 (0x2UL << TIM_CCMR1_OC2M_Pos)   /*!< 0x00002000 */
#define TIM_CCMR1_OC2M_2 (0x4UL << TIM_CCMR1_OC2M_Pos)   /*!< 0x00004000 */

#define TIM_CCMR1_OC2CE_Pos (15U)
#define TIM_CCMR1_OC2CE_Msk (0x1UL << TIM_CCMR1_OC2CE_Pos) /*!< 0x00008000 */
#define TIM_CCMR1_OC2CE TIM_CCMR1_OC2CE_Msk                /*!<Output Compare 2 Clear Enable */

/*---------------------------------------------------------------------------*/

#define TIM_CCMR1_IC1PSC_Pos (2U)
#define TIM_CCMR1_IC1PSC_Msk (0x3UL << TIM_CCMR1_IC1PSC_Pos) /*!< 0x0000000C */
#define TIM_CCMR1_IC1PSC TIM_CCMR1_IC1PSC_Msk                /*!<IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define TIM_CCMR1_IC1PSC_0 (0x1UL << TIM_CCMR1_IC1PSC_Pos)   /*!< 0x00000004 */
#define TIM_CCMR1_IC1PSC_1 (0x2UL << TIM_CCMR1_IC1PSC_Pos)   /*!< 0x00000008 */

#define TIM_CCMR1_IC1F_Pos (4U)
#define TIM_CCMR1_IC1F_Msk (0xFUL << TIM_CCMR1_IC1F_Pos) /*!< 0x000000F0 */
#define TIM_CCMR1_IC1F TIM_CCMR1_IC1F_Msk                /*!<IC1F[3:0] bits (Input Capture 1 Filter) */
#define TIM_CCMR1_IC1F_0 (0x1UL << TIM_CCMR1_IC1F_Pos)   /*!< 0x00000010 */
#define TIM_CCMR1_IC1F_1 (0x2UL << TIM_CCMR1_IC1F_Pos)   /*!< 0x00000020 */
#define TIM_CCMR1_IC1F_2 (0x4UL << TIM_CCMR1_IC1F_Pos)   /*!< 0x00000040 */
#define TIM_CCMR1_IC1F_3 (0x8UL << TIM_CCMR1_IC1F_Pos)   /*!< 0x00000080 */

#define TIM_CCMR1_IC2PSC_Pos (10U)
#define TIM_CCMR1_IC2PSC_Msk (0x3UL << TIM_CCMR1_IC2PSC_Pos) /*!< 0x00000C00 */
#define TIM_CCMR1_IC2PSC TIM_CCMR1_IC2PSC_Msk                /*!<IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define TIM_CCMR1_IC2PSC_0 (0x1UL << TIM_CCMR1_IC2PSC_Pos)   /*!< 0x00000400 */
#define TIM_CCMR1_IC2PSC_1 (0x2UL << TIM_CCMR1_IC2PSC_Pos)   /*!< 0x00000800 */

#define TIM_CCMR1_IC2F_Pos (12U)
#define TIM_CCMR1_IC2F_Msk (0xFUL << TIM_CCMR1_IC2F_Pos) /*!< 0x0000F000 */
#define TIM_CCMR1_IC2F TIM_CCMR1_IC2F_Msk                /*!<IC2F[3:0] bits (Input Capture 2 Filter) */
#define TIM_CCMR1_IC2F_0 (0x1UL << TIM_CCMR1_IC2F_Pos)   /*!< 0x00001000 */
#define TIM_CCMR1_IC2F_1 (0x2UL << TIM_CCMR1_IC2F_Pos)   /*!< 0x00002000 */
#define TIM_CCMR1_IC2F_2 (0x4UL << TIM_CCMR1_IC2F_Pos)   /*!< 0x00004000 */
#define TIM_CCMR1_IC2F_3 (0x8UL << TIM_CCMR1_IC2F_Pos)   /*!< 0x00008000 */

#define TIM_CCMR2_CC3S_Pos (0U)
#define TIM_CCMR2_CC3S_Msk (0x3UL << TIM_CCMR2_CC3S_Pos) /*!< 0x00000003 */
#define TIM_CCMR2_CC3S TIM_CCMR2_CC3S_Msk                /*!<CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define TIM_CCMR2_CC3S_0 (0x1UL << TIM_CCMR2_CC3S_Pos)   /*!< 0x00000001 */
#define TIM_CCMR2_CC3S_1 (0x2UL << TIM_CCMR2_CC3S_Pos)   /*!< 0x00000002 */

#define TIM_CCMR2_OC3FE_Pos (2U)
#define TIM_CCMR2_OC3FE_Msk (0x1UL << TIM_CCMR2_OC3FE_Pos) /*!< 0x00000004 */
#define TIM_CCMR2_OC3FE TIM_CCMR2_OC3FE_Msk                /*!<Output Compare 3 Fast enable */
#define TIM_CCMR2_OC3PE_Pos (3U)
#define TIM_CCMR2_OC3PE_Msk (0x1UL << TIM_CCMR2_OC3PE_Pos) /*!< 0x00000008 */
#define TIM_CCMR2_OC3PE TIM_CCMR2_OC3PE_Msk                /*!<Output Compare 3 Preload enable */

#define TIM_CCMR2_OC3M_Pos (4U)
#define TIM_CCMR2_OC3M_Msk (0x7UL << TIM_CCMR2_OC3M_Pos) /*!< 0x00000070 */
#define TIM_CCMR2_OC3M TIM_CCMR2_OC3M_Msk                /*!<OC3M[2:0] bits (Output Compare 3 Mode) */
#define TIM_CCMR2_OC3M_0 (0x1UL << TIM_CCMR2_OC3M_Pos)   /*!< 0x00000010 */
#define TIM_CCMR2_OC3M_1 (0x2UL << TIM_CCMR2_OC3M_Pos)   /*!< 0x00000020 */
#define TIM_CCMR2_OC3M_2 (0x4UL << TIM_CCMR2_OC3M_Pos)   /*!< 0x00000040 */

#define TIM_CCMR2_OC3CE_Pos (7U)
#define TIM_CCMR2_OC3CE_Msk (0x1UL << TIM_CCMR2_OC3CE_Pos) /*!< 0x00000080 */
#define TIM_CCMR2_OC3CE TIM_CCMR2_OC3CE_Msk                /*!<Output Compare 3 Clear Enable */

#define TIM_CCMR2_CC4S_Pos (8U)
#define TIM_CCMR2_CC4S_Msk (0x3UL << TIM_CCMR2_CC4S_Pos) /*!< 0x00000300 */
#define TIM_CCMR2_CC4S TIM_CCMR2_CC4S_Msk                /*!<CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define TIM_CCMR2_CC4S_0 (0x1UL << TIM_CCMR2_CC4S_Pos)   /*!< 0x00000100 */
#define TIM_CCMR2_CC4S_1 (0x2UL << TIM_CCMR2_CC4S_Pos)   /*!< 0x00000200 */

#define TIM_CCMR2_OC4FE_Pos (10U)
#define TIM_CCMR2_OC4FE_Msk (0x1UL << TIM_CCMR2_OC4FE_Pos) /*!< 0x00000400 */
#define TIM_CCMR2_OC4FE TIM_CCMR2_OC4FE_Msk                /*!<Output Compare 4 Fast enable */
#define TIM_CCMR2_OC4PE_Pos (11U)
#define TIM_CCMR2_OC4PE_Msk (0x1UL << TIM_CCMR2_OC4PE_Pos) /*!< 0x00000800 */
#define TIM_CCMR2_OC4PE TIM_CCMR2_OC4PE_Msk                /*!<Output Compare 4 Preload enable */

#define TIM_CCMR2_OC4M_Pos (12U)
#define TIM_CCMR2_OC4M_Msk (0x7UL << TIM_CCMR2_OC4M_Pos) /*!< 0x00007000 */
#define TIM_CCMR2_OC4M TIM_CCMR2_OC4M_Msk                /*!<OC4M[2:0] bits (Output Compare 4 Mode) */
#define TIM_CCMR2_OC4M_0 (0x1UL << TIM_CCMR2_OC4M_Pos)   /*!< 0x00001000 */
#define TIM_CCMR2_OC4M_1 (0x2UL << TIM_CCMR2_OC4M_Pos)   /*!< 0x00002000 */
#define TIM_CCMR2_OC4M_2 (0x4UL << TIM_CCMR2_OC4M_Pos)   /*!< 0x00004000 */

#define TIM_CCMR2_OC4CE_Pos (15U)
#define TIM_CCMR2_OC4CE_Msk (0x1UL << TIM_CCMR2_OC4CE_Pos) /*!< 0x00008000 */
#define TIM_CCMR2_OC4CE TIM_CCMR2_OC4CE_Msk                /*!<Output Compare 4 Clear Enable */

/*---------------------------------------------------------------------------*/

#define TIM_CCMR2_IC3PSC_Pos (2U)
#define TIM_CCMR2_IC3PSC_Msk (0x3UL << TIM_CCMR2_IC3PSC_Pos) /*!< 0x0000000C */
#define TIM_CCMR2_IC3PSC TIM_CCMR2_IC3PSC_Msk                /*!<IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define TIM_CCMR2_IC3PSC_0 (0x1UL << TIM_CCMR2_IC3PSC_Pos)   /*!< 0x00000004 */
#define TIM_CCMR2_IC3PSC_1 (0x2UL << TIM_CCMR2_IC3PSC_Pos)   /*!< 0x00000008 */

#define TIM_CCMR2_IC3F_Pos (4U)
#define TIM_CCMR2_IC3F_Msk (0xFUL << TIM_CCMR2_IC3F_Pos) /*!< 0x000000F0 */
#define TIM_CCMR2_IC3F TIM_CCMR2_IC3F_Msk                /*!<IC3F[3:0] bits (Input Capture 3 Filter) */
#define TIM_CCMR2_IC3F_0 (0x1UL << TIM_CCMR2_IC3F_Pos)   /*!< 0x00000010 */
#define TIM_CCMR2_IC3F_1 (0x2UL << TIM_CCMR2_IC3F_Pos)   /*!< 0x00000020 */
#define TIM_CCMR2_IC3F_2 (0x4UL << TIM_CCMR2_IC3F_Pos)   /*!< 0x00000040 */
#define TIM_CCMR2_IC3F_3 (0x8UL << TIM_CCMR2_IC3F_Pos)   /*!< 0x00000080 */

#define TIM_CCMR2_IC4PSC_Pos (10U)
#define TIM_CCMR2_IC4PSC_Msk (0x3UL << TIM_CCMR2_IC4PSC_Pos) /*!< 0x00000C00 */
#define TIM_CCMR2_IC4PSC TIM_CCMR2_IC4PSC_Msk                /*!<IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define TIM_CCMR2_IC4PSC_0 (0x1UL << TIM_CCMR2_IC4PSC_Pos)   /*!< 0x00000400 */
#define TIM_CCMR2_IC4PSC_1 (0x2UL << TIM_CCMR2_IC4PSC_Pos)   /*!< 0x00000800 */

#define TIM_CCMR2_IC4F_Pos (12U)
#define TIM_CCMR2_IC4F_Msk (0xFUL << TIM_CCMR2_IC4F_Pos) /*!< 0x0000F000 */
#define TIM_CCMR2_IC4F TIM_CCMR2_IC4F_Msk                /*!<IC4F[3:0] bits (Input Capture 4 Filter) */
#define TIM_CCMR2_IC4F_0 (0x1UL << TIM_CCMR2_IC4F_Pos)   /*!< 0x00001000 */
#define TIM_CCMR2_IC4F_1 (0x2UL << TIM_CCMR2_IC4F_Pos)   /*!< 0x00002000 */
#define TIM_CCMR2_IC4F_2 (0x4UL << TIM_CCMR2_IC4F_Pos)   /*!< 0x00004000 */
#define TIM_CCMR2_IC4F_3 (0x8UL << TIM_CCMR2_IC4F_Pos)   /*!< 0x00008000 */

#define TIM_CCER_CC1E_Pos (0U)
#define TIM_CCER_CC1E_Msk (0x1UL << TIM_CCER_CC1E_Pos) /*!< 0x00000001 */
#define TIM_CCER_CC1E TIM_CCER_CC1E_Msk                /*!<Capture/Compare 1 output enable */
#define TIM_CCER_CC1P_Pos (1U)
#define TIM_CCER_CC1P_Msk (0x1UL << TIM_CCER_CC1P_Pos) /*!< 0x00000002 */
#define TIM_CCER_CC1P TIM_CCER_CC1P_Msk                /*!<Capture/Compare 1 output Polarity */
#define TIM_CCER_CC1NE_Pos (2U)
#define TIM_CCER_CC1NE_Msk (0x1UL << TIM_CCER_CC1NE_Pos) /*!< 0x00000004 */
#define TIM_CCER_CC1NE TIM_CCER_CC1NE_Msk                /*!<Capture/Compare 1 Complementary output enable */
#define TIM_CCER_CC1NP_Pos (3U)
#define TIM_CCER_CC1NP_Msk (0x1UL << TIM_CCER_CC1NP_Pos) /*!< 0x00000008 */
#define TIM_CCER_CC1NP TIM_CCER_CC1NP_Msk                /*!<Capture/Compare 1 Complementary output Polarity */
#define TIM_CCER_CC2E_Pos (4U)
#define TIM_CCER_CC2E_Msk (0x1UL << TIM_CCER_CC2E_Pos) /*!< 0x00000010 */
#define TIM_CCER_CC2E TIM_CCER_CC2E_Msk                /*!<Capture/Compare 2 output enable */
#define TIM_CCER_CC2P_Pos (5U)
#define TIM_CCER_CC2P_Msk (0x1UL << TIM_CCER_CC2P_Pos) /*!< 0x00000020 */
#define TIM_CCER_CC2P TIM_CCER_CC2P_Msk                /*!<Capture/Compare 2 output Polarity */
#define TIM_CCER_CC2NE_Pos (6U)
#define TIM_CCER_CC2NE_Msk (0x1UL << TIM_CCER_CC2NE_Pos) /*!< 0x00000040 */
#define TIM_CCER_CC2NE TIM_CCER_CC2NE_Msk                /*!<Capture/Compare 2 Complementary output enable */
#define TIM_CCER_CC2NP_Pos (7U)
#define TIM_CCER_CC2NP_Msk (0x1UL << TIM_CCER_CC2NP_Pos) /*!< 0x00000080 */
#define TIM_CCER_CC2NP TIM_CCER_CC2NP_Msk                /*!<Capture/Compare 2 Complementary output Polarity */
#define TIM_CCER_CC3E_Pos (8U)
#define TIM_CCER_CC3E_Msk (0x1UL << TIM_CCER_CC3E_Pos) /*!< 0x00000100 */
#define TIM_CCER_CC3E TIM_CCER_CC3E_Msk                /*!<Capture/Compare 3 output enable */
#define TIM_CCER_CC3P_Pos (9U)
#define TIM_CCER_CC3P_Msk (0x1UL << TIM_CCER_CC3P_Pos) /*!< 0x00000200 */
#define TIM_CCER_CC3P TIM_CCER_CC3P_Msk                /*!<Capture/Compare 3 output Polarity */
#define TIM_CCER_CC3NE_Pos (10U)
#define TIM_CCER_CC3NE_Msk (0x1UL << TIM_CCER_CC3NE_Pos) /*!< 0x00000400 */
#define TIM_CCER_CC3NE TIM_CCER_CC3NE_Msk                /*!<Capture/Compare 3 Complementary output enable */
#define TIM_CCER_CC3NP_Pos (11U)
#define TIM_CCER_CC3NP_Msk (0x1UL << TIM_CCER_CC3NP_Pos) /*!< 0x00000800 */
#define TIM_CCER_CC3NP TIM_CCER_CC3NP_Msk                /*!<Capture/Compare 3 Complementary output Polarity */
#define TIM_CCER_CC4E_Pos (12U)
#define TIM_CCER_CC4E_Msk (0x1UL << TIM_CCER_CC4E_Pos) /*!< 0x00001000 */
#define TIM_CCER_CC4E TIM_CCER_CC4E_Msk                /*!<Capture/Compare 4 output enable */
#define TIM_CCER_CC4P_Pos (13U)
#define TIM_CCER_CC4P_Msk (0x1UL << TIM_CCER_CC4P_Pos) /*!< 0x00002000 */
#define TIM_CCER_CC4P TIM_CCER_CC4P_Msk                /*!<Capture/Compare 4 output Polarity */

#define TIM_CNT_CNT_Pos (0U)
#define TIM_CNT_CNT_Msk (0xFFFFFFFFUL << TIM_CNT_CNT_Pos) /*!< 0xFFFFFFFF */
#define TIM_CNT_CNT TIM_CNT_CNT_Msk                       /*!<Counter Value */

#define TIM_PSC_PSC_Pos (0U)
#define TIM_PSC_PSC_Msk (0xFFFFUL << TIM_PSC_PSC_Pos) /*!< 0x0000FFFF */
#define TIM_PSC_PSC TIM_PSC_PSC_Msk                   /*!<Prescaler Value */

#define TIM_ARR_ARR_Pos (0U)
#define TIM_ARR_ARR_Msk (0xFFFFFFFFUL << TIM_ARR_ARR_Pos) /*!< 0xFFFFFFFF */
#define TIM_ARR_ARR TIM_ARR_ARR_Msk                       /*!<actual auto-reload Value */

#define TIM_RCR_REP_Pos (0U)
#define TIM_RCR_REP_Msk (0xFFUL << TIM_RCR_REP_Pos) /*!< 0x000000FF */
#define TIM_RCR_REP TIM_RCR_REP_Msk                 /*!<Repetition Counter Value */

#define TIM_CCR1_CCR1_Pos (0U)
#define TIM_CCR1_CCR1_Msk (0xFFFFUL << TIM_CCR1_CCR1_Pos) /*!< 0x0000FFFF */
#define TIM_CCR1_CCR1 TIM_CCR1_CCR1_Msk                   /*!<Capture/Compare 1 Value */

#define TIM_CCR2_CCR2_Pos (0U)
#define TIM_CCR2_CCR2_Msk (0xFFFFUL << TIM_CCR2_CCR2_Pos) /*!< 0x0000FFFF */
#define TIM_CCR2_CCR2 TIM_CCR2_CCR2_Msk                   /*!<Capture/Compare 2 Value */

#define TIM_CCR3_CCR3_Pos (0U)
#define TIM_CCR3_CCR3_Msk (0xFFFFUL << TIM_CCR3_CCR3_Pos) /*!< 0x0000FFFF */
#define TIM_CCR3_CCR3 TIM_CCR3_CCR3_Msk                   /*!<Capture/Compare 3 Value */

#define TIM_CCR4_CCR4_Pos (0U)
#define TIM_CCR4_CCR4_Msk (0xFFFFUL << TIM_CCR4_CCR4_Pos) /*!< 0x0000FFFF */
#define TIM_CCR4_CCR4 TIM_CCR4_CCR4_Msk                   /*!<Capture/Compare 4 Value */

#define TIM_BDTR_DTG_Pos (0U)
#define TIM_BDTR_DTG_Msk (0xFFUL << TIM_BDTR_DTG_Pos) /*!< 0x000000FF */
#define TIM_BDTR_DTG TIM_BDTR_DTG_Msk                 /*!<DTG[0:7] bits (Dead-Time Generator set-up) */
#define TIM_BDTR_DTG_0 (0x01UL << TIM_BDTR_DTG_Pos)   /*!< 0x00000001 */
#define TIM_BDTR_DTG_1 (0x02UL << TIM_BDTR_DTG_Pos)   /*!< 0x00000002 */
#define TIM_BDTR_DTG_2 (0x04UL << TIM_BDTR_DTG_Pos)   /*!< 0x00000004 */
#define TIM_BDTR_DTG_3 (0x08UL << TIM_BDTR_DTG_Pos)   /*!< 0x00000008 */
#define TIM_BDTR_DTG_4 (0x10UL << TIM_BDTR_DTG_Pos)   /*!< 0x00000010 */
#define TIM_BDTR_DTG_5 (0x20UL << TIM_BDTR_DTG_Pos)   /*!< 0x00000020 */
#define TIM_BDTR_DTG_6 (0x40UL << TIM_BDTR_DTG_Pos)   /*!< 0x00000040 */
#define TIM_BDTR_DTG_7 (0x80UL << TIM_BDTR_DTG_Pos)   /*!< 0x00000080 */

#define TIM_BDTR_LOCK_Pos (8U)
#define TIM_BDTR_LOCK_Msk (0x3UL << TIM_BDTR_LOCK_Pos) /*!< 0x00000300 */
#define TIM_BDTR_LOCK TIM_BDTR_LOCK_Msk                /*!<LOCK[1:0] bits (Lock Configuration) */
#define TIM_BDTR_LOCK_0 (0x1UL << TIM_BDTR_LOCK_Pos)   /*!< 0x00000100 */
#define TIM_BDTR_LOCK_1 (0x2UL << TIM_BDTR_LOCK_Pos)   /*!< 0x00000200 */

#define TIM_BDTR_OSSI_Pos (10U)
#define TIM_BDTR_OSSI_Msk (0x1UL << TIM_BDTR_OSSI_Pos) /*!< 0x00000400 */
#define TIM_BDTR_OSSI TIM_BDTR_OSSI_Msk                /*!<Off-State Selection for Idle mode */
#define TIM_BDTR_OSSR_Pos (11U)
#define TIM_BDTR_OSSR_Msk (0x1UL << TIM_BDTR_OSSR_Pos) /*!< 0x00000800 */
#define TIM_BDTR_OSSR TIM_BDTR_OSSR_Msk                /*!<Off-State Selection for Run mode */
#define TIM_BDTR_BKE_Pos (12U)
#define TIM_BDTR_BKE_Msk (0x1UL << TIM_BDTR_BKE_Pos) /*!< 0x00001000 */
#define TIM_BDTR_BKE TIM_BDTR_BKE_Msk                /*!<Break enable */
#define TIM_BDTR_BKP_Pos (13U)
#define TIM_BDTR_BKP_Msk (0x1UL << TIM_BDTR_BKP_Pos) /*!< 0x00002000 */
#define TIM_BDTR_BKP TIM_BDTR_BKP_Msk                /*!<Break Polarity */
#define TIM_BDTR_AOE_Pos (14U)
#define TIM_BDTR_AOE_Msk (0x1UL << TIM_BDTR_AOE_Pos) /*!< 0x00004000 */
#define TIM_BDTR_AOE TIM_BDTR_AOE_Msk                /*!<Automatic Output enable */
#define TIM_BDTR_MOE_Pos (15U)
#define TIM_BDTR_MOE_Msk (0x1UL << TIM_BDTR_MOE_Pos) /*!< 0x00008000 */
#define TIM_BDTR_MOE TIM_BDTR_MOE_Msk                /*!<Main Output enable */

#define TIM_DCR_DBA_Pos (0U)
#define TIM_DCR_DBA_Msk (0x1FUL << TIM_DCR_DBA_Pos) /*!< 0x0000001F */
#define TIM_DCR_DBA TIM_DCR_DBA_Msk                 /*!<DBA[4:0] bits (DMA Base Address) */
#define TIM_DCR_DBA_0 (0x01UL << TIM_DCR_DBA_Pos)   /*!< 0x00000001 */
#define TIM_DCR_DBA_1 (0x02UL << TIM_DCR_DBA_Pos)   /*!< 0x00000002 */
#define TIM_DCR_DBA_2 (0x04UL << TIM_DCR_DBA_Pos)   /*!< 0x00000004 */
#define TIM_DCR_DBA_3 (0x08UL << TIM_DCR_DBA_Pos)   /*!< 0x00000008 */
#define TIM_DCR_DBA_4 (0x10UL << TIM_DCR_DBA_Pos)   /*!< 0x00000010 */

#define TIM_DCR_DBL_Pos (8U)
#define TIM_DCR_DBL_Msk (0x1FUL << TIM_DCR_DBL_Pos) /*!< 0x00001F00 */
#define TIM_DCR_DBL TIM_DCR_DBL_Msk                 /*!<DBL[4:0] bits (DMA Burst Length) */
#define TIM_DCR_DBL_0 (0x01UL << TIM_DCR_DBL_Pos)   /*!< 0x00000100 */
#define TIM_DCR_DBL_1 (0x02UL << TIM_DCR_DBL_Pos)   /*!< 0x00000200 */
#define TIM_DCR_DBL_2 (0x04UL << TIM_DCR_DBL_Pos)   /*!< 0x00000400 */
#define TIM_DCR_DBL_3 (0x08UL << TIM_DCR_DBL_Pos)   /*!< 0x00000800 */
#define TIM_DCR_DBL_4 (0x10UL << TIM_DCR_DBL_Pos)   /*!< 0x00001000 */

#define TIM_DMAR_DMAB_Pos (0U)
#define TIM_DMAR_DMAB_Msk (0xFFFFUL << TIM_DMAR_DMAB_Pos) /*!< 0x0000FFFF */
#define TIM_DMAR_DMAB TIM_DMAR_DMAB_Msk                   /*!<DMA register for burst accesses */

/*                                                                            */
/*                             Real-Time Clock                                */
/*                                                                            */

#define RTC_CRH_SECIE_Pos (0U)
#define RTC_CRH_SECIE_Msk (0x1UL << RTC_CRH_SECIE_Pos) /*!< 0x00000001 */
#define RTC_CRH_SECIE RTC_CRH_SECIE_Msk                /*!< Second Interrupt Enable */
#define RTC_CRH_ALRIE_Pos (1U)
#define RTC_CRH_ALRIE_Msk (0x1UL << RTC_CRH_ALRIE_Pos) /*!< 0x00000002 */
#define RTC_CRH_ALRIE RTC_CRH_ALRIE_Msk                /*!< Alarm Interrupt Enable */
#define RTC_CRH_OWIE_Pos (2U)
#define RTC_CRH_OWIE_Msk (0x1UL << RTC_CRH_OWIE_Pos) /*!< 0x00000004 */
#define RTC_CRH_OWIE RTC_CRH_OWIE_Msk                /*!< OverfloW Interrupt Enable */

#define RTC_CRL_SECF_Pos (0U)
#define RTC_CRL_SECF_Msk (0x1UL << RTC_CRL_SECF_Pos) /*!< 0x00000001 */
#define RTC_CRL_SECF RTC_CRL_SECF_Msk                /*!< Second Flag */
#define RTC_CRL_ALRF_Pos (1U)
#define RTC_CRL_ALRF_Msk (0x1UL << RTC_CRL_ALRF_Pos) /*!< 0x00000002 */
#define RTC_CRL_ALRF RTC_CRL_ALRF_Msk                /*!< Alarm Flag */
#define RTC_CRL_OWF_Pos (2U)
#define RTC_CRL_OWF_Msk (0x1UL << RTC_CRL_OWF_Pos) /*!< 0x00000004 */
#define RTC_CRL_OWF RTC_CRL_OWF_Msk                /*!< OverfloW Flag */
#define RTC_CRL_RSF_Pos (3U)
#define RTC_CRL_RSF_Msk (0x1UL << RTC_CRL_RSF_Pos) /*!< 0x00000008 */
#define RTC_CRL_RSF RTC_CRL_RSF_Msk                /*!< Registers Synchronized Flag */
#define RTC_CRL_CNF_Pos (4U)
#define RTC_CRL_CNF_Msk (0x1UL << RTC_CRL_CNF_Pos) /*!< 0x00000010 */
#define RTC_CRL_CNF RTC_CRL_CNF_Msk                /*!< Configuration Flag */
#define RTC_CRL_RTOFF_Pos (5U)
#define RTC_CRL_RTOFF_Msk (0x1UL << RTC_CRL_RTOFF_Pos) /*!< 0x00000020 */
#define RTC_CRL_RTOFF RTC_CRL_RTOFF_Msk                /*!< RTC operation OFF */

#define RTC_PRLH_PRL_Pos (0U)
#define RTC_PRLH_PRL_Msk (0xFUL << RTC_PRLH_PRL_Pos) /*!< 0x0000000F */
#define RTC_PRLH_PRL RTC_PRLH_PRL_Msk                /*!< RTC Prescaler Reload Value High */

#define RTC_PRLL_PRL_Pos (0U)
#define RTC_PRLL_PRL_Msk (0xFFFFUL << RTC_PRLL_PRL_Pos) /*!< 0x0000FFFF */
#define RTC_PRLL_PRL RTC_PRLL_PRL_Msk                   /*!< RTC Prescaler Reload Value Low */

#define RTC_DIVH_RTC_DIV_Pos (0U)
#define RTC_DIVH_RTC_DIV_Msk (0xFUL << RTC_DIVH_RTC_DIV_Pos) /*!< 0x0000000F */
#define RTC_DIVH_RTC_DIV RTC_DIVH_RTC_DIV_Msk                /*!< RTC Clock Divider High */

#define RTC_DIVL_RTC_DIV_Pos (0U)
#define RTC_DIVL_RTC_DIV_Msk (0xFFFFUL << RTC_DIVL_RTC_DIV_Pos) /*!< 0x0000FFFF */
#define RTC_DIVL_RTC_DIV RTC_DIVL_RTC_DIV_Msk                   /*!< RTC Clock Divider Low */

#define RTC_CNTH_RTC_CNT_Pos (0U)
#define RTC_CNTH_RTC_CNT_Msk (0xFFFFUL << RTC_CNTH_RTC_CNT_Pos) /*!< 0x0000FFFF */
#define RTC_CNTH_RTC_CNT RTC_CNTH_RTC_CNT_Msk                   /*!< RTC Counter High */

#define RTC_CNTL_RTC_CNT_Pos (0U)
#define RTC_CNTL_RTC_CNT_Msk (0xFFFFUL << RTC_CNTL_RTC_CNT_Pos) /*!< 0x0000FFFF */
#define RTC_CNTL_RTC_CNT RTC_CNTL_RTC_CNT_Msk                   /*!< RTC Counter Low */

#define RTC_ALRH_RTC_ALR_Pos (0U)
#define RTC_ALRH_RTC_ALR_Msk (0xFFFFUL << RTC_ALRH_RTC_ALR_Pos) /*!< 0x0000FFFF */
#define RTC_ALRH_RTC_ALR RTC_ALRH_RTC_ALR_Msk                   /*!< RTC Alarm High */

#define RTC_ALRL_RTC_ALR_Pos (0U)
#define RTC_ALRL_RTC_ALR_Msk (0xFFFFUL << RTC_ALRL_RTC_ALR_Pos) /*!< 0x0000FFFF */
#define RTC_ALRL_RTC_ALR RTC_ALRL_RTC_ALR_Msk                   /*!< RTC Alarm Low */

/*                                                                            */
/*                                   USB Device FS                            */
/*                                                                            */

/*!< Endpoint-specific registers */
#define USB_EP0R USB_BASE                /*!< Endpoint 0 register address */
#define USB_EP1R (USB_BASE + 0x00000004) /*!< Endpoint 1 register address */
#define USB_EP2R (USB_BASE + 0x00000008) /*!< Endpoint 2 register address */
#define USB_EP3R (USB_BASE + 0x0000000C) /*!< Endpoint 3 register address */
#define USB_EP4R (USB_BASE + 0x00000010) /*!< Endpoint 4 register address */
#define USB_EP5R (USB_BASE + 0x00000014) /*!< Endpoint 5 register address */
#define USB_EP6R (USB_BASE + 0x00000018) /*!< Endpoint 6 register address */
#define USB_EP7R (USB_BASE + 0x0000001C) /*!< Endpoint 7 register address */

/* bit positions */

#define USB_EP_CTR_RX (0x1UL << (15U))   /*!< EndPoint Correct TRansfer RX */
#define USB_EP_DTOG_RX (0x1UL << (14U))  /*!< EndPoint Data TOGGLE RX */
#define USB_EPRX_STAT (0x3UL << (12U))   /*!< EndPoint RX STATus bit field */
#define USB_EP_SETUP (0x1UL << (11U))    /*!< EndPoint SETUP */
#define USB_EP_T_FIELD (0x3UL << (9U))   /*!< EndPoint TYPE */
#define USB_EP_KIND (0x1UL << (8U))      /*!< EndPoint KIND */
#define USB_EP_CTR_TX (0x1UL << (7U))    /*!< EndPoint Correct TRansfer TX */
#define USB_EP_DTOG_TX (0x1UL << (6U))   /*!< EndPoint Data TOGGLE TX */
#define USB_EPTX_STAT (0x3UL << (4U))    /*!< EndPoint TX STATus bit field */
#define USB_EPADDR_FIELD (0xFUL << (0U)) /*!< EndPoint ADDRess FIELD */

/* EndPoint REGister MASK (no toggle fields) */

#define USB_EPREG_MASK (USB_EP_CTR_RX | USB_EP_SETUP | USB_EP_T_FIELD | USB_EP_KIND | USB_EP_CTR_TX | USB_EPADDR_FIELD)

/*!< EP_TYPE[1:0] EndPoint TYPE */

#define USB_EP_TYPE_MASK (0x3UL << (9U)) /*!< EndPoint TYPE Mask */
#define USB_EP_BULK 0x00000000U          /*!< EndPoint BULK */
#define USB_EP_CONTROL 0x00000200U       /*!< EndPoint CONTROL */
#define USB_EP_ISOCHRONOUS 0x00000400U   /*!< EndPoint ISOCHRONOUS */
#define USB_EP_INTERRUPT 0x00000600U     /*!< EndPoint INTERRUPT */
#define USB_EP_T_MASK (~USB_EP_T_FIELD & USB_EPREG_MASK)
#define USB_EPKIND_MASK (~USB_EP_KIND & USB_EPREG_MASK) /*!< EP_KIND EndPoint KIND */

/*!< STAT_TX[1:0] STATus for TX transfer */

#define USB_EP_TX_DIS 0x00000000U   /*!< EndPoint TX DISabled */
#define USB_EP_TX_STALL 0x00000010U /*!< EndPoint TX STALLed */
#define USB_EP_TX_NAK 0x00000020U   /*!< EndPoint TX NAKed */
#define USB_EP_TX_VALID 0x00000030U /*!< EndPoint TX VALID */
#define USB_EPTX_DTOG1 0x00000010U  /*!< EndPoint TX Data TOGgle bit1 */
#define USB_EPTX_DTOG2 0x00000020U  /*!< EndPoint TX Data TOGgle bit2 */
#define USB_EPTX_DTOGMASK (USB_EPTX_STAT | USB_EPREG_MASK)

/*!< STAT_RX[1:0] STATus for RX transfer */

#define USB_EP_RX_DIS 0x00000000U   /*!< EndPoint RX DISabled */
#define USB_EP_RX_STALL 0x00001000U /*!< EndPoint RX STALLed */
#define USB_EP_RX_NAK 0x00002000U   /*!< EndPoint RX NAKed */
#define USB_EP_RX_VALID 0x00003000U /*!< EndPoint RX VALID */
#define USB_EPRX_DTOG1 0x00001000U  /*!< EndPoint RX Data TOGgle bit1 */
#define USB_EPRX_DTOG2 0x00002000U  /*!< EndPoint RX Data TOGgle bit1 */
#define USB_EPRX_DTOGMASK (USB_EPRX_STAT | USB_EPREG_MASK)
#define USB_EP0R_EA (0xFUL << (0U))                        /*!< Endpoint Address */
#define USB_EP0R_STAT_TX (0x3UL << (4U))                   /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define USB_EP0R_STAT_TX_0 (0x1UL << USB_EP0R_STAT_TX_Pos) /*!< 0x00000010 */
#define USB_EP0R_STAT_TX_1 (0x2UL << USB_EP0R_STAT_TX_Pos) /*!< 0x00000020 */
#define USB_EP0R_DTOG_TX (0x1UL << (6U))                   /*!< Data Toggle, for transmission transfers */
#define USB_EP0R_CTR_TX (0x1UL << (7U))                    /*!< Correct Transfer for transmission */
#define USB_EP0R_EP_KIND (0x1UL << (8U))                   /*!< Endpoint Kind */
#define USB_EP0R_EP_TYPE (0x3UL << (9U))                   /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define USB_EP0R_EP_TYPE_0 (0x1UL << USB_EP0R_EP_TYPE_Pos) /*!< 0x00000200 */
#define USB_EP0R_EP_TYPE_1 (0x2UL << USB_EP0R_EP_TYPE_Pos) /*!< 0x00000400 */
#define USB_EP0R_SETUP (0x1UL << (11U))                    /*!< Setup transaction completed */
#define USB_EP0R_STAT_RX (0x3UL << (12U))                  /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define USB_EP0R_STAT_RX_0 (0x1UL << USB_EP0R_STAT_RX_Pos) /*!< 0x00001000 */
#define USB_EP0R_STAT_RX_1 (0x2UL << USB_EP0R_STAT_RX_Pos) /*!< 0x00002000 */
#define USB_EP0R_DTOG_RX (0x1UL << (14U))                  /*!< Data Toggle, for reception transfers */
#define USB_EP0R_CTR_RX (0x1UL << (15U))                   /*!< Correct Transfer for reception */
#define USB_EP1R_EA (0xFUL << (0U))                        /*!< Endpoint Address */
#define USB_EP1R_STAT_TX (0x3UL << (4U))                   /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define USB_EP1R_STAT_TX_0 (0x1UL << USB_EP1R_STAT_TX_Pos) /*!< 0x00000010 */
#define USB_EP1R_STAT_TX_1 (0x2UL << USB_EP1R_STAT_TX_Pos) /*!< 0x00000020 */
#define USB_EP1R_DTOG_TX (0x1UL << (6U))                   /*!< Data Toggle, for transmission transfers */
#define USB_EP1R_CTR_TX (0x1UL << (7U))                    /*!< Correct Transfer for transmission */
#define USB_EP1R_EP_KIND (0x1UL << (8U))                   /*!< Endpoint Kind */
#define USB_EP1R_EP_TYPE (0x3UL << (9U))                   /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define USB_EP1R_EP_TYPE_0 (0x1UL << USB_EP1R_EP_TYPE_Pos) /*!< 0x00000200 */
#define USB_EP1R_EP_TYPE_1 (0x2UL << USB_EP1R_EP_TYPE_Pos) /*!< 0x00000400 */
#define USB_EP1R_SETUP (0x1UL << (11U))                    /*!< Setup transaction completed */
#define USB_EP1R_STAT_RX (0x3UL << (12U))                  /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define USB_EP1R_STAT_RX_0 (0x1UL << USB_EP1R_STAT_RX_Pos) /*!< 0x00001000 */
#define USB_EP1R_STAT_RX_1 (0x2UL << USB_EP1R_STAT_RX_Pos) /*!< 0x00002000 */
#define USB_EP1R_DTOG_RX (0x1UL << (14U))                  /*!< Data Toggle, for reception transfers */
#define USB_EP1R_CTR_RX (0x1UL << (15U))                   /*!< Correct Transfer for reception */
#define USB_EP2R_EA (0xFUL << (0U))                        /*!< Endpoint Address */
#define USB_EP2R_STAT_TX (0x3UL << (4U))                   /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define USB_EP2R_STAT_TX_0 (0x1UL << USB_EP2R_STAT_TX_Pos) /*!< 0x00000010 */
#define USB_EP2R_STAT_TX_1 (0x2UL << USB_EP2R_STAT_TX_Pos) /*!< 0x00000020 */
#define USB_EP2R_DTOG_TX (0x1UL << (6U))                   /*!< Data Toggle, for transmission transfers */
#define USB_EP2R_CTR_TX (0x1UL << (7U))                    /*!< Correct Transfer for transmission */
#define USB_EP2R_EP_KIND (0x1UL << (8U))                   /*!< Endpoint Kind */
#define USB_EP2R_EP_TYPE (0x3UL << (9U))                   /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define USB_EP2R_EP_TYPE_0 (0x1UL << USB_EP2R_EP_TYPE_Pos) /*!< 0x00000200 */
#define USB_EP2R_EP_TYPE_1 (0x2UL << USB_EP2R_EP_TYPE_Pos) /*!< 0x00000400 */
#define USB_EP2R_SETUP (0x1UL << (11U))                    /*!< Setup transaction completed */
#define USB_EP2R_STAT_RX (0x3UL << (12U))                  /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define USB_EP2R_STAT_RX_0 (0x1UL << USB_EP2R_STAT_RX_Pos) /*!< 0x00001000 */
#define USB_EP2R_STAT_RX_1 (0x2UL << USB_EP2R_STAT_RX_Pos) /*!< 0x00002000 */
#define USB_EP2R_DTOG_RX (0x1UL << (14U))                  /*!< Data Toggle, for reception transfers */
#define USB_EP2R_CTR_RX (0x1UL << (15U))                   /*!< Correct Transfer for reception */
#define USB_EP3R_EA (0xFUL << (0U))                        /*!< Endpoint Address */
#define USB_EP3R_STAT_TX (0x3UL << (4U))                   /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define USB_EP3R_STAT_TX_0 (0x1UL << USB_EP3R_STAT_TX_Pos) /*!< 0x00000010 */
#define USB_EP3R_STAT_TX_1 (0x2UL << USB_EP3R_STAT_TX_Pos) /*!< 0x00000020 */
#define USB_EP3R_DTOG_TX (0x1UL << (6U))                   /*!< Data Toggle, for transmission transfers */
#define USB_EP3R_CTR_TX (0x1UL << (7U))                    /*!< Correct Transfer for transmission */
#define USB_EP3R_EP_KIND (0x1UL << (8U))                   /*!< Endpoint Kind */
#define USB_EP3R_EP_TYPE (0x3UL << (9U))                   /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define USB_EP3R_EP_TYPE_0 (0x1UL << USB_EP3R_EP_TYPE_Pos) /*!< 0x00000200 */
#define USB_EP3R_EP_TYPE_1 (0x2UL << USB_EP3R_EP_TYPE_Pos) /*!< 0x00000400 */
#define USB_EP3R_SETUP (0x1UL << (11U))                    /*!< Setup transaction completed */
#define USB_EP3R_STAT_RX (0x3UL << (12U))                  /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define USB_EP3R_STAT_RX_0 (0x1UL << USB_EP3R_STAT_RX_Pos) /*!< 0x00001000 */
#define USB_EP3R_STAT_RX_1 (0x2UL << USB_EP3R_STAT_RX_Pos) /*!< 0x00002000 */
#define USB_EP3R_DTOG_RX (0x1UL << (14U))                  /*!< Data Toggle, for reception transfers */
#define USB_EP3R_CTR_RX (0x1UL << (15U))                   /*!< Correct Transfer for reception */
#define USB_EP4R_EA (0xFUL << (0U))                        /*!< Endpoint Address */
#define USB_EP4R_STAT_TX (0x3UL << (4U))                   /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define USB_EP4R_STAT_TX_0 (0x1UL << USB_EP4R_STAT_TX_Pos) /*!< 0x00000010 */
#define USB_EP4R_STAT_TX_1 (0x2UL << USB_EP4R_STAT_TX_Pos) /*!< 0x00000020 */
#define USB_EP4R_DTOG_TX (0x1UL << (6U))                   /*!< Data Toggle, for transmission transfers */
#define USB_EP4R_CTR_TX (0x1UL << (7U))                    /*!< Correct Transfer for transmission */
#define USB_EP4R_EP_KIND (0x1UL << (8U))                   /*!< Endpoint Kind */
#define USB_EP4R_EP_TYPE (0x3UL << (9U))                   /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define USB_EP4R_EP_TYPE_0 (0x1UL << USB_EP4R_EP_TYPE_Pos) /*!< 0x00000200 */
#define USB_EP4R_EP_TYPE_1 (0x2UL << USB_EP4R_EP_TYPE_Pos) /*!< 0x00000400 */
#define USB_EP4R_SETUP (0x1UL << (11U))                    /*!< Setup transaction completed */
#define USB_EP4R_STAT_RX (0x3UL << (12U))                  /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define USB_EP4R_STAT_RX_0 (0x1UL << USB_EP4R_STAT_RX_Pos) /*!< 0x00001000 */
#define USB_EP4R_STAT_RX_1 (0x2UL << USB_EP4R_STAT_RX_Pos) /*!< 0x00002000 */
#define USB_EP4R_DTOG_RX (0x1UL << (14U))                  /*!< Data Toggle, for reception transfers */
#define USB_EP4R_CTR_RX (0x1UL << (15U))                   /*!< Correct Transfer for reception */
#define USB_EP5R_EA (0xFUL << (0U))                        /*!< Endpoint Address */
#define USB_EP5R_STAT_TX (0x3UL << (4U))                   /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define USB_EP5R_STAT_TX_0 (0x1UL << USB_EP5R_STAT_TX_Pos) /*!< 0x00000010 */
#define USB_EP5R_STAT_TX_1 (0x2UL << USB_EP5R_STAT_TX_Pos) /*!< 0x00000020 */
#define USB_EP5R_DTOG_TX (0x1UL << (6U))                   /*!< Data Toggle, for transmission transfers */
#define USB_EP5R_CTR_TX (0x1UL << (7U))                    /*!< Correct Transfer for transmission */
#define USB_EP5R_EP_KIND (0x1UL << (8U))                   /*!< Endpoint Kind */
#define USB_EP5R_EP_TYPE (0x3UL << (9U))                   /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define USB_EP5R_EP_TYPE_0 (0x1UL << USB_EP5R_EP_TYPE_Pos) /*!< 0x00000200 */
#define USB_EP5R_EP_TYPE_1 (0x2UL << USB_EP5R_EP_TYPE_Pos) /*!< 0x00000400 */
#define USB_EP5R_SETUP (0x1UL << (11U))                    /*!< Setup transaction completed */
#define USB_EP5R_STAT_RX (0x3UL << (12U))                  /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define USB_EP5R_STAT_RX_0 (0x1UL << USB_EP5R_STAT_RX_Pos) /*!< 0x00001000 */
#define USB_EP5R_STAT_RX_1 (0x2UL << USB_EP5R_STAT_RX_Pos) /*!< 0x00002000 */
#define USB_EP5R_DTOG_RX (0x1UL << (14U))                  /*!< Data Toggle, for reception transfers */
#define USB_EP5R_CTR_RX (0x1UL << (15U))                   /*!< Correct Transfer for reception */
#define USB_EP6R_EA (0xFUL << (0U))                        /*!< Endpoint Address */
#define USB_EP6R_STAT_TX (0x3UL << (4U))                   /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define USB_EP6R_STAT_TX_0 (0x1UL << USB_EP6R_STAT_TX_Pos) /*!< 0x00000010 */
#define USB_EP6R_STAT_TX_1 (0x2UL << USB_EP6R_STAT_TX_Pos) /*!< 0x00000020 */
#define USB_EP6R_DTOG_TX (0x1UL << (6U))                   /*!< Data Toggle, for transmission transfers */
#define USB_EP6R_CTR_TX (0x1UL << (7U))                    /*!< Correct Transfer for transmission */
#define USB_EP6R_EP_KIND (0x1UL << (8U))                   /*!< Endpoint Kind */
#define USB_EP6R_EP_TYPE (0x3UL << (9U))                   /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define USB_EP6R_EP_TYPE_0 (0x1UL << USB_EP6R_EP_TYPE_Pos) /*!< 0x00000200 */
#define USB_EP6R_EP_TYPE_1 (0x2UL << USB_EP6R_EP_TYPE_Pos) /*!< 0x00000400 */
#define USB_EP6R_SETUP (0x1UL << (11U))                    /*!< Setup transaction completed */
#define USB_EP6R_STAT_RX (0x3UL << (12U))                  /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define USB_EP6R_STAT_RX_0 (0x1UL << USB_EP6R_STAT_RX_Pos) /*!< 0x00001000 */
#define USB_EP6R_STAT_RX_1 (0x2UL << USB_EP6R_STAT_RX_Pos) /*!< 0x00002000 */
#define USB_EP6R_DTOG_RX (0x1UL << (14U))                  /*!< Data Toggle, for reception transfers */
#define USB_EP6R_CTR_RX (0x1UL << (15U))                   /*!< Correct Transfer for reception */
#define USB_EP7R_EA (0xFUL << (0U))                        /*!< Endpoint Address */
#define USB_EP7R_STAT_TX (0x3UL << (4U))                   /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define USB_EP7R_STAT_TX_0 (0x1UL << USB_EP7R_STAT_TX_Pos) /*!< 0x00000010 */
#define USB_EP7R_STAT_TX_1 (0x2UL << USB_EP7R_STAT_TX_Pos) /*!< 0x00000020 */
#define USB_EP7R_DTOG_TX (0x1UL << (6U))                   /*!< Data Toggle, for transmission transfers */
#define USB_EP7R_CTR_TX (0x1UL << (7U))                    /*!< Correct Transfer for transmission */
#define USB_EP7R_EP_KIND (0x1UL << (8U))                   /*!< Endpoint Kind */
#define USB_EP7R_EP_TYPE (0x3UL << (9U))                   /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define USB_EP7R_EP_TYPE_0 (0x1UL << USB_EP7R_EP_TYPE_Pos) /*!< 0x00000200 */
#define USB_EP7R_EP_TYPE_1 (0x2UL << USB_EP7R_EP_TYPE_Pos) /*!< 0x00000400 */
#define USB_EP7R_SETUP (0x1UL << (11U))                    /*!< Setup transaction completed */
#define USB_EP7R_STAT_RX (0x3UL << (12U))                  /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define USB_EP7R_STAT_RX_0 (0x1UL << USB_EP7R_STAT_RX_Pos) /*!< 0x00001000 */
#define USB_EP7R_STAT_RX_1 (0x2UL << USB_EP7R_STAT_RX_Pos) /*!< 0x00002000 */
#define USB_EP7R_DTOG_RX (0x1UL << (14U))                  /*!< Data Toggle, for reception transfers */
#define USB_EP7R_CTR_RX (0x1UL << (15U))                   /*!< Correct Transfer for reception */

/*!< Common registers */

#define USB_CNTR_FRES (0x1UL << (0U))        /*!< Force USB Reset */
#define USB_CNTR_PDWN (0x1UL << (1U))        /*!< Power down */
#define USB_CNTR_LP_MODE (0x1UL << (2U))     /*!< Low-power mode */
#define USB_CNTR_FSUSP (0x1UL << (3U))       /*!< Force suspend */
#define USB_CNTR_RESUME (0x1UL << (4U))      /*!< Resume request */
#define USB_CNTR_ESOFM (0x1UL << (8U))       /*!< Expected Start Of Frame Interrupt Mask */
#define USB_CNTR_SOFM (0x1UL << (9U))        /*!< Start Of Frame Interrupt Mask */
#define USB_CNTR_RESETM (0x1UL << (10U))     /*!< RESET Interrupt Mask */
#define USB_CNTR_SUSPM (0x1UL << (11U))      /*!< Suspend mode Interrupt Mask */
#define USB_CNTR_WKUPM (0x1UL << (12U))      /*!< Wakeup Interrupt Mask */
#define USB_CNTR_ERRM (0x1UL << (13U))       /*!< Error Interrupt Mask */
#define USB_CNTR_PMAOVRM (0x1UL << (14U))    /*!< Packet Memory Area Over / Underrun Interrupt Mask */
#define USB_CNTR_CTRM (0x1UL << (15U))       /*!< Correct Transfer Interrupt Mask */
#define USB_ISTR_EP_ID (0xFUL << (0U))       /*!< Endpoint Identifier */
#define USB_ISTR_DIR (0x1UL << (4U))         /*!< Direction of transaction */
#define USB_ISTR_ESOF (0x1UL << (8U))        /*!< Expected Start Of Frame */
#define USB_ISTR_SOF (0x1UL << (9U))         /*!< Start Of Frame */
#define USB_ISTR_RESET (0x1UL << (10U))      /*!< USB RESET request */
#define USB_ISTR_SUSP (0x1UL << (11U))       /*!< Suspend mode request */
#define USB_ISTR_WKUP (0x1UL << (12U))       /*!< Wake up */
#define USB_ISTR_ERR (0x1UL << (13U))        /*!< Error */
#define USB_ISTR_PMAOVR (0x1UL << (14U))     /*!< Packet Memory Area Over / Underrun */
#define USB_ISTR_CTR (0x1UL << (15U))        /*!< Correct Transfer */
#define USB_FNR_FN (0x7FFUL << (0U))         /*!< Frame Number */
#define USB_FNR_LSOF (0x3UL << (11U))        /*!< Lost SOF */
#define USB_FNR_LCK (0x1UL << (13U))         /*!< Locked */
#define USB_FNR_RXDM (0x1UL << (14U))        /*!< Receive Data - Line Status */
#define USB_FNR_RXDP (0x1UL << (15U))        /*!< Receive Data + Line Status */
#define USB_DADDR_ADD (0x7FUL << (0U))       /*!< ADD[6:0] bits (Device Address) */
#define USB_DADDR_ADD0 (0x1UL << (0U))       /*!< Bit 0 */
#define USB_DADDR_ADD1 (0x1UL << (1U))       /*!< Bit 1 */
#define USB_DADDR_ADD2 (0x1UL << (2U))       /*!< Bit 2 */
#define USB_DADDR_ADD3 (0x1UL << (3U))       /*!< Bit 3 */
#define USB_DADDR_ADD4 (0x1UL << (4U))       /*!< Bit 4 */
#define USB_DADDR_ADD5 (0x1UL << (5U))       /*!< Bit 5 */
#define USB_DADDR_ADD6 (0x1UL << (6U))       /*!< Bit 6 */
#define USB_DADDR_EF (0x1UL << (7U))         /*!< Enable Function */
#define USB_BTABLE_BTABLE (0x1FFFUL << (3U)) /*!< Buffer Table */

/*!< Buffer descriptor table */

#define USB_ADDR0_TX_ADDR0_TX (0x7FFFUL << (1U)) /*!< Transmission Buffer Address 0 */
#define USB_ADDR1_TX_ADDR1_TX (0x7FFFUL << (1U)) /*!< Transmission Buffer Address 1 */
#define USB_ADDR2_TX_ADDR2_TX (0x7FFFUL << (1U)) /*!< Transmission Buffer Address 2 */
#define USB_ADDR3_TX_ADDR3_TX (0x7FFFUL << (1U)) /*!< Transmission Buffer Address 3 */
#define USB_ADDR4_TX_ADDR4_TX (0x7FFFUL << (1U)) /*!< Transmission Buffer Address 4 */
#define USB_ADDR5_TX_ADDR5_TX (0x7FFFUL << (1U)) /*!< Transmission Buffer Address 5 */
#define USB_ADDR6_TX_ADDR6_TX (0x7FFFUL << (1U)) /*!< Transmission Buffer Address 6 */
#define USB_ADDR7_TX_ADDR7_TX (0x7FFFUL << (1U)) /*!< Transmission Buffer Address 7 */

/*----------------------------------------------------------------------------*/

#define USB_COUNT0_TX_COUNT0_TX (0x3FFUL << (0U)) /*!< Transmission Byte Count 0 */
#define USB_COUNT1_TX_COUNT1_TX (0x3FFUL << (0U)) /*!< Transmission Byte Count 1 */
#define USB_COUNT2_TX_COUNT2_TX (0x3FFUL << (0U)) /*!< Transmission Byte Count 2 */
#define USB_COUNT3_TX_COUNT3_TX (0x3FFUL << (0U)) /*!< Transmission Byte Count 3 */
#define USB_COUNT4_TX_COUNT4_TX (0x3FFUL << (0U)) /*!< Transmission Byte Count 4 */
#define USB_COUNT5_TX_COUNT5_TX (0x3FFUL << (0U)) /*!< Transmission Byte Count 5 */
#define USB_COUNT6_TX_COUNT6_TX (0x3FFUL << (0U)) /*!< Transmission Byte Count 6 */
#define USB_COUNT7_TX_COUNT7_TX (0x3FFUL << (0U)) /*!< Transmission Byte Count 7 */

/*----------------------------------------------------------------------------*/

#define USB_COUNT0_TX_0_COUNT0_TX_0 0x000003FFU /*!< Transmission Byte Count 0 (low) */
#define USB_COUNT0_TX_1_COUNT0_TX_1 0x03FF0000U /*!< Transmission Byte Count 0 (high) */
#define USB_COUNT1_TX_0_COUNT1_TX_0 0x000003FFU /*!< Transmission Byte Count 1 (low) */
#define USB_COUNT1_TX_1_COUNT1_TX_1 0x03FF0000U /*!< Transmission Byte Count 1 (high) */
#define USB_COUNT2_TX_0_COUNT2_TX_0 0x000003FFU /*!< Transmission Byte Count 2 (low) */
#define USB_COUNT2_TX_1_COUNT2_TX_1 0x03FF0000U /*!< Transmission Byte Count 2 (high) */
#define USB_COUNT3_TX_0_COUNT3_TX_0 0x000003FFU /*!< Transmission Byte Count 3 (low) */
#define USB_COUNT3_TX_1_COUNT3_TX_1 0x03FF0000U /*!< Transmission Byte Count 3 (high) */
#define USB_COUNT4_TX_0_COUNT4_TX_0 0x000003FFU /*!< Transmission Byte Count 4 (low) */
#define USB_COUNT4_TX_1_COUNT4_TX_1 0x03FF0000U /*!< Transmission Byte Count 4 (high) */
#define USB_COUNT5_TX_0_COUNT5_TX_0 0x000003FFU /*!< Transmission Byte Count 5 (low) */
#define USB_COUNT5_TX_1_COUNT5_TX_1 0x03FF0000U /*!< Transmission Byte Count 5 (high) */
#define USB_COUNT6_TX_0_COUNT6_TX_0 0x000003FFU /*!< Transmission Byte Count 6 (low) */
#define USB_COUNT6_TX_1_COUNT6_TX_1 0x03FF0000U /*!< Transmission Byte Count 6 (high) */
#define USB_COUNT7_TX_0_COUNT7_TX_0 0x000003FFU /*!< Transmission Byte Count 7 (low) */
#define USB_COUNT7_TX_1_COUNT7_TX_1 0x03FF0000U /*!< Transmission Byte Count 7 (high) */
/*----------------------------------------------------------------------------*/

#define USB_ADDR0_RX_ADDR0_RX_Pos (1U)
#define USB_ADDR0_RX_ADDR0_RX_Msk (0x7FFFUL << USB_ADDR0_RX_ADDR0_RX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR0_RX_ADDR0_RX USB_ADDR0_RX_ADDR0_RX_Msk                   /*!< Reception Buffer Address 0 */

#define USB_ADDR1_RX_ADDR1_RX_Pos (1U)
#define USB_ADDR1_RX_ADDR1_RX_Msk (0x7FFFUL << USB_ADDR1_RX_ADDR1_RX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR1_RX_ADDR1_RX USB_ADDR1_RX_ADDR1_RX_Msk                   /*!< Reception Buffer Address 1 */

#define USB_ADDR2_RX_ADDR2_RX_Pos (1U)
#define USB_ADDR2_RX_ADDR2_RX_Msk (0x7FFFUL << USB_ADDR2_RX_ADDR2_RX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR2_RX_ADDR2_RX USB_ADDR2_RX_ADDR2_RX_Msk                   /*!< Reception Buffer Address 2 */

#define USB_ADDR3_RX_ADDR3_RX_Pos (1U)
#define USB_ADDR3_RX_ADDR3_RX_Msk (0x7FFFUL << USB_ADDR3_RX_ADDR3_RX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR3_RX_ADDR3_RX USB_ADDR3_RX_ADDR3_RX_Msk                   /*!< Reception Buffer Address 3 */

#define USB_ADDR4_RX_ADDR4_RX_Pos (1U)
#define USB_ADDR4_RX_ADDR4_RX_Msk (0x7FFFUL << USB_ADDR4_RX_ADDR4_RX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR4_RX_ADDR4_RX USB_ADDR4_RX_ADDR4_RX_Msk                   /*!< Reception Buffer Address 4 */

#define USB_ADDR5_RX_ADDR5_RX_Pos (1U)
#define USB_ADDR5_RX_ADDR5_RX_Msk (0x7FFFUL << USB_ADDR5_RX_ADDR5_RX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR5_RX_ADDR5_RX USB_ADDR5_RX_ADDR5_RX_Msk                   /*!< Reception Buffer Address 5 */

#define USB_ADDR6_RX_ADDR6_RX_Pos (1U)
#define USB_ADDR6_RX_ADDR6_RX_Msk (0x7FFFUL << USB_ADDR6_RX_ADDR6_RX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR6_RX_ADDR6_RX USB_ADDR6_RX_ADDR6_RX_Msk                   /*!< Reception Buffer Address 6 */

#define USB_ADDR7_RX_ADDR7_RX_Pos (1U)
#define USB_ADDR7_RX_ADDR7_RX_Msk (0x7FFFUL << USB_ADDR7_RX_ADDR7_RX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR7_RX_ADDR7_RX USB_ADDR7_RX_ADDR7_RX_Msk                   /*!< Reception Buffer Address 7 */

/*----------------------------------------------------------------------------*/

#define USB_COUNT0_RX_COUNT0_RX_Pos (0U)
#define USB_COUNT0_RX_COUNT0_RX_Msk (0x3FFUL << USB_COUNT0_RX_COUNT0_RX_Pos) /*!< 0x000003FF */
#define USB_COUNT0_RX_COUNT0_RX USB_COUNT0_RX_COUNT0_RX_Msk                  /*!< Reception Byte Count */

#define USB_COUNT0_RX_NUM_BLOCK_Pos (10U)
#define USB_COUNT0_RX_NUM_BLOCK_Msk (0x1FUL << USB_COUNT0_RX_NUM_BLOCK_Pos) /*!< 0x00007C00 */
#define USB_COUNT0_RX_NUM_BLOCK USB_COUNT0_RX_NUM_BLOCK_Msk                 /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT0_RX_NUM_BLOCK_0 (0x01UL << USB_COUNT0_RX_NUM_BLOCK_Pos)   /*!< 0x00000400 */
#define USB_COUNT0_RX_NUM_BLOCK_1 (0x02UL << USB_COUNT0_RX_NUM_BLOCK_Pos)   /*!< 0x00000800 */
#define USB_COUNT0_RX_NUM_BLOCK_2 (0x04UL << USB_COUNT0_RX_NUM_BLOCK_Pos)   /*!< 0x00001000 */
#define USB_COUNT0_RX_NUM_BLOCK_3 (0x08UL << USB_COUNT0_RX_NUM_BLOCK_Pos)   /*!< 0x00002000 */
#define USB_COUNT0_RX_NUM_BLOCK_4 (0x10UL << USB_COUNT0_RX_NUM_BLOCK_Pos)   /*!< 0x00004000 */

#define USB_COUNT0_RX_BLSIZE_Pos (15U)
#define USB_COUNT0_RX_BLSIZE_Msk (0x1UL << USB_COUNT0_RX_BLSIZE_Pos) /*!< 0x00008000 */
#define USB_COUNT0_RX_BLSIZE USB_COUNT0_RX_BLSIZE_Msk                /*!< BLock SIZE */

#define USB_COUNT1_RX_COUNT1_RX_Pos (0U)
#define USB_COUNT1_RX_COUNT1_RX_Msk (0x3FFUL << USB_COUNT1_RX_COUNT1_RX_Pos) /*!< 0x000003FF */
#define USB_COUNT1_RX_COUNT1_RX USB_COUNT1_RX_COUNT1_RX_Msk                  /*!< Reception Byte Count */

#define USB_COUNT1_RX_NUM_BLOCK_Pos (10U)
#define USB_COUNT1_RX_NUM_BLOCK_Msk (0x1FUL << USB_COUNT1_RX_NUM_BLOCK_Pos) /*!< 0x00007C00 */
#define USB_COUNT1_RX_NUM_BLOCK USB_COUNT1_RX_NUM_BLOCK_Msk                 /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT1_RX_NUM_BLOCK_0 (0x01UL << USB_COUNT1_RX_NUM_BLOCK_Pos)   /*!< 0x00000400 */
#define USB_COUNT1_RX_NUM_BLOCK_1 (0x02UL << USB_COUNT1_RX_NUM_BLOCK_Pos)   /*!< 0x00000800 */
#define USB_COUNT1_RX_NUM_BLOCK_2 (0x04UL << USB_COUNT1_RX_NUM_BLOCK_Pos)   /*!< 0x00001000 */
#define USB_COUNT1_RX_NUM_BLOCK_3 (0x08UL << USB_COUNT1_RX_NUM_BLOCK_Pos)   /*!< 0x00002000 */
#define USB_COUNT1_RX_NUM_BLOCK_4 (0x10UL << USB_COUNT1_RX_NUM_BLOCK_Pos)   /*!< 0x00004000 */

#define USB_COUNT1_RX_BLSIZE_Pos (15U)
#define USB_COUNT1_RX_BLSIZE_Msk (0x1UL << USB_COUNT1_RX_BLSIZE_Pos) /*!< 0x00008000 */
#define USB_COUNT1_RX_BLSIZE USB_COUNT1_RX_BLSIZE_Msk                /*!< BLock SIZE */

#define USB_COUNT2_RX_COUNT2_RX_Pos (0U)
#define USB_COUNT2_RX_COUNT2_RX_Msk (0x3FFUL << USB_COUNT2_RX_COUNT2_RX_Pos) /*!< 0x000003FF */
#define USB_COUNT2_RX_COUNT2_RX USB_COUNT2_RX_COUNT2_RX_Msk                  /*!< Reception Byte Count */

#define USB_COUNT2_RX_NUM_BLOCK_Pos (10U)
#define USB_COUNT2_RX_NUM_BLOCK_Msk (0x1FUL << USB_COUNT2_RX_NUM_BLOCK_Pos) /*!< 0x00007C00 */
#define USB_COUNT2_RX_NUM_BLOCK USB_COUNT2_RX_NUM_BLOCK_Msk                 /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT2_RX_NUM_BLOCK_0 (0x01UL << USB_COUNT2_RX_NUM_BLOCK_Pos)   /*!< 0x00000400 */
#define USB_COUNT2_RX_NUM_BLOCK_1 (0x02UL << USB_COUNT2_RX_NUM_BLOCK_Pos)   /*!< 0x00000800 */
#define USB_COUNT2_RX_NUM_BLOCK_2 (0x04UL << USB_COUNT2_RX_NUM_BLOCK_Pos)   /*!< 0x00001000 */
#define USB_COUNT2_RX_NUM_BLOCK_3 (0x08UL << USB_COUNT2_RX_NUM_BLOCK_Pos)   /*!< 0x00002000 */
#define USB_COUNT2_RX_NUM_BLOCK_4 (0x10UL << USB_COUNT2_RX_NUM_BLOCK_Pos)   /*!< 0x00004000 */

#define USB_COUNT2_RX_BLSIZE_Pos (15U)
#define USB_COUNT2_RX_BLSIZE_Msk (0x1UL << USB_COUNT2_RX_BLSIZE_Pos) /*!< 0x00008000 */
#define USB_COUNT2_RX_BLSIZE USB_COUNT2_RX_BLSIZE_Msk                /*!< BLock SIZE */

#define USB_COUNT3_RX_COUNT3_RX_Pos (0U)
#define USB_COUNT3_RX_COUNT3_RX_Msk (0x3FFUL << USB_COUNT3_RX_COUNT3_RX_Pos) /*!< 0x000003FF */
#define USB_COUNT3_RX_COUNT3_RX USB_COUNT3_RX_COUNT3_RX_Msk                  /*!< Reception Byte Count */

#define USB_COUNT3_RX_NUM_BLOCK_Pos (10U)
#define USB_COUNT3_RX_NUM_BLOCK_Msk (0x1FUL << USB_COUNT3_RX_NUM_BLOCK_Pos) /*!< 0x00007C00 */
#define USB_COUNT3_RX_NUM_BLOCK USB_COUNT3_RX_NUM_BLOCK_Msk                 /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT3_RX_NUM_BLOCK_0 (0x01UL << USB_COUNT3_RX_NUM_BLOCK_Pos)   /*!< 0x00000400 */
#define USB_COUNT3_RX_NUM_BLOCK_1 (0x02UL << USB_COUNT3_RX_NUM_BLOCK_Pos)   /*!< 0x00000800 */
#define USB_COUNT3_RX_NUM_BLOCK_2 (0x04UL << USB_COUNT3_RX_NUM_BLOCK_Pos)   /*!< 0x00001000 */
#define USB_COUNT3_RX_NUM_BLOCK_3 (0x08UL << USB_COUNT3_RX_NUM_BLOCK_Pos)   /*!< 0x00002000 */
#define USB_COUNT3_RX_NUM_BLOCK_4 (0x10UL << USB_COUNT3_RX_NUM_BLOCK_Pos)   /*!< 0x00004000 */

#define USB_COUNT3_RX_BLSIZE_Pos (15U)
#define USB_COUNT3_RX_BLSIZE_Msk (0x1UL << USB_COUNT3_RX_BLSIZE_Pos) /*!< 0x00008000 */
#define USB_COUNT3_RX_BLSIZE USB_COUNT3_RX_BLSIZE_Msk                /*!< BLock SIZE */

#define USB_COUNT4_RX_COUNT4_RX_Pos (0U)
#define USB_COUNT4_RX_COUNT4_RX_Msk (0x3FFUL << USB_COUNT4_RX_COUNT4_RX_Pos) /*!< 0x000003FF */
#define USB_COUNT4_RX_COUNT4_RX USB_COUNT4_RX_COUNT4_RX_Msk                  /*!< Reception Byte Count */

#define USB_COUNT4_RX_NUM_BLOCK_Pos (10U)
#define USB_COUNT4_RX_NUM_BLOCK_Msk (0x1FUL << USB_COUNT4_RX_NUM_BLOCK_Pos) /*!< 0x00007C00 */
#define USB_COUNT4_RX_NUM_BLOCK USB_COUNT4_RX_NUM_BLOCK_Msk                 /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT4_RX_NUM_BLOCK_0 (0x01UL << USB_COUNT4_RX_NUM_BLOCK_Pos)   /*!< 0x00000400 */
#define USB_COUNT4_RX_NUM_BLOCK_1 (0x02UL << USB_COUNT4_RX_NUM_BLOCK_Pos)   /*!< 0x00000800 */
#define USB_COUNT4_RX_NUM_BLOCK_2 (0x04UL << USB_COUNT4_RX_NUM_BLOCK_Pos)   /*!< 0x00001000 */
#define USB_COUNT4_RX_NUM_BLOCK_3 (0x08UL << USB_COUNT4_RX_NUM_BLOCK_Pos)   /*!< 0x00002000 */
#define USB_COUNT4_RX_NUM_BLOCK_4 (0x10UL << USB_COUNT4_RX_NUM_BLOCK_Pos)   /*!< 0x00004000 */

#define USB_COUNT4_RX_BLSIZE_Pos (15U)
#define USB_COUNT4_RX_BLSIZE_Msk (0x1UL << USB_COUNT4_RX_BLSIZE_Pos) /*!< 0x00008000 */
#define USB_COUNT4_RX_BLSIZE USB_COUNT4_RX_BLSIZE_Msk                /*!< BLock SIZE */

#define USB_COUNT5_RX_COUNT5_RX_Pos (0U)
#define USB_COUNT5_RX_COUNT5_RX_Msk (0x3FFUL << USB_COUNT5_RX_COUNT5_RX_Pos) /*!< 0x000003FF */
#define USB_COUNT5_RX_COUNT5_RX USB_COUNT5_RX_COUNT5_RX_Msk                  /*!< Reception Byte Count */

#define USB_COUNT5_RX_NUM_BLOCK_Pos (10U)
#define USB_COUNT5_RX_NUM_BLOCK_Msk (0x1FUL << USB_COUNT5_RX_NUM_BLOCK_Pos) /*!< 0x00007C00 */
#define USB_COUNT5_RX_NUM_BLOCK USB_COUNT5_RX_NUM_BLOCK_Msk                 /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT5_RX_NUM_BLOCK_0 (0x01UL << USB_COUNT5_RX_NUM_BLOCK_Pos)   /*!< 0x00000400 */
#define USB_COUNT5_RX_NUM_BLOCK_1 (0x02UL << USB_COUNT5_RX_NUM_BLOCK_Pos)   /*!< 0x00000800 */
#define USB_COUNT5_RX_NUM_BLOCK_2 (0x04UL << USB_COUNT5_RX_NUM_BLOCK_Pos)   /*!< 0x00001000 */
#define USB_COUNT5_RX_NUM_BLOCK_3 (0x08UL << USB_COUNT5_RX_NUM_BLOCK_Pos)   /*!< 0x00002000 */
#define USB_COUNT5_RX_NUM_BLOCK_4 (0x10UL << USB_COUNT5_RX_NUM_BLOCK_Pos)   /*!< 0x00004000 */

#define USB_COUNT5_RX_BLSIZE_Pos (15U)
#define USB_COUNT5_RX_BLSIZE_Msk (0x1UL << USB_COUNT5_RX_BLSIZE_Pos) /*!< 0x00008000 */
#define USB_COUNT5_RX_BLSIZE USB_COUNT5_RX_BLSIZE_Msk                /*!< BLock SIZE */

#define USB_COUNT6_RX_COUNT6_RX_Pos (0U)
#define USB_COUNT6_RX_COUNT6_RX_Msk (0x3FFUL << USB_COUNT6_RX_COUNT6_RX_Pos) /*!< 0x000003FF */
#define USB_COUNT6_RX_COUNT6_RX USB_COUNT6_RX_COUNT6_RX_Msk                  /*!< Reception Byte Count */

#define USB_COUNT6_RX_NUM_BLOCK_Pos (10U)
#define USB_COUNT6_RX_NUM_BLOCK_Msk (0x1FUL << USB_COUNT6_RX_NUM_BLOCK_Pos) /*!< 0x00007C00 */
#define USB_COUNT6_RX_NUM_BLOCK USB_COUNT6_RX_NUM_BLOCK_Msk                 /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT6_RX_NUM_BLOCK_0 (0x01UL << USB_COUNT6_RX_NUM_BLOCK_Pos)   /*!< 0x00000400 */
#define USB_COUNT6_RX_NUM_BLOCK_1 (0x02UL << USB_COUNT6_RX_NUM_BLOCK_Pos)   /*!< 0x00000800 */
#define USB_COUNT6_RX_NUM_BLOCK_2 (0x04UL << USB_COUNT6_RX_NUM_BLOCK_Pos)   /*!< 0x00001000 */
#define USB_COUNT6_RX_NUM_BLOCK_3 (0x08UL << USB_COUNT6_RX_NUM_BLOCK_Pos)   /*!< 0x00002000 */
#define USB_COUNT6_RX_NUM_BLOCK_4 (0x10UL << USB_COUNT6_RX_NUM_BLOCK_Pos)   /*!< 0x00004000 */

#define USB_COUNT6_RX_BLSIZE_Pos (15U)
#define USB_COUNT6_RX_BLSIZE_Msk (0x1UL << USB_COUNT6_RX_BLSIZE_Pos) /*!< 0x00008000 */
#define USB_COUNT6_RX_BLSIZE USB_COUNT6_RX_BLSIZE_Msk                /*!< BLock SIZE */

#define USB_COUNT7_RX_COUNT7_RX_Pos (0U)
#define USB_COUNT7_RX_COUNT7_RX_Msk (0x3FFUL << USB_COUNT7_RX_COUNT7_RX_Pos) /*!< 0x000003FF */
#define USB_COUNT7_RX_COUNT7_RX USB_COUNT7_RX_COUNT7_RX_Msk                  /*!< Reception Byte Count */

#define USB_COUNT7_RX_NUM_BLOCK_Pos (10U)
#define USB_COUNT7_RX_NUM_BLOCK_Msk (0x1FUL << USB_COUNT7_RX_NUM_BLOCK_Pos) /*!< 0x00007C00 */
#define USB_COUNT7_RX_NUM_BLOCK USB_COUNT7_RX_NUM_BLOCK_Msk                 /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT7_RX_NUM_BLOCK_0 (0x01UL << USB_COUNT7_RX_NUM_BLOCK_Pos)   /*!< 0x00000400 */
#define USB_COUNT7_RX_NUM_BLOCK_1 (0x02UL << USB_COUNT7_RX_NUM_BLOCK_Pos)   /*!< 0x00000800 */
#define USB_COUNT7_RX_NUM_BLOCK_2 (0x04UL << USB_COUNT7_RX_NUM_BLOCK_Pos)   /*!< 0x00001000 */
#define USB_COUNT7_RX_NUM_BLOCK_3 (0x08UL << USB_COUNT7_RX_NUM_BLOCK_Pos)   /*!< 0x00002000 */
#define USB_COUNT7_RX_NUM_BLOCK_4 (0x10UL << USB_COUNT7_RX_NUM_BLOCK_Pos)   /*!< 0x00004000 */

#define USB_COUNT7_RX_BLSIZE_Pos (15U)
#define USB_COUNT7_RX_BLSIZE_Msk (0x1UL << USB_COUNT7_RX_BLSIZE_Pos) /*!< 0x00008000 */
#define USB_COUNT7_RX_BLSIZE USB_COUNT7_RX_BLSIZE_Msk                /*!< BLock SIZE */

/*----------------------------------------------------------------------------*/

#define USB_COUNT0_RX_0_COUNT0_RX_0 0x000003FFU /*!< Reception Byte Count (low) */

#define USB_COUNT0_RX_0_NUM_BLOCK_0 0x00007C00U   /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT0_RX_0_NUM_BLOCK_0_0 0x00000400U /*!< Bit 0 */
#define USB_COUNT0_RX_0_NUM_BLOCK_0_1 0x00000800U /*!< Bit 1 */
#define USB_COUNT0_RX_0_NUM_BLOCK_0_2 0x00001000U /*!< Bit 2 */
#define USB_COUNT0_RX_0_NUM_BLOCK_0_3 0x00002000U /*!< Bit 3 */
#define USB_COUNT0_RX_0_NUM_BLOCK_0_4 0x00004000U /*!< Bit 4 */

#define USB_COUNT0_RX_0_BLSIZE_0 0x00008000U /*!< BLock SIZE (low) */

#define USB_COUNT0_RX_1_COUNT0_RX_1 0x03FF0000U /*!< Reception Byte Count (high) */

#define USB_COUNT0_RX_1_NUM_BLOCK_1 0x7C000000U   /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT0_RX_1_NUM_BLOCK_1_0 0x04000000U /*!< Bit 1 */
#define USB_COUNT0_RX_1_NUM_BLOCK_1_1 0x08000000U /*!< Bit 1 */
#define USB_COUNT0_RX_1_NUM_BLOCK_1_2 0x10000000U /*!< Bit 2 */
#define USB_COUNT0_RX_1_NUM_BLOCK_1_3 0x20000000U /*!< Bit 3 */
#define USB_COUNT0_RX_1_NUM_BLOCK_1_4 0x40000000U /*!< Bit 4 */

#define USB_COUNT0_RX_1_BLSIZE_1 0x80000000U /*!< BLock SIZE (high) */

#define USB_COUNT1_RX_0_COUNT1_RX_0 0x000003FFU /*!< Reception Byte Count (low) */

#define USB_COUNT1_RX_0_NUM_BLOCK_0 0x00007C00U   /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT1_RX_0_NUM_BLOCK_0_0 0x00000400U /*!< Bit 0 */
#define USB_COUNT1_RX_0_NUM_BLOCK_0_1 0x00000800U /*!< Bit 1 */
#define USB_COUNT1_RX_0_NUM_BLOCK_0_2 0x00001000U /*!< Bit 2 */
#define USB_COUNT1_RX_0_NUM_BLOCK_0_3 0x00002000U /*!< Bit 3 */
#define USB_COUNT1_RX_0_NUM_BLOCK_0_4 0x00004000U /*!< Bit 4 */

#define USB_COUNT1_RX_0_BLSIZE_0 0x00008000U /*!< BLock SIZE (low) */

#define USB_COUNT1_RX_1_COUNT1_RX_1 0x03FF0000U /*!< Reception Byte Count (high) */

#define USB_COUNT1_RX_1_NUM_BLOCK_1 0x7C000000U   /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT1_RX_1_NUM_BLOCK_1_0 0x04000000U /*!< Bit 0 */
#define USB_COUNT1_RX_1_NUM_BLOCK_1_1 0x08000000U /*!< Bit 1 */
#define USB_COUNT1_RX_1_NUM_BLOCK_1_2 0x10000000U /*!< Bit 2 */
#define USB_COUNT1_RX_1_NUM_BLOCK_1_3 0x20000000U /*!< Bit 3 */
#define USB_COUNT1_RX_1_NUM_BLOCK_1_4 0x40000000U /*!< Bit 4 */

#define USB_COUNT1_RX_1_BLSIZE_1 0x80000000U /*!< BLock SIZE (high) */

#define USB_COUNT2_RX_0_COUNT2_RX_0 0x000003FFU /*!< Reception Byte Count (low) */

#define USB_COUNT2_RX_0_NUM_BLOCK_0 0x00007C00U   /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT2_RX_0_NUM_BLOCK_0_0 0x00000400U /*!< Bit 0 */
#define USB_COUNT2_RX_0_NUM_BLOCK_0_1 0x00000800U /*!< Bit 1 */
#define USB_COUNT2_RX_0_NUM_BLOCK_0_2 0x00001000U /*!< Bit 2 */
#define USB_COUNT2_RX_0_NUM_BLOCK_0_3 0x00002000U /*!< Bit 3 */
#define USB_COUNT2_RX_0_NUM_BLOCK_0_4 0x00004000U /*!< Bit 4 */

#define USB_COUNT2_RX_0_BLSIZE_0 0x00008000U /*!< BLock SIZE (low) */

#define USB_COUNT2_RX_1_COUNT2_RX_1 0x03FF0000U /*!< Reception Byte Count (high) */

#define USB_COUNT2_RX_1_NUM_BLOCK_1 0x7C000000U   /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT2_RX_1_NUM_BLOCK_1_0 0x04000000U /*!< Bit 0 */
#define USB_COUNT2_RX_1_NUM_BLOCK_1_1 0x08000000U /*!< Bit 1 */
#define USB_COUNT2_RX_1_NUM_BLOCK_1_2 0x10000000U /*!< Bit 2 */
#define USB_COUNT2_RX_1_NUM_BLOCK_1_3 0x20000000U /*!< Bit 3 */
#define USB_COUNT2_RX_1_NUM_BLOCK_1_4 0x40000000U /*!< Bit 4 */

#define USB_COUNT2_RX_1_BLSIZE_1 0x80000000U /*!< BLock SIZE (high) */

#define USB_COUNT3_RX_0_COUNT3_RX_0 0x000003FFU /*!< Reception Byte Count (low) */

#define USB_COUNT3_RX_0_NUM_BLOCK_0 0x00007C00U   /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT3_RX_0_NUM_BLOCK_0_0 0x00000400U /*!< Bit 0 */
#define USB_COUNT3_RX_0_NUM_BLOCK_0_1 0x00000800U /*!< Bit 1 */
#define USB_COUNT3_RX_0_NUM_BLOCK_0_2 0x00001000U /*!< Bit 2 */
#define USB_COUNT3_RX_0_NUM_BLOCK_0_3 0x00002000U /*!< Bit 3 */
#define USB_COUNT3_RX_0_NUM_BLOCK_0_4 0x00004000U /*!< Bit 4 */

#define USB_COUNT3_RX_0_BLSIZE_0 0x00008000U /*!< BLock SIZE (low) */

#define USB_COUNT3_RX_1_COUNT3_RX_1 0x03FF0000U /*!< Reception Byte Count (high) */

#define USB_COUNT3_RX_1_NUM_BLOCK_1 0x7C000000U   /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT3_RX_1_NUM_BLOCK_1_0 0x04000000U /*!< Bit 0 */
#define USB_COUNT3_RX_1_NUM_BLOCK_1_1 0x08000000U /*!< Bit 1 */
#define USB_COUNT3_RX_1_NUM_BLOCK_1_2 0x10000000U /*!< Bit 2 */
#define USB_COUNT3_RX_1_NUM_BLOCK_1_3 0x20000000U /*!< Bit 3 */
#define USB_COUNT3_RX_1_NUM_BLOCK_1_4 0x40000000U /*!< Bit 4 */

#define USB_COUNT3_RX_1_BLSIZE_1 0x80000000U /*!< BLock SIZE (high) */

#define USB_COUNT4_RX_0_COUNT4_RX_0 0x000003FFU /*!< Reception Byte Count (low) */

#define USB_COUNT4_RX_0_NUM_BLOCK_0 0x00007C00U   /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT4_RX_0_NUM_BLOCK_0_0 0x00000400U /*!< Bit 0 */
#define USB_COUNT4_RX_0_NUM_BLOCK_0_1 0x00000800U /*!< Bit 1 */
#define USB_COUNT4_RX_0_NUM_BLOCK_0_2 0x00001000U /*!< Bit 2 */
#define USB_COUNT4_RX_0_NUM_BLOCK_0_3 0x00002000U /*!< Bit 3 */
#define USB_COUNT4_RX_0_NUM_BLOCK_0_4 0x00004000U /*!< Bit 4 */

#define USB_COUNT4_RX_0_BLSIZE_0 0x00008000U /*!< BLock SIZE (low) */

#define USB_COUNT4_RX_1_COUNT4_RX_1 0x03FF0000U /*!< Reception Byte Count (high) */

#define USB_COUNT4_RX_1_NUM_BLOCK_1 0x7C000000U   /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT4_RX_1_NUM_BLOCK_1_0 0x04000000U /*!< Bit 0 */
#define USB_COUNT4_RX_1_NUM_BLOCK_1_1 0x08000000U /*!< Bit 1 */
#define USB_COUNT4_RX_1_NUM_BLOCK_1_2 0x10000000U /*!< Bit 2 */
#define USB_COUNT4_RX_1_NUM_BLOCK_1_3 0x20000000U /*!< Bit 3 */
#define USB_COUNT4_RX_1_NUM_BLOCK_1_4 0x40000000U /*!< Bit 4 */

#define USB_COUNT4_RX_1_BLSIZE_1 0x80000000U /*!< BLock SIZE (high) */

#define USB_COUNT5_RX_0_COUNT5_RX_0 0x000003FFU /*!< Reception Byte Count (low) */

#define USB_COUNT5_RX_0_NUM_BLOCK_0 0x00007C00U   /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT5_RX_0_NUM_BLOCK_0_0 0x00000400U /*!< Bit 0 */
#define USB_COUNT5_RX_0_NUM_BLOCK_0_1 0x00000800U /*!< Bit 1 */
#define USB_COUNT5_RX_0_NUM_BLOCK_0_2 0x00001000U /*!< Bit 2 */
#define USB_COUNT5_RX_0_NUM_BLOCK_0_3 0x00002000U /*!< Bit 3 */
#define USB_COUNT5_RX_0_NUM_BLOCK_0_4 0x00004000U /*!< Bit 4 */

#define USB_COUNT5_RX_0_BLSIZE_0 0x00008000U /*!< BLock SIZE (low) */

#define USB_COUNT5_RX_1_COUNT5_RX_1 0x03FF0000U /*!< Reception Byte Count (high) */

#define USB_COUNT5_RX_1_NUM_BLOCK_1 0x7C000000U   /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT5_RX_1_NUM_BLOCK_1_0 0x04000000U /*!< Bit 0 */
#define USB_COUNT5_RX_1_NUM_BLOCK_1_1 0x08000000U /*!< Bit 1 */
#define USB_COUNT5_RX_1_NUM_BLOCK_1_2 0x10000000U /*!< Bit 2 */
#define USB_COUNT5_RX_1_NUM_BLOCK_1_3 0x20000000U /*!< Bit 3 */
#define USB_COUNT5_RX_1_NUM_BLOCK_1_4 0x40000000U /*!< Bit 4 */

#define USB_COUNT5_RX_1_BLSIZE_1 0x80000000U /*!< BLock SIZE (high) */

#define USB_COUNT6_RX_0_COUNT6_RX_0 0x000003FFU /*!< Reception Byte Count (low) */

#define USB_COUNT6_RX_0_NUM_BLOCK_0 0x00007C00U   /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT6_RX_0_NUM_BLOCK_0_0 0x00000400U /*!< Bit 0 */
#define USB_COUNT6_RX_0_NUM_BLOCK_0_1 0x00000800U /*!< Bit 1 */
#define USB_COUNT6_RX_0_NUM_BLOCK_0_2 0x00001000U /*!< Bit 2 */
#define USB_COUNT6_RX_0_NUM_BLOCK_0_3 0x00002000U /*!< Bit 3 */
#define USB_COUNT6_RX_0_NUM_BLOCK_0_4 0x00004000U /*!< Bit 4 */

#define USB_COUNT6_RX_0_BLSIZE_0 0x00008000U /*!< BLock SIZE (low) */

#define USB_COUNT6_RX_1_COUNT6_RX_1 0x03FF0000U /*!< Reception Byte Count (high) */

#define USB_COUNT6_RX_1_NUM_BLOCK_1 0x7C000000U   /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT6_RX_1_NUM_BLOCK_1_0 0x04000000U /*!< Bit 0 */
#define USB_COUNT6_RX_1_NUM_BLOCK_1_1 0x08000000U /*!< Bit 1 */
#define USB_COUNT6_RX_1_NUM_BLOCK_1_2 0x10000000U /*!< Bit 2 */
#define USB_COUNT6_RX_1_NUM_BLOCK_1_3 0x20000000U /*!< Bit 3 */
#define USB_COUNT6_RX_1_NUM_BLOCK_1_4 0x40000000U /*!< Bit 4 */

#define USB_COUNT6_RX_1_BLSIZE_1 0x80000000U /*!< BLock SIZE (high) */

#define USB_COUNT7_RX_0_COUNT7_RX_0 0x000003FFU /*!< Reception Byte Count (low) */

#define USB_COUNT7_RX_0_NUM_BLOCK_0 0x00007C00U   /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT7_RX_0_NUM_BLOCK_0_0 0x00000400U /*!< Bit 0 */
#define USB_COUNT7_RX_0_NUM_BLOCK_0_1 0x00000800U /*!< Bit 1 */
#define USB_COUNT7_RX_0_NUM_BLOCK_0_2 0x00001000U /*!< Bit 2 */
#define USB_COUNT7_RX_0_NUM_BLOCK_0_3 0x00002000U /*!< Bit 3 */
#define USB_COUNT7_RX_0_NUM_BLOCK_0_4 0x00004000U /*!< Bit 4 */

#define USB_COUNT7_RX_0_BLSIZE_0 0x00008000U /*!< BLock SIZE (low) */

#define USB_COUNT7_RX_1_COUNT7_RX_1 0x03FF0000U /*!< Reception Byte Count (high) */

#define USB_COUNT7_RX_1_NUM_BLOCK_1 0x7C000000U   /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT7_RX_1_NUM_BLOCK_1_0 0x04000000U /*!< Bit 0 */
#define USB_COUNT7_RX_1_NUM_BLOCK_1_1 0x08000000U /*!< Bit 1 */
#define USB_COUNT7_RX_1_NUM_BLOCK_1_2 0x10000000U /*!< Bit 2 */
#define USB_COUNT7_RX_1_NUM_BLOCK_1_3 0x20000000U /*!< Bit 3 */
#define USB_COUNT7_RX_1_NUM_BLOCK_1_4 0x40000000U /*!< Bit 4 */

#define USB_COUNT7_RX_1_BLSIZE_1 0x80000000U /*!< BLock SIZE (high) */

/*                                                                            */
/*                        Serial Peripheral Interface                         */
/*                                                                            */

#define SPI_CR1_CPHA (0x1UL << 0U) /*!< Clock Phase */
#define SPI_CR1_CPOL (0x1UL << 1U) /*!< Clock Polarity */
#define SPI_CR1_MSTR (0x1UL << 2U) /*!< Master Selection */

#define SPI_CR1_BR (0x7UL << 3U)               /*!< BR[2:0] bits (Baud Rate Control) */
#define SPI_CR1_BR_0 (0x1UL << SPI_CR1_BR_Pos) /*!< 0x00000008 */
#define SPI_CR1_BR_1 (0x2UL << SPI_CR1_BR_Pos) /*!< 0x00000010 */
#define SPI_CR1_BR_2 (0x4UL << SPI_CR1_BR_Pos) /*!< 0x00000020 */

#define SPI_CR1_SPE (0x1UL << 6U)       /*!< SPI Enable */
#define SPI_CR1_LSBFIRST (0x1UL << 7U)  /*!< Frame Format */
#define SPI_CR1_SSI (0x1UL << 8U)       /*!< Internal slave select */
#define SPI_CR1_SSM (0x1UL << 9U)       /*!< Software slave management */
#define SPI_CR1_RXONLY (0x1UL << 10U)   /*!< Receive only */
#define SPI_CR1_DFF (0x1UL << 11U)      /*!< Data Frame Format */
#define SPI_CR1_CRCNEXT (0x1UL << 12U)  /*!< Transmit CRC next */
#define SPI_CR1_CRCEN (0x1UL << 13U)    /*!< Hardware CRC calculation enable */
#define SPI_CR1_BIDIOE (0x1UL << 14U)   /*!< Output enable in bidirectional mode */
#define SPI_CR1_BIDIMODE (0x1UL << 15U) /*!< Bidirectional data mode enable */

#define SPI_CR2_RXDMAEN (0x1UL << 0U) /*!< Rx Buffer DMA Enable */
#define SPI_CR2_TXDMAEN (0x1UL << 1U) /*!< Tx Buffer DMA Enable */
#define SPI_CR2_SSOE (0x1UL << 2U)    /*!< SS Output Enable */
#define SPI_CR2_ERRIE (0x1UL << 5U)   /*!< Error Interrupt Enable */
#define SPI_CR2_RXNEIE (0x1UL << 6U)  /*!< RX buffer Not Empty Interrupt Enable */
#define SPI_CR2_TXEIE (0x1UL << 7U)   /*!< Tx buffer Empty Interrupt Enable */

#define SPI_SR_RXNE (0x1UL << 0U)   /*!< Receive buffer Not Empty */
#define SPI_SR_TXE (0x1UL << 1U)    /*!< Transmit buffer Empty */
#define SPI_SR_CHSIDE (0x1UL << 2U) /*!< Channel side */
#define SPI_SR_UDR (0x1UL << 3U)    /*!< Underrun flag */
#define SPI_SR_CRCERR (0x1UL << 4U) /*!< CRC Error flag */
#define SPI_SR_MODF (0x1UL << 5U)   /*!< Mode fault */
#define SPI_SR_OVR (0x1UL << 6U)    /*!< Overrun flag */
#define SPI_SR_BSY (0x1UL << 7U)    /*!< Busy flag */

#define SPI_DR_DR (0xFFFFUL << 0U)         /*!< Data Register */
#define SPI_CRCPR_CRCPOLY (0xFFFFUL << 0U) /*!< CRC polynomial register */
#define SPI_RXCRCR_RXCRC (0xFFFFUL << 0U)  /*!< Rx CRC Register */
#define SPI_TXCRCR_TXCRC (0xFFFFUL << 0U)  /*!< Tx CRC Register */
#define SPI_I2SCFGR_I2SMOD (0x1UL << 11U)  /*!< I2S mode selection */

/*                                                                            */
/*                      FLASH and Option Bytes Registers                      */
/*                                                                            */

#define FLASH_ACR_LATENCY (0x7UL << 0U) /*!< LATENCY[2:0] bits (Latency) */

#define FLASH_ACR_HLFCYA (0x1UL << 3U) /*!< Flash Half Cycle Access Enable */
#define FLASH_ACR_PRFTBE (0x1UL << 4U) /*!< Prefetch Buffer Enable */
#define FLASH_ACR_PRFTBS (0x1UL << 5U) /*!< Prefetch Buffer Status */

#define FLASH_KEYR_FKEYR (0xFFFFFFFFUL << 0U) /*!< FPEC Key */

#define RDP_KEY (0xA5UL << 0U)          /*!< RDP Key */
#define FLASH_KEY1 (0x45670123UL << 0U) /*!< FPEC Key1 */
#define FLASH_KEY2 (0xCDEF89ABUL << 0U) /*!< FPEC Key2 */

#define FLASH_OPTKEYR_OPTKEYR (0xFFFFFFFFUL << 0U) /*!< Option Byte Key */

#define FLASH_OPTKEY1 FLASH_KEY1 /*!< Option Byte Key1 */
#define FLASH_OPTKEY2 FLASH_KEY2 /*!< Option Byte Key2 */

#define FLASH_SR_BSY (0x1UL << 0U)      /*!< Busy */
#define FLASH_SR_PGERR (0x1UL << 2U)    /*!< Programming Error */
#define FLASH_SR_WRPRTERR (0x1UL << 4U) /*!< Write Protection Error */
#define FLASH_SR_EOP (0x1UL << 5U)      /*!< End of operation */

#define FLASH_CR_PG (0x1UL << 0U)     /*!< Programming */
#define FLASH_CR_PER (0x1UL << 1U)    /*!< Page Erase */
#define FLASH_CR_MER (0x1UL << 2U)    /*!< Mass Erase */
#define FLASH_CR_OPTPG (0x1UL << 4U)  /*!< Option Byte Programming */
#define FLASH_CR_OPTER (0x1UL << 5U)  /*!< Option Byte Erase */
#define FLASH_CR_STRT (0x1UL << 6U)   /*!< Start */
#define FLASH_CR_LOCK (0x1UL << 7U)   /*!< Lock */
#define FLASH_CR_OPTWRE (0x1UL << 9U) /*!< Option Bytes Write Enable */
#define FLASH_CR_ERRIE (0x1UL << 10U) /*!< Error Interrupt Enable */
#define FLASH_CR_EOPIE (0x1UL << 12U) /*!< End of operation interrupt enable */

#define FLASH_AR_FAR (0xFFFFFFFFUL << 0U) /*!< Flash Address */

#define FLASH_OBR_OPTERR (0x1UL << 0U) /*!< Option Byte Error */
#define FLASH_OBR_RDPRT (0x1UL << 1U)  /*!< Read protection */

#define FLASH_OBR_IWDG_SW (0x1UL << 2U)    /*!< IWDG SW */
#define FLASH_OBR_nRST_STOP (0x1UL << 3U)  /*!< nRST_STOP */
#define FLASH_OBR_nRST_STDBY (0x1UL << 4U) /*!< nRST_STDBY */
#define FLASH_OBR_USER (0x7UL << 2U)       /*!< User Option Bytes */
#define FLASH_OBR_DATA0 (0xFFUL << 10U)    /*!< Data0 */
#define FLASH_OBR_DATA1 (0xFFUL << 18U)    /*!< Data1 */

#define FLASH_WRPR_WRP (0xFFFFFFFFUL << 0U) /*!< Write Protect */

/*----------------------------------------------------------------------------*/

#define FLASH_RDP_RDP (0xFFUL << 0U)  /*!< Read protection option byte */
#define FLASH_RDP_nRDP (0xFFUL << 8U) /*!< Read protection complemented option byte */

#define FLASH_USER_USER (0xFFUL << 16U)  /*!< User option byte */
#define FLASH_USER_nUSER (0xFFUL << 24U) /*!< User complemented option byte */

#define FLASH_DATA0_DATA0 (0xFFUL << 0U)  /*!< User data storage option byte */
#define FLASH_DATA0_nDATA0 (0xFFUL << 8U) /*!< User data storage complemented option byte */

#define FLASH_DATA1_DATA1 (0xFFUL << 16U)  /*!< User data storage option byte */
#define FLASH_DATA1_nDATA1 (0xFFUL << 24U) /*!< User data storage complemented option byte */

#define FLASH_WRP0_WRP0 (0xFFUL << 0U)  /*!< Flash memory write protection option bytes */
#define FLASH_WRP0_nWRP0 (0xFFUL << 8U) /*!< Flash memory write protection complemented option bytes */

#define FLASH_WRP1_WRP1 (0xFFUL << 16U)  /*!< Flash memory write protection option bytes */
#define FLASH_WRP1_nWRP1 (0xFFUL << 24U) /*!< Flash memory write protection complemented option bytes */

#define IS_ADC_ALL_INSTANCE(INSTANCE) (((INSTANCE) == ADC1) || ((INSTANCE) == ADC2))
#define IS_ADC_COMMON_INSTANCE(INSTANCE) ((INSTANCE) == ADC12_COMMON)
#define IS_ADC_MULTIMODE_MASTER_INSTANCE(INSTANCE) ((INSTANCE) == ADC1)
#define IS_ADC_DMA_CAPABILITY_INSTANCE(INSTANCE) ((INSTANCE) == ADC1)
#define IS_CAN_ALL_INSTANCE(INSTANCE) ((INSTANCE) == CAN1)
#define IS_CRC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == CRC)
#define IS_DMA_ALL_INSTANCE(INSTANCE) (((INSTANCE) == DMA1_Channel1) || ((INSTANCE) == DMA1_Channel2) || ((INSTANCE) == DMA1_Channel3) || ((INSTANCE) == DMA1_Channel4) || ((INSTANCE) == DMA1_Channel5) || ((INSTANCE) == DMA1_Channel6) || ((INSTANCE) == DMA1_Channel7))
#define IS_GPIO_ALL_INSTANCE(INSTANCE) (((INSTANCE) == GPIOA) || ((INSTANCE) == GPIOB) || ((INSTANCE) == GPIOC) || ((INSTANCE) == GPIOD) || ((INSTANCE) == GPIOE))
#define IS_GPIO_AF_INSTANCE(INSTANCE) IS_GPIO_ALL_INSTANCE(INSTANCE)
#define IS_GPIO_LOCK_INSTANCE(INSTANCE) IS_GPIO_ALL_INSTANCE(INSTANCE)
#define IS_I2C_ALL_INSTANCE(INSTANCE) (((INSTANCE) == I2C1) || ((INSTANCE) == I2C2))
#define IS_SMBUS_ALL_INSTANCE IS_I2C_ALL_INSTANCE
#define IS_IWDG_ALL_INSTANCE(INSTANCE) ((INSTANCE) == IWDG)
#define IS_SPI_ALL_INSTANCE(INSTANCE) (((INSTANCE) == SPI1) || ((INSTANCE) == SPI2))
#define IS_TIM_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || ((INSTANCE) == TIM2) || ((INSTANCE) == TIM3) || ((INSTANCE) == TIM4))
#define IS_TIM_ADVANCED_INSTANCE(INSTANCE) ((INSTANCE) == TIM1)
#define IS_TIM_CC1_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || ((INSTANCE) == TIM2) || ((INSTANCE) == TIM3) || ((INSTANCE) == TIM4))
#define IS_TIM_CC2_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || ((INSTANCE) == TIM2) || ((INSTANCE) == TIM3) || ((INSTANCE) == TIM4))
#define IS_TIM_CC3_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || ((INSTANCE) == TIM2) || ((INSTANCE) == TIM3) || ((INSTANCE) == TIM4))
#define IS_TIM_CC4_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || ((INSTANCE) == TIM2) || ((INSTANCE) == TIM3) || ((INSTANCE) == TIM4))
#define IS_TIM_CLOCKSOURCE_ETRMODE1_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || ((INSTANCE) == TIM2) || ((INSTANCE) == TIM3) || ((INSTANCE) == TIM4))
#define IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || ((INSTANCE) == TIM2) || ((INSTANCE) == TIM3) || ((INSTANCE) == TIM4))
#define IS_TIM_CLOCKSOURCE_TIX_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || ((INSTANCE) == TIM2) || ((INSTANCE) == TIM3) || ((INSTANCE) == TIM4))
#define IS_TIM_CLOCKSOURCE_ITRX_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || ((INSTANCE) == TIM2) || ((INSTANCE) == TIM3) || ((INSTANCE) == TIM4))
#define IS_TIM_OCXREF_CLEAR_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || ((INSTANCE) == TIM2) || ((INSTANCE) == TIM3) || ((INSTANCE) == TIM4))
#define IS_TIM_ENCODER_INTERFACE_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || ((INSTANCE) == TIM2) || ((INSTANCE) == TIM3) || ((INSTANCE) == TIM4))
#define IS_TIM_XOR_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || ((INSTANCE) == TIM2) || ((INSTANCE) == TIM3) || ((INSTANCE) == TIM4))
#define IS_TIM_MASTER_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || ((INSTANCE) == TIM2) || ((INSTANCE) == TIM3) || ((INSTANCE) == TIM4))
#define IS_TIM_SLAVE_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || ((INSTANCE) == TIM2) || ((INSTANCE) == TIM3) || ((INSTANCE) == TIM4))
#define IS_TIM_DMABURST_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || ((INSTANCE) == TIM2) || ((INSTANCE) == TIM3) || ((INSTANCE) == TIM4))
#define IS_TIM_BREAK_INSTANCE(INSTANCE) ((INSTANCE) == TIM1)
#define IS_TIM_CCX_INSTANCE(INSTANCE, CHANNEL) ((((INSTANCE) == TIM1) && (((CHANNEL) == TIM_CHANNEL_1) || ((CHANNEL) == TIM_CHANNEL_2) || ((CHANNEL) == TIM_CHANNEL_3) || ((CHANNEL) == TIM_CHANNEL_4))) || (((INSTANCE) == TIM2) && (((CHANNEL) == TIM_CHANNEL_1) || ((CHANNEL) == TIM_CHANNEL_2) || ((CHANNEL) == TIM_CHANNEL_3) || ((CHANNEL) == TIM_CHANNEL_4))) || (((INSTANCE) == TIM3) && (((CHANNEL) == TIM_CHANNEL_1) || ((CHANNEL) == TIM_CHANNEL_2) || ((CHANNEL) == TIM_CHANNEL_3) || ((CHANNEL) == TIM_CHANNEL_4))) || (((INSTANCE) == TIM4) && (((CHANNEL) == TIM_CHANNEL_1) || ((CHANNEL) == TIM_CHANNEL_2) || ((CHANNEL) == TIM_CHANNEL_3) || ((CHANNEL) == TIM_CHANNEL_4))))
#define IS_TIM_CCXN_INSTANCE(INSTANCE, CHANNEL) (((INSTANCE) == TIM1) && (((CHANNEL) == TIM_CHANNEL_1) || ((CHANNEL) == TIM_CHANNEL_2) || ((CHANNEL) == TIM_CHANNEL_3)))
#define IS_TIM_COUNTER_MODE_SELECT_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || ((INSTANCE) == TIM2) || ((INSTANCE) == TIM3) || ((INSTANCE) == TIM4))
#define IS_TIM_REPETITION_COUNTER_INSTANCE(INSTANCE) ((INSTANCE) == TIM1)
#define IS_TIM_CLOCK_DIVISION_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || ((INSTANCE) == TIM2) || ((INSTANCE) == TIM3) || ((INSTANCE) == TIM4))
#define IS_TIM_DMA_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || ((INSTANCE) == TIM2) || ((INSTANCE) == TIM3) || ((INSTANCE) == TIM4))
#define IS_TIM_DMA_CC_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || ((INSTANCE) == TIM2) || ((INSTANCE) == TIM3) || ((INSTANCE) == TIM4))
#define IS_TIM_COMMUTATION_EVENT_INSTANCE(INSTANCE) ((INSTANCE) == TIM1)
#define IS_TIM_ETR_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || ((INSTANCE) == TIM2) || ((INSTANCE) == TIM3) || ((INSTANCE) == TIM4))
#define IS_TIM_HALL_SENSOR_INTERFACE_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || ((INSTANCE) == TIM2) || ((INSTANCE) == TIM3) || ((INSTANCE) == TIM4))
#define IS_TIM_32B_COUNTER_INSTANCE(INSTANCE) 0U
#define IS_USART_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || ((INSTANCE) == USART2) || ((INSTANCE) == USART3))
#define IS_UART_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || ((INSTANCE) == USART2) || ((INSTANCE) == USART3))
#define IS_UART_HALFDUPLEX_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || ((INSTANCE) == USART2) || ((INSTANCE) == USART3))
#define IS_UART_LIN_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || ((INSTANCE) == USART2) || ((INSTANCE) == USART3))
#define IS_UART_HWFLOW_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || ((INSTANCE) == USART2) || ((INSTANCE) == USART3))
#define IS_SMARTCARD_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || ((INSTANCE) == USART2) || ((INSTANCE) == USART3))
#define IS_IRDA_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || ((INSTANCE) == USART2) || ((INSTANCE) == USART3))
#define IS_UART_MULTIPROCESSOR_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || ((INSTANCE) == USART2) || ((INSTANCE) == USART3))
#define IS_UART_DMA_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || ((INSTANCE) == USART2) || ((INSTANCE) == USART3))
#define IS_RTC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == RTC)
#define IS_WWDG_ALL_INSTANCE(INSTANCE) ((INSTANCE) == WWDG)
#define IS_PCD_ALL_INSTANCE(INSTANCE) ((INSTANCE) == USB)

#define RCC_HSE_MIN 4000000U
#define RCC_HSE_MAX 16000000U

#define RCC_MAX_FREQUENCY 72000000U

/* Aliases for __IRQn */
#define ADC1_IRQn ADC1_2_IRQn
#define TIM9_IRQn TIM1_BRK_IRQn
#define TIM1_BRK_TIM9_IRQn TIM1_BRK_IRQn
#define TIM1_BRK_TIM15_IRQn TIM1_BRK_IRQn
#define TIM1_TRG_COM_TIM17_IRQn TIM1_TRG_COM_IRQn
#define TIM1_TRG_COM_TIM11_IRQn TIM1_TRG_COM_IRQn
#define TIM11_IRQn TIM1_TRG_COM_IRQn
#define TIM10_IRQn TIM1_UP_IRQn
#define TIM1_UP_TIM10_IRQn TIM1_UP_IRQn
#define TIM1_UP_TIM16_IRQn TIM1_UP_IRQn
#define OTG_FS_WKUP_IRQn USBWakeUp_IRQn
#define CEC_IRQn USBWakeUp_IRQn
#define USB_HP_IRQn USB_HP_CAN1_TX_IRQn
#define CAN1_TX_IRQn USB_HP_CAN1_TX_IRQn
#define CAN1_RX0_IRQn USB_LP_CAN1_RX0_IRQn
#define USB_LP_IRQn USB_LP_CAN1_RX0_IRQn

/* Aliases for __IRQHandler */
#define ADC1_IRQHandler ADC1_2_IRQHandler
#define TIM9_IRQHandler TIM1_BRK_IRQHandler
#define TIM1_BRK_TIM9_IRQHandler TIM1_BRK_IRQHandler
#define TIM1_BRK_TIM15_IRQHandler TIM1_BRK_IRQHandler
#define TIM1_TRG_COM_TIM17_IRQHandler TIM1_TRG_COM_IRQHandler
#define TIM1_TRG_COM_TIM11_IRQHandler TIM1_TRG_COM_IRQHandler
#define TIM11_IRQHandler TIM1_TRG_COM_IRQHandler
#define TIM10_IRQHandler TIM1_UP_IRQHandler
#define TIM1_UP_TIM10_IRQHandler TIM1_UP_IRQHandler
#define TIM1_UP_TIM16_IRQHandler TIM1_UP_IRQHandler
#define OTG_FS_WKUP_IRQHandler USBWakeUp_IRQHandler
#define CEC_IRQHandler USBWakeUp_IRQHandler
#define USB_HP_IRQHandler USB_HP_CAN1_TX_IRQHandler
#define CAN1_TX_IRQHandler USB_HP_CAN1_TX_IRQHandler
#define CAN1_RX0_IRQHandler USB_LP_CAN1_RX0_IRQHandler
#define USB_LP_IRQHandler USB_LP_CAN1_RX0_IRQHandler

#endif /* __STM32F103xB_H */

typedef enum
{
  RESET = 0,
  SET = !RESET
} FlagStatus,
    ITStatus;

typedef enum
{
  DISABLE = 0,
  ENABLE = !DISABLE
} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum
{
  SUCCESS = 0U,
  ERROR = !SUCCESS
} ErrorStatus;

#define SET_BIT(REG, BIT) ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT) ((REG) & (BIT))
#define CLEAR_REG(REG) ((REG) = (0x0))
#define WRITE_REG(REG, VAL) ((REG) = (VAL))
#define READ_REG(REG) ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK) WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define POSITION_VAL(VAL) (__CLZ(__RBIT(VAL)))

// #endif /* __STM32F1xx_H */

#endif /* __STM32F1xx_HAL_H */








































#ifndef Inc_USB_Device__
#define Inc_USB_Device__

#define USBD_VID 1155
#define USBD_LANGID_STRING 1033
#define USBD_MANUFACTURER_STRING "STMicroelectronics"
#define USBD_PID_FS 22315
#define USBD_PRODUCT_STRING_FS "STM32 Human interface"
#define USBD_CONFIGURATION_STRING_FS "HID Config"
#define USBD_INTERFACE_STRING_FS "HID Interface"

#define USBD_MAX_NUM_INTERFACES 1
#define USBD_MAX_NUM_CONFIGURATION 1
#define USBD_MAX_STR_DESC_SIZ 512
#define USBD_DEBUG_LEVEL 0
#define USBD_SELF_POWERED 1
#define HID_FS_BINTERVAL 0xA

#define DEVICE_FS 0

#define USBD_malloc (uint32_t *)USBD_static_malloc
#define USBD_free USBD_static_free
#define USBD_Delay HAL_Delay

void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);

#define USBD_UsrLog(...)
#define USBD_ErrLog(...)
#define USBD_DbgLog(...)

#ifndef NULL
#define NULL 0U
#endif /* NULL */

#define USBD_MAX_NUM_INTERFACES 1U
#define USBD_MAX_NUM_CONFIGURATION 1U

#define USBD_LPM_ENABLED 0U

#ifndef USBD_SUPPORT_USER_STRING_DESC
#define USBD_SUPPORT_USER_STRING_DESC 0U
#endif /* USBD_SUPPORT_USER_STRING_DESC */

#define USB_LEN_DEV_QUALIFIER_DESC 0x0AU
#define USB_LEN_DEV_DESC 0x12U
#define USB_LEN_CFG_DESC 0x09U
#define USB_LEN_IF_DESC 0x09U
#define USB_LEN_EP_DESC 0x07U
#define USB_LEN_OTG_DESC 0x03U
#define USB_LEN_LANGID_STR_DESC 0x04U
#define USB_LEN_OTHER_SPEED_DESC_SIZ 0x09U

#define USBD_IDX_LANGID_STR 0x00U
#define USBD_IDX_MFC_STR 0x01U
#define USBD_IDX_PRODUCT_STR 0x02U
#define USBD_IDX_SERIAL_STR 0x03U
#define USBD_IDX_CONFIG_STR 0x04U
#define USBD_IDX_INTERFACE_STR 0x05U

#define USB_REQ_TYPE_STANDARD 0x00U
#define USB_REQ_TYPE_CLASS 0x20U
#define USB_REQ_TYPE_VENDOR 0x40U
#define USB_REQ_TYPE_MASK 0x60U

#define USB_REQ_RECIPIENT_DEVICE 0x00U
#define USB_REQ_RECIPIENT_INTERFACE 0x01U
#define USB_REQ_RECIPIENT_ENDPOINT 0x02U
#define USB_REQ_RECIPIENT_MASK 0x03U

#define USB_REQ_GET_STATUS 0x00U
#define USB_REQ_CLEAR_FEATURE 0x01U
#define USB_REQ_SET_FEATURE 0x03U
#define USB_REQ_SET_ADDRESS 0x05U
#define USB_REQ_GET_DESCRIPTOR 0x06U
#define USB_REQ_SET_DESCRIPTOR 0x07U
#define USB_REQ_GET_CONFIGURATION 0x08U
#define USB_REQ_SET_CONFIGURATION 0x09U
#define USB_REQ_GET_INTERFACE 0x0AU
#define USB_REQ_SET_INTERFACE 0x0BU
#define USB_REQ_SYNCH_FRAME 0x0CU

#define USB_DESC_TYPE_DEVICE 0x01U
#define USB_DESC_TYPE_CONFIGURATION 0x02U
#define USB_DESC_TYPE_STRING 0x03U
#define USB_DESC_TYPE_INTERFACE 0x04U
#define USB_DESC_TYPE_ENDPOINT 0x05U
#define USB_DESC_TYPE_DEVICE_QUALIFIER 0x06U
#define USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION 0x07U
#define USB_DESC_TYPE_BOS 0x0FU

#define USB_CONFIG_REMOTE_WAKEUP 0x02U
#define USB_CONFIG_SELF_POWERED 0x01U

#define USB_FEATURE_EP_HALT 0x00U
#define USB_FEATURE_REMOTE_WAKEUP 0x01U
#define USB_FEATURE_TEST_MODE 0x02U

#define USB_DEVICE_CAPABITY_TYPE 0x10U

#define USB_HS_MAX_PACKET_SIZE 512U
#define USB_FS_MAX_PACKET_SIZE 64U
#define USB_MAX_EP0_SIZE 64U

/*  Device Status */
#define USBD_STATE_DEFAULT 0x01U
#define USBD_STATE_ADDRESSED 0x02U
#define USBD_STATE_CONFIGURED 0x03U
#define USBD_STATE_SUSPENDED 0x04U

/*  EP0 State */
#define USBD_EP0_IDLE 0x00U
#define USBD_EP0_SETUP 0x01U
#define USBD_EP0_DATA_IN 0x02U
#define USBD_EP0_DATA_OUT 0x03U
#define USBD_EP0_STATUS_IN 0x04U
#define USBD_EP0_STATUS_OUT 0x05U
#define USBD_EP0_STALL 0x06U

#define USBD_EP_TYPE_CTRL 0x00U
#define USBD_EP_TYPE_ISOC 0x01U
#define USBD_EP_TYPE_BULK 0x02U
#define USBD_EP_TYPE_INTR 0x03U

#define SWAPBYTE(addr) (((uint16_t)(*((uint8_t *)(addr)))) + \
                        (((uint16_t)(*(((uint8_t *)(addr)) + 1U))) << 8U))

#define LOBYTE(x) ((uint8_t)((x) & 0x00FFU))
#define HIBYTE(x) ((uint8_t)(((x) & 0xFF00U) >> 8U))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

#define HID_EPIN_ADDR 0x81U
#define HID_EPIN_SIZE 0x04U

#define USB_HID_CONFIG_DESC_SIZ 34U
#define USB_HID_DESC_SIZ 9U
#define HID_MOUSE_REPORT_DESC_SIZE 63U

#define HID_DESCRIPTOR_TYPE 0x21U
#define HID_REPORT_DESC 0x22U

#define HID_HS_BINTERVAL 0x07U

#define HID_REQ_SET_PROTOCOL 0x0BU
#define HID_REQ_GET_PROTOCOL 0x03U

#define HID_REQ_SET_IDLE 0x0AU
#define HID_REQ_GET_IDLE 0x02U

#define HID_REQ_SET_REPORT 0x09U
#define HID_REQ_GET_REPORT 0x01U

typedef struct usb_setup_req
{
  uint8_t bmRequest;
  uint8_t bRequest;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
} USBD_SetupReqTypedef;

struct _USBD_HandleTypeDef;

typedef struct _Device_cb
{
  uint8_t (*Init)(struct _USBD_HandleTypeDef *pdev, uint8_t cfgidx);
  uint8_t (*DeInit)(struct _USBD_HandleTypeDef *pdev, uint8_t cfgidx);
  uint8_t (*Setup)(struct _USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
  uint8_t (*EP0_TxSent)(struct _USBD_HandleTypeDef *pdev);
  uint8_t (*EP0_RxReady)(struct _USBD_HandleTypeDef *pdev);
  uint8_t (*DataIn)(struct _USBD_HandleTypeDef *pdev, uint8_t epnum);
  uint8_t (*DataOut)(struct _USBD_HandleTypeDef *pdev, uint8_t epnum);
  uint8_t (*SOF)(struct _USBD_HandleTypeDef *pdev);
  uint8_t (*IsoINIncomplete)(struct _USBD_HandleTypeDef *pdev, uint8_t epnum);
  uint8_t (*IsoOUTIncomplete)(struct _USBD_HandleTypeDef *pdev, uint8_t epnum);
  uint8_t *(*GetHSConfigDescriptor)(uint16_t *length);
  uint8_t *(*GetFSConfigDescriptor)(uint16_t *length);
  uint8_t *(*GetOtherSpeedConfigDescriptor)(uint16_t *length);
  uint8_t *(*GetDeviceQualifierDescriptor)(uint16_t *length);
} USBD_ClassTypeDef;

/* Following USB Device Speed */
typedef enum
{
  USBD_SPEED_HIGH = 0U,
  USBD_SPEED_FULL = 1U,
  USBD_SPEED_LOW = 2U,
} USBD_SpeedTypeDef;

/* Following USB Device status */
typedef enum
{
  USBD_OK = 0U,
  USBD_BUSY,
  USBD_FAIL,
} USBD_StatusTypeDef;

/* USB Device descriptors structure */
typedef struct
{
  uint8_t *(*GetDeviceDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetLangIDStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetManufacturerStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetProductStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetSerialStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetConfigurationStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetInterfaceStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
} USBD_DescriptorsTypeDef;

/* USB Device handle structure */
typedef struct
{
  uint32_t status;
  uint32_t is_used;
  uint32_t total_length;
  uint32_t rem_length;
  uint32_t maxpacket;
} USBD_EndpointTypeDef;

/* USB Device handle structure */
typedef struct _USBD_HandleTypeDef
{
  uint8_t id;
  uint32_t dev_config;
  uint32_t dev_default_config;
  uint32_t dev_config_status;
  USBD_SpeedTypeDef dev_speed;
  USBD_EndpointTypeDef ep_in[16];
  USBD_EndpointTypeDef ep_out[16];
  uint32_t ep0_state;
  uint32_t ep0_data_len;
  uint8_t dev_state;
  uint8_t dev_old_state;
  uint8_t dev_address;
  uint8_t dev_connection_status;
  uint8_t dev_test_mode;
  uint32_t dev_remote_wakeup;

  USBD_SetupReqTypedef request;
  USBD_DescriptorsTypeDef *pDesc;
  USBD_ClassTypeDef *pClass;
  void *pClassData;
  void *pUserData;
  void *pData;
} USBD_HandleTypeDef;

USBD_StatusTypeDef USBD_StdDevReq(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
USBD_StatusTypeDef USBD_StdItfReq(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
USBD_StatusTypeDef USBD_StdEPReq(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

void USBD_CtlError(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

void USBD_ParseSetupRequest(USBD_SetupReqTypedef *req, uint8_t *pdata);

void USBD_GetString(uint8_t *desc, uint8_t *unicode, uint16_t *len);

#define USBD_SOF USBD_LL_SOF

USBD_StatusTypeDef USBD_Init(USBD_HandleTypeDef *pdev, USBD_DescriptorsTypeDef *pdesc, uint8_t id);
USBD_StatusTypeDef USBD_DeInit(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef USBD_Start(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef USBD_Stop(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef USBD_RegisterClass(USBD_HandleTypeDef *pdev, USBD_ClassTypeDef *pclass);

USBD_StatusTypeDef USBD_RunTestMode(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef USBD_SetClassConfig(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
USBD_StatusTypeDef USBD_ClrClassConfig(USBD_HandleTypeDef *pdev, uint8_t cfgidx);

USBD_StatusTypeDef USBD_LL_SetupStage(USBD_HandleTypeDef *pdev, uint8_t *psetup);
USBD_StatusTypeDef USBD_LL_DataOutStage(USBD_HandleTypeDef *pdev, uint8_t epnum, uint8_t *pdata);
USBD_StatusTypeDef USBD_LL_DataInStage(USBD_HandleTypeDef *pdev, uint8_t epnum, uint8_t *pdata);

USBD_StatusTypeDef USBD_LL_Reset(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef USBD_LL_SetSpeed(USBD_HandleTypeDef *pdev, USBD_SpeedTypeDef speed);
USBD_StatusTypeDef USBD_LL_Suspend(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef USBD_LL_Resume(USBD_HandleTypeDef *pdev);

USBD_StatusTypeDef USBD_LL_SOF(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef USBD_LL_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
USBD_StatusTypeDef USBD_LL_IsoOUTIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);

USBD_StatusTypeDef USBD_LL_DevConnected(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef USBD_LL_DevDisconnected(USBD_HandleTypeDef *pdev);

/* USBD Low Level Driver */
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps);

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr);
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr);
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr);
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr);
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr);
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr);
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint16_t size);

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint16_t size);

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr);
void USBD_LL_Delay(uint32_t Delay);

USBD_StatusTypeDef USBD_CtlSendData(USBD_HandleTypeDef *pdev, uint8_t *pbuf, uint16_t len);
USBD_StatusTypeDef USBD_CtlContinueSendData(USBD_HandleTypeDef *pdev, uint8_t *pbuf, uint16_t len);
USBD_StatusTypeDef USBD_CtlPrepareRx(USBD_HandleTypeDef *pdev, uint8_t *pbuf, uint16_t len);
USBD_StatusTypeDef USBD_CtlContinueRx(USBD_HandleTypeDef *pdev, uint8_t *pbuf, uint16_t len);
USBD_StatusTypeDef USBD_CtlSendStatus(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef USBD_CtlReceiveStatus(USBD_HandleTypeDef *pdev);

uint32_t USBD_GetRxCount(USBD_HandleTypeDef *pdev, uint8_t ep_addr);

typedef enum
{
  HID_IDLE = 0,
  HID_BUSY,
} HID_StateTypeDef;

typedef struct
{
  uint32_t Protocol;
  uint32_t IdleState;
  uint32_t AltSetting;
  HID_StateTypeDef state;
} USBD_HID_HandleTypeDef;

extern USBD_ClassTypeDef USBD_HID;
#define USBD_HID_CLASS &USBD_HID

uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len);
uint32_t USBD_HID_GetPollingInterval(USBD_HandleTypeDef *pdev);

#define UID_BASE 0x1FFFF7E8UL /*!< Unique device ID register base address */

#define DEVICE_ID1 (UID_BASE)
#define DEVICE_ID2 (UID_BASE + 0x4)
#define DEVICE_ID3 (UID_BASE + 0x8)
#define USB_SIZ_STRING_SERIAL 0x1A

#define USBD_SUPPORT_USER_STRING_DESC 0U
extern USBD_DescriptorsTypeDef FS_Desc;

USBD_HandleTypeDef hUsbDeviceFS;

typedef struct
{
  uint8_t ModifierKey;
  uint8_t Reserved;
  uint8_t KeyCode1;
  uint8_t KeyCode2;
  uint8_t KeyCode3;
  uint8_t KeyCode4;
  uint8_t KeyCode5;
  uint8_t KeyCode6;
} keyboardHID;

void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);

#endif // Inc_USB_Device__
