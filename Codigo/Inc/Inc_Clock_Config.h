#ifndef Clock_Config
#define Clock_Config

#define RCC_HSE_PREDIV_DIV1             0x00000000U
#define RCC_PLL_MUL9                    RCC_CFGR_PLLMULL9
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

#define RCC_HSI_ON                       0x1UL <<0U           /*!< HSI clock activation */
#define RCC_OSCILLATORTYPE_HSE           0x00000001U
#define RCC_HSE_ON                       RCC_CR_HSEON                /*!< HSE clock activation */
#define RCC_PLL_ON                       0x00000002U /*!< PLL activation */
#define RCC_PLLSOURCE_HSE                RCC_CFGR_PLLSRC      /*!< HSE clock selected as PLL entry clock source */
#define RCC_CLOCKTYPE_SYSCLK             0x00000001U /*!< SYSCLK to configure */
#define RCC_CLOCKTYPE_HCLK               0x00000002U /*!< HCLK to configure */
#define RCC_CLOCKTYPE_PCLK1              0x00000004U /*!< PCLK1 to configure */
#define RCC_CLOCKTYPE_PCLK2              0x00000008U /*!< PCLK2 to configure */
#define RCC_SYSCLKSOURCE_PLLCLK          RCC_CFGR_SW_PLL /*!< PLL selected as system clock */
#define RCC_HCLK_DIV1                    RCC_CFGR_PPRE1_DIV1  /*!< HCLK not divided */
#define RCC_HCLK_DIV2                    RCC_CFGR_PPRE1_DIV2  /*!< HCLK divided by 2 */
#define RCC_SYSCLK_DIV1                  RCC_CFGR_HPRE_DIV1  /*!< SYSCLK not divided */

typedef struct{
  uint32_t ClockType;
  uint32_t SYSCLKSource;
  uint32_t AHBCLKDivider;
  uint32_t APB1CLKDivider;
  uint32_t APB2CLKDivider;
} RCC_ClkInitTypeDef;

#endif //Clock_Config

