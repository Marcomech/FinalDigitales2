typedef int            int32_t;
typedef short          int16_t;
typedef char           int8_t;
typedef unsigned int   uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char  uint8_t;

#define SRAM_SIZE ((uint32_t) 0x00005000)
#define SRAM_BASE ((uint32_t) 0x20000000)
#define STACKINIT ((interrupt_t)(SRAM_BASE+SRAM_SIZE))

int main(void);

typedef void(*interrupt_t)(void);
const interrupt_t vector_table[256] __attribute__ ((section(".vtab"))) = {
	STACKINIT,												// 0x0000_0000 Stack Pointer
	(interrupt_t) main,										// 0x0000_0004 Reset
};

struct RCC {
    unsigned int CR;
    unsigned int CFGR;
    unsigned int CIR;
    unsigned int APB2RSTR;
    unsigned int APB1RSTR;
    unsigned int AHBENR;
    unsigned int APB2ENR;
    unsigned int APB1ENR;
    unsigned int BDCR;
    unsigned int CSR;
    unsigned int AHBRSTR;
    unsigned int CFGR2;
};

volatile struct RCC *const RCC = (struct RCC *)(0x40021000);
//RCC->APB2ENR |= (A=2; B=3; C=4)

struct GPIOx {
    volatile int CRL;
    volatile int CRH;
    volatile int IN ;
    volatile int OUT;
};

volatile struct GPIOx *const GPIOA = (struct GPIOx *)(0x40010800);
volatile struct GPIOx *const GPIOB = (struct GPIOx *)(0x40010C00);
volatile struct GPIOx *const GPIOC = (struct GPIOx *)(0x40011000);

struct SysTick{
    volatile int CSR;   //Para habilitar y leer
    // 0 Enable | 1 Interrupciones | 2 Clock (0= 1Megas o 1=8Megas) | 16 Flag

    volatile int ReloadValue;   //Reload, valor en el que empieza el contador
    volatile int CurrentValue;  //Current Value
    volatile int Calibracion;   //Calibracion, da 1 cada 0.01seg
};

volatile struct SysTick *const SYST = (struct SysTick *)(0xE000E010);
volatile int Time_1ms = 1000;

#define NULL 0

//
//
//
//
//
//
//Clock config
//
//
//
//
//



typedef struct
{
  uint32_t PLLState;
  uint32_t PLLSource;
  uint32_t PLLMUL;
} RCC_PLLInitTypeDef;

typedef struct
{
  uint32_t OscillatorType;
  uint32_t HSEState;
  uint32_t HSEPredivValue;
  uint32_t LSEState;
  uint32_t HSIState;
  uint32_t HSICalibrationValue;
  uint32_t LSIState;
  RCC_PLLInitTypeDef PLL;
} RCC_OscInitTypeDef;

typedef struct
{
  uint32_t PeriphClockSelection;
  uint32_t RTCClockSelection;
  uint32_t AdcClockSelection;
  uint32_t UsbClockSelection;
} RCC_PeriphCLKInitTypeDef;

typedef struct
{
  uint32_t ClockType;
  uint32_t SYSCLKSource;
  uint32_t AHBCLKDivider;
  uint32_t APB1CLKDivider;
  uint32_t APB2CLKDivider;
} RCC_ClkInitTypeDef;


typedef enum
{
  HAL_OK      = 0x00U,
  HAL_ERROR   = 0x01U,
  HAL_BUSY    = 0x02U,
  HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

uint32_t SystemCoreClock = 16000000;     /*!< System Clock Frequency (Core Clock) */

const uint8_t AHBPrescTable[16U] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8U] = {0, 0, 0, 0, 1, 2, 3, 4};


#define RCC_SYSCLKSOURCE_STATUS_HSI 0x00000000U
#define RCC_SYSCLKSOURCE_STATUS_HSE 0x00000004U
#define RCC_SYSCLKSOURCE_STATUS_PLLCLK 0x00000008U


#define HSI_TIMEOUT_VALUE 2U /* 2 ms (minimum Tick + 1) */
#define LSI_TIMEOUT_VALUE 2U /* 2 ms (minimum Tick + 1) */
#define PLL_TIMEOUT_VALUE 2U /* 2 ms (minimum Tick + 1) */
#define CLOCKSWITCH_TIMEOUT_VALUE 5000 /* 5 s    */

#define RCC_PLL_OFF 0x00000001U  /*!< PLL deactivation */


#define RCC_CFGR_PLLSRC (0x1UL << 16U) 
#define RCC_CFGR_PLLMULL (0xFUL << 18U) 

#define RCC_CFGR_HPRE_Pos (4U)

#define RCC_CR_HSION (0x1UL << 0U)
#define RCC_CR_HSIRDY (0x1UL << 1U)
#define RCC_CR_HSITRIM (0x1FUL << 3U)
#define RCC_CR_HSICAL (0xFFUL << 8U)
#define RCC_CR_HSERDY (0x1UL << 17U)
#define RCC_CR_HSEBYP (0x1UL << 18U)
#define RCC_CR_CSSON (0x1UL << 19U)
#define RCC_CR_PLLON (0x1UL << 24U)
#define RCC_CR_PLLRDY (0x1UL << 25U)

#define RCC_HSI_ON RCC_CR_HSION /*!< HSI clock activation */
#define RCC_OSCILLATORTYPE_HSE 0x00000001U
#define RCC_HSE_ON (0x1UL << 16U)             /*!< HSE clock activation */
#define RCC_PLL_ON 0x00000002U              /*!< PLL activation */
#define RCC_PLLSOURCE_HSE (0x1UL << 16U)    /*!< HSE clock selected as PLL entry clock source */
#define RCC_CLOCKTYPE_SYSCLK 0x00000001U    /*!< SYSCLK to configure */
#define RCC_CLOCKTYPE_HCLK 0x00000002U      /*!< HCLK to configure */
#define RCC_CLOCKTYPE_PCLK1 0x00000004U     /*!< PCLK1 to configure */
#define RCC_CLOCKTYPE_PCLK2 0x00000008U     /*!< PCLK2 to configure */
#define RCC_SYSCLKSOURCE_PLLCLK 0x00000002U /*!< PLL selected as system clock */
#define RCC_HCLK_DIV1 0x00000000U           /*!< HCLK not divided */
#define RCC_HCLK_DIV2 0x00000400U           /*!< HCLK divided by 2 */
#define RCC_SYSCLK_DIV1 0x00000000U         /*!< SYSCLK not divided */


#define RCC_HSE_PREDIV_DIV1 0x00000000U
#define RCC_PLL_MUL9 (0x7UL << 18U)
#define RCC_PERIPHCLK_USB 0x00000010U
#define RCC_USBCLKSOURCE_PLL_DIV1_5 0x00000000U

#define FLASH_LATENCY_0 0x00000000U /*!< FLASH Zero Latency cycle */
#define FLASH_LATENCY_1 0x00000001  /*!< FLASH One Latency cycle */
#define FLASH_LATENCY_2 0x00000002  /*!< FLASH Two Latency cycles */

#define RCC_CFGR_PPRE1_DIV4 0x00000500U  /*!< HCLK divided by 4 */
#define RCC_CFGR_PPRE1_DIV8 0x00000600U  /*!< HCLK divided by 8 */
#define RCC_CFGR_PPRE1_DIV16 0x00000700U /*!< HCLK divided by 16 */

#define RCC_FLAG_HSERDY ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_HSERDY_Pos)) /*!< External High Speed clock ready flag */

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

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CSR;
} PWR_TypeDef;

#define RCC_HSE_OFF 0x00000000U          



#define RCC_OSCILLATORTYPE_HSI 0x00000002U
#define RCC_OSCILLATORTYPE_LSE 0x00000004U
#define RCC_OSCILLATORTYPE_LSI 0x00000008U



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


#define CR_REG_INDEX ((uint8_t)1)

#define RCC_FLAG_HSIRDY ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_HSIRDY_Pos)) /*!< Internal High Speed clock ready flag */
#define RCC_FLAG_HSERDY ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_HSERDY_Pos)) /*!< External High Speed clock ready flag */
#define RCC_FLAG_PLLRDY ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_PLLRDY_Pos)) /*!< PLL clock ready flag */

#define RCC_FLAG_LSIRDY ((uint8_t)((CSR_REG_INDEX << 5U) | RCC_CSR_LSIRDY_Pos)) /*!< Internal Low Speed oscillator Ready */
#define RCC_FLAG_LSERDY ((uint8_t)((BDCR_REG_INDEX << 5U) | RCC_BDCR_LSERDY_Pos)) /*!< External Low Speed oscillator Ready */



#define HSE_VALUE 8000000U        /*!< Value of the External oscillator in Hz */
#define HSI_VALUE 8000000U        /*!< Value of the Internal oscillator in Hz*/
#define LSI_VALUE 40000U          /*!< LSI Typical Value in Hz */
#define LSE_VALUE 32768U          /*!< Value of the External oscillator in Hz*/
#define LSE_STARTUP_TIMEOUT 5000U /*!< Time out for LSE start up, in ms */
#define HSE_STARTUP_TIMEOUT 100U  /*!< Time out for HSE start up, in ms */
#define HSE_TIMEOUT_VALUE 100U


#define RCC_PLLSOURCE_HSI_DIV2 0x00000000U /*!< HSI clock divided by 2 selected as PLL entry clock source */


#define RCC_CFGR_SWS_Pos (2U)
#define RCC_CFGR_SW_HSE 0x00000001U /*!< HSE selected as system clock */

#define RCC_HSI_OFF 0x00000000U                                       /*!< HSI clock deactivation */

#define RCC_LSI_OFF 0x00000000U /*!< LSI clock deactivation */
#define CSR_REG_INDEX ((uint8_t)3)
#define RCC_CSR_LSIRDY_Pos (1U)

#define APB1PERIPH_BASE 0x40000000UL

#define PWR_BASE (APB1PERIPH_BASE + 0x00007000UL)

#define PWR ((PWR_TypeDef *)PWR_BASE)

#define PWR_CR_DBP_Pos (8U)
#define PWR_CR_DBP_Msk (0x1UL << PWR_CR_DBP_Pos) /*!< 0x00000100 */
#define PWR_CR_DBP PWR_CR_DBP_Msk                /*!< Disable Backup Domain write protection */

#define RCC_DBP_TIMEOUT_VALUE 100U /* 100 ms */
#define RCC_LSE_OFF 0x00000000U                                       /*!< LSE clock deactivation */
#define BDCR_REG_INDEX ((uint8_t)2)
#define RCC_BDCR_LSERDY_Pos (1U)
#define RCC_LSE_TIMEOUT_VALUE LSE_STARTUP_TIMEOUT
#define RCC_PLL_NONE 0x00000000U /*!< PLL is not configured */

#define RCC_CFGR_PPRE1_Pos (8U)
#define RCC_CFGR_PPRE1_Msk (0x7UL << RCC_CFGR_PPRE1_Pos) /*!< 0x00000700 */
#define RCC_CFGR_PPRE1 RCC_CFGR_PPRE1_Msk                /*!< PRE1[2:0] bits (APB1 prescaler) */

#define RCC_HCLK_DIV16 RCC_CFGR_PPRE1_DIV16                                   /*!< HCLK divided by 16 */
#define RCC_CFGR_PPRE2_Pos (11U)
#define RCC_CFGR_PPRE2_Msk (0x7UL << RCC_CFGR_PPRE2_Pos) /*!< 0x00003800 */
#define RCC_CFGR_PPRE2 RCC_CFGR_PPRE2_Msk                /*!< PRE2[2:0] bits (APB2 prescaler) */
#define RCC_CFGR_HPRE_Pos (4U)
#define RCC_CFGR_HPRE_Msk (0xFUL << RCC_CFGR_HPRE_Pos) /*!< 0x000000F0 */
#define RCC_CFGR_HPRE RCC_CFGR_HPRE_Msk                /*!< HPRE[3:0] bits (AHB prescaler) */
#define RCC_SYSCLKSOURCE_HSE RCC_CFGR_SW_HSE /*!< HSE selected as system clock */
extern uint32_t uwTickPrio;

HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);


//
//
//
//Enable USB
//
//
//

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
  uint8_t *(*GetDeviceDescriptor)            (USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetLangIDStrDescriptor)         (USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetManufacturerStrDescriptor)   (USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetProductStrDescriptor)        (USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetSerialStrDescriptor)         (USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetConfigurationStrDescriptor)  (USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetInterfaceStrDescriptor)      (USBD_SpeedTypeDef speed, uint16_t *length);
} USBD_DescriptorsTypeDef;


typedef struct usb_setup_req
{
  uint8_t bmRequest;
  uint8_t bRequest;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
} USBD_SetupReqTypedef;

typedef struct
{
  uint32_t status;
  uint32_t is_used;
  uint32_t total_length;
  uint32_t rem_length;
  uint32_t maxpacket;
} USBD_EndpointTypeDef;


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


#define DEVICE_FS 0

USBD_HandleTypeDef hUsbDeviceFS;

extern USBD_DescriptorsTypeDef FS_Desc;

