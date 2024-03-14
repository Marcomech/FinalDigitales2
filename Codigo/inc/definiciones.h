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


typedef enum{
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


#define RCC_BDCR_RTCSEL_Pos (8U)
#define RCC_BDCR_RTCSEL_Msk (0x3UL << RCC_BDCR_RTCSEL_Pos) /*!< 0x00000300 */
#define RCC_BDCR_RTCSEL RCC_BDCR_RTCSEL_Msk 

#define RCC_HSE_PREDIV_DIV1 0x00000000U
#define RCC_PLL_MUL9 (0x7UL << 18U)
#define RCC_PERIPHCLK_USB 0x00000010U
#define RCC_PERIPHCLK_ADC 0x00000002U
#define RCC_PERIPHCLK_RTC 0x00000001U
#define RCC_USBCLKSOURCE_PLL_DIV1_5 0x00000000U

#define FLASH_LATENCY_0 0x00000000U /*!< FLASH Zero Latency cycle */
#define FLASH_LATENCY_1 0x00000001  /*!< FLASH One Latency cycle */
#define FLASH_LATENCY_2 0x00000002  /*!< FLASH Two Latency cycles */

#define RCC_CFGR_PPRE1_DIV4 0x00000500U  /*!< HCLK divided by 4 */
#define RCC_CFGR_PPRE1_DIV8 0x00000600U  /*!< HCLK divided by 8 */
#define RCC_CFGR_PPRE1_DIV16 0x00000700U /*!< HCLK divided by 16 */

#define RCC_FLAG_HSERDY ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_HSERDY_Pos)) /*!< External High Speed clock ready flag */

typedef enum{
  RESET = 0,
  SET = !RESET
} FlagStatus,
    ITStatus;

typedef enum{
  DISABLE = 0,
  ENABLE = !DISABLE
} FunctionalState;

typedef struct{
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

#define RCC_BDCR_LSEON_Pos (0U)
#define RCC_BDCR_LSEON_Msk (0x1UL << RCC_BDCR_LSEON_Pos) /*!< 0x00000001 */
#define RCC_BDCR_LSEON RCC_BDCR_LSEON_Msk 

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


#define __I volatile const  /*!< Defines 'read only' permissions */
#define __O volatile        /*!< Defines 'write only' permissions */
#define __IO volatile       /*!< Defines 'read / write' permissions */
#define __IM volatile const /*! Defines 'read only' structure member permissions */
#define __OM volatile       /*! Defines 'write only' structure member permissions */
#define __IOM volatile      /*! Defines 'read / write' structure member permissions */

typedef enum{
  USBD_SPEED_HIGH = 0U,
  USBD_SPEED_FULL = 1U,
  USBD_SPEED_LOW = 2U,
} USBD_SpeedTypeDef;

/* Following USB Device status */
typedef enum{
  USBD_OK = 0U,
  USBD_BUSY,
  USBD_FAIL,
} USBD_StatusTypeDef;

/* USB Device descriptors structure */
typedef struct{
  uint8_t *(*GetDeviceDescriptor)            (USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetLangIDStrDescriptor)         (USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetManufacturerStrDescriptor)   (USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetProductStrDescriptor)        (USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetSerialStrDescriptor)         (USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetConfigurationStrDescriptor)  (USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetInterfaceStrDescriptor)      (USBD_SpeedTypeDef speed, uint16_t *length);
} USBD_DescriptorsTypeDef;


typedef struct usb_setup_req{
  uint8_t bmRequest;
  uint8_t bRequest;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
} USBD_SetupReqTypedef;

typedef struct{
  uint32_t status;
  uint32_t is_used;
  uint32_t total_length;
  uint32_t rem_length;
  uint32_t maxpacket;
} USBD_EndpointTypeDef;


typedef struct _Device_cb{
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


typedef struct _USBD_HandleTypeDef{
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


typedef enum{
  HID_IDLE = 0,
  HID_BUSY,
} HID_StateTypeDef;

typedef struct{
  uint32_t Protocol;
  uint32_t IdleState;
  uint32_t AltSetting;
  HID_StateTypeDef state;
} USBD_HID_HandleTypeDef;

#define DEVICE_FS 0


extern USBD_DescriptorsTypeDef FS_Desc;


USBD_HandleTypeDef hUsbDeviceFS;

static uint8_t  USBD_HID_Init					(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_HID_DeInit					(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_HID_Setup					(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t  USBD_HID_DataIn					(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  *USBD_HID_GetFSCfgDesc			(uint16_t *length);
static uint8_t  *USBD_HID_GetHSCfgDesc			(uint16_t *length);
static uint8_t  *USBD_HID_GetOtherSpeedCfgDesc	(uint16_t *length);
static uint8_t  *USBD_HID_GetDeviceQualifierDesc(uint16_t *length);

USBD_ClassTypeDef  USBD_HID ={
  USBD_HID_Init,
  USBD_HID_DeInit,
  USBD_HID_Setup,
  NULL, /*EP0_TxSent*/
  NULL, /*EP0_RxReady*/
  USBD_HID_DataIn, /*DataIn*/
  NULL, /*DataOut*/
  NULL, /*SOF */
  NULL,
  NULL,
  USBD_HID_GetHSCfgDesc,
  USBD_HID_GetFSCfgDesc,
  USBD_HID_GetOtherSpeedCfgDesc,
  USBD_HID_GetDeviceQualifierDesc,
};

typedef enum
{
  HAL_UNLOCKED = 0x00U,
  HAL_LOCKED = 0x01U
} HAL_LockTypeDef;

typedef enum
{
  LPM_L0 = 0x00, /* on */
  LPM_L1 = 0x01, /* LPM L1 sleep */
  LPM_L2 = 0x02, /* suspend */
  LPM_L3 = 0x03, /* off */
} PCD_LPM_StateTypeDef;

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
  USB_DEVICE_MODE = 0,
  USB_HOST_MODE = 1,
  USB_DRD_MODE = 2
} USB_ModeTypeDef;


/*  Device Status */
#define USBD_STATE_DEFAULT 0x01U
#define USBD_STATE_ADDRESSED 0x02U
#define USBD_STATE_CONFIGURED 0x03U
#define USBD_STATE_SUSPENDED 0x04U

#define USB_LEN_DEV_DESC 0x12U

#define USB_DESC_TYPE_DEVICE 0x01U

#define USB_MAX_EP0_SIZE 64U
#define USBD_VID 1155
#define USBD_PID_FS 22315

#define USBD_IDX_MFC_STR 0x01U
#define USBD_IDX_PRODUCT_STR 0x02U
#define USBD_IDX_SERIAL_STR 0x03U
#define USBD_MAX_NUM_CONFIGURATION 1

#define HID_EPIN_ADDR 0x81U
#define HID_EPIN_SIZE 0x04U


#define EP_TYPE_CTRL 0U
#define EP_TYPE_BULK 2U

#define EP_ADDR_MSK 0x7U

#define USBD_EP_TYPE_INTR 0x03U

#define HID_REPORT_DESC 0x22U

#define HID_MOUSE_REPORT_DESC_SIZE 63U

#define HID_DESCRIPTOR_TYPE 0x21U

#define USB_REQ_GET_STATUS 0x00U
#define USB_REQ_GET_DESCRIPTOR 0x06U

#define USB_REQ_TYPE_STANDARD 0x00U
#define USB_REQ_TYPE_MASK 0x60U

#define USB_REQ_GET_INTERFACE 0x0AU
#define USB_REQ_SET_INTERFACE 0x0BU

#define USB_HID_DESC_SIZ 9U


#define __ALIGN_BEGIN
#define __ALIGN_END __attribute__((aligned(4)))

#define LOBYTE(x) ((uint8_t)((x) & 0x00FFU))
#define HIBYTE(x) ((uint8_t)(((x) & 0xFF00U) >> 8U))

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

#define USBD_EP0_DATA_IN 0x02U

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr);
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr);
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps);
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint16_t size);

#define USBD_FS_SPEED 2U
#define PCD_SPEED_FULL USBD_FS_SPEED


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
  uint32_t dev_endpoints;           /*!< Device Endpoints number.										*/
  uint32_t speed;                   /*!< USB Core speed.										*/
  uint32_t ep0_mps;                 /*!< Set the Endpoint 0 Max Packet size.                                    */
  uint32_t phy_itface;              /*!< Select the used PHY interface.							 */
  uint32_t Sof_enable;              /*!< Enable or disable the output of the SOF signal.                        */
  uint32_t low_power_enable;        /*!< Enable or disable the low Power Mode.                                  */
  uint32_t lpm_enable;              /*!< Enable or disable Link Power Management.                               */
  uint32_t battery_charging_enable; /*!< Enable or disable Battery charging.                                    */
} USB_CfgTypeDef;


typedef USB_TypeDef PCD_TypeDef;
typedef USB_CfgTypeDef PCD_InitTypeDef;
typedef USB_EPTypeDef PCD_EPTypeDef;

#define USB_BASE (APB1PERIPH_BASE + 0x00005C00UL)    /*!< USB_IP Peripheral Registers base address */
#define USB ((USB_TypeDef *)USB_BASE)

#define PCD_SNG_BUF 0U

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




#define USBD_malloc (uint32_t *)USBD_static_malloc


#define USBD_LANGID_STRING 1033
#define USBD_MANUFACTURER_STRING "STMicroelectronics"
#define USBD_PID_FS 22315
#define USBD_PRODUCT_STRING_FS "STM32 Human interface"
#define USBD_CONFIGURATION_STRING_FS "HID Config"
#define USBD_INTERFACE_STRING_FS "HID Interface"

#define USB_LEN_LANGID_STR_DESC 0x04U
#define USB_DESC_TYPE_STRING 0x03U
#define USB_SIZ_STRING_SERIAL 0x1A
#define USB_HID_CONFIG_DESC_SIZ 34U
#define USB_DESC_TYPE_CONFIGURATION 0x02U
#define USB_DESC_TYPE_INTERFACE 0x04U
#define USB_DESC_TYPE_ENDPOINT 0x05U
#define HID_FS_BINTERVAL 0xA
#define USB_LEN_DEV_QUALIFIER_DESC 0x0AU
#define USB_DESC_TYPE_DEVICE_QUALIFIER 0x06U


#define USBD_MAX_STR_DESC_SIZ 512

#define HID_HS_BINTERVAL 0x07U





#define assert_param(expr) ((void)0U)

#define IS_PCD_ALL_INSTANCE(INSTANCE) ((INSTANCE) == USB)

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

#define __HAL_PCD_ENABLE(__HANDLE__) (void)USB_EnableGlobalInt((__HANDLE__)->Instance)
#define __HAL_PCD_DISABLE(__HANDLE__) (void)USB_DisableGlobalInt((__HANDLE__)->Instance)


#define USB_CNTR_FRES (0x1UL << (0U))        /*!< Force USB Reset */
#define BTABLE_ADDRESS 0x000U


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

#define USB_EPREG_MASK (USB_EP_CTR_RX | USB_EP_SETUP | USB_EP_T_FIELD | USB_EP_KIND | USB_EP_CTR_TX | USB_EPADDR_FIELD)

#define USB_EP_T_MASK (~USB_EP_T_FIELD & USB_EPREG_MASK)

#define USB_EP_CONTROL 0x00000200U       /*!< EndPoint CONTROL */
#define USB_EP_BULK 0x00000000U          /*!< EndPoint BULK */


#define USBD_EP_TYPE_INTR 0x03U

#define USB_EP_INTERRUPT 0x00000600U     /*!< EndPoint INTERRUPT */


#define USBD_EP_TYPE_ISOC 0x01U


#define EP_TYPE_CTRL 0U
#define EP_TYPE_ISOC 1U
#define EP_TYPE_BULK 2U
#define EP_TYPE_INTR 3U
#define EP_TYPE_MSK 3U

#define USB_EP_ISOCHRONOUS 0x00000400U   /*!< EndPoint ISOCHRONOUS */

#define USB_EP_RX_DIS 0x00000000U   /*!< EndPoint RX DISabled */
#define USB_EP_RX_STALL 0x00001000U /*!< EndPoint RX STALLed */
#define USB_EP_RX_NAK 0x00002000U   /*!< EndPoint RX NAKed */
#define USB_EP_RX_VALID 0x00003000U /*!< EndPoint RX VALID */

#define USB_EP_TX_DIS 0x00000000U   /*!< EndPoint TX DISabled */
#define USB_EP_TX_STALL 0x00000010U /*!< EndPoint TX STALLed */
#define USB_EP_TX_NAK 0x00000020U   /*!< EndPoint TX NAKed */
#define USB_EP_TX_VALID 0x00000030U /*!< EndPoint TX VALID */


#define USB_EP_CTR_RX (0x1UL << (15U))   /*!< EndPoint Correct TRansfer RX */
#define USB_EP_CTR_TX (0x1UL << (7U))    /*!< EndPoint Correct TRansfer TX */


#define UID_BASE 0x1FFFF7E8UL /*!< Unique device ID register base address */

#define DEVICE_ID1 (UID_BASE)
#define DEVICE_ID2 (UID_BASE + 0x4)
#define DEVICE_ID3 (UID_BASE + 0x8)

#define __HAL_RCC_USB_CLK_ENABLE()                                                               \
  do                                                                                             \
  {                                                                                              \
    __IO uint32_t tmpreg;                                                                        \
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USBEN); /* Delay after an RCC peripheral clock enabling */ \
    tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_USBEN);                                          \
    UNUSED(tmpreg);                                                                              \
  } while (0U)


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

#define PCD_SET_EP_RX_CNT(USBx, bEpNum, wCount)                                                         \
  do                                                                                                    \
  {                                                                                                     \
    uint32_t _wRegBase = (uint32_t)(USBx);                                                              \
    __IO uint16_t *_wRegVal;                                                                            \
    _wRegBase += (uint32_t)(USBx)->BTABLE;                                                              \
    _wRegVal = (__IO uint16_t *)(_wRegBase + 0x400U + ((((uint32_t)(bEpNum) * 8U) + 6U) * PMA_ACCESS)); \
    PCD_SET_EP_CNT_RX_REG(_wRegVal, (wCount));                                                          \
  } while (0)
  
#define PCD_SET_EP_RX_ADDRESS(USBx, bEpNum, wAddr)                                                      \
  do                                                                                                    \
  {                                                                                                     \
    __IO uint16_t *_wRegVal;                                                                            \
    uint32_t _wRegBase = (uint32_t)USBx;                                                                \
    _wRegBase += (uint32_t)(USBx)->BTABLE;                                                              \
    _wRegVal = (__IO uint16_t *)(_wRegBase + 0x400U + ((((uint32_t)(bEpNum) * 8U) + 4U) * PMA_ACCESS)); \
    *_wRegVal = ((wAddr) >> 1) << 1;                                                                    \
  } while (0) /* PCD_SET_EP_RX_ADDRESS */

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
  
#define PCD_SET_EP_TX_ADDRESS(USBx, bEpNum, wAddr)                                               \
  do                                                                                             \
  {                                                                                              \
    __IO uint16_t *_wRegVal;                                                                     \
    uint32_t _wRegBase = (uint32_t)USBx;                                                         \
    _wRegBase += (uint32_t)(USBx)->BTABLE;                                                       \
    _wRegVal = (__IO uint16_t *)(_wRegBase + 0x400U + (((uint32_t)(bEpNum) * 8U) * PMA_ACCESS)); \
    *_wRegVal = ((wAddr) >> 1) << 1;                                                             \
  } while (0) /* PCD_SET_EP_TX_ADDRESS */
  
#define PCD_SET_EP_ADDRESS(USBx, bEpNum, bAddr)                                     \
  do                                                                                \
  {                                                                                 \
    uint16_t _wRegVal;                                                              \
    _wRegVal = (PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPREG_MASK) | (bAddr);     \
    PCD_SET_ENDPOINT((USBx), (bEpNum), (_wRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX)); \
  } while (0) /* PCD_SET_EP_ADDRESS */
  
#define PCD_TX_DTOG(USBx, bEpNum)                                                                   \
  do                                                                                                \
  {                                                                                                 \
    uint16_t _wEPVal;                                                                               \
    _wEPVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPREG_MASK;                                  \
    PCD_SET_ENDPOINT((USBx), (bEpNum), (_wEPVal | USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_DTOG_TX)); \
  } while (0) /* PCD_TX_DTOG */

  
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

  
#define PCD_RX_DTOG(USBx, bEpNum)                                                                   \
  do                                                                                                \
  {                                                                                                 \
    uint16_t _wEPVal;                                                                               \
    _wEPVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPREG_MASK;                                  \
    PCD_SET_ENDPOINT((USBx), (bEpNum), (_wEPVal | USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_DTOG_RX)); \
  } while (0) /* PCD_RX_DTOG */

#define PCD_SET_ENDPOINT(USBx, bEpNum, wRegValue) (*(__IO uint16_t *)(&(USBx)->EP0R + ((bEpNum) * 2U)) = (uint16_t)(wRegValue))

#define PCD_GET_ENDPOINT(USBx, bEpNum) (*(__IO uint16_t *)(&(USBx)->EP0R + ((bEpNum) * 2U)))



HAL_StatusTypeDef USB_EPStartXfer(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EnableGlobalInt(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_DisableGlobalInt(USB_TypeDef *USBx);

#define RCC_APB1ENR_USBEN  (0x1UL << 23U)                /*!< USB Device clock enable */

#define USB_EPTX_DTOG1 0x00000010U  /*!< EndPoint TX Data TOGgle bit1 */
#define USB_EPTX_DTOG2 0x00000020U  /*!< EndPoint TX Data TOGgle bit2 */
#define USB_EPTX_DTOGMASK (USB_EPTX_STAT | USB_EPREG_MASK)

#define USB_EPRX_DTOG1 0x00001000U  /*!< EndPoint RX Data TOGgle bit1 */
#define USB_EPRX_DTOG2 0x00002000U  /*!< EndPoint RX Data TOGgle bit1 */
#define USB_EPRX_DTOGMASK (USB_EPRX_STAT | USB_EPREG_MASK)

#define PMA_ACCESS 2U

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


#define SET_BIT(REG, BIT) ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))

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

#define USB_CNTRX_BLSIZE (0x1U << 15)

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


#define SCS_BASE (0xE000E000UL)            /*!< System Control Space Base Address */
#define SCB_BASE (SCS_BASE + 0x0D00UL)     /*!< System Control Block Base Address */

#define SCB ((SCB_Type *)SCB_BASE)                   /*!< SCB configuration struct */
#define SCB_AIRCR_PRIGROUP_Pos 8U                              /*!< SCB AIRCR: PRIGROUP Position */
#define SCB_AIRCR_PRIGROUP_Msk (7UL << SCB_AIRCR_PRIGROUP_Pos) /*!< SCB AIRCR: PRIGROUP Mask */

#define __NVIC_PRIO_BITS 4U       /*!< STM32 uses 4 Bits for the Priority Levels    */
#define NVIC_BASE (SCS_BASE + 0x0100UL)    /*!< NVIC Base Address */

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

#define NVIC ((NVIC_Type *)NVIC_BASE)                /*!< NVIC configuration struct */


static inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) >> SCB_AIRCR_PRIGROUP_Pos));
}


#define SCB_AIRCR_VECTKEY_Pos 16U                                 /*!< SCB AIRCR: VECTKEY Position */
#define SCB_AIRCR_VECTKEY_Msk (0xFFFFUL << SCB_AIRCR_VECTKEY_Pos) /*!< SCB AIRCR: VECTKEY Mask */
static inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
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


static inline uint32_t NVIC_EncodePriority(uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
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

static inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}


static inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
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




#define PCD_SET_EP_TX_CNT(USBx, bEpNum, wCount)                                                         \
  do                                                                                                    \
  {                                                                                                     \
    uint32_t _wRegBase = (uint32_t)(USBx);                                                              \
    __IO uint16_t *_wRegVal;                                                                            \
    _wRegBase += (uint32_t)(USBx)->BTABLE;                                                              \
    _wRegVal = (__IO uint16_t *)(_wRegBase + 0x400U + ((((uint32_t)(bEpNum) * 8U) + 2U) * PMA_ACCESS)); \
    *_wRegVal = (uint16_t)(wCount);                                                                     \
  } while (0)


#define READ_BIT(REG, BIT) ((REG) & (BIT))
