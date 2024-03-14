typedef struct {
  uint32_t PLLState;
  uint32_t PLLSource;
  uint32_t PLLMUL;
} RCC_PLLInitTypeDef;
typedef struct {
  uint32_t OscillatorType;
  uint32_t HSEState;
  uint32_t HSEPredivValue;
  uint32_t LSEState;
  uint32_t HSIState;
  uint32_t HSICalibrationValue;
  uint32_t LSIState;
  RCC_PLLInitTypeDef PLL;
} RCC_OscInitTypeDef;
typedef struct {
  uint32_t PeriphClockSelection;
  uint32_t RTCClockSelection;
  uint32_t AdcClockSelection;
  uint32_t UsbClockSelection;
} RCC_PeriphCLKInitTypeDef;
typedef struct {
  uint32_t ClockType;
  uint32_t SYSCLKSource;
  uint32_t AHBCLKDivider;
  uint32_t APB1CLKDivider;
  uint32_t APB2CLKDivider;
} RCC_ClkInitTypeDef;
typedef enum {
  HAL_OK = 0x00U,
  HAL_ERROR = 0x01U,
  HAL_BUSY = 0x02U,
  HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;
typedef enum {
  HAL_UNLOCKED = 0x00U,
  HAL_LOCKED = 0x01U
} HAL_LockTypeDef;
typedef enum {
  USB_DEVICE_MODE = 0,
  USB_HOST_MODE = 1,
  USB_DRD_MODE = 2
} USB_ModeTypeDef;
typedef struct {
  uint32_t dev_endpoints;           /*!< Device Endpoints number.										*/
  uint32_t speed;                   /*!< USB Core speed.										*/
  uint32_t ep0_mps;                 /*!< Set the Endpoint 0 Max Packet size.                                    */
  uint32_t phy_itface;              /*!< Select the used PHY interface.							 */
  uint32_t Sof_enable;              /*!< Enable or disable the output of the SOF signal.                        */
  uint32_t low_power_enable;        /*!< Enable or disable the low Power Mode.                                  */
  uint32_t lpm_enable;              /*!< Enable or disable Link Power Management.                               */
  uint32_t battery_charging_enable; /*!< Enable or disable Battery charging.                                    */
} USB_CfgTypeDef;
typedef struct {
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
typedef struct {
  volatile uint16_t EP0R;          /*!< USB Endpoint 0 register,                   Address offset: 0x00 */
  volatile uint16_t RESERVED0;     /*!< Reserved */
  volatile uint16_t EP1R;          /*!< USB Endpoint 1 register,                   Address offset: 0x04 */
  volatile uint16_t RESERVED1;     /*!< Reserved */
  volatile uint16_t EP2R;          /*!< USB Endpoint 2 register,                   Address offset: 0x08 */
  volatile uint16_t RESERVED2;     /*!< Reserved */
  volatile uint16_t EP3R;          /*!< USB Endpoint 3 register,                   Address offset: 0x0C */
  volatile uint16_t RESERVED3;     /*!< Reserved */
  volatile uint16_t EP4R;          /*!< USB Endpoint 4 register,                   Address offset: 0x10 */
  volatile uint16_t RESERVED4;     /*!< Reserved */
  volatile uint16_t EP5R;          /*!< USB Endpoint 5 register,                   Address offset: 0x14 */
  volatile uint16_t RESERVED5;     /*!< Reserved */
  volatile uint16_t EP6R;          /*!< USB Endpoint 6 register,                   Address offset: 0x18 */
  volatile uint16_t RESERVED6;     /*!< Reserved */
  volatile uint16_t EP7R;          /*!< USB Endpoint 7 register,                   Address offset: 0x1C */
  volatile uint16_t RESERVED7[17]; /*!< Reserved */
  volatile uint16_t CNTR;          /*!< Control register,                          Address offset: 0x40 */
  volatile uint16_t RESERVED8;     /*!< Reserved */
  volatile uint16_t ISTR;          /*!< Interrupt status register,                 Address offset: 0x44 */
  volatile uint16_t RESERVED9;     /*!< Reserved */
  volatile uint16_t FNR;           /*!< Frame number register,                     Address offset: 0x48 */
  volatile uint16_t RESERVEDA;     /*!< Reserved */
  volatile uint16_t DADDR;         /*!< Device address register,                   Address offset: 0x4C */
  volatile uint16_t RESERVEDB;     /*!< Reserved */
  volatile uint16_t BTABLE;        /*!< Buffer Table address register,             Address offset: 0x50 */
  volatile uint16_t RESERVEDC;     /*!< Reserved */
} USB_TypeDef;
typedef enum {
  HAL_PCD_STATE_RESET = 0x00,
  HAL_PCD_STATE_READY = 0x01,
  HAL_PCD_STATE_ERROR = 0x02,
  HAL_PCD_STATE_BUSY = 0x03,
  HAL_PCD_STATE_TIMEOUT = 0x04
} PCD_StateTypeDef;
typedef enum {
  LPM_L0 = 0x00, /* on */
  LPM_L1 = 0x01, /* LPM L1 sleep */
  LPM_L2 = 0x02, /* suspend */
  LPM_L3 = 0x03, /* off */
} PCD_LPM_StateTypeDef;
typedef USB_TypeDef  PCD_TypeDef;
typedef USB_CfgTypeDef  PCD_InitTypeDef;
typedef USB_EPTypeDef  PCD_EPTypeDef;
typedef struct {
  PCD_TypeDef *Instance;          /*!< Register base address             */
  PCD_InitTypeDef Init;           /*!< PCD required parameters           */
  volatile uint8_t USB_Address;       /*!< USB Address                       */
  PCD_EPTypeDef IN_ep[8];         /*!< IN endpoint parameters            */
  PCD_EPTypeDef OUT_ep[8];        /*!< OUT endpoint parameters           */
  HAL_LockTypeDef Lock;           /*!< PCD peripheral status             */
  volatile PCD_StateTypeDef State;    /*!< PCD communication state           */
  volatile uint32_t ErrorCode;        /*!< PCD Error code                    */
  uint32_t Setup[12];             /*!< Setup packet buffer               */
  PCD_LPM_StateTypeDef LPM_State; /*!< LPM State                         */
  uint32_t BESL;
  uint32_t FrameNumber; /*!< Store Current Frame number        */
  void *pData;          /*!< Pointer to upper stack Handler    */
} PCD_HandleTypeDef;
typedef enum {
  HAL_TICK_FREQ_10HZ = 100U,
  HAL_TICK_FREQ_100HZ = 10U,
  HAL_TICK_FREQ_1KHZ = 1U,
  HAL_TICK_FREQ_DEFAULT = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;
typedef enum {
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
typedef struct {
  volatile uint32_t ISER[8U]; /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
  uint32_t RESERVED0[24U];
  volatile uint32_t ICER[8U]; /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
  uint32_t RSERVED1[24U];
  volatile uint32_t ISPR[8U]; /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
  uint32_t RESERVED2[24U];
  volatile uint32_t ICPR[8U]; /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
  uint32_t RESERVED3[24U];
  volatile uint32_t IABR[8U]; /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
  uint32_t RESERVED4[56U];
  volatile uint8_t IP[240U]; /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
  uint32_t RESERVED5[644U];
  volatile uint32_t STIR; /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
} NVIC_Type;
typedef struct {
  volatile const uint32_t CPUID;    /*!< Offset: 0x000 (R/ )  CPUID Base Register */
  volatile uint32_t ICSR;    /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
  volatile uint32_t VTOR;    /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
  volatile uint32_t AIRCR;   /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  volatile uint32_t SCR;     /*!< Offset: 0x010 (R/W)  System Control Register */
  volatile uint32_t CCR;     /*!< Offset: 0x014 (R/W)  Configuration Control Register */
  volatile uint8_t SHP[12U]; /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
  volatile uint32_t SHCSR;   /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
  volatile uint32_t CFSR;    /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
  volatile uint32_t HFSR;    /*!< Offset: 0x02C (R/W)  HardFault Status Register */
  volatile uint32_t DFSR;    /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
  volatile uint32_t MMFAR;   /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
  volatile uint32_t BFAR;    /*!< Offset: 0x038 (R/W)  BusFault Address Register */
  volatile uint32_t AFSR;    /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
  volatile const uint32_t PFR[2U];  /*!< Offset: 0x040 (R/ )  Processor Feature Register */
  volatile const uint32_t DFR;      /*!< Offset: 0x048 (R/ )  Debug Feature Register */
  volatile const uint32_t ADR;      /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
  volatile const uint32_t MMFR[4U]; /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
  volatile const uint32_t ISAR[5U]; /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
  uint32_t RESERVED0[5U];
  volatile uint32_t CPACR; /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_Type;
typedef struct {
  volatile uint32_t CTRL; /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
  volatile uint32_t LOAD; /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register */
  volatile uint32_t VAL;  /*!< Offset: 0x008 (R/W)  SysTick Current Value Register */
  volatile const uint32_t CALIB; /*!< Offset: 0x00C (R/ )  SysTick Calibration Register */
} SysTick_Type;

typedef struct {
  volatile uint32_t ACR;
  volatile uint32_t KEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t CR;
  volatile uint32_t AR;
  volatile uint32_t RESERVED;
  volatile uint32_t OBR;
  volatile uint32_t WRPR;
} FLASH_TypeDef;
typedef struct {
  volatile uint32_t CR;
  volatile uint32_t CSR;
} PWR_TypeDef;
typedef struct {
  volatile uint32_t CR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t AHBENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t APB1ENR;
  volatile uint32_t BDCR;
  volatile uint32_t CSR;
  } RCC_TypeDef;

typedef enum {
  RESET = 0,
  SET = !RESET
} FlagStatus,
    ITStatus;
    typedef enum {
  DISABLE = 0,
  ENABLE = !DISABLE
} FunctionalState;
typedef struct  usb_setup_re {
  uint8_t bmRequest;
  uint8_t bRequest;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
} USBD_SetupReqTypedef;
typedef struct  _Device_c {
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
typedef enum {
  USBD_SPEED_HIGH = 0U,
  USBD_SPEED_FULL = 1U,
  USBD_SPEED_LOW = 2U,
} USBD_SpeedTypeDef;
typedef enum {
  USBD_OK = 0U,
  USBD_BUSY,
  USBD_FAIL,
} USBD_StatusTypeDef;
typedef struct {
  uint8_t *(*GetDeviceDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetLangIDStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetManufacturerStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetProductStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetSerialStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetConfigurationStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t *(*GetInterfaceStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
} USBD_DescriptorsTypeDef;
typedef struct {
  uint32_t status;
  uint32_t is_used;
  uint32_t total_length;
  uint32_t rem_length;
  uint32_t maxpacket;
} USBD_EndpointTypeDef;
typedef struct  _USBD_HandleTypeDe {
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

typedef enum {
  HID_IDLE = 0,
  HID_BUSY,
} HID_StateTypeDef;
typedef struct {
  uint32_t Protocol;
  uint32_t IdleState;
  uint32_t AltSetting;
  HID_StateTypeDef state;
} USBD_HID_HandleTypeDef;
 typedef struct {
   uint8_t ModifierKey;
   uint8_t Reserved;
   uint8_t KeyCode1;
   uint8_t KeyCode2;
   uint8_t KeyCode3;
   uint8_t KeyCode4;
   uint8_t KeyCode5;
   uint8_t KeyCode6;
 } keyboardHID;
