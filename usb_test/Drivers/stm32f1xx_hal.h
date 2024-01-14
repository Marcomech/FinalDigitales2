#ifndef __STM32F1xx_HAL_H
#define __STM32F1xx_HAL_H

#define HAL_MODULE_ENABLED
#define HAL_PCD_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED

#define HSE_VALUE    			8000000U 	/*!< Value of the External oscillator in Hz */
#define HSE_STARTUP_TIMEOUT    	100U   		/*!< Time out for HSE start up, in ms */
#define HSI_VALUE    			8000000U 	/*!< Value of the Internal oscillator in Hz*/
#define LSI_VALUE               40000U    	/*!< LSI Typical Value in Hz */
#define LSE_VALUE    			32768U 		/*!< Value of the External oscillator in Hz*/
#define LSE_STARTUP_TIMEOUT    	5000U   	/*!< Time out for LSE start up, in ms */
#define TICK_INT_PRIORITY 		0U    		/*!< tick interrupt priority (lowest by default)  */

#define NVIC_PRIORITYGROUP_0         0x00000007U
#define NVIC_PRIORITYGROUP_1         0x00000006U
#define NVIC_PRIORITYGROUP_2         0x00000005U
#define NVIC_PRIORITYGROUP_3         0x00000004U
#define NVIC_PRIORITYGROUP_4         0x00000003U

#define __HAL_FLASH_SET_LATENCY(__LATENCY__)    	(FLASH->ACR = (FLASH->ACR&(~FLASH_ACR_LATENCY)) | (__LATENCY__))
#define __HAL_FLASH_GET_LATENCY()     				(READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY))
#define __HAL_FLASH_PREFETCH_BUFFER_ENABLE()    	(FLASH->ACR |= FLASH_ACR_PRFTBE)

#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32f1xx.h"
#include <stddef.h>

#include "../Core/Src/Inc_Clock_Config.h"

typedef enum{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

typedef enum{
  HAL_UNLOCKED = 0x00U,
  HAL_LOCKED   = 0x01U
} HAL_LockTypeDef;

#define HAL_MAX_DELAY      0xFFFFFFFFU
#define HAL_IS_BIT_SET(REG, BIT)         (((REG) & (BIT)) != 0U)
#define HAL_IS_BIT_CLR(REG, BIT)         (((REG) & (BIT)) == 0U)

#define UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */

#define __HAL_LOCK(__HANDLE__)                                           \
                                do{                                        \
                                    if((__HANDLE__)->Lock == HAL_LOCKED)   \
                                    {                                      \
                                       return HAL_BUSY;                    \
                                    }                                      \
                                    else                                   \
                                    {                                      \
                                       (__HANDLE__)->Lock = HAL_LOCKED;    \
                                    }                                      \
                                  }while (0U)

#define __HAL_UNLOCK(__HANDLE__)                                          \
                                  do{                                       \
                                      (__HANDLE__)->Lock = HAL_UNLOCKED;    \
                                    }while (0U)
#define __weak   __attribute__((weak))
#define __packed __attribute__((__packed__))
#define __ALIGN_BEGIN
#define __ALIGN_END      __attribute__ ((aligned (4)))
#define RCC_PLLSOURCE_HSI_DIV2      0x00000000U     /*!< HSI clock divided by 2 selected as PLL entry clock source */

#define RCC_OSCILLATORTYPE_HSI             0x00000002U
#define RCC_OSCILLATORTYPE_LSE             0x00000004U
#define RCC_OSCILLATORTYPE_LSI             0x00000008U
#define RCC_HSE_OFF                      0x00000000U                                /*!< HSE clock deactivation */
#define RCC_HSE_BYPASS                   ((uint32_t)(RCC_CR_HSEBYP | RCC_CR_HSEON)) /*!< External clock source for HSE clock */
#define RCC_LSE_OFF                      0x00000000U                                    /*!< LSE clock deactivation */
#define RCC_LSE_ON                       RCC_BDCR_LSEON                                 /*!< LSE clock activation */
#define RCC_LSE_BYPASS                   ((uint32_t)(RCC_BDCR_LSEBYP | RCC_BDCR_LSEON)) /*!< External clock source for LSE clock */
#define RCC_HSI_OFF                      0x00000000U                      /*!< HSI clock deactivation */

#define RCC_LSI_OFF                      0x00000000U              /*!< LSI clock deactivation */

#define RCC_PLL_NONE                      0x00000000U  /*!< PLL is not configured */
#define RCC_PLL_OFF                       0x00000001U  /*!< PLL deactivation */

#define RCC_SYSCLKSOURCE_HSE             RCC_CFGR_SW_HSE /*!< HSE selected as system clock */

#define RCC_SYSCLKSOURCE_STATUS_HSI      RCC_CFGR_SWS_HSI            /*!< HSI used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_HSE      RCC_CFGR_SWS_HSE            /*!< HSE used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_PLLCLK   RCC_CFGR_SWS_PLL            /*!< PLL used as system clock */
#define RCC_HCLK_DIV16                   RCC_CFGR_PPRE1_DIV16 /*!< HCLK divided by 16 */
#define RCC_FLAG_HSIRDY                  ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_HSIRDY_Pos)) /*!< Internal High Speed clock ready flag */
#define RCC_FLAG_HSERDY                  ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_HSERDY_Pos)) /*!< External High Speed clock ready flag */
#define RCC_FLAG_PLLRDY                  ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_PLLRDY_Pos)) /*!< PLL clock ready flag */

#define RCC_FLAG_LSIRDY                  ((uint8_t)((CSR_REG_INDEX << 5U) | RCC_CSR_LSIRDY_Pos))   /*!< Internal Low Speed oscillator Ready */

#define RCC_FLAG_LSERDY                  ((uint8_t)((BDCR_REG_INDEX << 5U) | RCC_BDCR_LSERDY_Pos)) /*!< External Low Speed oscillator Ready */

#define __HAL_RCC_PWR_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_PWR_CLK_DISABLE()       (RCC->APB1ENR &= ~(RCC_APB1ENR_PWREN))

#define __HAL_RCC_PWR_IS_CLK_DISABLED()       ((RCC->APB1ENR & (RCC_APB1ENR_PWREN)) == RESET)
#define __HAL_RCC_HSI_ENABLE()  (*(__IO uint32_t *) RCC_CR_HSION_BB = ENABLE)
#define __HAL_RCC_HSI_DISABLE() (*(__IO uint32_t *) RCC_CR_HSION_BB = DISABLE)

#define __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(_HSICALIBRATIONVALUE_) \
          (MODIFY_REG(RCC->CR, RCC_CR_HSITRIM, (uint32_t)(_HSICALIBRATIONVALUE_) << RCC_CR_HSITRIM_Pos))

#define __HAL_RCC_LSI_ENABLE()  (*(__IO uint32_t *) RCC_CSR_LSION_BB = ENABLE)

#define __HAL_RCC_LSI_DISABLE() (*(__IO uint32_t *) RCC_CSR_LSION_BB = DISABLE)

#define __HAL_RCC_HSE_CONFIG(__STATE__)                                     \
                    do{                                                     \
                      if ((__STATE__) == RCC_HSE_ON)                        \
                      {                                                     \
                        SET_BIT(RCC->CR, RCC_CR_HSEON);                     \
                      }                                                     \
                      else if ((__STATE__) == RCC_HSE_OFF)                  \
                      {                                                     \
                        CLEAR_BIT(RCC->CR, RCC_CR_HSEON);                   \
                        CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);                  \
                      }                                                     \
                      else if ((__STATE__) == RCC_HSE_BYPASS)               \
                      {                                                     \
                        SET_BIT(RCC->CR, RCC_CR_HSEBYP);                    \
                        SET_BIT(RCC->CR, RCC_CR_HSEON);                     \
                      }                                                     \
                      else                                                  \
                      {                                                     \
                        CLEAR_BIT(RCC->CR, RCC_CR_HSEON);                   \
                        CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);                  \
                      }                                                     \
                    }while(0U)

#define __HAL_RCC_LSE_CONFIG(__STATE__)                                     \
                    do{                                                     \
                      if ((__STATE__) == RCC_LSE_ON)                        \
                      {                                                     \
                        SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);                   \
                      }                                                     \
                      else if ((__STATE__) == RCC_LSE_OFF)                  \
                      {                                                     \
                        CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEON);                 \
                        CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEBYP);                \
                      }                                                     \
                      else if ((__STATE__) == RCC_LSE_BYPASS)               \
                      {                                                     \
                        SET_BIT(RCC->BDCR, RCC_BDCR_LSEBYP);                  \
                        SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);                   \
                      }                                                     \
                      else                                                  \
                      {                                                     \
                        CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEON);                 \
                        CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEBYP);                \
                      }                                                     \
                    }while(0U)

#define __HAL_RCC_PLL_ENABLE()          (*(__IO uint32_t *) RCC_CR_PLLON_BB = ENABLE)

#define __HAL_RCC_PLL_DISABLE()         (*(__IO uint32_t *) RCC_CR_PLLON_BB = DISABLE)

#define __HAL_RCC_PLL_CONFIG(__RCC_PLLSOURCE__, __PLLMUL__)\
          MODIFY_REG(RCC->CFGR, (RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL),((__RCC_PLLSOURCE__) | (__PLLMUL__) ))

#define __HAL_RCC_GET_PLL_OSCSOURCE() ((uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_PLLSRC)))

#define __HAL_RCC_SYSCLK_CONFIG(__SYSCLKSOURCE__) \
                  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, (__SYSCLKSOURCE__))

#define __HAL_RCC_GET_SYSCLK_SOURCE() ((uint32_t)(READ_BIT(RCC->CFGR,RCC_CFGR_SWS)))

#define __HAL_RCC_RTC_CONFIG(__RTC_CLKSOURCE__) MODIFY_REG(RCC->BDCR, RCC_BDCR_RTCSEL, (__RTC_CLKSOURCE__))
#define __HAL_RCC_BACKUPRESET_FORCE()   (*(__IO uint32_t *) RCC_BDCR_BDRST_BB = ENABLE)

#define __HAL_RCC_BACKUPRESET_RELEASE() (*(__IO uint32_t *) RCC_BDCR_BDRST_BB = DISABLE)
#define __HAL_RCC_GET_FLAG(__FLAG__) (((((__FLAG__) >> 5U) == CR_REG_INDEX)?   RCC->CR   : \
                                      ((((__FLAG__) >> 5U) == BDCR_REG_INDEX)? RCC->BDCR : \
                                                                              RCC->CSR)) & (1U << ((__FLAG__) & RCC_FLAG_MASK)))
                                                                              #define RCC_PERIPHCLK_RTC           0x00000001U
#define RCC_PERIPHCLK_ADC           0x00000002U
#define __HAL_RCC_USB_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USBEN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_USBEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_HSE_PREDIV_CONFIG(__HSE_PREDIV_VALUE__) \
                  MODIFY_REG(RCC->CFGR,RCC_CFGR_PLLXTPRE, (uint32_t)(__HSE_PREDIV_VALUE__))
#define __HAL_RCC_USB_CONFIG(__USBCLKSOURCE__) MODIFY_REG(RCC->CFGR, RCC_CFGR_USBPRE, (uint32_t)(__USBCLKSOURCE__))
#define __HAL_RCC_ADC_CONFIG(__ADCCLKSOURCE__) MODIFY_REG(RCC->CFGR, RCC_CFGR_ADCPRE, (uint32_t)(__ADCCLKSOURCE__))

HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct);

uint32_t          HAL_RCC_GetSysClockFreq(void);
void              HAL_RCC_GetOscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct);
#define RCC_DBP_TIMEOUT_VALUE          100U    /* 100 ms */
#define RCC_LSE_TIMEOUT_VALUE          LSE_STARTUP_TIMEOUT
#define CLOCKSWITCH_TIMEOUT_VALUE      5000    /* 5 s    */
#define HSE_TIMEOUT_VALUE              HSE_STARTUP_TIMEOUT
#define HSI_TIMEOUT_VALUE              2U      /* 2 ms (minimum Tick + 1) */
#define LSI_TIMEOUT_VALUE              2U      /* 2 ms (minimum Tick + 1) */
#define PLL_TIMEOUT_VALUE              2U      /* 2 ms (minimum Tick + 1) */

#define RCC_OFFSET                (RCC_BASE - PERIPH_BASE)
#define RCC_CR_OFFSET             0x00U

#define RCC_BDCR_OFFSET           0x20U
#define RCC_CSR_OFFSET            0x24U

#define RCC_CR_OFFSET_BB          (RCC_OFFSET + RCC_CR_OFFSET)

#define RCC_BDCR_OFFSET_BB        (RCC_OFFSET + RCC_BDCR_OFFSET)
#define RCC_CSR_OFFSET_BB         (RCC_OFFSET + RCC_CSR_OFFSET)
#define RCC_HSION_BIT_NUMBER      RCC_CR_HSION_Pos
#define RCC_CR_HSION_BB           ((uint32_t)(PERIPH_BB_BASE + (RCC_CR_OFFSET_BB * 32U) + (RCC_HSION_BIT_NUMBER * 4U)))
#define RCC_PLLON_BIT_NUMBER      RCC_CR_PLLON_Pos
#define RCC_CR_PLLON_BB           ((uint32_t)(PERIPH_BB_BASE + (RCC_CR_OFFSET_BB * 32U) + (RCC_PLLON_BIT_NUMBER * 4U)))
#define RCC_LSION_BIT_NUMBER      RCC_CSR_LSION_Pos
#define RCC_CSR_LSION_BB          ((uint32_t)(PERIPH_BB_BASE + (RCC_CSR_OFFSET_BB * 32U) + (RCC_LSION_BIT_NUMBER * 4U)))
#define RCC_BDRST_BIT_NUMBER      RCC_BDCR_BDRST_Pos
#define RCC_BDCR_BDRST_BB         ((uint32_t)(PERIPH_BB_BASE + (RCC_BDCR_OFFSET_BB * 32U) + (RCC_BDRST_BIT_NUMBER * 4U)))
#define CR_REG_INDEX                     ((uint8_t)1)
#define BDCR_REG_INDEX                   ((uint8_t)2)
#define CSR_REG_INDEX                    ((uint8_t)3)
#define RCC_FLAG_MASK                    ((uint8_t)0x1F)

#endif /* HAL_RCC_MODULE_ENABLED */

#ifdef HAL_PCD_MODULE_ENABLED
#ifndef STM32F1xx_HAL_PCD_H
#define STM32F1xx_HAL_PCD_H

typedef enum{
  USB_DEVICE_MODE = 0,
  USB_HOST_MODE   = 1,
  USB_DRD_MODE    = 2
} USB_ModeTypeDef;


typedef enum{
  URB_IDLE = 0,
  URB_DONE,
  URB_NOTREADY,
  URB_NYET,
  URB_ERROR,
  URB_STALL
} USB_URBStateTypeDef;


typedef enum{
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

typedef struct{
  uint32_t dev_endpoints;           /*!< Device Endpoints number.										*/
  uint32_t speed;                   /*!< USB Core speed.										*/
  uint32_t ep0_mps;                 /*!< Set the Endpoint 0 Max Packet size.                                    */
  uint32_t phy_itface;              /*!< Select the used PHY interface.							 */
  uint32_t Sof_enable;              /*!< Enable or disable the output of the SOF signal.                        */
  uint32_t low_power_enable;        /*!< Enable or disable the low Power Mode.                                  */
  uint32_t lpm_enable;              /*!< Enable or disable Link Power Management.                               */
  uint32_t battery_charging_enable; /*!< Enable or disable Battery charging.                                    */
} USB_CfgTypeDef;

typedef struct{
  uint8_t   num;                  /*!< Endpoint number */
  uint8_t   is_in;                /*!< Endpoint direction  */
  uint8_t   is_stall;             /*!< Endpoint stall condition   */
  uint8_t   type;                 /*!< Endpoint type     */
  uint8_t   data_pid_start;       /*!< Initial data PID    */
  uint16_t  pmaadress;            /*!< PMA Address */
  uint16_t  pmaaddr0;             /*!< PMA Address0   */
  uint16_t  pmaaddr1;             /*!< PMA Address1 */
  uint8_t   doublebuffer;         /*!< Double buffer enable                                      */
  uint32_t  maxpacket;            /*!< Endpoint Max packet size */
  uint8_t   *xfer_buff;           /*!< Pointer to transfer buffer                                               */
  uint32_t  xfer_len;             /*!< Current transfer length                                                  */
  uint32_t  xfer_count;           /*!< Partial transfer length in case of multi packet transfer                 */
  uint32_t  xfer_len_db;          /*!< double buffer transfer length used with bulk double buffer in            */
  uint8_t   xfer_fill_db;         /*!< double buffer Need to Fill new buffer  used with bulk_in                 */
} USB_EPTypeDef;

typedef struct{
  uint8_t   dev_addr;           /*!< USB device address.  */
  uint8_t   ch_num;             /*!< Host channel number.   */
  uint8_t   ep_num;             /*!< Endpoint number.    */
  uint8_t   ep_is_in;           /*!< Endpoint direction    */
  uint8_t   speed;              /*!< USB Host Channel speed.           */
  uint8_t   do_ping;            /*!< Enable or disable the use of the PING protocol for HS mode.                */
  uint8_t   hub_port_nbr;       /*!< USB HUB port number                                                        */
  uint8_t   hub_addr;           /*!< USB HUB address                                                            */
  uint8_t   ep_type;            /*!< Endpoint Type.                   */
  uint16_t  max_packet;         /*!< Endpoint Max packet size.  */
  uint8_t   data_pid;           /*!< Initial data PID. */
  uint8_t   *xfer_buff;         /*!< Pointer to transfer buffer.                                                */
  uint32_t  XferSize;           /*!< OTG Channel transfer size.                                                 */
  uint32_t  xfer_len;           /*!< Current transfer length.                                                   */
  uint32_t  xfer_count;         /*!< Partial transfer length in case of multi packet transfer.                  */
  uint8_t   toggle_in;          /*!< IN transfer current toggle flag.   */
  uint8_t   toggle_out;         /*!< OUT transfer current toggle flag    */
  uint32_t  dma_addr;           /*!< 32 bits aligned transfer buffer address.                                   */
  uint32_t  ErrCnt;             /*!< Host channel error count.                                                  */
  USB_URBStateTypeDef urb_state;  /*!< URB state.            */
  USB_HCStateTypeDef state;       /*!< Host Channel state.          */
} USB_HCTypeDef;

#define EP_MPS_64                              0U
#define EP_MPS_32                              1U
#define EP_MPS_16                              2U
#define EP_MPS_8                               3U

#define EP_TYPE_CTRL                           0U
#define EP_TYPE_ISOC                           1U
#define EP_TYPE_BULK                           2U
#define EP_TYPE_INTR                           3U
#define EP_TYPE_MSK                            3U

#define EP_SPEED_LOW                           0U
#define EP_SPEED_FULL                          1U
#define EP_SPEED_HIGH                          2U

#define HC_PID_DATA0                           0U
#define HC_PID_DATA2                           1U
#define HC_PID_DATA1                           2U
#define HC_PID_SETUP                           3U

#define USBD_FS_SPEED                          2U
#define USBH_FSLS_SPEED                        1U

#define BTABLE_ADDRESS                         0x000U
#define PMA_ACCESS                             2U

#define USB_EP_RX_STRX                         (0x3U << 12)

#define EP_ADDR_MSK                            0x7U

#define USE_USB_DOUBLE_BUFFER                  1U

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

void             *USB_ReadPacket(USB_TypeDef *USBx, uint8_t *dest, uint16_t len);

uint32_t          USB_ReadInterrupts(USB_TypeDef const *USBx);
uint32_t          USB_ReadDevAllOutEpInterrupt(USB_TypeDef *USBx);
uint32_t          USB_ReadDevOutEPInterrupt(USB_TypeDef *USBx, uint8_t epnum);
uint32_t          USB_ReadDevAllInEpInterrupt(USB_TypeDef *USBx);
uint32_t          USB_ReadDevInEPInterrupt(USB_TypeDef *USBx, uint8_t epnum);
void              USB_ClearInterrupts(USB_TypeDef *USBx, uint32_t interrupt);
HAL_StatusTypeDef USB_ActivateRemoteWakeup(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_DeActivateRemoteWakeup(USB_TypeDef *USBx);

void              USB_WritePMA(USB_TypeDef const *USBx, uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes);
void              USB_ReadPMA (USB_TypeDef const *USBx, uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes);



typedef enum{
  HAL_PCD_STATE_RESET   = 0x00,
  HAL_PCD_STATE_READY   = 0x01,
  HAL_PCD_STATE_ERROR   = 0x02,
  HAL_PCD_STATE_BUSY    = 0x03,
  HAL_PCD_STATE_TIMEOUT = 0x04
} PCD_StateTypeDef;

typedef enum{
  LPM_L0 = 0x00, /* on */
  LPM_L1 = 0x01, /* LPM L1 sleep */
  LPM_L2 = 0x02, /* suspend */
  LPM_L3 = 0x03, /* off */
} PCD_LPM_StateTypeDef;

typedef USB_TypeDef        PCD_TypeDef;
typedef USB_CfgTypeDef     PCD_InitTypeDef;
typedef USB_EPTypeDef      PCD_EPTypeDef;

typedef struct{
  PCD_TypeDef             *Instance;   /*!< Register base address             */
  PCD_InitTypeDef         Init;        /*!< PCD required parameters           */
  __IO uint8_t            USB_Address; /*!< USB Address                       */
  PCD_EPTypeDef           IN_ep[8];    /*!< IN endpoint parameters            */
  PCD_EPTypeDef           OUT_ep[8];   /*!< OUT endpoint parameters           */
  HAL_LockTypeDef         Lock;        /*!< PCD peripheral status             */
  __IO PCD_StateTypeDef   State;       /*!< PCD communication state           */
  __IO  uint32_t          ErrorCode;   /*!< PCD Error code                    */
  uint32_t                Setup[12];   /*!< Setup packet buffer               */
  PCD_LPM_StateTypeDef    LPM_State;   /*!< LPM State                         */
  uint32_t                BESL;
  uint32_t                FrameNumber; /*!< Store Current Frame number        */
  void                    *pData;      /*!< Pointer to upper stack Handler    */
} PCD_HandleTypeDef;

#define PCD_SPEED_FULL               USBD_FS_SPEED

#define __HAL_PCD_ENABLE(__HANDLE__)                       (void)USB_EnableGlobalInt ((__HANDLE__)->Instance)
#define __HAL_PCD_DISABLE(__HANDLE__)                      (void)USB_DisableGlobalInt ((__HANDLE__)->Instance)
#define __HAL_PCD_CLEAR_FLAG(__HANDLE__, __INTERRUPT__)           (((__HANDLE__)->Instance->ISTR)\
                                                                   &= (uint16_t)(~(__INTERRUPT__)))

HAL_StatusTypeDef HAL_PCD_SetAddress(PCD_HandleTypeDef *hpcd, uint8_t address);

#define PCD_ENDP0                                                     0U
#define PCD_SNG_BUF                                                   0U
#define USB_CNTRX_BLSIZE                      (0x1U << 15)

#define PCD_SET_ENDPOINT(USBx, bEpNum, wRegValue) \
  (*(__IO uint16_t *)(&(USBx)->EP0R + ((bEpNum) * 2U)) = (uint16_t)(wRegValue))

#define PCD_GET_ENDPOINT(USBx, bEpNum)             (*(__IO uint16_t *)(&(USBx)->EP0R + ((bEpNum) * 2U)))

#define PCD_FREE_USER_BUFFER(USBx, bEpNum, bDir) \
  do { \
    if ((bDir) == 0U) \
    { \
      /* OUT double buffered endpoint */ \
      PCD_TX_DTOG((USBx), (bEpNum)); \
    } \
    else if ((bDir) == 1U) \
    { \
      /* IN double buffered endpoint */ \
      PCD_RX_DTOG((USBx), (bEpNum)); \
    } \
  } while(0)


#define PCD_SET_EP_TX_STATUS(USBx, bEpNum, wState) \
  do { \
    uint16_t _wRegVal; \
    \
    _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPTX_DTOGMASK; \
    /* toggle first bit ? */ \
    if ((USB_EPTX_DTOG1 & (wState))!= 0U) \
    { \
      _wRegVal ^= USB_EPTX_DTOG1; \
    } \
    /* toggle second bit ?  */ \
    if ((USB_EPTX_DTOG2 & (wState))!= 0U) \
    { \
      _wRegVal ^= USB_EPTX_DTOG2; \
    } \
    PCD_SET_ENDPOINT((USBx), (bEpNum), (_wRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX)); \
  } while(0) /* PCD_SET_EP_TX_STATUS */


#define PCD_SET_EP_RX_STATUS(USBx, bEpNum,wState) \
  do { \
    uint16_t _wRegVal; \
    \
    _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPRX_DTOGMASK; \
    /* toggle first bit ? */ \
    if ((USB_EPRX_DTOG1 & (wState))!= 0U) \
    { \
      _wRegVal ^= USB_EPRX_DTOG1; \
    } \
    /* toggle second bit ? */ \
    if ((USB_EPRX_DTOG2 & (wState))!= 0U) \
    { \
      _wRegVal ^= USB_EPRX_DTOG2; \
    } \
    PCD_SET_ENDPOINT((USBx), (bEpNum), (_wRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX)); \
  } while(0) /* PCD_SET_EP_RX_STATUS */


#define PCD_SET_EP_KIND(USBx, bEpNum) \
  do { \
    uint16_t _wRegVal; \
    \
    _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPREG_MASK; \
    \
    PCD_SET_ENDPOINT((USBx), (bEpNum), (_wRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_KIND)); \
  } while(0) /* PCD_SET_EP_KIND */

#define PCD_CLEAR_EP_KIND(USBx, bEpNum) \
  do { \
    uint16_t _wRegVal; \
    \
    _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPKIND_MASK; \
    \
    PCD_SET_ENDPOINT((USBx), (bEpNum), (_wRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX)); \
  } while(0) /* PCD_CLEAR_EP_KIND */



#define PCD_SET_BULK_EP_DBUF(USBx, bEpNum)     PCD_SET_EP_KIND((USBx), (bEpNum))
#define PCD_CLEAR_BULK_EP_DBUF(USBx, bEpNum)   PCD_CLEAR_EP_KIND((USBx), (bEpNum))

#define PCD_CLEAR_RX_EP_CTR(USBx, bEpNum) \
  do { \
    uint16_t _wRegVal; \
    \
    _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & (0x7FFFU & USB_EPREG_MASK); \
    \
    PCD_SET_ENDPOINT((USBx), (bEpNum), (_wRegVal | USB_EP_CTR_TX)); \
  } while(0) /* PCD_CLEAR_RX_EP_CTR */

#define PCD_CLEAR_TX_EP_CTR(USBx, bEpNum) \
  do { \
    uint16_t _wRegVal; \
    \
    _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & (0xFF7FU & USB_EPREG_MASK); \
    \
    PCD_SET_ENDPOINT((USBx), (bEpNum), (_wRegVal | USB_EP_CTR_RX)); \
  } while(0) /* PCD_CLEAR_TX_EP_CTR */


#define PCD_RX_DTOG(USBx, bEpNum) \
  do { \
    uint16_t _wEPVal; \
    \
    _wEPVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPREG_MASK; \
    \
    PCD_SET_ENDPOINT((USBx), (bEpNum), (_wEPVal | USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_DTOG_RX)); \
  } while(0) /* PCD_RX_DTOG */

#define PCD_TX_DTOG(USBx, bEpNum) \
  do { \
    uint16_t _wEPVal; \
    \
    _wEPVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPREG_MASK; \
    \
    PCD_SET_ENDPOINT((USBx), (bEpNum), (_wEPVal | USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_DTOG_TX)); \
  } while(0) /* PCD_TX_DTOG */

#define PCD_CLEAR_RX_DTOG(USBx, bEpNum) \
  do { \
    uint16_t _wRegVal; \
    \
    _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum)); \
    \
    if ((_wRegVal & USB_EP_DTOG_RX) != 0U)\
    { \
      PCD_RX_DTOG((USBx), (bEpNum)); \
    } \
  } while(0) /* PCD_CLEAR_RX_DTOG */

#define PCD_CLEAR_TX_DTOG(USBx, bEpNum) \
  do { \
    uint16_t _wRegVal; \
    \
    _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum)); \
    \
    if ((_wRegVal & USB_EP_DTOG_TX) != 0U)\
    { \
      PCD_TX_DTOG((USBx), (bEpNum)); \
    } \
  } while(0) /* PCD_CLEAR_TX_DTOG */


#define PCD_SET_EP_ADDRESS(USBx, bEpNum, bAddr) \
  do { \
    uint16_t _wRegVal; \
    \
    _wRegVal = (PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPREG_MASK) | (bAddr); \
    \
    PCD_SET_ENDPOINT((USBx), (bEpNum), (_wRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX)); \
  } while(0) /* PCD_SET_EP_ADDRESS */



#define PCD_EP_TX_CNT(USBx, bEpNum) \
  ((uint16_t *)((((uint32_t)(USBx)->BTABLE + \
                  ((uint32_t)(bEpNum) * 8U) + 2U) * PMA_ACCESS) + ((uint32_t)(USBx) + 0x400U)))

#define PCD_EP_RX_CNT(USBx, bEpNum) \
  ((uint16_t *)((((uint32_t)(USBx)->BTABLE + \
                  ((uint32_t)(bEpNum) * 8U) + 6U) * PMA_ACCESS) + ((uint32_t)(USBx) + 0x400U)))


#define PCD_SET_EP_TX_ADDRESS(USBx, bEpNum, wAddr) \
  do { \
    __IO uint16_t *_wRegVal; \
    uint32_t _wRegBase = (uint32_t)USBx; \
    \
    _wRegBase += (uint32_t)(USBx)->BTABLE; \
    _wRegVal = (__IO uint16_t *)(_wRegBase + 0x400U + (((uint32_t)(bEpNum) * 8U) * PMA_ACCESS)); \
    *_wRegVal = ((wAddr) >> 1) << 1; \
  } while(0) /* PCD_SET_EP_TX_ADDRESS */

#define PCD_SET_EP_RX_ADDRESS(USBx, bEpNum, wAddr) \
  do { \
    __IO uint16_t *_wRegVal; \
    uint32_t _wRegBase = (uint32_t)USBx; \
    \
    _wRegBase += (uint32_t)(USBx)->BTABLE; \
    _wRegVal = (__IO uint16_t *)(_wRegBase + 0x400U + ((((uint32_t)(bEpNum) * 8U) + 4U) * PMA_ACCESS)); \
    *_wRegVal = ((wAddr) >> 1) << 1; \
  } while(0) /* PCD_SET_EP_RX_ADDRESS */



#define PCD_CALC_BLK32(pdwReg, wCount, wNBlocks) \
  do { \
    (wNBlocks) = (wCount) >> 5; \
    if (((wCount) & 0x1fU) == 0U) \
    { \
      (wNBlocks)--; \
    } \
    *(pdwReg) |= (uint16_t)(((wNBlocks) << 10) | USB_CNTRX_BLSIZE); \
  } while(0) /* PCD_CALC_BLK32 */

#define PCD_CALC_BLK2(pdwReg, wCount, wNBlocks) \
  do { \
    (wNBlocks) = (wCount) >> 1; \
    if (((wCount) & 0x1U) != 0U) \
    { \
      (wNBlocks)++; \
    } \
    *(pdwReg) |= (uint16_t)((wNBlocks) << 10); \
  } while(0) /* PCD_CALC_BLK2 */

#define PCD_SET_EP_CNT_RX_REG(pdwReg, wCount) \
  do { \
    uint32_t wNBlocks; \
    \
    *(pdwReg) &= 0x3FFU; \
    \
    if ((wCount) > 62U) \
    { \
      PCD_CALC_BLK32((pdwReg), (wCount), wNBlocks); \
    } \
    else \
    { \
      if ((wCount) == 0U) \
      { \
        *(pdwReg) |= USB_CNTRX_BLSIZE; \
      } \
      else \
      { \
        PCD_CALC_BLK2((pdwReg), (wCount), wNBlocks); \
      } \
    } \
  } while(0) /* PCD_SET_EP_CNT_RX_REG */

#define PCD_SET_EP_RX_DBUF0_CNT(USBx, bEpNum, wCount) \
  do { \
    uint32_t _wRegBase = (uint32_t)(USBx); \
    __IO uint16_t *pdwReg; \
    \
    _wRegBase += (uint32_t)(USBx)->BTABLE; \
    pdwReg = (__IO uint16_t *)(_wRegBase + 0x400U + ((((uint32_t)(bEpNum) * 8U) + 2U) * PMA_ACCESS)); \
    PCD_SET_EP_CNT_RX_REG(pdwReg, (wCount)); \
  } while(0)


#define PCD_SET_EP_TX_CNT(USBx, bEpNum, wCount) \
  do { \
    uint32_t _wRegBase = (uint32_t)(USBx); \
    __IO uint16_t *_wRegVal; \
    \
    _wRegBase += (uint32_t)(USBx)->BTABLE; \
    _wRegVal = (__IO uint16_t *)(_wRegBase + 0x400U + ((((uint32_t)(bEpNum) * 8U) + 2U) * PMA_ACCESS)); \
    *_wRegVal = (uint16_t)(wCount); \
  } while(0)

#define PCD_SET_EP_RX_CNT(USBx, bEpNum, wCount) \
  do { \
    uint32_t _wRegBase = (uint32_t)(USBx); \
    __IO uint16_t *_wRegVal; \
    \
    _wRegBase += (uint32_t)(USBx)->BTABLE; \
    _wRegVal = (__IO uint16_t *)(_wRegBase + 0x400U + ((((uint32_t)(bEpNum) * 8U) + 6U) * PMA_ACCESS)); \
    PCD_SET_EP_CNT_RX_REG(_wRegVal, (wCount)); \
  } while(0)

#define PCD_GET_EP_TX_CNT(USBx, bEpNum)        ((uint32_t)(*PCD_EP_TX_CNT((USBx), (bEpNum))) & 0x3ffU)
#define PCD_GET_EP_RX_CNT(USBx, bEpNum)        ((uint32_t)(*PCD_EP_RX_CNT((USBx), (bEpNum))) & 0x3ffU)

#define PCD_SET_EP_DBUF0_ADDR(USBx, bEpNum, wBuf0Addr) \
  do { \
    PCD_SET_EP_TX_ADDRESS((USBx), (bEpNum), (wBuf0Addr)); \
  } while(0) /* PCD_SET_EP_DBUF0_ADDR */

#define PCD_SET_EP_DBUF1_ADDR(USBx, bEpNum, wBuf1Addr) \
  do { \
    PCD_SET_EP_RX_ADDRESS((USBx), (bEpNum), (wBuf1Addr)); \
  } while(0) /* PCD_SET_EP_DBUF1_ADDR */


#define PCD_SET_EP_DBUF_ADDR(USBx, bEpNum, wBuf0Addr, wBuf1Addr) \
  do { \
    PCD_SET_EP_DBUF0_ADDR((USBx), (bEpNum), (wBuf0Addr)); \
    PCD_SET_EP_DBUF1_ADDR((USBx), (bEpNum), (wBuf1Addr)); \
  } while(0) /* PCD_SET_EP_DBUF_ADDR */


#define PCD_GET_EP_DBUF1_ADDR(USBx, bEpNum)    (PCD_GET_EP_RX_ADDRESS((USBx), (bEpNum)))


#define PCD_SET_EP_DBUF0_CNT(USBx, bEpNum, bDir, wCount) \
  do { \
    if ((bDir) == 0U) \
      /* OUT endpoint */ \
    { \
      PCD_SET_EP_RX_DBUF0_CNT((USBx), (bEpNum), (wCount)); \
    } \
    else \
    { \
      if ((bDir) == 1U) \
      { \
        /* IN endpoint */ \
        PCD_SET_EP_TX_CNT((USBx), (bEpNum), (wCount)); \
      } \
    } \
  } while(0) /* SetEPDblBuf0Count*/

#define PCD_SET_EP_DBUF1_CNT(USBx, bEpNum, bDir, wCount) \
  do { \
    uint32_t _wBase = (uint32_t)(USBx); \
    __IO uint16_t *_wEPRegVal; \
    \
    if ((bDir) == 0U) \
    { \
      /* OUT endpoint */ \
      PCD_SET_EP_RX_CNT((USBx), (bEpNum), (wCount)); \
    } \
    else \
    { \
      if ((bDir) == 1U) \
      { \
        /* IN endpoint */ \
        _wBase += (uint32_t)(USBx)->BTABLE; \
        _wEPRegVal = (__IO uint16_t *)(_wBase + 0x400U + ((((uint32_t)(bEpNum) * 8U) + 6U) * PMA_ACCESS)); \
        *_wEPRegVal = (uint16_t)(wCount); \
      } \
    } \
  } while(0) /* SetEPDblBuf1Count */

#define PCD_SET_EP_DBUF_CNT(USBx, bEpNum, bDir, wCount) \
  do { \
    PCD_SET_EP_DBUF0_CNT((USBx), (bEpNum), (bDir), (wCount)); \
    PCD_SET_EP_DBUF1_CNT((USBx), (bEpNum), (bDir), (wCount)); \
  } while(0) /* PCD_SET_EP_DBUF_CNT */


#define PCD_GET_EP_DBUF0_CNT(USBx, bEpNum)     (PCD_GET_EP_TX_CNT((USBx), (bEpNum)))
#define PCD_GET_EP_DBUF1_CNT(USBx, bEpNum)     (PCD_GET_EP_RX_CNT((USBx), (bEpNum)))




#endif /* STM32F1xx_HAL_PCD_H */

#endif /* HAL_PCD_MODULE_ENABLED */

#define assert_param(expr) ((void)0U)

typedef enum{
  HAL_TICK_FREQ_10HZ         = 100U,
  HAL_TICK_FREQ_100HZ        = 10U,
  HAL_TICK_FREQ_1KHZ         = 1U,
  HAL_TICK_FREQ_DEFAULT      = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);

extern __IO uint32_t uwTick;
extern uint32_t uwTickPrio;
extern HAL_TickFreqTypeDef uwTickFreq;

extern uint32_t SystemCoreClock;           /*!< System Clock Frequency (Core Clock) */
extern const uint8_t  AHBPrescTable[16U];  /*!< AHB prescalers table values */
extern const uint8_t  APBPrescTable[8U];   /*!< APB prescalers table values */

#endif /* __STM32F1xx_HAL_H */

