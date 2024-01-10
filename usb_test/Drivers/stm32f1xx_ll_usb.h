#ifndef STM32F1xx_LL_USB_H
#define STM32F1xx_LL_USB_H

#ifndef __STM32F1xx_HAL_DEF
#define __STM32F1xx_HAL_DEF

#include "stm32f1xx.h"
#include <stddef.h>

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

#define __HAL_LINKDMA(__HANDLE__, __PPP_DMA_FIELD__, __DMA_HANDLE__)               \
                        do{                                                      \
                              (__HANDLE__)->__PPP_DMA_FIELD__ = &(__DMA_HANDLE__); \
                              (__DMA_HANDLE__).Parent = (__HANDLE__);             \
                          } while(0U)

#if !defined(UNUSED)
#define UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */
#endif /* UNUSED */


#define __HAL_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = 0U)

#if (USE_RTOS == 1U)
#error "USE_RTOS should be 0 in the current HAL release"
#else
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
#endif /* USE_RTOS */



#ifndef __weak
#define __weak   __attribute__((weak))
#endif /* __weak */

#ifndef __packed
#define __packed __attribute__((__packed__))
#endif /* __packed */

#ifndef __ALIGN_BEGIN
#define __ALIGN_BEGIN
#endif

#ifndef __ALIGN_END
#define __ALIGN_END      __attribute__ ((aligned (4)))
#endif

#endif /* ___STM32F1xx_HAL_DEF */

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
  uint32_t dev_endpoints;           /*!< Device Endpoints number.
                                         This parameter depends on the used USB core.
                                         This parameter must be a number between Min_Data = 1 and Max_Data = 15 */
  uint32_t speed;                   /*!< USB Core speed.
                                         This parameter can be any value of @ref PCD_Speed/HCD_Speed
                                                                                 (HCD_SPEED_xxx, HCD_SPEED_xxx) */
  uint32_t ep0_mps;                 /*!< Set the Endpoint 0 Max Packet size.                                    */

  uint32_t phy_itface;              /*!< Select the used PHY interface.
                                         This parameter can be any value of @ref PCD_PHY_Module/HCD_PHY_Module  */

  uint32_t Sof_enable;              /*!< Enable or disable the output of the SOF signal.                        */

  uint32_t low_power_enable;        /*!< Enable or disable the low Power Mode.                                  */

  uint32_t lpm_enable;              /*!< Enable or disable Link Power Management.                               */

  uint32_t battery_charging_enable; /*!< Enable or disable Battery charging.                                    */
} USB_CfgTypeDef;

typedef struct
{
  uint8_t   num;                  /*!< Endpoint number
                                       This parameter must be a number between Min_Data = 1 and Max_Data = 15   */

  uint8_t   is_in;                /*!< Endpoint direction
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 1    */

  uint8_t   is_stall;             /*!< Endpoint stall condition
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 1    */


  uint8_t   type;                 /*!< Endpoint type
                                       This parameter can be any value of @ref USB_LL_EP_Type                   */

  uint8_t   data_pid_start;       /*!< Initial data PID
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 1    */


  uint16_t  pmaadress;            /*!< PMA Address
                                       This parameter can be any value between Min_addr = 0 and Max_addr = 1K   */

  uint16_t  pmaaddr0;             /*!< PMA Address0
                                       This parameter can be any value between Min_addr = 0 and Max_addr = 1K   */

  uint16_t  pmaaddr1;             /*!< PMA Address1
                                       This parameter can be any value between Min_addr = 0 and Max_addr = 1K   */

  uint8_t   doublebuffer;         /*!< Double buffer enable
                                       This parameter can be 0 or 1                                             */


  uint32_t  maxpacket;            /*!< Endpoint Max packet size
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 64KB */

  uint8_t   *xfer_buff;           /*!< Pointer to transfer buffer                                               */

  uint32_t  xfer_len;             /*!< Current transfer length                                                  */

  uint32_t  xfer_count;           /*!< Partial transfer length in case of multi packet transfer                 */

  uint32_t  xfer_len_db;          /*!< double buffer transfer length used with bulk double buffer in            */

  uint8_t   xfer_fill_db;         /*!< double buffer Need to Fill new buffer  used with bulk_in                 */

} USB_EPTypeDef;

typedef struct
{
  uint8_t   dev_addr;           /*!< USB device address.
                                     This parameter must be a number between Min_Data = 1 and Max_Data = 255    */

  uint8_t   ch_num;             /*!< Host channel number.
                                     This parameter must be a number between Min_Data = 1 and Max_Data = 15     */

  uint8_t   ep_num;             /*!< Endpoint number.
                                     This parameter must be a number between Min_Data = 1 and Max_Data = 15     */

  uint8_t   ep_is_in;           /*!< Endpoint direction
                                     This parameter must be a number between Min_Data = 0 and Max_Data = 1      */

  uint8_t   speed;              /*!< USB Host Channel speed.
                                     This parameter can be any value of @ref HCD_Device_Speed:
                                                                             (HCD_DEVICE_SPEED_xxx)             */

  uint8_t   do_ping;            /*!< Enable or disable the use of the PING protocol for HS mode.                */

  uint8_t   hub_port_nbr;       /*!< USB HUB port number                                                        */
  uint8_t   hub_addr;           /*!< USB HUB address                                                            */

  uint8_t   ep_type;            /*!< Endpoint Type.
                                     This parameter can be any value of @ref USB_LL_EP_Type                     */

  uint16_t  max_packet;         /*!< Endpoint Max packet size.
                                     This parameter must be a number between Min_Data = 0 and Max_Data = 64KB   */

  uint8_t   data_pid;           /*!< Initial data PID.
                                     This parameter must be a number between Min_Data = 0 and Max_Data = 1      */

  uint8_t   *xfer_buff;         /*!< Pointer to transfer buffer.                                                */

  uint32_t  XferSize;           /*!< OTG Channel transfer size.                                                 */

  uint32_t  xfer_len;           /*!< Current transfer length.                                                   */

  uint32_t  xfer_count;         /*!< Partial transfer length in case of multi packet transfer.                  */

  uint8_t   toggle_in;          /*!< IN transfer current toggle flag.
                                     This parameter must be a number between Min_Data = 0 and Max_Data = 1      */

  uint8_t   toggle_out;         /*!< OUT transfer current toggle flag
                                     This parameter must be a number between Min_Data = 0 and Max_Data = 1      */

  uint32_t  dma_addr;           /*!< 32 bits aligned transfer buffer address.                                   */

  uint32_t  ErrCnt;             /*!< Host channel error count.                                                  */

  USB_URBStateTypeDef urb_state;  /*!< URB state.
                                       This parameter can be any value of @ref USB_URBStateTypeDef              */

  USB_HCStateTypeDef state;       /*!< Host Channel state.
                                       This parameter can be any value of @ref USB_HCStateTypeDef               */
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

#ifndef USB_EP_RX_STRX
#define USB_EP_RX_STRX                         (0x3U << 12)
#endif /* USB_EP_RX_STRX */

#define EP_ADDR_MSK                            0x7U

#ifndef USE_USB_DOUBLE_BUFFER
#define USE_USB_DOUBLE_BUFFER                  1U
#endif /* USE_USB_DOUBLE_BUFFER */



HAL_StatusTypeDef USB_CoreInit(USB_TypeDef *USBx, USB_CfgTypeDef cfg);
HAL_StatusTypeDef USB_DevInit(USB_TypeDef *USBx, USB_CfgTypeDef cfg);
HAL_StatusTypeDef USB_EnableGlobalInt(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_DisableGlobalInt(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_SetCurrentMode(USB_TypeDef *USBx, USB_ModeTypeDef mode);
HAL_StatusTypeDef USB_SetDevSpeed(USB_TypeDef *USBx, uint8_t speed);

HAL_StatusTypeDef USB_FlushRxFifo(USB_TypeDef const *USBx);
HAL_StatusTypeDef USB_FlushTxFifo(USB_TypeDef const *USBx, uint32_t num);

#if defined (HAL_PCD_MODULE_ENABLED)
HAL_StatusTypeDef USB_ActivateEndpoint(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_DeactivateEndpoint(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPStartXfer(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPSetStall(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPClearStall(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPStopXfer(USB_TypeDef *USBx, USB_EPTypeDef *ep);
#endif /* defined (HAL_PCD_MODULE_ENABLED) */

HAL_StatusTypeDef USB_SetDevAddress(USB_TypeDef *USBx, uint8_t address);
HAL_StatusTypeDef USB_DevConnect(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_DevDisconnect(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_StopDevice(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_EP0_OutStart(USB_TypeDef *USBx, uint8_t *psetup);
HAL_StatusTypeDef USB_WritePacket(USB_TypeDef *USBx, uint8_t *src,
                                  uint8_t ch_ep_num, uint16_t len);

void             *USB_ReadPacket(USB_TypeDef *USBx, uint8_t *dest, uint16_t len);

uint32_t          USB_ReadInterrupts(USB_TypeDef const *USBx);
uint32_t          USB_ReadDevAllOutEpInterrupt(USB_TypeDef *USBx);
uint32_t          USB_ReadDevOutEPInterrupt(USB_TypeDef *USBx, uint8_t epnum);
uint32_t          USB_ReadDevAllInEpInterrupt(USB_TypeDef *USBx);
uint32_t          USB_ReadDevInEPInterrupt(USB_TypeDef *USBx, uint8_t epnum);
void              USB_ClearInterrupts(USB_TypeDef *USBx, uint32_t interrupt);
HAL_StatusTypeDef USB_ActivateRemoteWakeup(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_DeActivateRemoteWakeup(USB_TypeDef *USBx);

void              USB_WritePMA(USB_TypeDef const *USBx, uint8_t *pbUsrBuf,
                               uint16_t wPMABufAddr, uint16_t wNBytes);

void              USB_ReadPMA(USB_TypeDef const *USBx, uint8_t *pbUsrBuf,
                              uint16_t wPMABufAddr, uint16_t wNBytes);





#endif /* STM32F1xx_LL_USB_H */
