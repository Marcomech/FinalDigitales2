#ifndef __USB_DEVICE__H__
#define __USB_DEVICE__H__


#include "../Core/Src/Inc_USB_Device2.h"

#define USBD_MAX_NUM_INTERFACES           1
#define USBD_MAX_NUM_CONFIGURATION        1
#define USBD_MAX_STR_DESC_SIZ             512
#define USBD_DEBUG_LEVEL     			        0
#define USBD_SELF_POWERED     		        1
#define HID_FS_BINTERVAL     			        0xA

#define DEVICE_FS 			                  0

#define USBD_malloc   (uint32_t *)USBD_static_malloc
#define USBD_free     USBD_static_free
#define USBD_Delay    HAL_Delay

void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);

#define USBD_UsrLog(...)
#define USBD_ErrLog(...)
#define USBD_DbgLog(...)

#ifndef NULL
#define NULL        0U
#endif /* NULL */

#ifndef USBD_MAX_NUM_INTERFACES
#define USBD_MAX_NUM_INTERFACES       1U
#endif /* USBD_MAX_NUM_CONFIGURATION */

#ifndef USBD_MAX_NUM_CONFIGURATION
#define USBD_MAX_NUM_CONFIGURATION    1U
#endif /* USBD_MAX_NUM_CONFIGURATION */

#ifndef USBD_LPM_ENABLED
#define USBD_LPM_ENABLED        0U
#endif /* USBD_LPM_ENABLED */

#ifndef USBD_SELF_POWERED
#define USBD_SELF_POWERED       1U
#endif /*USBD_SELF_POWERED */

#ifndef USBD_SUPPORT_USER_STRING_DESC
#define USBD_SUPPORT_USER_STRING_DESC       0U
#endif /* USBD_SUPPORT_USER_STRING_DESC */

#define  USB_LEN_DEV_QUALIFIER_DESC         0x0AU
#define  USB_LEN_DEV_DESC                   0x12U
#define  USB_LEN_CFG_DESC                   0x09U
#define  USB_LEN_IF_DESC                    0x09U
#define  USB_LEN_EP_DESC                    0x07U
#define  USB_LEN_OTG_DESC                   0x03U
#define  USB_LEN_LANGID_STR_DESC            0x04U
#define  USB_LEN_OTHER_SPEED_DESC_SIZ       0x09U

#define  USBD_IDX_LANGID_STR                0x00U
#define  USBD_IDX_MFC_STR                   0x01U
#define  USBD_IDX_PRODUCT_STR               0x02U
#define  USBD_IDX_SERIAL_STR                0x03U
#define  USBD_IDX_CONFIG_STR                0x04U
#define  USBD_IDX_INTERFACE_STR             0x05U

#define  USB_REQ_TYPE_STANDARD              0x00U
#define  USB_REQ_TYPE_CLASS                 0x20U
#define  USB_REQ_TYPE_VENDOR                0x40U
#define  USB_REQ_TYPE_MASK                  0x60U

#define  USB_REQ_RECIPIENT_DEVICE           0x00U
#define  USB_REQ_RECIPIENT_INTERFACE        0x01U
#define  USB_REQ_RECIPIENT_ENDPOINT         0x02U
#define  USB_REQ_RECIPIENT_MASK             0x03U

#define  USB_REQ_GET_STATUS                 0x00U
#define  USB_REQ_CLEAR_FEATURE              0x01U
#define  USB_REQ_SET_FEATURE                0x03U
#define  USB_REQ_SET_ADDRESS                0x05U
#define  USB_REQ_GET_DESCRIPTOR             0x06U
#define  USB_REQ_SET_DESCRIPTOR             0x07U
#define  USB_REQ_GET_CONFIGURATION          0x08U
#define  USB_REQ_SET_CONFIGURATION          0x09U
#define  USB_REQ_GET_INTERFACE              0x0AU
#define  USB_REQ_SET_INTERFACE              0x0BU
#define  USB_REQ_SYNCH_FRAME                0x0CU

#define  USB_DESC_TYPE_DEVICE                     0x01U
#define  USB_DESC_TYPE_CONFIGURATION              0x02U
#define  USB_DESC_TYPE_STRING                     0x03U
#define  USB_DESC_TYPE_INTERFACE                  0x04U
#define  USB_DESC_TYPE_ENDPOINT                   0x05U
#define  USB_DESC_TYPE_DEVICE_QUALIFIER           0x06U
#define  USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION  0x07U
#define  USB_DESC_TYPE_BOS                        0x0FU

#define USB_CONFIG_REMOTE_WAKEUP                  0x02U
#define USB_CONFIG_SELF_POWERED                   0x01U

#define USB_FEATURE_EP_HALT                       0x00U
#define USB_FEATURE_REMOTE_WAKEUP                 0x01U
#define USB_FEATURE_TEST_MODE                     0x02U

#define USB_DEVICE_CAPABITY_TYPE                  0x10U

#define USB_HS_MAX_PACKET_SIZE                    512U
#define USB_FS_MAX_PACKET_SIZE                    64U
#define USB_MAX_EP0_SIZE                          64U

/*  Device Status */
#define USBD_STATE_DEFAULT                        0x01U
#define USBD_STATE_ADDRESSED                      0x02U
#define USBD_STATE_CONFIGURED                     0x03U
#define USBD_STATE_SUSPENDED                      0x04U


/*  EP0 State */
#define USBD_EP0_IDLE                             0x00U
#define USBD_EP0_SETUP                            0x01U
#define USBD_EP0_DATA_IN                          0x02U
#define USBD_EP0_DATA_OUT                         0x03U
#define USBD_EP0_STATUS_IN                        0x04U
#define USBD_EP0_STATUS_OUT                       0x05U
#define USBD_EP0_STALL                            0x06U

#define USBD_EP_TYPE_CTRL                         0x00U
#define USBD_EP_TYPE_ISOC                         0x01U
#define USBD_EP_TYPE_BULK                         0x02U
#define USBD_EP_TYPE_INTR                         0x03U

#define  SWAPBYTE(addr)  (((uint16_t)(*((uint8_t *)(addr)))) + \
       (((uint16_t)(*(((uint8_t *)(addr)) + 1U))) << 8U))

#define LOBYTE(x)  ((uint8_t)((x) & 0x00FFU))
#define HIBYTE(x)  ((uint8_t)(((x) & 0xFF00U) >> 8U))
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))

#ifndef __USBD_CORE_H
#define __USBD_CORE_H

USBD_StatusTypeDef  USBD_StdDevReq(USBD_HandleTypeDef  *pdev, USBD_SetupReqTypedef  *req);
USBD_StatusTypeDef  USBD_StdItfReq(USBD_HandleTypeDef  *pdev, USBD_SetupReqTypedef  *req);
USBD_StatusTypeDef  USBD_StdEPReq(USBD_HandleTypeDef  *pdev, USBD_SetupReqTypedef  *req);


void USBD_CtlError(USBD_HandleTypeDef  *pdev, USBD_SetupReqTypedef *req);

void USBD_ParseSetupRequest(USBD_SetupReqTypedef *req, uint8_t *pdata);

void USBD_GetString(uint8_t *desc, uint8_t *unicode, uint16_t *len);

#ifndef USBD_DEBUG_LEVEL
#define USBD_DEBUG_LEVEL     0U
#endif /* USBD_DEBUG_LEVEL */

#define USBD_SOF    USBD_LL_SOF

USBD_StatusTypeDef USBD_Init(USBD_HandleTypeDef *pdev, USBD_DescriptorsTypeDef *pdesc, uint8_t id);
USBD_StatusTypeDef USBD_DeInit(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef USBD_Start(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef USBD_Stop(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef USBD_RegisterClass(USBD_HandleTypeDef *pdev, USBD_ClassTypeDef *pclass);

USBD_StatusTypeDef USBD_RunTestMode(USBD_HandleTypeDef  *pdev);
USBD_StatusTypeDef USBD_SetClassConfig(USBD_HandleTypeDef  *pdev, uint8_t cfgidx);
USBD_StatusTypeDef USBD_ClrClassConfig(USBD_HandleTypeDef  *pdev, uint8_t cfgidx);

USBD_StatusTypeDef USBD_LL_SetupStage(USBD_HandleTypeDef *pdev, uint8_t *psetup);
USBD_StatusTypeDef USBD_LL_DataOutStage(USBD_HandleTypeDef *pdev, uint8_t epnum, uint8_t *pdata);
USBD_StatusTypeDef USBD_LL_DataInStage(USBD_HandleTypeDef *pdev, uint8_t epnum, uint8_t *pdata);

USBD_StatusTypeDef USBD_LL_Reset(USBD_HandleTypeDef  *pdev);
USBD_StatusTypeDef USBD_LL_SetSpeed(USBD_HandleTypeDef  *pdev, USBD_SpeedTypeDef speed);
USBD_StatusTypeDef USBD_LL_Suspend(USBD_HandleTypeDef  *pdev);
USBD_StatusTypeDef USBD_LL_Resume(USBD_HandleTypeDef  *pdev);

USBD_StatusTypeDef USBD_LL_SOF(USBD_HandleTypeDef  *pdev);
USBD_StatusTypeDef USBD_LL_IsoINIncomplete(USBD_HandleTypeDef  *pdev, uint8_t epnum);
USBD_StatusTypeDef USBD_LL_IsoOUTIncomplete(USBD_HandleTypeDef  *pdev, uint8_t epnum);

USBD_StatusTypeDef USBD_LL_DevConnected(USBD_HandleTypeDef  *pdev);
USBD_StatusTypeDef USBD_LL_DevDisconnected(USBD_HandleTypeDef  *pdev);

/* USBD Low Level Driver */
USBD_StatusTypeDef  USBD_LL_Init(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef  USBD_LL_DeInit(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef  USBD_LL_Start(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef  USBD_LL_Stop(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef  USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t  ep_addr, uint8_t  ep_type, uint16_t ep_mps);

USBD_StatusTypeDef  USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr);
USBD_StatusTypeDef  USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr);
USBD_StatusTypeDef  USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr);
USBD_StatusTypeDef  USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr);
uint8_t       USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr);
USBD_StatusTypeDef  USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr);
USBD_StatusTypeDef  USBD_LL_Transmit(USBD_HandleTypeDef *pdev,   uint8_t  ep_addr, uint8_t  *pbuf, uint16_t  size);

USBD_StatusTypeDef  USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t  ep_addr, uint8_t  *pbuf, uint16_t  size);

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t  ep_addr);
void  USBD_LL_Delay(uint32_t Delay);

#endif /* __USBD_CORE_H */

USBD_StatusTypeDef USBD_CtlSendData(USBD_HandleTypeDef *pdev, uint8_t *pbuf, uint16_t len);
USBD_StatusTypeDef USBD_CtlContinueSendData(USBD_HandleTypeDef  *pdev, uint8_t *pbuf, uint16_t len);
USBD_StatusTypeDef USBD_CtlPrepareRx(USBD_HandleTypeDef  *pdev, uint8_t *pbuf, uint16_t len);
USBD_StatusTypeDef USBD_CtlContinueRx(USBD_HandleTypeDef  *pdev, uint8_t *pbuf, uint16_t len);
USBD_StatusTypeDef USBD_CtlSendStatus(USBD_HandleTypeDef  *pdev);
USBD_StatusTypeDef USBD_CtlReceiveStatus(USBD_HandleTypeDef  *pdev);

uint32_t  USBD_GetRxCount(USBD_HandleTypeDef *pdev, uint8_t ep_addr);

#define HID_EPIN_ADDR                                0x81U
#define HID_EPIN_SIZE                                0x04U

#define USB_HID_CONFIG_DESC_SIZ                      34U
#define USB_HID_DESC_SIZ                             9U
#define HID_MOUSE_REPORT_DESC_SIZE                   63U

#define HID_DESCRIPTOR_TYPE                          0x21U
#define HID_REPORT_DESC                              0x22U

#ifndef HID_HS_BINTERVAL
#define HID_HS_BINTERVAL                             0x07U
#endif

#ifndef HID_FS_BINTERVAL
#define HID_FS_BINTERVAL                             0x0AU
#endif

#define HID_REQ_SET_PROTOCOL                         0x0BU
#define HID_REQ_GET_PROTOCOL                         0x03U

#define HID_REQ_SET_IDLE                             0x0AU
#define HID_REQ_GET_IDLE                             0x02U

#define HID_REQ_SET_REPORT                           0x09U
#define HID_REQ_GET_REPORT                           0x01U

typedef enum{
  HID_IDLE = 0,
  HID_BUSY,
} HID_StateTypeDef;


typedef struct{
  uint32_t       Protocol;
  uint32_t       IdleState;
  uint32_t       AltSetting;
  HID_StateTypeDef     state;
} USBD_HID_HandleTypeDef;

extern USBD_ClassTypeDef  USBD_HID;
#define USBD_HID_CLASS    &USBD_HID

uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len);
uint32_t USBD_HID_GetPollingInterval(USBD_HandleTypeDef *pdev);

#endif
