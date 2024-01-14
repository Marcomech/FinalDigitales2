#include "../../Drivers/stm32f1xx.h"

#ifndef __USB_DEVICE__H__
#define __USB_DEVICE__H__

typedef  struct  usb_setup_req{
  uint8_t   bmRequest;
  uint8_t   bRequest;
  uint16_t  wValue;
  uint16_t  wIndex;
  uint16_t  wLength;
} USBD_SetupReqTypedef;

struct _USBD_HandleTypeDef;

typedef struct _Device_cb{
  uint8_t (*Init)(struct _USBD_HandleTypeDef *pdev, uint8_t cfgidx);
  uint8_t (*DeInit)(struct _USBD_HandleTypeDef *pdev, uint8_t cfgidx);
  uint8_t (*Setup)(struct _USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef  *req);
  uint8_t (*EP0_TxSent)(struct _USBD_HandleTypeDef *pdev);
  uint8_t (*EP0_RxReady)(struct _USBD_HandleTypeDef *pdev);
  uint8_t (*DataIn)(struct _USBD_HandleTypeDef *pdev, uint8_t epnum);
  uint8_t (*DataOut)(struct _USBD_HandleTypeDef *pdev, uint8_t epnum);
  uint8_t (*SOF)(struct _USBD_HandleTypeDef *pdev);
  uint8_t (*IsoINIncomplete)(struct _USBD_HandleTypeDef *pdev, uint8_t epnum);
  uint8_t (*IsoOUTIncomplete)(struct _USBD_HandleTypeDef *pdev, uint8_t epnum);
  uint8_t  *(*GetHSConfigDescriptor)(uint16_t *length);
  uint8_t  *(*GetFSConfigDescriptor)(uint16_t *length);
  uint8_t  *(*GetOtherSpeedConfigDescriptor)(uint16_t *length);
  uint8_t  *(*GetDeviceQualifierDescriptor)(uint16_t *length);
} USBD_ClassTypeDef;

/* Following USB Device Speed */
typedef enum{
  USBD_SPEED_HIGH  = 0U,
  USBD_SPEED_FULL  = 1U,
  USBD_SPEED_LOW   = 2U,
} USBD_SpeedTypeDef;

/* Following USB Device status */
typedef enum{
  USBD_OK   = 0U,
  USBD_BUSY,
  USBD_FAIL,
} USBD_StatusTypeDef;

/* USB Device descriptors structure */
typedef struct{
  uint8_t  *(*GetDeviceDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t  *(*GetLangIDStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t  *(*GetManufacturerStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t  *(*GetProductStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t  *(*GetSerialStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t  *(*GetConfigurationStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
  uint8_t  *(*GetInterfaceStrDescriptor)(USBD_SpeedTypeDef speed, uint16_t *length);
} USBD_DescriptorsTypeDef;

/* USB Device handle structure */
typedef struct{
  uint32_t                status;
  uint32_t                is_used;
  uint32_t                total_length;
  uint32_t                rem_length;
  uint32_t                maxpacket;
} USBD_EndpointTypeDef;

/* USB Device handle structure */
typedef struct _USBD_HandleTypeDef{
  uint8_t                 id;
  uint32_t                dev_config;
  uint32_t                dev_default_config;
  uint32_t                dev_config_status;
  USBD_SpeedTypeDef       dev_speed;
  USBD_EndpointTypeDef    ep_in[16];
  USBD_EndpointTypeDef    ep_out[16];
  uint32_t                ep0_state;
  uint32_t                ep0_data_len;
  uint8_t                 dev_state;
  uint8_t                 dev_old_state;
  uint8_t                 dev_address;
  uint8_t                 dev_connection_status;
  uint8_t                 dev_test_mode;
  uint32_t                dev_remote_wakeup;

  USBD_SetupReqTypedef    request;
  USBD_DescriptorsTypeDef *pDesc;
  USBD_ClassTypeDef       *pClass;
  void                    *pClassData;
  void                    *pUserData;
  void                    *pData;
} USBD_HandleTypeDef;

#endif
