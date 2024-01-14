#include "../Core/Src/Inc_USB_Device2.h"
#include "../Core/Src/Inc_USB_Device.h"

#include "../Drivers/stm32f1xx.h"
#include "../Drivers/stm32f1xx_hal.h"

USBD_StatusTypeDef USBD_Init(USBD_HandleTypeDef *pdev, USBD_DescriptorsTypeDef *pdesc, uint8_t id){
  /* Check whether the USB Host handle is valid */
  if (pdev == NULL){
    return USBD_FAIL;
  }
  /* Unlink previous class*/
  if (pdev->pClass != NULL){
    pdev->pClass = NULL;
  }
  /* Assign USBD Descriptors */
  if (pdesc != NULL){
    pdev->pDesc = pdesc;
  }
  /* Set Device initial State */
  pdev->dev_state = USBD_STATE_DEFAULT;
  pdev->id = id;
  /* Initialize low level driver */
  USBD_LL_Init(pdev);
  return USBD_OK;
}

USBD_StatusTypeDef  USBD_RegisterClass(USBD_HandleTypeDef *pdev, USBD_ClassTypeDef *pclass){
  USBD_StatusTypeDef status = USBD_OK;
  if (pclass != NULL){
    /* link the class to the USB Device handle */
    pdev->pClass = pclass;
    status = USBD_OK;
  }
  else{
    status = USBD_FAIL;
  }
  return status;
}

USBD_StatusTypeDef  USBD_Start(USBD_HandleTypeDef *pdev){
  /* Start the low level driver  */
  USBD_LL_Start(pdev);
  return USBD_OK;
}

USBD_StatusTypeDef  USBD_RunTestMode(USBD_HandleTypeDef  *pdev){
  /* Prevent unused argument compilation warning */
  UNUSED(pdev);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_SetClassConfig(USBD_HandleTypeDef  *pdev, uint8_t cfgidx){
  USBD_StatusTypeDef ret = USBD_FAIL;
  if (pdev->pClass != NULL){
    /* Set configuration  and Start the Class*/
    if (pdev->pClass->Init(pdev, cfgidx) == 0U)
    {
      ret = USBD_OK;
    }
  }
  return ret;
}

USBD_StatusTypeDef USBD_ClrClassConfig(USBD_HandleTypeDef  *pdev, uint8_t cfgidx){
  /* Clear configuration  and De-initialize the Class process*/
  pdev->pClass->DeInit(pdev, cfgidx);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_SetupStage(USBD_HandleTypeDef *pdev, uint8_t *psetup){
  USBD_ParseSetupRequest(&pdev->request, psetup);
  pdev->ep0_state = USBD_EP0_SETUP;
  pdev->ep0_data_len = pdev->request.wLength;
  switch (pdev->request.bmRequest & 0x1FU){
    case USB_REQ_RECIPIENT_DEVICE:
      USBD_StdDevReq(pdev, &pdev->request);
      break;
    case USB_REQ_RECIPIENT_INTERFACE:
      USBD_StdItfReq(pdev, &pdev->request);
      break;
    case USB_REQ_RECIPIENT_ENDPOINT:
      USBD_StdEPReq(pdev, &pdev->request);
      break;
    default:
      USBD_LL_StallEP(pdev, (pdev->request.bmRequest & 0x80U));
      break;
  }
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DataOutStage(USBD_HandleTypeDef *pdev, uint8_t epnum, uint8_t *pdata){
  USBD_EndpointTypeDef *pep;
  if (epnum == 0U){
    pep = &pdev->ep_out[0];
    if (pdev->ep0_state == USBD_EP0_DATA_OUT)
    {
      if (pep->rem_length > pep->maxpacket)
      {
        pep->rem_length -= pep->maxpacket;
        USBD_CtlContinueRx(pdev, pdata,
                           (uint16_t)MIN(pep->rem_length, pep->maxpacket));
      }
      else
      {
        if ((pdev->pClass->EP0_RxReady != NULL) &&
            (pdev->dev_state == USBD_STATE_CONFIGURED))
        {
          pdev->pClass->EP0_RxReady(pdev);
        }
        USBD_CtlSendStatus(pdev);
      }
    }
    else
    {
      if (pdev->ep0_state == USBD_EP0_STATUS_OUT)
      {
        /*
         * STATUS PHASE completed, update ep0_state to idle
         */
        pdev->ep0_state = USBD_EP0_IDLE;
        USBD_LL_StallEP(pdev, 0U);
      }
    }
  }
  else if ((pdev->pClass->DataOut != NULL) &&
           (pdev->dev_state == USBD_STATE_CONFIGURED)){
    pdev->pClass->DataOut(pdev, epnum);
  }
  else{
    /* should never be in this condition */
    return USBD_FAIL;
  }
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DataInStage(USBD_HandleTypeDef *pdev, uint8_t epnum, uint8_t *pdata){
  USBD_EndpointTypeDef *pep;
  if (epnum == 0U){
    pep = &pdev->ep_in[0];
    if (pdev->ep0_state == USBD_EP0_DATA_IN)
    {
      if (pep->rem_length > pep->maxpacket)
      {
        pep->rem_length -= pep->maxpacket;
        USBD_CtlContinueSendData(pdev, pdata, (uint16_t)pep->rem_length);
        /* Prepare endpoint for premature end of transfer */
        USBD_LL_PrepareReceive(pdev, 0U, NULL, 0U);
      }
      else
      {
        /* last packet is MPS multiple, so send ZLP packet */
        if ((pep->total_length % pep->maxpacket == 0U) &&
            (pep->total_length >= pep->maxpacket) &&
            (pep->total_length < pdev->ep0_data_len))
        {
          USBD_CtlContinueSendData(pdev, NULL, 0U);
          pdev->ep0_data_len = 0U;
          /* Prepare endpoint for premature end of transfer */
          USBD_LL_PrepareReceive(pdev, 0U, NULL, 0U);
        }
        else
        {
          if ((pdev->pClass->EP0_TxSent != NULL) &&
              (pdev->dev_state == USBD_STATE_CONFIGURED))
          {
            pdev->pClass->EP0_TxSent(pdev);
          }
          USBD_LL_StallEP(pdev, 0x80U);
          USBD_CtlReceiveStatus(pdev);
        }
      }
    }
    else
    {
      if ((pdev->ep0_state == USBD_EP0_STATUS_IN) ||
          (pdev->ep0_state == USBD_EP0_IDLE))
      {
        USBD_LL_StallEP(pdev, 0x80U);
      }
    }
    if (pdev->dev_test_mode == 1U)
    {
      USBD_RunTestMode(pdev);
      pdev->dev_test_mode = 0U;
    }
  }
  else if ((pdev->pClass->DataIn != NULL) &&
           (pdev->dev_state == USBD_STATE_CONFIGURED)){
    pdev->pClass->DataIn(pdev, epnum);
  }
  else{
    /* should never be in this condition */
    return USBD_FAIL;
  }
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Reset(USBD_HandleTypeDef *pdev){
  /* Open EP0 OUT */
  USBD_LL_OpenEP(pdev, 0x00U, USBD_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);
  pdev->ep_out[0x00U & 0xFU].is_used = 1U;
  pdev->ep_out[0].maxpacket = USB_MAX_EP0_SIZE;
  /* Open EP0 IN */
  USBD_LL_OpenEP(pdev, 0x80U, USBD_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);
  pdev->ep_in[0x80U & 0xFU].is_used = 1U;
  pdev->ep_in[0].maxpacket = USB_MAX_EP0_SIZE;
  /* Upon Reset call user call back */
  pdev->dev_state = USBD_STATE_DEFAULT;
  pdev->ep0_state = USBD_EP0_IDLE;
  pdev->dev_config = 0U;
  pdev->dev_remote_wakeup = 0U;
  if (pdev->pClassData){
    pdev->pClass->DeInit(pdev, (uint8_t)pdev->dev_config);
  }
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_SetSpeed(USBD_HandleTypeDef *pdev, USBD_SpeedTypeDef speed){
  pdev->dev_speed = speed;
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Suspend(USBD_HandleTypeDef *pdev){
  pdev->dev_old_state =  pdev->dev_state;
  pdev->dev_state  = USBD_STATE_SUSPENDED;
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Resume(USBD_HandleTypeDef *pdev){
  if (pdev->dev_state == USBD_STATE_SUSPENDED){
    pdev->dev_state = pdev->dev_old_state;
  }
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_SOF(USBD_HandleTypeDef *pdev){
  if (pdev->dev_state == USBD_STATE_CONFIGURED){
    if (pdev->pClass->SOF != NULL)
    {
      pdev->pClass->SOF(pdev);
    }
  }
  return USBD_OK;
}

PCD_HandleTypeDef hpcd_USB_FS;
static USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status);

void HAL_PCD_MspInit(PCD_HandleTypeDef* pcdHandle){
  if(pcdHandle->Instance==USB)  {
    __HAL_RCC_USB_CLK_ENABLE();
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  }
}

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd){
  USBD_LL_SetupStage((USBD_HandleTypeDef*)hpcd->pData, (uint8_t *)hpcd->Setup);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){
  USBD_LL_DataOutStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){
  USBD_LL_DataInStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd){
  USBD_LL_SOF((USBD_HandleTypeDef*)hpcd->pData);
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd){
  USBD_SpeedTypeDef speed = USBD_SPEED_FULL;
  if ( hpcd->Init.speed != PCD_SPEED_FULL)  {
    Error_Handler();
  }
  USBD_LL_SetSpeed((USBD_HandleTypeDef*)hpcd->pData, speed);
  USBD_LL_Reset((USBD_HandleTypeDef*)hpcd->pData);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd){
  USBD_LL_Suspend((USBD_HandleTypeDef*)hpcd->pData);
  if (hpcd->Init.low_power_enable)  {
    SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
  }
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd){
  USBD_LL_Resume((USBD_HandleTypeDef*)hpcd->pData);
}

HAL_StatusTypeDef  HAL_PCDEx_PMAConfig(PCD_HandleTypeDef *hpcd, uint16_t ep_addr, uint16_t ep_kind, uint32_t pmaadress){
  PCD_EPTypeDef *ep;
  if ((0x80U & ep_addr) == 0x80U)  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
  }
  else  {
    ep = &hpcd->OUT_ep[ep_addr];
  }
  if (ep_kind == PCD_SNG_BUF)  {
    ep->doublebuffer = 0U;
    ep->pmaadress = (uint16_t)pmaadress;
  }
  else {
    ep->doublebuffer = 1U;
    ep->pmaaddr0 = (uint16_t)(pmaadress & 0xFFFFU);
    ep->pmaaddr1 = (uint16_t)((pmaadress & 0xFFFF0000U) >> 16);
  }
  return HAL_OK;
}

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev){
  hpcd_USB_FS.pData = pdev;
  pdev->pData = &hpcd_USB_FS;
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)  {
    Error_Handler( );
  }
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x00 , PCD_SNG_BUF, 0x18);
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x80 , PCD_SNG_BUF, 0x58);
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x81 , PCD_SNG_BUF, 0x100);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev){
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
  hal_status = HAL_PCD_Start(pdev->pData);
  usb_status =  USBD_Get_USB_Status(hal_status);
  return usb_status;
}

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps){
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
  hal_status = HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type);
  usb_status =  USBD_Get_USB_Status(hal_status);
  return usb_status;
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr){
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
  hal_status = HAL_PCD_EP_Close(pdev->pData, ep_addr);
  usb_status =  USBD_Get_USB_Status(hal_status);
  return usb_status;
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr){
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
  hal_status = HAL_PCD_EP_SetStall(pdev->pData, ep_addr);
  usb_status =  USBD_Get_USB_Status(hal_status);
  return usb_status;
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr){
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
  hal_status = HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);
  usb_status =  USBD_Get_USB_Status(hal_status);
  return usb_status;
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr){
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef*) pdev->pData;
  if((ep_addr & 0x80) == 0x80)  {
    return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
  }
  else  {
    return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
  }
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr){
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
  hal_status = HAL_PCD_SetAddress(pdev->pData, dev_addr);
  usb_status =  USBD_Get_USB_Status(hal_status);
  return usb_status;
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint16_t size){
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
  hal_status = HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
  usb_status =  USBD_Get_USB_Status(hal_status);
  return usb_status;
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint16_t size){
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
  hal_status = HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);
  usb_status =  USBD_Get_USB_Status(hal_status);
  return usb_status;
}

void *USBD_static_malloc(uint32_t size){
  static uint32_t mem[(sizeof(USBD_HID_HandleTypeDef)/4)+1];/* On 32-bit boundary */
  return mem;
}

USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status){
  USBD_StatusTypeDef usb_status = USBD_OK;
  switch (hal_status)  {
    case HAL_OK :
      usb_status = USBD_OK;
      break;
    case HAL_ERROR :
      usb_status = USBD_FAIL;
      break;
    case HAL_BUSY :
      usb_status = USBD_BUSY;
      break;
    case HAL_TIMEOUT :
      usb_status = USBD_FAIL;
      break;
    default :
      usb_status = USBD_FAIL;
      break;
  }
  return usb_status;
}
static void USBD_GetDescriptor(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void USBD_SetAddress(USBD_HandleTypeDef *pdev,    USBD_SetupReqTypedef *req);
static void USBD_SetConfig(USBD_HandleTypeDef *pdev,     USBD_SetupReqTypedef *req);
static void USBD_GetConfig(USBD_HandleTypeDef *pdev,     USBD_SetupReqTypedef *req);
static void USBD_GetStatus(USBD_HandleTypeDef *pdev,     USBD_SetupReqTypedef *req);
static void USBD_SetFeature(USBD_HandleTypeDef *pdev,    USBD_SetupReqTypedef *req);
static void USBD_ClrFeature(USBD_HandleTypeDef *pdev,    USBD_SetupReqTypedef *req);
static uint8_t USBD_GetLen(uint8_t *buf);
USBD_StatusTypeDef  USBD_StdDevReq(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req){
  USBD_StatusTypeDef ret = USBD_OK;
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
    case USB_REQ_TYPE_VENDOR:
      pdev->pClass->Setup(pdev, req);
      break;
    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest){
        case USB_REQ_GET_DESCRIPTOR:
          USBD_GetDescriptor(pdev, req);
          break;
        case USB_REQ_SET_ADDRESS:
          USBD_SetAddress(pdev, req);
          break;
        case USB_REQ_SET_CONFIGURATION:
          USBD_SetConfig(pdev, req);
          break;
        case USB_REQ_GET_CONFIGURATION:
          USBD_GetConfig(pdev, req);
          break;
        case USB_REQ_GET_STATUS:
          USBD_GetStatus(pdev, req);
          break;
        case USB_REQ_SET_FEATURE:
          USBD_SetFeature(pdev, req);
          break;
        case USB_REQ_CLEAR_FEATURE:
          USBD_ClrFeature(pdev, req);
          break;
        default:
          USBD_CtlError(pdev, req);
          break;
      }
      break;
    default:
      USBD_CtlError(pdev, req);
      break;
  }
  return ret;
}

USBD_StatusTypeDef  USBD_StdItfReq(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef  *req){
  USBD_StatusTypeDef ret = USBD_OK;
  switch (req->bmRequest & USB_REQ_TYPE_MASK)  {
    case USB_REQ_TYPE_CLASS:
    case USB_REQ_TYPE_VENDOR:
    case USB_REQ_TYPE_STANDARD:
      switch (pdev->dev_state)      {
        case USBD_STATE_DEFAULT:
        case USBD_STATE_ADDRESSED:
        case USBD_STATE_CONFIGURED:
          if (LOBYTE(req->wIndex) <= USBD_MAX_NUM_INTERFACES)          {
            ret = (USBD_StatusTypeDef)pdev->pClass->Setup(pdev, req);
            if ((req->wLength == 0U) && (ret == USBD_OK))
            {
              USBD_CtlSendStatus(pdev);
            }
          }
          else          {
            USBD_CtlError(pdev, req);
          }
          break;
        default:
          USBD_CtlError(pdev, req);
          break;
      }
      break;
    default:
      USBD_CtlError(pdev, req);
      break;
  }
  return USBD_OK;
}

USBD_StatusTypeDef  USBD_StdEPReq (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef  *req){
  USBD_EndpointTypeDef *pep;
  uint8_t   ep_addr;
  USBD_StatusTypeDef ret = USBD_OK;
  ep_addr  = LOBYTE(req->wIndex);
  switch (req->bmRequest & USB_REQ_TYPE_MASK)  {
    case USB_REQ_TYPE_CLASS:
    case USB_REQ_TYPE_VENDOR:
      pdev->pClass->Setup(pdev, req);
      break;
    case USB_REQ_TYPE_STANDARD:
      /* Check if it is a class request */
      if ((req->bmRequest & 0x60U) == 0x20U)      {
        ret = (USBD_StatusTypeDef)pdev->pClass->Setup(pdev, req);
        return ret;
      }
      switch (req->bRequest)      {
        case USB_REQ_SET_FEATURE:
          switch (pdev->dev_state)          {
            case USBD_STATE_ADDRESSED:
              if ((ep_addr != 0x00U) && (ep_addr != 0x80U))
              {
                USBD_LL_StallEP(pdev, ep_addr);
                USBD_LL_StallEP(pdev, 0x80U);
              }
              else
              {
                USBD_CtlError(pdev, req);
              }
              break;
            case USBD_STATE_CONFIGURED:
              if (req->wValue == USB_FEATURE_EP_HALT)
              {
                if ((ep_addr != 0x00U) &&
                    (ep_addr != 0x80U) && (req->wLength == 0x00U))
                {
                  USBD_LL_StallEP(pdev, ep_addr);
                }
              }
              USBD_CtlSendStatus(pdev);
              break;
            default:
              USBD_CtlError(pdev, req);
              break;
          }
          break;
        case USB_REQ_CLEAR_FEATURE:
          switch (pdev->dev_state)
          {
            case USBD_STATE_ADDRESSED:
              if ((ep_addr != 0x00U) && (ep_addr != 0x80U))
              {
                USBD_LL_StallEP(pdev, ep_addr);
                USBD_LL_StallEP(pdev, 0x80U);
              }
              else
              {
                USBD_CtlError(pdev, req);
              }
              break;
            case USBD_STATE_CONFIGURED:
              if (req->wValue == USB_FEATURE_EP_HALT)
              {
                if ((ep_addr & 0x7FU) != 0x00U)
                {
                  USBD_LL_ClearStallEP(pdev, ep_addr);
                }
                USBD_CtlSendStatus(pdev);
              }
              break;
            default:
              USBD_CtlError(pdev, req);
              break;
          }
          break;
        case USB_REQ_GET_STATUS:
          switch (pdev->dev_state)
          {
            case USBD_STATE_ADDRESSED:
              if ((ep_addr != 0x00U) && (ep_addr != 0x80U))
              {
                USBD_CtlError(pdev, req);
                break;
              }
              pep = ((ep_addr & 0x80U) == 0x80U) ? &pdev->ep_in[ep_addr & 0x7FU] : \
                    &pdev->ep_out[ep_addr & 0x7FU];
              pep->status = 0x0000U;
              USBD_CtlSendData(pdev, (uint8_t *)(void *)&pep->status, 2U);
              break;
            case USBD_STATE_CONFIGURED:
              if ((ep_addr & 0x80U) == 0x80U)
              {
                if (pdev->ep_in[ep_addr & 0xFU].is_used == 0U)
                {
                  USBD_CtlError(pdev, req);
                  break;
                }
              }
              else
              {
                if (pdev->ep_out[ep_addr & 0xFU].is_used == 0U)
                {
                  USBD_CtlError(pdev, req);
                  break;
                }
              }
              pep = ((ep_addr & 0x80U) == 0x80U) ? &pdev->ep_in[ep_addr & 0x7FU] : \
                    &pdev->ep_out[ep_addr & 0x7FU];
              if ((ep_addr == 0x00U) || (ep_addr == 0x80U))
              {
                pep->status = 0x0000U;
              }
              else if (USBD_LL_IsStallEP(pdev, ep_addr))
              {
                pep->status = 0x0001U;
              }
              else
              {
                pep->status = 0x0000U;
              }
              USBD_CtlSendData(pdev, (uint8_t *)(void *)&pep->status, 2U);
              break;
            default:
              USBD_CtlError(pdev, req);
              break;
          }
          break;
        default:
          USBD_CtlError(pdev, req);
          break;
      }
      break;
    default:
      USBD_CtlError(pdev, req);
      break;
  }
  return ret;
}

static void USBD_GetDescriptor    (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req){
  uint16_t len = 0U;
  uint8_t *pbuf = NULL;
  uint8_t err = 0U;
  switch (req->wValue >> 8)
  {
    case USB_DESC_TYPE_DEVICE:
      pbuf = pdev->pDesc->GetDeviceDescriptor(pdev->dev_speed, &len);
      break;
    case USB_DESC_TYPE_CONFIGURATION:
      if (pdev->dev_speed == USBD_SPEED_HIGH)
      {
        pbuf = pdev->pClass->GetHSConfigDescriptor(&len);
        pbuf[1] = USB_DESC_TYPE_CONFIGURATION;
      }
      else
      {
        pbuf = pdev->pClass->GetFSConfigDescriptor(&len);
        pbuf[1] = USB_DESC_TYPE_CONFIGURATION;
      }
      break;
    case USB_DESC_TYPE_STRING:
      switch ((uint8_t)(req->wValue))
      {
        case USBD_IDX_LANGID_STR:
          if (pdev->pDesc->GetLangIDStrDescriptor != NULL)
          {
            pbuf = pdev->pDesc->GetLangIDStrDescriptor(pdev->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(pdev, req);
            err++;
          }
          break;
        case USBD_IDX_MFC_STR:
          if (pdev->pDesc->GetManufacturerStrDescriptor != NULL)
          {
            pbuf = pdev->pDesc->GetManufacturerStrDescriptor(pdev->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(pdev, req);
            err++;
          }
          break;
        case USBD_IDX_PRODUCT_STR:
          if (pdev->pDesc->GetProductStrDescriptor != NULL)
          {
            pbuf = pdev->pDesc->GetProductStrDescriptor(pdev->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(pdev, req);
            err++;
          }
          break;
        case USBD_IDX_SERIAL_STR:
          if (pdev->pDesc->GetSerialStrDescriptor != NULL)
          {
            pbuf = pdev->pDesc->GetSerialStrDescriptor(pdev->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(pdev, req);
            err++;
          }
          break;
        case USBD_IDX_CONFIG_STR:
          if (pdev->pDesc->GetConfigurationStrDescriptor != NULL)
          {
            pbuf = pdev->pDesc->GetConfigurationStrDescriptor(pdev->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(pdev, req);
            err++;
          }
          break;
        case USBD_IDX_INTERFACE_STR:
          if (pdev->pDesc->GetInterfaceStrDescriptor != NULL)
          {
            pbuf = pdev->pDesc->GetInterfaceStrDescriptor(pdev->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(pdev, req);
            err++;
          }
          break;
        default:
          USBD_CtlError(pdev, req);
          err++;
      }
      break;
    case USB_DESC_TYPE_DEVICE_QUALIFIER:
      if (pdev->dev_speed == USBD_SPEED_HIGH)
      {
        pbuf = pdev->pClass->GetDeviceQualifierDescriptor(&len);
      }
      else
      {
        USBD_CtlError(pdev, req);
        err++;
      }
      break;
    case USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION:
      if (pdev->dev_speed == USBD_SPEED_HIGH)
      {
        pbuf = pdev->pClass->GetOtherSpeedConfigDescriptor(&len);
        pbuf[1] = USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION;
      }
      else
      {
        USBD_CtlError(pdev, req);
        err++;
      }
      break;
    default:
      USBD_CtlError(pdev, req);
      err++;
      break;
  }
  if (err != 0U)
  {
    return;
  }
  else
  {
    if ((len != 0U) && (req->wLength != 0U))
    {
      len = MIN(len, req->wLength);
      (void)USBD_CtlSendData(pdev, pbuf, len);
    }
    if (req->wLength == 0U)
    {
      (void)USBD_CtlSendStatus(pdev);
    }
  }
}
static void USBD_SetAddress		  (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req){
  uint8_t  dev_addr;
  if ((req->wIndex == 0U) && (req->wLength == 0U) && (req->wValue < 128U))
  {
    dev_addr = (uint8_t)(req->wValue) & 0x7FU;
    if (pdev->dev_state == USBD_STATE_CONFIGURED)
    {
      USBD_CtlError(pdev, req);
    }
    else
    {
      pdev->dev_address = dev_addr;
      USBD_LL_SetUSBAddress(pdev, dev_addr);
      USBD_CtlSendStatus(pdev);
      if (dev_addr != 0U)
      {
        pdev->dev_state = USBD_STATE_ADDRESSED;
      }
      else
      {
        pdev->dev_state = USBD_STATE_DEFAULT;
      }
    }
  }
  else
  {
    USBD_CtlError(pdev, req);
  }
}

static void USBD_SetConfig		  (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req){
  static uint8_t cfgidx;
  cfgidx = (uint8_t)(req->wValue);
  if (cfgidx > USBD_MAX_NUM_CONFIGURATION)
  {
    USBD_CtlError(pdev, req);
  }
  else
  {
    switch (pdev->dev_state)
    {
      case USBD_STATE_ADDRESSED:
        if (cfgidx)
        {
          pdev->dev_config = cfgidx;
          pdev->dev_state = USBD_STATE_CONFIGURED;
          if (USBD_SetClassConfig(pdev, cfgidx) == USBD_FAIL)
          {
            USBD_CtlError(pdev, req);
            return;
          }
          USBD_CtlSendStatus(pdev);
        }
        else
        {
          USBD_CtlSendStatus(pdev);
        }
        break;
      case USBD_STATE_CONFIGURED:
        if (cfgidx == 0U)
        {
          pdev->dev_state = USBD_STATE_ADDRESSED;
          pdev->dev_config = cfgidx;
          USBD_ClrClassConfig(pdev, cfgidx);
          USBD_CtlSendStatus(pdev);
        }
        else if (cfgidx != pdev->dev_config)
        {
          /* Clear old configuration */
          USBD_ClrClassConfig(pdev, (uint8_t)pdev->dev_config);
          /* set new configuration */
          pdev->dev_config = cfgidx;
          if (USBD_SetClassConfig(pdev, cfgidx) == USBD_FAIL)
          {
            USBD_CtlError(pdev, req);
            return;
          }
          USBD_CtlSendStatus(pdev);
        }
        else
        {
          USBD_CtlSendStatus(pdev);
        }
        break;
      default:
        USBD_CtlError(pdev, req);
        USBD_ClrClassConfig(pdev, cfgidx);
        break;
    }
  }
}

static void USBD_GetConfig		  (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req){
  if (req->wLength != 1U)
  {
    USBD_CtlError(pdev, req);
  }
  else
  {
    switch (pdev->dev_state)
    {
      case USBD_STATE_DEFAULT:
      case USBD_STATE_ADDRESSED:
        pdev->dev_default_config = 0U;
        USBD_CtlSendData(pdev, (uint8_t *)(void *)&pdev->dev_default_config, 1U);
        break;
      case USBD_STATE_CONFIGURED:
        USBD_CtlSendData(pdev, (uint8_t *)(void *)&pdev->dev_config, 1U);
        break;
      default:
        USBD_CtlError(pdev, req);
        break;
    }
  }
}

static void USBD_GetStatus		  (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req){
  switch (pdev->dev_state)
  {
    case USBD_STATE_DEFAULT:
    case USBD_STATE_ADDRESSED:
    case USBD_STATE_CONFIGURED:
      if (req->wLength != 0x2U)
      {
        USBD_CtlError(pdev, req);
        break;
      }
      pdev->dev_config_status = USB_CONFIG_SELF_POWERED;
      if (pdev->dev_remote_wakeup)
      {
        pdev->dev_config_status |= USB_CONFIG_REMOTE_WAKEUP;
      }
      USBD_CtlSendData(pdev, (uint8_t *)(void *)&pdev->dev_config_status, 2U);
      break;
    default:
      USBD_CtlError(pdev, req);
      break;
  }
}

static void USBD_SetFeature		  (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req){
  if (req->wValue == USB_FEATURE_REMOTE_WAKEUP)
  {
    pdev->dev_remote_wakeup = 1U;
    USBD_CtlSendStatus(pdev);
  }
}

static void USBD_ClrFeature       (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req){
  switch (pdev->dev_state)
  {
    case USBD_STATE_DEFAULT:
    case USBD_STATE_ADDRESSED:
    case USBD_STATE_CONFIGURED:
      if (req->wValue == USB_FEATURE_REMOTE_WAKEUP)
      {
        pdev->dev_remote_wakeup = 0U;
        USBD_CtlSendStatus(pdev);
      }
      break;
    default:
      USBD_CtlError(pdev, req);
      break;
  }
}

void USBD_CtlError				  (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req){
  USBD_LL_StallEP(pdev, 0x80U);
  USBD_LL_StallEP(pdev, 0U);
}

void USBD_ParseSetupRequest		  (USBD_SetupReqTypedef *req, uint8_t *pdata){
  req->bmRequest = *(uint8_t *)(pdata);
  req->bRequest = *(uint8_t *)(pdata + 1U);
  req->wValue = SWAPBYTE(pdata + 2U);
  req->wIndex = SWAPBYTE(pdata + 4U);
  req->wLength = SWAPBYTE(pdata + 6U);
}

void USBD_GetString(uint8_t *desc, uint8_t *unicode, uint16_t *len){
  uint8_t idx = 0U;
  if (desc != NULL)
  {
    *len = (uint16_t)USBD_GetLen(desc) * 2U + 2U;
    unicode[idx++] = *(uint8_t *)(void *)len;
    unicode[idx++] = USB_DESC_TYPE_STRING;
    while (*desc != '\0')
    {
      unicode[idx++] = *desc++;
      unicode[idx++] =  0U;
    }
  }
}

static uint8_t USBD_GetLen(uint8_t *buf){
  uint8_t  len = 0U;
  while (*buf != '\0')
  {
    len++;
    buf++;
  }
  return len;
}
static void Get_SerialNum(void);
static void IntToUnicode(uint32_t value, uint8_t * pbuf, uint8_t len);

uint8_t * USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);

USBD_DescriptorsTypeDef FS_Desc = {
  USBD_FS_DeviceDescriptor,
  USBD_FS_LangIDStrDescriptor,
  USBD_FS_ManufacturerStrDescriptor,
  USBD_FS_ProductStrDescriptor,
  USBD_FS_SerialStrDescriptor,
  USBD_FS_ConfigStrDescriptor,
  USBD_FS_InterfaceStrDescriptor
};
__ALIGN_BEGIN uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = {
  0x12,                       /*bLength */
  USB_DESC_TYPE_DEVICE,       /*bDescriptorType*/
  0x00,                       /*bcdUSB */
  0x02,
  0x00,                       /*bDeviceClass*/
  0x00,                       /*bDeviceSubClass*/
  0x00,                       /*bDeviceProtocol*/
  USB_MAX_EP0_SIZE,           /*bMaxPacketSize*/
  LOBYTE(USBD_VID),           /*idVendor*/
  HIBYTE(USBD_VID),           /*idVendor*/
  LOBYTE(USBD_PID_FS),        /*idProduct*/
  HIBYTE(USBD_PID_FS),        /*idProduct*/
  0x00,                       /*bcdDevice rel. 2.00*/
  0x02,
  USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
  USBD_IDX_PRODUCT_STR,       /*Index of product string*/
  USBD_IDX_SERIAL_STR,        /*Index of serial number string*/
  USBD_MAX_NUM_CONFIGURATION  /*bNumConfigurations*/
};
__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END ={
     USB_LEN_LANGID_STR_DESC,
     USB_DESC_TYPE_STRING,
     LOBYTE(USBD_LANGID_STRING),
     HIBYTE(USBD_LANGID_STRING)
};
__ALIGN_BEGIN uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

__ALIGN_BEGIN uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] __ALIGN_END = {
  USB_SIZ_STRING_SERIAL,
  USB_DESC_TYPE_STRING,
};
uint8_t * USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length){
  UNUSED(speed);
  *length = sizeof(USBD_FS_DeviceDesc);
  return USBD_FS_DeviceDesc;
}

uint8_t * USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length){
  UNUSED(speed);
  *length = sizeof(USBD_LangIDDesc);
  return USBD_LangIDDesc;
}

uint8_t * USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length){
  USBD_GetString((uint8_t *)USBD_PRODUCT_STRING_FS, USBD_StrDesc, length);
  return USBD_StrDesc;
}

uint8_t * USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length){
  UNUSED(speed);
  USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

uint8_t * USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length){
  UNUSED(speed);
  *length = USB_SIZ_STRING_SERIAL;
  Get_SerialNum();
  return (uint8_t *) USBD_StringSerial;
}

uint8_t * USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length){
  USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING_FS, USBD_StrDesc, length);
  return USBD_StrDesc;
}

uint8_t * USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length){
  USBD_GetString((uint8_t *)USBD_INTERFACE_STRING_FS, USBD_StrDesc, length);
  return USBD_StrDesc;
}

static void Get_SerialNum(void){
  uint32_t deviceserial0, deviceserial1, deviceserial2;

  deviceserial0 = *(uint32_t *) DEVICE_ID1;
  deviceserial1 = *(uint32_t *) DEVICE_ID2;
  deviceserial2 = *(uint32_t *) DEVICE_ID3;

  deviceserial0 += deviceserial2;
  if (deviceserial0 != 0)  {
    IntToUnicode(deviceserial0, &USBD_StringSerial[2], 8);
    IntToUnicode(deviceserial1, &USBD_StringSerial[18], 4);
  }
}

static void IntToUnicode(uint32_t value, uint8_t * pbuf, uint8_t len){
  uint8_t idx = 0;
  for (idx = 0; idx < len; idx++)  {
    if (((value >> 28)) < 0xA)    {
      pbuf[2 * idx] = (value >> 28) + '0';
    }
    else    {
      pbuf[2 * idx] = (value >> 28) + 'A' - 10;
    }
    value = value << 4;
    pbuf[2 * idx + 1] = 0;
  }
}

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
/* USB HID device FS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_CfgFSDesc[USB_HID_CONFIG_DESC_SIZ]  __ALIGN_END ={
  0x09, /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  USB_HID_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x01,         /*bNumInterfaces: 1 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x00,         /*iConfiguration: Index of string descriptor describing the configuration*/
  0xE0,         /*bmAttributes: bus powered and Support Remote Wake-up */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
    /* 09 */
  0x09,         /*bLength: Interface Descriptor size*/
  USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
  0x00,         /*bInterfaceNumber: Number of Interface*/
  0x00,         /*bAlternateSetting: Alternate setting*/
  0x01,         /*bNumEndpoints*/
  0x03,         /*bInterfaceClass: HID*/
  0x01,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
  0x01,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
  0,            /*iInterface: Index of string descriptor*/

  /* 18 */
  0x09,         /*bLength: HID Descriptor size*/
  HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
  0x11,         /*bcdHID: HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  HID_MOUSE_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,

  /* 27 */
  0x07,          /*bLength: Endpoint Descriptor size*/
  USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

  HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
  0x03,          /*bmAttributes: Interrupt endpoint*/
  HID_EPIN_SIZE, /*wMaxPacketSize: 4 Byte max */
  0x00,
  HID_FS_BINTERVAL,          /*bInterval: Polling Interval */
  /* 34 */
};

/* USB HID device HS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_CfgHSDesc[USB_HID_CONFIG_DESC_SIZ]  __ALIGN_END ={
  0x09, /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  USB_HID_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x01,         /*bNumInterfaces: 1 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x00,         /*iConfiguration: Index of string descriptor describing the configuration*/
  0xE0,         /*bmAttributes: bus powered and Support Remote Wake-up */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
    /* 09 */
  0x09,         /*bLength: Interface Descriptor size*/
  USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
  0x00,         /*bInterfaceNumber: Number of Interface*/
  0x00,         /*bAlternateSetting: Alternate setting*/
  0x01,         /*bNumEndpoints*/
  0x03,         /*bInterfaceClass: HID*/
  0x01,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
  0x02,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
  0,            /*iInterface: Index of string descriptor*/

  /* 18 */
  0x09,         /*bLength: HID Descriptor size*/
  HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
  0x11,         /*bcdHID: HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  HID_MOUSE_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,

  /* 27 */
  0x07,          /*bLength: Endpoint Descriptor size*/
  USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

  HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
  0x03,          /*bmAttributes: Interrupt endpoint*/
  HID_EPIN_SIZE, /*wMaxPacketSize: 4 Byte max */
  0x00,
  HID_HS_BINTERVAL,          /*bInterval: Polling Interval */
  /* 34 */
};

/* USB HID device Other Speed Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_OtherSpeedCfgDesc[USB_HID_CONFIG_DESC_SIZ]  __ALIGN_END ={
  0x09, /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  USB_HID_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x01,         /*bNumInterfaces: 1 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x00,         /*iConfiguration: Index of string descriptor describing
  the configuration*/
  0xE0,         /*bmAttributes: bus powered and Support Remote Wake-up */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
    /* 09 */
  0x09,         /*bLength: Interface Descriptor size*/
  USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
  0x00,         /*bInterfaceNumber: Number of Interface*/
  0x00,         /*bAlternateSetting: Alternate setting*/
  0x01,         /*bNumEndpoints*/
  0x03,         /*bInterfaceClass: HID*/
  0x01,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
  0x02,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
  0,            /*iInterface: Index of string descriptor*/

  /* 18 */
  0x09,         /*bLength: HID Descriptor size*/
  HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
  0x11,         /*bcdHID: HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  HID_MOUSE_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,

  /* 27 */
  0x07,          /*bLength: Endpoint Descriptor size*/
  USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

  HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
  0x03,          /*bmAttributes: Interrupt endpoint*/
  HID_EPIN_SIZE, /*wMaxPacketSize: 4 Byte max */
  0x00,
  HID_FS_BINTERVAL,          /*bInterval: Polling Interval */
  /* 34 */
};

__ALIGN_BEGIN static uint8_t USBD_HID_Desc[USB_HID_DESC_SIZ]  __ALIGN_END  ={
  /* 18 */
  0x09,         /*bLength: HID Descriptor size*/
  HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
  0x11,         /*bcdHID: HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  HID_MOUSE_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC]  __ALIGN_END ={
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};
__ALIGN_BEGIN static uint8_t HID_MOUSE_ReportDesc[HID_MOUSE_REPORT_DESC_SIZE]  __ALIGN_END ={
	0x05, 0x01,                    /* USAGE_PAGE (Generic Desktop)*/
	0x09, 0x06,                    /* USAGE (Keyboard)*/
	0xa1, 0x01,                    /* COLLECTION (Application)*/
	0x05, 0x07,                    /*   USAGE_PAGE (Keyboard)*/
	0x19, 0xe0,                    /*   USAGE_MINIMUM (Keyboard LeftControl)*/
	0x29, 0xe7,                    /*   USAGE_MAXIMUM (Keyboard Right GUI)*/
	0x15, 0x00,                    /*   LOGICAL_MINIMUM (0)*/
	0x25, 0x01,                    /*   LOGICAL_MAXIMUM (1)*/
	0x75, 0x01,                    /*   REPORT_SIZE (1)*/
	0x95, 0x08,                    /*   REPORT_COUNT (8)*/
	0x81, 0x02,                    /*   INPUT (Data,Var,Abs)*/
	0x95, 0x01,                    /*   REPORT_COUNT (1)*/
	0x75, 0x08,                    /*   REPORT_SIZE (8)*/
	0x81, 0x03,                    /*   INPUT (Cnst,Var,Abs)*/
	0x95, 0x05,                    /*   REPORT_COUNT (5)*/
	0x75, 0x01,                    /*   REPORT_SIZE (1)*/
	0x05, 0x08,                    /*   USAGE_PAGE (LEDs)*/
	0x19, 0x01,                    /*   USAGE_MINIMUM (Num Lock)*/
	0x29, 0x05,                    /*   USAGE_MAXIMUM (Kana)*/
	0x91, 0x02,                    /*   OUTPUT (Data,Var,Abs)*/
	0x95, 0x01,                    /*   REPORT_COUNT (1)*/
	0x75, 0x03,                    /*   REPORT_SIZE (3)*/
	0x91, 0x03,                    /*   OUTPUT (Cnst,Var,Abs)*/
	0x95, 0x06,                    /*   REPORT_COUNT (6)*/
	0x75, 0x08,                    /*   REPORT_SIZE (8)*/
	0x15, 0x00,                    /*   LOGICAL_MINIMUM (0)*/
	0x25, 0x65,                    /*   LOGICAL_MAXIMUM (101)*/
	0x05, 0x07,                    /*   USAGE_PAGE (Keyboard)*/
	0x19, 0x00,                    /*   USAGE_MINIMUM (Reserved (no event indicated))*/
	0x29, 0x65,                    /*   USAGE_MAXIMUM (Keyboard Application)*/
	0x81, 0x00,                    /*   INPUT (Data,Ary,Abs)*/
	0xc0                           /* END_COLLECTION*/
};
static uint8_t  USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx){
  /* Open EP IN */
  USBD_LL_OpenEP(pdev, HID_EPIN_ADDR, USBD_EP_TYPE_INTR, HID_EPIN_SIZE);
  pdev->ep_in[HID_EPIN_ADDR & 0xFU].is_used = 1U;

  pdev->pClassData = USBD_malloc(sizeof(USBD_HID_HandleTypeDef));

  if (pdev->pClassData == NULL)
  {
    return USBD_FAIL;
  }

  ((USBD_HID_HandleTypeDef *)pdev->pClassData)->state = HID_IDLE;

  return USBD_OK;
}

static uint8_t  USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx){
  /* Close HID EPs */
  USBD_LL_CloseEP(pdev, HID_EPIN_ADDR);
  pdev->ep_in[HID_EPIN_ADDR & 0xFU].is_used = 0U;

  /* FRee allocated memory */
  if (pdev->pClassData != NULL)
  {
    pdev->pClassData = NULL;
  }

  return USBD_OK;
}

static uint8_t  USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req){
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *) pdev->pClassData;
  uint16_t len = 0U;
  uint8_t *pbuf = NULL;
  uint16_t status_info = 0U;
  USBD_StatusTypeDef ret = USBD_OK;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS :
      switch (req->bRequest)
      {
        case HID_REQ_SET_PROTOCOL:
          hhid->Protocol = (uint8_t)(req->wValue);
          break;

        case HID_REQ_GET_PROTOCOL:
          USBD_CtlSendData(pdev, (uint8_t *)(void *)&hhid->Protocol, 1U);
          break;

        case HID_REQ_SET_IDLE:
          hhid->IdleState = (uint8_t)(req->wValue >> 8);
          break;

        case HID_REQ_GET_IDLE:
          USBD_CtlSendData(pdev, (uint8_t *)(void *)&hhid->IdleState, 1U);
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;
    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            USBD_CtlSendData(pdev, (uint8_t *)(void *)&status_info, 2U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_DESCRIPTOR:
          if (req->wValue >> 8 == HID_REPORT_DESC)
          {
            len = MIN(HID_MOUSE_REPORT_DESC_SIZE, req->wLength);
            pbuf = HID_MOUSE_ReportDesc;
          }
          else if (req->wValue >> 8 == HID_DESCRIPTOR_TYPE)
          {
            pbuf = USBD_HID_Desc;
            len = MIN(USB_HID_DESC_SIZ, req->wLength);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
          }
          USBD_CtlSendData(pdev, pbuf, len);
          break;

        case USB_REQ_GET_INTERFACE :
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            USBD_CtlSendData(pdev, (uint8_t *)(void *)&hhid->AltSetting, 1U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE :
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            hhid->AltSetting = (uint8_t)(req->wValue);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return ret;
}

uint8_t USBD_HID_SendReport(USBD_HandleTypeDef  *pdev, uint8_t *report, uint16_t len){
  USBD_HID_HandleTypeDef     *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassData;

  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
    if (hhid->state == HID_IDLE)
    {
      hhid->state = HID_BUSY;
      USBD_LL_Transmit(pdev,
                       HID_EPIN_ADDR,
                       report,
                       len);
    }
  }
  return USBD_OK;
}

static uint8_t  *USBD_HID_GetFSCfgDesc(uint16_t *length){
  *length = sizeof(USBD_HID_CfgFSDesc);
  return USBD_HID_CfgFSDesc;
}

static uint8_t  *USBD_HID_GetHSCfgDesc(uint16_t *length){
  *length = sizeof(USBD_HID_CfgHSDesc);
  return USBD_HID_CfgHSDesc;
}

static uint8_t  *USBD_HID_GetOtherSpeedCfgDesc(uint16_t *length){
  *length = sizeof(USBD_HID_OtherSpeedCfgDesc);
  return USBD_HID_OtherSpeedCfgDesc;
}

static uint8_t  USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum){
  ((USBD_HID_HandleTypeDef *)pdev->pClassData)->state = HID_IDLE;
  return USBD_OK;
}

static uint8_t  *USBD_HID_GetDeviceQualifierDesc(uint16_t *length){
  *length = sizeof(USBD_HID_DeviceQualifierDesc);
  return USBD_HID_DeviceQualifierDesc;
}

USBD_StatusTypeDef USBD_CtlSendData(USBD_HandleTypeDef *pdev, uint8_t *pbuf, uint16_t len){
  pdev->ep0_state = USBD_EP0_DATA_IN;
  pdev->ep_in[0].total_length = len;
  pdev->ep_in[0].rem_length   = len;
  USBD_LL_Transmit(pdev, 0x00U, pbuf, len);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_CtlContinueSendData(USBD_HandleTypeDef *pdev, uint8_t *pbuf, uint16_t len){
  USBD_LL_Transmit(pdev, 0x00U, pbuf, len);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_CtlContinueRx(USBD_HandleTypeDef *pdev, uint8_t *pbuf, uint16_t len){
  USBD_LL_PrepareReceive(pdev, 0U, pbuf, len);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_CtlSendStatus(USBD_HandleTypeDef *pdev){
  pdev->ep0_state = USBD_EP0_STATUS_IN;
  USBD_LL_Transmit(pdev, 0x00U, NULL, 0U);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_CtlReceiveStatus(USBD_HandleTypeDef *pdev){
  pdev->ep0_state = USBD_EP0_STATUS_OUT;
  USBD_LL_PrepareReceive(pdev, 0U, NULL, 0U);
  return USBD_OK;
}

