#include "../Drivers/stm32f1xx.h"
#include "../Drivers/stm32f1xx_hal.h"
#include "../Core/Src/Inc_USB_Device.h"

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

  if ( hpcd->Init.speed != PCD_SPEED_FULL)
  {
    Error_Handler();
  }
    /* Set Speed. */
  USBD_LL_SetSpeed((USBD_HandleTypeDef*)hpcd->pData, speed);

  /* Reset Device. */
  USBD_LL_Reset((USBD_HandleTypeDef*)hpcd->pData);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd){
  /* Inform USB library that core enters in suspend Mode. */
  USBD_LL_Suspend((USBD_HandleTypeDef*)hpcd->pData);
  /* Enter in STOP mode. */
  /* USER CODE BEGIN 2 */
  if (hpcd->Init.low_power_enable)
  {
    /* Set SLEEPDEEP bit and SleepOnExit of Cortex System Control Register. */
    SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
  }
  /* USER CODE END 2 */
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd){
  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
  USBD_LL_Resume((USBD_HandleTypeDef*)hpcd->pData);
}

HAL_StatusTypeDef  HAL_PCDEx_PMAConfig(PCD_HandleTypeDef *hpcd, uint16_t ep_addr, uint16_t ep_kind, uint32_t pmaadress){
  PCD_EPTypeDef *ep;

  /* initialize ep structure*/
  if ((0x80U & ep_addr) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr];
  }

  /* Here we check if the endpoint is single or double Buffer*/
  if (ep_kind == PCD_SNG_BUF)
  {
    /* Single Buffer */
    ep->doublebuffer = 0U;
    /* Configure the PMA */
    ep->pmaadress = (uint16_t)pmaadress;
  }
#if (USE_USB_DOUBLE_BUFFER == 1U)
  else /* USB_DBL_BUF */
  {
    /* Double Buffer Endpoint */
    ep->doublebuffer = 1U;
    /* Configure the PMA */
    ep->pmaaddr0 = (uint16_t)(pmaadress & 0xFFFFU);
    ep->pmaaddr1 = (uint16_t)((pmaadress & 0xFFFF0000U) >> 16);
  }
#endif /* (USE_USB_DOUBLE_BUFFER == 1U) */

  return HAL_OK;
}

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev){
  /* Init USB Ip. */
  /* Link the driver to the stack. */
  hpcd_USB_FS.pData = pdev;
  pdev->pData = &hpcd_USB_FS;

  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler( );
  }

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
  /* Register USB PCD CallBacks */
  HAL_PCD_RegisterCallback(&hpcd_USB_FS, HAL_PCD_SOF_CB_ID, PCD_SOFCallback);
  HAL_PCD_RegisterCallback(&hpcd_USB_FS, HAL_PCD_SETUPSTAGE_CB_ID, PCD_SetupStageCallback);
  HAL_PCD_RegisterCallback(&hpcd_USB_FS, HAL_PCD_RESET_CB_ID, PCD_ResetCallback);
  HAL_PCD_RegisterCallback(&hpcd_USB_FS, HAL_PCD_SUSPEND_CB_ID, PCD_SuspendCallback);
  HAL_PCD_RegisterCallback(&hpcd_USB_FS, HAL_PCD_RESUME_CB_ID, PCD_ResumeCallback);
  HAL_PCD_RegisterCallback(&hpcd_USB_FS, HAL_PCD_CONNECT_CB_ID, PCD_ConnectCallback);
  HAL_PCD_RegisterCallback(&hpcd_USB_FS, HAL_PCD_DISCONNECT_CB_ID, PCD_DisconnectCallback);

  HAL_PCD_RegisterDataOutStageCallback(&hpcd_USB_FS, PCD_DataOutStageCallback);
  HAL_PCD_RegisterDataInStageCallback(&hpcd_USB_FS, PCD_DataInStageCallback);
  HAL_PCD_RegisterIsoOutIncpltCallback(&hpcd_USB_FS, PCD_ISOOUTIncompleteCallback);
  HAL_PCD_RegisterIsoInIncpltCallback(&hpcd_USB_FS, PCD_ISOINIncompleteCallback);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
  /* USER CODE BEGIN EndPoint_Configuration */
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x00 , PCD_SNG_BUF, 0x18);
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x80 , PCD_SNG_BUF, 0x58);
  /* USER CODE END EndPoint_Configuration */
  /* USER CODE BEGIN EndPoint_Configuration_HID */
  HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x81 , PCD_SNG_BUF, 0x100);
  /* USER CODE END EndPoint_Configuration_HID */
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

  switch (hal_status)
  {
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
