#include "inc/definiciones.h"

void SystemClock_Config(void);
void USB_DEVICE_Init(void);
void Error_Handler();

int main(void)
{    
  SystemClock_Config();
  USB_DEVICE_Init();

  RCC->APB2ENR |= (1<<4);               //Enable clock for GPIOC
  GPIOC->CRH = (3 << 4*(13-8));         //C13  output

  SYST->ReloadValue = Time_1ms*1000;    //Seteo el reset a 1s
  SYST->CSR |= (0 << 2);                //Clock interno
  SYST->CSR |= (1 << 0);                //Enable

  for (;;){
    if(SYST->CSR & (1 << 16)){
      GPIOC->OUT ^=  (1 << 13); 
    }
  }
  return 0;
}


HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FLatency);
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *PeriphClkInit);
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {    Error_Handler();  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {    Error_Handler();  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {    Error_Handler();  }
}

USBD_StatusTypeDef  USBD_Start(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef  USBD_RegisterClass(USBD_HandleTypeDef *pdev, USBD_ClassTypeDef *pclass);
USBD_StatusTypeDef  USBD_Init(USBD_HandleTypeDef *pdev, USBD_DescriptorsTypeDef *pdesc, uint8_t id);

void USBD_CtlError(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
USBD_StatusTypeDef USBD_CtlSendData(USBD_HandleTypeDef *pdev, uint8_t *pbuf, uint16_t len);
void USB_WritePMA(USB_TypeDef const *USBx, uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes);


HAL_StatusTypeDef USB_CoreInit(USB_TypeDef *USBx, USB_CfgTypeDef cfg);
HAL_StatusTypeDef USB_SetCurrentMode(USB_TypeDef *USBx, USB_ModeTypeDef mode);
HAL_StatusTypeDef USB_DevInit(USB_TypeDef *USBx, USB_CfgTypeDef cfg);
HAL_StatusTypeDef USB_ActivateEndpoint(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_DeactivateEndpoint(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPSetStall(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_DevConnect(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_DevDisconnect(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_EP0_OutStart(USB_TypeDef *USBx, uint8_t *psetup);

void HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup);

void HAL_PCD_MspInit(PCD_HandleTypeDef* pcdHandle);


void USB_DEVICE_Init(void)
{
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK){
    Error_Handler();
    }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_HID) != USBD_OK){
    Error_Handler();
    }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK){
    Error_Handler();
    }
}

void Error_Handler(){
  while (1)  {}
}

//
//
//
//
//
//
//               USB_FUNCTIONS.c
//
//
//
//
//
//
//


// #define assert_param(expr) ((void)0U)

// HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);

// extern __IO uint32_t uwTick;
// extern uint32_t uwTickPrio;
// extern HAL_TickFreqTypeDef uwTickFreq;

// __weak uint32_t HAL_GetTick(void)
// {
//   return uwTick;
// }

void HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  assert_param(IS_NVIC_PRIORITY_GROUP(PriorityGroup));
  NVIC_SetPriorityGrouping(PriorityGroup);
}

void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t prioritygroup = 0x00U;
  assert_param(IS_NVIC_SUB_PRIORITY(SubPriority));
  assert_param(IS_NVIC_PREEMPTION_PRIORITY(PreemptPriority));

  prioritygroup = NVIC_GetPriorityGrouping();

  NVIC_SetPriority(IRQn, NVIC_EncodePriority(prioritygroup, PreemptPriority, SubPriority));
}

void HAL_NVIC_EnableIRQ(IRQn_Type IRQn)
{
  assert_param(IS_NVIC_DEVICE_IRQ(IRQn));
  NVIC_EnableIRQ(IRQn);
}

// uint32_t HAL_SYSTICK_Config(uint32_t TicksNumb)
// {
//   return SysTick_Config(TicksNumb);
// }

// static void RCC_Delay(uint32_t mdelay);

// #define NULL 0

HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct)
{
  uint32_t tickstart;
  uint32_t pll_config;

  if (RCC_OscInitStruct == NULL)
  {
    return HAL_ERROR;
  }

  assert_param(IS_RCC_OSCILLATORTYPE(RCC_OscInitStruct->OscillatorType));

  /*------------------------------- HSE Configuration ------------------------*/
  if (((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSE) == RCC_OSCILLATORTYPE_HSE)
  {
    /* Check the parameters */
    assert_param(IS_RCC_HSE(RCC_OscInitStruct->HSEState));

    /* When the HSE is used as system clock or clock source for PLL in these cases it is not allowed to be disabled */
    if ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_HSE) || ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK) && (__HAL_RCC_GET_PLL_OSCSOURCE() == RCC_PLLSOURCE_HSE)))
    {
      if ((__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) != RESET) && (RCC_OscInitStruct->HSEState == RCC_HSE_OFF))
      {
        return HAL_ERROR;
      }
    }
    else
    {
      /* Set the new HSE configuration ---------------------------------------*/
      __HAL_RCC_HSE_CONFIG(RCC_OscInitStruct->HSEState);

      /* Check the HSE State */
      if (RCC_OscInitStruct->HSEState != RCC_HSE_OFF)
      {
        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till HSE is ready */
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET)
        {
          if ((HAL_GetTick() - tickstart) > HSE_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
      else
      {
        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till HSE is disabled */
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) != RESET)
        {
          if ((HAL_GetTick() - tickstart) > HSE_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
  }
  /*----------------------------- HSI Configuration --------------------------*/
  if (((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSI) == RCC_OSCILLATORTYPE_HSI)
  {
    /* Check the parameters */
    assert_param(IS_RCC_HSI(RCC_OscInitStruct->HSIState));
    assert_param(IS_RCC_CALIBRATION_VALUE(RCC_OscInitStruct->HSICalibrationValue));

    /* Check if HSI is used as system clock or as PLL source when PLL is selected as system clock */
    if ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_HSI) || ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK) && (__HAL_RCC_GET_PLL_OSCSOURCE() == RCC_PLLSOURCE_HSI_DIV2)))
    {
      /* When HSI is used as system clock it will not disabled */
      if ((__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) != RESET) && (RCC_OscInitStruct->HSIState != RCC_HSI_ON))
      {
        return HAL_ERROR;
      }
      /* Otherwise, just the calibration is allowed */
      else
      {
        /* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
        __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->HSICalibrationValue);
      }
    }
    else
    {
      /* Check the HSI State */
      if (RCC_OscInitStruct->HSIState != RCC_HSI_OFF)
      {
        /* Enable the Internal High Speed oscillator (HSI). */
        __HAL_RCC_HSI_ENABLE();

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till HSI is ready */
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET)
        {
          if ((HAL_GetTick() - tickstart) > HSI_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }

        /* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
        __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->HSICalibrationValue);
      }
      else
      {
        /* Disable the Internal High Speed oscillator (HSI). */
        __HAL_RCC_HSI_DISABLE();

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till HSI is disabled */
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) != RESET)
        {
          if ((HAL_GetTick() - tickstart) > HSI_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
  }
  /*------------------------------ LSI Configuration -------------------------*/
  if (((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_LSI) == RCC_OSCILLATORTYPE_LSI)
  {
    /* Check the parameters */
    assert_param(IS_RCC_LSI(RCC_OscInitStruct->LSIState));

    /* Check the LSI State */
    if (RCC_OscInitStruct->LSIState != RCC_LSI_OFF)
    {
      /* Enable the Internal Low Speed oscillator (LSI). */
      __HAL_RCC_LSI_ENABLE();

      /* Get Start Tick */
      tickstart = HAL_GetTick();

      /* Wait till LSI is ready */
      while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) == RESET)
      {
        if ((HAL_GetTick() - tickstart) > LSI_TIMEOUT_VALUE)
        {
          return HAL_TIMEOUT;
        }
      }
      /*  To have a fully stabilized clock in the specified range, a software delay of 1ms
          should be added.*/
      RCC_Delay(1);
    }
    else
    {
      /* Disable the Internal Low Speed oscillator (LSI). */
      __HAL_RCC_LSI_DISABLE();

      /* Get Start Tick */
      tickstart = HAL_GetTick();

      /* Wait till LSI is disabled */
      while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) != RESET)
      {
        if ((HAL_GetTick() - tickstart) > LSI_TIMEOUT_VALUE)
        {
          return HAL_TIMEOUT;
        }
      }
    }
  }
  /*------------------------------ LSE Configuration -------------------------*/
  if (((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_LSE) == RCC_OSCILLATORTYPE_LSE)
  {
    FlagStatus pwrclkchanged = RESET;

    /* Check the parameters */
    assert_param(IS_RCC_LSE(RCC_OscInitStruct->LSEState));

    /* Update LSE configuration in Backup Domain control register    */
    /* Requires to enable write access to Backup Domain of necessary */
    if (__HAL_RCC_PWR_IS_CLK_DISABLED())
    {
      __HAL_RCC_PWR_CLK_ENABLE();
      pwrclkchanged = SET;
    }

    if (HAL_IS_BIT_CLR(PWR->CR, PWR_CR_DBP))
    {
      /* Enable write access to Backup domain */
      SET_BIT(PWR->CR, PWR_CR_DBP);

      /* Wait for Backup domain Write protection disable */
      tickstart = HAL_GetTick();

      while (HAL_IS_BIT_CLR(PWR->CR, PWR_CR_DBP))
      {
        if ((HAL_GetTick() - tickstart) > RCC_DBP_TIMEOUT_VALUE)
        {
          return HAL_TIMEOUT;
        }
      }
    }

    /* Set the new LSE configuration -----------------------------------------*/
    __HAL_RCC_LSE_CONFIG(RCC_OscInitStruct->LSEState);
    /* Check the LSE State */
    if (RCC_OscInitStruct->LSEState != RCC_LSE_OFF)
    {
      /* Get Start Tick */
      tickstart = HAL_GetTick();

      /* Wait till LSE is ready */
      while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET)
      {
        if ((HAL_GetTick() - tickstart) > RCC_LSE_TIMEOUT_VALUE)
        {
          return HAL_TIMEOUT;
        }
      }
    }
    else
    {
      /* Get Start Tick */
      tickstart = HAL_GetTick();

      /* Wait till LSE is disabled */
      while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != RESET)
      {
        if ((HAL_GetTick() - tickstart) > RCC_LSE_TIMEOUT_VALUE)
        {
          return HAL_TIMEOUT;
        }
      }
    }

    /* Require to disable power clock if necessary */
    if (pwrclkchanged == SET)
    {
      __HAL_RCC_PWR_CLK_DISABLE();
    }
  }
  /*-------------------------------- PLL Configuration -----------------------*/
  /* Check the parameters */
  assert_param(IS_RCC_PLL(RCC_OscInitStruct->PLL.PLLState));
  if ((RCC_OscInitStruct->PLL.PLLState) != RCC_PLL_NONE)
  {
    /* Check if the PLL is used as system clock or not */
    if (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_PLLCLK)
    {
      if ((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_ON)
      {
        /* Check the parameters */
        assert_param(IS_RCC_PLLSOURCE(RCC_OscInitStruct->PLL.PLLSource));
        assert_param(IS_RCC_PLL_MUL(RCC_OscInitStruct->PLL.PLLMUL));

        /* Disable the main PLL. */
        __HAL_RCC_PLL_DISABLE();

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till PLL is disabled */
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) != RESET)
        {
          if ((HAL_GetTick() - tickstart) > PLL_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }

        /* Configure the HSE prediv factor --------------------------------*/
        /* It can be written only when the PLL is disabled. Not used in PLL source is different than HSE */
        if (RCC_OscInitStruct->PLL.PLLSource == RCC_PLLSOURCE_HSE)
        {
          /* Check the parameter */
          assert_param(IS_RCC_HSE_PREDIV(RCC_OscInitStruct->HSEPredivValue));
           /* Set PREDIV1 Value */
          __HAL_RCC_HSE_PREDIV_CONFIG(RCC_OscInitStruct->HSEPredivValue);
        }

        /* Configure the main PLL clock source and multiplication factors. */
        __HAL_RCC_PLL_CONFIG(RCC_OscInitStruct->PLL.PLLSource,
                             RCC_OscInitStruct->PLL.PLLMUL);
        /* Enable the main PLL. */
        __HAL_RCC_PLL_ENABLE();

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till PLL is ready */
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET)
        {
          if ((HAL_GetTick() - tickstart) > PLL_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
      else
      {
        /* Disable the main PLL. */
        __HAL_RCC_PLL_DISABLE();

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till PLL is disabled */
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) != RESET)
        {
          if ((HAL_GetTick() - tickstart) > PLL_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
    else
    {
      /* Check if there is a request to disable the PLL used as System clock source */
      if ((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_OFF)
      {
        return HAL_ERROR;
      }
      else
      {
        /* Do not return HAL_ERROR if request repeats the current configuration */
        pll_config = RCC->CFGR;
        if ((READ_BIT(pll_config, RCC_CFGR_PLLSRC) != RCC_OscInitStruct->PLL.PLLSource) ||
            (READ_BIT(pll_config, RCC_CFGR_PLLMULL) != RCC_OscInitStruct->PLL.PLLMUL))
        {
          return HAL_ERROR;
        }
      }
    }
  }

  return HAL_OK;
}

HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FLatency)
{
  uint32_t tickstart;

  if (RCC_ClkInitStruct == NULL)
  {
    return HAL_ERROR;
  }

  assert_param(IS_RCC_CLOCKTYPE(RCC_ClkInitStruct->ClockType));
  assert_param(IS_FLASH_LATENCY(FLatency));

  /* To correctly read data from FLASH memory, the number of wait states (LATENCY)
  must be correctly programmed according to the frequency of the CPU clock
    (HCLK) of the device. */

#if defined(FLASH_ACR_LATENCY)
  /* Increasing the number of wait states because of higher CPU frequency */
  if (FLatency > __HAL_FLASH_GET_LATENCY())
  {
    /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
    __HAL_FLASH_SET_LATENCY(FLatency);

    /* Check that the new number of wait states is taken into account to access the Flash
    memory by reading the FLASH_ACR register */
    if (__HAL_FLASH_GET_LATENCY() != FLatency)
    {
      return HAL_ERROR;
    }
  }

#endif /* FLASH_ACR_LATENCY */
  /*-------------------------- HCLK Configuration --------------------------*/
  if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_HCLK) == RCC_CLOCKTYPE_HCLK)
  {
    /* Set the highest APBx dividers in order to ensure that we do not go through
    a non-spec phase whatever we decrease or increase HCLK. */
    if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
    {
      MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_HCLK_DIV16);
    }

    if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
    {
      MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, (RCC_HCLK_DIV16 << 3));
    }

    /* Set the new HCLK clock divider */
    assert_param(IS_RCC_HCLK(RCC_ClkInitStruct->AHBCLKDivider));
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_ClkInitStruct->AHBCLKDivider);
  }

  /*------------------------- SYSCLK Configuration ---------------------------*/
  if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_SYSCLK) == RCC_CLOCKTYPE_SYSCLK)
  {
    assert_param(IS_RCC_SYSCLKSOURCE(RCC_ClkInitStruct->SYSCLKSource));

    /* HSE is selected as System Clock Source */
    if (RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_HSE)
    {
      /* Check the HSE ready flag */
      if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET)
      {
        return HAL_ERROR;
      }
    }
    /* PLL is selected as System Clock Source */
    else if (RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_PLLCLK)
    {
      /* Check the PLL ready flag */
      if (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET)
      {
        return HAL_ERROR;
      }
    }
    /* HSI is selected as System Clock Source */
    else
    {
      /* Check the HSI ready flag */
      if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET)
      {
        return HAL_ERROR;
      }
    }
    __HAL_RCC_SYSCLK_CONFIG(RCC_ClkInitStruct->SYSCLKSource);

    /* Get Start Tick */
    tickstart = HAL_GetTick();

    while (__HAL_RCC_GET_SYSCLK_SOURCE() != (RCC_ClkInitStruct->SYSCLKSource << RCC_CFGR_SWS_Pos))
    {
      if ((HAL_GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE)
      {
        return HAL_TIMEOUT;
      }
    }
  }

#if defined(FLASH_ACR_LATENCY)
  /* Decreasing the number of wait states because of lower CPU frequency */
  if (FLatency < __HAL_FLASH_GET_LATENCY())
  {
    /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
    __HAL_FLASH_SET_LATENCY(FLatency);

    /* Check that the new number of wait states is taken into account to access the Flash
    memory by reading the FLASH_ACR register */
    if (__HAL_FLASH_GET_LATENCY() != FLatency)
    {
      return HAL_ERROR;
    }
  }
#endif /* FLASH_ACR_LATENCY */

  /*-------------------------- PCLK1 Configuration ---------------------------*/
  if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
  {
    assert_param(IS_RCC_PCLK(RCC_ClkInitStruct->APB1CLKDivider));
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_ClkInitStruct->APB1CLKDivider);
  }

  /*-------------------------- PCLK2 Configuration ---------------------------*/
  if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
  {
    assert_param(IS_RCC_PCLK(RCC_ClkInitStruct->APB2CLKDivider));
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, ((RCC_ClkInitStruct->APB2CLKDivider) << 3));
  }

  /* Update the SystemCoreClock global variable */
  SystemCoreClock = HAL_RCC_GetSysClockFreq() >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];

  /* Configure the source of time base considering new system clocks settings*/
  HAL_InitTick(uwTickPrio);

  return HAL_OK;
}

HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *PeriphClkInit)
{
  uint32_t tickstart = 0U, temp_reg = 0U;
#if defined(STM32F105xC) || defined(STM32F107xC)
  uint32_t pllactive = 0U;
#endif /* STM32F105xC || STM32F107xC */

  /* Check the parameters */
  assert_param(IS_RCC_PERIPHCLOCK(PeriphClkInit->PeriphClockSelection));

  /*------------------------------- RTC/LCD Configuration ------------------------*/
  if ((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_RTC) == RCC_PERIPHCLK_RTC))
  {
    FlagStatus pwrclkchanged = RESET;

    /* check for RTC Parameters used to output RTCCLK */
    assert_param(IS_RCC_RTCCLKSOURCE(PeriphClkInit->RTCClockSelection));

    /* As soon as function is called to change RTC clock source, activation of the
       power domain is done. */
    /* Requires to enable write access to Backup Domain of necessary */
    if (__HAL_RCC_PWR_IS_CLK_DISABLED())
    {
      __HAL_RCC_PWR_CLK_ENABLE();
      pwrclkchanged = SET;
    }

    if (HAL_IS_BIT_CLR(PWR->CR, PWR_CR_DBP))
    {
      /* Enable write access to Backup domain */
      SET_BIT(PWR->CR, PWR_CR_DBP);

      /* Wait for Backup domain Write protection disable */
      tickstart = HAL_GetTick();

      while (HAL_IS_BIT_CLR(PWR->CR, PWR_CR_DBP))
      {
        if ((HAL_GetTick() - tickstart) > RCC_DBP_TIMEOUT_VALUE)
        {
          return HAL_TIMEOUT;
        }
      }
    }

    /* Reset the Backup domain only if the RTC Clock source selection is modified from reset value */
    temp_reg = (RCC->BDCR & RCC_BDCR_RTCSEL);
    if ((temp_reg != 0x00000000U) && (temp_reg != (PeriphClkInit->RTCClockSelection & RCC_BDCR_RTCSEL)))
    {
      /* Store the content of BDCR register before the reset of Backup Domain */
      temp_reg = (RCC->BDCR & ~(RCC_BDCR_RTCSEL));
      /* RTC Clock selection can be changed only if the Backup Domain is reset */
      __HAL_RCC_BACKUPRESET_FORCE();
      __HAL_RCC_BACKUPRESET_RELEASE();
      /* Restore the Content of BDCR register */
      RCC->BDCR = temp_reg;

      /* Wait for LSERDY if LSE was enabled */
      if (HAL_IS_BIT_SET(temp_reg, RCC_BDCR_LSEON))
      {
        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till LSE is ready */
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET)
        {
          if ((HAL_GetTick() - tickstart) > RCC_LSE_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
    __HAL_RCC_RTC_CONFIG(PeriphClkInit->RTCClockSelection);

    /* Require to disable power clock if necessary */
    if (pwrclkchanged == SET)
    {
      __HAL_RCC_PWR_CLK_DISABLE();
    }
  }

  /*------------------------------ ADC clock Configuration ------------------*/
  if (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_ADC) == RCC_PERIPHCLK_ADC)
  {
    /* Check the parameters */
    assert_param(IS_RCC_ADCPLLCLK_DIV(PeriphClkInit->AdcClockSelection));

    /* Configure the ADC clock source */
    __HAL_RCC_ADC_CONFIG(PeriphClkInit->AdcClockSelection);
  }

#if defined(STM32F105xC) || defined(STM32F107xC)
  /*------------------------------ I2S2 Configuration ------------------------*/
  if (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2S2) == RCC_PERIPHCLK_I2S2)
  {
    /* Check the parameters */
    assert_param(IS_RCC_I2S2CLKSOURCE(PeriphClkInit->I2s2ClockSelection));

    /* Configure the I2S2 clock source */
    __HAL_RCC_I2S2_CONFIG(PeriphClkInit->I2s2ClockSelection);
  }

  /*------------------------------ I2S3 Configuration ------------------------*/
  if (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2S3) == RCC_PERIPHCLK_I2S3)
  {
    /* Check the parameters */
    assert_param(IS_RCC_I2S3CLKSOURCE(PeriphClkInit->I2s3ClockSelection));

    /* Configure the I2S3 clock source */
    __HAL_RCC_I2S3_CONFIG(PeriphClkInit->I2s3ClockSelection);
  }

  /*------------------------------ PLL I2S Configuration ----------------------*/
  /* Check that PLLI2S need to be enabled */
  if (HAL_IS_BIT_SET(RCC->CFGR2, RCC_CFGR2_I2S2SRC) || HAL_IS_BIT_SET(RCC->CFGR2, RCC_CFGR2_I2S3SRC))
  {
    /* Update flag to indicate that PLL I2S should be active */
    pllactive = 1;
  }

  /* Check if PLL I2S need to be enabled */
  if (pllactive == 1)
  {
    /* Enable PLL I2S only if not active */
    if (HAL_IS_BIT_CLR(RCC->CR, RCC_CR_PLL3ON))
    {
      /* Check the parameters */
      assert_param(IS_RCC_PLLI2S_MUL(PeriphClkInit->PLLI2S.PLLI2SMUL));
      assert_param(IS_RCC_HSE_PREDIV2(PeriphClkInit->PLLI2S.HSEPrediv2Value));

      /* Prediv2 can be written only when the PLL2 is disabled. */
      /* Return an error only if new value is different from the programmed value */
      if (HAL_IS_BIT_SET(RCC->CR, RCC_CR_PLL2ON) &&
          (__HAL_RCC_HSE_GET_PREDIV2() != PeriphClkInit->PLLI2S.HSEPrediv2Value))
      {
        return HAL_ERROR;
      }

      /* Configure the HSE prediv2 factor --------------------------------*/
      __HAL_RCC_HSE_PREDIV2_CONFIG(PeriphClkInit->PLLI2S.HSEPrediv2Value);

      /* Configure the main PLLI2S multiplication factors. */
      __HAL_RCC_PLLI2S_CONFIG(PeriphClkInit->PLLI2S.PLLI2SMUL);

      /* Enable the main PLLI2S. */
      __HAL_RCC_PLLI2S_ENABLE();

      /* Get Start Tick*/
      tickstart = HAL_GetTick();

      /* Wait till PLLI2S is ready */
      while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLI2SRDY) == RESET)
      {
        if ((HAL_GetTick() - tickstart) > PLLI2S_TIMEOUT_VALUE)
        {
          return HAL_TIMEOUT;
        }
      }
    }
    else
    {
      /* Return an error only if user wants to change the PLLI2SMUL whereas PLLI2S is active */
      if (READ_BIT(RCC->CFGR2, RCC_CFGR2_PLL3MUL) != PeriphClkInit->PLLI2S.PLLI2SMUL)
      {
        return HAL_ERROR;
      }
    }
  }
#endif /* STM32F105xC || STM32F107xC */

#if defined(STM32F102x6) || defined(STM32F102xB) || defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || defined(STM32F107xC)
  /*------------------------------ USB clock Configuration ------------------*/
  if (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_USB) == RCC_PERIPHCLK_USB)
  {
    /* Check the parameters */
    assert_param(IS_RCC_USBPLLCLK_DIV(PeriphClkInit->UsbClockSelection));

    /* Configure the USB clock source */
    __HAL_RCC_USB_CONFIG(PeriphClkInit->UsbClockSelection);
  }
#endif /* STM32F102x6 || STM32F102xB || STM32F103x6 || STM32F103xB || STM32F103xE || STM32F103xG || STM32F105xC || STM32F107xC */

  return HAL_OK;
}

// uint32_t HAL_RCC_GetSysClockFreq(void)
// {

//   static const uint8_t aPLLMULFactorTable[16U] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 16};
//   static const uint8_t aPredivFactorTable[2U] = {1, 2};

//   uint32_t tmpreg = 0U, prediv = 0U, pllclk = 0U, pllmul = 0U;
//   uint32_t sysclockfreq = 0U;

//   tmpreg = RCC->CFGR;

//   /* Get SYSCLK source -------------------------------------------------------*/
//   switch (tmpreg & RCC_CFGR_SWS)
//   {
//   case RCC_SYSCLKSOURCE_STATUS_HSE: /* HSE used as system clock */
//   {
//     sysclockfreq = HSE_VALUE;
//     break;
//   }
//   case RCC_SYSCLKSOURCE_STATUS_PLLCLK: /* PLL used as system clock */
//   {
//     pllmul = aPLLMULFactorTable[(uint32_t)(tmpreg & RCC_CFGR_PLLMULL) >> RCC_CFGR_PLLMULL_Pos];
//     if ((tmpreg & RCC_CFGR_PLLSRC) != RCC_PLLSOURCE_HSI_DIV2)
//     {
// #if defined(RCC_CFGR2_PREDIV1)
//       prediv = aPredivFactorTable[(uint32_t)(RCC->CFGR2 & RCC_CFGR2_PREDIV1) >> RCC_CFGR2_PREDIV1_Pos];
// #else
//       prediv = aPredivFactorTable[(uint32_t)(RCC->CFGR & RCC_CFGR_PLLXTPRE) >> RCC_CFGR_PLLXTPRE_Pos];
// #endif /*RCC_CFGR2_PREDIV1*/
// #if defined(RCC_CFGR2_PREDIV1SRC)

//       if (HAL_IS_BIT_SET(RCC->CFGR2, RCC_CFGR2_PREDIV1SRC))
//       {
//         /* PLL2 selected as Prediv1 source */
//         /* PLLCLK = PLL2CLK / PREDIV1 * PLLMUL with PLL2CLK = HSE/PREDIV2 * PLL2MUL */
//         prediv2 = ((RCC->CFGR2 & RCC_CFGR2_PREDIV2) >> RCC_CFGR2_PREDIV2_Pos) + 1;
//         pll2mul = ((RCC->CFGR2 & RCC_CFGR2_PLL2MUL) >> RCC_CFGR2_PLL2MUL_Pos) + 2;
//         pllclk = (uint32_t)(((uint64_t)HSE_VALUE * (uint64_t)pll2mul * (uint64_t)pllmul) / ((uint64_t)prediv2 * (uint64_t)prediv));
//       }
//       else
//       {
//         /* HSE used as PLL clock source : PLLCLK = HSE/PREDIV1 * PLLMUL */
//         pllclk = (uint32_t)((HSE_VALUE * pllmul) / prediv);
//       }

//       /* If PLLMUL was set to 13 means that it was to cover the case PLLMUL 6.5 (avoid using float) */
//       /* In this case need to divide pllclk by 2 */
//       if (pllmul == aPLLMULFactorTable[(uint32_t)(RCC_CFGR_PLLMULL6_5) >> RCC_CFGR_PLLMULL_Pos])
//       {
//         pllclk = pllclk / 2;
//       }
// #else
//       /* HSE used as PLL clock source : PLLCLK = HSE/PREDIV1 * PLLMUL */
//       pllclk = (uint32_t)((HSE_VALUE * pllmul) / prediv);
// #endif /*RCC_CFGR2_PREDIV1SRC*/
//     }
//     else
//     {
//       /* HSI used as PLL clock source : PLLCLK = HSI/2 * PLLMUL */
//       pllclk = (uint32_t)((HSI_VALUE >> 1) * pllmul);
//     }
//     sysclockfreq = pllclk;
//     break;
//   }
//   case RCC_SYSCLKSOURCE_STATUS_HSI: /* HSI used as system clock source */
//   default:                          /* HSI used as system clock */
//   {
//     sysclockfreq = HSI_VALUE;
//     break;
//   }
//   }
//   return sysclockfreq;
// }

// static void RCC_Delay(uint32_t mdelay)
// {
//   __IO uint32_t Delay = mdelay * (SystemCoreClock / 8U / 1000U);
//   do
//   {
//     __NOP();
//   } while (Delay--);
// }

// __weak void HAL_RCC_CSSCallback(void) {}

// #ifdef HAL_PCD_MODULE_ENABLED

// #define PCD_MIN(a, b) (((a) < (b)) ? (a) : (b))
// #define PCD_MAX(a, b) (((a) > (b)) ? (a) : (b))

// static HAL_StatusTypeDef PCD_EP_ISR_Handler(PCD_HandleTypeDef *hpcd);
// static HAL_StatusTypeDef HAL_PCD_EP_DB_Transmit(PCD_HandleTypeDef *hpcd, PCD_EPTypeDef *ep, uint16_t wEPVal);
// static uint16_t HAL_PCD_EP_DB_Receive(PCD_HandleTypeDef *hpcd, PCD_EPTypeDef *ep, uint16_t wEPVal);

HAL_StatusTypeDef HAL_PCD_Init(PCD_HandleTypeDef *hpcd)
{
  uint8_t i;

  if (hpcd == NULL)
  {
    return HAL_ERROR;
  }
  assert_param(IS_PCD_ALL_INSTANCE(hpcd->Instance));
  if (hpcd->State == HAL_PCD_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hpcd->Lock = HAL_UNLOCKED;
    HAL_PCD_MspInit(hpcd);
  }

  hpcd->State = HAL_PCD_STATE_BUSY;
  __HAL_PCD_DISABLE(hpcd);

  if (USB_CoreInit(hpcd->Instance, hpcd->Init) != HAL_OK)
  {
    hpcd->State = HAL_PCD_STATE_ERROR;
    return HAL_ERROR;
  }

  (void)USB_SetCurrentMode(hpcd->Instance, USB_DEVICE_MODE);

  for (i = 0U; i < hpcd->Init.dev_endpoints; i++)
  {
    /* Init ep structure */
    hpcd->IN_ep[i].is_in = 1U;
    hpcd->IN_ep[i].num = i;
    /* Control until ep is activated */
    hpcd->IN_ep[i].type = EP_TYPE_CTRL;
    hpcd->IN_ep[i].maxpacket = 0U;
    hpcd->IN_ep[i].xfer_buff = 0U;
    hpcd->IN_ep[i].xfer_len = 0U;
  }

  for (i = 0U; i < hpcd->Init.dev_endpoints; i++)
  {
    hpcd->OUT_ep[i].is_in = 0U;
    hpcd->OUT_ep[i].num = i;
    /* Control until ep is activated */
    hpcd->OUT_ep[i].type = EP_TYPE_CTRL;
    hpcd->OUT_ep[i].maxpacket = 0U;
    hpcd->OUT_ep[i].xfer_buff = 0U;
    hpcd->OUT_ep[i].xfer_len = 0U;
  }

  /* Init Device */
  if (USB_DevInit(hpcd->Instance, hpcd->Init) != HAL_OK)
  {
    hpcd->State = HAL_PCD_STATE_ERROR;
    return HAL_ERROR;
  }

  hpcd->USB_Address = 0U;
  hpcd->State = HAL_PCD_STATE_READY;
  (void)USB_DevDisconnect(hpcd->Instance);

  return HAL_OK;
}

HAL_StatusTypeDef HAL_PCD_Start(PCD_HandleTypeDef *hpcd)
{
  __HAL_LOCK(hpcd);
  __HAL_PCD_ENABLE(hpcd);

  (void)USB_DevConnect(hpcd->Instance);
  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

// void HAL_PCD_IRQHandler(PCD_HandleTypeDef *hpcd)
// {
//   uint32_t wIstr = USB_ReadInterrupts(hpcd->Instance);
//   uint16_t store_ep[8];
//   uint8_t i;

//   if ((wIstr & USB_ISTR_CTR) == USB_ISTR_CTR)
//   {
//     /* servicing of the endpoint correct transfer interrupt */
//     /* clear of the CTR flag into the sub */
//     (void)PCD_EP_ISR_Handler(hpcd);

//     return;
//   }

//   if ((wIstr & USB_ISTR_RESET) == USB_ISTR_RESET)
//   {
//     __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_RESET);

//     HAL_PCD_ResetCallback(hpcd);

//     (void)HAL_PCD_SetAddress(hpcd, 0U);

//     return;
//   }

//   if ((wIstr & USB_ISTR_PMAOVR) == USB_ISTR_PMAOVR)
//   {
//     __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_PMAOVR);

//     return;
//   }

//   if ((wIstr & USB_ISTR_ERR) == USB_ISTR_ERR)
//   {
//     __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_ERR);

//     return;
//   }

//   if ((wIstr & USB_ISTR_WKUP) == USB_ISTR_WKUP)
//   {
//     hpcd->Instance->CNTR &= (uint16_t) ~(USB_CNTR_LP_MODE);
//     hpcd->Instance->CNTR &= (uint16_t) ~(USB_CNTR_FSUSP);

//     HAL_PCD_ResumeCallback(hpcd);

//     __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_WKUP);

//     return;
//   }

//   if ((wIstr & USB_ISTR_SUSP) == USB_ISTR_SUSP)
//   {
//     /* WA: To Clear Wakeup flag if raised with suspend signal */

//     /* Store Endpoint registers */
//     for (i = 0U; i < 8U; i++)
//     {
//       store_ep[i] = PCD_GET_ENDPOINT(hpcd->Instance, i);
//     }

//     /* FORCE RESET */
//     hpcd->Instance->CNTR |= (uint16_t)(USB_CNTR_FRES);

//     /* CLEAR RESET */
//     hpcd->Instance->CNTR &= (uint16_t)(~USB_CNTR_FRES);

//     /* wait for reset flag in ISTR */
//     while ((hpcd->Instance->ISTR & USB_ISTR_RESET) == 0U)
//     {
//     }

//     /* Clear Reset Flag */
//     __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_RESET);

//     /* Restore Registre */
//     for (i = 0U; i < 8U; i++)
//     {
//       PCD_SET_ENDPOINT(hpcd->Instance, i, store_ep[i]);
//     }

//     /* Force low-power mode in the macrocell */
//     hpcd->Instance->CNTR |= (uint16_t)USB_CNTR_FSUSP;

//     /* clear of the ISTR bit must be done after setting of CNTR_FSUSP */
//     __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_SUSP);

//     hpcd->Instance->CNTR |= (uint16_t)USB_CNTR_LP_MODE;

//     HAL_PCD_SuspendCallback(hpcd);

//     return;
//   }

//   if ((wIstr & USB_ISTR_SOF) == USB_ISTR_SOF)
//   {
//     __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_SOF);

//     HAL_PCD_SOFCallback(hpcd);

//     return;
//   }

//   if ((wIstr & USB_ISTR_ESOF) == USB_ISTR_ESOF)
//   {
//     /* clear ESOF flag in ISTR */
//     __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_ESOF);

//     return;
//   }
// }

// HAL_StatusTypeDef HAL_PCD_SetAddress(PCD_HandleTypeDef *hpcd, uint8_t address)
// {
//   __HAL_LOCK(hpcd);
//   hpcd->USB_Address = address;
//   (void)USB_SetDevAddress(hpcd->Instance, address);
//   __HAL_UNLOCK(hpcd);

//   return HAL_OK;
// }

HAL_StatusTypeDef HAL_PCD_EP_Open(PCD_HandleTypeDef *hpcd, uint8_t ep_addr,
                                  uint16_t ep_mps, uint8_t ep_type)
{
  HAL_StatusTypeDef ret = HAL_OK;
  PCD_EPTypeDef *ep;

  if ((ep_addr & 0x80U) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 0U;
  }

  ep->num = ep_addr & EP_ADDR_MSK;
  ep->maxpacket = ep_mps;
  ep->type = ep_type;

  /* Set initial data PID. */
  if (ep_type == EP_TYPE_BULK)
  {
    ep->data_pid_start = 0U;
  }

  __HAL_LOCK(hpcd);
  (void)USB_ActivateEndpoint(hpcd->Instance, ep);
  __HAL_UNLOCK(hpcd);

  return ret;
}

HAL_StatusTypeDef HAL_PCD_EP_Close(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  PCD_EPTypeDef *ep;

  if ((ep_addr & 0x80U) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 0U;
  }
  ep->num = ep_addr & EP_ADDR_MSK;

  __HAL_LOCK(hpcd);
  (void)USB_DeactivateEndpoint(hpcd->Instance, ep);
  __HAL_UNLOCK(hpcd);
  return HAL_OK;
}

// HAL_StatusTypeDef HAL_PCD_EP_Receive(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len)
// {
//   PCD_EPTypeDef *ep;

//   ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];

//   /*setup and start the Xfer */
//   ep->xfer_buff = pBuf;
//   ep->xfer_len = len;
//   ep->xfer_count = 0U;
//   ep->is_in = 0U;
//   ep->num = ep_addr & EP_ADDR_MSK;

//   (void)USB_EPStartXfer(hpcd->Instance, ep);

//   return HAL_OK;
// }

HAL_StatusTypeDef HAL_PCD_EP_Transmit(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len)
{
  PCD_EPTypeDef *ep;

  ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];

  /*setup and start the Xfer */
  ep->xfer_buff = pBuf;
  ep->xfer_len = len;
#if defined(USB)
  ep->xfer_fill_db = 1U;
  ep->xfer_len_db = len;
#endif /* defined (USB) */
  ep->xfer_count = 0U;
  ep->is_in = 1U;
  ep->num = ep_addr & EP_ADDR_MSK;

  (void)USB_EPStartXfer(hpcd->Instance, ep);

  return HAL_OK;
}

HAL_StatusTypeDef HAL_PCD_EP_SetStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  PCD_EPTypeDef *ep;

  if (((uint32_t)ep_addr & EP_ADDR_MSK) > hpcd->Init.dev_endpoints)
  {
    return HAL_ERROR;
  }

  if ((0x80U & ep_addr) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr];
    ep->is_in = 0U;
  }

  ep->is_stall = 1U;
  ep->num = ep_addr & EP_ADDR_MSK;

  __HAL_LOCK(hpcd);

  (void)USB_EPSetStall(hpcd->Instance, ep);

  if ((ep_addr & EP_ADDR_MSK) == 0U)
  {
    (void)USB_EP0_OutStart(hpcd->Instance, (uint8_t *)hpcd->Setup);
  }

  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

// HAL_StatusTypeDef HAL_PCD_EP_ClrStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
// {
//   PCD_EPTypeDef *ep;

//   if (((uint32_t)ep_addr & 0x0FU) > hpcd->Init.dev_endpoints)
//   {
//     return HAL_ERROR;
//   }

//   if ((0x80U & ep_addr) == 0x80U)
//   {
//     ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
//     ep->is_in = 1U;
//   }
//   else
//   {
//     ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
//     ep->is_in = 0U;
//   }

//   ep->is_stall = 0U;
//   ep->num = ep_addr & EP_ADDR_MSK;

//   __HAL_LOCK(hpcd);
//   (void)USB_EPClearStall(hpcd->Instance, ep);
//   __HAL_UNLOCK(hpcd);

//   return HAL_OK;
// }

// static HAL_StatusTypeDef PCD_EP_ISR_Handler(PCD_HandleTypeDef *hpcd)
// {
//   PCD_EPTypeDef *ep;
//   uint16_t count;
//   uint16_t wIstr;
//   uint16_t wEPVal;
//   uint16_t TxPctSize;
//   uint8_t epindex;

// #if (USE_USB_DOUBLE_BUFFER != 1U)
//   count = 0U;
// #endif /* USE_USB_DOUBLE_BUFFER */

//   /* stay in loop while pending interrupts */
//   while ((hpcd->Instance->ISTR & USB_ISTR_CTR) != 0U)
//   {
//     wIstr = hpcd->Instance->ISTR;

//     /* extract highest priority endpoint number */
//     epindex = (uint8_t)(wIstr & USB_ISTR_EP_ID);

//     if (epindex == 0U)
//     {
//       /* Decode and service control endpoint interrupt */

//       /* DIR bit = origin of the interrupt */
//       if ((wIstr & USB_ISTR_DIR) == 0U)
//       {
//         /* DIR = 0 */

//         /* DIR = 0 => IN  int */
//         /* DIR = 0 implies that (EP_CTR_TX = 1) always */
//         PCD_CLEAR_TX_EP_CTR(hpcd->Instance, PCD_ENDP0);
//         ep = &hpcd->IN_ep[0];

//         ep->xfer_count = PCD_GET_EP_TX_CNT(hpcd->Instance, ep->num);
//         ep->xfer_buff += ep->xfer_count;

//         HAL_PCD_DataInStageCallback(hpcd, 0U);

//         if ((hpcd->USB_Address > 0U) && (ep->xfer_len == 0U))
//         {
//           hpcd->Instance->DADDR = ((uint16_t)hpcd->USB_Address | USB_DADDR_EF);
//           hpcd->USB_Address = 0U;
//         }
//       }
//       else
//       {
//         /* DIR = 1 */

//         /* DIR = 1 & CTR_RX => SETUP or OUT int */
//         /* DIR = 1 & (CTR_TX | CTR_RX) => 2 int pending */
//         ep = &hpcd->OUT_ep[0];
//         wEPVal = PCD_GET_ENDPOINT(hpcd->Instance, PCD_ENDP0);

//         if ((wEPVal & USB_EP_SETUP) != 0U)
//         {
//           /* Get SETUP Packet */
//           ep->xfer_count = PCD_GET_EP_RX_CNT(hpcd->Instance, ep->num);

//           USB_ReadPMA(hpcd->Instance, (uint8_t *)hpcd->Setup,
//                       ep->pmaadress, (uint16_t)ep->xfer_count);

//           /* SETUP bit kept frozen while CTR_RX = 1 */
//           PCD_CLEAR_RX_EP_CTR(hpcd->Instance, PCD_ENDP0);

//           HAL_PCD_SetupStageCallback(hpcd);
//         }
//         else if ((wEPVal & USB_EP_CTR_RX) != 0U)
//         {
//           PCD_CLEAR_RX_EP_CTR(hpcd->Instance, PCD_ENDP0);

//           /* Get Control Data OUT Packet */
//           ep->xfer_count = PCD_GET_EP_RX_CNT(hpcd->Instance, ep->num);

//           if ((ep->xfer_count != 0U) && (ep->xfer_buff != 0U))
//           {
//             USB_ReadPMA(hpcd->Instance, ep->xfer_buff,
//                         ep->pmaadress, (uint16_t)ep->xfer_count);

//             ep->xfer_buff += ep->xfer_count;

//             HAL_PCD_DataOutStageCallback(hpcd, 0U);
//           }

//           wEPVal = (uint16_t)PCD_GET_ENDPOINT(hpcd->Instance, PCD_ENDP0);

//           if (((wEPVal & USB_EP_SETUP) == 0U) && ((wEPVal & USB_EP_RX_STRX) != USB_EP_RX_VALID))
//           {
//             PCD_SET_EP_RX_CNT(hpcd->Instance, PCD_ENDP0, ep->maxpacket);
//             PCD_SET_EP_RX_STATUS(hpcd->Instance, PCD_ENDP0, USB_EP_RX_VALID);
//           }
//         }
//       }
//     }
//     else
//     {
//       /* Decode and service non control endpoints interrupt */
//       /* process related endpoint register */
//       wEPVal = PCD_GET_ENDPOINT(hpcd->Instance, epindex);

//       if ((wEPVal & USB_EP_CTR_RX) != 0U)
//       {
//         /* clear int flag */
//         PCD_CLEAR_RX_EP_CTR(hpcd->Instance, epindex);
//         ep = &hpcd->OUT_ep[epindex];

//         /* OUT Single Buffering */
//         if (ep->doublebuffer == 0U)
//         {
//           count = (uint16_t)PCD_GET_EP_RX_CNT(hpcd->Instance, ep->num);

//           if (count != 0U)
//           {
//             USB_ReadPMA(hpcd->Instance, ep->xfer_buff, ep->pmaadress, count);
//           }
//         }
// #if (USE_USB_DOUBLE_BUFFER == 1U)
//         else
//         {
//           /* manage double buffer bulk out */
//           if (ep->type == EP_TYPE_BULK)
//           {
//             count = HAL_PCD_EP_DB_Receive(hpcd, ep, wEPVal);
//           }
//           else /* manage double buffer iso out */
//           {
//             /* free EP OUT Buffer */
//             PCD_FREE_USER_BUFFER(hpcd->Instance, ep->num, 0U);

//             if ((PCD_GET_ENDPOINT(hpcd->Instance, ep->num) & USB_EP_DTOG_RX) != 0U)
//             {
//               /* read from endpoint BUF0Addr buffer */
//               count = (uint16_t)PCD_GET_EP_DBUF0_CNT(hpcd->Instance, ep->num);

//               if (count != 0U)
//               {
//                 USB_ReadPMA(hpcd->Instance, ep->xfer_buff, ep->pmaaddr0, count);
//               }
//             }
//             else
//             {
//               /* read from endpoint BUF1Addr buffer */
//               count = (uint16_t)PCD_GET_EP_DBUF1_CNT(hpcd->Instance, ep->num);

//               if (count != 0U)
//               {
//                 USB_ReadPMA(hpcd->Instance, ep->xfer_buff, ep->pmaaddr1, count);
//               }
//             }
//           }
//         }
// #endif /* (USE_USB_DOUBLE_BUFFER == 1U) */

//         /* multi-packet on the NON control OUT endpoint */
//         ep->xfer_count += count;
//         ep->xfer_buff += count;

//         if ((ep->xfer_len == 0U) || (count < ep->maxpacket))
//         {
//           /* RX COMPLETE */
// #if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
//           hpcd->DataOutStageCallback(hpcd, ep->num);
// #else
//           HAL_PCD_DataOutStageCallback(hpcd, ep->num);
// #endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
//         }
//         else
//         {
//           (void)USB_EPStartXfer(hpcd->Instance, ep);
//         }
//       }

//       if ((wEPVal & USB_EP_CTR_TX) != 0U)
//       {
//         ep = &hpcd->IN_ep[epindex];

//         /* clear int flag */
//         PCD_CLEAR_TX_EP_CTR(hpcd->Instance, epindex);

//         if (ep->type == EP_TYPE_ISOC)
//         {
//           ep->xfer_len = 0U;

// #if (USE_USB_DOUBLE_BUFFER == 1U)
//           if (ep->doublebuffer != 0U)
//           {
//             if ((wEPVal & USB_EP_DTOG_TX) != 0U)
//             {
//               PCD_SET_EP_DBUF0_CNT(hpcd->Instance, ep->num, ep->is_in, 0U);
//             }
//             else
//             {
//               PCD_SET_EP_DBUF1_CNT(hpcd->Instance, ep->num, ep->is_in, 0U);
//             }
//           }
// #endif /* (USE_USB_DOUBLE_BUFFER == 1U) */

//           /* TX COMPLETE */
// #if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
//           hpcd->DataInStageCallback(hpcd, ep->num);
// #else
//           HAL_PCD_DataInStageCallback(hpcd, ep->num);
// #endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
//         }
//         else
//         {
//           /* Manage Single Buffer Transaction */
//           if ((wEPVal & USB_EP_KIND) == 0U)
//           {
//             /* multi-packet on the NON control IN endpoint */
//             TxPctSize = (uint16_t)PCD_GET_EP_TX_CNT(hpcd->Instance, ep->num);

//             if (ep->xfer_len > TxPctSize)
//             {
//               ep->xfer_len -= TxPctSize;
//             }
//             else
//             {
//               ep->xfer_len = 0U;
//             }

//             /* Zero Length Packet? */
//             if (ep->xfer_len == 0U)
//             {
//               /* TX COMPLETE */
// #if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
//               hpcd->DataInStageCallback(hpcd, ep->num);
// #else
//               HAL_PCD_DataInStageCallback(hpcd, ep->num);
// #endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
//             }
//             else
//             {
//               /* Transfer is not yet Done */
//               ep->xfer_buff += TxPctSize;
//               ep->xfer_count += TxPctSize;
//               (void)USB_EPStartXfer(hpcd->Instance, ep);
//             }
//           }
// #if (USE_USB_DOUBLE_BUFFER == 1U)
//           /* Double Buffer bulk IN (bulk transfer Len > Ep_Mps) */
//           else
//           {
//             (void)HAL_PCD_EP_DB_Transmit(hpcd, ep, wEPVal);
//           }
// #endif /* (USE_USB_DOUBLE_BUFFER == 1U) */
//         }
//       }
//     }
//   }

//   return HAL_OK;
// }

// static uint16_t HAL_PCD_EP_DB_Receive(PCD_HandleTypeDef *hpcd,
//                                       PCD_EPTypeDef *ep, uint16_t wEPVal)
// {
//   uint16_t count;

//   /* Manage Buffer0 OUT */
//   if ((wEPVal & USB_EP_DTOG_RX) != 0U)
//   {
//     /* Get count of received Data on buffer0 */
//     count = (uint16_t)PCD_GET_EP_DBUF0_CNT(hpcd->Instance, ep->num);

//     if (ep->xfer_len >= count)
//     {
//       ep->xfer_len -= count;
//     }
//     else
//     {
//       ep->xfer_len = 0U;
//     }

//     if (ep->xfer_len == 0U)
//     {
//       /* set NAK to OUT endpoint since double buffer is enabled */
//       PCD_SET_EP_RX_STATUS(hpcd->Instance, ep->num, USB_EP_RX_NAK);
//     }

//     /* Check if Buffer1 is in blocked state which requires to toggle */
//     if ((wEPVal & USB_EP_DTOG_TX) != 0U)
//     {
//       PCD_FREE_USER_BUFFER(hpcd->Instance, ep->num, 0U);
//     }

//     if (count != 0U)
//     {
//       USB_ReadPMA(hpcd->Instance, ep->xfer_buff, ep->pmaaddr0, count);
//     }
//   }
//   /* Manage Buffer 1 DTOG_RX=0 */
//   else
//   {
//     /* Get count of received data */
//     count = (uint16_t)PCD_GET_EP_DBUF1_CNT(hpcd->Instance, ep->num);

//     if (ep->xfer_len >= count)
//     {
//       ep->xfer_len -= count;
//     }
//     else
//     {
//       ep->xfer_len = 0U;
//     }

//     if (ep->xfer_len == 0U)
//     {
//       /* set NAK on the current endpoint */
//       PCD_SET_EP_RX_STATUS(hpcd->Instance, ep->num, USB_EP_RX_NAK);
//     }

//     /*Need to FreeUser Buffer*/
//     if ((wEPVal & USB_EP_DTOG_TX) == 0U)
//     {
//       PCD_FREE_USER_BUFFER(hpcd->Instance, ep->num, 0U);
//     }

//     if (count != 0U)
//     {
//       USB_ReadPMA(hpcd->Instance, ep->xfer_buff, ep->pmaaddr1, count);
//     }
//   }

//   return count;
// }

// static HAL_StatusTypeDef HAL_PCD_EP_DB_Transmit(PCD_HandleTypeDef *hpcd, PCD_EPTypeDef *ep, uint16_t wEPVal)
// {
//   uint32_t len;
//   uint16_t TxPctSize;

//   /* Data Buffer0 ACK received */
//   if ((wEPVal & USB_EP_DTOG_TX) != 0U)
//   {
//     /* multi-packet on the NON control IN endpoint */
//     TxPctSize = (uint16_t)PCD_GET_EP_DBUF0_CNT(hpcd->Instance, ep->num);

//     if (ep->xfer_len > TxPctSize)
//     {
//       ep->xfer_len -= TxPctSize;
//     }
//     else
//     {
//       ep->xfer_len = 0U;
//     }

//     /* Transfer is completed */
//     if (ep->xfer_len == 0U)
//     {
//       PCD_SET_EP_DBUF0_CNT(hpcd->Instance, ep->num, ep->is_in, 0U);
//       PCD_SET_EP_DBUF1_CNT(hpcd->Instance, ep->num, ep->is_in, 0U);

//       /* TX COMPLETE */
// #if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
//       hpcd->DataInStageCallback(hpcd, ep->num);
// #else
//       HAL_PCD_DataInStageCallback(hpcd, ep->num);
// #endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

//       if ((wEPVal & USB_EP_DTOG_RX) != 0U)
//       {
//         PCD_FREE_USER_BUFFER(hpcd->Instance, ep->num, 1U);
//       }
//     }
//     else /* Transfer is not yet Done */
//     {
//       /* need to Free USB Buff */
//       if ((wEPVal & USB_EP_DTOG_RX) != 0U)
//       {
//         PCD_FREE_USER_BUFFER(hpcd->Instance, ep->num, 1U);
//       }

//       /* Still there is data to Fill in the next Buffer */
//       if (ep->xfer_fill_db == 1U)
//       {
//         ep->xfer_buff += TxPctSize;
//         ep->xfer_count += TxPctSize;

//         /* Calculate the len of the new buffer to fill */
//         if (ep->xfer_len_db >= ep->maxpacket)
//         {
//           len = ep->maxpacket;
//           ep->xfer_len_db -= len;
//         }
//         else if (ep->xfer_len_db == 0U)
//         {
//           len = TxPctSize;
//           ep->xfer_fill_db = 0U;
//         }
//         else
//         {
//           ep->xfer_fill_db = 0U;
//           len = ep->xfer_len_db;
//           ep->xfer_len_db = 0U;
//         }

//         /* Write remaining Data to Buffer */
//         /* Set the Double buffer counter for pma buffer1 */
//         PCD_SET_EP_DBUF0_CNT(hpcd->Instance, ep->num, ep->is_in, len);

//         /* Copy user buffer to USB PMA */
//         USB_WritePMA(hpcd->Instance, ep->xfer_buff, ep->pmaaddr0, (uint16_t)len);
//       }
//     }
//   }
//   else /* Data Buffer1 ACK received */
//   {
//     /* multi-packet on the NON control IN endpoint */
//     TxPctSize = (uint16_t)PCD_GET_EP_DBUF1_CNT(hpcd->Instance, ep->num);

//     if (ep->xfer_len >= TxPctSize)
//     {
//       ep->xfer_len -= TxPctSize;
//     }
//     else
//     {
//       ep->xfer_len = 0U;
//     }

//     /* Transfer is completed */
//     if (ep->xfer_len == 0U)
//     {
//       PCD_SET_EP_DBUF0_CNT(hpcd->Instance, ep->num, ep->is_in, 0U);
//       PCD_SET_EP_DBUF1_CNT(hpcd->Instance, ep->num, ep->is_in, 0U);

//       /* TX COMPLETE */
// #if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
//       hpcd->DataInStageCallback(hpcd, ep->num);
// #else
//       HAL_PCD_DataInStageCallback(hpcd, ep->num);
// #endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

//       /* need to Free USB Buff */
//       if ((wEPVal & USB_EP_DTOG_RX) == 0U)
//       {
//         PCD_FREE_USER_BUFFER(hpcd->Instance, ep->num, 1U);
//       }
//     }
//     else /* Transfer is not yet Done */
//     {
//       /* need to Free USB Buff */
//       if ((wEPVal & USB_EP_DTOG_RX) == 0U)
//       {
//         PCD_FREE_USER_BUFFER(hpcd->Instance, ep->num, 1U);
//       }

//       /* Still there is data to Fill in the next Buffer */
//       if (ep->xfer_fill_db == 1U)
//       {
//         ep->xfer_buff += TxPctSize;
//         ep->xfer_count += TxPctSize;

//         /* Calculate the len of the new buffer to fill */
//         if (ep->xfer_len_db >= ep->maxpacket)
//         {
//           len = ep->maxpacket;
//           ep->xfer_len_db -= len;
//         }
//         else if (ep->xfer_len_db == 0U)
//         {
//           len = TxPctSize;
//           ep->xfer_fill_db = 0U;
//         }
//         else
//         {
//           len = ep->xfer_len_db;
//           ep->xfer_len_db = 0U;
//           ep->xfer_fill_db = 0;
//         }

//         /* Set the Double buffer counter for pmabuffer1 */
//         PCD_SET_EP_DBUF1_CNT(hpcd->Instance, ep->num, ep->is_in, len);

//         /* Copy the user buffer to USB PMA */
//         USB_WritePMA(hpcd->Instance, ep->xfer_buff, ep->pmaaddr1, (uint16_t)len);
//       }
//     }
//   }

//   /*enable endpoint IN*/
//   PCD_SET_EP_TX_STATUS(hpcd->Instance, ep->num, USB_EP_TX_VALID);

//   return HAL_OK;
// }

// #endif /* HAL_PCD_MODULE_ENABLED */

// #ifdef HAL_MODULE_ENABLED

// #define IDCODE_DEVID_MASK 0x00000FFFU
// __IO uint32_t uwTick;

// uint32_t uwTickPrio = (1UL << __NVIC_PRIO_BITS); /* Invalid PRIO */

// HAL_TickFreqTypeDef uwTickFreq = HAL_TICK_FREQ_DEFAULT; /* 1KHz */

// __weak HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
// {
//   /* Configure the SysTick to have interrupt in 1ms time basis*/
//   if (HAL_SYSTICK_Config(SystemCoreClock / (1000U / uwTickFreq)) > 0U)
//   {
//     return HAL_ERROR;
//   }

//   /* Configure the SysTick IRQ priority */
//   if (TickPriority < (1UL << __NVIC_PRIO_BITS))
//   {
//     HAL_NVIC_SetPriority(SysTick_IRQn, TickPriority, 0U);
//     uwTickPrio = TickPriority;
//   }
//   else
//   {
//     return HAL_ERROR;
//   }

//   /* Return function status */
//   return HAL_OK;
// }

// __weak void HAL_IncTick(void)
// {
//   uwTick += uwTickFreq;
// }

// __weak void HAL_Delay(uint32_t Delay)
// {
//   uint32_t tickstart = HAL_GetTick();
//   uint32_t wait = Delay;

//   /* Add a freq to guarantee minimum wait */
//   if (wait < HAL_MAX_DELAY)
//   {
//     wait += (uint32_t)(uwTickFreq);
//   }

//   while ((HAL_GetTick() - tickstart) < wait)
//   {
//   }
// }

// #endif /* HAL_MODULE_ENABLED */

// #ifndef __USBD_CORE
// #define __USBD_CORE

HAL_StatusTypeDef USB_CoreInit(USB_TypeDef *USBx, USB_CfgTypeDef cfg){
  UNUSED(USBx);
  UNUSED(cfg);
  return HAL_OK;
}

HAL_StatusTypeDef USB_EnableGlobalInt(USB_TypeDef *USBx){
  uint32_t winterruptmask;

  /* Clear pending interrupts */
  USBx->ISTR = 0U;

  /* Set winterruptmask variable */
  winterruptmask = USB_CNTR_CTRM  | USB_CNTR_WKUPM |
                   USB_CNTR_SUSPM | USB_CNTR_ERRM |
                   USB_CNTR_SOFM | USB_CNTR_ESOFM |
                   USB_CNTR_RESETM;

  /* Set interrupt mask */
  USBx->CNTR = (uint16_t)winterruptmask;

  return HAL_OK;
}

HAL_StatusTypeDef USB_DisableGlobalInt(USB_TypeDef *USBx){
  uint32_t winterruptmask;

  /* Set winterruptmask variable */
  winterruptmask = USB_CNTR_CTRM  | USB_CNTR_WKUPM |
                   USB_CNTR_SUSPM | USB_CNTR_ERRM |
                   USB_CNTR_SOFM | USB_CNTR_ESOFM |
                   USB_CNTR_RESETM;

  /* Clear interrupt mask */
  USBx->CNTR &= (uint16_t)(~winterruptmask);

  return HAL_OK;
}

HAL_StatusTypeDef USB_SetCurrentMode(USB_TypeDef *USBx, USB_ModeTypeDef mode){
  /* Prevent unused argument(s) compilation warning */
  UNUSED(USBx);
  UNUSED(mode);

  /* NOTE : - This function is not required by USB Device FS peripheral, it is used
              only by USB OTG FS peripheral.
            - This function is added to ensure compatibility across platforms.
   */
  return HAL_OK;
}

HAL_StatusTypeDef USB_DevInit(USB_TypeDef *USBx, USB_CfgTypeDef cfg){
  /* Prevent unused argument(s) compilation warning */
  UNUSED(cfg);

  /* Init Device */
  /* CNTR_FRES = 1 */
  USBx->CNTR = (uint16_t)USB_CNTR_FRES;

  /* CNTR_FRES = 0 */
  USBx->CNTR = 0U;

  /* Clear pending interrupts */
  USBx->ISTR = 0U;

  /*Set Btable Address*/
  USBx->BTABLE = BTABLE_ADDRESS;

  return HAL_OK;
}

HAL_StatusTypeDef USB_ActivateEndpoint(USB_TypeDef *USBx, USB_EPTypeDef *ep){
  HAL_StatusTypeDef ret = HAL_OK;
  uint16_t wEpRegVal;

  wEpRegVal = PCD_GET_ENDPOINT(USBx, ep->num) & USB_EP_T_MASK;

  /* initialize Endpoint */
  switch (ep->type)
  {
    case EP_TYPE_CTRL:
      wEpRegVal |= USB_EP_CONTROL;
      break;

    case EP_TYPE_BULK:
      wEpRegVal |= USB_EP_BULK;
      break;

    case EP_TYPE_INTR:
      wEpRegVal |= USB_EP_INTERRUPT;
      break;

    case EP_TYPE_ISOC:
      wEpRegVal |= USB_EP_ISOCHRONOUS;
      break;

    default:
      ret = HAL_ERROR;
      break;
  }

  PCD_SET_ENDPOINT(USBx, ep->num, (wEpRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX));

  PCD_SET_EP_ADDRESS(USBx, ep->num, ep->num);

  if (ep->doublebuffer == 0U)
  {
    if (ep->is_in != 0U)
    {
      /*Set the endpoint Transmit buffer address */
      PCD_SET_EP_TX_ADDRESS(USBx, ep->num, ep->pmaadress);
      PCD_CLEAR_TX_DTOG(USBx, ep->num);

      if (ep->type != EP_TYPE_ISOC)
      {
        /* Configure NAK status for the Endpoint */
        PCD_SET_EP_TX_STATUS(USBx, ep->num, USB_EP_TX_NAK);
      }
      else
      {
        /* Configure TX Endpoint to disabled state */
        PCD_SET_EP_TX_STATUS(USBx, ep->num, USB_EP_TX_DIS);
      }
    }
    else
    {
      /* Set the endpoint Receive buffer address */
      PCD_SET_EP_RX_ADDRESS(USBx, ep->num, ep->pmaadress);

      /* Set the endpoint Receive buffer counter */
      PCD_SET_EP_RX_CNT(USBx, ep->num, ep->maxpacket);
      PCD_CLEAR_RX_DTOG(USBx, ep->num);

      if (ep->num == 0U)
      {
        /* Configure VALID status for EP0 */
        PCD_SET_EP_RX_STATUS(USBx, ep->num, USB_EP_RX_VALID);
      }
      else
      {
        /* Configure NAK status for OUT Endpoint */
        PCD_SET_EP_RX_STATUS(USBx, ep->num, USB_EP_RX_NAK);
      }
    }
  }
#if (USE_USB_DOUBLE_BUFFER == 1U)
  /* Double Buffer */
  else
  {
    if (ep->type == EP_TYPE_BULK)
    {
      /* Set bulk endpoint as double buffered */
      PCD_SET_BULK_EP_DBUF(USBx, ep->num);
    }
    else
    {
      /* Set the ISOC endpoint in double buffer mode */
      PCD_CLEAR_EP_KIND(USBx, ep->num);
    }

    /* Set buffer address for double buffered mode */
    PCD_SET_EP_DBUF_ADDR(USBx, ep->num, ep->pmaaddr0, ep->pmaaddr1);

    if (ep->is_in == 0U)
    {
      /* Clear the data toggle bits for the endpoint IN/OUT */
      PCD_CLEAR_RX_DTOG(USBx, ep->num);
      PCD_CLEAR_TX_DTOG(USBx, ep->num);

      PCD_SET_EP_RX_STATUS(USBx, ep->num, USB_EP_RX_VALID);
      PCD_SET_EP_TX_STATUS(USBx, ep->num, USB_EP_TX_DIS);
    }
    else
    {
      /* Clear the data toggle bits for the endpoint IN/OUT */
      PCD_CLEAR_RX_DTOG(USBx, ep->num);
      PCD_CLEAR_TX_DTOG(USBx, ep->num);

      if (ep->type != EP_TYPE_ISOC)
      {
        /* Configure NAK status for the Endpoint */
        PCD_SET_EP_TX_STATUS(USBx, ep->num, USB_EP_TX_NAK);
      }
      else
      {
        /* Configure TX Endpoint to disabled state */
        PCD_SET_EP_TX_STATUS(USBx, ep->num, USB_EP_TX_DIS);
      }

      PCD_SET_EP_RX_STATUS(USBx, ep->num, USB_EP_RX_DIS);
    }
  }
#endif /* (USE_USB_DOUBLE_BUFFER == 1U) */

  return ret;
}

HAL_StatusTypeDef USB_DeactivateEndpoint(USB_TypeDef *USBx, USB_EPTypeDef *ep){
  if (ep->doublebuffer == 0U)
  {
    if (ep->is_in != 0U)
    {
      PCD_CLEAR_TX_DTOG(USBx, ep->num);

      /* Configure DISABLE status for the Endpoint */
      PCD_SET_EP_TX_STATUS(USBx, ep->num, USB_EP_TX_DIS);
    }

    else
    {
      PCD_CLEAR_RX_DTOG(USBx, ep->num);

      /* Configure DISABLE status for the Endpoint */
      PCD_SET_EP_RX_STATUS(USBx, ep->num, USB_EP_RX_DIS);
    }
  }
#if (USE_USB_DOUBLE_BUFFER == 1U)
  /* Double Buffer */
  else
  {
    if (ep->is_in == 0U)
    {
      /* Clear the data toggle bits for the endpoint IN/OUT*/
      PCD_CLEAR_RX_DTOG(USBx, ep->num);
      PCD_CLEAR_TX_DTOG(USBx, ep->num);

      /* Reset value of the data toggle bits for the endpoint out*/
      PCD_TX_DTOG(USBx, ep->num);

      PCD_SET_EP_RX_STATUS(USBx, ep->num, USB_EP_RX_DIS);
      PCD_SET_EP_TX_STATUS(USBx, ep->num, USB_EP_TX_DIS);
    }
    else
    {
      /* Clear the data toggle bits for the endpoint IN/OUT*/
      PCD_CLEAR_RX_DTOG(USBx, ep->num);
      PCD_CLEAR_TX_DTOG(USBx, ep->num);
      PCD_RX_DTOG(USBx, ep->num);

      /* Configure DISABLE status for the Endpoint*/
      PCD_SET_EP_TX_STATUS(USBx, ep->num, USB_EP_TX_DIS);
      PCD_SET_EP_RX_STATUS(USBx, ep->num, USB_EP_RX_DIS);
    }
  }
#endif /* (USE_USB_DOUBLE_BUFFER == 1U) */

  return HAL_OK;
}

HAL_StatusTypeDef USB_EPStartXfer(USB_TypeDef *USBx, USB_EPTypeDef *ep){
  uint32_t len;
#if (USE_USB_DOUBLE_BUFFER == 1U)
  uint16_t pmabuffer;
  uint16_t wEPVal;
#endif /* (USE_USB_DOUBLE_BUFFER == 1U) */

  /* IN endpoint */
  if (ep->is_in == 1U)
  {
    /*Multi packet transfer*/
    if (ep->xfer_len > ep->maxpacket)
    {
      len = ep->maxpacket;
    }
    else
    {
      len = ep->xfer_len;
    }

    /* configure and validate Tx endpoint */
    if (ep->doublebuffer == 0U)
    {
      USB_WritePMA(USBx, ep->xfer_buff, ep->pmaadress, (uint16_t)len);
      PCD_SET_EP_TX_CNT(USBx, ep->num, len);
    }
#if (USE_USB_DOUBLE_BUFFER == 1U)
    else
    {
      /* double buffer bulk management */
      if (ep->type == EP_TYPE_BULK)
      {
        if (ep->xfer_len_db > ep->maxpacket)
        {
          /* enable double buffer */
          PCD_SET_BULK_EP_DBUF(USBx, ep->num);

          /* each Time to write in PMA xfer_len_db will */
          ep->xfer_len_db -= len;

          /* Fill the two first buffer in the Buffer0 & Buffer1 */
          if ((PCD_GET_ENDPOINT(USBx, ep->num) & USB_EP_DTOG_TX) != 0U)
          {
            /* Set the Double buffer counter for pmabuffer1 */
            PCD_SET_EP_DBUF1_CNT(USBx, ep->num, ep->is_in, len);
            pmabuffer = ep->pmaaddr1;

            /* Write the user buffer to USB PMA */
            USB_WritePMA(USBx, ep->xfer_buff, pmabuffer, (uint16_t)len);
            ep->xfer_buff += len;

            if (ep->xfer_len_db > ep->maxpacket)
            {
              ep->xfer_len_db -= len;
            }
            else
            {
              len = ep->xfer_len_db;
              ep->xfer_len_db = 0U;
            }

            /* Set the Double buffer counter for pmabuffer0 */
            PCD_SET_EP_DBUF0_CNT(USBx, ep->num, ep->is_in, len);
            pmabuffer = ep->pmaaddr0;

            /* Write the user buffer to USB PMA */
            USB_WritePMA(USBx, ep->xfer_buff, pmabuffer, (uint16_t)len);
          }
          else
          {
            /* Set the Double buffer counter for pmabuffer0 */
            PCD_SET_EP_DBUF0_CNT(USBx, ep->num, ep->is_in, len);
            pmabuffer = ep->pmaaddr0;

            /* Write the user buffer to USB PMA */
            USB_WritePMA(USBx, ep->xfer_buff, pmabuffer, (uint16_t)len);
            ep->xfer_buff += len;

            if (ep->xfer_len_db > ep->maxpacket)
            {
              ep->xfer_len_db -= len;
            }
            else
            {
              len = ep->xfer_len_db;
              ep->xfer_len_db = 0U;
            }

            /* Set the Double buffer counter for pmabuffer1 */
            PCD_SET_EP_DBUF1_CNT(USBx, ep->num, ep->is_in, len);
            pmabuffer = ep->pmaaddr1;

            /* Write the user buffer to USB PMA */
            USB_WritePMA(USBx, ep->xfer_buff, pmabuffer, (uint16_t)len);
          }
        }
        /* auto Switch to single buffer mode when transfer <Mps no need to manage in double buffer */
        else
        {
          len = ep->xfer_len_db;

          /* disable double buffer mode for Bulk endpoint */
          PCD_CLEAR_BULK_EP_DBUF(USBx, ep->num);

          /* Set Tx count with nbre of byte to be transmitted */
          PCD_SET_EP_TX_CNT(USBx, ep->num, len);
          pmabuffer = ep->pmaaddr0;

          /* Write the user buffer to USB PMA */
          USB_WritePMA(USBx, ep->xfer_buff, pmabuffer, (uint16_t)len);
        }
      }
      else /* manage isochronous double buffer IN mode */
      {
        /* each Time to write in PMA xfer_len_db will */
        ep->xfer_len_db -= len;

        /* Fill the data buffer */
        if ((PCD_GET_ENDPOINT(USBx, ep->num) & USB_EP_DTOG_TX) != 0U)
        {
          /* Set the Double buffer counter for pmabuffer1 */
          PCD_SET_EP_DBUF1_CNT(USBx, ep->num, ep->is_in, len);
          pmabuffer = ep->pmaaddr1;

          /* Write the user buffer to USB PMA */
          USB_WritePMA(USBx, ep->xfer_buff, pmabuffer, (uint16_t)len);
        }
        else
        {
          /* Set the Double buffer counter for pmabuffer0 */
          PCD_SET_EP_DBUF0_CNT(USBx, ep->num, ep->is_in, len);
          pmabuffer = ep->pmaaddr0;

          /* Write the user buffer to USB PMA */
          USB_WritePMA(USBx, ep->xfer_buff, pmabuffer, (uint16_t)len);
        }
      }
    }
#endif /* (USE_USB_DOUBLE_BUFFER == 1U) */

    PCD_SET_EP_TX_STATUS(USBx, ep->num, USB_EP_TX_VALID);
  }
  else /* OUT endpoint */
  {
    if (ep->doublebuffer == 0U)
    {
      /* Multi packet transfer */
      if (ep->xfer_len > ep->maxpacket)
      {
        len = ep->maxpacket;
        ep->xfer_len -= len;
      }
      else
      {
        len = ep->xfer_len;
        ep->xfer_len = 0U;
      }
      /* configure and validate Rx endpoint */
      PCD_SET_EP_RX_CNT(USBx, ep->num, len);
    }
#if (USE_USB_DOUBLE_BUFFER == 1U)
    else
    {
      /* First Transfer Coming From HAL_PCD_EP_Receive & From ISR */
      /* Set the Double buffer counter */
      if (ep->type == EP_TYPE_BULK)
      {
        PCD_SET_EP_DBUF_CNT(USBx, ep->num, ep->is_in, ep->maxpacket);

        /* Coming from ISR */
        if (ep->xfer_count != 0U)
        {
          /* update last value to check if there is blocking state */
          wEPVal = PCD_GET_ENDPOINT(USBx, ep->num);

          /*Blocking State */
          if ((((wEPVal & USB_EP_DTOG_RX) != 0U) && ((wEPVal & USB_EP_DTOG_TX) != 0U)) ||
              (((wEPVal & USB_EP_DTOG_RX) == 0U) && ((wEPVal & USB_EP_DTOG_TX) == 0U)))
          {
            PCD_FREE_USER_BUFFER(USBx, ep->num, 0U);
          }
        }
      }
      /* iso out double */
      else if (ep->type == EP_TYPE_ISOC)
      {
        /* Multi packet transfer */
        if (ep->xfer_len > ep->maxpacket)
        {
          len = ep->maxpacket;
          ep->xfer_len -= len;
        }
        else
        {
          len = ep->xfer_len;
          ep->xfer_len = 0U;
        }
        PCD_SET_EP_DBUF_CNT(USBx, ep->num, ep->is_in, len);
      }
      else
      {
        return HAL_ERROR;
      }
    }
#endif /* (USE_USB_DOUBLE_BUFFER == 1U) */

    PCD_SET_EP_RX_STATUS(USBx, ep->num, USB_EP_RX_VALID);
  }

  return HAL_OK;
}

HAL_StatusTypeDef USB_EPSetStall(USB_TypeDef *USBx, USB_EPTypeDef *ep){
  if (ep->is_in != 0U)
  {
    PCD_SET_EP_TX_STATUS(USBx, ep->num, USB_EP_TX_STALL);
  }
  else
  {
    PCD_SET_EP_RX_STATUS(USBx, ep->num, USB_EP_RX_STALL);
  }

  return HAL_OK;
}

// HAL_StatusTypeDef USB_EPClearStall(USB_TypeDef *USBx, USB_EPTypeDef *ep){
//   if (ep->doublebuffer == 0U)
//   {
//     if (ep->is_in != 0U)
//     {
//       PCD_CLEAR_TX_DTOG(USBx, ep->num);

//       if (ep->type != EP_TYPE_ISOC)
//       {
//         /* Configure NAK status for the Endpoint */
//         PCD_SET_EP_TX_STATUS(USBx, ep->num, USB_EP_TX_NAK);
//       }
//     }
//     else
//     {
//       PCD_CLEAR_RX_DTOG(USBx, ep->num);

//       /* Configure VALID status for the Endpoint */
//       PCD_SET_EP_RX_STATUS(USBx, ep->num, USB_EP_RX_VALID);
//     }
//   }

//   return HAL_OK;
// }

// HAL_StatusTypeDef USB_SetDevAddress(USB_TypeDef *USBx, uint8_t address){
//   if (address == 0U)
//   {
//     /* set device address and enable function */
//     USBx->DADDR = (uint16_t)USB_DADDR_EF;
//   }

//   return HAL_OK;
// }

HAL_StatusTypeDef USB_DevConnect(USB_TypeDef *USBx){
  /* Prevent unused argument(s) compilation warning */
  UNUSED(USBx);

  /* NOTE : - This function is not required by USB Device FS peripheral, it is used
              only by USB OTG FS peripheral.
            - This function is added to ensure compatibility across platforms.
   */

  return HAL_OK;
}

HAL_StatusTypeDef USB_DevDisconnect(USB_TypeDef *USBx){
  /* Prevent unused argument(s) compilation warning */
  UNUSED(USBx);

  /* NOTE : - This function is not required by USB Device FS peripheral, it is used
              only by USB OTG FS peripheral.
            - This function is added to ensure compatibility across platforms.
   */

  return HAL_OK;
}

// uint32_t USB_ReadInterrupts(USB_TypeDef const *USBx){
//   uint32_t tmpreg;

//   tmpreg = USBx->ISTR;
//   return tmpreg;
// }

HAL_StatusTypeDef USB_EP0_OutStart(USB_TypeDef *USBx, uint8_t *psetup)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(USBx);
  UNUSED(psetup);
  /* NOTE : - This function is not required by USB Device FS peripheral, it is used
              only by USB OTG FS peripheral.
            - This function is added to ensure compatibility across platforms.
   */
  return HAL_OK;
}

void USB_WritePMA(USB_TypeDef const *USBx, uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes){
  uint32_t n = ((uint32_t)wNBytes + 1U) >> 1;
  uint32_t BaseAddr = (uint32_t)USBx;
  uint32_t count;
  uint16_t WrVal;
  __IO uint16_t *pdwVal;
  uint8_t *pBuf = pbUsrBuf;

  pdwVal = (__IO uint16_t *)(BaseAddr + 0x400U + ((uint32_t)wPMABufAddr * PMA_ACCESS));

  for (count = n; count != 0U; count--)
  {
    WrVal = pBuf[0];
    WrVal |= (uint16_t)pBuf[1] << 8;
    *pdwVal = (WrVal & 0xFFFFU);
    pdwVal++;

#if PMA_ACCESS > 1U
    pdwVal++;
#endif /* PMA_ACCESS */

    pBuf++;
    pBuf++;
  }
}

// void USB_ReadPMA(USB_TypeDef const *USBx, uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes)
// {
//   uint32_t n = (uint32_t)wNBytes >> 1;
//   uint32_t BaseAddr = (uint32_t)USBx;
//   uint32_t count;
//   uint32_t RdVal;
//   __IO uint16_t *pdwVal;
//   uint8_t *pBuf = pbUsrBuf;

//   pdwVal = (__IO uint16_t *)(BaseAddr + 0x400U + ((uint32_t)wPMABufAddr * PMA_ACCESS));

//   for (count = n; count != 0U; count--)
//   {
//     RdVal = *(__IO uint16_t *)pdwVal;
//     pdwVal++;
//     *pBuf = (uint8_t)((RdVal >> 0) & 0xFFU);
//     pBuf++;
//     *pBuf = (uint8_t)((RdVal >> 8) & 0xFFU);
//     pBuf++;

// #if PMA_ACCESS > 1U
//     pdwVal++;
// #endif /* PMA_ACCESS */
//   }

//   if ((wNBytes % 2U) != 0U)
//   {
//     RdVal = *pdwVal;
//     *pBuf = (uint8_t)((RdVal >> 0) & 0xFFU);
//   }
// }

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

// USBD_StatusTypeDef  USBD_RunTestMode(USBD_HandleTypeDef  *pdev){
//   /* Prevent unused argument compilation warning */
//   UNUSED(pdev);
//   return USBD_OK;
// }

// USBD_StatusTypeDef USBD_SetClassConfig(USBD_HandleTypeDef  *pdev, uint8_t cfgidx){
//   USBD_StatusTypeDef ret = USBD_FAIL;
//   if (pdev->pClass != NULL){
//     /* Set configuration  and Start the Class*/
//     if (pdev->pClass->Init(pdev, cfgidx) == 0U)
//     {
//       ret = USBD_OK;
//     }
//   }
//   return ret;
// }

// USBD_StatusTypeDef USBD_ClrClassConfig(USBD_HandleTypeDef  *pdev, uint8_t cfgidx){
//   /* Clear configuration  and De-initialize the Class process*/
//   pdev->pClass->DeInit(pdev, cfgidx);
//   return USBD_OK;
// }

// USBD_StatusTypeDef USBD_LL_SetupStage(USBD_HandleTypeDef *pdev, uint8_t *psetup){
//   USBD_ParseSetupRequest(&pdev->request, psetup);
//   pdev->ep0_state = USBD_EP0_SETUP;
//   pdev->ep0_data_len = pdev->request.wLength;
//   switch (pdev->request.bmRequest & 0x1FU){
//     case USB_REQ_RECIPIENT_DEVICE:
//       USBD_StdDevReq(pdev, &pdev->request);
//       break;
//     case USB_REQ_RECIPIENT_INTERFACE:
//       USBD_StdItfReq(pdev, &pdev->request);
//       break;
//     case USB_REQ_RECIPIENT_ENDPOINT:
//       USBD_StdEPReq(pdev, &pdev->request);
//       break;
//     default:
//       USBD_LL_StallEP(pdev, (pdev->request.bmRequest & 0x80U));
//       break;
//   }
//   return USBD_OK;
// }

// USBD_StatusTypeDef USBD_LL_DataOutStage(USBD_HandleTypeDef *pdev, uint8_t epnum, uint8_t *pdata){
//   USBD_EndpointTypeDef *pep;
//   if (epnum == 0U){
//     pep = &pdev->ep_out[0];
//     if (pdev->ep0_state == USBD_EP0_DATA_OUT)
//     {
//       if (pep->rem_length > pep->maxpacket)
//       {
//         pep->rem_length -= pep->maxpacket;
//         USBD_CtlContinueRx(pdev, pdata,
//                            (uint16_t)MIN(pep->rem_length, pep->maxpacket));
//       }
//       else
//       {
//         if ((pdev->pClass->EP0_RxReady != NULL) &&
//             (pdev->dev_state == USBD_STATE_CONFIGURED))
//         {
//           pdev->pClass->EP0_RxReady(pdev);
//         }
//         USBD_CtlSendStatus(pdev);
//       }
//     }
//     else
//     {
//       if (pdev->ep0_state == USBD_EP0_STATUS_OUT)
//       {
//         /*
//          * STATUS PHASE completed, update ep0_state to idle
//          */
//         pdev->ep0_state = USBD_EP0_IDLE;
//         USBD_LL_StallEP(pdev, 0U);
//       }
//     }
//   }
//   else if ((pdev->pClass->DataOut != NULL) &&
//            (pdev->dev_state == USBD_STATE_CONFIGURED)){
//     pdev->pClass->DataOut(pdev, epnum);
//   }
//   else{
//     /* should never be in this condition */
//     return USBD_FAIL;
//   }
//   return USBD_OK;
// }

// USBD_StatusTypeDef USBD_LL_DataInStage(USBD_HandleTypeDef *pdev, uint8_t epnum, uint8_t *pdata){
//   USBD_EndpointTypeDef *pep;
//   if (epnum == 0U){
//     pep = &pdev->ep_in[0];
//     if (pdev->ep0_state == USBD_EP0_DATA_IN)
//     {
//       if (pep->rem_length > pep->maxpacket)
//       {
//         pep->rem_length -= pep->maxpacket;
//         USBD_CtlContinueSendData(pdev, pdata, (uint16_t)pep->rem_length);
//         /* Prepare endpoint for premature end of transfer */
//         USBD_LL_PrepareReceive(pdev, 0U, NULL, 0U);
//       }
//       else
//       {
//         /* last packet is MPS multiple, so send ZLP packet */
//         if ((pep->total_length % pep->maxpacket == 0U) &&
//             (pep->total_length >= pep->maxpacket) &&
//             (pep->total_length < pdev->ep0_data_len))
//         {
//           USBD_CtlContinueSendData(pdev, NULL, 0U);
//           pdev->ep0_data_len = 0U;
//           /* Prepare endpoint for premature end of transfer */
//           USBD_LL_PrepareReceive(pdev, 0U, NULL, 0U);
//         }
//         else
//         {
//           if ((pdev->pClass->EP0_TxSent != NULL) &&
//               (pdev->dev_state == USBD_STATE_CONFIGURED))
//           {
//             pdev->pClass->EP0_TxSent(pdev);
//           }
//           USBD_LL_StallEP(pdev, 0x80U);
//           USBD_CtlReceiveStatus(pdev);
//         }
//       }
//     }
//     else
//     {
//       if ((pdev->ep0_state == USBD_EP0_STATUS_IN) ||
//           (pdev->ep0_state == USBD_EP0_IDLE))
//       {
//         USBD_LL_StallEP(pdev, 0x80U);
//       }
//     }
//     if (pdev->dev_test_mode == 1U)
//     {
//       USBD_RunTestMode(pdev);
//       pdev->dev_test_mode = 0U;
//     }
//   }
//   else if ((pdev->pClass->DataIn != NULL) &&
//            (pdev->dev_state == USBD_STATE_CONFIGURED)){
//     pdev->pClass->DataIn(pdev, epnum);
//   }
//   else{
//     /* should never be in this condition */
//     return USBD_FAIL;
//   }
//   return USBD_OK;
// }

// USBD_StatusTypeDef USBD_LL_Reset(USBD_HandleTypeDef *pdev){
//   /* Open EP0 OUT */
//   USBD_LL_OpenEP(pdev, 0x00U, USBD_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);
//   pdev->ep_out[0x00U & 0xFU].is_used = 1U;
//   pdev->ep_out[0].maxpacket = USB_MAX_EP0_SIZE;
//   /* Open EP0 IN */
//   USBD_LL_OpenEP(pdev, 0x80U, USBD_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);
//   pdev->ep_in[0x80U & 0xFU].is_used = 1U;
//   pdev->ep_in[0].maxpacket = USB_MAX_EP0_SIZE;
//   /* Upon Reset call user call back */
//   pdev->dev_state = USBD_STATE_DEFAULT;
//   pdev->ep0_state = USBD_EP0_IDLE;
//   pdev->dev_config = 0U;
//   pdev->dev_remote_wakeup = 0U;
//   if (pdev->pClassData){
//     pdev->pClass->DeInit(pdev, (uint8_t)pdev->dev_config);
//   }
//   return USBD_OK;
// }

// USBD_StatusTypeDef USBD_LL_SetSpeed(USBD_HandleTypeDef *pdev, USBD_SpeedTypeDef speed){
//   pdev->dev_speed = speed;
//   return USBD_OK;
// }

// USBD_StatusTypeDef USBD_LL_Suspend(USBD_HandleTypeDef *pdev){
//   pdev->dev_old_state =  pdev->dev_state;
//   pdev->dev_state  = USBD_STATE_SUSPENDED;
//   return USBD_OK;
// }

// USBD_StatusTypeDef USBD_LL_Resume(USBD_HandleTypeDef *pdev){
//   if (pdev->dev_state == USBD_STATE_SUSPENDED){
//     pdev->dev_state = pdev->dev_old_state;
//   }
//   return USBD_OK;
// }

// USBD_StatusTypeDef USBD_LL_SOF(USBD_HandleTypeDef *pdev){
//   if (pdev->dev_state == USBD_STATE_CONFIGURED){
//     if (pdev->pClass->SOF != NULL)
//     {
//       pdev->pClass->SOF(pdev);
//     }
//   }
//   return USBD_OK;
// }

PCD_HandleTypeDef hpcd_USB_FS;
static USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status);

void HAL_PCD_MspInit(PCD_HandleTypeDef* pcdHandle){
  if(pcdHandle->Instance==USB)  {
    __HAL_RCC_USB_CLK_ENABLE();
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  }
}

// void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd){
//   USBD_LL_SetupStage((USBD_HandleTypeDef*)hpcd->pData, (uint8_t *)hpcd->Setup);
// }

// void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){
//   USBD_LL_DataOutStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
// }

// void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){
//   USBD_LL_DataInStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
// }

// void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd){
//   USBD_LL_SOF((USBD_HandleTypeDef*)hpcd->pData);
// }

// void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd){
//   USBD_SpeedTypeDef speed = USBD_SPEED_FULL;
//   if ( hpcd->Init.speed != PCD_SPEED_FULL)  {
//     Error_Handler();
//   }
//   USBD_LL_SetSpeed((USBD_HandleTypeDef*)hpcd->pData, speed);
//   USBD_LL_Reset((USBD_HandleTypeDef*)hpcd->pData);
// }

// void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd){
//   USBD_LL_Suspend((USBD_HandleTypeDef*)hpcd->pData);
//   if (hpcd->Init.low_power_enable)  {
//     SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
//   }
// }

// void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd){
//   USBD_LL_Resume((USBD_HandleTypeDef*)hpcd->pData);
// }

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

// USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr){
//   HAL_StatusTypeDef hal_status = HAL_OK;
//   USBD_StatusTypeDef usb_status = USBD_OK;
//   hal_status = HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);
//   usb_status =  USBD_Get_USB_Status(hal_status);
//   return usb_status;
// }

// uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr){
//   PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef*) pdev->pData;
//   if((ep_addr & 0x80) == 0x80)  {
//     return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
//   }
//   else  {
//     return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
//   }
// }

// USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr){
//   HAL_StatusTypeDef hal_status = HAL_OK;
//   USBD_StatusTypeDef usb_status = USBD_OK;
//   hal_status = HAL_PCD_SetAddress(pdev->pData, dev_addr);
//   usb_status =  USBD_Get_USB_Status(hal_status);
//   return usb_status;
// }

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint16_t size){
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
  hal_status = HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
  usb_status =  USBD_Get_USB_Status(hal_status);
  return usb_status;
}

// USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint16_t size){
//   HAL_StatusTypeDef hal_status = HAL_OK;
//   USBD_StatusTypeDef usb_status = USBD_OK;
//   hal_status = HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);
//   usb_status =  USBD_Get_USB_Status(hal_status);
//   return usb_status;
// }

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
// static void USBD_GetDescriptor(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
// static void USBD_SetAddress(USBD_HandleTypeDef *pdev,    USBD_SetupReqTypedef *req);
// static void USBD_SetConfig(USBD_HandleTypeDef *pdev,     USBD_SetupReqTypedef *req);
// static void USBD_GetConfig(USBD_HandleTypeDef *pdev,     USBD_SetupReqTypedef *req);
// static void USBD_GetStatus(USBD_HandleTypeDef *pdev,     USBD_SetupReqTypedef *req);
// static void USBD_SetFeature(USBD_HandleTypeDef *pdev,    USBD_SetupReqTypedef *req);
// static void USBD_ClrFeature(USBD_HandleTypeDef *pdev,    USBD_SetupReqTypedef *req);
static uint8_t USBD_GetLen(uint8_t *buf);
// USBD_StatusTypeDef  USBD_StdDevReq(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req){
//   USBD_StatusTypeDef ret = USBD_OK;
//   switch (req->bmRequest & USB_REQ_TYPE_MASK)
//   {
//     case USB_REQ_TYPE_CLASS:
//     case USB_REQ_TYPE_VENDOR:
//       pdev->pClass->Setup(pdev, req);
//       break;
//     case USB_REQ_TYPE_STANDARD:
//       switch (req->bRequest){
//         case USB_REQ_GET_DESCRIPTOR:
//           USBD_GetDescriptor(pdev, req);
//           break;
//         case USB_REQ_SET_ADDRESS:
//           USBD_SetAddress(pdev, req);
//           break;
//         case USB_REQ_SET_CONFIGURATION:
//           USBD_SetConfig(pdev, req);
//           break;
//         case USB_REQ_GET_CONFIGURATION:
//           USBD_GetConfig(pdev, req);
//           break;
//         case USB_REQ_GET_STATUS:
//           USBD_GetStatus(pdev, req);
//           break;
//         case USB_REQ_SET_FEATURE:
//           USBD_SetFeature(pdev, req);
//           break;
//         case USB_REQ_CLEAR_FEATURE:
//           USBD_ClrFeature(pdev, req);
//           break;
//         default:
//           USBD_CtlError(pdev, req);
//           break;
//       }
//       break;
//     default:
//       USBD_CtlError(pdev, req);
//       break;
//   }
//   return ret;
// }

// USBD_StatusTypeDef  USBD_StdItfReq(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef  *req){
//   USBD_StatusTypeDef ret = USBD_OK;
//   switch (req->bmRequest & USB_REQ_TYPE_MASK)  {
//     case USB_REQ_TYPE_CLASS:
//     case USB_REQ_TYPE_VENDOR:
//     case USB_REQ_TYPE_STANDARD:
//       switch (pdev->dev_state)      {
//         case USBD_STATE_DEFAULT:
//         case USBD_STATE_ADDRESSED:
//         case USBD_STATE_CONFIGURED:
//           if (LOBYTE(req->wIndex) <= USBD_MAX_NUM_INTERFACES)          {
//             ret = (USBD_StatusTypeDef)pdev->pClass->Setup(pdev, req);
//             if ((req->wLength == 0U) && (ret == USBD_OK))
//             {
//               USBD_CtlSendStatus(pdev);
//             }
//           }
//           else          {
//             USBD_CtlError(pdev, req);
//           }
//           break;
//         default:
//           USBD_CtlError(pdev, req);
//           break;
//       }
//       break;
//     default:
//       USBD_CtlError(pdev, req);
//       break;
//   }
//   return USBD_OK;
// }

// USBD_StatusTypeDef  USBD_StdEPReq (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef  *req){
//   USBD_EndpointTypeDef *pep;
//   uint8_t   ep_addr;
//   USBD_StatusTypeDef ret = USBD_OK;
//   ep_addr  = LOBYTE(req->wIndex);
//   switch (req->bmRequest & USB_REQ_TYPE_MASK)  {
//     case USB_REQ_TYPE_CLASS:
//     case USB_REQ_TYPE_VENDOR:
//       pdev->pClass->Setup(pdev, req);
//       break;
//     case USB_REQ_TYPE_STANDARD:
//       /* Check if it is a class request */
//       if ((req->bmRequest & 0x60U) == 0x20U)      {
//         ret = (USBD_StatusTypeDef)pdev->pClass->Setup(pdev, req);
//         return ret;
//       }
//       switch (req->bRequest)      {
//         case USB_REQ_SET_FEATURE:
//           switch (pdev->dev_state)          {
//             case USBD_STATE_ADDRESSED:
//               if ((ep_addr != 0x00U) && (ep_addr != 0x80U))
//               {
//                 USBD_LL_StallEP(pdev, ep_addr);
//                 USBD_LL_StallEP(pdev, 0x80U);
//               }
//               else
//               {
//                 USBD_CtlError(pdev, req);
//               }
//               break;
//             case USBD_STATE_CONFIGURED:
//               if (req->wValue == USB_FEATURE_EP_HALT)
//               {
//                 if ((ep_addr != 0x00U) &&
//                     (ep_addr != 0x80U) && (req->wLength == 0x00U))
//                 {
//                   USBD_LL_StallEP(pdev, ep_addr);
//                 }
//               }
//               USBD_CtlSendStatus(pdev);
//               break;
//             default:
//               USBD_CtlError(pdev, req);
//               break;
//           }
//           break;
//         case USB_REQ_CLEAR_FEATURE:
//           switch (pdev->dev_state)
//           {
//             case USBD_STATE_ADDRESSED:
//               if ((ep_addr != 0x00U) && (ep_addr != 0x80U))
//               {
//                 USBD_LL_StallEP(pdev, ep_addr);
//                 USBD_LL_StallEP(pdev, 0x80U);
//               }
//               else
//               {
//                 USBD_CtlError(pdev, req);
//               }
//               break;
//             case USBD_STATE_CONFIGURED:
//               if (req->wValue == USB_FEATURE_EP_HALT)
//               {
//                 if ((ep_addr & 0x7FU) != 0x00U)
//                 {
//                   USBD_LL_ClearStallEP(pdev, ep_addr);
//                 }
//                 USBD_CtlSendStatus(pdev);
//               }
//               break;
//             default:
//               USBD_CtlError(pdev, req);
//               break;
//           }
//           break;
//         case USB_REQ_GET_STATUS:
//           switch (pdev->dev_state)
//           {
//             case USBD_STATE_ADDRESSED:
//               if ((ep_addr != 0x00U) && (ep_addr != 0x80U))
//               {
//                 USBD_CtlError(pdev, req);
//                 break;
//               }
//               pep = ((ep_addr & 0x80U) == 0x80U) ? &pdev->ep_in[ep_addr & 0x7FU] : \
//                     &pdev->ep_out[ep_addr & 0x7FU];
//               pep->status = 0x0000U;
//               USBD_CtlSendData(pdev, (uint8_t *)(void *)&pep->status, 2U);
//               break;
//             case USBD_STATE_CONFIGURED:
//               if ((ep_addr & 0x80U) == 0x80U)
//               {
//                 if (pdev->ep_in[ep_addr & 0xFU].is_used == 0U)
//                 {
//                   USBD_CtlError(pdev, req);
//                   break;
//                 }
//               }
//               else
//               {
//                 if (pdev->ep_out[ep_addr & 0xFU].is_used == 0U)
//                 {
//                   USBD_CtlError(pdev, req);
//                   break;
//                 }
//               }
//               pep = ((ep_addr & 0x80U) == 0x80U) ? &pdev->ep_in[ep_addr & 0x7FU] : \
//                     &pdev->ep_out[ep_addr & 0x7FU];
//               if ((ep_addr == 0x00U) || (ep_addr == 0x80U))
//               {
//                 pep->status = 0x0000U;
//               }
//               else if (USBD_LL_IsStallEP(pdev, ep_addr))
//               {
//                 pep->status = 0x0001U;
//               }
//               else
//               {
//                 pep->status = 0x0000U;
//               }
//               USBD_CtlSendData(pdev, (uint8_t *)(void *)&pep->status, 2U);
//               break;
//             default:
//               USBD_CtlError(pdev, req);
//               break;
//           }
//           break;
//         default:
//           USBD_CtlError(pdev, req);
//           break;
//       }
//       break;
//     default:
//       USBD_CtlError(pdev, req);
//       break;
//   }
//   return ret;
// }

// static void USBD_GetDescriptor    (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req){
//   uint16_t len = 0U;
//   uint8_t *pbuf = NULL;
//   uint8_t err = 0U;
//   switch (req->wValue >> 8)
//   {
//     case USB_DESC_TYPE_DEVICE:
//       pbuf = pdev->pDesc->GetDeviceDescriptor(pdev->dev_speed, &len);
//       break;
//     case USB_DESC_TYPE_CONFIGURATION:
//       if (pdev->dev_speed == USBD_SPEED_HIGH)
//       {
//         pbuf = pdev->pClass->GetHSConfigDescriptor(&len);
//         pbuf[1] = USB_DESC_TYPE_CONFIGURATION;
//       }
//       else
//       {
//         pbuf = pdev->pClass->GetFSConfigDescriptor(&len);
//         pbuf[1] = USB_DESC_TYPE_CONFIGURATION;
//       }
//       break;
//     case USB_DESC_TYPE_STRING:
//       switch ((uint8_t)(req->wValue))
//       {
//         case USBD_IDX_LANGID_STR:
//           if (pdev->pDesc->GetLangIDStrDescriptor != NULL)
//           {
//             pbuf = pdev->pDesc->GetLangIDStrDescriptor(pdev->dev_speed, &len);
//           }
//           else
//           {
//             USBD_CtlError(pdev, req);
//             err++;
//           }
//           break;
//         case USBD_IDX_MFC_STR:
//           if (pdev->pDesc->GetManufacturerStrDescriptor != NULL)
//           {
//             pbuf = pdev->pDesc->GetManufacturerStrDescriptor(pdev->dev_speed, &len);
//           }
//           else
//           {
//             USBD_CtlError(pdev, req);
//             err++;
//           }
//           break;
//         case USBD_IDX_PRODUCT_STR:
//           if (pdev->pDesc->GetProductStrDescriptor != NULL)
//           {
//             pbuf = pdev->pDesc->GetProductStrDescriptor(pdev->dev_speed, &len);
//           }
//           else
//           {
//             USBD_CtlError(pdev, req);
//             err++;
//           }
//           break;
//         case USBD_IDX_SERIAL_STR:
//           if (pdev->pDesc->GetSerialStrDescriptor != NULL)
//           {
//             pbuf = pdev->pDesc->GetSerialStrDescriptor(pdev->dev_speed, &len);
//           }
//           else
//           {
//             USBD_CtlError(pdev, req);
//             err++;
//           }
//           break;
//         case USBD_IDX_CONFIG_STR:
//           if (pdev->pDesc->GetConfigurationStrDescriptor != NULL)
//           {
//             pbuf = pdev->pDesc->GetConfigurationStrDescriptor(pdev->dev_speed, &len);
//           }
//           else
//           {
//             USBD_CtlError(pdev, req);
//             err++;
//           }
//           break;
//         case USBD_IDX_INTERFACE_STR:
//           if (pdev->pDesc->GetInterfaceStrDescriptor != NULL)
//           {
//             pbuf = pdev->pDesc->GetInterfaceStrDescriptor(pdev->dev_speed, &len);
//           }
//           else
//           {
//             USBD_CtlError(pdev, req);
//             err++;
//           }
//           break;
//         default:
//           USBD_CtlError(pdev, req);
//           err++;
//       }
//       break;
//     case USB_DESC_TYPE_DEVICE_QUALIFIER:
//       if (pdev->dev_speed == USBD_SPEED_HIGH)
//       {
//         pbuf = pdev->pClass->GetDeviceQualifierDescriptor(&len);
//       }
//       else
//       {
//         USBD_CtlError(pdev, req);
//         err++;
//       }
//       break;
//     case USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION:
//       if (pdev->dev_speed == USBD_SPEED_HIGH)
//       {
//         pbuf = pdev->pClass->GetOtherSpeedConfigDescriptor(&len);
//         pbuf[1] = USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION;
//       }
//       else
//       {
//         USBD_CtlError(pdev, req);
//         err++;
//       }
//       break;
//     default:
//       USBD_CtlError(pdev, req);
//       err++;
//       break;
//   }
//   if (err != 0U)
//   {
//     return;
//   }
//   else
//   {
//     if ((len != 0U) && (req->wLength != 0U))
//     {
//       len = MIN(len, req->wLength);
//       (void)USBD_CtlSendData(pdev, pbuf, len);
//     }
//     if (req->wLength == 0U)
//     {
//       (void)USBD_CtlSendStatus(pdev);
//     }
//   }
// }
// static void USBD_SetAddress		  (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req){
//   uint8_t  dev_addr;
//   if ((req->wIndex == 0U) && (req->wLength == 0U) && (req->wValue < 128U))
//   {
//     dev_addr = (uint8_t)(req->wValue) & 0x7FU;
//     if (pdev->dev_state == USBD_STATE_CONFIGURED)
//     {
//       USBD_CtlError(pdev, req);
//     }
//     else
//     {
//       pdev->dev_address = dev_addr;
//       USBD_LL_SetUSBAddress(pdev, dev_addr);
//       USBD_CtlSendStatus(pdev);
//       if (dev_addr != 0U)
//       {
//         pdev->dev_state = USBD_STATE_ADDRESSED;
//       }
//       else
//       {
//         pdev->dev_state = USBD_STATE_DEFAULT;
//       }
//     }
//   }
//   else
//   {
//     USBD_CtlError(pdev, req);
//   }
// }

// static void USBD_SetConfig		  (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req){
//   static uint8_t cfgidx;
//   cfgidx = (uint8_t)(req->wValue);
//   if (cfgidx > USBD_MAX_NUM_CONFIGURATION)
//   {
//     USBD_CtlError(pdev, req);
//   }
//   else
//   {
//     switch (pdev->dev_state)
//     {
//       case USBD_STATE_ADDRESSED:
//         if (cfgidx)
//         {
//           pdev->dev_config = cfgidx;
//           pdev->dev_state = USBD_STATE_CONFIGURED;
//           if (USBD_SetClassConfig(pdev, cfgidx) == USBD_FAIL)
//           {
//             USBD_CtlError(pdev, req);
//             return;
//           }
//           USBD_CtlSendStatus(pdev);
//         }
//         else
//         {
//           USBD_CtlSendStatus(pdev);
//         }
//         break;
//       case USBD_STATE_CONFIGURED:
//         if (cfgidx == 0U)
//         {
//           pdev->dev_state = USBD_STATE_ADDRESSED;
//           pdev->dev_config = cfgidx;
//           USBD_ClrClassConfig(pdev, cfgidx);
//           USBD_CtlSendStatus(pdev);
//         }
//         else if (cfgidx != pdev->dev_config)
//         {
//           /* Clear old configuration */
//           USBD_ClrClassConfig(pdev, (uint8_t)pdev->dev_config);
//           /* set new configuration */
//           pdev->dev_config = cfgidx;
//           if (USBD_SetClassConfig(pdev, cfgidx) == USBD_FAIL)
//           {
//             USBD_CtlError(pdev, req);
//             return;
//           }
//           USBD_CtlSendStatus(pdev);
//         }
//         else
//         {
//           USBD_CtlSendStatus(pdev);
//         }
//         break;
//       default:
//         USBD_CtlError(pdev, req);
//         USBD_ClrClassConfig(pdev, cfgidx);
//         break;
//     }
//   }
// }

// static void USBD_GetConfig		  (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req){
//   if (req->wLength != 1U)
//   {
//     USBD_CtlError(pdev, req);
//   }
//   else
//   {
//     switch (pdev->dev_state)
//     {
//       case USBD_STATE_DEFAULT:
//       case USBD_STATE_ADDRESSED:
//         pdev->dev_default_config = 0U;
//         USBD_CtlSendData(pdev, (uint8_t *)(void *)&pdev->dev_default_config, 1U);
//         break;
//       case USBD_STATE_CONFIGURED:
//         USBD_CtlSendData(pdev, (uint8_t *)(void *)&pdev->dev_config, 1U);
//         break;
//       default:
//         USBD_CtlError(pdev, req);
//         break;
//     }
//   }
// }

// static void USBD_GetStatus		  (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req){
//   switch (pdev->dev_state)
//   {
//     case USBD_STATE_DEFAULT:
//     case USBD_STATE_ADDRESSED:
//     case USBD_STATE_CONFIGURED:
//       if (req->wLength != 0x2U)
//       {
//         USBD_CtlError(pdev, req);
//         break;
//       }
//       pdev->dev_config_status = USB_CONFIG_SELF_POWERED;
//       if (pdev->dev_remote_wakeup)
//       {
//         pdev->dev_config_status |= USB_CONFIG_REMOTE_WAKEUP;
//       }
//       USBD_CtlSendData(pdev, (uint8_t *)(void *)&pdev->dev_config_status, 2U);
//       break;
//     default:
//       USBD_CtlError(pdev, req);
//       break;
//   }
// }

// static void USBD_SetFeature		  (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req){
//   if (req->wValue == USB_FEATURE_REMOTE_WAKEUP)
//   {
//     pdev->dev_remote_wakeup = 1U;
//     USBD_CtlSendStatus(pdev);
//   }
// }

// static void USBD_ClrFeature       (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req){
//   switch (pdev->dev_state)
//   {
//     case USBD_STATE_DEFAULT:
//     case USBD_STATE_ADDRESSED:
//     case USBD_STATE_CONFIGURED:
//       if (req->wValue == USB_FEATURE_REMOTE_WAKEUP)
//       {
//         pdev->dev_remote_wakeup = 0U;
//         USBD_CtlSendStatus(pdev);
//       }
//       break;
//     default:
//       USBD_CtlError(pdev, req);
//       break;
//   }
// }

void USBD_CtlError				  (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req){
  USBD_LL_StallEP(pdev, 0x80U);
  USBD_LL_StallEP(pdev, 0U);
}

// void USBD_ParseSetupRequest		  (USBD_SetupReqTypedef *req, uint8_t *pdata){
//   req->bmRequest = *(uint8_t *)(pdata);
//   req->bRequest = *(uint8_t *)(pdata + 1U);
//   req->wValue = SWAPBYTE(pdata + 2U);
//   req->wIndex = SWAPBYTE(pdata + 4U);
//   req->wLength = SWAPBYTE(pdata + 6U);
// }

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

// USBD_ClassTypeDef  USBD_HID ={
//   USBD_HID_Init,
//   USBD_HID_DeInit,
//   USBD_HID_Setup,
//   NULL, /*EP0_TxSent*/
//   NULL, /*EP0_RxReady*/
//   USBD_HID_DataIn, /*DataIn*/
//   NULL, /*DataOut*/
//   NULL, /*SOF */
//   NULL,
//   NULL,
//   USBD_HID_GetHSCfgDesc,
//   USBD_HID_GetFSCfgDesc,
//   USBD_HID_GetOtherSpeedCfgDesc,
//   USBD_HID_GetDeviceQualifierDesc,
// };
// /* USB HID device FS Configuration Descriptor */
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

// /* USB HID device HS Configuration Descriptor */
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

  if ( (req->bmRequest & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD ){
      switch (req->bRequest)      {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)          {
            USBD_CtlSendData(pdev, (uint8_t *)(void *)&status_info, 2U);
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          } break;
        case USB_REQ_GET_DESCRIPTOR:
          if (req->wValue >> 8 == HID_REPORT_DESC)          {
            len = MIN(HID_MOUSE_REPORT_DESC_SIZE, req->wLength);
            pbuf = HID_MOUSE_ReportDesc;
          } else if (req->wValue >> 8 == HID_DESCRIPTOR_TYPE){
            pbuf = USBD_HID_Desc;
            len = MIN(USB_HID_DESC_SIZ, req->wLength);
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
          }
          USBD_CtlSendData(pdev, pbuf, len);
          break;
        case USB_REQ_GET_INTERFACE :
          if (pdev->dev_state == USBD_STATE_CONFIGURED) {
            USBD_CtlSendData(pdev, (uint8_t *)(void *)&hhid->AltSetting, 1U);
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          } break;
        case USB_REQ_SET_INTERFACE :
          if (pdev->dev_state == USBD_STATE_CONFIGURED) {
            hhid->AltSetting = (uint8_t)(req->wValue);
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          } break;
        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
  }else{
    	  USBD_CtlError(pdev, req);
    	  ret = USBD_FAIL;
      }
  return ret;
}


// uint8_t USBD_HID_SendReport(USBD_HandleTypeDef  *pdev, uint8_t *report, uint16_t len){
//   USBD_HID_HandleTypeDef     *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassData;

//   if (pdev->dev_state == USBD_STATE_CONFIGURED)
//   {
//     if (hhid->state == HID_IDLE)
//     {
//       hhid->state = HID_BUSY;
//       USBD_LL_Transmit(pdev,
//                        HID_EPIN_ADDR,
//                        report,
//                        len);
//     }
//   }
//   return USBD_OK;
// }

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

// USBD_StatusTypeDef USBD_CtlContinueSendData(USBD_HandleTypeDef *pdev, uint8_t *pbuf, uint16_t len){
//   USBD_LL_Transmit(pdev, 0x00U, pbuf, len);
//   return USBD_OK;
// }

// USBD_StatusTypeDef USBD_CtlContinueRx(USBD_HandleTypeDef *pdev, uint8_t *pbuf, uint16_t len){
//   USBD_LL_PrepareReceive(pdev, 0U, pbuf, len);
//   return USBD_OK;
// }

// USBD_StatusTypeDef USBD_CtlSendStatus(USBD_HandleTypeDef *pdev){
//   pdev->ep0_state = USBD_EP0_STATUS_IN;
//   USBD_LL_Transmit(pdev, 0x00U, NULL, 0U);
//   return USBD_OK;
// }

// USBD_StatusTypeDef USBD_CtlReceiveStatus(USBD_HandleTypeDef *pdev){
//   pdev->ep0_state = USBD_EP0_STATUS_OUT;
//   USBD_LL_PrepareReceive(pdev, 0U, NULL, 0U);
//   return USBD_OK;
// }



// #endif  __USBD_CORE