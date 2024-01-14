#include "stm32f1xx_hal.h"


__weak uint32_t HAL_GetTick(void){
  return uwTick;
}

void HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup){
  assert_param(IS_NVIC_PRIORITY_GROUP(PriorityGroup));
  NVIC_SetPriorityGrouping(PriorityGroup);
}

void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority){
  uint32_t prioritygroup = 0x00U;
  assert_param(IS_NVIC_SUB_PRIORITY(SubPriority));
  assert_param(IS_NVIC_PREEMPTION_PRIORITY(PreemptPriority));

  prioritygroup = NVIC_GetPriorityGrouping();

  NVIC_SetPriority(IRQn, NVIC_EncodePriority(prioritygroup, PreemptPriority, SubPriority));
}

void HAL_NVIC_EnableIRQ(IRQn_Type IRQn){
  assert_param(IS_NVIC_DEVICE_IRQ(IRQn));
  NVIC_EnableIRQ(IRQn);
}

uint32_t HAL_SYSTICK_Config(uint32_t TicksNumb){
   return SysTick_Config(TicksNumb);
}

static void RCC_Delay(uint32_t mdelay);

HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct){
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
    if ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_HSE)
        || ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK) && (__HAL_RCC_GET_PLL_OSCSOURCE() == RCC_PLLSOURCE_HSE)))
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
    if ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_HSI)
        || ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK) && (__HAL_RCC_GET_PLL_OSCSOURCE() == RCC_PLLSOURCE_HSI_DIV2)))
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
    FlagStatus       pwrclkchanged = RESET;

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

#if defined(RCC_CR_PLL2ON)
  /*-------------------------------- PLL2 Configuration -----------------------*/
  /* Check the parameters */
  assert_param(IS_RCC_PLL2(RCC_OscInitStruct->PLL2.PLL2State));
  if ((RCC_OscInitStruct->PLL2.PLL2State) != RCC_PLL2_NONE)
  {
    /* This bit can not be cleared if the PLL2 clock is used indirectly as system
      clock (i.e. it is used as PLL clock entry that is used as system clock). */
    if ((__HAL_RCC_GET_PLL_OSCSOURCE() == RCC_PLLSOURCE_HSE) && \
        (__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK) && \
        ((READ_BIT(RCC->CFGR2, RCC_CFGR2_PREDIV1SRC)) == RCC_CFGR2_PREDIV1SRC_PLL2))
    {
      return HAL_ERROR;
    }
    else
    {
      if ((RCC_OscInitStruct->PLL2.PLL2State) == RCC_PLL2_ON)
      {
        /* Check the parameters */
        assert_param(IS_RCC_PLL2_MUL(RCC_OscInitStruct->PLL2.PLL2MUL));
        assert_param(IS_RCC_HSE_PREDIV2(RCC_OscInitStruct->PLL2.HSEPrediv2Value));

        /* Prediv2 can be written only when the PLLI2S is disabled. */
        /* Return an error only if new value is different from the programmed value */
        if (HAL_IS_BIT_SET(RCC->CR, RCC_CR_PLL3ON) && \
            (__HAL_RCC_HSE_GET_PREDIV2() != RCC_OscInitStruct->PLL2.HSEPrediv2Value))
        {
          return HAL_ERROR;
        }

        /* Disable the main PLL2. */
        __HAL_RCC_PLL2_DISABLE();

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till PLL2 is disabled */
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLL2RDY) != RESET)
        {
          if ((HAL_GetTick() - tickstart) > PLL2_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }

        /* Configure the HSE prediv2 factor --------------------------------*/
        __HAL_RCC_HSE_PREDIV2_CONFIG(RCC_OscInitStruct->PLL2.HSEPrediv2Value);

        /* Configure the main PLL2 multiplication factors. */
        __HAL_RCC_PLL2_CONFIG(RCC_OscInitStruct->PLL2.PLL2MUL);

        /* Enable the main PLL2. */
        __HAL_RCC_PLL2_ENABLE();

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till PLL2 is ready */
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLL2RDY)  == RESET)
        {
          if ((HAL_GetTick() - tickstart) > PLL2_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
      else
      {
        /* Set PREDIV1 source to HSE */
        CLEAR_BIT(RCC->CFGR2, RCC_CFGR2_PREDIV1SRC);

        /* Disable the main PLL2. */
        __HAL_RCC_PLL2_DISABLE();

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till PLL2 is disabled */
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLL2RDY)  != RESET)
        {
          if ((HAL_GetTick() - tickstart) > PLL2_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
  }

#endif /* RCC_CR_PLL2ON */
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
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)  != RESET)
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
#if defined(RCC_CFGR2_PREDIV1SRC)
          assert_param(IS_RCC_PREDIV1_SOURCE(RCC_OscInitStruct->Prediv1Source));

          /* Set PREDIV1 source */
          SET_BIT(RCC->CFGR2, RCC_OscInitStruct->Prediv1Source);
#endif /* RCC_CFGR2_PREDIV1SRC */

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
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)  == RESET)
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
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)  != RESET)
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

HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct, uint32_t FLatency){
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

HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit){
  uint32_t tickstart = 0U, temp_reg = 0U;
#if defined(STM32F105xC) || defined(STM32F107xC)
  uint32_t  pllactive = 0U;
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
      if (HAL_IS_BIT_SET(RCC->CR, RCC_CR_PLL2ON) && \
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
      while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLI2SRDY)  == RESET)
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

#if defined(STM32F102x6) || defined(STM32F102xB) || defined(STM32F103x6)\
 || defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG)\
 || defined(STM32F105xC) || defined(STM32F107xC)
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

uint32_t HAL_RCC_GetSysClockFreq(void){

  static const uint8_t aPLLMULFactorTable[16U] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 16};
  static const uint8_t aPredivFactorTable[2U] = {1, 2};

  uint32_t tmpreg = 0U, prediv = 0U, pllclk = 0U, pllmul = 0U;
  uint32_t sysclockfreq = 0U;

  tmpreg = RCC->CFGR;

  /* Get SYSCLK source -------------------------------------------------------*/
  switch (tmpreg & RCC_CFGR_SWS)
  {
    case RCC_SYSCLKSOURCE_STATUS_HSE:  /* HSE used as system clock */
    {
      sysclockfreq = HSE_VALUE;
      break;
    }
    case RCC_SYSCLKSOURCE_STATUS_PLLCLK:  /* PLL used as system clock */
    {
      pllmul = aPLLMULFactorTable[(uint32_t)(tmpreg & RCC_CFGR_PLLMULL) >> RCC_CFGR_PLLMULL_Pos];
      if ((tmpreg & RCC_CFGR_PLLSRC) != RCC_PLLSOURCE_HSI_DIV2)
      {
#if defined(RCC_CFGR2_PREDIV1)
        prediv = aPredivFactorTable[(uint32_t)(RCC->CFGR2 & RCC_CFGR2_PREDIV1) >> RCC_CFGR2_PREDIV1_Pos];
#else
        prediv = aPredivFactorTable[(uint32_t)(RCC->CFGR & RCC_CFGR_PLLXTPRE) >> RCC_CFGR_PLLXTPRE_Pos];
#endif /*RCC_CFGR2_PREDIV1*/
#if defined(RCC_CFGR2_PREDIV1SRC)

        if (HAL_IS_BIT_SET(RCC->CFGR2, RCC_CFGR2_PREDIV1SRC))
        {
          /* PLL2 selected as Prediv1 source */
          /* PLLCLK = PLL2CLK / PREDIV1 * PLLMUL with PLL2CLK = HSE/PREDIV2 * PLL2MUL */
          prediv2 = ((RCC->CFGR2 & RCC_CFGR2_PREDIV2) >> RCC_CFGR2_PREDIV2_Pos) + 1;
          pll2mul = ((RCC->CFGR2 & RCC_CFGR2_PLL2MUL) >> RCC_CFGR2_PLL2MUL_Pos) + 2;
          pllclk = (uint32_t)(((uint64_t)HSE_VALUE * (uint64_t)pll2mul * (uint64_t)pllmul) / ((uint64_t)prediv2 * (uint64_t)prediv));
        }
        else
        {
          /* HSE used as PLL clock source : PLLCLK = HSE/PREDIV1 * PLLMUL */
          pllclk = (uint32_t)((HSE_VALUE * pllmul) / prediv);
        }

        /* If PLLMUL was set to 13 means that it was to cover the case PLLMUL 6.5 (avoid using float) */
        /* In this case need to divide pllclk by 2 */
        if (pllmul == aPLLMULFactorTable[(uint32_t)(RCC_CFGR_PLLMULL6_5) >> RCC_CFGR_PLLMULL_Pos])
        {
          pllclk = pllclk / 2;
        }
#else
        /* HSE used as PLL clock source : PLLCLK = HSE/PREDIV1 * PLLMUL */
        pllclk = (uint32_t)((HSE_VALUE  * pllmul) / prediv);
#endif /*RCC_CFGR2_PREDIV1SRC*/
      }
      else
      {
        /* HSI used as PLL clock source : PLLCLK = HSI/2 * PLLMUL */
        pllclk = (uint32_t)((HSI_VALUE >> 1) * pllmul);
      }
      sysclockfreq = pllclk;
      break;
    }
    case RCC_SYSCLKSOURCE_STATUS_HSI:  /* HSI used as system clock source */
    default: /* HSI used as system clock */
    {
      sysclockfreq = HSI_VALUE;
      break;
    }
  }
  return sysclockfreq;
}

static void RCC_Delay(uint32_t mdelay){
  __IO uint32_t Delay = mdelay * (SystemCoreClock / 8U / 1000U);
  do  {
    __NOP();
  }
  while (Delay --);
}

__weak void HAL_RCC_CSSCallback(void){}

#ifdef HAL_PCD_MODULE_ENABLED

#define PCD_MIN(a, b)  (((a) < (b)) ? (a) : (b))
#define PCD_MAX(a, b)  (((a) > (b)) ? (a) : (b))

static HAL_StatusTypeDef PCD_EP_ISR_Handler(PCD_HandleTypeDef *hpcd);
static HAL_StatusTypeDef HAL_PCD_EP_DB_Transmit(PCD_HandleTypeDef *hpcd, PCD_EPTypeDef *ep, uint16_t wEPVal);
static uint16_t HAL_PCD_EP_DB_Receive(PCD_HandleTypeDef *hpcd, PCD_EPTypeDef *ep, uint16_t wEPVal);


HAL_StatusTypeDef HAL_PCD_Init(PCD_HandleTypeDef *hpcd){
  uint8_t i;

  if (hpcd == NULL)  {
    return HAL_ERROR;
  }
  assert_param(IS_PCD_ALL_INSTANCE(hpcd->Instance));
  if (hpcd->State == HAL_PCD_STATE_RESET)  {
    /* Allocate lock resource and initialize it */
    hpcd->Lock = HAL_UNLOCKED;
    HAL_PCD_MspInit(hpcd);
  }

  hpcd->State = HAL_PCD_STATE_BUSY;
  __HAL_PCD_DISABLE(hpcd);

  if (USB_CoreInit(hpcd->Instance, hpcd->Init) != HAL_OK)  {
    hpcd->State = HAL_PCD_STATE_ERROR;
    return HAL_ERROR;
  }

  (void)USB_SetCurrentMode(hpcd->Instance, USB_DEVICE_MODE);

  for (i = 0U; i < hpcd->Init.dev_endpoints; i++)  {
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

HAL_StatusTypeDef HAL_PCD_Start(PCD_HandleTypeDef *hpcd){
  __HAL_LOCK(hpcd);
  __HAL_PCD_ENABLE(hpcd);

  (void)USB_DevConnect(hpcd->Instance);
  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

void HAL_PCD_IRQHandler(PCD_HandleTypeDef *hpcd)
{
  uint32_t wIstr = USB_ReadInterrupts(hpcd->Instance);
  uint16_t store_ep[8];
  uint8_t i;

  if ((wIstr & USB_ISTR_CTR) == USB_ISTR_CTR)
  {
    /* servicing of the endpoint correct transfer interrupt */
    /* clear of the CTR flag into the sub */
    (void)PCD_EP_ISR_Handler(hpcd);

    return;
  }

  if ((wIstr & USB_ISTR_RESET) == USB_ISTR_RESET)
  {
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_RESET);


    HAL_PCD_ResetCallback(hpcd);

    (void)HAL_PCD_SetAddress(hpcd, 0U);

    return;
  }

  if ((wIstr & USB_ISTR_PMAOVR) == USB_ISTR_PMAOVR)
  {
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_PMAOVR);

    return;
  }

  if ((wIstr & USB_ISTR_ERR) == USB_ISTR_ERR)
  {
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_ERR);

    return;
  }

  if ((wIstr & USB_ISTR_WKUP) == USB_ISTR_WKUP)
  {
    hpcd->Instance->CNTR &= (uint16_t) ~(USB_CNTR_LP_MODE);
    hpcd->Instance->CNTR &= (uint16_t) ~(USB_CNTR_FSUSP);


    HAL_PCD_ResumeCallback(hpcd);

    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_WKUP);

    return;
  }

  if ((wIstr & USB_ISTR_SUSP) == USB_ISTR_SUSP)
  {
    /* WA: To Clear Wakeup flag if raised with suspend signal */

    /* Store Endpoint registers */
    for (i = 0U; i < 8U; i++)
    {
      store_ep[i] = PCD_GET_ENDPOINT(hpcd->Instance, i);
    }

    /* FORCE RESET */
    hpcd->Instance->CNTR |= (uint16_t)(USB_CNTR_FRES);

    /* CLEAR RESET */
    hpcd->Instance->CNTR &= (uint16_t)(~USB_CNTR_FRES);

    /* wait for reset flag in ISTR */
    while ((hpcd->Instance->ISTR & USB_ISTR_RESET) == 0U)
    {
    }

    /* Clear Reset Flag */
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_RESET);

    /* Restore Registre */
    for (i = 0U; i < 8U; i++)
    {
      PCD_SET_ENDPOINT(hpcd->Instance, i, store_ep[i]);
    }

    /* Force low-power mode in the macrocell */
    hpcd->Instance->CNTR |= (uint16_t)USB_CNTR_FSUSP;

    /* clear of the ISTR bit must be done after setting of CNTR_FSUSP */
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_SUSP);

    hpcd->Instance->CNTR |= (uint16_t)USB_CNTR_LP_MODE;

    HAL_PCD_SuspendCallback(hpcd);

    return;
  }

  if ((wIstr & USB_ISTR_SOF) == USB_ISTR_SOF)
  {
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_SOF);

    HAL_PCD_SOFCallback(hpcd);

    return;
  }

  if ((wIstr & USB_ISTR_ESOF) == USB_ISTR_ESOF)
  {
    /* clear ESOF flag in ISTR */
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_ESOF);

    return;
  }
}

HAL_StatusTypeDef HAL_PCD_SetAddress(PCD_HandleTypeDef *hpcd, uint8_t address)
{
  __HAL_LOCK(hpcd);
  hpcd->USB_Address = address;
  (void)USB_SetDevAddress(hpcd->Instance, address);
  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

HAL_StatusTypeDef HAL_PCD_EP_Open(PCD_HandleTypeDef *hpcd, uint8_t ep_addr,
                                  uint16_t ep_mps, uint8_t ep_type)
{
  HAL_StatusTypeDef  ret = HAL_OK;
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

HAL_StatusTypeDef HAL_PCD_EP_Receive(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len)
{
  PCD_EPTypeDef *ep;

  ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];

  /*setup and start the Xfer */
  ep->xfer_buff = pBuf;
  ep->xfer_len = len;
  ep->xfer_count = 0U;
  ep->is_in = 0U;
  ep->num = ep_addr & EP_ADDR_MSK;

  (void)USB_EPStartXfer(hpcd->Instance, ep);

  return HAL_OK;
}

HAL_StatusTypeDef HAL_PCD_EP_Transmit(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len)
{
  PCD_EPTypeDef *ep;

  ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];

  /*setup and start the Xfer */
  ep->xfer_buff = pBuf;
  ep->xfer_len = len;
#if defined (USB)
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

HAL_StatusTypeDef HAL_PCD_EP_ClrStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  PCD_EPTypeDef *ep;

  if (((uint32_t)ep_addr & 0x0FU) > hpcd->Init.dev_endpoints)
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
    ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 0U;
  }

  ep->is_stall = 0U;
  ep->num = ep_addr & EP_ADDR_MSK;

  __HAL_LOCK(hpcd);
  (void)USB_EPClearStall(hpcd->Instance, ep);
  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

static HAL_StatusTypeDef PCD_EP_ISR_Handler(PCD_HandleTypeDef *hpcd)
{
  PCD_EPTypeDef *ep;
  uint16_t count;
  uint16_t wIstr;
  uint16_t wEPVal;
  uint16_t TxPctSize;
  uint8_t epindex;

#if (USE_USB_DOUBLE_BUFFER != 1U)
  count = 0U;
#endif /* USE_USB_DOUBLE_BUFFER */

  /* stay in loop while pending interrupts */
  while ((hpcd->Instance->ISTR & USB_ISTR_CTR) != 0U)
  {
    wIstr = hpcd->Instance->ISTR;

    /* extract highest priority endpoint number */
    epindex = (uint8_t)(wIstr & USB_ISTR_EP_ID);

    if (epindex == 0U)
    {
      /* Decode and service control endpoint interrupt */

      /* DIR bit = origin of the interrupt */
      if ((wIstr & USB_ISTR_DIR) == 0U)
      {
        /* DIR = 0 */

        /* DIR = 0 => IN  int */
        /* DIR = 0 implies that (EP_CTR_TX = 1) always */
        PCD_CLEAR_TX_EP_CTR(hpcd->Instance, PCD_ENDP0);
        ep = &hpcd->IN_ep[0];

        ep->xfer_count = PCD_GET_EP_TX_CNT(hpcd->Instance, ep->num);
        ep->xfer_buff += ep->xfer_count;

        HAL_PCD_DataInStageCallback(hpcd, 0U);

        if ((hpcd->USB_Address > 0U) && (ep->xfer_len == 0U))
        {
          hpcd->Instance->DADDR = ((uint16_t)hpcd->USB_Address | USB_DADDR_EF);
          hpcd->USB_Address = 0U;
        }
      }
      else
      {
        /* DIR = 1 */

        /* DIR = 1 & CTR_RX => SETUP or OUT int */
        /* DIR = 1 & (CTR_TX | CTR_RX) => 2 int pending */
        ep = &hpcd->OUT_ep[0];
        wEPVal = PCD_GET_ENDPOINT(hpcd->Instance, PCD_ENDP0);

        if ((wEPVal & USB_EP_SETUP) != 0U)
        {
          /* Get SETUP Packet */
          ep->xfer_count = PCD_GET_EP_RX_CNT(hpcd->Instance, ep->num);

          USB_ReadPMA(hpcd->Instance, (uint8_t *)hpcd->Setup,
                      ep->pmaadress, (uint16_t)ep->xfer_count);

          /* SETUP bit kept frozen while CTR_RX = 1 */
          PCD_CLEAR_RX_EP_CTR(hpcd->Instance, PCD_ENDP0);


          HAL_PCD_SetupStageCallback(hpcd);
        }
        else if ((wEPVal & USB_EP_CTR_RX) != 0U)
        {
          PCD_CLEAR_RX_EP_CTR(hpcd->Instance, PCD_ENDP0);

          /* Get Control Data OUT Packet */
          ep->xfer_count = PCD_GET_EP_RX_CNT(hpcd->Instance, ep->num);

          if ((ep->xfer_count != 0U) && (ep->xfer_buff != 0U))
          {
            USB_ReadPMA(hpcd->Instance, ep->xfer_buff,
                        ep->pmaadress, (uint16_t)ep->xfer_count);

            ep->xfer_buff += ep->xfer_count;

            HAL_PCD_DataOutStageCallback(hpcd, 0U);
          }

          wEPVal = (uint16_t)PCD_GET_ENDPOINT(hpcd->Instance, PCD_ENDP0);

          if (((wEPVal & USB_EP_SETUP) == 0U) && ((wEPVal & USB_EP_RX_STRX) != USB_EP_RX_VALID))
          {
            PCD_SET_EP_RX_CNT(hpcd->Instance, PCD_ENDP0, ep->maxpacket);
            PCD_SET_EP_RX_STATUS(hpcd->Instance, PCD_ENDP0, USB_EP_RX_VALID);
          }
        }
      }
    }
    else
    {
      /* Decode and service non control endpoints interrupt */
      /* process related endpoint register */
      wEPVal = PCD_GET_ENDPOINT(hpcd->Instance, epindex);

      if ((wEPVal & USB_EP_CTR_RX) != 0U)
      {
        /* clear int flag */
        PCD_CLEAR_RX_EP_CTR(hpcd->Instance, epindex);
        ep = &hpcd->OUT_ep[epindex];

        /* OUT Single Buffering */
        if (ep->doublebuffer == 0U)
        {
          count = (uint16_t)PCD_GET_EP_RX_CNT(hpcd->Instance, ep->num);

          if (count != 0U)
          {
            USB_ReadPMA(hpcd->Instance, ep->xfer_buff, ep->pmaadress, count);
          }
        }
#if (USE_USB_DOUBLE_BUFFER == 1U)
        else
        {
          /* manage double buffer bulk out */
          if (ep->type == EP_TYPE_BULK)
          {
            count = HAL_PCD_EP_DB_Receive(hpcd, ep, wEPVal);
          }
          else /* manage double buffer iso out */
          {
            /* free EP OUT Buffer */
            PCD_FREE_USER_BUFFER(hpcd->Instance, ep->num, 0U);

            if ((PCD_GET_ENDPOINT(hpcd->Instance, ep->num) & USB_EP_DTOG_RX) != 0U)
            {
              /* read from endpoint BUF0Addr buffer */
              count = (uint16_t)PCD_GET_EP_DBUF0_CNT(hpcd->Instance, ep->num);

              if (count != 0U)
              {
                USB_ReadPMA(hpcd->Instance, ep->xfer_buff, ep->pmaaddr0, count);
              }
            }
            else
            {
              /* read from endpoint BUF1Addr buffer */
              count = (uint16_t)PCD_GET_EP_DBUF1_CNT(hpcd->Instance, ep->num);

              if (count != 0U)
              {
                USB_ReadPMA(hpcd->Instance, ep->xfer_buff, ep->pmaaddr1, count);
              }
            }
          }
        }
#endif /* (USE_USB_DOUBLE_BUFFER == 1U) */

        /* multi-packet on the NON control OUT endpoint */
        ep->xfer_count += count;
        ep->xfer_buff += count;

        if ((ep->xfer_len == 0U) || (count < ep->maxpacket))
        {
          /* RX COMPLETE */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
          hpcd->DataOutStageCallback(hpcd, ep->num);
#else
          HAL_PCD_DataOutStageCallback(hpcd, ep->num);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
        }
        else
        {
          (void)USB_EPStartXfer(hpcd->Instance, ep);
        }
      }

      if ((wEPVal & USB_EP_CTR_TX) != 0U)
      {
        ep = &hpcd->IN_ep[epindex];

        /* clear int flag */
        PCD_CLEAR_TX_EP_CTR(hpcd->Instance, epindex);

        if (ep->type == EP_TYPE_ISOC)
        {
          ep->xfer_len = 0U;

#if (USE_USB_DOUBLE_BUFFER == 1U)
          if (ep->doublebuffer != 0U)
          {
            if ((wEPVal & USB_EP_DTOG_TX) != 0U)
            {
              PCD_SET_EP_DBUF0_CNT(hpcd->Instance, ep->num, ep->is_in, 0U);
            }
            else
            {
              PCD_SET_EP_DBUF1_CNT(hpcd->Instance, ep->num, ep->is_in, 0U);
            }
          }
#endif /* (USE_USB_DOUBLE_BUFFER == 1U) */

          /* TX COMPLETE */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
          hpcd->DataInStageCallback(hpcd, ep->num);
#else
          HAL_PCD_DataInStageCallback(hpcd, ep->num);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
        }
        else
        {
          /* Manage Single Buffer Transaction */
          if ((wEPVal & USB_EP_KIND) == 0U)
          {
            /* multi-packet on the NON control IN endpoint */
            TxPctSize = (uint16_t)PCD_GET_EP_TX_CNT(hpcd->Instance, ep->num);

            if (ep->xfer_len > TxPctSize)
            {
              ep->xfer_len -= TxPctSize;
            }
            else
            {
              ep->xfer_len = 0U;
            }

            /* Zero Length Packet? */
            if (ep->xfer_len == 0U)
            {
              /* TX COMPLETE */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
              hpcd->DataInStageCallback(hpcd, ep->num);
#else
              HAL_PCD_DataInStageCallback(hpcd, ep->num);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
            }
            else
            {
              /* Transfer is not yet Done */
              ep->xfer_buff += TxPctSize;
              ep->xfer_count += TxPctSize;
              (void)USB_EPStartXfer(hpcd->Instance, ep);
            }
          }
#if (USE_USB_DOUBLE_BUFFER == 1U)
          /* Double Buffer bulk IN (bulk transfer Len > Ep_Mps) */
          else
          {
            (void)HAL_PCD_EP_DB_Transmit(hpcd, ep, wEPVal);
          }
#endif /* (USE_USB_DOUBLE_BUFFER == 1U) */
        }
      }
    }
  }

  return HAL_OK;
}

static uint16_t HAL_PCD_EP_DB_Receive(PCD_HandleTypeDef *hpcd,
                                      PCD_EPTypeDef *ep, uint16_t wEPVal)
{
  uint16_t count;

  /* Manage Buffer0 OUT */
  if ((wEPVal & USB_EP_DTOG_RX) != 0U)
  {
    /* Get count of received Data on buffer0 */
    count = (uint16_t)PCD_GET_EP_DBUF0_CNT(hpcd->Instance, ep->num);

    if (ep->xfer_len >= count)
    {
      ep->xfer_len -= count;
    }
    else
    {
      ep->xfer_len = 0U;
    }

    if (ep->xfer_len == 0U)
    {
      /* set NAK to OUT endpoint since double buffer is enabled */
      PCD_SET_EP_RX_STATUS(hpcd->Instance, ep->num, USB_EP_RX_NAK);
    }

    /* Check if Buffer1 is in blocked state which requires to toggle */
    if ((wEPVal & USB_EP_DTOG_TX) != 0U)
    {
      PCD_FREE_USER_BUFFER(hpcd->Instance, ep->num, 0U);
    }

    if (count != 0U)
    {
      USB_ReadPMA(hpcd->Instance, ep->xfer_buff, ep->pmaaddr0, count);
    }
  }
  /* Manage Buffer 1 DTOG_RX=0 */
  else
  {
    /* Get count of received data */
    count = (uint16_t)PCD_GET_EP_DBUF1_CNT(hpcd->Instance, ep->num);

    if (ep->xfer_len >= count)
    {
      ep->xfer_len -= count;
    }
    else
    {
      ep->xfer_len = 0U;
    }

    if (ep->xfer_len == 0U)
    {
      /* set NAK on the current endpoint */
      PCD_SET_EP_RX_STATUS(hpcd->Instance, ep->num, USB_EP_RX_NAK);
    }

    /*Need to FreeUser Buffer*/
    if ((wEPVal & USB_EP_DTOG_TX) == 0U)
    {
      PCD_FREE_USER_BUFFER(hpcd->Instance, ep->num, 0U);
    }

    if (count != 0U)
    {
      USB_ReadPMA(hpcd->Instance, ep->xfer_buff, ep->pmaaddr1, count);
    }
  }

  return count;
}

static HAL_StatusTypeDef HAL_PCD_EP_DB_Transmit(PCD_HandleTypeDef *hpcd, PCD_EPTypeDef *ep, uint16_t wEPVal)
{
  uint32_t len;
  uint16_t TxPctSize;

  /* Data Buffer0 ACK received */
  if ((wEPVal & USB_EP_DTOG_TX) != 0U)
  {
    /* multi-packet on the NON control IN endpoint */
    TxPctSize = (uint16_t)PCD_GET_EP_DBUF0_CNT(hpcd->Instance, ep->num);

    if (ep->xfer_len > TxPctSize)
    {
      ep->xfer_len -= TxPctSize;
    }
    else
    {
      ep->xfer_len = 0U;
    }

    /* Transfer is completed */
    if (ep->xfer_len == 0U)
    {
      PCD_SET_EP_DBUF0_CNT(hpcd->Instance, ep->num, ep->is_in, 0U);
      PCD_SET_EP_DBUF1_CNT(hpcd->Instance, ep->num, ep->is_in, 0U);

      /* TX COMPLETE */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->DataInStageCallback(hpcd, ep->num);
#else
      HAL_PCD_DataInStageCallback(hpcd, ep->num);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

      if ((wEPVal & USB_EP_DTOG_RX) != 0U)
      {
        PCD_FREE_USER_BUFFER(hpcd->Instance, ep->num, 1U);
      }
    }
    else /* Transfer is not yet Done */
    {
      /* need to Free USB Buff */
      if ((wEPVal & USB_EP_DTOG_RX) != 0U)
      {
        PCD_FREE_USER_BUFFER(hpcd->Instance, ep->num, 1U);
      }

      /* Still there is data to Fill in the next Buffer */
      if (ep->xfer_fill_db == 1U)
      {
        ep->xfer_buff += TxPctSize;
        ep->xfer_count += TxPctSize;

        /* Calculate the len of the new buffer to fill */
        if (ep->xfer_len_db >= ep->maxpacket)
        {
          len = ep->maxpacket;
          ep->xfer_len_db -= len;
        }
        else if (ep->xfer_len_db == 0U)
        {
          len = TxPctSize;
          ep->xfer_fill_db = 0U;
        }
        else
        {
          ep->xfer_fill_db = 0U;
          len = ep->xfer_len_db;
          ep->xfer_len_db = 0U;
        }

        /* Write remaining Data to Buffer */
        /* Set the Double buffer counter for pma buffer1 */
        PCD_SET_EP_DBUF0_CNT(hpcd->Instance, ep->num, ep->is_in, len);

        /* Copy user buffer to USB PMA */
        USB_WritePMA(hpcd->Instance, ep->xfer_buff,  ep->pmaaddr0, (uint16_t)len);
      }
    }
  }
  else /* Data Buffer1 ACK received */
  {
    /* multi-packet on the NON control IN endpoint */
    TxPctSize = (uint16_t)PCD_GET_EP_DBUF1_CNT(hpcd->Instance, ep->num);

    if (ep->xfer_len >= TxPctSize)
    {
      ep->xfer_len -= TxPctSize;
    }
    else
    {
      ep->xfer_len = 0U;
    }

    /* Transfer is completed */
    if (ep->xfer_len == 0U)
    {
      PCD_SET_EP_DBUF0_CNT(hpcd->Instance, ep->num, ep->is_in, 0U);
      PCD_SET_EP_DBUF1_CNT(hpcd->Instance, ep->num, ep->is_in, 0U);

      /* TX COMPLETE */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->DataInStageCallback(hpcd, ep->num);
#else
      HAL_PCD_DataInStageCallback(hpcd, ep->num);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

      /* need to Free USB Buff */
      if ((wEPVal & USB_EP_DTOG_RX) == 0U)
      {
        PCD_FREE_USER_BUFFER(hpcd->Instance, ep->num, 1U);
      }
    }
    else /* Transfer is not yet Done */
    {
      /* need to Free USB Buff */
      if ((wEPVal & USB_EP_DTOG_RX) == 0U)
      {
        PCD_FREE_USER_BUFFER(hpcd->Instance, ep->num, 1U);
      }

      /* Still there is data to Fill in the next Buffer */
      if (ep->xfer_fill_db == 1U)
      {
        ep->xfer_buff += TxPctSize;
        ep->xfer_count += TxPctSize;

        /* Calculate the len of the new buffer to fill */
        if (ep->xfer_len_db >= ep->maxpacket)
        {
          len = ep->maxpacket;
          ep->xfer_len_db -= len;
        }
        else if (ep->xfer_len_db == 0U)
        {
          len = TxPctSize;
          ep->xfer_fill_db = 0U;
        }
        else
        {
          len = ep->xfer_len_db;
          ep->xfer_len_db = 0U;
          ep->xfer_fill_db = 0;
        }

        /* Set the Double buffer counter for pmabuffer1 */
        PCD_SET_EP_DBUF1_CNT(hpcd->Instance, ep->num, ep->is_in, len);

        /* Copy the user buffer to USB PMA */
        USB_WritePMA(hpcd->Instance, ep->xfer_buff,  ep->pmaaddr1, (uint16_t)len);
      }
    }
  }

  /*enable endpoint IN*/
  PCD_SET_EP_TX_STATUS(hpcd->Instance, ep->num, USB_EP_TX_VALID);

  return HAL_OK;
}

#endif /* HAL_PCD_MODULE_ENABLED */


#ifdef HAL_MODULE_ENABLED

#define IDCODE_DEVID_MASK    0x00000FFFU
__IO uint32_t uwTick;

uint32_t uwTickPrio   = (1UL << __NVIC_PRIO_BITS); /* Invalid PRIO */

HAL_TickFreqTypeDef uwTickFreq = HAL_TICK_FREQ_DEFAULT;  /* 1KHz */

HAL_StatusTypeDef HAL_Init(void){

  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();

  /* Set Interrupt Group Priority */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* Use systick as time base source and configure 1ms tick (default clock after Reset is HSI) */
  HAL_InitTick(TICK_INT_PRIORITY);

  /* Return function status */
  return HAL_OK;
}

__weak HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  /* Configure the SysTick to have interrupt in 1ms time basis*/
  if (HAL_SYSTICK_Config(SystemCoreClock / (1000U / uwTickFreq)) > 0U)
  {
    return HAL_ERROR;
  }

  /* Configure the SysTick IRQ priority */
  if (TickPriority < (1UL << __NVIC_PRIO_BITS))
  {
    HAL_NVIC_SetPriority(SysTick_IRQn, TickPriority, 0U);
    uwTickPrio = TickPriority;
  }
  else
  {
    return HAL_ERROR;
  }

  /* Return function status */
  return HAL_OK;
}

__weak void HAL_IncTick(void)
{
  uwTick += uwTickFreq;
}



__weak void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  /* Add a freq to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }

  while ((HAL_GetTick() - tickstart) < wait)
  {
  }
}

#endif /* HAL_MODULE_ENABLED */

