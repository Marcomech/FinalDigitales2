

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"



#ifdef HAL_FLASH_MODULE_ENABLED



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/



/* Private macro ---------------------------- ---------------------------------*/

 


/* Private variables ---------------------------------------------------------*/

/* Variables used for Erase pages under interruption*/
FLASH_ProcessTypeDef pFlash;


/* Private function prototypes -----------------------------------------------*/

static  void   FLASH_Program_HalfWord(uint32_t Address, uint16_t Data);
static  void   FLASH_SetErrorCode(void);
extern void    FLASH_PageErase(uint32_t PageAddress);


/* Exported functions ---------------------------------------------------------*/

  



HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  uint8_t index = 0;
  uint8_t nbiterations = 0;
  
  /* Process Locked */
  __HAL_LOCK(&pFlash);

  /* Check the parameters */
  assert_param(IS_FLASH_TYPEPROGRAM(TypeProgram));
  assert_param(IS_FLASH_PROGRAM_ADDRESS(Address));

#if defined(FLASH_BANK2_END)
  if(Address <= FLASH_BANK1_END)
  {
#endif /* FLASH_BANK2_END */
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
#if defined(FLASH_BANK2_END)
  }
  else
  {
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperationBank2(FLASH_TIMEOUT_VALUE);
  }
#endif /* FLASH_BANK2_END */
  
  if(status == HAL_OK)
  {
    if(TypeProgram == FLASH_TYPEPROGRAM_HALFWORD)
    {
      /* Program halfword (16-bit) at a specified address. */
      nbiterations = 1U;
    }
    else if(TypeProgram == FLASH_TYPEPROGRAM_WORD)
    {
      /* Program word (32-bit = 2*16-bit) at a specified address. */
      nbiterations = 2U;
    }
    else
    {
      /* Program double word (64-bit = 4*16-bit) at a specified address. */
      nbiterations = 4U;
    }

    for (index = 0U; index < nbiterations; index++)
    {
      FLASH_Program_HalfWord((Address + (2U*index)), (uint16_t)(Data >> (16U*index)));

#if defined(FLASH_BANK2_END)
      if(Address <= FLASH_BANK1_END)
      {
#endif /* FLASH_BANK2_END */
        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
    
        /* If the program operation is completed, disable the PG Bit */
        CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
#if defined(FLASH_BANK2_END)
      }
      else
      {
        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastOperationBank2(FLASH_TIMEOUT_VALUE);
        
        /* If the program operation is completed, disable the PG Bit */
        CLEAR_BIT(FLASH->CR2, FLASH_CR2_PG);
      }
#endif /* FLASH_BANK2_END */
      /* In case of error, stop programation procedure */
      if (status != HAL_OK)
      {
        break;
      }
    }
  }

  /* Process Unlocked */
  __HAL_UNLOCK(&pFlash);

  return status;
}


HAL_StatusTypeDef HAL_FLASH_Program_IT(uint32_t TypeProgram, uint32_t Address, uint64_t Data)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  /* Process Locked */
  __HAL_LOCK(&pFlash);

  /* Check the parameters */
  assert_param(IS_FLASH_TYPEPROGRAM(TypeProgram));
  assert_param(IS_FLASH_PROGRAM_ADDRESS(Address));

#if defined(FLASH_BANK2_END)
  /* If procedure already ongoing, reject the next one */
  if (pFlash.ProcedureOnGoing != FLASH_PROC_NONE)
  {
    return HAL_ERROR;
  }
  
  if(Address <= FLASH_BANK1_END)
  {
    /* Enable End of FLASH Operation and Error source interrupts */
    __HAL_FLASH_ENABLE_IT(FLASH_IT_EOP_BANK1 | FLASH_IT_ERR_BANK1);

  }else
  {
    /* Enable End of FLASH Operation and Error source interrupts */
    __HAL_FLASH_ENABLE_IT(FLASH_IT_EOP_BANK2 | FLASH_IT_ERR_BANK2);
  }
#else
  /* Enable End of FLASH Operation and Error source interrupts */
  __HAL_FLASH_ENABLE_IT(FLASH_IT_EOP | FLASH_IT_ERR);
#endif /* FLASH_BANK2_END */
  
  pFlash.Address = Address;
  pFlash.Data = Data;

  if(TypeProgram == FLASH_TYPEPROGRAM_HALFWORD)
  {
    pFlash.ProcedureOnGoing = FLASH_PROC_PROGRAMHALFWORD;
    /* Program halfword (16-bit) at a specified address. */
    pFlash.DataRemaining = 1U;
  }
  else if(TypeProgram == FLASH_TYPEPROGRAM_WORD)
  {
    pFlash.ProcedureOnGoing = FLASH_PROC_PROGRAMWORD;
    /* Program word (32-bit : 2*16-bit) at a specified address. */
    pFlash.DataRemaining = 2U;
  }
  else
  {
    pFlash.ProcedureOnGoing = FLASH_PROC_PROGRAMDOUBLEWORD;
    /* Program double word (64-bit : 4*16-bit) at a specified address. */
    pFlash.DataRemaining = 4U;
  }

  /* Program halfword (16-bit) at a specified address. */
  FLASH_Program_HalfWord(Address, (uint16_t)Data);

  return status;
}


void HAL_FLASH_IRQHandler(void)
{
  uint32_t addresstmp = 0U;
  
  /* Check FLASH operation error flags */
#if defined(FLASH_BANK2_END)
  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR_BANK1) || __HAL_FLASH_GET_FLAG(FLASH_FLAG_PGERR_BANK1) || \
    (__HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR_BANK2) || __HAL_FLASH_GET_FLAG(FLASH_FLAG_PGERR_BANK2)))
#else
  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR) ||__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGERR))
#endif /* FLASH_BANK2_END */
  {
    /* Return the faulty address */
    addresstmp = pFlash.Address;
    /* Reset address */
    pFlash.Address = 0xFFFFFFFFU;
  
    /* Save the Error code */
    FLASH_SetErrorCode();
    
    /* FLASH error interrupt user callback */
    HAL_FLASH_OperationErrorCallback(addresstmp);

    /* Stop the procedure ongoing */
    pFlash.ProcedureOnGoing = FLASH_PROC_NONE;
  }

  /* Check FLASH End of Operation flag  */
#if defined(FLASH_BANK2_END)
  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_EOP_BANK1))
  {
    /* Clear FLASH End of Operation pending bit */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP_BANK1);
#else
  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_EOP))
  {
    /* Clear FLASH End of Operation pending bit */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
#endif /* FLASH_BANK2_END */
    
    /* Process can continue only if no error detected */
    if(pFlash.ProcedureOnGoing != FLASH_PROC_NONE)
    {
      if(pFlash.ProcedureOnGoing == FLASH_PROC_PAGEERASE)
      {
        /* Nb of pages to erased can be decreased */
        pFlash.DataRemaining--;

        /* Check if there are still pages to erase */
        if(pFlash.DataRemaining != 0U)
        {
          addresstmp = pFlash.Address;
          /*Indicate user which sector has been erased */
          HAL_FLASH_EndOfOperationCallback(addresstmp);

          /*Increment sector number*/
          addresstmp = pFlash.Address + FLASH_PAGE_SIZE;
          pFlash.Address = addresstmp;

          /* If the erase operation is completed, disable the PER Bit */
          CLEAR_BIT(FLASH->CR, FLASH_CR_PER);

          FLASH_PageErase(addresstmp);
        }
        else
        {
          /* No more pages to Erase, user callback can be called. */
          /* Reset Sector and stop Erase pages procedure */
          pFlash.Address = addresstmp = 0xFFFFFFFFU;
          pFlash.ProcedureOnGoing = FLASH_PROC_NONE;
          /* FLASH EOP interrupt user callback */
          HAL_FLASH_EndOfOperationCallback(addresstmp);
        }
      }
      else if(pFlash.ProcedureOnGoing == FLASH_PROC_MASSERASE)
      {
        /* Operation is completed, disable the MER Bit */
        CLEAR_BIT(FLASH->CR, FLASH_CR_MER);

#if defined(FLASH_BANK2_END)
        /* Stop Mass Erase procedure if no pending mass erase on other bank */
        if (HAL_IS_BIT_CLR(FLASH->CR2, FLASH_CR2_MER))
        {
#endif /* FLASH_BANK2_END */
          /* MassErase ended. Return the selected bank */
          /* FLASH EOP interrupt user callback */
          HAL_FLASH_EndOfOperationCallback(0U);

          /* Stop Mass Erase procedure*/
          pFlash.ProcedureOnGoing = FLASH_PROC_NONE;
        }
#if defined(FLASH_BANK2_END)
      }
#endif /* FLASH_BANK2_END */
      else
      {
        /* Nb of 16-bit data to program can be decreased */
        pFlash.DataRemaining--;
        
        /* Check if there are still 16-bit data to program */
        if(pFlash.DataRemaining != 0U)
        {
          /* Increment address to 16-bit */
          pFlash.Address += 2U;
          addresstmp = pFlash.Address;
          
          /* Shift to have next 16-bit data */
          pFlash.Data = (pFlash.Data >> 16U);
          
          /* Operation is completed, disable the PG Bit */
          CLEAR_BIT(FLASH->CR, FLASH_CR_PG);

          /*Program halfword (16-bit) at a specified address.*/
          FLASH_Program_HalfWord(addresstmp, (uint16_t)pFlash.Data);
        }
        else
        {
          /* Program ended. Return the selected address */
          /* FLASH EOP interrupt user callback */
          if (pFlash.ProcedureOnGoing == FLASH_PROC_PROGRAMHALFWORD)
          {
            HAL_FLASH_EndOfOperationCallback(pFlash.Address);
          }
          else if (pFlash.ProcedureOnGoing == FLASH_PROC_PROGRAMWORD)
          {
            HAL_FLASH_EndOfOperationCallback(pFlash.Address - 2U);
          }
          else 
          {
            HAL_FLASH_EndOfOperationCallback(pFlash.Address - 6U);
          }
        
          /* Reset Address and stop Program procedure */
          pFlash.Address = 0xFFFFFFFFU;
          pFlash.ProcedureOnGoing = FLASH_PROC_NONE;
        }
      }
    }
  }
  
#if defined(FLASH_BANK2_END)
  /* Check FLASH End of Operation flag  */
  if(__HAL_FLASH_GET_FLAG( FLASH_FLAG_EOP_BANK2))
  {
    /* Clear FLASH End of Operation pending bit */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP_BANK2);
    
    /* Process can continue only if no error detected */
    if(pFlash.ProcedureOnGoing != FLASH_PROC_NONE)
    {
      if(pFlash.ProcedureOnGoing == FLASH_PROC_PAGEERASE)
      {
        /* Nb of pages to erased can be decreased */
        pFlash.DataRemaining--;
        
        /* Check if there are still pages to erase*/
        if(pFlash.DataRemaining != 0U)
        {
          /* Indicate user which page address has been erased*/
          HAL_FLASH_EndOfOperationCallback(pFlash.Address);
        
          /* Increment page address to next page */
          pFlash.Address += FLASH_PAGE_SIZE;
          addresstmp = pFlash.Address;

          /* Operation is completed, disable the PER Bit */
          CLEAR_BIT(FLASH->CR2, FLASH_CR2_PER);

          FLASH_PageErase(addresstmp);
        }
        else
        {
          /*No more pages to Erase*/
          
          /*Reset Address and stop Erase pages procedure*/
          pFlash.Address = 0xFFFFFFFFU;
          pFlash.ProcedureOnGoing = FLASH_PROC_NONE;

          /* FLASH EOP interrupt user callback */
          HAL_FLASH_EndOfOperationCallback(pFlash.Address);
        }
      }
      else if(pFlash.ProcedureOnGoing == FLASH_PROC_MASSERASE)
      {
        /* Operation is completed, disable the MER Bit */
        CLEAR_BIT(FLASH->CR2, FLASH_CR2_MER);

        if (HAL_IS_BIT_CLR(FLASH->CR, FLASH_CR_MER))
        {
          /* MassErase ended. Return the selected bank*/
          /* FLASH EOP interrupt user callback */
          HAL_FLASH_EndOfOperationCallback(0U);
        
          pFlash.ProcedureOnGoing = FLASH_PROC_NONE;
        }
      }
      else
      {
        /* Nb of 16-bit data to program can be decreased */
        pFlash.DataRemaining--;
        
        /* Check if there are still 16-bit data to program */
        if(pFlash.DataRemaining != 0U)
        {
          /* Increment address to 16-bit */
          pFlash.Address += 2U;
          addresstmp = pFlash.Address;
          
          /* Shift to have next 16-bit data */
          pFlash.Data = (pFlash.Data >> 16U);
          
          /* Operation is completed, disable the PG Bit */
          CLEAR_BIT(FLASH->CR2, FLASH_CR2_PG);

          /*Program halfword (16-bit) at a specified address.*/
          FLASH_Program_HalfWord(addresstmp, (uint16_t)pFlash.Data);
        }
        else
        {
          /*Program ended. Return the selected address*/
          /* FLASH EOP interrupt user callback */
          if (pFlash.ProcedureOnGoing == FLASH_PROC_PROGRAMHALFWORD)
          {
            HAL_FLASH_EndOfOperationCallback(pFlash.Address);
          }
          else if (pFlash.ProcedureOnGoing == FLASH_PROC_PROGRAMWORD)
          {
            HAL_FLASH_EndOfOperationCallback(pFlash.Address-2U);
          }
          else 
          {
            HAL_FLASH_EndOfOperationCallback(pFlash.Address-6U);
          }
          
          /* Reset Address and stop Program procedure*/
          pFlash.Address = 0xFFFFFFFFU;
          pFlash.ProcedureOnGoing = FLASH_PROC_NONE;
        }
      }
    }
  }
#endif 

  if(pFlash.ProcedureOnGoing == FLASH_PROC_NONE)
  {
#if defined(FLASH_BANK2_END)
    /* Operation is completed, disable the PG, PER and MER Bits for both bank */
    CLEAR_BIT(FLASH->CR, (FLASH_CR_PG | FLASH_CR_PER | FLASH_CR_MER));
    CLEAR_BIT(FLASH->CR2, (FLASH_CR2_PG | FLASH_CR2_PER | FLASH_CR2_MER));  
  
    /* Disable End of FLASH Operation and Error source interrupts for both banks */
    __HAL_FLASH_DISABLE_IT(FLASH_IT_EOP_BANK1 | FLASH_IT_ERR_BANK1 | FLASH_IT_EOP_BANK2 | FLASH_IT_ERR_BANK2);
#else
    /* Operation is completed, disable the PG, PER and MER Bits */
    CLEAR_BIT(FLASH->CR, (FLASH_CR_PG | FLASH_CR_PER | FLASH_CR_MER));

    /* Disable End of FLASH Operation and Error source interrupts */
    __HAL_FLASH_DISABLE_IT(FLASH_IT_EOP | FLASH_IT_ERR);
#endif /* FLASH_BANK2_END */

    /* Process Unlocked */
    __HAL_UNLOCK(&pFlash);
  }
}


__weak void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(ReturnValue);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_FLASH_EndOfOperationCallback could be implemented in the user file
   */ 
}


__weak void HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(ReturnValue);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_FLASH_OperationErrorCallback could be implemented in the user file
   */ 
}






HAL_StatusTypeDef HAL_FLASH_Unlock(void)
{
  HAL_StatusTypeDef status = HAL_OK;

  if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET)
  {
    /* Authorize the FLASH Registers access */
    WRITE_REG(FLASH->KEYR, FLASH_KEY1);
    WRITE_REG(FLASH->KEYR, FLASH_KEY2);

    /* Verify Flash is unlocked */
    if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET)
    {
      status = HAL_ERROR;
    }
  }
#if defined(FLASH_BANK2_END)
  if(READ_BIT(FLASH->CR2, FLASH_CR2_LOCK) != RESET)
  {
    /* Authorize the FLASH BANK2 Registers access */
    WRITE_REG(FLASH->KEYR2, FLASH_KEY1);
    WRITE_REG(FLASH->KEYR2, FLASH_KEY2);
    
    /* Verify Flash BANK2 is unlocked */
    if(READ_BIT(FLASH->CR2, FLASH_CR2_LOCK) != RESET)
    {
      status = HAL_ERROR;
    }
  }
#endif /* FLASH_BANK2_END */

  return status;
}


HAL_StatusTypeDef HAL_FLASH_Lock(void)
{
  /* Set the LOCK Bit to lock the FLASH Registers access */
  SET_BIT(FLASH->CR, FLASH_CR_LOCK);
  
#if defined(FLASH_BANK2_END)
  /* Set the LOCK Bit to lock the FLASH BANK2 Registers access */
  SET_BIT(FLASH->CR2, FLASH_CR2_LOCK);

#endif /* FLASH_BANK2_END */
  return HAL_OK;  
}


HAL_StatusTypeDef HAL_FLASH_OB_Unlock(void)
{
  if (HAL_IS_BIT_CLR(FLASH->CR, FLASH_CR_OPTWRE))
  {
    /* Authorizes the Option Byte register programming */
    WRITE_REG(FLASH->OPTKEYR, FLASH_OPTKEY1);
    WRITE_REG(FLASH->OPTKEYR, FLASH_OPTKEY2);
  }
  else
  {
    return HAL_ERROR;
  }  
  
  return HAL_OK;  
}


HAL_StatusTypeDef HAL_FLASH_OB_Lock(void)
{
  /* Clear the OPTWRE Bit to lock the FLASH Option Byte Registers access */
  CLEAR_BIT(FLASH->CR, FLASH_CR_OPTWRE);
  
  return HAL_OK;  
}
  

void HAL_FLASH_OB_Launch(void)
{
  /* Initiates a system reset request to launch the option byte loading */
  HAL_NVIC_SystemReset();
}

  




uint32_t HAL_FLASH_GetError(void)
{
   return pFlash.ErrorCode;
}








static void FLASH_Program_HalfWord(uint32_t Address, uint16_t Data)
{
  /* Clean the error context */
  pFlash.ErrorCode = HAL_FLASH_ERROR_NONE;
  
#if defined(FLASH_BANK2_END)
  if(Address <= FLASH_BANK1_END)
  {
#endif /* FLASH_BANK2_END */
    /* Proceed to program the new data */
    SET_BIT(FLASH->CR, FLASH_CR_PG);
#if defined(FLASH_BANK2_END)
  }
  else
  {
    /* Proceed to program the new data */
    SET_BIT(FLASH->CR2, FLASH_CR2_PG);
  }
#endif /* FLASH_BANK2_END */

  /* Write data in the address */
  *(__IO uint16_t*)Address = Data;
}


HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout)
{
  /* Wait for the FLASH operation to complete by polling on BUSY flag to be reset.
     Even if the FLASH operation fails, the BUSY flag will be reset and an error
     flag will be set */
     
  uint32_t tickstart = HAL_GetTick();
     
  while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) 
  { 
    if (Timeout != HAL_MAX_DELAY)
    {
      if((Timeout == 0U) || ((HAL_GetTick()-tickstart) > Timeout))
      {
        return HAL_TIMEOUT;
      }
    }
  }
  
  /* Check FLASH End of Operation flag  */
  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_EOP))
  {
    /* Clear FLASH End of Operation pending bit */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
  }
  
  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR)  || 
     __HAL_FLASH_GET_FLAG(FLASH_FLAG_OPTVERR) || 
     __HAL_FLASH_GET_FLAG(FLASH_FLAG_PGERR))
  {
    /*Save the error code*/
    FLASH_SetErrorCode();
    return HAL_ERROR;
  }

  /* There is no error flag set */
  return HAL_OK;
}

#if defined(FLASH_BANK2_END)

HAL_StatusTypeDef FLASH_WaitForLastOperationBank2(uint32_t Timeout)
{ 
  /* Wait for the FLASH BANK2 operation to complete by polling on BUSY flag to be reset.
     Even if the FLASH BANK2 operation fails, the BUSY flag will be reset and an error
     flag will be set */
     
  uint32_t tickstart = HAL_GetTick();
     
  while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY_BANK2)) 
  { 
    if (Timeout != HAL_MAX_DELAY)
    {
      if((Timeout == 0U) || ((HAL_GetTick()-tickstart) > Timeout))
      {
        return HAL_TIMEOUT;
      }
    }
  }
  
  /* Check FLASH End of Operation flag  */
  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_EOP_BANK2))
  {
    /* Clear FLASH End of Operation pending bit */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP_BANK2);
  }

  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR_BANK2) || __HAL_FLASH_GET_FLAG(FLASH_FLAG_PGERR_BANK2))
  {
    /*Save the error code*/
    FLASH_SetErrorCode();
    return HAL_ERROR;
  }

  /* If there is an error flag set */
  return HAL_OK;
  
}
#endif /* FLASH_BANK2_END */


static void FLASH_SetErrorCode(void)
{
  uint32_t flags = 0U;
  
#if defined(FLASH_BANK2_END)
  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR) || __HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR_BANK2))
#else
  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR))
#endif /* FLASH_BANK2_END */
  {
    pFlash.ErrorCode |= HAL_FLASH_ERROR_WRP;
#if defined(FLASH_BANK2_END)
    flags |= FLASH_FLAG_WRPERR | FLASH_FLAG_WRPERR_BANK2;
#else
    flags |= FLASH_FLAG_WRPERR;
#endif /* FLASH_BANK2_END */
  }
#if defined(FLASH_BANK2_END)
  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGERR) || __HAL_FLASH_GET_FLAG(FLASH_FLAG_PGERR_BANK2))
#else
  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGERR))
#endif /* FLASH_BANK2_END */
  {
    pFlash.ErrorCode |= HAL_FLASH_ERROR_PROG;
#if defined(FLASH_BANK2_END)
    flags |= FLASH_FLAG_PGERR | FLASH_FLAG_PGERR_BANK2;
#else
    flags |= FLASH_FLAG_PGERR;
#endif /* FLASH_BANK2_END */
  }
  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_OPTVERR))
  {
    pFlash.ErrorCode |= HAL_FLASH_ERROR_OPTV;
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
  }

  /* Clear FLASH error pending bits */
  __HAL_FLASH_CLEAR_FLAG(flags);
}  




#endif /* HAL_FLASH_MODULE_ENABLED */



