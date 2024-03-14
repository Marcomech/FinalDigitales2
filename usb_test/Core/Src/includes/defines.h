typedef __INT8_TYPE__ int8_t;
typedef __INT16_TYPE__ int16_t;
typedef __INT32_TYPE__ int32_t;
typedef __INT64_TYPE__ int64_t;
typedef __UINT8_TYPE__ uint8_t;
typedef __UINT16_TYPE__ uint16_t;
typedef __UINT32_TYPE__ uint32_t;
typedef __UINT64_TYPE__ uint64_t;

#define HAL_MODULE_ENABLED
#define HAL_PCD_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define FLASH_LATENCY_0            0x00000000U               /*!< FLASH Zero Latency cycle */
#define FLASH_LATENCY_1            0x00000001                /*!< FLASH One Latency cycle */
#define LSI_VALUE 40000U          /*!< LSI Typical Value in Hz */
#define LSE_VALUE 32768U          /*!< Value of the External oscillator in Hz*/
#define VDD_VALUE 3300U      /*!< Value of VDD in mv */
#define TICK_INT_PRIORITY 0U /*!< tick interrupt priority (lowest by default)  */
#define USE_RTOS 0U
#define PREFETCH_ENABLE 1U

#define MAC_ADDR0 2U
#define MAC_ADDR1 0U
#define MAC_ADDR2 0U
#define MAC_ADDR3 0U
#define MAC_ADDR4 0U
#define MAC_ADDR5 0U
#define ETH_RX_BUF_SIZE ETH_MAX_PACKET_SIZE /* buffer size for receive               */
#define ETH_TX_BUF_SIZE ETH_MAX_PACKET_SIZE /* buffer size for transmit              */
#define ETH_RXBUFNB 8U                      /* 4 Rx buffers of size ETH_RX_BUF_SIZE  */
#define ETH_TXBUFNB 4U                      /* 4 Tx buffers of size ETH_TX_BUF_SIZE  */
#define DP83848_PHY_ADDRESS 0x01U
#define PHY_RESET_DELAY 0x000000FFU
#define PHY_CONFIG_DELAY 0x00000FFFU
#define PHY_READ_TO 0x0000FFFFU
#define PHY_WRITE_TO 0x0000FFFFU
#define PHY_BCR ((uint16_t)0x00) /*!< Transceiver Basic Control Register   */
#define PHY_BSR ((uint16_t)0x01) /*!< Transceiver Basic Status Register    */
#define PHY_RESET ((uint16_t)0x8000)                   /*!< PHY Reset */
#define PHY_LOOPBACK ((uint16_t)0x4000)                /*!< Select loop-back mode */
#define PHY_FULLDUPLEX_100M ((uint16_t)0x2100)         /*!< Set the full-duplex mode at 100 Mb/s */
#define PHY_HALFDUPLEX_100M ((uint16_t)0x2000)         /*!< Set the half-duplex mode at 100 Mb/s */
#define PHY_FULLDUPLEX_10M ((uint16_t)0x0100)          /*!< Set the full-duplex mode at 10 Mb/s  */
#define PHY_HALFDUPLEX_10M ((uint16_t)0x0000)          /*!< Set the half-duplex mode at 10 Mb/s  */
#define PHY_AUTONEGOTIATION ((uint16_t)0x1000)         /*!< Enable auto-negotiation function     */
#define PHY_RESTART_AUTONEGOTIATION ((uint16_t)0x0200) /*!< Restart auto-negotiation function    */
#define PHY_POWERDOWN ((uint16_t)0x0800)               /*!< Select the power down mode           */
#define PHY_ISOLATE ((uint16_t)0x0400)                 /*!< Isolate PHY from MII                 */
#define PHY_AUTONEGO_COMPLETE ((uint16_t)0x0020) /*!< Auto-Negotiation process completed   */
#define PHY_LINKED_STATUS ((uint16_t)0x0004)     /*!< Valid link established               */
#define PHY_JABBER_DETECTION ((uint16_t)0x0002)  /*!< Jabber condition detected            */
#define PHY_SR ((uint16_t)0x10U) /*!< PHY status register Offset                      */
#define PHY_SPEED_STATUS ((uint16_t)0x0002U)  /*!< PHY Speed mask                                  */
#define PHY_DUPLEX_STATUS ((uint16_t)0x0004U) /*!< PHY Duplex mask                                 */
#define USE_SPI_CRC 0U
#define HAL_MAX_DELAY 0xFFFFFFFFU
#define __HAL_LOCK(__HANDLE__)              do                                        {                                           if ((__HANDLE__)->Lock == HAL_LOCKED)     {                                           return HAL_BUSY;                        }                                         else                                      {                                           (__HANDLE__)->Lock = HAL_LOCKED;        }                                       } while (0U)
#define __HAL_UNLOCK(__HANDLE__)         do                                     {                                        (__HANDLE__)->Lock = HAL_UNLOCKED;   } while (0U)
#define __packed __attribute__((__packed__))
#define __HAL_RCC_USB_CLK_ENABLE()                                                                 do                                                                                               {                                                                                                  volatile uint32_t tmpreg;                                                                            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USBEN); /* Delay after an RCC peripheral clock enabling */     tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_USBEN);                                              UNUSED(tmpreg);                                                                                } while (0U)
#define NVIC_PRIORITYGROUP_0 0x00000007U
#define NVIC_PRIORITYGROUP_1 0x00000006U
#define NVIC_PRIORITYGROUP_2 0x00000005U
#define NVIC_PRIORITYGROUP_3 0x00000004U
#define NVIC_PRIORITYGROUP_4 0x00000003U
#define __HAL_FLASH_SET_LATENCY(__LATENCY__) (FLASH->ACR = (FLASH->ACR & (~FLASH_ACR_LATENCY)) | (__LATENCY__))
#define __HAL_FLASH_GET_LATENCY() (READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY))
#define __HAL_FLASH_PREFETCH_BUFFER_ENABLE() (FLASH->ACR |= FLASH_ACR_PRFTBE)
#define EP_MPS_64 0U
#define EP_MPS_32 1U
#define EP_MPS_16 2U
#define EP_MPS_8 3U
#define EP_TYPE_MSK 3U
#define EP_SPEED_LOW 0U
#define EP_SPEED_FULL 1U
#define EP_SPEED_HIGH 2U
#define HC_PID_DATA0 0U
#define HC_PID_DATA2 1U
#define HC_PID_DATA1 2U
#define HC_PID_SETUP 3U
#define USBH_FSLS_SPEED 1U
#define USB_EP_RX_STRX (0x3U << 12)
#define USE_USB_DOUBLE_BUFFER 1U
#define __HAL_PCD_ENABLE(__HANDLE__) (void)USB_EnableGlobalInt((__HANDLE__)->Instance)
#define __HAL_PCD_DISABLE(__HANDLE__) (void)USB_DisableGlobalInt((__HANDLE__)->Instance)
#define __HAL_PCD_CLEAR_FLAG(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->ISTR) &= (uint16_t)(~(__INTERRUPT__)))
#define PCD_ENDP0 0U
#define USB_CNTRX_BLSIZE (0x1U << 15)
#define PCD_SET_ENDPOINT(USBx, bEpNum, wRegValue) (*(volatile uint16_t *)(&(USBx)->EP0R + ((bEpNum) * 2U)) = (uint16_t)(wRegValue))
#define PCD_GET_ENDPOINT(USBx, bEpNum) (*(volatile uint16_t *)(&(USBx)->EP0R + ((bEpNum) * 2U)))
#define PCD_FREE_USER_BUFFER(USBx, bEpNum, bDir)   do                                               {                                                  if ((bDir) == 0U)                                { /* OUT double buffered endpoint */               PCD_TX_DTOG((USBx), (bEpNum));                 }                                                else if ((bDir) == 1U)                           { /* IN double buffered endpoint */                PCD_RX_DTOG((USBx), (bEpNum));                 }                                              } while (0)
#define PCD_SET_EP_TX_STATUS(USBx, bEpNum, wState)                                                do                                                                                              {                                                                                                 uint16_t _wRegVal;                                                                              _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPTX_DTOGMASK; /* toggle first bit ? */     if ((USB_EPTX_DTOG1 & (wState)) != 0U)                                                          {                                                                                                 _wRegVal ^= USB_EPTX_DTOG1;                                                                   } /* toggle second bit ?  */                                                                    if ((USB_EPTX_DTOG2 & (wState)) != 0U)                                                          {                                                                                                 _wRegVal ^= USB_EPTX_DTOG2;                                                                   }                                                                                               PCD_SET_ENDPOINT((USBx), (bEpNum), (_wRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX));               } while (0) /* PCD_SET_EP_TX_STATUS */
#define PCD_SET_EP_RX_STATUS(USBx, bEpNum, wState)                                                do                                                                                              {                                                                                                 uint16_t _wRegVal;                                                                              _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPRX_DTOGMASK; /* toggle first bit ? */     if ((USB_EPRX_DTOG1 & (wState)) != 0U)                                                          {                                                                                                 _wRegVal ^= USB_EPRX_DTOG1;                                                                   } /* toggle second bit ? */                                                                     if ((USB_EPRX_DTOG2 & (wState)) != 0U)                                                          {                                                                                                 _wRegVal ^= USB_EPRX_DTOG2;                                                                   }                                                                                               PCD_SET_ENDPOINT((USBx), (bEpNum), (_wRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX));               } while (0) /* PCD_SET_EP_RX_STATUS */
#define PCD_SET_EP_KIND(USBx, bEpNum)                                                               do                                                                                                {                                                                                                   uint16_t _wRegVal;                                                                                _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPREG_MASK;                                   PCD_SET_ENDPOINT((USBx), (bEpNum), (_wRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_KIND));   } while (0) /* PCD_SET_EP_KIND */
#define PCD_CLEAR_EP_KIND(USBx, bEpNum)                                               do                                                                                  {                                                                                     uint16_t _wRegVal;                                                                  _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPKIND_MASK;                    PCD_SET_ENDPOINT((USBx), (bEpNum), (_wRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX));   } while (0) /* PCD_CLEAR_EP_KIND */
#define PCD_SET_BULK_EP_DBUF(USBx, bEpNum) PCD_SET_EP_KIND((USBx), (bEpNum))
#define PCD_CLEAR_BULK_EP_DBUF(USBx, bEpNum) PCD_CLEAR_EP_KIND((USBx), (bEpNum))
#define PCD_CLEAR_RX_EP_CTR(USBx, bEpNum)                                         do                                                                              {                                                                                 uint16_t _wRegVal;                                                              _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & (0x7FFFU & USB_EPREG_MASK);     PCD_SET_ENDPOINT((USBx), (bEpNum), (_wRegVal | USB_EP_CTR_TX));               } while (0) /* PCD_CLEAR_RX_EP_CTR */
#define PCD_CLEAR_TX_EP_CTR(USBx, bEpNum)                                         do                                                                              {                                                                                 uint16_t _wRegVal;                                                              _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & (0xFF7FU & USB_EPREG_MASK);     PCD_SET_ENDPOINT((USBx), (bEpNum), (_wRegVal | USB_EP_CTR_RX));               } while (0) /* PCD_CLEAR_TX_EP_CTR */
#define PCD_RX_DTOG(USBx, bEpNum)                                                                     do                                                                                                  {                                                                                                     uint16_t _wEPVal;                                                                                   _wEPVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPREG_MASK;                                      PCD_SET_ENDPOINT((USBx), (bEpNum), (_wEPVal | USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_DTOG_RX));   } while (0) /* PCD_RX_DTOG */
#define PCD_TX_DTOG(USBx, bEpNum)                                                                     do                                                                                                  {                                                                                                     uint16_t _wEPVal;                                                                                   _wEPVal = PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPREG_MASK;                                      PCD_SET_ENDPOINT((USBx), (bEpNum), (_wEPVal | USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_DTOG_TX));   } while (0) /* PCD_TX_DTOG */
#define PCD_CLEAR_RX_DTOG(USBx, bEpNum)              do                                                 {                                                    uint16_t _wRegVal;                                 _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum));     if ((_wRegVal & USB_EP_DTOG_RX) != 0U)             {                                                    PCD_RX_DTOG((USBx), (bEpNum));                   }                                                } while (0) /* PCD_CLEAR_RX_DTOG */
#define PCD_CLEAR_TX_DTOG(USBx, bEpNum)              do                                                 {                                                    uint16_t _wRegVal;                                 _wRegVal = PCD_GET_ENDPOINT((USBx), (bEpNum));     if ((_wRegVal & USB_EP_DTOG_TX) != 0U)             {                                                    PCD_TX_DTOG((USBx), (bEpNum));                   }                                                } while (0) /* PCD_CLEAR_TX_DTOG */
#define PCD_SET_EP_ADDRESS(USBx, bEpNum, bAddr)                                       do                                                                                  {                                                                                     uint16_t _wRegVal;                                                                  _wRegVal = (PCD_GET_ENDPOINT((USBx), (bEpNum)) & USB_EPREG_MASK) | (bAddr);         PCD_SET_ENDPOINT((USBx), (bEpNum), (_wRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX));   } while (0) /* PCD_SET_EP_ADDRESS */
#define PCD_EP_TX_CNT(USBx, bEpNum) ((uint16_t *)((((uint32_t)(USBx)->BTABLE + ((uint32_t)(bEpNum) * 8U) + 2U) * PMA_ACCESS) + ((uint32_t)(USBx) + 0x400U)))
#define PCD_EP_RX_CNT(USBx, bEpNum) ((uint16_t *)((((uint32_t)(USBx)->BTABLE + ((uint32_t)(bEpNum) * 8U) + 6U) * PMA_ACCESS) + ((uint32_t)(USBx) + 0x400U)))
#define PCD_SET_EP_TX_ADDRESS(USBx, bEpNum, wAddr)                                                 do                                                                                               {                                                                                                  volatile uint16_t *_wRegVal;                                                                         uint32_t _wRegBase = (uint32_t)USBx;                                                             _wRegBase += (uint32_t)(USBx)->BTABLE;                                                           _wRegVal = (volatile uint16_t *)(_wRegBase + 0x400U + (((uint32_t)(bEpNum) * 8U) * PMA_ACCESS));     *_wRegVal = ((wAddr) >> 1) << 1;                                                               } while (0) /* PCD_SET_EP_TX_ADDRESS */
#define PCD_SET_EP_RX_ADDRESS(USBx, bEpNum, wAddr)                                                        do                                                                                                      {                                                                                                         volatile uint16_t *_wRegVal;                                                                                uint32_t _wRegBase = (uint32_t)USBx;                                                                    _wRegBase += (uint32_t)(USBx)->BTABLE;                                                                  _wRegVal = (volatile uint16_t *)(_wRegBase + 0x400U + ((((uint32_t)(bEpNum) * 8U) + 4U) * PMA_ACCESS));     *_wRegVal = ((wAddr) >> 1) << 1;                                                                      } while (0) /* PCD_SET_EP_RX_ADDRESS */
#define PCD_CALC_BLK32(pdwReg, wCount, wNBlocks)                      do                                                                  {                                                                     (wNBlocks) = (wCount) >> 5;                                         if (((wCount) & 0x1fU) == 0U)                                       {                                                                     (wNBlocks)--;                                                     }                                                                   *(pdwReg) |= (uint16_t)(((wNBlocks) << 10) | USB_CNTRX_BLSIZE);   } while (0) /* PCD_CALC_BLK32 */
#define PCD_CALC_BLK2(pdwReg, wCount, wNBlocks)   do                                              {                                                 (wNBlocks) = (wCount) >> 1;                     if (((wCount) & 0x1U) != 0U)                    {                                                 (wNBlocks)++;                                 }                                               *(pdwReg) |= (uint16_t)((wNBlocks) << 10);    } while (0) /* PCD_CALC_BLK2 */
#define PCD_SET_EP_CNT_RX_REG(pdwReg, wCount)          do                                                   {                                                      uint32_t wNBlocks;                                   *(pdwReg) &= 0x3FFU;                                 if ((wCount) > 62U)                                  {                                                      PCD_CALC_BLK32((pdwReg), (wCount), wNBlocks);      }                                                    else                                                 {                                                      if ((wCount) == 0U)                                  {                                                      *(pdwReg) |= USB_CNTRX_BLSIZE;                     }                                                    else                                                 {                                                      PCD_CALC_BLK2((pdwReg), (wCount), wNBlocks);       }                                                  }                                                  } while (0) /* PCD_SET_EP_CNT_RX_REG */
#define PCD_SET_EP_RX_DBUF0_CNT(USBx, bEpNum, wCount)                                                   do                                                                                                    {                                                                                                       uint32_t _wRegBase = (uint32_t)(USBx);                                                                volatile uint16_t *pdwReg;                                                                                _wRegBase += (uint32_t)(USBx)->BTABLE;                                                                pdwReg = (volatile uint16_t *)(_wRegBase + 0x400U + ((((uint32_t)(bEpNum) * 8U) + 2U) * PMA_ACCESS));     PCD_SET_EP_CNT_RX_REG(pdwReg, (wCount));                                                            } while (0)
#define PCD_SET_EP_TX_CNT(USBx, bEpNum, wCount)                                                           do                                                                                                      {                                                                                                         uint32_t _wRegBase = (uint32_t)(USBx);                                                                  volatile uint16_t *_wRegVal;                                                                                _wRegBase += (uint32_t)(USBx)->BTABLE;                                                                  _wRegVal = (volatile uint16_t *)(_wRegBase + 0x400U + ((((uint32_t)(bEpNum) * 8U) + 2U) * PMA_ACCESS));     *_wRegVal = (uint16_t)(wCount);                                                                       } while (0)
#define PCD_SET_EP_RX_CNT(USBx, bEpNum, wCount)                                                           do                                                                                                      {                                                                                                         uint32_t _wRegBase = (uint32_t)(USBx);                                                                  volatile uint16_t *_wRegVal;                                                                                _wRegBase += (uint32_t)(USBx)->BTABLE;                                                                  _wRegVal = (volatile uint16_t *)(_wRegBase + 0x400U + ((((uint32_t)(bEpNum) * 8U) + 6U) * PMA_ACCESS));     PCD_SET_EP_CNT_RX_REG(_wRegVal, (wCount));                                                            } while (0)
#define PCD_GET_EP_TX_CNT(USBx, bEpNum) ((uint32_t)(*PCD_EP_TX_CNT((USBx), (bEpNum))) & 0x3ffU)
#define PCD_GET_EP_RX_CNT(USBx, bEpNum) ((uint32_t)(*PCD_EP_RX_CNT((USBx), (bEpNum))) & 0x3ffU)
#define PCD_SET_EP_DBUF0_ADDR(USBx, bEpNum, wBuf0Addr)      do                                                        {                                                           PCD_SET_EP_TX_ADDRESS((USBx), (bEpNum), (wBuf0Addr));   } while (0) /* PCD_SET_EP_DBUF0_ADDR */
#define PCD_SET_EP_DBUF1_ADDR(USBx, bEpNum, wBuf1Addr)      do                                                        {                                                           PCD_SET_EP_RX_ADDRESS((USBx), (bEpNum), (wBuf1Addr));   } while (0) /* PCD_SET_EP_DBUF1_ADDR */
#define PCD_SET_EP_DBUF_ADDR(USBx, bEpNum, wBuf0Addr, wBuf1Addr)   do                                                               {                                                                  PCD_SET_EP_DBUF0_ADDR((USBx), (bEpNum), (wBuf0Addr));            PCD_SET_EP_DBUF1_ADDR((USBx), (bEpNum), (wBuf1Addr));          } while (0) /* PCD_SET_EP_DBUF_ADDR */
#define PCD_GET_EP_DBUF1_ADDR(USBx, bEpNum) (PCD_GET_EP_RX_ADDRESS((USBx), (bEpNum)))
#define PCD_SET_EP_DBUF0_CNT(USBx, bEpNum, bDir, wCount)     do                                                         {                                                            if ((bDir) == 0U) /* OUT endpoint */                       {                                                            PCD_SET_EP_RX_DBUF0_CNT((USBx), (bEpNum), (wCount));     }                                                          else                                                       {                                                            if ((bDir) == 1U)                                          { /* IN endpoint */                                          PCD_SET_EP_TX_CNT((USBx), (bEpNum), (wCount));           }                                                        }                                                        } while (0) /* SetEPDblBuf0Count*/
#define PCD_SET_EP_DBUF1_CNT(USBx, bEpNum, bDir, wCount)                                                     do                                                                                                         {                                                                                                            uint32_t _wBase = (uint32_t)(USBx);                                                                        volatile uint16_t *_wEPRegVal;                                                                                 if ((bDir) == 0U)                                                                                          { /* OUT endpoint */                                                                                         PCD_SET_EP_RX_CNT((USBx), (bEpNum), (wCount));                                                           }                                                                                                          else                                                                                                       {                                                                                                            if ((bDir) == 1U)                                                                                          { /* IN endpoint */                                                                                          _wBase += (uint32_t)(USBx)->BTABLE;                                                                        _wEPRegVal = (volatile uint16_t *)(_wBase + 0x400U + ((((uint32_t)(bEpNum) * 8U) + 6U) * PMA_ACCESS));         *_wEPRegVal = (uint16_t)(wCount);                                                                        }                                                                                                        }                                                                                                        } while (0) /* SetEPDblBuf1Count */
#define PCD_SET_EP_DBUF_CNT(USBx, bEpNum, bDir, wCount)         do                                                            {                                                               PCD_SET_EP_DBUF0_CNT((USBx), (bEpNum), (bDir), (wCount));     PCD_SET_EP_DBUF1_CNT((USBx), (bEpNum), (bDir), (wCount));   } while (0) /* PCD_SET_EP_DBUF_CNT */
#define PCD_GET_EP_DBUF0_CNT(USBx, bEpNum) (PCD_GET_EP_TX_CNT((USBx), (bEpNum)))
#define PCD_GET_EP_DBUF1_CNT(USBx, bEpNum) (PCD_GET_EP_RX_CNT((USBx), (bEpNum)))
#define assert_param(expr) ((void)0U)
#define __HAL_DBGMCU_FREEZE_TIM2() SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM2_STOP)
#define __HAL_DBGMCU_UNFREEZE_TIM2() CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM2_STOP)
#define __HAL_DBGMCU_FREEZE_TIM3() SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM3_STOP)
#define __HAL_DBGMCU_UNFREEZE_TIM3() CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_TIM3_STOP)
#define IS_TICKFREQ(FREQ) (((FREQ) == HAL_TICK_FREQ_10HZ) || ((FREQ) == HAL_TICK_FREQ_100HZ) || ((FREQ) == HAL_TICK_FREQ_1KHZ))
#define __STM32F1XX_H
#define STM32F1
#define __STM32F1_CMSIS_VERSION_MAIN (0x04) /*!< [31:24] main version */
#define __STM32F1_CMSIS_VERSION_SUB1 (0x03) /*!< [23:16] sub1 version */
#define __STM32F1_CMSIS_VERSION_SUB2 (0x04) /*!< [15:8]  sub2 version */
#define __STM32F1_CMSIS_VERSION_RC (0x00)   /*!< [7:0]  release candidate */
#define __STM32F1_CMSIS_VERSION ((__STM32F1_CMSIS_VERSION_MAIN << 24) | (__STM32F1_CMSIS_VERSION_SUB1 << 16) | (__STM32F1_CMSIS_VERSION_SUB2 << 8) | (__STM32F1_CMSIS_VERSION_RC))
#define __STM32F103xB_H
#define __CM3_REV 0x0200U         /*!< Core Revision r2p0                           */
#define __MPU_PRESENT 0U          /*!< Other STM32 devices does not provide an MPU  */
#define __Vendor_SysTickConfig 0U /*!< Set to 1 if different SysTick Config is used */
#define __CMSIS_GCC_H
#define __CM_CMSIS_VERSION_MAIN (5U)
#define __CM_CMSIS_VERSION_SUB (1U)
#define __CM_CMSIS_VERSION ((__CM_CMSIS_VERSION_MAIN << 16U) | __CM_CMSIS_VERSION_SUB)
#define __has_builtin(x) (0)

#define SysTick_CTRL_COUNTFLAG_Pos 16U                                 /*!< SysTick CTRL: COUNTFLAG Position */
#define SysTick_CTRL_COUNTFLAG_Msk (1UL << SysTick_CTRL_COUNTFLAG_Pos) /*!< SysTick CTRL: COUNTFLAG Mask */
#define SysTick_VAL_CURRENT_Pos 0U                                          /*!< SysTick VAL: CURRENT Position */
#define SysTick_VAL_CURRENT_Msk (0xFFFFFFUL /*<< SysTick_VAL_CURRENT_Pos*/) /*!< SysTick VAL: CURRENT Mask */
#define SysTick_CALIB_NOREF_Pos 31U                              /*!< SysTick CALIB: NOREF Position */
#define SysTick_CALIB_NOREF_Msk (1UL << SysTick_CALIB_NOREF_Pos) /*!< SysTick CALIB: NOREF Mask */
#define SysTick_CALIB_SKEW_Pos 30U                             /*!< SysTick CALIB: SKEW Position */
#define SysTick_CALIB_SKEW_Msk (1UL << SysTick_CALIB_SKEW_Pos) /*!< SysTick CALIB: SKEW Mask */
#define SysTick_CALIB_TENMS_Pos 0U                                          /*!< SysTick CALIB: TENMS Position */
#define SysTick_CALIB_TENMS_Msk (0xFFFFFFUL /*<< SysTick_CALIB_TENMS_Pos*/) /*!< SysTick CALIB: TENMS Mask */
#define NVIC_SetPriorityGrouping __NVIC_SetPriorityGrouping
#define NVIC_GetPriorityGrouping __NVIC_GetPriorityGrouping
#define NVIC_EnableIRQ __NVIC_EnableIRQ
#define NVIC_SetPriority __NVIC_SetPriority
#define NVIC_GetPriority __NVIC_GetPriority
#define FLASH_R_BASE (AHBPERIPH_BASE + 0x00002000UL) /*!< Flash registers base address */

#define FLASH ((FLASH_TypeDef *)FLASH_R_BASE)

#define RCC_APB1ENR_USBEN_Pos (23U)
#define RCC_APB1ENR_USBEN_Msk (0x1UL << RCC_APB1ENR_USBEN_Pos) /*!< 0x00800000 */
#define RCC_APB1ENR_USBEN RCC_APB1ENR_USBEN_Msk                /*!< USB Device clock enable */

#define USB_EP_DTOG_RX (0x1UL << (14U))  /*!< EndPoint Data TOGGLE RX */
#define USB_EPRX_STAT (0x3UL << (12U))   /*!< EndPoint RX STATus bit field */
#define USB_EP_DTOG_TX (0x1UL << (6U))   /*!< EndPoint Data TOGGLE TX */
#define USB_EPTX_STAT (0x3UL << (4U))    /*!< EndPoint TX STATus bit field */
#define USB_EP_TYPE_MASK (0x3UL << (9U)) /*!< EndPoint TYPE Mask */
#define USB_EPKIND_MASK (~USB_EP_KIND & USB_EPREG_MASK) /*!< EP_KIND EndPoint KIND */
#define USB_EPTX_DTOG1 0x00000010U  /*!< EndPoint TX Data TOGgle bit1 */
#define USB_EPTX_DTOG2 0x00000020U  /*!< EndPoint TX Data TOGgle bit2 */
#define USB_EPTX_DTOGMASK (USB_EPTX_STAT | USB_EPREG_MASK)
#define USB_EPRX_DTOG1 0x00001000U  /*!< EndPoint RX Data TOGgle bit1 */
#define USB_EPRX_DTOG2 0x00002000U  /*!< EndPoint RX Data TOGgle bit1 */
#define USB_EPRX_DTOGMASK (USB_EPRX_STAT | USB_EPREG_MASK)

#define USB_CNTR_PDWN (0x1UL << (1U))        /*!< Power down */
#define USB_CNTR_LP_MODE (0x1UL << (2U))     /*!< Low-power mode */
#define USB_CNTR_FSUSP (0x1UL << (3U))       /*!< Force suspend */
#define USB_CNTR_RESUME (0x1UL << (4U))      /*!< Resume request */
#define USB_CNTR_PMAOVRM (0x1UL << (14U))    /*!< Packet Memory Area Over / Underrun Interrupt Mask */
#define USB_ISTR_EP_ID (0xFUL << (0U))       /*!< Endpoint Identifier */
#define USB_ISTR_DIR (0x1UL << (4U))         /*!< Direction of transaction */
#define USB_ISTR_ESOF (0x1UL << (8U))        /*!< Expected Start Of Frame */
#define USB_ISTR_SOF (0x1UL << (9U))         /*!< Start Of Frame */
#define USB_ISTR_RESET (0x1UL << (10U))      /*!< USB RESET request */
#define USB_ISTR_SUSP (0x1UL << (11U))       /*!< Suspend mode request */
#define USB_ISTR_WKUP (0x1UL << (12U))       /*!< Wake up */
#define USB_ISTR_ERR (0x1UL << (13U))        /*!< Error */
#define USB_ISTR_PMAOVR (0x1UL << (14U))     /*!< Packet Memory Area Over / Underrun */
#define USB_ISTR_CTR (0x1UL << (15U))        /*!< Correct Transfer */


#define FLASH_ACR_LATENCY (0x7UL << 0U) /*!< LATENCY[2:0] bits (Latency) */
#define FLASH_ACR_HLFCYA (0x1UL << 3U) /*!< Flash Half Cycle Access Enable */
#define FLASH_ACR_PRFTBE (0x1UL << 4U) /*!< Prefetch Buffer Enable */
#define FLASH_ACR_PRFTBS (0x1UL << 5U) /*!< Prefetch Buffer Status */
#define FLASH_KEYR_FKEYR (0xFFFFFFFFUL << 0U) /*!< FPEC Key */
#define RDP_KEY (0xA5UL << 0U)          /*!< RDP Key */
#define FLASH_KEY1 (0x45670123UL << 0U) /*!< FPEC Key1 */
#define FLASH_KEY2 (0xCDEF89ABUL << 0U) /*!< FPEC Key2 */

#define OTG_FS_WKUP_IRQn USBWakeUp_IRQn
#define CEC_IRQn USBWakeUp_IRQn
#define USB_HP_IRQn USB_HP_CAN1_TX_IRQn
#define CAN1_TX_IRQn USB_HP_CAN1_TX_IRQn
#define CAN1_RX0_IRQn USB_LP_CAN1_RX0_IRQn
#define USB_LP_IRQn USB_LP_CAN1_RX0_IRQn
#define ADC1_IRQHandler ADC1_2_IRQHandler
#define TIM9_IRQHandler TIM1_BRK_IRQHandler
#define TIM1_BRK_TIM9_IRQHandler TIM1_BRK_IRQHandler
#define TIM1_BRK_TIM15_IRQHandler TIM1_BRK_IRQHandler
#define TIM1_TRG_COM_TIM17_IRQHandler TIM1_TRG_COM_IRQHandler
#define TIM1_TRG_COM_TIM11_IRQHandler TIM1_TRG_COM_IRQHandler
#define TIM11_IRQHandler TIM1_TRG_COM_IRQHandler
#define TIM10_IRQHandler TIM1_UP_IRQHandler
#define TIM1_UP_TIM10_IRQHandler TIM1_UP_IRQHandler
#define TIM1_UP_TIM16_IRQHandler TIM1_UP_IRQHandler
#define OTG_FS_WKUP_IRQHandler USBWakeUp_IRQHandler
#define CEC_IRQHandler USBWakeUp_IRQHandler
#define USB_HP_IRQHandler USB_HP_CAN1_TX_IRQHandler
#define CAN1_TX_IRQHandler USB_HP_CAN1_TX_IRQHandler
#define CAN1_RX0_IRQHandler USB_LP_CAN1_RX0_IRQHandler
#define USB_LP_IRQHandler USB_LP_CAN1_RX0_IRQHandler
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))
#define CLEAR_REG(REG) ((REG) = (0x0))
#define POSITION_VAL(VAL) (__CLZ(__RBIT(VAL)))


#define RCC_HSE_PREDIV_DIV1             0x00000000U
#define RCC_PLL_MUL9                    (0x7UL << 18U)
#define RCC_PERIPHCLK_USB               0x00000010U
#define RCC_USBCLKSOURCE_PLL_DIV1_5     0x00000000U
#define FLASH_LATENCY_2            0x00000002                /*!< FLASH Two Latency cycles */
#define RCC_HSI_ON                       RCC_CR_HSION           /*!< HSI clock activation */
#define RCC_OSCILLATORTYPE_HSE           0x00000001U
#define RCC_HSE_ON                       RCC_CR_HSEON                /*!< HSE clock activation */
#define RCC_PLL_ON                       0x00000002U /*!< PLL activation */
#define RCC_PLLSOURCE_HSE                RCC_CFGR_PLLSRC      /*!< HSE clock selected as PLL entry clock source */
#define RCC_CLOCKTYPE_SYSCLK             0x00000001U /*!< SYSCLK to configure */
#define RCC_CLOCKTYPE_HCLK               0x00000002U /*!< HCLK to configure */
#define RCC_CLOCKTYPE_PCLK1              0x00000004U /*!< PCLK1 to configure */
#define RCC_CLOCKTYPE_PCLK2              0x00000008U /*!< PCLK2 to configure */
#define RCC_SYSCLKSOURCE_PLLCLK          0x00000002U /*!< PLL selected as system clock */
#define RCC_HCLK_DIV1                    0x00000000U  /*!< HCLK not divided */
#define RCC_HCLK_DIV2                    0x00000400U  /*!< HCLK divided by 2 */
#define RCC_SYSCLK_DIV1                  0x00000000U  /*!< SYSCLK not divided */
#define HSE_VALUE 8000000U        /*!< Value of the External oscillator in Hz */
#define HSE_STARTUP_TIMEOUT 100U  /*!< Time out for HSE start up, in ms */
#define HSI_VALUE 8000000U        /*!< Value of the Internal oscillator in Hz*/
#define LSE_STARTUP_TIMEOUT 5000U /*!< Time out for LSE start up, in ms */
#define HAL_IS_BIT_SET(REG, BIT) (((REG) & (BIT)) != 0U)
#define HAL_IS_BIT_CLR(REG, BIT) (((REG) & (BIT)) == 0U)
#define UNUSED(X) (void)X /* To avoid gcc/g++ warnings */
#define __weak __attribute__((weak)) 
#define RCC_PLLSOURCE_HSI_DIV2 0x00000000U /*!< HSI clock divided by 2 selected as PLL entry clock source */
#define RCC_OSCILLATORTYPE_HSI 0x00000002U
#define RCC_OSCILLATORTYPE_LSE 0x00000004U
#define RCC_OSCILLATORTYPE_LSI 0x00000008U
#define RCC_HSE_OFF 0x00000000U                                       /*!< HSE clock deactivation */
#define RCC_HSE_BYPASS ((uint32_t)(RCC_CR_HSEBYP | RCC_CR_HSEON))     /*!< External clock source for HSE clock */
#define RCC_LSE_OFF 0x00000000U                                       /*!< LSE clock deactivation */
#define RCC_LSE_ON RCC_BDCR_LSEON                                     /*!< LSE clock activation */
#define RCC_LSE_BYPASS ((uint32_t)(RCC_BDCR_LSEBYP | RCC_BDCR_LSEON)) /*!< External clock source for LSE clock */
#define RCC_HSI_OFF 0x00000000U                                       /*!< HSI clock deactivation */
#define RCC_LSI_OFF 0x00000000U /*!< LSI clock deactivation */
#define RCC_PLL_NONE 0x00000000U /*!< PLL is not configured */
#define RCC_PLL_OFF 0x00000001U  /*!< PLL deactivation */
#define RCC_SYSCLKSOURCE_HSE RCC_CFGR_SW_HSE /*!< HSE selected as system clock */
#define RCC_SYSCLKSOURCE_STATUS_HSI RCC_CFGR_SWS_HSI                          /*!< HSI used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_HSE RCC_CFGR_SWS_HSE                          /*!< HSE used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_PLLCLK RCC_CFGR_SWS_PLL                       /*!< PLL used as system clock */
#define RCC_HCLK_DIV16 RCC_CFGR_PPRE1_DIV16                                   /*!< HCLK divided by 16 */
#define RCC_FLAG_HSIRDY ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_HSIRDY_Pos)) /*!< Internal High Speed clock ready flag */
#define RCC_FLAG_HSERDY ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_HSERDY_Pos)) /*!< External High Speed clock ready flag */
#define RCC_FLAG_PLLRDY ((uint8_t)((CR_REG_INDEX << 5U) | RCC_CR_PLLRDY_Pos)) /*!< PLL clock ready flag */
#define RCC_FLAG_LSIRDY ((uint8_t)((CSR_REG_INDEX << 5U) | RCC_CSR_LSIRDY_Pos)) /*!< Internal Low Speed oscillator Ready */
#define RCC_FLAG_LSERDY ((uint8_t)((BDCR_REG_INDEX << 5U) | RCC_BDCR_LSERDY_Pos)) /*!< External Low Speed oscillator Ready */
#define __HAL_RCC_PWR_CLK_ENABLE()                                                                 do                                                                                               {                                                                                                  volatile uint32_t tmpreg;                                                                            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN); /* Delay after an RCC peripheral clock enabling */     tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);                                              UNUSED(tmpreg);                                                                                } while (0U)
#define __HAL_RCC_PWR_CLK_DISABLE() (RCC->APB1ENR &= ~(RCC_APB1ENR_PWREN))
#define __HAL_RCC_PWR_IS_CLK_DISABLED() ((RCC->APB1ENR & (RCC_APB1ENR_PWREN)) == RESET)
#define __HAL_RCC_HSI_ENABLE() (*(volatile uint32_t *)RCC_CR_HSION_BB = ENABLE)
#define __HAL_RCC_HSI_DISABLE() (*(volatile uint32_t *)RCC_CR_HSION_BB = DISABLE)
#define __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(_HSICALIBRATIONVALUE_) (MODIFY_REG(RCC->CR, RCC_CR_HSITRIM, (uint32_t)(_HSICALIBRATIONVALUE_) << RCC_CR_HSITRIM_Pos))
#define __HAL_RCC_LSI_ENABLE() (*(volatile uint32_t *)RCC_CSR_LSION_BB = ENABLE)
#define __HAL_RCC_LSI_DISABLE() (*(volatile uint32_t *)RCC_CSR_LSION_BB = DISABLE)
#define __HAL_RCC_HSE_CONFIG(__STATE__)       do                                          {                                             if ((__STATE__) == RCC_HSE_ON)              {                                             SET_BIT(RCC->CR, RCC_CR_HSEON);           }                                           else if ((__STATE__) == RCC_HSE_OFF)        {                                             CLEAR_BIT(RCC->CR, RCC_CR_HSEON);           CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);        }                                           else if ((__STATE__) == RCC_HSE_BYPASS)     {                                             SET_BIT(RCC->CR, RCC_CR_HSEBYP);            SET_BIT(RCC->CR, RCC_CR_HSEON);           }                                           else                                        {                                             CLEAR_BIT(RCC->CR, RCC_CR_HSEON);           CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);        }                                         } while (0U)
#define __HAL_RCC_LSE_CONFIG(__STATE__)        do                                           {                                              if ((__STATE__) == RCC_LSE_ON)               {                                              SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);        }                                            else if ((__STATE__) == RCC_LSE_OFF)         {                                              CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEON);        CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEBYP);     }                                            else if ((__STATE__) == RCC_LSE_BYPASS)      {                                              SET_BIT(RCC->BDCR, RCC_BDCR_LSEBYP);         SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);        }                                            else                                         {                                              CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEON);        CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEBYP);     }                                          } while (0U)
#define __HAL_RCC_PLL_ENABLE() (*(volatile uint32_t *)RCC_CR_PLLON_BB = ENABLE)
#define __HAL_RCC_PLL_DISABLE() (*(volatile uint32_t *)RCC_CR_PLLON_BB = DISABLE)
#define __HAL_RCC_PLL_CONFIG(__RCC_PLLSOURCE__, __PLLMUL__) MODIFY_REG(RCC->CFGR, (RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL), ((__RCC_PLLSOURCE__) | (__PLLMUL__)))
#define __HAL_RCC_GET_PLL_OSCSOURCE() ((uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_PLLSRC)))
#define __HAL_RCC_SYSCLK_CONFIG(__SYSCLKSOURCE__) MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, (__SYSCLKSOURCE__))
#define __HAL_RCC_GET_SYSCLK_SOURCE() ((uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_SWS)))
#define __HAL_RCC_RTC_CONFIG(__RTC_CLKSOURCE__) MODIFY_REG(RCC->BDCR, RCC_BDCR_RTCSEL, (__RTC_CLKSOURCE__))
#define __HAL_RCC_BACKUPRESET_FORCE() (*(volatile uint32_t *)RCC_BDCR_BDRST_BB = ENABLE)
#define __HAL_RCC_BACKUPRESET_RELEASE() (*(volatile uint32_t *)RCC_BDCR_BDRST_BB = DISABLE)
#define __HAL_RCC_GET_FLAG(__FLAG__) (((((__FLAG__) >> 5U) == CR_REG_INDEX) ? RCC->CR : ((((__FLAG__) >> 5U) == BDCR_REG_INDEX) ? RCC->BDCR : RCC->CSR)) & (1U << ((__FLAG__) & RCC_FLAG_MASK)))
#define RCC_PERIPHCLK_RTC 0x00000001U
#define RCC_PERIPHCLK_ADC 0x00000002U
#define __HAL_RCC_HSE_PREDIV_CONFIG(__HSE_PREDIV_VALUE__) MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLXTPRE, (uint32_t)(__HSE_PREDIV_VALUE__))
#define __HAL_RCC_USB_CONFIG(__USBCLKSOURCE__) MODIFY_REG(RCC->CFGR, RCC_CFGR_USBPRE, (uint32_t)(__USBCLKSOURCE__))
#define __HAL_RCC_ADC_CONFIG(__ADCCLKSOURCE__) MODIFY_REG(RCC->CFGR, RCC_CFGR_ADCPRE, (uint32_t)(__ADCCLKSOURCE__))
#define RCC_DBP_TIMEOUT_VALUE 100U /* 100 ms */
#define RCC_LSE_TIMEOUT_VALUE LSE_STARTUP_TIMEOUT
#define CLOCKSWITCH_TIMEOUT_VALUE 5000 /* 5 s    */
#define HSE_TIMEOUT_VALUE HSE_STARTUP_TIMEOUT
#define HSI_TIMEOUT_VALUE 2U /* 2 ms (minimum Tick + 1) */
#define LSI_TIMEOUT_VALUE 2U /* 2 ms (minimum Tick + 1) */
#define PLL_TIMEOUT_VALUE 2U /* 2 ms (minimum Tick + 1) */
#define RCC_OFFSET (RCC_BASE - PERIPH_BASE)
#define RCC_CR_OFFSET 0x00U
#define RCC_BDCR_OFFSET 0x20U
#define RCC_CSR_OFFSET 0x24U
#define RCC_CR_OFFSET_BB (RCC_OFFSET + RCC_CR_OFFSET)
#define RCC_BDCR_OFFSET_BB (RCC_OFFSET + RCC_BDCR_OFFSET)
#define RCC_CSR_OFFSET_BB (RCC_OFFSET + RCC_CSR_OFFSET)
#define RCC_HSION_BIT_NUMBER RCC_CR_HSION_Pos
#define RCC_CR_HSION_BB ((uint32_t)(PERIPH_BB_BASE + (RCC_CR_OFFSET_BB * 32U) + (RCC_HSION_BIT_NUMBER * 4U)))
#define RCC_PLLON_BIT_NUMBER RCC_CR_PLLON_Pos
#define RCC_CR_PLLON_BB ((uint32_t)(PERIPH_BB_BASE + (RCC_CR_OFFSET_BB * 32U) + (RCC_PLLON_BIT_NUMBER * 4U)))
#define RCC_LSION_BIT_NUMBER RCC_CSR_LSION_Pos
#define RCC_CSR_LSION_BB ((uint32_t)(PERIPH_BB_BASE + (RCC_CSR_OFFSET_BB * 32U) + (RCC_LSION_BIT_NUMBER * 4U)))
#define RCC_BDRST_BIT_NUMBER RCC_BDCR_BDRST_Pos
#define RCC_BDCR_BDRST_BB ((uint32_t)(PERIPH_BB_BASE + (RCC_BDCR_OFFSET_BB * 32U) + (RCC_BDRST_BIT_NUMBER * 4U)))
#define CR_REG_INDEX ((uint8_t)1)
#define BDCR_REG_INDEX ((uint8_t)2)
#define CSR_REG_INDEX ((uint8_t)3)
#define RCC_FLAG_MASK ((uint8_t)0x1F)
#define EP_TYPE_CTRL 0U
#define EP_TYPE_ISOC 1U
#define EP_TYPE_BULK 2U
#define EP_TYPE_INTR 3U
#define USBD_FS_SPEED 2U
#define BTABLE_ADDRESS 0x000U
#define PMA_ACCESS 2U
#define EP_ADDR_MSK 0x7U
#define PCD_SPEED_FULL USBD_FS_SPEED
#define PCD_SNG_BUF 0U
#define __NVIC_PRIO_BITS 4U       /*!< STM32 uses 4 Bits for the Priority Levels    */
#define __ASM __asm
#define __INLINE inline
#define __STATIC_INLINE static inline
#define __STATIC_FORCEINLINE __attribute__((always_inline)) static inline
#define __NO_RETURN __attribute__((__noreturn__))
#define __USED __attribute__((used))
#define __WEAK __attribute__((weak))
#define __PACKED __attribute__((packed, aligned(1)))
#define __PACKED_STRUCT struct __attribute__((packed, aligned(1)))
#define __PACKED_UNION union __attribute__((packed, aligned(1)))
#define __NOP() __ASM volatile("nop")
#define __WFI() __ASM volatile("wfi")
#define __WFE() __ASM volatile("wfe")
#define __SEV() __ASM volatile("sev")
#define SCB_AIRCR_VECTKEY_Pos 16U                                 /*!< SCB AIRCR: VECTKEY Position */
#define SCB_AIRCR_VECTKEY_Msk (0xFFFFUL << SCB_AIRCR_VECTKEY_Pos) /*!< SCB AIRCR: VECTKEY Mask */
#define SCB_AIRCR_PRIGROUP_Pos 8U                              /*!< SCB AIRCR: PRIGROUP Position */
#define SCB_AIRCR_PRIGROUP_Msk (7UL << SCB_AIRCR_PRIGROUP_Pos) /*!< SCB AIRCR: PRIGROUP Mask */
#define SCB_AIRCR_SYSRESETREQ_Pos 2U                                 /*!< SCB AIRCR: SYSRESETREQ Position */
#define SCB_AIRCR_SYSRESETREQ_Msk (1UL << SCB_AIRCR_SYSRESETREQ_Pos) /*!< SCB AIRCR: SYSRESETREQ Mask */
#define SCB_SCR_SLEEPDEEP_Pos 2U                             /*!< SCB SCR: SLEEPDEEP Position */
#define SCB_SCR_SLEEPDEEP_Msk (1UL << SCB_SCR_SLEEPDEEP_Pos) /*!< SCB SCR: SLEEPDEEP Mask */
#define SCB_SCR_SLEEPONEXIT_Pos 1U                               /*!< SCB SCR: SLEEPONEXIT Position */
#define SCB_SCR_SLEEPONEXIT_Msk (1UL << SCB_SCR_SLEEPONEXIT_Pos) /*!< SCB SCR: SLEEPONEXIT Mask */
#define SysTick_CTRL_CLKSOURCE_Pos 2U                                  /*!< SysTick CTRL: CLKSOURCE Position */
#define SysTick_CTRL_CLKSOURCE_Msk (1UL << SysTick_CTRL_CLKSOURCE_Pos) /*!< SysTick CTRL: CLKSOURCE Mask */
#define SysTick_CTRL_TICKINT_Pos 1U                                /*!< SysTick CTRL: TICKINT Position */
#define SysTick_CTRL_TICKINT_Msk (1UL << SysTick_CTRL_TICKINT_Pos) /*!< SysTick CTRL: TICKINT Mask */
#define SysTick_CTRL_ENABLE_Pos 0U                                   /*!< SysTick CTRL: ENABLE Position */
#define SysTick_CTRL_ENABLE_Msk (1UL /*<< SysTick_CTRL_ENABLE_Pos*/) /*!< SysTick CTRL: ENABLE Mask */
#define SysTick_LOAD_RELOAD_Pos 0U                                          /*!< SysTick LOAD: RELOAD Position */
#define SysTick_LOAD_RELOAD_Msk (0xFFFFFFUL /*<< SysTick_LOAD_RELOAD_Pos*/) /*!< SysTick LOAD: RELOAD Mask */
#define SCS_BASE (0xE000E000UL)            /*!< System Control Space Base Address */
#define SysTick_BASE (SCS_BASE + 0x0010UL) /*!< SysTick Base Address */
#define NVIC_BASE (SCS_BASE + 0x0100UL)    /*!< NVIC Base Address */
#define SCB_BASE (SCS_BASE + 0x0D00UL)     /*!< System Control Block Base Address */
#define SCB ((SCB_Type *)SCB_BASE)                   /*!< SCB configuration struct */
#define SysTick ((SysTick_Type *)SysTick_BASE)       /*!< SysTick configuration struct */
#define NVIC ((NVIC_Type *)NVIC_BASE)                /*!< NVIC configuration struct */
#define NVIC_USER_IRQ_OFFSET 16
#define PERIPH_BASE 0x40000000UL     /*!< Peripheral base address in the alias region */
#define PERIPH_BB_BASE 0x42000000UL /*!< Peripheral base address in the bit-band region */
#define APB1PERIPH_BASE PERIPH_BASE
#define AHBPERIPH_BASE (PERIPH_BASE + 0x00020000UL)
#define PWR_BASE (APB1PERIPH_BASE + 0x00007000UL)
#define RCC_BASE (AHBPERIPH_BASE + 0x00001000UL)
#define USB_BASE (APB1PERIPH_BASE + 0x00005C00UL)    /*!< USB_IP Peripheral Registers base address */
#define USB ((USB_TypeDef *)USB_BASE)
#define PWR ((PWR_TypeDef *)PWR_BASE)
#define RCC ((RCC_TypeDef *)RCC_BASE)
#define PWR_CR_DBP_Pos (8U)
#define PWR_CR_DBP_Msk (0x1UL << PWR_CR_DBP_Pos) /*!< 0x00000100 */
#define PWR_CR_DBP PWR_CR_DBP_Msk                /*!< Disable Backup Domain write protection */
#define RCC_CR_HSION_Pos (0U)
#define RCC_CR_HSION_Msk (0x1UL << RCC_CR_HSION_Pos) /*!< 0x00000001 */
#define RCC_CR_HSION RCC_CR_HSION_Msk                /*!< Internal High Speed clock enable */
#define RCC_CR_HSIRDY_Pos (1U)
#define RCC_CR_HSITRIM_Pos (3U)
#define RCC_CR_HSITRIM_Msk (0x1FUL << RCC_CR_HSITRIM_Pos) /*!< 0x000000F8 */
#define RCC_CR_HSITRIM RCC_CR_HSITRIM_Msk                 /*!< Internal High Speed clock trimming */
#define RCC_CR_HSEON_Pos (16U)
#define RCC_CR_HSEON_Msk (0x1UL << RCC_CR_HSEON_Pos) /*!< 0x00010000 */
#define RCC_CR_HSEON RCC_CR_HSEON_Msk                /*!< External High Speed clock enable */
#define RCC_CR_HSERDY_Pos (17U)
#define RCC_CR_HSEBYP_Pos (18U)
#define RCC_CR_HSEBYP_Msk (0x1UL << RCC_CR_HSEBYP_Pos) /*!< 0x00040000 */
#define RCC_CR_HSEBYP RCC_CR_HSEBYP_Msk                /*!< External High Speed clock Bypass */
#define RCC_CR_PLLON_Pos (24U)
#define RCC_CR_PLLRDY_Pos (25U)
#define RCC_CFGR_SW_Pos (0U)
#define RCC_CFGR_SW_Msk (0x3UL << RCC_CFGR_SW_Pos) /*!< 0x00000003 */
#define RCC_CFGR_SW RCC_CFGR_SW_Msk                /*!< SW[1:0] bits (System clock Switch) */
#define RCC_CFGR_SW_HSE 0x00000001U /*!< HSE selected as system clock */
#define RCC_CFGR_SWS_Pos (2U)
#define RCC_CFGR_SWS_Msk (0x3UL << RCC_CFGR_SWS_Pos) /*!< 0x0000000C */
#define RCC_CFGR_SWS RCC_CFGR_SWS_Msk                /*!< SWS[1:0] bits (System Clock Switch Status) */
#define RCC_CFGR_SWS_HSI 0x00000000U /*!< HSI oscillator used as system clock */
#define RCC_CFGR_SWS_HSE 0x00000004U /*!< HSE oscillator used as system clock */
#define RCC_CFGR_SWS_PLL 0x00000008U /*!< PLL used as system clock */
#define RCC_CFGR_HPRE_Pos (4U)
#define RCC_CFGR_HPRE_Msk (0xFUL << RCC_CFGR_HPRE_Pos) /*!< 0x000000F0 */
#define RCC_CFGR_HPRE RCC_CFGR_HPRE_Msk                /*!< HPRE[3:0] bits (AHB prescaler) */
#define RCC_CFGR_PPRE1_Pos (8U)
#define RCC_CFGR_PPRE1_Msk (0x7UL << RCC_CFGR_PPRE1_Pos) /*!< 0x00000700 */
#define RCC_CFGR_PPRE1 RCC_CFGR_PPRE1_Msk                /*!< PRE1[2:0] bits (APB1 prescaler) */
#define RCC_CFGR_PPRE1_DIV16 0x00000700U /*!< HCLK divided by 16 */
#define RCC_CFGR_PPRE2_Pos (11U)
#define RCC_CFGR_PPRE2_Msk (0x7UL << RCC_CFGR_PPRE2_Pos) /*!< 0x00003800 */
#define RCC_CFGR_PPRE2 RCC_CFGR_PPRE2_Msk                /*!< PRE2[2:0] bits (APB2 prescaler) */
#define RCC_CFGR_ADCPRE_Pos (14U)
#define RCC_CFGR_ADCPRE_Msk (0x3UL << RCC_CFGR_ADCPRE_Pos) /*!< 0x0000C000 */
#define RCC_CFGR_ADCPRE RCC_CFGR_ADCPRE_Msk                /*!< ADCPRE[1:0] bits (ADC prescaler) */
#define RCC_CFGR_PLLSRC_Pos (16U)
#define RCC_CFGR_PLLSRC_Msk (0x1UL << RCC_CFGR_PLLSRC_Pos) /*!< 0x00010000 */
#define RCC_CFGR_PLLSRC RCC_CFGR_PLLSRC_Msk                /*!< PLL entry clock source */
#define RCC_CFGR_PLLXTPRE_Pos (17U)
#define RCC_CFGR_PLLXTPRE_Msk (0x1UL << RCC_CFGR_PLLXTPRE_Pos) /*!< 0x00020000 */
#define RCC_CFGR_PLLXTPRE RCC_CFGR_PLLXTPRE_Msk                /*!< HSE divider for PLL entry */
#define RCC_CFGR_PLLMULL_Pos (18U)
#define RCC_CFGR_PLLMULL_Msk (0xFUL << RCC_CFGR_PLLMULL_Pos) /*!< 0x003C0000 */
#define RCC_CFGR_PLLMULL RCC_CFGR_PLLMULL_Msk                /*!< PLLMUL[3:0] bits (PLL multiplication factor) */
#define RCC_CFGR_USBPRE_Pos (22U)
#define RCC_CFGR_USBPRE_Msk (0x1UL << RCC_CFGR_USBPRE_Pos) /*!< 0x00400000 */
#define RCC_CFGR_USBPRE RCC_CFGR_USBPRE_Msk                /*!< USB Device prescaler */
#define RCC_APB1ENR_PWREN_Pos (28U)
#define RCC_APB1ENR_PWREN_Msk (0x1UL << RCC_APB1ENR_PWREN_Pos) /*!< 0x10000000 */
#define RCC_APB1ENR_PWREN RCC_APB1ENR_PWREN_Msk                /*!< Power interface clock enable */
#define RCC_BDCR_LSEON_Pos (0U)
#define RCC_BDCR_LSEON_Msk (0x1UL << RCC_BDCR_LSEON_Pos) /*!< 0x00000001 */
#define RCC_BDCR_LSEON RCC_BDCR_LSEON_Msk                /*!< External Low Speed oscillator enable */
#define RCC_BDCR_LSERDY_Pos (1U)
#define RCC_BDCR_LSEBYP_Pos (2U)
#define RCC_BDCR_LSEBYP_Msk (0x1UL << RCC_BDCR_LSEBYP_Pos) /*!< 0x00000004 */
#define RCC_BDCR_LSEBYP RCC_BDCR_LSEBYP_Msk                /*!< External Low Speed oscillator Bypass */
#define RCC_BDCR_RTCSEL_Pos (8U)
#define RCC_BDCR_RTCSEL_Msk (0x3UL << RCC_BDCR_RTCSEL_Pos) /*!< 0x00000300 */
#define RCC_BDCR_RTCSEL RCC_BDCR_RTCSEL_Msk                /*!< RTCSEL[1:0] bits (RTC clock source selection) */
#define RCC_BDCR_BDRST_Pos (16U)
#define RCC_CSR_LSION_Pos (0U)
#define RCC_CSR_LSIRDY_Pos (1U)
#define USB_EP_CTR_RX (0x1UL << (15U))   /*!< EndPoint Correct TRansfer RX */
#define USB_EP_SETUP (0x1UL << (11U))    /*!< EndPoint SETUP */
#define USB_EP_T_FIELD (0x3UL << (9U))   /*!< EndPoint TYPE */
#define USB_EP_KIND (0x1UL << (8U))      /*!< EndPoint KIND */
#define USB_EP_CTR_TX (0x1UL << (7U))    /*!< EndPoint Correct TRansfer TX */
#define USB_EPADDR_FIELD (0xFUL << (0U)) /*!< EndPoint ADDRess FIELD */
#define USB_EPREG_MASK (USB_EP_CTR_RX | USB_EP_SETUP | USB_EP_T_FIELD | USB_EP_KIND | USB_EP_CTR_TX | USB_EPADDR_FIELD)
#define USB_EP_BULK 0x00000000U          /*!< EndPoint BULK */
#define USB_EP_CONTROL 0x00000200U       /*!< EndPoint CONTROL */
#define USB_EP_ISOCHRONOUS 0x00000400U   /*!< EndPoint ISOCHRONOUS */
#define USB_EP_INTERRUPT 0x00000600U     /*!< EndPoint INTERRUPT */
#define USB_EP_T_MASK (~USB_EP_T_FIELD & USB_EPREG_MASK)
#define USB_EP_TX_DIS 0x00000000U   /*!< EndPoint TX DISabled */
#define USB_EP_TX_STALL 0x00000010U /*!< EndPoint TX STALLed */
#define USB_EP_TX_NAK 0x00000020U   /*!< EndPoint TX NAKed */
#define USB_EP_TX_VALID 0x00000030U /*!< EndPoint TX VALID */
#define USB_EP_RX_DIS 0x00000000U   /*!< EndPoint RX DISabled */
#define USB_EP_RX_STALL 0x00001000U /*!< EndPoint RX STALLed */
#define USB_EP_RX_NAK 0x00002000U   /*!< EndPoint RX NAKed */
#define USB_EP_RX_VALID 0x00003000U /*!< EndPoint RX VALID */
#define USB_CNTR_FRES (0x1UL << (0U))        /*!< Force USB Reset */
#define USB_CNTR_ESOFM (0x1UL << (8U))       /*!< Expected Start Of Frame Interrupt Mask */
#define USB_CNTR_SOFM (0x1UL << (9U))        /*!< Start Of Frame Interrupt Mask */
#define USB_CNTR_RESETM (0x1UL << (10U))     /*!< RESET Interrupt Mask */
#define USB_CNTR_SUSPM (0x1UL << (11U))      /*!< Suspend mode Interrupt Mask */
#define USB_CNTR_WKUPM (0x1UL << (12U))      /*!< Wakeup Interrupt Mask */
#define USB_CNTR_ERRM (0x1UL << (13U))       /*!< Error Interrupt Mask */
#define USB_CNTR_CTRM (0x1UL << (15U))       /*!< Correct Transfer Interrupt Mask */
#define USB_DADDR_EF (0x1UL << (7U))         /*!< Enable Function */
#define SET_BIT(REG, BIT) ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT) ((REG) & (BIT))
#define WRITE_REG(REG, VAL) ((REG) = (VAL))
#define READ_REG(REG) ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK) WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
 #define USBD_VID 1155
#define USBD_LANGID_STRING 1033
#define USBD_MANUFACTURER_STRING "STMicroelectronics"
#define USBD_PID_FS 22315
#define USBD_PRODUCT_STRING_FS "STM32 Human interface"
#define USBD_CONFIGURATION_STRING_FS "HID Config"
#define USBD_INTERFACE_STRING_FS "HID Interface"
#define USBD_MAX_NUM_INTERFACES 1
#define USBD_MAX_NUM_CONFIGURATION 1
#define USBD_MAX_STR_DESC_SIZ 512
#define HID_FS_BINTERVAL 0xA
#define DEVICE_FS 0
#define USBD_malloc (uint32_t *)USBD_static_malloc
#define NULL 0U
#define USB_LEN_DEV_QUALIFIER_DESC 0x0AU
#define USB_LEN_DEV_DESC 0x12U
#define USB_LEN_LANGID_STR_DESC 0x04U
#define USBD_IDX_LANGID_STR 0x00U
#define USBD_IDX_MFC_STR 0x01U
#define USBD_IDX_PRODUCT_STR 0x02U
#define USBD_IDX_SERIAL_STR 0x03U
#define USBD_IDX_CONFIG_STR 0x04U
#define USBD_IDX_INTERFACE_STR 0x05U
#define USB_REQ_TYPE_STANDARD 0x00U
#define USB_REQ_TYPE_CLASS 0x20U
#define USB_REQ_TYPE_VENDOR 0x40U
#define USB_REQ_TYPE_MASK 0x60U
#define USB_REQ_RECIPIENT_DEVICE 0x00U
#define USB_REQ_RECIPIENT_INTERFACE 0x01U
#define USB_REQ_RECIPIENT_ENDPOINT 0x02U
#define USB_REQ_RECIPIENT_MASK 0x03U
#define USB_REQ_GET_STATUS 0x00U
#define USB_REQ_CLEAR_FEATURE 0x01U
#define USB_REQ_SET_FEATURE 0x03U
#define USB_REQ_SET_ADDRESS 0x05U
#define USB_REQ_GET_DESCRIPTOR 0x06U
#define USB_REQ_SET_DESCRIPTOR 0x07U
#define USB_REQ_GET_CONFIGURATION 0x08U
#define USB_REQ_SET_CONFIGURATION 0x09U
#define USB_REQ_GET_INTERFACE 0x0AU
#define USB_REQ_SET_INTERFACE 0x0BU
#define USB_REQ_SYNCH_FRAME 0x0CU
#define USB_DESC_TYPE_DEVICE 0x01U
#define USB_DESC_TYPE_CONFIGURATION 0x02U
#define USB_DESC_TYPE_STRING 0x03U
#define USB_DESC_TYPE_INTERFACE 0x04U
#define USB_DESC_TYPE_ENDPOINT 0x05U
#define USB_DESC_TYPE_DEVICE_QUALIFIER 0x06U
#define USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION 0x07U
#define USB_CONFIG_REMOTE_WAKEUP 0x02U
#define USB_CONFIG_SELF_POWERED 0x01U
#define USB_FEATURE_EP_HALT 0x00U
#define USB_FEATURE_REMOTE_WAKEUP 0x01U
#define USB_MAX_EP0_SIZE 64U
#define USBD_STATE_DEFAULT 0x01U
#define USBD_STATE_ADDRESSED 0x02U
#define USBD_STATE_CONFIGURED 0x03U
#define USBD_STATE_SUSPENDED 0x04U
#define USBD_EP0_IDLE 0x00U
#define USBD_EP0_SETUP 0x01U
#define USBD_EP0_DATA_IN 0x02U
#define USBD_EP0_DATA_OUT 0x03U
#define USBD_EP0_STATUS_IN 0x04U
#define USBD_EP0_STATUS_OUT 0x05U
#define USBD_EP_TYPE_CTRL 0x00U
#define USBD_EP_TYPE_ISOC 0x01U
#define USBD_EP_TYPE_BULK 0x02U
#define USBD_EP_TYPE_INTR 0x03U
#define SWAPBYTE(addr) (((uint16_t)(*((uint8_t *)(addr)))) +                         (((uint16_t)(*(((uint8_t *)(addr)) + 1U))) << 8U))
#define LOBYTE(x) ((uint8_t)((x) & 0x00FFU))
#define HIBYTE(x) ((uint8_t)(((x) & 0xFF00U) >> 8U))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define HID_EPIN_ADDR 0x81U
#define HID_EPIN_SIZE 0x04U
#define USB_HID_CONFIG_DESC_SIZ 34U
#define USB_HID_DESC_SIZ 9U
#define HID_MOUSE_REPORT_DESC_SIZE 63U
#define HID_DESCRIPTOR_TYPE 0x21U
#define HID_REPORT_DESC 0x22U
#define HID_HS_BINTERVAL 0x07U
#define USBD_HID_CLASS &USBD_HID
#define UID_BASE 0x1FFFF7E8UL /*!< Unique device ID register base address */
#define DEVICE_ID1 (UID_BASE)
#define DEVICE_ID2 (UID_BASE + 0x4)
#define DEVICE_ID3 (UID_BASE + 0x8)
#define USB_SIZ_STRING_SERIAL 0x1A

#define assert_param(expr) ((void)0U)

#define PCD_MIN(a, b) (((a) < (b)) ? (a) : (b))

#define PCD_MAX(a, b) (((a) > (b)) ? (a) : (b))

#define IDCODE_DEVID_MASK 0x00000FFFU
