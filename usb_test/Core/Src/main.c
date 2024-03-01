#define NULL 0
#include "./IncludesForDrivers/stm32f1xx.h"

const uint8_t AHBPrescTable[16U] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8U] = {0, 0, 0, 0, 1, 2, 3, 4};

#include "Inc_USB_Device.h"
#include "Inc_Clock_Config.h"

//#include "./IncludesForDrivers/stm32f1xx_hal.c"
//#include "./IncludesForDrivers/usbd_core.c"

void SystemInit(void) {}
void SystemClock_Config(void);
void USB_DEVICE_Init(void);
void Error_Handler();

extern PCD_HandleTypeDef hpcd_USB_FS;

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
}

keyboardHID keyboardhid = {0, 0, 0, 0, 0, 0, 0, 0};

int main(void)
{
  SystemClock_Config();
  USB_DEVICE_Init();

  uint8_t report[sizeof(keyboardHID)];

  for (volatile int i = 0; i <= 3; i++)
  {
    HAL_Delay(200);
    keyboardhid.KeyCode1 = 0x2C;
    keyboardhid.KeyCode2 = 0x0B;
    keyboardhid.KeyCode3 = 0x12;
    keyboardhid.KeyCode4 = 0x0F;
    keyboardhid.KeyCode5 = 0x04;
    USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(report));
    HAL_Delay(30);

    keyboardhid.KeyCode1 = 0x00;
    keyboardhid.KeyCode2 = 0x00;
    keyboardhid.KeyCode3 = 0x00;
    keyboardhid.KeyCode4 = 0x00;
    keyboardhid.KeyCode5 = 0x00;
    USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(report));
    HAL_Delay(200);
  }
  while (1)
  {
  }
}

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
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

void USB_DEVICE_Init(void)
{
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_HID) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    Error_Handler();
  }
}

void Error_Handler()
{
  //__disable_irq();
  while (1)
  {
  }
}
