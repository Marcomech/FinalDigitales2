#include "usbd_hid.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

extern USBD_HandleTypeDef hUsbDeviceFS;

typedef struct {
	uint8_t ModifierKey;
	uint8_t Reserved;
	uint8_t KeyCode1;
	uint8_t KeyCode2;
	uint8_t KeyCode3;
	uint8_t KeyCode4;
	uint8_t KeyCode5;
	uint8_t KeyCode6;
} keyboardHID;

keyboardHID keyboardhid = {0,0,0,0,0,0,0,0};


int main(void){
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_USB_DEVICE_Init();


  while (1)  {
	keyboardhid.KeyCode1 = 0x0B;
	keyboardhid.KeyCode2 = 0x12;
	keyboardhid.KeyCode3 = 0x0F;
	keyboardhid.KeyCode4 = 0x04;
	USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
	HAL_Delay(50);
	keyboardhid.KeyCode1 = 0x00;
	keyboardhid.KeyCode2 = 0x00;
	keyboardhid.KeyCode3 = 0x00;
	keyboardhid.KeyCode4 = 0x00;
	USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
	HAL_Delay(1000);
  }
}

void SystemClock_Config(void){
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

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)  {
    Error_Handler();
  }
  
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void){
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

void Error_Handler(void){
  __disable_irq();
  while (1)  {  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line){}
#endif 
