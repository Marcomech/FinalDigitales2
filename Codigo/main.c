#include "./Inc/definiciones.h"
#include "../../Middlewares/usbd_hid.h"

void MX_USB_DEVICE_Init(void);
void SystemClock_Config(void);
static void GPIO_Init(void);

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
  GPIO_Init();
  MX_USB_DEVICE_Init();

  uint8_t report[sizeof(keyboardHID)];

  for (volatile int i=0; i<6; i++){
	  keyboardhid.KeyCode1 = 0x0B;
	  keyboardhid.KeyCode2 = 0x12;
	  keyboardhid.KeyCode3 = 0x0F;
	  keyboardhid.KeyCode4 = 0x04;

	  memcpy(report, &keyboardhid, sizeof(keyboardHID));
	  USBD_HID_SendReport(&hUsbDeviceFS, report, sizeof(report));

	  HAL_Delay(50);

	  keyboardhid.KeyCode1 = 0x00;
	  keyboardhid.KeyCode2 = 0x00;
	  keyboardhid.KeyCode3 = 0x00;
	  keyboardhid.KeyCode4 = 0x00;

	  memcpy(report, &keyboardhid, sizeof(keyboardHID));
	  USBD_HID_SendReport(&hUsbDeviceFS, report, sizeof(report));

	  HAL_Delay(400);
  }
  
	for (;;){}
	return 0; 
};

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
	
	/*  Original
 	void SetClock(){
 	RCC->CR	 |= (1 << 16);				// Habilita la fuente de reloj externo de alta velocidad (HSE)
 	while (!(RCC->CR & (1 << 17)));		// Espera al lock (bit 17 en el registro de ctrl.)

 	RCC->CR	 &= ~(1 << 24);				// Desactiva el PLL 
 	RCC->CFGR |= (0b0100 << 18);		// Set PLLMULL to 6. multipica el clock 8MHz * 6 = 48Mhz.
 	RCC->CFGR |= (1 << 16);				// Select HSE as the PLL source clock.
 	RCC->CR	  |= (1 << 24);				// Enable PLL

 	while (!(RCC->CR & (1 << 25)));		// Wait for PLL to lock.

 	FLASH->ACR |= (0b010 << 0);			// Set FLASH WAIT STATE to 2. 

 	RCC->APB1ENR |= (1 << 23);			// Enable USB clock.

 	RCC->CFGR |= (1 << 22);				// Set USB prescaler to 0.
 	RCC->CFGR |= (0b0000 << 4);			// Set AHB HPRE division to 1.
 	//RCC->CFGR |= (0b0000 << 8);			// Set APB1 PPRE1 division to 1. 
 	RCC->CFGR |= (0b10	<< 0);			// Select PLL clock as the system clock

 	while (!(RCC->CFGR & (0b10 << 2)));	// Wait for PLL clock to be selected

 };
 */
}

static void GPIO_Init(void){
	RCC->APB2ENR	= (1<<2);				// Enable clock for GPIOA
	RCC->APB2ENR |= (1<<3);				// Enable clock for GPIOB
	RCC->APB2ENR |= (1<<4);				// Enable clock for GPIOC

	GPIOC->CRH	= (3 << (13-8)*4); 	 //C13 output
	GPIOB->CRH	= (8 << (12-8)*4);	 //B12 input
	GPIOB->CRH |= (8 << (13-8)*4);	 //B13 input
	GPIOB->CRH |= (8 << (14-8)*4);	 //B14 input
	GPIOB->CRH |= (8 << (15-8)*4);	 //B15 input
}

void Error_Handler(void){
  __disable_irq();
  while (1)  {  }
}