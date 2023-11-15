#include "definiciones.h"
#define N 1024

void sigma_delta(int *wavein, int *waveout);

void SetClock(){
	RCC->CR   |= (1 << 16);						// Habilita la fuente de reloj externo de alta velocidad (HSE)
	while (!(RCC->CR & (1 << 17)));		// Espera al lock (bit 17 en el registro de ctrl.)

	RCC->CR   &= ~(1 << 24);					// Desactiva el PLL 

	RCC->CFGR |= (0b0100 << 18);			// Set PLLMULL to 6. multipica el clock 8MHz * 6 = 48Mhz.
	RCC->CFGR |=  (1 << 16);					// Select HSE as the PLL source clock.
	RCC->CR   |=  (1 << 24);					// Enable PLL

	while (!(RCC->CR & (1 << 25)));		// Wait for PLL to lock.

	FLASH->ACR |= (0b010 << 0);				// Set FLASH WAIT STATE to 2. 

	RCC->CFGR |= (0b0000 << 4);				// Set AHB HPRE division to 1. 	 (No divide la frecuencia para el general)
  RCC->CFGR |= (0b100  << 8);				// Set APB1 PPRE1 division to 2. (Setea una frec de 72MHz/2= 36 Mhz. Para APB1)
	RCC->CFGR |= (0b10   << 0);				// Select PLL clock as the system clock

	while (!(RCC->CFGR & (0b10 << 2)));			// Wait for PLL clock to be selected
};

void ConfigDMA(unsigned int *data){
  RCC->APB2ENR  = (1<<2);          // Enable clock for GPIOA
  RCC->APB2ENR |= (1<<3);          // Enable clock for GPIOB
  RCC->APB2ENR |= (1<<4);          // Enable clock for GPIOC
  RCC->APB1ENR |= (1 << 0);	       // Enable TIM2 clock.
	RCC->AHBENR  |= (1 << 0);	     		 // Enable DMA1 clock.

  GPIOC->CRH  = (3 << (13-8)*4); 	 //C13 output
  GPIOB->CRH  = (8 << (12-8)*4);   //B12 input
  GPIOB->CRH |= (8 << (13-8)*4);   //B13 input
  GPIOB->CRH |= (8 << (14-8)*4);   //B14 input
  GPIOB->CRH |= (8 << (15-8)*4);   //B15 input

	DMA1->CHN[CHN2].CNDTR = N;   														// Transfer size
	DMA1->CHN[CHN2].CMAR  = (unsigned int) data;				  	// Memory source address
	DMA1->CHN[CHN2].CPAR  = (unsigned int) & GPIOC->OUT;    // Peripheral destination address

	DMA1->CHN[CHN2].CCR  = 0;				         // Reset CCR
	DMA1->CHN[CHN2].CCR &= ~(1 << 14);	     // Disable memory to memory transfer on DMA1 channel 2 para q no se incrementen los dos punteros (el del dispositivo es fijo)(EL AND IGUAL ES PARA PONER UN 0 EN UNA POSICION DETERMINADA, EN ESTE CASO UN 1 NEGADO EN POS 14 Y UN 1 EN EL RESTO)
	DMA1->CHN[CHN2].CCR |=  (0b11 << 12);    // Set DMA priority to very high. Con esto se decide cual tiene prioridad de paso
	DMA1->CHN[CHN2].CCR |=  (0b10 << 10);    // Set memory transfer size to 32-bits. Configura el tamaño de transferencia de memoria para una operación de DMA a 32 bits, ajustando ciertos bits en el registro CCR de la estructura de datos seleccionada.
	DMA1->CHN[CHN2].CCR |=  (0b10 << 8);	   // Set peripheral transfer size to 32-bits. Este código ajusta ciertos bits en el registro CCR de la estructura de datos seleccionada, posiblemente para configurar una característica específica relacionada con la transferencia de datos a través de DMA. La operación de bits aquí está diseñada para establecer valores específicos en esos bits sin cambiar los demás.
	DMA1->CHN[CHN2].CCR |=  (1 << 7);		     // Enable memory increment mode
	DMA1->CHN[CHN2].CCR &= ~(1 << 6);		     // Disable peripheral increment mode
	DMA1->CHN[CHN2].CCR |=  (1 << 5);		     // Enable circular mode. Vuelve a empezar
	DMA1->CHN[CHN2].CCR |=  (1 << 4);		     // Read from memory
	DMA1->CHN[CHN2].CCR |=  (1 << 2);		     // Enable half transfer completed interrupt. Habilito la interrupcion a la mitad del buffer para que el procesador sepa que la primera mitad ya se encuentra vacia y se copie mas info
	DMA1->CHN[CHN2].CCR |=  (1 << 1);		     // Enable transfer completed interrupt. Habilito la interrupcion al final del buffer para lo mismo que antes.
	ENA_IRQ(IRQ_DMA1CHN2);									 // Enable DMA1 Channel2 inturrupt on NVIC

	DMA1->CHN[CHN2].CCR |= (1 << 0);		      // Enable DMA. 
	
	ENA_IRQ(IRQ_TIM2);										    // Enable TIM2 interrupt on NVIC
	TIM2->CR1   = 0x0000;					            // Reset CR1 just in case
    //	TIM2->CR1  |= (1 << 4);							// Down counter mode
	TIM2->PSC   = 72e6/(N*8*1e3)-1;	          // VELOCIDAD DE ENVIO DE DATOS fCK_PSC / (PSC[15:0] + 1)
	TIM2->ARR   = 8-1;					              // AUTO RELOAD. si lo pongo en 1 transfiero todo el tiempo
	TIM2->DIER |= (1 << 14);				          // Trigger DMA request enable. El timer le dice al DMA que transmita cuando el timer termine de contar.
	TIM2->DIER |= (1 <<  8);				          // Update DMA request enable
    //	TIM2->DIER |= (1 <<  6);				    // Enable interrupt
    //	TIM2->DIER |= (1 <<  0);				    // Update interrupt enable

	TIM2->CR1  |= (1 << 0);				            // Finally enable TIM1 module
};

int main(void){   
	unsigned int data[N];

	SetClock(); 
  ConfigDMA(data);

	for (volatile int i = 0; i < N; i++){
		data[i] = (i%2 == 0) ? (0x00000000) : (0x0000FFFF);
  }

	for (;;){}

  return 0; 
};

void handler_dma1chn2(void){
	DMA1->IFCR |= (0xf << 1);
	CLR_IRQ(IRQ_DMA1CHN2);
};

void handler_tim2(void){
	TIM2->SR &= ~(1 << 0);
	CLR_IRQ(IRQ_TIM2);
};

void sigma_delta(int *wavein, int *waveout){
    int integrator = wavein[0];

    for (int i = 0; i < N; i++) {
        if (i > 0) {integrator += (wavein[i] - waveout[i - 1] * 127);}
        if (integrator > 0) {waveout[i] = 1;} 
        else {waveout[i] = -1;}
    }
};
