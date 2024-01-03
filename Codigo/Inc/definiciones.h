typedef int						int32_t;
typedef short					int16_t;
typedef char					 int8_t;
typedef unsigned int	 uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char	uint8_t;

#define SRAM_SIZE ((uint32_t) 0x00005000)
#define SRAM_BASE ((uint32_t) 0x20000000)
#define STACKINIT ((interrupt_t)(SRAM_BASE+SRAM_SIZE))

#define ENA_IRQ(IRQ) {NVIC->ISER[((uint32_t)(IRQ) >> 5)] = (1 << ((uint32_t)(IRQ) & 0x1F));}
#define DIS_IRQ(IRQ) {NVIC->ICER[((uint32_t)(IRQ) >> 5)] = (1 << ((uint32_t)(IRQ) & 0x1F));}
#define CLR_IRQ(IRQ) {NVIC->ICPR[((uint32_t)(IRQ) >> 5)] = (1 << ((uint32_t)(IRQ) & 0x1F));}


int	main(void);
void handler_dma1chn2(void);
void handler_tim2(void);

typedef void(*interrupt_t)(void);
const interrupt_t vector_table[256] __attribute__ ((section(".vtab"))) = {
	STACKINIT,									// 0x0000_0000 Stack Pointer
	(interrupt_t) main,					// 0x0000_0004 Reset									
	0,													// 0x0000_0008
	0,													// 0x0000_000C
	0,													// 0x0000_0010
	0,													// 0x0000_0014
	0,													// 0x0000_0018
	0,													// 0x0000_001C
	0,													// 0x0000_0020
	0,													// 0x0000_0024
	0,													// 0x0000_0028
	0,													// 0x0000_002C
	0,													// 0x0000_0030
	0,													// 0x0000_0034
	0,													// 0x0000_0038
	0,													// 0x0000_003C handler_systick
	0,													// 0x0000_0040
	0,													// 0x0000_0044
	0,													// 0x0000_0048
	0,													// 0x0000_004C
	0,													// 0x0000_0050
	0,													// 0x0000_0054
	0,													// 0x0000_0058
	0,													// 0x0000_005C
	0,													// 0x0000_0060
	0,													// 0x0000_0064
	0,													// 0x0000_0068
	0,													// 0x0000_006C
	handler_dma1chn2,						// 0x0000_0070 DMA1_CHN2
	0,													// 0x0000_0074
	0,													// 0x0000_0078
	0,													// 0x0000_007C
	0,													// 0x0000_0080
	0,													// 0x0000_0084
	0,													// 0x0000_0088
	0,													// 0x0000_008C
	0,													// 0x0000_0090
	0,													// 0x0000_0094
	0,													// 0x0000_0098
	0,													// 0x0000_009C
	0,													// 0x0000_00A0
	0,													// 0x0000_00A4
	0,													// 0x0000_00A8
	0,													// 0x0000_00AC
	handler_tim2,								// 0x0000_00B0 TIM2
};

enum IRQs {
	IRQ_DMA1CHN2	= 12,
	IRQ_ADC1_2		= 18,
	IRQ_TIM2			= 28,
	IRQ_USART1		= 37,
};

struct NVIC {
	int	ISER[8];
	int	RES0[24];
	int	ICER[8];
	int	RES1[24];
	int	ISPR[8];
	int	RES2[24];
	int	ICPR[8];
	int	RES3[24];
	int	IABR[8];
	int	RES4[56];
	char IPR[240];
	int	RES5[644];
	int	STIR;
};
volatile struct NVIC *const NVIC = (struct NVIC *)(0xE000E100);

struct DMAs{
	int ISR;
	int IFCR;
	struct {
		int CCR;
		int CNDTR;
		int CPAR;
		int CMAR;
		int RESERVED;
	} CHN[8];
};		 
enum {CHN1	= 0, CHN2	= 1, CHN3	= 2, CHN4	= 3, CHN5	= 4, CHN6	= 5, CHN7 = 6, CHN8 = 7 };

volatile struct DMAs *const DMA1 = (struct DMAs *)(0x40020000);
volatile struct DMAs *const DMA2 = (struct DMAs *)(0x40020400);

struct TIMs{
	int CR1;
	int CR2;
	int SMCR;
	int DIER;
	int SR;
	int EGR;
	int CCMR1;
	int CCMR2;
	int CCER;
	int CNT;
	int PSC;
	int ARR;
	int RES1;
	int CCR1;
	int CCR2;
	int CCR3;
	int CCR4;
	int BDTR;
	int DCR;
	int DMAR;	
};
volatile struct TIMs *const TIM2 = (struct TIMs *)(0x40000000);
volatile struct TIMs *const TIM3 = (struct TIMs *)(0x40000400);
volatile struct TIMs *const TIM4 = (struct TIMs *)(0x40000800);

struct FLASH{
	int ACR;
	int KEYR;
	int OPTKEYR;
	int SR;
	int CR;
	int AR;
	int reserved;
	int OBR;
	int WRPR;
};
volatile struct FLASH *const FLASH = (struct FLASH *)(0x40022000);


struct RCC {
		int CR;
		int CFGR;
		int CIR;
		int APB2RSTR;
		int APB1RSTR;
		int AHBENR;
		int APB2ENR;
		int APB1ENR;
		int BDCR;
		int CSR;
		int AHBRSTR;
		int CFGR2;
};
volatile struct RCC *const RCC = (struct RCC *)(0x40021000);

struct GPIOx {
		int CRL;
		int CRH;
		int IN ;
		int OUT;
};

struct GPIOx volatile *const GPIOA = (struct GPIOx *)(0x40010800);
struct GPIOx volatile *const GPIOB = (struct GPIOx *)(0x40010C00);
struct GPIOx volatile *const GPIOC = (struct GPIOx *)(0x40011000);

struct SysTick{
		int CSR;	 //Para habilitar y leer
		// 0 Enable | 1 Interrupciones | 2 Clock (0= 1Megas o 1=8Megas) | 16 Flag

		int ReloadValue;	 //Reload, valor en el que empieza el contador
		int CurrentValue;	//Current Value
		int Calibracion;	 //Calibracion, da 1 cada 0.01seg
};

struct SysTick volatile *const SYST = (struct SysTick *)(0xE000E010);

