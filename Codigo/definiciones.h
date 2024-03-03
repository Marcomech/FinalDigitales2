
typedef __INT8_TYPE__ int8_t;
typedef __INT16_TYPE__ int16_t;
typedef __INT32_TYPE__ int32_t;
typedef __INT64_TYPE__ int64_t;
typedef __UINT8_TYPE__ uint8_t;
typedef __UINT16_TYPE__ uint16_t;
typedef __UINT32_TYPE__ uint32_t;
typedef __UINT64_TYPE__ uint64_t;

#define SRAM_SIZE ((uint32_t)0x00005000)
#define SRAM_BASE ((uint32_t)0x20000000)
#define STACKINIT ((interrupt_t)(SRAM_BASE + SRAM_SIZE))

#define ENA_IRQ(IRQ)                                                          \
	{                                                                         \
		NVIC->ISER[((uint32_t)(IRQ) >> 5)] = (1 << ((uint32_t)(IRQ) & 0x1F)); \
	}
#define DIS_IRQ(IRQ)                                                          \
	{                                                                         \
		NVIC->ICER[((uint32_t)(IRQ) >> 5)] = (1 << ((uint32_t)(IRQ) & 0x1F)); \
	}
#define CLR_IRQ(IRQ)                                                          \
	{                                                                         \
		NVIC->ICPR[((uint32_t)(IRQ) >> 5)] = (1 << ((uint32_t)(IRQ) & 0x1F)); \
	}

int main(void);

typedef void (*interrupt_t)(void);
const interrupt_t vector_table[256] __attribute__((section(".vtab"))) = {
	STACKINIT,		   // 0x0000_0000 Stack Pointer
	(interrupt_t)main, // 0x0000_0004 Reset
	0,				   // 0x0000_0008
	0,				   // 0x0000_000C
	0,				   // 0x0000_0010
	0,				   // 0x0000_0014
	0,				   // 0x0000_0018
	0,				   // 0x0000_001C
	0,				   // 0x0000_0020
	0,				   // 0x0000_0024
	0,				   // 0x0000_0028
	0,				   // 0x0000_002C
	0,				   // 0x0000_0030
	0,				   // 0x0000_0034
	0,				   // 0x0000_0038
	0,				   //(interrupt_t) handler_systick,                          // 0x0000_003C handler_systick
	0,				   // 0x0000_0040
	0,				   // 0x0000_0044
	0,				   // 0x0000_0048
	0,				   // 0x0000_004C
	0,				   // 0x0000_0050
	0,				   // 0x0000_0054
	0,				   // 0x0000_0058
	0,				   // 0x0000_005C
	0,				   // 0x0000_0060
	0,				   // 0x0000_0064
	0,				   // 0x0000_0068
	0,				   // 0x0000_006C
	0,				   // handler_dma1chn2,  // 0x0000_0070 DMA1_CHN2
	0,				   // 0x0000_0074
	0,				   // 0x0000_0078
	0,				   // 0x0000_007C
	0,				   // 0x0000_0080
	0,				   // 0x0000_0084
	0,				   // 0x0000_0088
	0,				   // 0x0000_008C
	0,				   // 0x0000_0090
	0,				   // 0x0000_0094
	0,				   // 0x0000_0098
	0,				   // 0x0000_009C
	0,				   // 0x0000_00A0
	0,				   // 0x0000_00A4
	0,				   // 0x0000_00A8
	0,				   // 0x0000_00AC
	0,				   // handler_tim2,	   // 0x0000_00B0 TIM2
};
