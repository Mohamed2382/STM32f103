/**
 ******************************************************************************
 * @file           : STM32f103x6.h
 * @author         : Mohamed Yahya
 ******************************************************************************
 */

#ifndef INC_STM32F103X6_H_
#define INC_STM32F103X6_H_

//-----------------------------
//Includes
//-----------------------------

#include "stdlib.h"
#include <stdint.h>

//-----------------------------
//Base addresses for Memories
//-----------------------------

#define FLASH_Memory_BASE 							0x08000000UL
#define System_Memory_BASE 							0x1FFFF000UL
#define SRAM_Memory_BASE 							0x20000000UL


#define Peripherals_BASE 							0x40000000UL

#define Cortex_M3_Internal_Peripherals_BASE 		0xE0000000UL


//-----------------------------
//Base addresses for AHB Peripherals
//-----------------------------
//RCC
#define RCC_BASE 									0x40021000UL
//#define RCC_BASE              				(Peripherals_BASE + 0x00021000UL)
//CRC
#define CRC_BASE 									0x40023400UL
//flash interface
#define FLASH_INTERFACE_BASE 						0x40022000UL
//DMA
#define DMA_BASE 									0x40020000UL








//-----------------------------
//Base addresses for APB2 Peripherals
//-----------------------------


//GPIO
//A,B fully included in LQFP48 Package
#define GPIOA_BASE 							0x40010800UL
#define GPIOB_BASE 							0x40010C00UL


//C,D Partial  included in LQFP48 Package
#define GPIOC_BASE 							0x40011000UL
#define GPIOD_BASE 							0x40011400UL


//E not  included in LQFP48 Package
#define GPIOE_BASE 							0x40011800UL

//EXTI
#define EXTI_BASE 							0x40010400UL

//AFIO
#define AFIO_BASE 							0x40010000UL

#define TIM11_BASE 							0x40015400UL
#define TIM10_BASE 							0x40015000UL
#define TIM9_BASE 							0x40014C00 UL
#define ADC3_BASE 							0x40013C00UL
#define USART1_BASE 						0x40013800UL
#define TIM8_BASE 							0x40013400 UL
#define SPI1_BASE 							0x40013000UL
#define TIMER1_BASE 						0x40012C00UL
#define ADC2_BASE 							0x40012800UL
#define ADC1_BASE 							0x40012400UL



//-----------------------------
//Base addresses for APB1 Peripherals
//-----------------------------
#define I2C2_BASE 							0x40005800UL
#define I2C1_BASE 							0x40005400UL
#define UART5_BASE 							0x40005000UL
#define UART4_BASE 							0x40004C00UL
#define USART3_BASE 						0x40004800UL
#define USART2_BASE 						0x40004400UL
#define TIMER3_BASE 						0x40000400UL
#define TIMER2_BASE 						0x40000000UL
#define SPI2_BASE 							0x40003800UL
#define TIM4_Base   0x40000800UL



//======================================================================

//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
//Peripheral register
//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*

//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: GPIO
//-*-*-*-*-*-*-*-*-*-*-*
typedef struct
{
	volatile uint32_t  CRL ;
	volatile uint32_t  CRH ;
	volatile uint32_t  IDR ;
	volatile uint32_t  ODR ;
	volatile uint32_t  BSRR ;
	volatile uint32_t  BRR ;
	volatile uint32_t  LCKR ;
}GPIO_TypeDef;





//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: RCC
//-*-*-*-*-*-*-*-*-*-*-*

typedef struct
{
	volatile uint32_t  CR ;
	volatile uint32_t  CFGR ;
	volatile uint32_t CIR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t APB1RSTR;
	volatile uint32_t AHBENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t APB1ENR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
}RCC_TypeDef;


//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: EXTI
//-*-*-*-*-*-*-*-*-*-*-*

typedef struct
{
	volatile uint32_t  IMR ;
	volatile uint32_t  EMR ;
	volatile uint32_t  RTSR ;
	volatile uint32_t  FTSR ;
	volatile uint32_t  SWIER ;
	volatile uint32_t  PR ;

}EXTI_TypeDef;


//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: AFIO
//-*-*-*-*-*-*-*-*-*-*-*
typedef struct
{
	volatile uint32_t  EVCR ;
	volatile uint32_t  MAPR ;
	volatile uint32_t  EXTICR[4] ;
	uint32_t  		   RESERVED0 ; //0x18
	volatile uint32_t  MAPR2 ; // 0x1c


}AFIO_TypeDef;



//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: USART
//-*-*-*-*-*-*-*-*-*-*-*
typedef struct
{
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;
} USART_TypeDef;


//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: SPI
//-*-*-*-*-*-*-*-*-*-*-*
typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
} SPI_Typedef_t;

//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: I2C
//-*-*-*-*-*-*-*-*-*-*-*

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;

} I2C_TypeDef;



//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: TIMER
//-*-*-*-*-*-*-*-*-*-*-*

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMCR;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	volatile uint32_t CCMR1;
	volatile uint32_t CCMR2;
	volatile uint32_t CCER;
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
	volatile uint32_t RCR;
	volatile uint32_t CCR1;
	volatile uint32_t CCR2;
	volatile uint32_t CCR3;
	volatile uint32_t CCR4;
	volatile uint32_t BDTR;
	volatile uint32_t DCR;
	volatile uint32_t DMAR;
} TIMERS_typeDef;


//==============================================================


//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants:
//-*-*-*-*-*-*-*-*-*-*-*
#define GPIOA  					((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB      				((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC      				((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD      				((GPIO_TypeDef *)GPIOD_BASE)
#define GPIOE      				((GPIO_TypeDef *)GPIOE_BASE)

#define RCC      				((RCC_TypeDef *)RCC_BASE)

#define EXTI      				((EXTI_TypeDef *)EXTI_BASE)

#define AFIO      				((AFIO_TypeDef *)AFIO_BASE)

#define USART1                	((USART_TypeDef *)USART1_BASE)
#define USART2                	((USART_TypeDef *)USART2_BASE)
#define USART3                	((USART_TypeDef *)USART3_BASE)

#define SPI1                	((SPI_Typedef_t *)SPI1_BASE)
#define SPI2                	((SPI_Typedef_t *)SPI2_BASE)


#define I2C1                	((I2C_TypeDef *)I2C1_BASE)
#define I2C2                	((I2C_TypeDef *)I2C2_BASE)

#define TIM1                	((TIMERS_typeDef *)TIMER1_BASE)
#define TIM2                	((TIMERS_typeDef *)TIMER2_BASE)
#define TIM3                	((TIMERS_typeDef *)TIMER3_BASE)
#define TIM4   					((TIMERS_typeDef*) TIM4_Base)



//==============================================================

//-*-*-*-*-*-*-*-*-*-*-*-
//clock enable Macros:
//-*-*-*-*-*-*-*-*-*-*-*

#define RCC_GPIOA_CLK_EN()	(RCC->APB2ENR |= 1<<2)
#define RCC_GPIOB_CLK_EN()	(RCC->APB2ENR |= 1<<3)
#define RCC_GPIOC_CLK_EN()	(RCC->APB2ENR |= 1<<4)
#define RCC_GPIOD_CLK_EN()	(RCC->APB2ENR |= 1<<5)
#define RCC_GPIOE_CLK_EN()	(RCC->APB2ENR |= 1<<6)

#define RCC_AFIO_GPIOE_CLK_EN()	(RCC->APB2ENR |= 1<<0)

//RCC_Enable
#define RCC_USART1_CLK_EN()	( RCC->APB2ENR |= (1<<14) )
#define RCC_USART2_CLK_EN()	( RCC->APB1ENR |= (1<<17) )
#define RCC_USART3_CLK_EN()	( RCC->APB1ENR |= (1<<18) )


//RCC_Reset
#define RCC_USART1_Reset()	( RCC->APB2RSTR &= ~(1<<14) )
#define RCC_USART2_Reset()	( RCC->APB1RSTR &= ~(1<<17) )
#define RCC_USART3_Reset()	( RCC->APB1RSTR &= ~(1<<18) )

//SPI_Enable
#define RCC_SPI1_CLK_EN()	( RCC->APB2ENR |= (1<<12) )
#define RCC_SPI2_CLK_EN()	( RCC->APB1ENR |= (1<<14) )

//SPI_Reset
#define RCC_SPI1_Reset()	( RCC->APB2RSTR &= ~(1<<12) )
#define RCC_SPI2_Reset()	( RCC->APB1RSTR &= ~(1<<14) )

//I2C_Enable
#define RCC_I2C1_CLK_EN()	(RCC->APB1ENR |= 1<<21)
#define RCC_I2C2_CLK_EN()	(RCC->APB1ENR |= 1<<22)

//I2C_Reset
#define RCC_I2C1_Reset()	(RCC->APB1RSTR &= ~(1<<21))
#define RCC_I2C2_Reset()	(RCC->APB1RSTR &= ~(1<<22))

//TIMER_Enable
#define RCC_TIMER1_CLK_EN()	(RCC->APB2ENR |= 1<<11)
#define RCC_TIMER2_CLK_EN()	(RCC->APB1ENR |= 1<<0)
#define RCC_TIMER3_CLK_EN()	(RCC->APB1ENR |= 1<<1)

//TIMER_Reset
#define RCC_TIMER1_Reset()	(RCC->APB2RSTR &= ~(1<<11))
#define RCC_TIMER2_Reset()	(RCC->APB1RSTR &= ~(1<<0))
#define RCC_TIMER3_Reset()	(RCC->APB1RSTR &= ~(1<<1))


//-*-*-*-*-*-*-*-*-*-*-*-
//Interrupt Vector Table:
//-*-*-*-*-*-*-*-*-*-*-*
/*
 * EXTI
 */
#define EXTI0_IRQ		6   // 6 number of interrupt request send to cpu
#define EXTI1_IRQ		7
#define EXTI2_IRQ		8
#define EXTI3_IRQ		9
#define EXTI4_IRQ		10
#define EXTI5_IRQ		23
#define EXTI6_IRQ		23
#define EXTI7_IRQ		23
#define EXTI8_IRQ		23
#define EXTI9_IRQ		23
#define EXTI10_IRQ		40
#define EXTI11_IRQ		40
#define EXTI12_IRQ		40
#define EXTI13_IRQ		40
#define EXTI14_IRQ		40
#define EXTI15_IRQ		40

#define USART1_IRQ		37
#define	USART2_IRQ		38
#define	USART3_IRQ		39

#define SPI1_IRQ		35
#define	SPI2_IRQ		36

#define I2C1_EV_IRQ		31
#define I2C1_ER_IRQ		32
#define I2C2_EV_IRQ		33
#define I2C2_ER_IRQ		34

#define TIMER1_BRK_IRQ  24
#define TIMER1_UP_IRQ  25
#define TIMER1_TRG_COM_IRQ  26
#define TIMER1_CC_IRQ  27
#define TIMER2_IRQ  28
#define TIMER3_IRQ  29



// NVIC ( Non-Vectored Interrupt Controller )
#define NVIC_BASE_ADDRESS	0xE000E100UL
#define NVIC_ISER0			*(volatile uint32_t *)(NVIC_BASE_ADDRESS + 0x00)
#define NVIC_ISER1			*(volatile uint32_t *)(NVIC_BASE_ADDRESS + 0x04)
#define NVIC_ISER2			*(volatile uint32_t *)(NVIC_BASE_ADDRESS + 0x08)
#define NVIC_ICER0			*(volatile uint32_t *)(NVIC_BASE_ADDRESS + 0x80)
#define NVIC_ICER1			*(volatile uint32_t *)(NVIC_BASE_ADDRESS + 0x84)
#define NVIC_ICER2			*(volatile uint32_t *)(NVIC_BASE_ADDRESS + 0x88)


//-*-*-*-*-*-*-*-*-*-*-*--*-*-*--*-*-*-
//NVIC IRQ Enable/Disable Macros
//-*-*-*-*-*-*-*-*-*-*-*-*-*-*--*-*-*-

/*  Enable Interrupt Requests  */

// We Started From IRQ6 Due To DataSheet
// ISER : Interrupt Set Register

#define NVIC_IRQ6_EXTI0_EN()			(NVIC_ISER0 |= 1<<6)				// EXTI0 ---> PIN6 in NVIC
#define NVIC_IRQ7_EXTI1_EN()			(NVIC_ISER0 |= 1<<7)				// EXTI1 ---> PIN7 in NVIC
#define NVIC_IRQ8_EXTI2_EN()			(NVIC_ISER0 |= 1<<8)				// EXTI2 ---> PIN8 in NVIC
#define NVIC_IRQ9_EXTI3_EN()			(NVIC_ISER0 |= 1<<9)				// EXTI3 ---> PIN9 in NVIC
#define NVIC_IRQ10_EXTI4_EN()			(NVIC_ISER0 |= 1<<10)				// EXTI4 ---> PIN10 in NVIC
#define NVIC_IRQ23_EXTI5_9_EN()			(NVIC_ISER0 |= 1<<23)				// EXTI5, EXTI6, EXTI7, EXTI8, EXTI9 ---> PIN23 in NVIC

// 40 - 32 = 8
#define NVIC_IRQ40_EXTI10_15_EN()		(NVIC_ISER1 |= 1<<8)				// EXTI10, EXTI11, EXTI12, EXTI13, EXTI14, EXTI15


/* Disable Interrupt Requests */

// We Started From IRQ6 Due To DataSheet
// ICER : Interrupt Clear Register

#define NVIC_IRQ6_EXTI0_DI()			(NVIC_ICER0 |= 1<<6)				// EXTI0 ---> PIN6 in NVIC
#define NVIC_IRQ7_EXTI1_DI()			(NVIC_ICER0 |= 1<<7)				// EXTI1 ---> PIN7 in NVIC
#define NVIC_IRQ8_EXTI2_DI()			(NVIC_ICER0 |= 1<<8)				// EXTI2 ---> PIN8 in NVIC
#define NVIC_IRQ9_EXTI3_DI()			(NVIC_ICER0 |= 1<<9)				// EXTI3 ---> PIN9 in NVIC
#define NVIC_IRQ10_EXTI4_DI()			(NVIC_ICER0 |= 1<<10)				// EXTI4 ---> PIN10 in NVIC
#define NVIC_IRQ23_EXTI5_9_DI()			(NVIC_ICER0 |= 1<<23)				// EXTI5, EXTI6, EXTI7, EXTI8, EXTI9 ---> PIN23 in NVIC

// 40 - 32 = 8
#define NVIC_IRQ40_EXTI10_15_DI()		(NVIC_ICER1 |= 1<<8)				// EXTI10, EXTI11, EXTI12, EXTI13, EXTI14, EXTI15
//  ---> PIN40 in NVIC
//USART
#define NVIC_IRQ37_USART1_Enable()   	(NVIC_ISER1 |= 1<<( USART1_IRQ - 32 )) //IRQ-32
#define NVIC_IRQ38_USART2_Enable()   	(NVIC_ISER1 |= 1<<( USART2_IRQ - 32 )) //IRQ-32
#define NVIC_IRQ39_USART3_Enable()   	(NVIC_ISER1 |= 1<<( USART3_IRQ - 32 )) //IRQ-32

#define NVIC_IRQ37_USART1_Disable()   	(NVIC_ICER1 |= 1<<( USART1_IRQ- 32 )) //IRQ-32
#define NVIC_IRQ38_USART2_Disable()   	(NVIC_ICER1 |= 1<<( USART2_IRQ- 32 )) //IRQ-32
#define NVIC_IRQ39_USART3_Disable()   	(NVIC_ICER1 |= 1<<( USART3_IRQ- 32 )) //IRQ-32

//SPI

#define NVIC_IRQ35_SPI1_Enable()   	(NVIC_ISER1 |= 1<<( SPI1_IRQ - 32 )) //IRQ-32
#define NVIC_IRQ36_SPI2_Enable()   	(NVIC_ISER1 |= 1<<( SPI1_IRQ - 32 )) //IRQ-32

#define NVIC_IRQ35_SPI1_Disable()   	(NVIC_ICER1 |= 1<<( SPI1_IRQ- 32 )) //IRQ-32
#define NVIC_IRQ36_SPI2_Disable()   	(NVIC_ICER1 |= 1<<( SPI1_IRQ- 32 )) //IRQ-32

//I2C
#define NVIC_IRQ31_I2C1_EV_Enable   	(NVIC_ISER0 |= 1<<( I2C1_EV_IRQ )) //NVIC_ISER0
#define NVIC_IRQ32_I2C1_ER_Enable   	(NVIC_ISER1 |= 1<<( I2C1_ER_IRQ - 32 )) //NVIC_ISER1 32-32
#define NVIC_IRQ33_I2C2_EV_Enable   	(NVIC_ISER1 |= 1<<( I2C2_EV_IRQ - 32 )) //NVIC_ISER1 33-32
#define NVIC_IRQ34_I2C2_ER_Enable   	(NVIC_ISER1 |= 1<<( I2C2_ER_IRQ - 32 )) //NVIC_ISER1 34-32


#define NVIC_IRQ31_I2C1_EV_Disable   	(NVIC_ICER0 |= 1<<( I2C1_EV_IRQ )) //NVIC_ICER1 //31
#define NVIC_IRQ32_I2C1_ER_Disable   	(NVIC_ICER1 |= 1<<( I2C1_ER_IRQ - 32 )) //NVIC_ICER1 32-32
#define NVIC_IRQ33_I2C2_EV_Disable   	(NVIC_ICER1 |= 1<<( I2C2_EV_IRQ - 32 )) //NVIC_ICER1 33-32
#define NVIC_IRQ34_I2C2_ER_Disable   	(NVIC_ICER1 |= 1<<( I2C2_ER_IRQ - 32 )) //NVIC_ICER1 34-32


//TIMER
#define NVIC_IRQ24_TIMER1_BRK_Enable()			(NVIC_ISER0 |= 1<<24)
#define NVIC_IRQ24_TIMER1_UP_Enable()			(NVIC_ISER0 |= 1<<25)
#define NVIC_IRQ24_TIMER1_TRG_COM_Enable()		(NVIC_ISER0 |= 1<<26)
#define NVIC_IRQ24_TIMER1_CC_Enable()			(NVIC_ISER0 |= 1<<27)
#define NVIC_IRQ24_TIMER2_BRK_Enable()			(NVIC_ISER0 |= 1<<28)
#define NVIC_IRQ24_TIMER3_BRK_Enable()			(NVIC_ISER0 |= 1<<29)
#define NVIC_IRQ24_TIMER4_BRK_Enable()			(NVIC_ISER0 |= 1<<30)


#define NVIC_IRQ24_TIMER1_BRK_Disable()			(NVIC_ICER0 |= 1<<24)
#define NVIC_IRQ24_TIMER1_UP_Disable()			(NVIC_ICER0 |= 1<<25)
#define NVIC_IRQ24_TIMER1_TRG_COM_Disable()		(NVIC_ICER0 |= 1<<26)
#define NVIC_IRQ24_TIMER1_CC_Disable()			(NVIC_ICER0 |= 1<<27)
#define NVIC_IRQ24_TIMER2_BRK_Disable()			(NVIC_ICER0 |= 1<<28)
#define NVIC_IRQ24_TIMER3_BRK_Disable()			(NVIC_ICER0 |= 1<<29)

#endif /* INC_STM32F103X6_H_ */
