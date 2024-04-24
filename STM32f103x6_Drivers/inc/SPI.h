/**
 ******************************************************************************
 * @file           : SPI.h
 * @author         : Mohamed Yahya
 * @brief          : SPI_functions
 ******************************************************************************
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

/* ================================================================ */
/* =========================== Includes =========================== */
/* ================================================================ */
#include "STM32f103x6.h"
#include "GPIO.h"

/* ================================================================ */
/* ============= User type definitions (structures) =============== */
/* ================================================================ */

typedef struct S_IRQ_SRC
{
	uint8_t		TXE:1;						// TX Buffer Empty Interrupt
	uint8_t		RXNE:1;						// RX Buffer Not Empty Interrupt
	uint8_t		ERRI:1;						// Error Interrupt
	uint8_t		Reserved:5;
}S_IRQ_SRC_t;

//configuration structure
typedef struct {
	uint16_t       Device_Mode ;    						//@ref SPI_Device_Mode
	uint16_t       Communication ;  						//@ref SPI_Communication
	uint16_t       Frame_Format ;  							//@ref SPI_Frame_Format
	uint16_t       Data_Size ;      						//@ref SPI_Data_Size
	uint16_t       CLK_Polarity ;   						//@ref SPI_CLK_Polarity
	uint16_t       CLK_Phase ;      						//@ref SPI_CLK_Phase
	uint16_t       NSS ;            						//@ref SPI_NSS
	uint16_t       SPI_Clock_Frequency ; 					//@ref SPI_Clock_Frequency
	uint16_t	   IRQ_Enable;								//@ref IRQ_ENABLE_DEFINE
	void(*P_IRQ_Call_Back)(S_IRQ_SRC_t *IRQ_src);			//set the c function which will be called once the IRQ Happen
}SPI_Config_t;


/* ================================================================ */
/* =============== Macros Configuration References ================ */
/* ================================================================ */

//@ref SPI_Device_Mode
#define SPI_Device_Mode_Slave   			(uint32_t)(0)
#define SPI_Device_Mode_Master  			(uint32_t)(1<<2)

//@ref SPI_Communication
#define SPI_Direction_2Lines 				(uint32_t)(0)
#define SPI_Direction_2Lines_RXONLY 		(uint32_t)(1<<10)
#define SPI_Direction_1Lines_Recieve_Only 	(uint32_t)(1<<15)
#define SPI_Direction_1Lines_Transmit_Only 	(uint32_t) 1<<10 | 1<<14)

//@ref SPI_Frame_Format
#define SPI_Frame_Fomate_MSP_Transmitted_First (uint32_t)(0)
#define SPI_Frame_Fomate_LSP_Transmitted_First (uint32_t)(1<<7)

//@ref SPI_Data_Size
#define SPI_Data_Size_8BIT  (uint32_t)(0)
#define SPI_Data_Size_16BIT  (uint32_t)(1<<11)

//@ref SPI_CLK_Polarity
#define SPI_CLK_Polarity_LOW_Idle  (uint32_t)(0)
#define SPI_CLK_Polarity_HIGH_Idle  (uint32_t)(1<<1)

//@ref SPI_CLK_Phase
#define SPI_CLK_Phase_First_Edge (uint32_t)(0)
#define SPI_CLK_Phase_Second_Edge (uint32_t)(1<<0)

//@ref SPI_NSS
/*
 *Software NSS management (SSM = 1)The slave select (SSI bit in the SPI_CR1 register)
 *Hardware NSS management (SSM = 0)
 *NSS output enabled (SSM = 0, SSOE = 1)
 *NSS output disabled (SSM = 0, SSOE = 0)
 *
 */

	/* Hardware (SPI_CR2 Register) */
#define SPI_NSS_HW_SLAVE							(uint16_t) (0)
#define SPI_NSS_HW_MASTER_SS_OUTPUT_ENABLED			(uint16_t) (1<<2)	/* Bit 2 SSOE: SS output enable -> 1: SS output is enabled in master mode and when the cell is enabled. 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	  	  	  	  	  	  	  	  The cell cannot work in a multimaster environment. */
#define SPI_NSS_HW_MASTER_SS_OUTPUT_DISABLED		(uint16_t) ~(1<<2)

/* Software (NSS is driven by Software for "Master or Slave" )*/
#define SPI_NSS_SW_SSI_SET							(uint16_t) ((1<<9) | (1<<8))

//@ref SPI_Clock_Frequency
#define SPI_Boudrate_Prescaler_2  	(uint32_t)(0)
#define SPI_Boudrate_Prescaler_4  	(uint32_t)(0b001<<3)
#define SPI_Boudrate_Prescaler_8  	(uint32_t)(0b010<<3)
#define SPI_Boudrate_Prescaler_16   (uint32_t)(0b011<<3)
#define SPI_Boudrate_Prescaler_32 	(uint32_t)(0b100<<3)
#define SPI_Boudrate_Prescaler_64  	(uint32_t)(0b101<<3)
#define SPI_Boudrate_Prescaler_128  (uint32_t)(0b110<<3)
#define SPI_Boudrate_Prescaler_256  (uint32_t)(0b111<<3)

//@SPI_IRQ_Enable_Define
#define SPI_IRQ_Enable_None (uint32_t)(0)
#define SPI_IRQ_Enable_TXEIE (uint32_t)(1<<7)
#define SPI_IRQ_Enable_RXEIE (uint32_t)(1<<6)
#define SPI_IRQ_Enable_ERRIE (uint32_t)(1<<5)


enum Polling_Mechanism
{
	SPI_disable,
	SPI_enable
};

/* ================================================================ */
/* =========== APIs Supported by "MCAL SPI DRIVER" ================ */
/* ================================================================ */


void MCAL_SPI_Init(SPI_Typedef_t *SPIx, SPI_Config_t *Config);
void MCAL_SPI_DeInit(SPI_Typedef_t *SPIx);

void MCAL_SPI_GPIO_Set_Pins(SPI_Typedef_t *SPIx);

void MCAL_SPI_Send_Data(SPI_Typedef_t *SPIx, uint16_t *pTxBuffer, enum Polling_Mechanism Polling_En);
void MCAL_SPI_Recieve_Data(SPI_Typedef_t *SPIx, uint16_t *pRxBuffer, enum Polling_Mechanism Polling_En);

void MCAL_SPI_TX_RX(SPI_Typedef_t *SPIx, uint16_t *pTxBuffer, enum Polling_Mechanism Polling_En);


#endif /* INC_SPI_H_ */
