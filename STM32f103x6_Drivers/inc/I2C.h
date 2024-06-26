/**
 ******************************************************************************
 * @file           : I2C.h
 * @author         : Mohamed Yahya
 * @brief          : I2C_functions
 ******************************************************************************
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include "STM32f103x6.h"
#include "GPIO.h"
#include "RCC.h"



typedef struct
{
	uint16_t 				Enable_Dual_ADD ;  //1-Enable  0-Disable
	uint16_t 				Primary_slave_address ;
	uint16_t 				Secondary_slave_address ;
	uint32_t 				I2C_Addressing_Slave_mode; //@I2C_Addressing_Slave_

}S_I2C_Slave_Device_Address;

//@Slave_Event_CallBack
typedef enum
{
	//page 770
	I2C_EV_STOP,
	I2C_ERROR_AF ,
	I2C_EV_ADDR_Matched,
	I2C_EV_DATA_REQ  ,//the APP layer should send the data (I2C_SlaveSendData ) in this state
	I2C_EV_DATA_RCV  //the APP layer should read the data (I2C_SlaveReceiveData ) in this state
} Slave_State;



typedef struct
{
	uint32_t									I2C_ClockSpeed;						// @I2C_SCLK

	uint32_t					 				StretchMode;						//@ref @I2C_StretchMode_


	uint32_t 									I2C_Mode;                			// @ref I2C_mode

	S_I2C_Slave_Device_Address			       I2C_Slave_Address;                  //@ref I2C_Slave_Address

	uint32_t 									I2C_ACK_Control;  					 // @ref I2C_Ack_


	uint32_t					 				General_Call_Address_Detection ;	//@ref I2C_ENGC


	void(* P_Slave_Event_CallBack)(Slave_State state);								//Set the C Function() which will be called once the IRQ  Happen

}I2C_InitTypeDef;


// @I2C_SCLK
//– Standard Speed (up to 100 kHz)
//– Fast Speed (up to 400 kHz)
#define I2C_SCLK_SM_50K  				(0x50000U)
#define I2C_SCLK_SM_100K  				(100000U)
#define I2C_SCLK_FM_200K  				(200000U) //Fast Mode Not Supported yet
#define I2C_SCLK_FM_400K  				(400000U) //Fast Mode Not Supported yet


//@ref @I2C_StretchMode_
//I2C_CR1
//Bit 7 NOSTRETCH: Clock stretching disable (Slave mode)
//0: Clock stretching enabled
//1: Clock stretching disabled

#define I2C_StretchMode_Enable                   0x00000000U
#define I2C_StretchMode_Disable                 (uint16_t)(1<<7)

//@ref I2C_mode
//I2C_CR1
//Bit 1 SMBUS: SMBus mode
//0: I2C mode
//1: SMBus mode

#define I2C_mode_I2C  					(uint16_t)(0)
#define I2C_mode_SMBus					(uint16_t)(1<<1)

//@ref I2C_Slave_Address
//I2C_OAR1.ADDMODE
//Bit 15 ADDMODE Addressing mode (slave mode)
//0: 7-bit slave address (10-bit address not acknowledged)
//1: 10-bit slave address (7-bit address not acknowledged)

#define I2C_Addressing_Slave_mode_7Bit                  0x00000000U
#define I2C_Addressing_Slave_mode_10Bit                 (uint16_t)(1<<15)

// @ref I2C_Ack_
//I2C_CR1
//Bit 10 ACK: Acknowledge enable
//0: No acknowledge returned
//1: Acknowledge returned after a byte is received (matched address or data)

#define I2C_Ack_Enable                  (uint16_t)(1<<10)
#define I2C_Ack_Disable                 (uint16_t)(0x0000)

//@ref I2C_ENGC
//I2C_CR1
//Bit 6 ENGC: General call enable
//0: General call disabled. Address 00h is NACKed.
//1: General call enabled. Address 00h is ACKed
#define I2C_ENGC_Enable                   (uint16_t)(1<<6)
#define I2C_ENGC_Disable                  0x00000000U



//=============================================================================//

typedef enum
{
	With_Stop,
	without_Stop
} Stop_Condition;


typedef enum
{
	Start,
	repeated_start
}
Repeated_Start;

typedef enum {DISABLE = 0, ENABLE = 1} FunctionalState;
typedef enum {RESET = 0, SET = 1} FlagStatus;

typedef enum {I2C_Direction_Transmitter = 0, I2C_Direction_Recieve = 1} I2C_Direction;

#define I2C_EVENT_MASTER_BYTE_TRANSMITTING                 ((uint32_t)0x00070080) /* TRA, BUSY, MSL, TXE flags */

typedef enum
{
	I2C_FLAG_BUSY=0,
	EV5, //EV5: SB=1, cleared by reading SR1 register followed by writing DR register with Address.
	EV6,  //EV6: ADDR=1, cleared by reading SR1 register followed by reading SR2.
	EV8, //EV8: TxE=1, shift register not empty, d . ata register empty, cleared by writing DR register
	EV8_1,
	EV7,
	MASTER_BYTE_TRANSMITTING = ((uint32_t)0x00070080) /* TRA, BUSY, MSL, TXE flags */

}Status;


#define I2C_OAR2_ENDUAL  				(uint16_t)(0x1<<0)
#define I2C_OAR2_ADD2_Pos				(uint16_t)(1)
#define I2C_CR2_ITBUFEN					(uint16_t)(0x1<<10)
#define I2C_CR2_ITEVTEN					(uint16_t)(0x1<<9)
#define I2C_CR2_ITERREN					(uint16_t)(0x1<<8)
#define I2C_CR1_PE						(uint16_t)(0x1<<0)

#define I2C_CR1_ACK						(uint16_t)(0x1<<10)
#define I2C_CR1_START					(uint16_t)(0x1<<8)
#define I2C_CR1_STOP					(uint16_t)(0x1<<9)
#define I2C_SR1_SB						(uint16_t)(0x1<<0)
#define I2C_SR1_ADDR					(uint16_t)(0x1<<1)
#define I2C_SR1_TXE						(uint16_t)(0x1<<7)
#define I2C_SR1_RXNE					(uint16_t)(0x1<<6)
#define I2C_SR1_STOPF					(uint16_t)(0x1<<4)
#define I2C_SR2_MSL						(uint16_t)(0x1<<0)
#define I2C_SR2_TRA						(uint16_t)(0x1<<2)
#define I2C_SR2_BUSY					(uint16_t)(0x1<<1)



/*
 *
 *
 * =======================================================================================
 * 							APIs Supported by "MCAL I2C DRIVER"
 * =======================================================================================
 */


void MCAL_I2C_Init(I2C_TypeDef* I2Cx, I2C_InitTypeDef* I2C_InitStruct);
void MCAL_I2C_DInit(I2C_TypeDef* I2Cx);


void MCAL_I2C_GPIO_Set_Pins (I2C_TypeDef* I2Cx);


//Master Polling Mechanism

void MCAL_I2C_Master_TX (I2C_TypeDef* I2Cx, uint16_t devAddr , uint8_t* dataOut , uint32_t dataLen , Stop_Condition Stop, Repeated_Start start);
void MCAL_I2C_Master_RX (I2C_TypeDef* I2Cx ,uint16_t devAddr, uint8_t* dataOut, uint32_t dataLen, Stop_Condition Stop , Repeated_Start start);




//Slave interrupt mechanism
void MCAL_I2C_SlaveSendData(I2C_TypeDef* I2Cx,uint8_t data);
uint8_t MCAL_I2C_SlaveReceiveData(I2C_TypeDef* I2Cx);


//Generic APIs

void I2C_GenerateSTART(I2C_TypeDef* I2Cx, FunctionalState NewState , Repeated_Start start);

FlagStatus  I2C_GetFlagStatus(I2C_TypeDef* I2Cx, Status flag);

void I2C_SendAddress(I2C_TypeDef* I2Cx, uint16_t Address , I2C_Direction Direction);

void I2C_GenerateSTOP(I2C_TypeDef* I2Cx, FunctionalState NewState);

void I2C_AcknowledgeConfig(I2C_TypeDef* I2Cx, FunctionalState NewState);













































#endif /* INC_I2C_H_ */
