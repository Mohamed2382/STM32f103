/**
 ******************************************************************************
 * @file           : GPIO.c
 * @author         : Mohamed Yahya
 * @brief          : GPIO_functions Implementation
 ******************************************************************************
 */
#include "GPIO.h"



uint8_t Get_CRLH_Position(uint16_t PinNumber)
{
	switch (PinNumber)
	{
	case GPIO_PIN_0:
		return 0 ;
		break ;
	case GPIO_PIN_1:
		return 4 ;
		break ;
	case GPIO_PIN_2:
		return 8 ;
		break ;
	case GPIO_PIN_3:
		return 12 ;
		break ;

	case GPIO_PIN_4:
		return 16 ;
		break ;


	case GPIO_PIN_5:
		return 20 ;
		break ;

	case GPIO_PIN_6:
		return 24 ;
		break ;

	case GPIO_PIN_7:
		return 28 ;
		break ;


	case GPIO_PIN_8:
		return 0 ;
		break ;
	case GPIO_PIN_9:
		return 4 ;
		break ;

	case GPIO_PIN_10:
		return 8 ;
		break ;

	case GPIO_PIN_11:
		return 12 ;
		break ;

	case GPIO_PIN_12:
		return 16 ;
		break ;


	case GPIO_PIN_13:
		return 20 ;
		break ;

	case GPIO_PIN_14:
		return 24 ;
		break ;

	case GPIO_PIN_15:
		return 28 ;
		break ;



	}

	return 0 ;
}




/**================================================================
 * @Fn				-MCAL_GPIO_Init
 * @brief 			-Initializes the GPIOx PINy according to the specified parameters in the PinConfig
 * @param [in] 	-GPIOx: where x can be (A..E depending on device used) to select the GPIO peripheral
 * @param [in] 	-PinConfig pointer to a GPIO_PinConfig_t structure that contains
 *         		 the configuration information for the specified GPIO PIN.
 * @retval 		-none
 * Note			-Stm32F103C6 MCU has GPIO A,B,C,D,E Modules
 * 				 But LQFP48 Package has only GPIO A,B,PART of C/D exported as external PINS from the MCU
 */

void MCAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_PinConfig_t* PinConfig)
{
	// Port configuration register low (GPIO_Pin_CRL) Configure PINS from 0 >>> 7
	// Port configuration register High (GPIOx_CRH) Configure PINS from 8 >>> 15
	volatile uint32_t* configregister = NULL;
	uint8_t PIN_Config = 0;

	// Determine whether to configure low or high register based on Pin number
	configregister = (PinConfig->GPIO_PinNumber < GPIO_PIN_8) ? &GPIOx->CRL : &GPIOx->CRH;

	// Clear CNF[1:0] MODE[1:0] bits in the configuration register
	(*configregister) &= ~(0xf << Get_CRLH_Position(PinConfig->GPIO_PinNumber));

	// Check if the pin is configured as an output
	if ((PinConfig->GPIO_MODE == GPIO_MODE_OUTPUT_AF_OD) || (PinConfig->GPIO_MODE == GPIO_MODE_OUTPUT_AF_PP) || (PinConfig->GPIO_MODE == GPIO_MODE_OUTPUT_OD) || (PinConfig->GPIO_MODE == GPIO_MODE_OUTPUT_PP))
	{
		// Set CNF[1:0] MODE[1:0] bits based on GPIO mode and output speed
		PIN_Config = ((((PinConfig->GPIO_MODE - 4) << 2) | (PinConfig->GPIO_Output_Speed)) & 0x0f);
	}
	// If pin is configured as an input
	else
	{
		if ((PinConfig->GPIO_MODE == GPIO_MODE_INPUT_FLO) || (PinConfig->GPIO_MODE == GPIO_MODE_Analog))
		{
			// Set CNF[1:0] MODE[1:0] bits for input mode
			PIN_Config = ((((PinConfig->GPIO_MODE) << 2) | 0x0) & 0x0f);
		}
		else if (PinConfig->GPIO_MODE == GPIO_MODE_AF_INPUT) // Consider it as input floating
		{
			// Set CNF[1:0] MODE[1:0] bits for input mode (floating)
			PIN_Config = ((((GPIO_MODE_INPUT_FLO) << 2) | 0x0) & 0x0f);
		}
		else // Pull-up or Pull-down input
		{
			PIN_Config = ((((GPIO_MODE_INPUT_PU) << 2) | 0x0) & 0x0f);

			// Configure pull-up or pull-down in ODR based on GPIO mode
			if (PinConfig->GPIO_MODE == GPIO_MODE_INPUT_PU)
			{
				// PxODR = 1: Input pull-up
				GPIOx->ODR |= PinConfig->GPIO_PinNumber;
			}
			else
			{
				// PxODR = 0: Input pull-down
				GPIOx->ODR &= ~(PinConfig->GPIO_PinNumber);
			}
		}
	}

	// Write the configuration to the CRL or CRH register
	(*configregister) |= ((PIN_Config) << Get_CRLH_Position(PinConfig->GPIO_PinNumber));
}




/**================================================================
 * @Fn				-MCAL_GPIO_DeInit
 * @brief 			-reset all the GPIO Registers
 * @param [in] 	-GPIOx: where x can be (A..E depending on device used) to select the GPIO peripheral
 * @retval 		-none
 * Note			-none
 */
void MCAL_GPIO_DeInit (GPIO_TypeDef *GPIOx)
{
	//	GPIOx->BRR = 0x00000000 ;
	//	GPIOx->BSRR =0x00000000 ;
	//	GPIOx->CRH = 0x44444444 ;
	//	GPIOx->CRL =  0x44444444 ;
	////	GPIOx->IDR = ;  (READ Only)
	//	GPIOx->LCKR =  0x00000000 ;
	//	GPIOx->ODR = 0x00000000;

	//or you can use the reset Controller
	//APB2 peripheral reset register (RCC_APB2RSTR)
	//Set and cleared by software.
	if (GPIOx == GPIOA)
	{
		RCC->APB2RSTR |= (1<<2);  //Bit 2 IOPARST: IO port A reset
		RCC->APB2RSTR &= ~(1<<2);

	}else if (GPIOx == GPIOB)
	{
		RCC->APB2RSTR |= (1<<3);  //Bit 3 IOPBRST: IO port B reset
		RCC->APB2RSTR &= ~(1<<3);

	}else if (GPIOx == GPIOC)
	{
		RCC->APB2RSTR |= (1<<4);  //Bit 4 IOPCRST: IO port C reset
		RCC->APB2RSTR &= ~(1<<4);

	}else if (GPIOx == GPIOD)
	{
		RCC->APB2RSTR |= (1<<5);  //Bit 5 IOPDRST: IO port D reset
		RCC->APB2RSTR &= ~(1<<5);

	}else if (GPIOx == GPIOE)
	{
		RCC->APB2RSTR |= (1<<6);  //Bit 6 IOPERST: IO port E reset
		RCC->APB2RSTR &= ~(1<<6);

	}

}


/**================================================================
 * @Fn				-MCAL_GPIO_ReadPin
 * @brief 			-Read Specific PIN
 * @param [in] 	-GPIOx: where x can be (A..E depending on device used) to select the GPIO peripheral
 * @param [in] 	-PinNumber: Set Pin Number according @ref GPIO_PINS_define
 * @retval 		-the input pin value (two values based on @ref GPIO_PIN_state )
 * Note			-none
 */
uint8_t MCAL_GPIO_ReadPin (GPIO_TypeDef *GPIOx,uint16_t PinNumber )
{
	uint8_t bitstatus ;
	if (((GPIOx->IDR) & PinNumber ) !=  (uint32_t)GPIO_PIN_RESET )
	{
		bitstatus = GPIO_PIN_SET ;
	}else
	{
		bitstatus = GPIO_PIN_RESET ;
	}
	return  bitstatus ;
}



/**================================================================
 * @Fn					-MCAL_GPIO_ReadPort
 * @brief 				-read the input port VALUE
 * @param [in] 			-GPIOx: where x can be (A..G depending on device used) to select the GPIO peripheral
 * @retval 				-the input port VALUE
 * Note					-none
 */
uint16_t MCAL_GPIO_ReadPort (GPIO_TypeDef *GPIOx)
{
	uint16_t port_value ;
	port_value  =   (uint16_t)(GPIOx->IDR);
	return port_value ;

}


/**================================================================
 * @Fn					-MCAL_GPIO_WritePin
 * @brief 				-write on specific input pin
 * @param [in] 			-GPIOx: where x can be (A..G depending on device used) to select the GPIO peripheral
 *@param [in] 			-PinNumber:  specifies the port bit to read. Set by @ref GPIO_PINS_define
 *@param [in] 			-Value: Pin Value
 * @retval 			-none
 * Note				-none
 */
void MCAL_GPIO_WritePin (GPIO_TypeDef *GPIOx,uint16_t PinNumber , uint8_t Value)
{
	if ( Value !=  GPIO_PIN_RESET )
	{
		//		GPIOx->ODR |= PinNumber ;
		//		or
		//		Bits 15:0 BSy: Port x Set bit y (y= 0 .. 15)
		//		These bits are write-only and can be accessed in Word mode only.
		//		0: No action on the corresponding ODRx bit
		//		1: Set the corresponding ODRx bit
		GPIOx->BSRR = (uint32_t)PinNumber ;

	}else
	{
		//		Bits 31:16 BRy: Port x Reset bit y (y= 0 .. 15)
		//		These bits are write-only and can be accessed in Word mode only.
		//		0: No action on the corresponding ODRx bit
		//		1: Reset the corresponding ODRx bit
		GPIOx->BRR = (uint32_t)PinNumber ;
	}

}

/**================================================================
 * @Fn					-MCAL_GPIO_WritePort
 * @brief 				-write on output port
 * @param [in] 			-GPIOx: where x can be (A..G depending on device used) to select the GPIO peripheral
 * @retval 				-none
 * Note					-none
 */
void MCAL_GPIO_WritePort (GPIO_TypeDef *GPIOx , uint16_t Value)
{
	GPIOx->ODR = (uint32_t)Value ;
}


/**================================================================
 * @Fn					-MCAL_GPIO_TogglePin
 * @brief 				-Toggles the specified GPIO pin
 * @param [in] 			-GPIOx: where x can be (A..G depending on device used) to select the GPIO peripheral
 * @param [in] 			-PinNumber: specifies the port bit to read. Set by @ref GPIO_PINS_define
 * @retval 			-none
 * Note				-none
 */
void MCAL_GPIO_TogglePin (GPIO_TypeDef *GPIOx,uint16_t PinNumber )
{
	GPIOx->ODR ^= (PinNumber) ;
}


/**================================================================
 * @Fn					-MCAL_GPIO_LockPin
 * @brief 				-The locking mechanism allows the IO configuration to be frozen
 * @param [in] 			-GPIOx: where x can be (A..G depending on device used) to select the GPIO peripheral
 * @param [in] 		-PinNumber: specifies the port bit to read. Set by @ref GPIO_PINS_define
 * @retval 			-Ok if pin Config is locked Or ERROR if pin  not locked  (OK & ERROR are defined @ref GPIO_RETURN_LOCK
 * Note				-none
 */
uint8_t MCAL_GPIO_LockPin (GPIO_TypeDef *GPIOx,uint16_t PinNumber )
{

	//	Bit 16 LCKK[16]: Lock key
	//	This bit can be read anytime. It can only be modified using the Lock Key Writing Sequence.
	//	0: Port configuration lock key not active
	//	1: Port configuration lock key active. GPIOx_LCKR register is locked until the next reset.
	//	LOCK key writing sequence:
	//	Write 1
	//	Write 0
	//	Write 1
	//	Read 0
	//	Read 1 (this read is optional but confirms that the lock is active)
	//	Note: During the LOCK Key Writing sequence, the value of LCK[15:0] must not change.
	//	Any error in the lock sequence will abort the lock.
	//	Bits 15:0 LCKy: Port x Lock bit y (y= 0 .. 15)
	//	These bits are read write but can only be written when the LCKK bit is 0.
	//	0: Port configuration not locked
	//	1: Port configuration locked

	//Set LCKK[16]
	volatile uint32_t tmp = 1<<16 ;
	//Set the LCKy
	tmp |= PinNumber ;

	//	Write 1
	GPIOx->LCKR = tmp ;
	//	Write 0
	GPIOx->LCKR = PinNumber ;
	//	Write 1
	GPIOx->LCKR = tmp ;

	//	Read 0
	tmp = GPIOx->LCKR  ;
	//	Read 1 (this read is optional but confirms that the lock is active)
	if ( (uint32_t) (GPIOx->LCKR  & 1<<16 ))
	{
		return GPIO_RETURN_LOCK_Enabled ;
	}else
	{
		return GPIO_RETURN_LOCK_ERROR ;

	}

}




//comment

uint8_t getposition(uint16_t GPIO_PIN__num){
	uint8_t rval;
	if(GPIO_PIN__num == GPIO_PIN_0){
		rval =0;
	}
	if(GPIO_PIN__num ==GPIO_PIN_1){
		rval =4;
	}
	if(GPIO_PIN__num == GPIO_PIN_2){
		rval =8;
	}
	if(GPIO_PIN__num == GPIO_PIN_3){
		rval =12;
	}
	if(GPIO_PIN__num == GPIO_PIN_4){
		rval =16;
	}
	if(GPIO_PIN__num == GPIO_PIN_5){
		rval =20;
	}
	if(GPIO_PIN__num == GPIO_PIN_6){
		rval =24;
	}
	if(GPIO_PIN__num == GPIO_PIN_7){
		rval =28;
	}
	if(GPIO_PIN__num == GPIO_PIN_8){
		rval =0;
	}
	if(GPIO_PIN__num == GPIO_PIN_9){
		rval =4;
	}
	if(GPIO_PIN__num == GPIO_PIN_10){
		rval =8;
	}
	if(GPIO_PIN__num == GPIO_PIN_11){
		rval =12;
	}
	if(GPIO_PIN__num == GPIO_PIN_12){
		rval =16;
	}
	if(GPIO_PIN__num == GPIO_PIN_13){
		rval =20;
	}
	if(GPIO_PIN__num == GPIO_PIN_14){
		rval =24;
	}
	if(GPIO_PIN__num == GPIO_PIN_15){
		rval =28;
	}
	return rval;


}

void pinmode(GPIO_TypeDef* GPIOx,uint16_t pin,uint32_t pinmode){
	if(GPIOx==GPIOA){
		RCC_GPIOA_CLK_EN();

	}
	else if(GPIOx==GPIOB){
		RCC_GPIOB_CLK_EN();

	}
	else{
		RCC_GPIOC_CLK_EN();
	}

	if(pin<8){
		GPIOx-> CRL &=~(0xf<<(getposition(pin)));

		if((pinmode==GPIO_MODE_INPUT_PD)||(pinmode==GPIO_MODE_INPUT_PU)){
			if(pinmode == GPIO_MODE_INPUT_PU){
				GPIOx-> CRL |=(0b1000<<(getposition(pin)));
				GPIOx-> ODR |= (1<<pin);
			}
			else{
				GPIOx-> CRL |=(0b1000<<(getposition(pin)));
				GPIOx-> ODR &=~(1<<pin);
			}

		}
		else GPIOx-> CRL |=(pinmode<<(getposition(pin)));

	}
	else if(pin>7){
		//GPIOx-> CRH
		GPIOx-> CRH &=~(0xf<<(getposition(pin)));

		if((pinmode==GPIO_MODE_INPUT_PD)||(pinmode==GPIO_MODE_INPUT_PU)){
			if(pinmode == GPIO_MODE_INPUT_PU){
				GPIOx-> CRH |=(0b1000<<(getposition(pin)));
				GPIOx-> ODR |= (1<<pin);
			}
			else{
				GPIOx-> CRH |=(0b1000<<(getposition(pin)));
				GPIOx-> ODR &=~(1<<pin);
			}

		}
		else GPIOx-> CRH |=(pinmode<<(getposition(pin)));

	}



}


/**================================================================
 * @Fn- pinwrite
 * @brief - Write the GPIOx PINy according to specified parameters in Pin_config
 * @param [in] - GPIOx: where x can be (A...E Depending on device used) to select the GPIO Peripheral
 * @param [in] - pin: GPIOx PIN Number
 * @param [in] - status: The desired value to write
 * @retval - None
 * Note-
 */
void pinwrite(GPIO_TypeDef* GPIOx,uint16_t pin,uint8_t status){
	if(status!=0){
		GPIOx-> ODR |=(1<<pin);
	}
	else GPIOx-> ODR &=~(1<<pin);

}


/**================================================================
 * @Fn- WRITE_PORT
 * @brief - Write the GPIOx according to specified parameters in Pin_config
 * @param [in] - GPIOx: where x can be (A...E Depending on device used) to select the GPIO Peripheral
 * @param [in] - value: The desired value to write
 * @retval -
 * Note-
 */
void WRITE_PORT(GPIO_TypeDef* GPIOx,uint16_t status){
	GPIOx-> ODR = (uint16_t)status;

}


