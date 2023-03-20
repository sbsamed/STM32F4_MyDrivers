/*
 * gpio.c
 *
 *  Created on: 5 Kas 2022
 *      Author: Samed
 */

#include "gpio.h"


/*
 * @brief GPIO_Init configures port and pin
 *
 * @param GPIOx : Gpio port base address
 * @param GPIO_ConfigStruct : user config structer
 *
 * @retval void
 */


void GPIO_Init(GPIO_Typedef_t *GPIOx,GPIO_InitTypedef_t *GPIO_ConfigStruct){

	uint32_t position;
	uint32_t fakePosition=0;
	uint32_t lastPosition=0;

	for(position=0;position<16;position++){

		fakePosition=(0x1<<position);
		lastPosition=(uint32_t)(GPIO_ConfigStruct->pinNumber) & fakePosition;

		if(fakePosition==lastPosition){

			/* MODE Config*/
			uint32_t tempValue= GPIOx->MODER;
			tempValue &= ~( 0x3U << (position * 2) );
			tempValue |=  ( GPIO_ConfigStruct->Mode << (position * 2) );
		    GPIOx->MODER=tempValue;

		    if( GPIO_ConfigStruct->Mode == GPIO_MODE_OUTPUT ||  GPIO_ConfigStruct->Mode== GPIO_MODE_AF ){

		    	/* OUTPUT type  Config*/
		    	 tempValue = GPIOx->OTYPE;
		    	 tempValue &= ~( 0x1U << position );
		    	 tempValue |= (GPIO_ConfigStruct->Otype << position);
		    	 GPIOx->OTYPE =tempValue;

			    /* OUTPUT SPEEDR  Config*/
			    tempValue = GPIOx->OSPEEDR;
			    tempValue &= ~( 0x3U << (position*2) );
			    tempValue |= (GPIO_ConfigStruct->Speed << (position*2));
			    GPIOx->OSPEEDR =tempValue;
		}
		    /* PUSH PULL  Config*/
		    tempValue = GPIOx->PUPDR;
		    tempValue &= ~( 0x3U << (position*2) );
		    tempValue |= (GPIO_ConfigStruct->PuPd << (position*2));
		    GPIOx->PUPDR =tempValue;

		    if(GPIO_ConfigStruct->Mode ==GPIO_MODE_AF){

		    	tempValue=0;

		    	tempValue=GPIOx->AFR[ (position>>3U) ];

		    	tempValue &= ~ (0x3FU<< (position & 0x7U) * 4 ) ;

		    	tempValue |= (GPIO_ConfigStruct->Alternate << (position & 0x7U) * 4 ) ;

		    	GPIOx->AFR[ (position>>3U) ]=tempValue;


		    }

	}

  }
}

/*
 * @brief GPIO_WritePin makes pin low or ,high
 *
 * @param GPIOx : Gpio port base address
 * @param pinNumber : pin number 0-15
 * @param pinState : pin GPIO_Pin_Set or GPIO_Pin_Reset
 *
 * @retval void
 */
void GPIO_WritePin(GPIO_Typedef_t *GPIOx, uint16_t pinNumber, GPIO_PinState_t pinState){


	if(pinState== GPIO_Pin_Set){
		GPIOx->BSRR =pinNumber;
	}else{
		GPIOx->BSRR =(pinNumber<<16U);

	}
}

/*
 * @brief GPIO_ReadPin reads the pin of gpıox port
 *
 * @param GPIOx : Gpio port base address
 * @param pinNumber : pin number 0-15
 *
 * @retval GPIO_PinState_t , GPIO_Pin_Reset or GPIO_Pin_Set
 */

GPIO_PinState_t GPIO_ReadPin(GPIO_Typedef_t *GPIOx, uint16_t pinNumber){

	GPIO_PinState_t bitStatus = GPIO_Pin_Reset;

	if( ( GPIOx->IDR & pinNumber ) != GPIO_Pin_Set){

		bitStatus=GPIO_Pin_Set;

	}
	return bitStatus;

}

/*
 * @brief GPIO_LockPin lock the pin of gpıox port
 *
 * @param GPIOx : Gpio port base address
 * @param pinNumber : pin number 0-15
 *
 * @retval void
 */
void GPIO_LockPin(GPIO_Typedef_t *GPIOx, uint16_t pinNumber){

	uint32_t tempValue = (0x1U<<16) | pinNumber;

	GPIOx->LCKR=tempValue; 		/*!< LCKR[16]= '1'  LCKR[15:0]= 'data'  */
	GPIOx->LCKR=pinNumber; 		/*!< LCKR[16]= '0'  LCKR[15:0]= 'data'  */
	tempValue=GPIOx->LCKR;      /*!< Read Lock Register				    */
}

/*
 * @brief GPIO_TogglePin toggle the pin of gpıox port
 *
 * @param GPIOx : Gpio port base address
 * @param pinNumber : pin number 0-15
 *
 * @retval void
 */
void GPIO_TogglePin(GPIO_Typedef_t *GPIOx, uint16_t pinNumber){

	uint16_t tempOddrRegister = GPIOx->ODR;
	GPIOx->BSRR= ( (tempOddrRegister & pinNumber) << 16U ) |
					( ~tempOddrRegister & pinNumber);

}
