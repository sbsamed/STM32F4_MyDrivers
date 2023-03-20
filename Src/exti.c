/*
 * exti.c
 *
 *  Created on: 8 Ara 2022
 *      Author: SAMEDBASKİN
 */

#include "exti.h"

/*
 * @brief EXTI_LineConfig configures port and pin
 *
 * @param portSoruce : port value A-G @def_group PORT_Values
 * @param EXTI_Line_Source :  line value 0-15 number @def_group LINE_Values
 *
 * @retval void
 */

void EXTI_LineConfig(uint8_t portSoruce , uint8_t EXTI_Line_Source){

	uint32_t tempValue;

    tempValue = SYSCFG->EXTI_CR[EXTI_Line_Source >> 2U];
    tempValue &= ~(0xFU <<  (EXTI_Line_Source & 0x3U) * 4);
    tempValue = (portSoruce << (EXTI_Line_Source & 0x3U) * 4);

    SYSCFG->EXTI_CR[EXTI_Line_Source >> 2U]=tempValue;
}
/*
 * @brief EXTI_Init for EXTI register
 *
 * @param EXTI_InitStruct : user config structer
 *
 * @retval void
 */

void EXTI_Init(EXTI_InitTyeDef_t *EXTI_InitStruct){

	uint32_t tempValue=0;

	tempValue = (uint32_t)EXTI_BASE_ADDR;

	EXTI->IMR &= ~(0x1U << EXTI_InitStruct->EXTI_LineNumber );
	EXTI->EMR &= ~(0x1U << EXTI_InitStruct->EXTI_LineNumber );

	if(EXTI_InitStruct->EXTI_LineCmd !=DISABLE){

		tempValue+= EXTI_InitStruct->EXTI_Mode;

		*( (__IO uint32_t*)tempValue) |= (0x1U << EXTI_InitStruct->EXTI_LineNumber);


		tempValue=(uint32_t)EXTI_BASE_ADDR;

		EXTI->RTSR &= ~(0x1U << EXTI_InitStruct->EXTI_LineNumber );
		EXTI->FTSR &= ~(0x1U << EXTI_InitStruct->EXTI_LineNumber );

         if(EXTI_InitStruct->TriggerSelection == EXTI_TRIG_RF){

        	EXTI->RTSR |= (0x1U << EXTI_InitStruct->EXTI_LineNumber );
        	EXTI->FTSR |= (0x1U << EXTI_InitStruct->EXTI_LineNumber );

         }else{

        	tempValue+= EXTI_InitStruct->TriggerSelection;
        	*( (__IO uint32_t*)tempValue) |= (0x1U << EXTI_InitStruct->EXTI_LineNumber);
         }

	}else{

		tempValue = (uint32_t)EXTI_BASE_ADDR;

		tempValue+= EXTI_InitStruct->EXTI_Mode;

		*( (__IO uint32_t*)tempValue) &= ~(0x1U << EXTI_InitStruct->EXTI_LineNumber);

	}
}

/*
 * @brief NVIC_EnableInterrupt
 *
 * @param IRQNumber : IRQ Number of line
 *
 * @retval void
 */
void NVIC_EnableInterrupt(IRQNumber_TypeDef_t IRQNumber){

	uint32_t tempValue=0;

	tempValue = *( ( IRQNumber >> 5U ) + NVIC_ISER0 );

	tempValue &= ~( 0x1U << ( IRQNumber & 0x1FU ) );
	tempValue |=  ( 0x1U << ( IRQNumber & 0x1FU ) );

    *( ( IRQNumber >> 5U ) + NVIC_ISER0 ) = tempValue ;

}








