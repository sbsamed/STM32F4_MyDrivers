/*
 * exti.h
 *
 *  Created on: 8 Ara 2022
 *      Author: SAMEDBASKİN
 */

#ifndef INC_EXTI_H_
#define INC_EXTI_H_

#include "stm32f407xx.h"

/*
 * @def_group PORT_Values
 */
#define EXTI_PortSource_GPIOA   ( (uint8_t)(0x0) )
#define EXTI_PortSource_GPIOB   ( (uint8_t)(0x1) )
#define EXTI_PortSource_GPIOC   ( (uint8_t)(0x2) )
#define EXTI_PortSource_GPIOD   ( (uint8_t)(0x3) )
#define EXTI_PortSource_GPIOE   ( (uint8_t)(0x4) )
#define EXTI_PortSource_GPIOF   ( (uint8_t)(0x5) )
#define EXTI_PortSource_GPIOG   ( (uint8_t)(0x6) )

/*
 * @def_group LINE_Values
 */
#define EXTI_Line_Source_0       ( (uint8_t)(0x0) )
#define EXTI_Line_Source_1       ( (uint8_t)(0x1) )
#define EXTI_Line_Source_2       ( (uint8_t)(0x2) )
#define EXTI_Line_Source_3       ( (uint8_t)(0x2) )
#define EXTI_Line_Source_4       ( (uint8_t)(0x4) )
#define EXTI_Line_Source_5       ( (uint8_t)(0x5) )
#define EXTI_Line_Source_6       ( (uint8_t)(0x6) )
#define EXTI_Line_Source_7       ( (uint8_t)(0x7) )
#define EXTI_Line_Source_8       ( (uint8_t)(0x8) )
#define EXTI_Line_Source_9       ( (uint8_t)(0x9) )
#define EXTI_Line_Source_10      ( (uint8_t)(0xA) )
#define EXTI_Line_Source_11      ( (uint8_t)(0xB) )
#define EXTI_Line_Source_12      ( (uint8_t)(0xC) )
#define EXTI_Line_Source_13      ( (uint8_t)(0xD) )
#define EXTI_Line_Source_14      ( (uint8_t)(0xE) )
#define EXTI_Line_Source_15      ( (uint8_t)(0x0F))

/*
* @def_group EXTI_Modes
*/

#define EXTI_MODE_Interrupt  (0x00U)
#define EXTI_MODE_Event		 (0x04U)

/*
* @def_group TriggerSelection
*/

#define EXTI_TRIG_Rising    (0x08U)
#define EXTI_TRIG_Falling	(0x0CU)
#define EXTI_TRIG_RF		(0x10U)

typedef struct{

	uint8_t EXTI_LineNumber;         /*!< EXTI line number for valid GPIO pin @def_group LINE_Values  */
	uint8_t TriggerSelection;        /*!< EXTI trigger selection  @def_group TriggerSelection          */
	uint8_t EXTI_Mode;               /*!< EXTI mode selection event or interrupt @def_group EXTI_Modes */
	FunctionalState_t EXTI_LineCmd;  /*!< EXTI mask selection enable or disable  */

}EXTI_InitTyeDef_t;

void EXTI_Init(EXTI_InitTyeDef_t *EXTI_InitStruct);
void EXTI_LineConfig(uint8_t portSoruce , uint8_t EXTI_Line_Source);
void NVIC_EnableInterrupt(IRQNumber_TypeDef_t IRQNumber);
#endif /* INC_EXTI_H_ */
