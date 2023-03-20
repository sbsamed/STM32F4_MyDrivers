/*
 * gpio.h
 *
 *  Created on: 5 Kas 2022
 *      Author: Samed
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "stm32f407xx.h"

/*
 * @def_group GPIO_Pins
 */
#define GPIO_PIN_0			(uint16_t)(0x0001)			/*!<GPIO0  PİN SELECTED */
#define GPIO_PIN_1			(uint16_t)(0x0002)			/*!<GPIO1  PİN SELECTED */
#define GPIO_PIN_2			(uint16_t)(0x0004)			/*!<GPIO2  PİN SELECTED */
#define GPIO_PIN_3			(uint16_t)(0x0008)			/*!<GPIO3  PİN SELECTED */
#define GPIO_PIN_4			(uint16_t)(0x0010)			/*!<GPIO4  PİN SELECTED */
#define GPIO_PIN_5			(uint16_t)(0x0020)			/*!<GPIO5  PİN SELECTED */
#define GPIO_PIN_6			(uint16_t)(0x0040)			/*!<GPIO6  PİN SELECTED */
#define GPIO_PIN_7			(uint16_t)(0x0080)			/*!<GPIO7  PİN SELECTED */
#define GPIO_PIN_8			(uint16_t)(0x0100)			/*!<GPIO8  PİN SELECTED */
#define GPIO_PIN_9			(uint16_t)(0x0200)			/*!<GPIO9  PİN SELECTED */
#define GPIO_PIN_10			(uint16_t)(0x0400)			/*!<GPIO10 PİN SELECTED */
#define GPIO_PIN_11			(uint16_t)(0x0800)			/*!<GPIO11 PİN SELECTED */
#define GPIO_PIN_12			(uint16_t)(0x1000)			/*!<GPIO12 PİN SELECTED */
#define GPIO_PIN_13			(uint16_t)(0x2000)			/*!<GPIO13 PİN SELECTED */
#define GPIO_PIN_14			(uint16_t)(0x4000)			/*!<GPIO14 PİN SELECTED */
#define GPIO_PIN_15 		(uint16_t)(0x8000)			/*!<GPIO15 PİN SELECTED */
#define GPIO_PIN_All 		(uint16_t)(0xFFFF)			/*!<GPIO   ALL SELECTED */

/*
 * @def_group GPIO_Modes
 */
#define GPIO_MODE_INPUT 		(0x0U)		/*!<GPIO MODE İS INPUT  			*/
#define GPIO_MODE_OUTPUT 		(0x1U)  	/*!<GPIO MODE İS OUTPUT			    */
#define GPIO_MODE_AF		 	(0x2U)		/*!<GPIO MODE İS ALERNATE FUNCTION  */
#define GPIO_MODE_ANALOG 		(0x3U)		/*!<GPIO MODE İS ANALOG  			*/

/*
 * @def_group GPIO_Otype
 */
#define GPIO_OTYPE_PP			(0x0U)    	/*!<GPIO OTYPE İS PUSH-PULL  		*/
#define GPIO_OTYPE_PD			(0x1U)    	/*!<GPIO OTYPE İS OPEN-DRAİN  		*/

/*
* @def_group GPIO_PuPd
*/
#define GPIO_OTYPE_NOPULL		(0x0U)    	/*!<GPIO OTYPE İS NO PULL UP/DOWN  	*/
#define GPIO_OTYPE_PULLUP		(0x1U)    	/*!<GPIO OTYPE İS PULLUP	 		*/
#define GPIO_OTYPE_PULLDOWN		(0x2U)    	/*!<GPIO OTYPE İS PULLDOWN	  		*/


/*
 * @def_group GPIO_Speed
 */
#define GPIO_OTYPE_LOW_SPEED		(0x0U)    	/*!<GPIO Speed İS LOWSPEED	     */
#define GPIO_OTYPE_MEDIUM_SPEED		(0x1U)    	/*!<GPIO Speed İS MEDIUMSPEED	 */
#define GPIO_OTYPE_HIGH_SPEED		(0x2U)    	/*!<GPIO Speed İS HIGHSPEED	  	 */
#define GPIO_OTYPE_VERY_HIGH_SPEED	(0x3U)    	/*!<GPIO Speed İS VERYHIGHSPEED	 */


/*
* @def_group GPIO_AF_MODES
*/
#define GPIO_AF_MODES_0		(0x0U)    	/*!<GPIO OTYPE İS NO PULL UP/DOWN  	*/
#define GPIO_AF_MODES_1		(0x1U)    	/*!<GPIO OTYPE İS PULLUP	 		*/
#define GPIO_AF_MODES_2		(0x2U)    	/*!<GPIO OTYPE İS PULLDOWN	  		*/
#define GPIO_AF_MODES_3		(0x3U)    	/*!<GPIO OTYPE İS PULLDOWN	  		*/
#define GPIO_AF_MODES_4		(0x4U)    	/*!<GPIO OTYPE İS PULLDOWN	  		*/
#define GPIO_AF_MODES_5		(0x5U)    	/*!<GPIO OTYPE İS PULLDOWN	  		*/
#define GPIO_AF_MODES_6		(0x6U)    	/*!<GPIO OTYPE İS PULLDOWN	  		*/
#define GPIO_AF_MODES_7		(0x7U)    	/*!<GPIO OTYPE İS PULLDOWN	  		*/
#define GPIO_AF_MODES_8		(0x8U)    	/*!<GPIO OTYPE İS PULLDOWN	  		*/
#define GPIO_AF_MODES_9		(0x9U)    	/*!<GPIO OTYPE İS PULLDOWN	  		*/
#define GPIO_AF_MODES_10	(0xAU)    	/*!<GPIO OTYPE İS PULLDOWN	  		*/
#define GPIO_AF_MODES_11	(0xBU)    	/*!<GPIO OTYPE İS PULLDOWN	  		*/
#define GPIO_AF_MODES_12	(0xCU)    	/*!<GPIO OTYPE İS PULLDOWN	  		*/
#define GPIO_AF_MODES_13	(0xDU)    	/*!<GPIO OTYPE İS PULLDOWN	  		*/
#define GPIO_AF_MODES_14	(0xEU)    	/*!<GPIO OTYPE İS PULLDOWN	  		*/
#define GPIO_AF_MODES_15	(0xFU)    	/*!<GPIO OTYPE İS PULLDOWN	  		*/


typedef enum{

	GPIO_Pin_Reset= 0x0U,
	GPIO_Pin_Set= !GPIO_Pin_Reset

}GPIO_PinState_t;


typedef struct{

	uint32_t pinNumber;		/*!GPIO Pin numbers @def_group GPIO_Pins 		*/
	uint32_t Mode;			/*!GPIO Pin numbers @def_group GPIO_Modes 		*/
	uint32_t Otype;			/*!GPIO Pin numbers @def_group GPIO_Otype	    */
	uint32_t PuPd;			/*!GPIO Pin numbers @def_group GPIO_PuPd 		*/
	uint32_t Speed;			/*!GPIO Pin numbers @def_group GPIO_Speed 		*/
	uint32_t Alternate;     /*!GPIO Pin numbers @def_group GPIO_Alternate	*/

}GPIO_InitTypedef_t;


void GPIO_Init(GPIO_Typedef_t *GPIOx,GPIO_InitTypedef_t *GPIO_ConfigStruct);
void GPIO_WritePin(GPIO_Typedef_t *GPIOx, uint16_t pinNumber, GPIO_PinState_t pinState);
GPIO_PinState_t GPIO_ReadPin(GPIO_Typedef_t *GPIOx, uint16_t pinNumber);
void GPIO_LockPin(GPIO_Typedef_t *GPIOx, uint16_t pinNumber);
void GPIO_TogglePin(GPIO_Typedef_t *GPIOx, uint16_t pinNumber);

#endif /* INC_GPIO_H_ */


















