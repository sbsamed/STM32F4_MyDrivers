/*
 * stm32f407xx.h
 *
 *  Created on: Oct 17, 2022
 *      Author: SAMEDBASKIN
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <string.h>
#include <stddef.h>


/*
 * Microprocessor Defines
 *
*/

#define NVIC_ISER0 	((uint32_t*)(0xE000E100))


#define __IO volatile

#define  SET_BIT(REG,BIT)  		( (REG) |= (BIT) )
#define  CLEAR_BIT(REG,BIT)  	( (REG) &= ~(BIT) )
#define  READ_BIT(REG,BIT)  	( (REG) &= (BIT) )


typedef enum{
	DISABLE=0x0U,
	ENABLE=!DISABLE
}FunctionalState_t;

/*
 *
 * IRQ Numbers of MCU= Vector Table
 */
typedef enum{

	EXTI0_IRQNumber = 6 ,
	EXTI1_IRQNumber = 7 ,
	EXTI2_IRQNumber = 8 ,
	EXTI3_IRQNumber = 9 ,
	EXTI4_IRQNumber = 10

}IRQNumber_TypeDef_t;
/*
 * Memory Base Address
 */

#define FLAS_BASE_ADDR				(0x08000000UL) /* Flash Base Addres (up to 1 MB)			*/
#define SRAM1_BASE_ADDR				(0x20000000UL) /* SRAM1 Base Address (up to 112 KB)		    */
#define SRAM2_BASE_ADDR				(0x2001C000UL) /* SRAM1 Base Address (up to 16 KB)			*/
#define UNUSED(x)					(void)x


/*
 * Peripheral Base Address
 */

#define PERIPH_BASE_ADDR			(0x40000000UL)                		/* Base Address for All periherals		*/
#define APB1_BASE_ADDR				(PERIPH_BASE_ADDR)                 	/* APB1 Bus Domain Base Address			*/
#define APB2_BASE_ADDR				(PERIPH_BASE_ADDR + 0x10000UL)  	/* APB2 Bus Domain Base Address			*/
#define AHB1_BASE_ADDR				(PERIPH_BASE_ADDR + 0x20000UL)      /* AHB1 Bus Domain Base Address			*/
#define AHB2_BASE_ADDR				(PERIPH_BASE_ADDR + 0x10000000UL)   /* AHB2 Bus Domain Base Address			*/

/*
 * APB1  Periphereals Base Address
 */

#define TIM2_BASE_ADDR				(APB1_BASE_ADDR + 0x0000UL)		/* TIM2 Base Address 			*/
#define TIM3_BASE_ADDR				(APB1_BASE_ADDR + 0x0400UL)		/* TIM3 Base Address 			*/
#define TIM4_BASE_ADDR				(APB1_BASE_ADDR + 0x0800UL)		/* TIM4 Base Address 			*/
#define TIM5_BASE_ADDR				(APB1_BASE_ADDR + 0x0C00UL)		/* TIM5 Base Address 			*/
#define TIM6_BASE_ADDR				(APB1_BASE_ADDR + 0x1000UL)		/* TIM6 Base Address 			*/
#define TIM7_BASE_ADDR				(APB1_BASE_ADDR + 0x1400UL)		/* TIM7 Base Address 			*/

#define SPI2_BASE_ADDR				(APB1_BASE_ADDR + 0x3800UL)		/* SPI2 Base Address 			*/
#define SPI3_BASE_ADDR				(APB1_BASE_ADDR + 0x3C00UL)		/* SPI3 Base Address 			*/

#define USART2_BASE_ADDR			(APB1_BASE_ADDR + 0x4400UL)		/* USART2 Base Address 			*/
#define USART3_BASE_ADDR			(APB1_BASE_ADDR + 0x4800UL)		/* USART2 Base Address 			*/
#define UART4_BASE_ADDR			    (APB1_BASE_ADDR + 0x4C00UL)		/* UART4 Base Address 			*/
#define UART5_BASE_ADDR			    (APB1_BASE_ADDR + 0x5000UL)		/* UART5 Base Address 			*/

#define I2C1_BASE_ADDR			    (APB1_BASE_ADDR + 0x5400UL)		/* I2C1 Base Address 			*/
#define I2C2_BASE_ADDR			    (APB1_BASE_ADDR + 0x5800UL)		/* I2C2 Base Address 			*/
#define I2C3_BASE_ADDR			    (APB1_BASE_ADDR + 0x5C00UL)		/* I2C3 Base Address 			*/

/*
 * APB2  Periphereals Base Address
 */

#define	TIM1_BASE_ADDR				(APB2_BASE_ADDR + 0x0000UL)		/* TIM1 Base Address 			*/
#define	TIM8_BASE_ADDR				(APB2_BASE_ADDR + 0x0400UL)		/* TIM8 Base Address 			*/

#define USART1_BASE_ADDR			(APB2_BASE_ADDR + 0x1000UL)		/* USART1 Base Address 			*/
#define USART6_BASE_ADDR			(APB2_BASE_ADDR + 0x1400UL)		/* USART6 Base Address 			*/

#define SPI1_BASE_ADDR				(APB2_BASE_ADDR + 0x3000UL)		/* SPI1 Base Address 			*/
#define SPI4_BASE_ADDR				(APB2_BASE_ADDR + 0x3400UL)		/* SPI4 Base Address 			*/

#define SYSCFG_BASE_ADDR			(APB2_BASE_ADDR + 0x3800UL)		/* SYSCFG Base Address 			*/
#define EXTI_BASE_ADDR			    (APB2_BASE_ADDR + 0x3C00UL)	    /* EXTI Base Address 			*/

/*
 * AHB1  Periphereals Base Address
 */

#define GPIOA_BASE_ADDR			   (AHB1_BASE_ADDR + 0x0000UL)		/* GPIOA Base Address 			*/
#define GPIOB_BASE_ADDR			   (AHB1_BASE_ADDR + 0x0400UL)		/* GPIOB Base Address 			*/
#define GPIOC_BASE_ADDR			   (AHB1_BASE_ADDR + 0x0800UL)		/* GPIOC Base Address 			*/
#define GPIOD_BASE_ADDR			   (AHB1_BASE_ADDR + 0x0C00UL)		/* GPIOD Base Address 			*/
#define GPIOE_BASE_ADDR			   (AHB1_BASE_ADDR + 0x1000UL)		/* GPIOE Base Address 			*/
#define GPIOF_BASE_ADDR			   (AHB1_BASE_ADDR + 0x1400UL)		/* GPIOF Base Address 			*/
#define GPIOG_BASE_ADDR			   (AHB1_BASE_ADDR + 0x1800UL)		/* GPIOG Base Address 			*/
#define GPIOH_BASE_ADDR			   (AHB1_BASE_ADDR + 0x1C00UL)		/* GPIOH Base Address 			*/
#define RCC_BASE_ADDR         	   (AHB1_BASE_ADDR + 0x3800UL)		/*RCC Base Address				*/

/*
 * Peripheral Structure Definations
 */

/*
 * GPIO registers
 */
typedef struct{

	__IO uint32_t MODER;		/*!< GPIO Port Mode Register               Address Offset=0x0000 		*/
	__IO uint32_t OTYPE;		/*!< GPIO Port Output Type Register        Address Offset=0x0004 		*/
	__IO uint32_t OSPEEDR;		/*!< GPIO Port Speed Register			   Address Offset=0x0008 		*/
	__IO uint32_t PUPDR;		/*!< GPIO Port Pull-Up/Pull-Down Register  Address Offset=0x000C 	    */
	__IO uint32_t IDR;			/*!< GPIO Port input  Register             Address Offset=0x0010 		*/
	__IO uint32_t ODR;			/*!< GPIO Port Output Data Register        Address Offset=0x0014 		*/
	__IO uint32_t BSRR;			/*!< GPIO Port Bit Reset-Set Register      Address Offset=0x0018 		*/
	__IO uint32_t LCKR;			/*!< GPIO Port Lock Configuration Register Address Offset=0x001C 		*/
	__IO uint32_t AFR[2];		/*!< GPIO Port Alternate Function Register Address Offset=0x0020    	*/


}GPIO_Typedef_t;

/*
 * RCC registers
 */
typedef struct{

	__IO uint32_t CR;        	/*!< RCC Control Register Address Offset:0x0000 		*/
	__IO uint32_t PLLCFGR;		/*!< RCC PLLCFGR Register Address Offset:0x0004 		*/
	__IO uint32_t CFGR;			/*!< RCC CFGR Register Address Offset:0x0008 			*/
	__IO uint32_t CIR;			/*!< RCC CIR Register Address Offset:0x000C 			*/
	__IO uint32_t AHB1RSTR;		/*!< RCC AHB1RSTR Register Address Offset:0x0010 		*/
	__IO uint32_t AHB2RSTR; 	/*!< RCC AHB2RSTR Register Address Offset:0x0014 		*/
	__IO uint32_t AHB3RSTR;		/*!< RCC AHB3RSTR Register Address Offset:0x0018 		*/
	__IO uint32_t Reserved;		/*!< RCC Reserved Register Address Offset:0x001C 		*/
	__IO uint32_t APB1RSTR;		/*!< RCC APB1RSTR Register Address Offset:0x0020 		*/
	__IO uint32_t APB2RSTR;		/*!< RCC APB2RSTR Register Address Offset:0x0024 		*/
	__IO uint32_t Reserved2[2]; /*!< RCC Reserved2[2] Register Address Offset:0x0028 	*/
	__IO uint32_t AHB1ENR;		/*!< RCC AHB1ENR Register Address Offset:0x0030 		*/
	__IO uint32_t AHB2ENR;		/*!< RCC AHB2ENR Register Address Offset:0x0034 		*/
	__IO uint32_t AHB3ENR;		/*!< RCC AHB3ENR Register Address Offset:0x0038 		*/
	__IO uint32_t Reserved3;	/*!< RCC Reserved3 Register Address Offset:0x003C	    */
	__IO uint32_t APB1ENR;		/*!< RCC APB1ENR Register Address Offset:0x0040 		*/
	__IO uint32_t APB2ENR;		/*!< RCC APB2ENR Register Address Offset:0x0044 		*/
	__IO uint32_t Reserved4[2];	/*!< RCC Reserved4[2] Register Address Offset:0x0048 	*/
	__IO uint32_t AHB1LPENR;	/*!< RCC AHB1LPENR Register Address Offset:0x0050 		*/
	__IO uint32_t AHB2LPENR;    /*!< RCC AHB2LPENR Register Address Offset:0x0054 		*/
	__IO uint32_t AHB3LPENR;    /*!< RCC AHB3LPENR Register Address Offset:0x0058 		*/
	__IO uint32_t Reserved5;    /*!< RCC Reserved5 Register Address Offset:0x005C 		*/
	__IO uint32_t APB1LPENR;    /*!< RCC APB1LPENR Register Address Offset:0x0060 		*/
	__IO uint32_t APB2LPENR;    /*!< RCC APB2LPENR Register Address Offset:0x0064 		*/
	__IO uint32_t Reserved6[2]; /*!< RCC Reserved6[2] Register Address Offset:0x0068 	*/
	__IO uint32_t BDCR;			/*!< RCC BDCR Register Address Offset:0x0070 			*/
	__IO uint32_t CSR; 			/*!< RCC CSR Register Address Offset:0x0074 			*/
	__IO uint32_t Reserved7[2]; /*!< RCC Reserved7[2] Register Address Offset:0x0078 	*/
	__IO uint32_t SSCGR;		/*!< RCC SSCGR Register Address Offset:0x0080 			*/
	__IO uint32_t PLLLI2SCFGR;  /*!< RCC PLLLI2SCFGR Register Address Offset:0x0084 	*/

}RCC_typedef_t;


/*
 * SYSCFG registers
 */
typedef struct{

	__IO uint32_t MEMRMP;		/*!< SYSCFG memeory remap  Register               Address Offset=0x0000 		*/
	__IO uint32_t PMC;		    /*!< SYSCFG periphereal mode control Register     Address Offset=0x0004 		*/
	__IO uint32_t EXTI_CR[4];	/*!< SYSCFG exti control register Speed Register  Address Offset=0x0008 		*/
	__IO uint32_t CMPCR;		/*!< SYSCFG compensation sell control register    Address Offset=0x0020 	    */


}SYSCFG_Typedef_t;

/*
 * EXTI registers
 */
typedef struct{
	__IO uint32_t IMR;	    /*!< EXTI interrupt mask  Register                Address Offset=0x0000 	*/
	__IO uint32_t EMR;		/*!< EXTI event mask Register                     Address Offset=0x0004 	*/
	__IO uint32_t RTSR;	    /*!< EXTI rising trigger selection register       Address Offset=0x0008 	*/
	__IO uint32_t FTSR;		/*!< EXTI FALLİNG trigger selection register      Address Offset=0x000C	 	*/
	__IO uint32_t SWIER;    /*!< EXTI software ınterrupt register    		  Address Offset=0x0010      */
	__IO uint32_t PR;		/*!< EXTI pending register  					  Address Offset=0x0014      */

}EXTI_Typedef_t;



/*
 * SPI REGISTERS
 */
typedef struct{

	__IO uint32_t CR1;      /*!< SPI control register 1   		 Address Offset = 0x00   */
	__IO uint32_t CR2;      /*!< SPI control register 2  		 Address Offset = 0x04   */
	__IO uint32_t SR;       /*!< SPI status  register    		 Address Offset = 0x08   */
	__IO uint32_t DR;       /*!< SPI data register 1      		 Address Offset = 0x0C   */
	__IO uint32_t CRCPR;    /*!< SPI CRC polynomial register     Address Offset = 0x10   */
	__IO uint32_t RXCRCR;   /*!< SPI RX CRC register             Address Offset = 0x14   */
	__IO uint32_t TXCRCR;   /*!< SPI TX CRC register             Address Offset = 0x18   */
	__IO uint32_t I2SCFGR;  /*!< SPI_I2S configuration register  Address Offset = 0X1C   */
	__IO uint32_t I2SPR;    /*!< SPI_I2S prescaler register      Address Offset = 0x20   */


}SPI_TypeDef_t;

#define GPIOA 		  ((GPIO_Typedef_t*)(GPIOA_BASE_ADDR))
#define GPIOB         ((GPIO_Typedef_t*)(GPIOB_BASE_ADDR))
#define GPIOC 		  ((GPIO_Typedef_t*)(GPIOC_BASE_ADDR))
#define GPIOD 		  ((GPIO_Typedef_t*)(GPIOD_BASE_ADDR))
#define GPIOE 	      ((GPIO_Typedef_t*)(GPIOE_BASE_ADDR))

#define RCC		      ( (RCC_typedef_t*)(RCC_BASE_ADDR)    )

#define SYSCFG	      ((SYSCFG_Typedef_t*)(SYSCFG_BASE_ADDR))
#define EXTI	      ((EXTI_Typedef_t*)(EXTI_BASE_ADDR)    )

#define SPI1		  ((SPI_TypeDef_t*)(SPI1_BASE_ADDR)    )
#define SPI2	      ((SPI_TypeDef_t*)(SPI2_BASE_ADDR)    )
#define SPI3          ((SPI_TypeDef_t*)(SPI3_BASE_ADDR)    )



/*
 * AHB1 ENABLE
 */
#define RCC_AHB1ENR_GPIOAENPos		0U                           	/*!<  RCC AHB1ENR GPIOA Enable Position */
#define RCC_AHB1ENR_GPIOAENMsk		(1<<RCC_AHB1ENR_GPIOAENPos)     /*!<  RCC AHB1ENR GPIOA Enable Bit Mask */
#define RCC_AHB1ENR_GPIOAEN			RCC_AHB1ENR_GPIOAENMsk          /*!<  RCC AHB1ENR GPIOA Enable Macro 	*/

#define RCC_AHB1ENR_GPIOBENPos		1U                          	/*!<  RCC AHB1ENR GPIOB Enable Position  */
#define RCC_AHB1ENR_GPIOBENMsk		(1<<RCC_AHB1ENR_GPIOBENPos)     /*!<  RCC AHB1ENR GPIOB Enable Bit Mask  */
#define RCC_AHB1ENR_GPIOBEN			RCC_AHB1ENR_GPIOBENMsk          /*!<  RCC AHB1ENR GPIOB Enable Macro 	 */

#define RCC_AHB1ENR_GPIOCENPos		2U                            	/*!<  RCC AHB1ENR GPIOC Enable Position */
#define RCC_AHB1ENR_GPIOCENMsk		(1<<RCC_AHB1ENR_GPIOCENPos)     /*!<  RCC AHB1ENR GPIOC Enable Bit Mask */
#define RCC_AHB1ENR_GPIOCEN			RCC_AHB1ENR_GPIOCENMsk          /*!<  RCC AHB1ENR GPIOC Enable Macro 	*/

#define RCC_AHB1ENR_GPIODENPos     	3U                              /*!<  RCC AHB1ENR GPIOD Enable Position */
#define RCC_AHB1ENR_GPIODENMsk		(1<<RCC_AHB1ENR_GPIODENPos)     /*!<  RCC AHB1ENR GPIOD Enable Bit Mask */
#define RCC_AHB1ENR_GPIODEN			RCC_AHB1ENR_GPIODENMsk          /*!<  RCC AHB1ENR GPIOD Enable Macro 	*/

/*
 * APB2 ENABLE
 */
#define RCC_APB2ENR_SYSCFGENPos		14U                           	/*!<  RCC APB2ENR SYSCFG Enable Position */
#define RCC_APB2ENR_SYSCFGENMsk		(1<<RCC_APB2ENR_SYSCFGENPos)    /*!<  RCC APB2ENR SYSCFG Enable Bit Mask */
#define RCC_APB2ENR_SYSCFGEN		RCC_APB2ENR_SYSCFGENMsk         /*!<  RCC APB2ENR SYSCFG Enable Macro 	 */

#define RCC_APB2ENR_SPI1ENPos		12U                           /*!<  RCC APB2ENR SPI1 Enable Position */
#define RCC_APB2ENR_SPI1ENMsk		(1<<RCC_APB2ENR_SPI1ENPos)    /*!<  RCC APB2ENR SPI1 Enable Bit Mask */
#define RCC_APB2ENR_SPI1EN			RCC_APB2ENR_SPI1ENMsk         /*!<  RCC APB2ENR SPI1 Enable Macro 	 */

#define SPI_CR1_SPE					(6U)

/*
 * SPI FALAG DEFINATIONS
 */

#define SPI_CR1_SPE   	           (6U)
#define SPI_TxE 	               (1U)
#define SPI_Busy 	               (7U)
#define SPI_RxE 	               (0U)
#define SPI_CR2_TXIE               (7U)
#define SPI_CR1_DFF                (11U)
#define SPI_SR_TxE                 (5U)

#define SPI_CR1_SPE_FLAG 	       (0x1U<<SPI_CR1_SPE)
#define SPI_TxE_FLAG 	           (0x1U<< SPI_TxE)
#define SPI_Busy_FLAG 	           (0x1U<< SPI_Busy)
#define SPI_RxE_FLAG               (0x1U<< SPI_RxE)
#define SPI_SR_TxE_FLAG            (0x1U<< SPI_SR_TxE)

#include "rcc.h"
#include "gpio.h"
#include "exti.h"
#include "spi.h"

#endif /* INC_STM32F407XX_H_ */
