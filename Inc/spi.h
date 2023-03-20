/*
 * spi.h
 *
 *  Created on: 14 Ara 2022
 *      Author: SAMEDBASKİN
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

#include "stm32f407xx.h"

typedef enum{

	SPI_BUSY_FREE=0x0U,
	SPI_BUSY_TX=0x1U,
	SPI_BUSY_RX=0x2U


}SPI_BusStatus_t;
/*
* @def_group SPI_Baudrates
*/

#define SPI_BAUDRATE_DIV2    ((uint32_t)(0x00000000))
#define SPI_BAUDRATE_DIV4    ((uint32_t)(0x00000008))
#define SPI_BAUDRATE_DIV8    ((uint32_t)(0x00000010))
#define SPI_BAUDRATE_DIV16   ((uint32_t)(0x00000018))
#define SPI_BAUDRATE_DIV32   ((uint32_t)(0x00000020))
#define SPI_BAUDRATE_DIV64   ((uint32_t)(0x00000028))
#define SPI_BAUDRATE_DIV128  ((uint32_t)(0x00000030))

/*
* @def_group SPI_CPHA
*/
#define SPI_CPHA_FIRST       ((uint32_t)(0x00000000))
#define SPI_CPHA_SECOND      ((uint32_t)(0x00000001))

/*
* @def_group SPI_CPOL
*/
#define SPI_CPOL_LOW         ((uint32_t)(0x00000000))
#define SPI_CPOL_HIGH        ((uint32_t)(0x00000002))

/*
 * @def_group SPI_DFF_Format
 */
#define SPI_DFF_8BITS        ((uint32_t)(0x00000000))
#define SPI_DFF_16BITS       ((uint32_t)(0x00000800))

/*
 * @def_group SPI_MOde_values
*/
#define SPI_MODE_MASTER     ((uint32_t)(0x00000000))
#define SPI_MODE_SLAVE      ((uint32_t)(0x00000000))

/*
 * @def_group SPI_FRAMEFORMAT
 */
#define SPI_FRAMEFORMAT_MSB ((uint32_t)(0x00000000))
#define SPI_FRAMEFORMAT_LSB ((uint32_t)(0x00000080))

/*
 * @def_group SPI_BusConfig
*/

#define SPI_BUS_FullDuplex   		     ((uint32_t)(0x00000000))
#define SPI_BUS_FullDuplex_ReceiveOnly   ((uint32_t)(0x00000400))
#define SPI_BUS_HalfDuplex_Transmit      ((uint32_t)(0x0000C000))
#define SPI_BUS_HalfDuplex_Receive       ((uint32_t)(0x00008000))

/*
 * @def_group SSM_Values
 */

#define SPI_SSM_DISABLE 	 ((uint32_t)(0x00000000))
#define SPI_SSM_ENABLE  	 ((uint32_t)(0x00000300))

typedef enum{

	SPI_FLAG_RESET= 0x0U,
	SPI_FLAG_SET= !SPI_FLAG_RESET


}SPI_FlagStatus;

typedef struct{

	uint32_t Mode ;    		 /*!< Mode values for spi @def_group SPI_MOde_values*/
	uint32_t CPHA;     		 /*!< CPHA values for spi @def_group SPI_CPHA */
	uint32_t CPOL;      	 /*!< CPOL values for spi @def_group SPI_CPOL */
	uint32_t Baudrate;		 /*!< Baudrate values for spi @def_group SPI_Baudrates*/
	uint32_t SSM_Cmd; 		 /*!< Software @def_group SPI_Baudrates*/
	uint32_t DFF_Format;  	 /*!< dss values for spi @def_group SPI_DFF_Format*/
	uint32_t BusConfig; 	 /*!< BusConfig values for spi @def_group SPI_BusConfig*/
	uint32_t FrameFormat;	 /*!< FrameFormat values for spi @def_group SPI_FRAMEFORMAT */

}SPI_InitTypeDef_t;



typedef struct __SPI_HandleTypeDef_t{

	SPI_TypeDef_t *Instance;
	SPI_InitTypeDef_t Init;
	uint8_t  *pTxDataAdr;
	uint16_t TxDataSize;
	uint8_t busStateTx;
	void(*TxISRFunction)(struct __SPI_HandleTypeDef_t *SPI_Handle) ;

}SPI_HandleTypeDef_t;



void SPI_Init(SPI_HandleTypeDef_t *SPI_Handle);
void SPI_PeriphCmd(SPI_HandleTypeDef_t *SPI_Handle ,FunctionalState_t stateOfSPI);
void SPI_TransmitData(SPI_HandleTypeDef_t *SPI_Handle, uint8_t *pData,uint16_t sizeOfData);
SPI_FlagStatus SPI_GetFlagStatus(SPI_HandleTypeDef_t *SPI_Handle, uint16_t SPI_Flag);
void SPI_ReceiveData(SPI_HandleTypeDef_t *SPI_Handle, uint8_t *pBuffer,uint16_t sizeOfData);
void SPI_InterruptHandler(SPI_HandleTypeDef_t *SPI_Handle);




#endif /* INC_SPI_H_ */
