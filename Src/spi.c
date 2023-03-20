
/*
 * spi.c
 *
 *  Created on: 14 Ara 2022
 *      Author: SAMEDBASKİN
 */

#include "spi.h"

static void SPI_CloseISR_TX(SPI_HandleTypeDef_t *SPI_Handle){

	SPI_Handle->Instance->CR2 &= (0x1<<SPI_CR2_TXIE);
	SPI_Handle->TxDataSize=0;
	SPI_Handle->pTxDataAdr=NULL;
	SPI_Handle->busStateTx=SPI_BUSY_FREE;

}

static void SPI_TransmitHelper_16_bits(SPI_HandleTypeDef_t *SPI_Handle){

	SPI_Handle->Instance->DR = *( (uint16_t*)SPI_Handle->pTxDataAdr );
	SPI_Handle->pTxDataAdr+=2;
	SPI_Handle->TxDataSize-=2;

	 if( SPI_Handle->TxDataSize==0){

	    SPI_CloseISR_TX(SPI_Handle);

	    }
}

static void SPI_TransmitHelper_8_bits(SPI_HandleTypeDef_t *SPI_Handle){

	SPI_Handle->Instance->DR = *( (uint8_t*)SPI_Handle->pTxDataAdr );
	SPI_Handle->pTxDataAdr+=1;
    SPI_Handle->TxDataSize-=1;

    if( SPI_Handle->TxDataSize==0){

    	SPI_CloseISR_TX(SPI_Handle);

    }

}
 /*
 * @brief SPI_Init SPI peripherals ınit func
 *
 * @param SPI_Handle : User config structure
 *
 * @retval void
 */

void SPI_Init(SPI_HandleTypeDef_t *SPI_Handle){

	uint32_t tempValue=0;

	tempValue = SPI_Handle->Instance->CR1;

	tempValue |= ( SPI_Handle->Init.Baudrate ) | ( SPI_Handle->Init.CPHA ) | ( SPI_Handle->Init.CPOL)
		      | ( SPI_Handle->Init.DFF_Format) |  ( SPI_Handle->Init.Mode) |  ( SPI_Handle->Init.FrameFormat)
			  | ( SPI_Handle->Init.BusConfig)  | ( SPI_Handle->Init.SSM_Cmd);

	SPI_Handle->Instance->CR1=tempValue ;
}



/*
 * @brief SPI_PeriphCmd enable or disable spı peripheral
 *
 * @param SPI_Handle : User config structure
 *
 *@param stateOfSPI : enable or disable
 *
 * @retval void
 */

void SPI_PeriphCmd(SPI_HandleTypeDef_t *SPI_Handle ,FunctionalState_t stateOfSPI){

	if(stateOfSPI== ENABLE){
		SPI_Handle->Instance->CR1 |=  (0x1U<<SPI_CR1_SPE);
	}else{
		SPI_Handle->Instance->CR1 &= ~(0x1U<<SPI_CR1_SPE);
	}

}

/*
 * @brief SPI_TransmitData Transmits data to the slave
 *
 * @param SPI_Handle : User config structure
 *
 *@param *pData : address of data send
 *
 *@param  sizeOfData lenght of your data in byte
 *
 * @retval void
 */

void SPI_TransmitData(SPI_HandleTypeDef_t *SPI_Handle, uint8_t *pData, uint16_t sizeOfData){


	if(SPI_Handle->Init.DFF_Format==SPI_DFF_16BITS){

		while(sizeOfData>0){

			if( SPI_GetFlagStatus(SPI_Handle, SPI_TxE_FLAG) ){
				SPI_Handle->Instance->DR = *( (uint16_t*)pData );
				*pData+=2;
				sizeOfData-=2;
			}
		}

	}else{

		while(sizeOfData>0){

			if( SPI_GetFlagStatus(SPI_Handle, SPI_TxE_FLAG) ){
				SPI_Handle->Instance->DR = *pData;
				*pData+=1;
				sizeOfData--;
			}
		}
	}

	while( SPI_GetFlagStatus(SPI_Handle, SPI_Busy_FLAG) );
}


/*
 * @brief SPI_GetFlagStatus return to flag of sr register
 *
 * @param SPI_Handle : User config structure
 * @param SPI_Flag : selected spı flag
 *
 * @retval SPI_FlagStatus
 */


SPI_FlagStatus SPI_GetFlagStatus(SPI_HandleTypeDef_t *SPI_Handle, uint16_t SPI_Flag){


	return  ( SPI_Handle->Instance->SR & SPI_Flag ) ? SPI_FLAG_SET :SPI_FLAG_RESET;

}

/*
 * @brief SPI_ReceiveData Recieve data to the slave
 *
 * @param SPI_Handle : User config structure
 *
 *@param *pData : address of data send
 *
 *@param  sizeOfData lenght of your data in byte
 *
 * @retval void
 */
void SPI_ReceiveData(SPI_HandleTypeDef_t *SPI_Handle, uint8_t *pBuffer,uint16_t sizeOfData){

	if(SPI_Handle->Init.DFF_Format==SPI_DFF_16BITS){

		while(sizeOfData>0){

			if( SPI_GetFlagStatus(SPI_Handle, SPI_RxE_FLAG) ){
				*( (uint16_t*)pBuffer )=(uint16_t)SPI_Handle->Instance->DR;
				pBuffer+=2;
				sizeOfData-=2;
				}
			}

	}else{
		while(sizeOfData>0){

		  if( SPI_GetFlagStatus(SPI_Handle, SPI_RxE_FLAG) ){
				*(pBuffer )=*((__IO uint8_t*)&SPI_Handle->Instance->DR);
				 pBuffer+=1;
			    sizeOfData-=1;
				}
		}
	}

}

void SPI_TransmitData_IT(SPI_HandleTypeDef_t *SPI_Handle, uint8_t *pData, uint16_t sizeOfData){

	SPI_BusStatus_t busState = SPI_Handle->busStateTx;

	if(busState!=SPI_BUSY_TX){
		SPI_Handle->pTxDataAdr = (uint8_t*)pData;
		SPI_Handle->TxDataSize= (uint16_t)sizeOfData;
		SPI_Handle->busStateTx=SPI_BUSY_TX;

		if(SPI_Handle->Instance->CR1 & (0x1U<<SPI_CR1_DFF)){

			SPI_Handle->TxISRFunction =SPI_TransmitHelper_16_bits;

		}else{

			SPI_Handle->TxISRFunction =SPI_TransmitHelper_8_bits;

		}
		SPI_Handle->Instance->CR2 |= (0x1U   << SPI_CR2_TXIE);
	}


}
void SPI_InterruptHandler(SPI_HandleTypeDef_t *SPI_Handle){

	uint8_t source=0;
	uint8_t flag=0;

	source=SPI_Handle->Instance->CR2 & (0x1U << SPI_CR2_TXIE);
	flag=SPI_Handle->Instance->SR & (0x1U << SPI_SR_TxE);

	if( (source!=0) &&  (flag!=0) ){
		SPI_Handle->TxISRFunction(SPI_Handle);
	}

}
