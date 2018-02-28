/*============================================================================*/
/*! \file   SPI_RF.c

    \author your name

    \brief  SPI module: This is the source file containing the SPI functionality.

   \version $Revision$
*/
/*============================================================================*/

/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/

#define __DECL_SPI_SPIRIT1_H__


#include <msp430.h>
#include "driverlib.h"
#include <SPI_SPIRIT1.h>
#include "globals.h"
#include "buffer.h"



/*==============================================================================
                                     MACROS
 =============================================================================*/


/*==============================================================================
                                     ENUMS
 =============================================================================*/


/*==============================================================================
                         STRUCTURES AND OTHER TYPEDEFS
 =============================================================================*/


/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/


/*==============================================================================
                                LOCAL CONSTANTS
 =============================================================================*/


/*==============================================================================
                           LOCAL FUNCTION PROTOTYPES
 =============================================================================*/


/*==============================================================================
                                LOCAL FUNCTIONS
 =============================================================================*/


/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/


/*============================================================================*/
/*!
    \brief   spi_initMaster_RF()
 	 	 	 init the SPI for the RF

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_initMaster_RF(void)
{

    //Set NSS of the RF as an output pin.
	GPIO_setAsOutputPin(
	   SPI_RF_GPIO_NSS,
	   SPI_RF_PIN_NSS
       );

    //Set NSS of the RF as Output High.
    GPIO_setOutputHighOnPin(
	   SPI_RF_GPIO_NSS,
	   SPI_RF_PIN_NSS
       );

    // Configure SPI pins
    // Configure Pins for UCB0CLK
    GPIO_setAsPeripheralModuleFunctionInputPin(
        SPI_RF_GPIO_CLK,
        SPI_RF_PIN_CLK,
        GPIO_SECONDARY_MODULE_FUNCTION
        );

    // Configure Pins for UCB0RXD/UCB0SOMI
    GPIO_setAsPeripheralModuleFunctionInputPin(
        SPI_RF_GPIO_SxMx,
        SPI_RF_PIN_SIMO + SPI_RF_PIN_SOMI,
        GPIO_SECONDARY_MODULE_FUNCTION
        );


    //Initialize Master
    EUSCI_A_SPI_initMasterParam param = {0};
    param.selectClockSource = EUSCI_A_SPI_CLOCKSOURCE_SMCLK;
    param.clockSourceFrequency = CS_getSMCLK();
    param.desiredSpiClock = 1000000;
    param.msbFirst = EUSCI_A_SPI_MSB_FIRST;
    param.clockPhase = EUSCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT;
    param.clockPolarity = EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW;
    param.spiMode = EUSCI_A_SPI_3PIN;
    EUSCI_A_SPI_initMaster(SPI_RF_BASE, &param);

    //Enable SPI module
    EUSCI_A_SPI_enable(SPI_RF_BASE);

    //Clear receive interrupt flag
    EUSCI_A_SPI_clearInterrupt(SPI_RF_BASE, SPI_RF_RECEIVE_INTERRUPT);



    // Enable SPI_RF_BASE RX interrupt
    EUSCI_A_SPI_enableInterrupt(SPI_RF_BASE, SPI_RF_RECEIVE_INTERRUPT);


}/* spi_initMaster_RF() */

/*============================================================================*/
/*!
    \brief   spi_init_RF()
 	 	 	 init the RF and the interrupt Pins for communication/flags

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_init_RF(void)
{
	SGpioInit gpio3_Init = {
			SPIRIT_GPIO_3,    /* Specifies the GPIO pins to be configured.
			                                        This parameter can be any value of @ref SpiritGpioPin */

			SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,  /* Specifies the operating mode for the selected pins.
			                                        This parameter can be a value of @ref SpiritGpioMode */

			SPIRIT_GPIO_DIG_OUT_IRQ      /* Specifies the I/O selection for the selected pins.
			                                        This parameter can be a value of @ref SpiritGpioIO */
	};

	SGpioInit gpio2_Init = {
			SPIRIT_GPIO_2,    /* Specifies the GPIO pins to be configured.
			                                        This parameter can be any value of @ref SpiritGpioPin */

			SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,  /* Specifies the operating mode for the selected pins.
			                                        This parameter can be a value of @ref SpiritGpioMode */

			SPIRIT_GPIO_DIG_OUT_TX_FIFO_ALMOST_FULL      /* Specifies the I/O selection for the selected pins.
			                                        This parameter can be a value of @ref SpiritGpioIO */
	};

	SGpioInit gpio1_Init = {
			SPIRIT_GPIO_1,    /* Specifies the GPIO pins to be configured.
			                                        This parameter can be any value of @ref SpiritGpioPin */

			SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,  /* Specifies the operating mode for the selected pins.
			                                        This parameter can be a value of @ref SpiritGpioMode */

			SPIRIT_GPIO_DIG_OUT_SLEEP_OR_STANDBY       /* Specifies the I/O selection for the selected pins.
			                                        This parameter can be a value of @ref SpiritGpioIO */
	};

	SGpioInit gpio0_Init = {
			SPIRIT_GPIO_0,    /* Specifies the GPIO pins to be configured.
			                                        This parameter can be any value of @ref SpiritGpioPin */

			SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,  /* Specifies the operating mode for the selected pins.
			                                        This parameter can be a value of @ref SpiritGpioMode */

			SPIRIT_GPIO_DIG_OUT_WUT_EXP     /* Specifies the I/O selection for the selected pins.
			                                        This parameter can be a value of @ref SpiritGpioIO */
	};


    SpiritBaseConfiguration();

	SpiritCmdStrobeSabort();

	do
	{
		SpiritRefreshStatus();
	}while(g_xStatus.MC_STATE!=MC_STATE_READY);

	SpiritVcoCalibration();

    /* Spirit IRQs enable */
    SpiritIrqDeInit(NULL);
//    SpiritIrq(RX_DATA_READY, S_ENABLE);
    SpiritIrq(TX_DATA_SENT, S_ENABLE);
//    SpiritIrq(RX_DATA_DISC, S_ENABLE);
//    SpiritIrq(READY, S_ENABLE);
//    SpiritIrq(STANDBY_DELAYED, S_ENABLE);
//    SpiritIrq(LOCK, S_ENABLE);
//    SpiritIrq(AES_END, S_ENABLE);

    /* Init the GPIO-Pin of the RF*/
    SpiritGpioInit(&gpio3_Init);
    SpiritGpioInit(&gpio2_Init);
    SpiritGpioInit(&gpio1_Init);
    SpiritGpioInit(&gpio0_Init);



    //make the MCU Pins as "only" inputs -> disable the pulldown
	GPIO_setAsInputPin(SPI_RF_GPIO_0_INT, SPI_RF_PIN_0_INT);
	GPIO_setAsInputPin(SPI_RF_GPIO_1_INT, SPI_RF_PIN_1_INT);
	GPIO_setAsInputPin(SPI_RF_GPIO_2_INT, SPI_RF_PIN_2_INT);
	GPIO_setAsInputPin(SPI_RF_GPIO_3_INT, SPI_RF_PIN_3_INT);

    // Configure the Interrupt Edge */
    GPIO_selectInterruptEdge(SPI_RF_GPIO_0_INT, SPI_RF_PIN_0_INT, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_selectInterruptEdge(SPI_RF_GPIO_1_INT, SPI_RF_PIN_1_INT, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_selectInterruptEdge(SPI_RF_GPIO_2_INT, SPI_RF_PIN_2_INT, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_selectInterruptEdge(SPI_RF_GPIO_3_INT, SPI_RF_PIN_3_INT, GPIO_HIGH_TO_LOW_TRANSITION);

    //P1.1 IFG cleared
    GPIO_clearInterrupt(SPI_RF_GPIO_0_INT, SPI_RF_PIN_0_INT);
    GPIO_clearInterrupt(SPI_RF_GPIO_1_INT, SPI_RF_PIN_1_INT);
    GPIO_clearInterrupt(SPI_RF_GPIO_2_INT, SPI_RF_PIN_2_INT);
    GPIO_clearInterrupt(SPI_RF_GPIO_3_INT, SPI_RF_PIN_3_INT);

    //P1.1 interrupt enabled
//    GPIO_enableInterrupt(SPI_RF_GPIO_0_INT, SPI_RF_PIN_0_INT);
//    GPIO_enableInterrupt(SPI_RF_GPIO_1_INT, SPI_RF_PIN_1_INT);
//    GPIO_enableInterrupt(SPI_RF_GPIO_2_INT, SPI_RF_PIN_2_INT);
    GPIO_enableInterrupt(SPI_RF_GPIO_3_INT, SPI_RF_PIN_3_INT);

    /* IRQ registers blanking */
    SpiritIrqClearStatus();
}

/*============================================================================*/
/*!
    \brief   spi_Deinit_RF()
 	 	 	 Deinit the RF and the interrupt Pins for communication/flags

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_Deinit_RF(void)
{

    /* Spirit IRQs enable */
    SpiritIrqDeInit(NULL);


    //make the MCU Pins as inputs with pulldown
	GPIO_setAsInputPinWithPullDownResistor(SPI_RF_GPIO_0_INT, SPI_RF_PIN_0_INT);
	GPIO_setAsInputPinWithPullDownResistor(SPI_RF_GPIO_1_INT, SPI_RF_PIN_1_INT);
	GPIO_setAsInputPinWithPullDownResistor(SPI_RF_GPIO_2_INT, SPI_RF_PIN_2_INT);
	GPIO_setAsInputPinWithPullDownResistor(SPI_RF_GPIO_3_INT, SPI_RF_PIN_3_INT);


    //P1.1 IFG cleared
    GPIO_clearInterrupt(SPI_RF_GPIO_0_INT, SPI_RF_PIN_0_INT);
    GPIO_clearInterrupt(SPI_RF_GPIO_1_INT, SPI_RF_PIN_1_INT);
    GPIO_clearInterrupt(SPI_RF_GPIO_2_INT, SPI_RF_PIN_2_INT);
    GPIO_clearInterrupt(SPI_RF_GPIO_3_INT, SPI_RF_PIN_3_INT);

    //P1.1 interrupt disable
    GPIO_disableInterrupt(SPI_RF_GPIO_0_INT, SPI_RF_PIN_0_INT);
    GPIO_disableInterrupt(SPI_RF_GPIO_1_INT, SPI_RF_PIN_1_INT);
    GPIO_disableInterrupt(SPI_RF_GPIO_2_INT, SPI_RF_PIN_2_INT);
    GPIO_disableInterrupt(SPI_RF_GPIO_3_INT, SPI_RF_PIN_3_INT);

    /* IRQ registers blanking */
    SpiritIrqClearStatus();
}


/*============================================================================*/
/*!
    \brief   spi_DeinitMaster_RF()
 	 	 	 disable the SPI for the RF

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_DeinitMaster_RF(void)
{
    EUSCI_A_SPI_disableInterrupt(SPI_RF_BASE, SPI_RF_TRANSMIT_INTERRUPT);
    EUSCI_A_SPI_disableInterrupt(SPI_RF_BASE, SPI_RF_RECEIVE_INTERRUPT);

    EUSCI_A_SPI_disable(SPI_RF_BASE);

}/* spi_DeinitMaster_RF() */


/*============================================================================*/
/*!
    \brief   spi_checkFIFO_IRQ_RF()
 	 	 	 check the FIFO of IRQ_RF

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_checkFIFO_IRQ_RF(void)
{
	uint8_t tmp;
	uint8_t cRxData;
	uint8_t vectcRxBuff[96];

	if(CircularBuffer_Out(&tmp, &FIFO_IRQ_RF) == BUFFER_SUCCESS)
	{
		if(tmp == 0xAA)
		{
			//load the Status Registers
			SpiritIrqs irqStatus;
			SpiritIrqGetStatus(&irqStatus);

			//check the Status Registers and do something!
			//after this, clear the Flag
			if((irqStatus.IRQ_RX_DATA_READY) == true)
			{
				/* Get the RX FIFO size */
				cRxData=SpiritLinearFifoReadNumElementsRxFifo();

				/* Read the RX FIFO */
				SpiritSpiReadLinearFifo(cRxData, &(vectcRxBuff[0]));

				/* Flush the RX FIFO */
				SpiritCmdStrobeFlushRxFifo();

				/* if no ack has been request from the tx put the device in Rx now */
				if(SpiritPktStackGetReceivedNackRx()!=0)
				{
					SpiritCmdStrobeRx();
				} else
				{
					/* go to ready state */
					SpiritCmdStrobeSabort();
				}

			}

			if((irqStatus.IRQ_RX_DATA_DISC) == true)
			{

				/* Get the RX FIFO size */
				cRxData=SpiritLinearFifoReadNumElementsRxFifo();

				/* Read the RX FIFO */
				SpiritSpiReadLinearFifo(cRxData, &(vectcRxBuff[0]));

				/* Flush the RX FIFO */
				SpiritCmdStrobeFlushRxFifo();

				/* go to ready state */
				SpiritCmdStrobeSabort();

			}

			if((irqStatus.IRQ_TX_DATA_SENT) == true)
			{
				/* set the send flag */
				x_data_sent_flag = 1;

				//flush the TX FIFO
				SpiritCmdStrobeFlushTxFifo();


				//Put it in Ready-Mode
				SpiritCmdStrobeSabort();

			}

			if((irqStatus.IRQ_WKUP_TOUT_LDC) == true)
			{


			}

			if((irqStatus.IRQ_READY) == true)
			{


			}

			if((irqStatus.IRQ_STANDBY_DELAYED) == true)
			{


			}

			if((irqStatus.IRQ_LOCK) == true)
			{

			}

			if((irqStatus.IRQ_AES_END) == true)
			{


			}

			if((irqStatus.IRQ_RX_FIFO_ERROR) == true)
			{
				/* Flush the RX FIFO */
				SpiritCmdStrobeFlushRxFifo();
			}

			if((irqStatus.IRQ_TX_FIFO_ERROR) == true)
			{
				/* Flush the RX FIFO */
//				SpiritCmdStrobeFlushTxFifo();
			}



			SpiritIrqClearStatus();

		}
	}
} /* spi_checkFIFO_IRQ_RF() */


/*============================================================================*/
/*!
    \brief   spi_getRF_FIFO()
 	 	 	 get Data of the FIFO

    \param	 Pointer for data, number of Bytes.

	\return  SPIRIT status
*/
/*============================================================================*/
StatusBytesRF spi_getRF_FIFO(uint8_t* tmp, uint8_t nBytes)
{
	StatusBytesRF status;
	status = spi_getRF_Data(&(tmp[0]), LINEAR_FIFO_ADDRESS, nBytes);

	return(status);
}

/*============================================================================*/
/*!
    \brief   spi_setRF_FIFO()
 	 	 	 set Data to the FIFO

    \param	 Pointer for data, number of Bytes.

	\return  SPIRIT status
*/
/*============================================================================*/
StatusBytesRF spi_setRF_FIFO(uint8_t* tmp, uint8_t nBytes)
{
	StatusBytesRF status;
	status = spi_setRF_Data(&(tmp[0]), LINEAR_FIFO_ADDRESS, nBytes);

	return(status);
}

/*============================================================================*/
/*!
    \brief   spi_getRF_Data()
 	 	 	 get Data of the RF

    \param	 Pointer for data, address of RF-Register, number of Bytes.

	\return  SPIRIT status
*/
/*============================================================================*/
StatusBytesRF spi_getRF_Data(uint8_t* tmp, uint8_t address, uint8_t nBytes)
{
	StatusBytesRF status;

	//wait until all data are send

	while(Buffer128_allData(&Buffer_RF) == false)
	{
//		__bis_SR_register(LPM0_bits + GIE);      // CPU off
//		__no_operation();
		__delay_cycles(100);
	}

	Buffer128_clean(&Buffer_RF);


	Buffer_RF.data[0] = READ_HEADER; 		//set READ_HEADER
	Buffer_RF.data[1] = address; 		//set address
	//Number of rx data
	//two more Byte, because two byte will be only TX
	Buffer_RF.dataLength = nBytes + 2;

	spi_startRF_communicationLPM();

//	__delay_cycles(10);

	while(Buffer128_allData(&Buffer_RF) == false)
	{
	}


	//copy data to tmp
	memcpy(&tmp[0], &(Buffer_RF.data[2]), nBytes);

	((uint8_t*)&status)[1]=Buffer_RF.data[0];
	((uint8_t*)&status)[0]=Buffer_RF.data[1];

  return status;

}/* spi_getRF_Data */



/*============================================================================*/
/*!
    \brief   spi_setRF_Data()
 	 	 	 send Data to the RF

    \param	 Pointer of data, address of RF-Register (write offset will be set), number of Bytes.

	\return  SPIRIT status
*/
/*============================================================================*/
StatusBytesRF spi_setRF_Data(uint8_t* tmp, uint8_t address, uint8_t nBytes)
{
	StatusBytesRF status;
	//wait until all data are send
	while(Buffer128_allData(&Buffer_RF) == false)
	{
//		__bis_SR_register(LPM0_bits + GIE);      // CPU off
//		__no_operation();
		__delay_cycles(100);
	}

	Buffer128_clean(&Buffer_RF);

	Buffer_RF.data[0] = WRITE_HEADER; 		//set READ_HEADER
	Buffer_RF.data[1] = address; 		//set address
	//Number of rx data
	//two more Byte, because two byte will be only address
	Buffer_RF.dataLength = nBytes + 2;

	//copy data from tmp
	memcpy(&(Buffer_RF.data[2]), &(tmp[0]), nBytes);

	spi_startRF_communication();


	((uint8_t*)&status)[1]=Buffer_RF.data[0];
	((uint8_t*)&status)[0]=Buffer_RF.data[1];

  return status;

}/* spi_setRF_Data */


/*============================================================================*/
/*!
    \brief   spi_setRF_Command()
 	 	 	 Send a command

    \param	 cCommandCode: command code to be sent

	\return  SPIRIT status
*/
/*============================================================================*/
StatusBytesRF spi_setRF_Command(uint8_t cCommandCode)
{
  StatusBytesRF status;

	//wait until all data are send
  while(Buffer128_allData(&Buffer_RF) == false)
	{
		__bis_SR_register(LPM0_bits + GIE);      // CPU off
		__no_operation();
		__delay_cycles(100);
	}

	Buffer128_clean(&Buffer_RF);

	Buffer_RF.data[0] = COMMAND_HEADER; 		//set COMMAND_HEADER
	Buffer_RF.data[1] = cCommandCode; 		//set cCommandCode
	Buffer_RF.dataLength = 2;				//two Bytes will be send


	spi_startRF_communicationLPM();


	//wait until communication is finished
	while(Buffer128_allData(&Buffer_RF) == false)
	{
	}

	((uint8_t*)&status)[1]=Buffer_RF.data[0];
	((uint8_t*)&status)[0]=Buffer_RF.data[1];

  return status;
}


/*============================================================================*/
/*!
    \brief   spi_startRF_communication(
 	 	 	 put the data of the Buffer_RF to SPI, handel the communication in the ISR

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_startRF_communication(void)
{
	/*
	 * starts the communication to the RF
	 * send data to the RF
	 */

	if(Buffer128_NotEmpty(&Buffer_RF) == false)
	{
		while(EUSCI_A_SPI_getInterruptStatus(SPI_RF_BASE,
												SPI_RF_TRANSMIT_INTERRUPT) == false)
		{
		}


		//set NSS Pin to 0 -> start communication
		SPI_enable_RF_NSS();

		//send data
		EUSCI_A_SPI_transmitData(SPI_RF_BASE, Buffer_RF.data[0]);
		//send other Data is finished in the ISR

		//check if the first two bytes are received -> status of the RF-Chip
		while(Buffer_RF.counter < 2)
		{
		}
	}

}/* spi_spi_startRF_communication() */


/*============================================================================*/
/*!
    \brief   spi_startRF_communicationLPM()
 	 	 	 put the data of the Buffer_RF to SPI, handels the communication
 	 	 	 Go to LPM0 and wake up, if the communication is complete!

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_startRF_communicationLPM(void)
{
	/*
	 * starts the communication to the RF
	 * send data to the RF
	 */


	if(Buffer128_NotEmpty(&Buffer_RF) == false)
	{
		while(EUSCI_A_SPI_getInterruptStatus(SPI_RF_BASE, SPI_RF_TRANSMIT_INTERRUPT) == false)
		{
		}

		//set NSS Pin to 0 -> start communication
		SPI_enable_RF_NSS();

		//send data
		EUSCI_A_SPI_transmitData(SPI_RF_BASE, Buffer_RF.data[0]);
		//send other Data is handelt in the ISR


		__bis_SR_register(LPM0_bits + GIE);      // CPU off
		__no_operation();
	}

	/* Errata of the Chip: busy should not be used! */
	/*
	while(EUSCI_A_SPI_isBusy(SPI_RF_BASE) != 0)
	{

	}
	*/

}/* spi_startRF_communication() */

