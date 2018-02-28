/*============================================================================*/
/*! \file   spi.c

    \author your name

    \brief  SPI module: This is the source file containing the SPI functionality.

   \version $Revision$
*/
/*============================================================================*/

/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/

#define __DECL_SPI_SENSOR_H__


#include <msp430.h>
#include "driverlib.h"
#include <SPI_Sensor.h>
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
    \brief   spi_initMaster_Sensor()
 	 	 	 init the SPI for the Sensors

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_initMaster_Sensor(void)
{

	//Pins are defined by enabling the VCC of the Sensors


    //Initialize Master
    EUSCI_A_SPI_initMasterParam param = {0};
    param.selectClockSource = EUSCI_A_SPI_CLOCKSOURCE_SMCLK;
    param.clockSourceFrequency = CS_getSMCLK();
    param.desiredSpiClock = 1000000;
    param.msbFirst = EUSCI_A_SPI_MSB_FIRST;
    param.clockPhase = EUSCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT;
    param.clockPolarity = EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW;
    param.spiMode = EUSCI_A_SPI_3PIN;
    EUSCI_A_SPI_initMaster(SPI_Sensor_BASE, &param);

    //Enable SPI module
    EUSCI_A_SPI_enable(SPI_Sensor_BASE);

    //Clear receive interrupt flag
    EUSCI_A_SPI_clearInterrupt(SPI_Sensor_BASE, SPI_Sensor_RECEIVE_INTERRUPT);



    // Enable SPI_Sensor_BASE RX interrupt
    EUSCI_A_SPI_enableInterrupt(SPI_Sensor_BASE, SPI_Sensor_RECEIVE_INTERRUPT);


}/* spi_initMaster_Sensor() */


/*============================================================================*/
/*!
    \brief   spi_DeinitMaster_Sensor()
 	 	 	 disable the SPI for the Sensor

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_DeinitMaster_Sensor(void)
{
    EUSCI_A_SPI_disableInterrupt(SPI_Sensor_BASE,	SPI_Sensor_TRANSMIT_INTERRUPT);
    EUSCI_A_SPI_disableInterrupt(SPI_Sensor_BASE, SPI_Sensor_RECEIVE_INTERRUPT);

    EUSCI_A_SPI_disable(SPI_Sensor_BASE);

}/* spi_DeinitMaster_Sensor() */


/*============================================================================*/
/*!
    \brief   spi_enable_Sensor_VCC()
 	 	 	 enable the Sensor VCC and set the SPI-Pins for the communication

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_enable_Sensor_VCC(void)
{
    //PowerUp the Sensors
    GPIO_setOutputHighOnPin(SPI_Sensor_GPIO_VCC, SPI_Sensor_Pin_VCC);


    // Configure SPI pins
    // Configure Pins for UCA1CLK
    GPIO_setAsPeripheralModuleFunctionInputPin(
        SPI_Sensor_GPIO_CLK,
        SPI_Sensor_PIN_CLK,
        GPIO_SECONDARY_MODULE_FUNCTION
        );

    // Configure Pins for UCA1SOMI and UCA1SIMO
    GPIO_setAsPeripheralModuleFunctionInputPin(
        SPI_Sensor_GPIO_SxMx,
        SPI_Sensor_PIN_SIMO + SPI_Sensor_PIN_SOMI,
        GPIO_SECONDARY_MODULE_FUNCTION
        );

    //disable all Pins
    SPI_disable_Sensor1_NSS();
    SPI_disable_Sensor2_NSS();
    SPI_disable_Sensor3_NSS();

    //enable the output
	GPIO_setAsOutputPin(SPI_Sensor1_GPIO_NSS, SPI_Sensor1_PIN_NSS);
	GPIO_setAsOutputPin(SPI_Sensor2_GPIO_NSS, SPI_Sensor2_PIN_NSS);
	GPIO_setAsOutputPin(SPI_Sensor3_GPIO_NSS, SPI_Sensor3_PIN_NSS);




}/* spi_enable_Sensor_VCC() */

/*============================================================================*/
/*!
    \brief   spi_disable_Sensor_VCC()
 	 	 	 disable the Sensor VCC and set the SPI-Pins for the communication

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_disable_Sensor_VCC(void)
{
    //PowerDown the Sensors
    GPIO_setOutputLowOnPin(SPI_Sensor_GPIO_VCC, SPI_Sensor_Pin_VCC);

    // Configure SPI pins
    GPIO_setAsInputPinWithPullDownResistor(SPI_Sensor_GPIO_CLK, SPI_Sensor_PIN_CLK);

    // Configure Pins for UCA1SOMI and UCA1SIMO
    GPIO_setAsInputPinWithPullDownResistor(SPI_Sensor_GPIO_SxMx, SPI_Sensor_PIN_SIMO + SPI_Sensor_PIN_SOMI);


	//NSS Pins for the sensors -> set as input with pullDown -> if used, set as output!
		//pullDown: the Sensors are not powered -> if pullUp, the Sensors will PowerUp with the current of the PullUp!!!
	GPIO_setAsInputPinWithPullDownResistor(SPI_Sensor1_GPIO_NSS, SPI_Sensor1_PIN_NSS);
	GPIO_setAsInputPinWithPullDownResistor(SPI_Sensor2_GPIO_NSS, SPI_Sensor2_PIN_NSS);
	GPIO_setAsInputPinWithPullDownResistor(SPI_Sensor3_GPIO_NSS, SPI_Sensor3_PIN_NSS);

}/* spi_disable_Sensor_VCC() */


/*============================================================================*/
/*!
    \brief   spi_getSensor3_Data()
 	 	 	 get Data of the Sensor (TMP123)

    \param	 Pointer for data (uint16_t)

	\return  None.
*/
/*============================================================================*/
void spi_getSensor3_Data(uint16_t* tmp)
{
	/*
	 * read the current data
	 */

	//wait until all data are send
	while(Buffer128_allData(&Buffer_Sensor) == false)
	{
		__bis_SR_register(LPM0_bits + GIE);      // CPU off
		__no_operation();
		__delay_cycles(10);
	}

	Buffer128_clean(&Buffer_Sensor);

	//Number of rx data
	//2 Byte (one 16Bit) -> Sensor create a 12Bit value
	Buffer_Sensor.dataLength = 2;

	spi_startSensor_communicationLPM(SPI_Sensor3);


	while(Buffer128_allData(&Buffer_Sensor) == false)
	{
	}

	//copy data to tmp
	tmp[0] = (uint16_t) ((Buffer_Sensor.data[0] << 8) + (Buffer_Sensor.data[1]));	//make the 16Bit value

}/* spi_getSensor_Data */


/*============================================================================*/
/*!
    \brief   SPI_enable_Sensor_NSS()
 	 	 	 enable the needed Sensor

    \param	 select the Sensor
    			0xFF: all Sensors are disabled

	\return  None.
*/
/*============================================================================*/
void SPI_enable_Sensor_NSS(uint8_t selectSensor)
{
	switch(selectSensor)
	{
		case SPI_Sensor1: //enable the correct Pin and disable the others
			SPI_enable_Sensor1_NSS();
			SPI_disable_Sensor2_NSS();
			SPI_disable_Sensor3_NSS();
			break;

		case SPI_Sensor2: //enable the correct Pin and disable the others
			SPI_disable_Sensor1_NSS();
			SPI_enable_Sensor2_NSS();
			SPI_disable_Sensor3_NSS();
			break;

		case SPI_Sensor3: //enable the correct Pin and disable the others
			SPI_disable_Sensor1_NSS();
			SPI_disable_Sensor2_NSS();

			GPIO_setAsInputPin(SPI_Sensor_GPIO_SxMx, SPI_Sensor_PIN_SIMO);		//disable the output of the pin!
			SPI_enable_Sensor3_NSS();
			break;

		default:	//disable all Pins
			SPI_disable_Sensor1_NSS();
			SPI_disable_Sensor2_NSS();
			SPI_disable_Sensor3_NSS();

		    // Configure Pins for UCA1SOMI and UCA1SIMO
		    GPIO_setAsPeripheralModuleFunctionInputPin(SPI_Sensor_GPIO_SxMx, SPI_Sensor_PIN_SIMO + SPI_Sensor_PIN_SOMI, GPIO_SECONDARY_MODULE_FUNCTION);
			break;
	}

}


/*============================================================================*/
/*!
    \brief   spi_startSensor_communication()
 	 	 	 put the data of the Buffer_Sensor to SPI, handel the communication in the ISR

    \param	 select the Sensor

	\return  None.
*/
/*============================================================================*/
void spi_startSensor_communication(uint8_t selectSensor)
{
	/*
	 * starts the communication to the Sensor
	 * send data to the Sensor
	 */

	if(Buffer128_NotEmpty(&Buffer_Sensor) == false)
	{
		while(EUSCI_A_SPI_getInterruptStatus(SPI_Sensor_BASE,
												SPI_Sensor_TRANSMIT_INTERRUPT) == false)
		{
		}


		//set NSS Pin -> start communication
		SPI_enable_Sensor_NSS(selectSensor);

		//send data
		EUSCI_A_SPI_transmitData(SPI_Sensor_BASE, Buffer_Sensor.data[0]);
		//send other Data is finished in the ISR

		//check if the first two bytes are received -> status of the Sensor-Chip
		while(Buffer_Sensor.counter < 2)
		{
		}
	}

}/* spi_spi_startSensor_communication() */


/*============================================================================*/
/*!
    \brief   spi_startSensor_communicationLPM()
 	 	 	 put the data of the Buffer_Sensor to SPI, handels the communication
 	 	 	 Go to LPM0 and wake up, if the communication is complete!

    \param	 select the Sensor

	\return  None.
*/
/*============================================================================*/
void spi_startSensor_communicationLPM(uint8_t selectSensor)
{
	/*
	 * starts the communication to the Sensor
	 * send data to the Sensor
	 */


	if(Buffer128_NotEmpty(&Buffer_Sensor) == false)
	{
		while(EUSCI_A_SPI_getInterruptStatus(SPI_Sensor_BASE,
												SPI_Sensor_TRANSMIT_INTERRUPT) == false)
		{
		}


		//set NSS Pin -> start communication
		SPI_enable_Sensor_NSS(selectSensor);

		//send data
		EUSCI_A_SPI_transmitData(SPI_Sensor_BASE, Buffer_Sensor.data[0]);
		//send other Data is handelt in the ISR


		__bis_SR_register(LPM0_bits + GIE);      // CPU off
		__no_operation();
	}

	/* Errata of the Chip: busy should not be used! */
	/*
	while(EUSCI_A_SPI_isBusy(SPI_Sensor_BASE) != 0)
	{

	}
	*/

}/* spi_startSensor_communication() */
