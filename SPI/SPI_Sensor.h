#ifndef __SPI_SENSOR_H__
#define __SPI_SENSOR_H__
#ifndef __DECL_SPI_SENSOR_H__
#define __DECL_SPI_SENSOR_H__ extern
#endif

#include <stdint.h>
#include <stdbool.h>
#include "globals.h"


/*============================================================================*/
/*! \file   SPI_Sensor.h

    \author enter your name here

    \brief  SPI module: This is the header file containing the API for the
    		SPI functionality and the necessary constants and macros.
            
   \version $Revision$
*/
/*============================================================================*/

/*==============================================================================
                                 INCLUDE FILES
==============================================================================*/


/*==============================================================================
                                     MACROS
==============================================================================*/

/* SPI_MASTER */
#define SPI_Sensor_BASE              EUSCI_A1_BASE
#define SPI_Sensor_GPIO_CLK          GPIO_PORT_P2
#define SPI_Sensor_GPIO_SxMx         GPIO_PORT_P2
#define SPI_Sensor_PIN_CLK           GPIO_PIN4
#define SPI_Sensor_PIN_SOMI          GPIO_PIN6
#define SPI_Sensor_PIN_SIMO          GPIO_PIN5

#define SPI_Sensor_VECTOR					USCI_A1_VECTOR
#define SPI_Sensor_IV						UCA1IV
#define SPI_Sensor_TRANSMIT_INTERRUPT		EUSCI_A_SPI_TRANSMIT_INTERRUPT
#define SPI_Sensor_RECEIVE_INTERRUPT		EUSCI_A_SPI_RECEIVE_INTERRUPT
#define SPI_Sensor_ISR						USCI_A1_ISR


/* control the NSS Pin for the HTS221 */
#define SPI_Sensor1_GPIO_NSS          GPIO_PORT_PJ
#define SPI_Sensor1_PIN_NSS           GPIO_PIN4

#define SPI_enable_Sensor1_NSS()        GPIO_setOutputLowOnPin(SPI_Sensor1_GPIO_NSS, SPI_Sensor1_PIN_NSS)
#define SPI_disable_Sensor1_NSS()       GPIO_setOutputHighOnPin(SPI_Sensor1_GPIO_NSS, SPI_Sensor1_PIN_NSS)

#define SPI_Sensor1_GPIO_DRDY		GPIO_PORT_P2
#define SPI_Sensor1_PIN_DRDY		GPIO_PIN7

/* control the NSS Pin for the BME280 */
#define SPI_Sensor2_GPIO_NSS          GPIO_PORT_P2
#define SPI_Sensor2_PIN_NSS           GPIO_PIN3

#define SPI_enable_Sensor2_NSS()        GPIO_setOutputLowOnPin(SPI_Sensor2_GPIO_NSS, SPI_Sensor2_PIN_NSS)
#define SPI_disable_Sensor2_NSS()       GPIO_setOutputHighOnPin(SPI_Sensor2_GPIO_NSS, SPI_Sensor2_PIN_NSS)


/* control the NSS Pin for the TMP123 */
#define SPI_Sensor3_GPIO_NSS          GPIO_PORT_PJ
#define SPI_Sensor3_PIN_NSS           GPIO_PIN5

#define SPI_enable_Sensor3_NSS()        GPIO_setOutputLowOnPin(SPI_Sensor3_GPIO_NSS, SPI_Sensor3_PIN_NSS)
#define SPI_disable_Sensor3_NSS()       GPIO_setOutputHighOnPin(SPI_Sensor3_GPIO_NSS, SPI_Sensor3_PIN_NSS)


/* control the VCC-Enable Pin for the sensors */
#define SPI_Sensor_GPIO_VCC          GPIO_PORT_P4
#define SPI_Sensor_Pin_VCC          GPIO_PIN6


/* define the NSS Pins */
#define SPI_Sensor1				0x01
#define SPI_Sensor2				0x02
#define SPI_Sensor3				0x04




/* define the used Timer for the Sensor delay time */
#define Timer_SENSOR_initUpModeParam	Timer_A_initUpModeParam
#define Timer_Sensor_clearTimerInterrupt	Timer_A_clearTimerInterrupt
#define CS_getTIMER_SENSOR_CLK	CS_getSMCLK
#define TIMER_SENSOR_CLOCKSOURCE_DIVIDER_1	TIMER_A_CLOCKSOURCE_DIVIDER_1
#define timerInterruptEnable_SENSOR	timerInterruptEnable_TAIE
#define TIMER_SENSOR_INTERRUPT_ENABLE	TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE
#define TIMER_SENSOR_DO_CLEAR	TIMER_A_DO_CLEAR
#define TIMER_SENSOR_CLOCKSOURCE_SMCLK	TIMER_A_CLOCKSOURCE_SMCLK
#define Timer_SENSOR_initUpMode	Timer_A_initUpMode
#define Timer_SENSOR_startCounter	Timer_A_startCounter
#define TIMER_SENSOR_UP_MODE	 TIMER_A_UP_MODE
#define Timer_SENSOR_startCounter	Timer_A_startCounter
#define Timer_SENSOR_stop	Timer_A_stop

/* definitions for the ISR */
#define TIMER_SENSOR_BASE	TIMER_A1_BASE
#define TIMER_SENSOR_VECTOR	TIMER1_A1_VECTOR
#define TIMER_SENSOR_ISR	TIMER1_A1_ISR


/*============================================================================*/
/*!
    \brief   spi_initMaster_Sensor()
 	 	 	 init the SPI for the Sensors

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_initMaster_Sensor(void);

/*============================================================================*/
/*!
    \brief   spi_DeinitMaster_Sensor()
 	 	 	 disable the SPI for the Sensor

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_DeinitMaster_Sensor(void);


/*============================================================================*/
/*!
    \brief   spi_enable_Sensor_VCC()
 	 	 	 enable the Sensor VCC and set the SPI-Pins for the communication

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_enable_Sensor_VCC(void);

/*============================================================================*/
/*!
    \brief   spi_disable_Sensor_VCC()
 	 	 	 disable the Sensor VCC and set the SPI-Pins for the communication

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_disable_Sensor_VCC(void);

/*============================================================================*/
/*!
    \brief   spi_getSensor3_Data()
 	 	 	 get Data of the Sensor (TMP123)

    \param	 Pointer for data (uint16_t)

	\return  None.
*/
/*============================================================================*/
void spi_getSensor3_Data(uint16_t* tmp);

/*============================================================================*/
/*!
    \brief   SPI_enable_Sensor_NSS()
 	 	 	 enable the needed Sensor

    \param	 select the Sensor
    			0xFF: all Sensors are disabled

	\return  None.
*/
/*============================================================================*/
void SPI_enable_Sensor_NSS(uint8_t selectSensor);

/*============================================================================*/
/*!
    \brief   spi_startSensor_communication()
 	 	 	 put the data of the Buffer_Sensor to SPI, handel the communication in the ISR

    \param	 select the Sensor

	\return  None.
*/
/*============================================================================*/
void spi_startSensor_communication(uint8_t selectSensor);


/*============================================================================*/
/*!
    \brief   spi_startSensor_communicationLPM()
 	 	 	 put the data of the Buffer_Sensor to SPI, handels the communication
 	 	 	 Go to LPM0 and wake up, if the communication is complete!

    \param	 select the Sensor

	\return  None.
*/
/*============================================================================*/
void spi_startSensor_communicationLPM(uint8_t selectSensor);




#endif /* __SPI_SENSOR_H__ */
