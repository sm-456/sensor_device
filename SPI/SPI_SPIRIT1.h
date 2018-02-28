#ifndef __SPI_SPIRIT1_H__
#define __SPI_SPIRIT1_H__
#ifndef __DECL_SPI_SPIRIT1_H__
#define __DECL_SPI_SPIRIT1_H__ extern
#endif

#include <stdint.h>
#include <stdio.h>
#include "globals.h"

#include "MCU_Interface.h"



/*============================================================================*/
/*! \file   SPI_SPIRIT1.h

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

/* RF Modul */
#define SPI_RF_BASE              EUSCI_A0_BASE
#define SPI_RF_GPIO_NSS          GPIO_PORT_P1
#define SPI_RF_GPIO_CLK          GPIO_PORT_P1
#define SPI_RF_GPIO_SxMx         GPIO_PORT_P2
#define SPI_RF_PIN_NSS           GPIO_PIN4
#define SPI_RF_PIN_CLK           GPIO_PIN5
#define SPI_RF_PIN_SOMI          GPIO_PIN1
#define SPI_RF_PIN_SIMO          GPIO_PIN0

#define SPI_RF_VECTOR					USCI_A0_VECTOR
#define SPI_RF_IV						UCA0IV
#define SPI_RF_TRANSMIT_INTERRUPT		EUSCI_A_SPI_TRANSMIT_INTERRUPT
#define SPI_RF_RECEIVE_INTERRUPT		EUSCI_A_SPI_RECEIVE_INTERRUPT
#define SPI_RF_ISR						USCI_A0_ISR

/* Peripheral Pins of the RF Modul */
#define SPI_RF_GPIO_0_INT			GPIO_PORT_P3
#define SPI_RF_PIN_0_INT			GPIO_PIN1

#define SPI_RF_GPIO_1_INT			GPIO_PORT_P3
#define SPI_RF_PIN_1_INT			GPIO_PIN2

#define SPI_RF_GPIO_2_INT			GPIO_PORT_P3
#define SPI_RF_PIN_2_INT			GPIO_PIN3

#define SPI_RF_GPIO_3_INT			GPIO_PORT_P4
#define SPI_RF_PIN_3_INT			GPIO_PIN7

/* control the NSS Pin */
#define SPI_enable_RF_NSS()        GPIO_setOutputLowOnPin(SPI_RF_GPIO_NSS, SPI_RF_PIN_NSS)
#define SPI_disable_RF_NSS()       GPIO_setOutputHighOnPin(SPI_RF_GPIO_NSS, SPI_RF_PIN_NSS)


/* control the SDN Pin */
#define SPI_RF_SDN					GPIO_PORT_P4, GPIO_PIN0
#define SPI_enable_RF()        GPIO_setOutputLowOnPin(SPI_RF_SDN)
#define SPI_disable_RF()       GPIO_setOutputHighOnPin(SPI_RF_SDN)

/* Flags for the RF */
#define HEADER_WRITE_MASK     0x00 /*!< Write mask for header byte*/
#define HEADER_READ_MASK      0x01 /*!< Read mask for header byte*/
#define HEADER_ADDRESS_MASK   0x00 /*!< Address mask for header byte*/
#define HEADER_COMMAND_MASK   0x80 /*!< Command mask for header byte*/

#define LINEAR_FIFO_ADDRESS 0xFF  /*!< Linear FIFO address*/

/* other modules */


/*==============================================================================
                                     ENUMS
==============================================================================*/


/*==============================================================================
                         STRUCTURES AND OTHER TYPEDEFS
==============================================================================*/

typedef SpiritStatus StatusBytesRF;

/*==============================================================================
                          GLOBAL VARIABLE DECLARATIONS
==============================================================================*/


/*==============================================================================
                         FUNCTION PROTOTYPES OF THE API
==============================================================================*/

/** \defgroup spi_api SPI controller API */
/* @{ */

/*============================================================================*/
/*!
    \brief   spi_initMaster_RF()
    		 Initialize SPI interface as MASTER:
    		 - Configure GPIOs properly to work as SPI
    		 - Configure baud rate
    		 - Enable Rx and Tx interrupt
    \param	 None.
    \param	 None.

*/
/*============================================================================*/
void spi_initMaster_RF(void);

/*============================================================================*/
/*!
    \brief   spi_DeinitMaster_RF()
 	 	 	 disable the SPI for the RF

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_DeinitMaster_RF(void);


/*============================================================================*/
/*!
    \brief   spi_Deinit_RF()
 	 	 	 Deinit the RF and the interrupt Pins for communication/flags

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_Deinit_RF(void);

/*============================================================================*/
/*!
    \brief   spi_checkFIFO_IRQ_RF()
 	 	 	 check the FIFO of IRQ_RF

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_checkFIFO_IRQ_RF(void);

/*============================================================================*/
/*!
    \brief   spi_init_RF()
 	 	 	 init the RF and the interrupt Pins for communication/flags

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_init_RF(void);


/*============================================================================*/
/*!
    \brief   spi_getRF_FIFO()
 	 	 	 get Data of the FIFO

    \param	 Pointer for data, number of Bytes.

	\return  SPIRIT status
*/
/*============================================================================*/
StatusBytesRF spi_getRF_FIFO(uint8_t* tmp, uint8_t nBytes);


/*============================================================================*/
/*!
    \brief   spi_setRF_FIFO()
 	 	 	 set Data to the FIFO

    \param	 Pointer for data, number of Bytes.

	\return  SPIRIT status
*/
/*============================================================================*/
StatusBytesRF spi_setRF_FIFO(uint8_t* tmp, uint8_t nBytes);


/*============================================================================*/
/*!
    \brief   spi_getRF_Data()
 	 	 	 get Data of the RF

    \param	 Pointer for data, address of RF-Register, number of Bytes.

	\return  SPIRIT status
*/
/*============================================================================*/
StatusBytesRF spi_getRF_Data(uint8_t* tmp, uint8_t address, uint8_t nBytes);


/*============================================================================*/
/*!
    \brief   spi_setRF_Data()
 	 	 	 send Data to the RF

    \param	 Pointer of data, address of RF-Register (write offset will be set), number of Bytes.

	\return  SPIRIT status
*/
/*============================================================================*/
StatusBytesRF spi_setRF_Data(uint8_t* tmp, uint8_t address, uint8_t nBytes);

/*============================================================================*/
/*!
    \brief   spi_startRF_communication(
 	 	 	 put the data of the Buffer_RF to SPI, handel the communication in the ISR

    \param	 None.

	\return  None.
*/

/*============================================================================*/
/*!
    \brief   spi_setRF_Command()
 	 	 	 Send a command

    \param	 cCommandCode: command code to be sent

	\return  SPIRIT status
*/
/*============================================================================*/
StatusBytesRF spi_setRF_Command(uint8_t cCommandCode);


/*============================================================================*/
void spi_startRF_communication(void);

/*============================================================================*/
/*!
    \brief   spi_startRF_communicationLPM()
 	 	 	 put the data of the Buffer_RF to SPI, handels the communication
 	 	 	 Go to LPM0 and wake up, if the communication is complete!

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_startRF_communicationLPM(void);


/* This is the function that initializes the SPIRIT with the configuration
that the user has exported using the GUI */
void SpiritBaseConfiguration(void);


/* This is a VCO calibration routine used to recalibrate the VCO of SPIRIT1 in a safe way.
 IMPORTANT: It must be called from READY state. */
void SpiritVcoCalibration(void);

#endif /* __SPI_SPIRIT1_H__ */
