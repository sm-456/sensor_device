#ifndef __SPI_AM1815_H__
#define __SPI_AM1815_H__
#ifndef __DECL_SPI_AM1815_H__
#define __DECL_SPI_AM1815_H__ extern
#endif

#include <stdint.h>
#include <stdbool.h>
#include "globals.h"

/*============================================================================*/
/*! \file   SPI_AM1815.h

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

/* Important! */
/* the used levelshifters are inverting the NSS, CLK and the SIMO Pins!!!! */

/* SPI_MASTER */
#define SPI_RTC_BASE              EUSCI_B0_BASE
#define SPI_RTC_GPIO_NSS          GPIO_PORT_P1
#define SPI_RTC_GPIO_CLK          GPIO_PORT_P2
#define SPI_RTC_GPIO_SxMx         GPIO_PORT_P1
#define SPI_RTC_PIN_NSS           GPIO_PIN3
#define SPI_RTC_PIN_CLK           GPIO_PIN2
#define SPI_RTC_PIN_SOMI          GPIO_PIN7
#define SPI_RTC_PIN_SIMO          GPIO_PIN6

#define SPI_RTC_VECTOR					USCI_B0_VECTOR
#define SPI_RTC_IV						UCB0IV
#define SPI_RTC_TRANSMIT_INTERRUPT		EUSCI_B_SPI_TRANSMIT_INTERRUPT
#define SPI_RTC_RECEIVE_INTERRUPT		EUSCI_B_SPI_RECEIVE_INTERRUPT
#define SPI_RTC_ISR						USCI_B0_ISR


/* control the NSS Pin */
#define SPI_enable_RTC_NSS()        GPIO_setOutputHighOnPin(SPI_RTC_GPIO_NSS, SPI_RTC_PIN_NSS)
#define SPI_disable_RTC_NSS()       GPIO_setOutputLowOnPin(SPI_RTC_GPIO_NSS, SPI_RTC_PIN_NSS)



/* AM1815 */
//Register Address
#define AM1815_RA_HUNDREDTHS        0X00
#define AM1815_RA_SECONDS           0x01
#define AM1815_RA_MINUTES           0x02
#define AM1815_RA_HOURS             0x03
#define AM1815_RA_DATE              0x04
#define AM1815_RA_MONTH             0x05
#define AM1815_RA_YEAR              0x06
#define AM1815_RA_DAY               0X07

#define AM1815_RA_HUNDREDTHS_ALARM  0X08
#define AM1815_RA_SECONDS_ALARM     0X09
#define AM1815_RA_MINUTES_ALARM     0X0A
#define AM1815_RA_HOURS_ALARM       0X0B
#define AM1815_RA_DATE_ALARM        0X0C
#define AM1815_RA_MONTH_ALARM       0X0D
#define AM1815_RA_WEEKDAYS_ALARM    0X0E

#define AM1815_RA_STATUS            0X0F
#define AM1815_RA_CONTROL1          0x10
#define AM1815_RA_CONTROL2          0x11            //interrupt control
#define AM1815_RA_INTERRUPT_MASK	0x12
#define AM1815_RA_SLEEP				0x17
#define AM1815_RA_COUNTDOWN_CONTROL	0x18
#define AM1815_RA_COUNTDOWN_VALUE	0x19
#define AM1815_RA_COUNTDOWN_INIT	0x1A
#define AM1815_RA_OSC_CONTROL       0x1C
#define AM1815_RA_OSC_STATUS        0x1D
//Config Key-written with specific values to access certain registers
//access Oscillator control (0x1C) -> write 0xA1
//Software Reset (doesn't update Config Key) -> write 0x3C
#define AM1815_RA_CONFIG_KEY      	0X1F
#define AM1815_RA_AFCTRL			0x26
#define AM1815_RA_BATMODE			0x27
#define AM1815_RA_ID0               0X28            //0x08
#define AM1815_RA_ID1 				0x29 //0x05
#define AM1815_RA_ANALOG_STATUS		0x2F
#define AM1815_RA_OUTPUT_CONTROL	0x30
#define AM1815_RA_EXTENSION_RAM		0x3F


//Bit Mask


//AM1815_RA_CONTROL1          0x10
#define AM1815_BM_STOP_STOP		0x80
#define AM1815_BM_STOP_RUN		0x00
#define AM1815_BM_12HOUR		0x40
#define AM1815_BM_24HOUR		0x00
#define AM1815_BM_OUTB_STATIC	0x20
#define AM1815_BM_OUTB_DYNAMIC	0x00
#define AM1815_BM_OUT_STATIC	0x10
#define AM1815_BM_OUT_DYNAMIC	0x00
#define AM1815_BM_RSP_HIGH		0x08
#define AM1815_BM_RSP_LOW		0x00
#define AM1815_BM_ARST_CLEAR	0x04
#define AM1815_BM_ARST_NO_CLEAR	0x00
#define AM1815_BM_PWR2_PSW		0x02
#define AM1815_BM_PWR2_OPENDRAIN	0x00
#define AM1815_BM_WRTC_ENABLED	0x01
#define AM1815_BM_WRTC_DISABLED	0x00

//AM1815_RA_CONTROL2          0x11
#define AM1815_BM_RS1E_ENABLED	0x20
#define AM1815_BM_RS1E_DISABLED	0x00
#define AM1815_BM_OUT2S_NIRQ	0x00
#define AM1815_BM_OUT2S_SQW		0x04
#define AM1815_BM_OUT2S_RESERVED		0x08
#define AM1815_BM_OUT2S_NAIRQ	0x0C
#define AM1815_BM_OUT2S_TIRQ	0x10
#define AM1815_BM_OUT2S_NTIRQ	0x14
#define AM1815_BM_OUT2S_SLEEP	0x18
#define AM1815_BM_OUT2S_OUTB	0x1C
#define AM1815_BM_OUT1S_NIRQ	0x00
#define AM1815_BM_OUT1S_SQW		0x01
#define AM1815_BM_OUT1S_SQW_NIRQ	0x02
#define AM1815_BM_OUT1S_NAIRQ	0x03

//AM1815_RA_INTERRUPT_MASK	0x12
#define AM1815_BM_CEB_NO_UPDATE	0x00
#define AM1815_BM_CEB_TOGGLE	0x80
#define AM1815_BM_IM_STATIC		0x00
#define AM1815_BM_IM_1			0x20
#define AM1815_BM_IM_2			0x40
#define AM1815_BM_IM_3			0x60
#define AM1815_BM_BLIE_DISABLED	0x00
#define AM1815_BM_BLIE_ENABLED	0x10
#define AM1815_BM_TIE_DISABLED	0x00
#define AM1815_BM_TIE_ENABLED	0x08
#define AM1815_BM_AIE_ENABLED	0x04
#define AM1815_BM_AIE_DISABLED	0x00
#define AM1815_BM_EX2E_ENABLED	0x02
#define AM1815_BM_EX2E_DISABLED	0x00
#define AM1815_BM_EX1E_DISABLED	0x00
#define AM1815_BM_EX1E_ENABLED	0x01


//AM1815_RA_SLEEP				0x17
#define AM1815_BM_SLP_ENABLED		0x80
#define AM1815_BM_SLP_DISABLED		0x00
#define AM1815_BM_SLRES_HIGH		0x40
#define AM1815_BM_SLRES_LOW			0x00
#define AM1815_BM_EX2P_RISE			0x20
#define AM1815_BM_EX2P_FALL			0x00
#define AM1815_BM_EX1P_RISE			0x10
#define AM1815_BM_EX1P_FALL			0x00
#define AM1815_BM_SLST_SLEEP		0x08
#define AM1815_BM_SLTO_TIME			0x07

//AM1815_RA_COUNTDOWN_CONTROL	0x18
#define AM1815_BM_TE_COUNTTIMER_ENABLED		0x80
#define AM1815_BM_TE_COUNTTIMER_DISABLED	0x00
#define AM1815_BM_TM_TIMER_INT_ENABLED		0x40
#define AM1815_BM_TM_TIMER_INT_DISABLED		0x00
#define AM1815_BM_TRPT_RELOAD_ENABLED		0x20
#define AM1815_BM_TRPT_RELOAD_DISABLED		0x00
#define AM1815_BM_RPT_SECOND				0x1C
#define AM1815_BM_RPT_MINUTE				0x18
#define AM1815_BM_RPT_HOUR					0x14
#define AM1815_BM_RPT_DAY					0x10
#define AM1815_BM_RPT_WEEK					0x0C
#define AM1815_BM_RPT_MONTH					0x08
#define AM1815_BM_RPT_YEAR					0x04
#define AM1815_BM_RPT_DISABLED				0x00
#define AM1815_BM_TFS_0						0x00
#define AM1815_BM_TFS_1						0x01
#define AM1815_BM_TFS_2						0x02
#define AM1815_BM_TFS_3						0x03

//AM1815_RA_OSC_CONTROL       0x1C
#define AM1815_BM_OSEL_RC			0x80
#define AM1815_BM_OSEL_XT			0x00
#define AM1815_BM_ACAL_NO			0x00
#define AM1815_BM_ACAL_RESERVED		0x20
#define AM1815_BM_ACAL_1024SEC		0x40
#define AM1815_BM_ACAL_512SEC		0x60
#define AM1815_BM_AOS_AUTOSWITCH	0x10
#define AM1815_BM_AOS_NO_AUTOSWITCH	0x00
#define AM1815_BM_FOS_AUTOSWITCH	0x08
#define AM1815_BM_FOS_NO_AUTOSWITCH	0x00
#define AM1815_BM_PWGT_IO_DISABLED	0x04
#define AM1815_BM_PWGT_IO_ALWAYS_ENABLED	0x00
#define AM1815_BM_OFIE_INT_OSCI_FAIL_ENABLED	0x02
#define AM1815_BM_OFIE_INT_OSCI_FAIL_DISABLED	0x00
#define AM1815_BM_ACIE_INT_AUTOCAL_ENABLED		0x01
#define AM1815_BM_ACIE_INT_AUTOCAL_DISABLED		0x00


//AM1815_RA_OSC_STATUS        0x1D
#define AM1815_BM_XTAL_0		0x00
#define AM1815_BM_XTAL_1		0x40
#define AM1815_BM_XTAL_2		0x60
#define AM1815_BM_XTAL_3		0x80
#define AM1815_BM_LKO2_LOCK_OUT2	0x20
#define AM1815_BM_LKO2_NO_LOCK	0x00
#define AM1815_BM_OMODE			0x10
#define AM1815_BM_OF_DETECT		0x02
#define AM1815_BM_OF_CLEAR		0x00
#define AM1815_BM_ACF_DETECT	0x01
#define AM1815_BM_ACF_CLEAR		0x00



//AM1815_RA_CONFIG_KEY      	0X1F
#define AM1815_BM_KEY_OSCI		0xA1
#define AM1815_BM_KEY_RESET		0x3C
#define AM1815_BM_KEY_OTHER_REG	0x9D

//AM1815_RA_AFCTRL			0x26
#define AM1815_BM_AFCTRL_ENABLED	0xA0
#define AM1815_BM_AFCTRL_DISABLED	0x00

//AM1815_RA_BATMODE			0x27
#define AM1815_BM_BATMODE_IO_ALWAYS_ENABLED	0x80
#define AM1815_BM_BATMODE_IO_DISABLED	0x00

//AM1815_RA_OUTPUT_CONTROL	0x30
#define AM1815_BM_WDBM_WFI_ENABLED		0x80
#define AM1815_BM_WDBM_WFI_DISABLED		0x00
#define AM1815_BM_EXBM_EXTI_ENABLED		0x40
#define AM1815_BM_EXBM_EXTI_DISABLED	0x00
#define AM1815_BM_WDDS_WDI_ENABLED		0x00
#define AM1815_BM_WDDS_WDI_DISABLED		0x20
#define AM1815_BM_EXDS_EXTI_ENABLED		0x00
#define AM1815_BM_EXDS_EXTI_DISABLED	0x10
#define AM1815_BM_RSEN_NRST_ENABLED		0x08
#define AM1815_BM_RSEN_NRST_DISABLED	0x00
#define AM1815_BM_O4EN_CLKOUT_ENABLED	0x04
#define AM1815_BM_O4EN_CLKOUT_DISABLED	0x00
#define AM1815_BM_O3EN_NTIRQ_ENABLED	0x02
#define AM1815_BM_O3EN_NTIRQ_DISABLED	0x00
#define AM1815_BM_O1EN_FOUT_ENABLED		0x01
#define AM1815_BM_O1EN_FOUT_DISABLED	0x00


/*==============================================================================
                                     ENUMS
==============================================================================*/


/*==============================================================================
                         STRUCTURES AND OTHER TYPEDEFS
==============================================================================*/


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
    \brief   spi_initMaster_RTC()
    		 Initialize SPI interface as MASTER:
    		 - Configure GPIOs properly to work as SPI
    		 - Configure baud rate
    		 - Enable Rx and Tx interrupt
    \param	 None.
    \param	 None.

*/
/*============================================================================*/
void spi_initMaster_RTC(void);


/*============================================================================*/
/*!
    \brief   spi_configRTC_init()
    		 configure the RTC AM1815

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_configRTC_init();


/*============================================================================*/
/*!
    \brief   spi_configRTC_time
    		 config the time of the RTC AM1815
    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_configRTC_time(uint8_t* pc_char);

/*============================================================================*/
/*!
    \brief   spi_configRTC_countdowntimer()

    \param	 time - that should be count into sec, enable the timer.

	\return  None.
*/
/*============================================================================*/
void spi_configRTC_countdowntimer(uint16_t counterTime, bool counterEnable);


/*============================================================================*/
/*!
    \brief   spi_configRTC_oscillator()
    		 configure the Oscillator Control Register

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_configRTC_oscillator();


/*============================================================================*/
/*!
    \brief   spi_configRTC_autocalibration()
    		 enable the autocalibration on the AF pin

    \param	 enableCal.

	\return  None.
*/
/*============================================================================*/
void spi_configRTC_autocalibration(bool enableCal);


/*============================================================================*/
/*!
    \brief   spi_configRTC_batmode()
    		 enable or disable the I/O interfaces if VCC goes away

    \param	 enableCal.

	\return  None.
*/
/*============================================================================*/
void spi_configRTC_batmode(bool enableIO);


/*============================================================================*/
/*!
    \brief   spi_configRTC_key()
			 must be written with specific value to get access of registers and functions

    \param	 key.

	\return  None.
*/
/*============================================================================*/
void spi_configRTC_key(uint8_t key);

/*============================================================================*/
/*!
    \brief   spi_getRTC_time()

    \param	 read the current date.

	\return  None.
*/
/*============================================================================*/
void spi_getRTC_time(uint8_t* date);


/*============================================================================*/
/*!
    \brief   spi_getRTC_countdowntime()
    		 read the Countdowntimer registers

    \param	 read current date.

	\return  None.
*/
/*============================================================================*/
void spi_getRTC_countdowntime(uint8_t* time);


/*============================================================================*/
/*!
    \brief   spi_getRTC_ID()

    \param	 read 7-Byte ID of the RTC.

	\return  None.
*/
/*============================================================================*/
void spi_getRTC_ID(uint8_t* tmp);

/*============================================================================*/
/*!
    \brief   spi_getRTC_status()

    \param	 None.

	\return  Status of the RTC.
*/
/*============================================================================*/
uint8_t spi_getRTC_status();


/*============================================================================*/
/*!
    \brief   spi_getRTC_GP0()

    \param	 None.

	\return  GP0-Flag of the RTC.
*/
/*============================================================================*/
uint8_t spi_getRTC_GP0();

/*============================================================================*/
/*!
    \brief   spi_getRTC_Data()
 	 	 	 get Data of the RTC

    \param	 Pointer for data, address of RTC-Register, number of Bytes.

	\return  None.
*/
/*============================================================================*/
void spi_getRTC_Data(uint8_t* tmp, uint8_t address, uint8_t nBytes);


/*============================================================================*/
/*!
    \brief   spi_setRTC_GP0()

    \param	 1: set the GP0-Flag
    		 0: clear the GP0-Flag

	\return  None.
*/
/*============================================================================*/
void spi_setRTC_GP0(uint8_t GP0_bit);


/*============================================================================*/
/*!
    \brief   spi_setRTC_countdowntime()
    		 set the time of the Countdowntimer

    \param	 time until wakeup in sec.

	\return  None.
*/
/*============================================================================*/
void spi_setRTC_countdowntime(uint8_t timeUntilWakeup);


/*============================================================================*/
/*!
    \brief   spi_setRTC_sleep()
    		 set the RTC into sleep

    \param	 time until sleep (3Bit, every Bit = 7,8ms).

	\return  None.
*/
/*============================================================================*/
void spi_setRTC_sleep(uint8_t timeUntilSleep);

/*============================================================================*/
/*!
    \brief   spi_setRTC_OUT2S_sleep()
    		 configure the OUT2S-Flag to sleep

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_setRTC_OUT2S_sleep();


/*============================================================================*/
/*!
    \brief   spi_setRTC_OUT2S_OUTB()
    		 configure the OUT2S-Flag to OUTB

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_setRTC_OUT2S_OUTB();


/*============================================================================*/
/*!
    \brief   spi_setRTC_Data()
 	 	 	 send Data to the RTC

    \param	 Pointer of data, address of RTC-Register (write offset will be set), number of Bytes.

	\return  None.
*/
/*============================================================================*/
void spi_setRTC_Data(uint8_t* tmp, uint8_t address, uint8_t nBytes);


/*============================================================================*/
/*!
    \brief   spi_clearRTC_status()
    		 clear the Status Register

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_clearRTC_status();


/*============================================================================*/
/*!
    \brief   spi_startRTC_communication()
 	 	 	 put the data of the Buffer_RTC to SPI, handels the communication

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_startRTC_communication();

/*============================================================================*/
/*!
    \brief   spi_startRTC_communicationLPM()
 	 	 	 put the data of the Buffer_RTC to SPI, handels the communication
 	 	 	 Go to LPM0 an wake up, if the communication is complete!

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void spi_startRTC_communicationLPM();

/* @} */

/*============================================================================*/

#endif /* __SPI_AM1815_H__ */
