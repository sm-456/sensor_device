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

#define __DECL_SPI_AM1815_H__


#include <msp430.h>
#include "driverlib.h"
#include <SPI_AM1815.h>
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


/*==============================================================================
  spi_initMaster_RTC()
 =============================================================================*/
void spi_initMaster_RTC(void)
{

    //Set NSS of the RTC as an output pin.
    GPIO_setAsOutputPin(SPI_RTC_GPIO_NSS, SPI_RTC_PIN_NSS);

    SPI_disable_RTC_NSS();  //disable the RTC


    // Configure SPI pins
    // Configure Pins for UCA1CLK
    GPIO_setAsPeripheralModuleFunctionInputPin(
        SPI_RTC_GPIO_CLK,
        SPI_RTC_PIN_CLK,
        GPIO_SECONDARY_MODULE_FUNCTION
        );

    // Configure Pins for UCA1RXD/UCA1SOMI
    GPIO_setAsPeripheralModuleFunctionInputPin(
        SPI_RTC_GPIO_SxMx,
        SPI_RTC_PIN_SIMO + SPI_RTC_PIN_SOMI,
        GPIO_SECONDARY_MODULE_FUNCTION
        );

    //Initialize Master
    EUSCI_B_SPI_initMasterParam param = {0};
    param.selectClockSource = EUSCI_B_SPI_CLOCKSOURCE_SMCLK;
    param.clockSourceFrequency = CS_getSMCLK();
    param.desiredSpiClock = 800000;
    param.msbFirst = EUSCI_B_SPI_MSB_FIRST;
//    param.clockPhase = EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
   param.clockPhase = EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT;
    param.clockPolarity = EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH;        //invert -> levelshifter
//    param.clockPolarity = EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW;
    param.spiMode = EUSCI_B_SPI_3PIN;
    EUSCI_B_SPI_initMaster(SPI_RTC_BASE, &param);

    //Enable SPI module
    EUSCI_B_SPI_enable(SPI_RTC_BASE);

    //Clear receive interrupt flag
    EUSCI_B_SPI_clearInterrupt(SPI_RTC_BASE, SPI_RTC_RECEIVE_INTERRUPT);



    // Enable SPI_RTC_BASE RX interrupt
    EUSCI_B_SPI_enableInterrupt(SPI_RTC_BASE, SPI_RTC_RECEIVE_INTERRUPT);


}/* spi_initMaster_RTC() */


/*============================================================================*/
/*!
    \brief   spi_configRTC_init()
             configure the RTC AM1815

    \param   None.

    \return  None.
*/
/*============================================================================*/
void spi_configRTC_init()
{
    /* Set the Control registers and interrupt mask */
    uint8_t ui8Control[3] = {0,0,0};

    spi_configRTC_oscillator();

    //AM1815_RA_CONTROL1          0x10
    ui8Control[0] = AM1815_BM_STOP_RUN | AM1815_BM_24HOUR | AM1815_BM_OUTB_DYNAMIC | AM1815_BM_RSP_LOW | AM1815_BM_ARST_NO_CLEAR | AM1815_BM_PWR2_PSW | AM1815_BM_WRTC_ENABLED;

    //AM1815_RA_CONTROL2          0x11
    ui8Control[1] = AM1815_BM_RS1E_ENABLED  | AM1815_BM_OUT2S_SLEEP | AM1815_BM_OUT1S_NIRQ;

    //AM1815_RA_INTERRUPT_MASK  0x12
    ui8Control[2] = AM1815_BM_CEB_TOGGLE | AM1815_BM_IM_3 | AM1815_BM_BLIE_DISABLED | AM1815_BM_TIE_ENABLED | AM1815_BM_AIE_DISABLED | AM1815_BM_EX2E_DISABLED | AM1815_BM_EX1E_DISABLED;

    spi_setRTC_Data(&(ui8Control[0]), AM1815_RA_CONTROL1, 3);

    spi_configRTC_key(AM1815_BM_KEY_OTHER_REG);


    //AM1815_RA_OUTPUT_CONTROL  0x30
    ui8Control[0] = 0;
    ui8Control[0] = (AM1815_BM_WDBM_WFI_DISABLED | AM1815_BM_EXBM_EXTI_DISABLED | AM1815_BM_WDDS_WDI_DISABLED | AM1815_BM_EXDS_EXTI_DISABLED | AM1815_BM_RSEN_NRST_DISABLED | AM1815_BM_O4EN_CLKOUT_DISABLED | AM1815_BM_O3EN_NTIRQ_DISABLED | AM1815_BM_O1EN_FOUT_DISABLED);

    spi_setRTC_Data(&(ui8Control[0]), AM1815_RA_OUTPUT_CONTROL, 1);

    spi_configRTC_autocalibration(true);

    spi_setRTC_OUT2S_sleep();   //moser 05.01.2017

}/* spi_configRTC_init() */

/*============================================================================*/
/*!
    \brief   spi_configRTC_time()
             config the time of the RTC AM1815
    \param   time.

    \return  None.
*/
/*============================================================================*/
void spi_configRTC_time(uint8_t* date)
{
    /*
     * config the used RTC AM1815
     * load the data into the Buffer
     */

    //Address = 0x01, 6 Bytes
    spi_setRTC_Data(&date[0], AM1815_RA_SECONDS, 6);

    spi_startRTC_communication();
}/* spi_configRTC_time() */


/*============================================================================*/
/*!
    \brief   spi_configRTC_countdowntimer()

    \param   time - that should be count into sec (maximum is 15300 seconds => 255 minutes => ~ 4h
            ,enable the timer.

    \return  None.
*/
/*============================================================================*/
void spi_configRTC_countdowntimer(uint16_t counterTime, bool counterEnable)
{
    /*
     * configure the used RTC AM1815
     * load the data into the Buffer
     */

    //Set the Control registers and interrupt mask
    uint8_t ui8Control[2] = {0, 0};
    uint8_t ui8TFS  = AM1815_BM_TFS_2;

    //clear register
    spi_setRTC_Data(&(ui8Control[0]), AM1815_RA_COUNTDOWN_CONTROL, 1);

    if(counterTime >= 0x00FF)
    {
        // set a lower clock frequenz (1/60 Hz) per
        counterTime =  counterTime / 60;
        ui8TFS = AM1815_BM_TFS_3;

        //the count register is only 8 Bit big
        if(counterTime > 0x00FF)
        {
            counterTime = 0x00FF;
        }
    }
    //else: clock frequency 1Hz

    //0x19 - Countdown Timer
    ui8Control[0] = (uint8_t) counterTime;
    ui8Control[1] = (uint8_t) counterTime;

    spi_setRTC_Data(&(ui8Control[0]), AM1815_RA_COUNTDOWN_VALUE, 2);


    //0x18 - Countdown Timer Control
    ui8Control[0] = (AM1815_BM_TM_TIMER_INT_DISABLED | AM1815_BM_TRPT_RELOAD_ENABLED | AM1815_BM_RPT_DISABLED | ui8TFS);

    if(counterEnable == true)
    {
        ui8Control[0] = ui8Control[0] | AM1815_BM_TE_COUNTTIMER_ENABLED;
    }

    spi_setRTC_Data(&(ui8Control[0]), AM1815_RA_COUNTDOWN_CONTROL, 1);


}/* spi_configRTC_countdowntimer() */

/*============================================================================*/
/*!
    \brief   spi_configRTC_oscillator()
             configure the Oscillator Control Register
             SPI-Communication is disabled, if the RTC is sleeping!

    \param   None.

    \return  None.
*/
/*============================================================================*/
void spi_configRTC_oscillator()
{
    uint8_t ui8Control[2] = {0, 0};


    //enable the access to the oscillator register
    spi_configRTC_key(AM1815_BM_KEY_OSCI);

    //AM1815_RA_OSC_CONTROL       0x1C
    ui8Control[0] = AM1815_BM_OSEL_RC | AM1815_BM_ACAL_512SEC | AM1815_BM_AOS_AUTOSWITCH | AM1815_BM_FOS_AUTOSWITCH | AM1815_BM_PWGT_IO_DISABLED | AM1815_BM_OFIE_INT_OSCI_FAIL_DISABLED | AM1815_BM_ACIE_INT_AUTOCAL_DISABLED;

    //AM1815_RA_OSC_STATUS        0x1D
    ui8Control[1] = AM1815_BM_LKO2_NO_LOCK | AM1815_BM_OF_CLEAR;

    spi_setRTC_Data(&(ui8Control[0]), AM1815_RA_OSC_CONTROL, 2);

}/* spi_configRTC_oscillator() */

/*============================================================================*/
/*!
    \brief   spi_configRTC_autocalibration()
             enable the autocalibration on the AF pin

    \param   enableCal.

    \return  None.
*/
/*============================================================================*/
void spi_configRTC_autocalibration(bool enableCal)
{
    //enable the access to the register
    spi_configRTC_key(AM1815_BM_KEY_OTHER_REG);

    uint8_t ui8Control = 0;

    ui8Control = ui8Control | AM1815_BM_AFCTRL_DISABLED;

    if(enableCal == true)
    {
        ui8Control = AM1815_BM_AFCTRL_ENABLED;
    }

    spi_setRTC_Data(&ui8Control, AM1815_RA_AFCTRL, 1);


}/* spi_configRTC_autocalibration() */


/*============================================================================*/
/*!
    \brief   spi_configRTC_batmode()
             enable or disable the I/O interfaces if VCC goes away

    \param   enableCal.

    \return  None.
*/
/*============================================================================*/
void spi_configRTC_batmode(bool disableIO)
{
    //enable the access to the register
    spi_configRTC_key(AM1815_BM_KEY_OTHER_REG);


    uint8_t ui8address = AM1815_RA_BATMODE;
    uint8_t ui8Control = 0 | AM1815_BM_BATMODE_IO_ALWAYS_ENABLED;

    if(disableIO == true)
    {
        ui8Control = AM1815_BM_BATMODE_IO_DISABLED;
    }
    spi_setRTC_Data(&ui8Control, ui8address, 1);


}/* spi_configRTC_batmode() */


/*============================================================================*/
/*!
    \brief   spi_configRTC_key()
             must be written with specific value to get access of registers and functions

    \param   key.

    \return  None.
*/
/*============================================================================*/
void spi_configRTC_key(uint8_t key)
{

    spi_setRTC_Data(&key, AM1815_RA_CONFIG_KEY, 1);

}/* spi_configRTC_key() */


/*============================================================================*/
/*!
    \brief   spi_getRTC_time()

    \param   read current date.

    \return  None.
*/
/*============================================================================*/
void spi_getRTC_time(uint8_t* date)
{

    spi_getRTC_Data((date), AM1815_RA_SECONDS, 6);

}/* spi_getRTC_time */

/*============================================================================*/
/*!
    \brief   spi_getRTC_countdowntime()
             read the Countdowntimer registers

    \param   read current date.

    \return  None.
*/
/*============================================================================*/
void spi_getRTC_countdowntime(uint8_t* time)
{
    //3 Bytes will be read
    spi_getRTC_Data((time), AM1815_RA_COUNTDOWN_CONTROL, 0x03);

}/* spi_getRTC_countdowntime */

/*============================================================================*/
/*!
    \brief   spi_getRTC_ID()

    \param   read 7-Byte ID of the RTC.

    \return  None.
*/
/*============================================================================*/
void spi_getRTC_ID(uint8_t* tmp)
{
    /*
     * read the current data
     */

    //Address of first ID Register = 0x28, 7 Bytes
    spi_getRTC_Data(&(tmp[0]), AM1815_RA_ID0, 7);


}/* spi_getRTC_ID */

/*============================================================================*/
/*!
    \brief   spi_getRTC_status()

    \param   None.

    \return  Status of the RTC.
*/
/*============================================================================*/
uint8_t spi_getRTC_status()
{
    /*
     * read the current Status
     */
    uint8_t tmp;

    //Address of Status Register = 0x0F, one Byte
    spi_getRTC_Data(&tmp, AM1815_RA_STATUS, 1);

    return tmp;
}/* spi_getRTC_status */


/*============================================================================*/
/*!
    \brief   spi_getRTC_GP0()

    \param   None.

    \return  GP0-Flag of the RTC.
*/
/*============================================================================*/
uint8_t spi_getRTC_GP0()
{
    /*
     * read the "GP0-Flag"
     */
    uint8_t tmp;

    //Address of Status Register = 0x01, one Byte
    spi_getRTC_Data(&tmp, AM1815_RA_SECONDS, 1);

    //GP0 is the seventh bit
    tmp = ((tmp >> 7) & 0x01);

    return tmp;
}/* spi_getRTC_GP0 */

/*============================================================================*/
/*!
    \brief   spi_getRTC_Data()
             get Data of the RTC

    \param   Pointer for data, address of RTC-Register, number of Bytes.

    \return  None.
*/
/*============================================================================*/
void spi_getRTC_Data(uint8_t* tmp, uint8_t address, uint8_t nBytes)
{
    /*
     * read the current data
     */

    //wait until all data are send
    while(Buffer128_allData(&Buffer_RTC) == false)
    {
        __bis_SR_register(LPM0_bits + GIE);      // CPU off
        __no_operation();
    }

    Buffer128_clean(&Buffer_RTC);

    Buffer_RTC.data[0] = address;       //set address
    //Number of rx data
    //one more Byte, because first byte will be only TX
    Buffer_RTC.dataLength = nBytes + 1;

    spi_startRTC_communicationLPM();


    while(Buffer128_allData(&Buffer_RTC) == false)
    {
    }

    //copy data to tmp
    memcpy(&tmp[0], &(Buffer_RTC.data[1]), nBytes);

}/* spi_getRTC_Data */


/*============================================================================*/
/*!
    \brief   spi_setRTC_GP0()

    \param   1: set the GP0-Flag
             0: clear the GP0-Flag

    \return  None.
*/
/*============================================================================*/
void spi_setRTC_GP0(uint8_t GP0_bit)
{
    /*
     * set or clear the "GP0-Flag"
     */
    uint8_t tmp;

    //Address of Status Register = 0x01, one Byte
    spi_getRTC_Data(&tmp, AM1815_RA_SECONDS, 1);

    //modify GP0
    if(GP0_bit == 0)
    {   //clear flag
        tmp = tmp & 0x7F;
    } else {
        //set flag
        tmp = tmp | 0x80;
    }

    // write it back
    spi_setRTC_Data(&tmp, AM1815_RA_SECONDS, 1);
}/* spi_setRTC_GP0 */

/*============================================================================*/
/*!
    \brief   spi_setRTC_countdowntime()
             set the time of the Countdowntimer

    \param   time until wakeup in sec.

    \return  None.
*/
/*============================================================================*/
void spi_setRTC_countdowntime(uint8_t timeUntilWakeup)
{

    spi_setRTC_Data(&timeUntilWakeup, AM1815_RA_COUNTDOWN_VALUE, 1);

}/* spi_setRTC_countdowntime() */

/*============================================================================*/
/*!
    \brief   spi_setRTC_sleep()
             set the RTC into sleep

    \param   time until sleep (3Bit, every Bit = 7,8ms).

    \return  None.
*/
/*============================================================================*/
void spi_setRTC_sleep(uint8_t timeUntilSleep)
{

//  uint8_t ui8Control = AM1815_BM_SLP_ENABLED | AM1815_BM_SLRES_HIGH | AM1815_BM_EX2P_RISE | AM1815_BM_EX1P_RISE | AM1815_BM_SLST_SLEEP | (timeUntilSleep & AM1815_BM_SLTO_TIME);
//  uint8_t ui8Control = AM1815_BM_SLP_ENABLED | AM1815_BM_SLRES_HIGH | AM1815_BM_EX2P_FALL | AM1815_BM_EX1P_FALL | (timeUntilSleep & AM1815_BM_SLTO_TIME);
    uint8_t ui8Control = AM1815_BM_SLP_ENABLED | AM1815_BM_SLRES_LOW | AM1815_BM_EX2P_FALL | AM1815_BM_EX1P_FALL | (timeUntilSleep & AM1815_BM_SLTO_TIME);
    spi_setRTC_Data(&ui8Control, AM1815_RA_SLEEP, 1);


}/* spi_setRTC_sleep() */


/*============================================================================*/
/*!
    \brief   spi_setRTC_OUT2S_sleep()
             configure the OUT2S-Flag to sleep

    \param   None.

    \return  None.
*/
/*============================================================================*/
void spi_setRTC_OUT2S_sleep()
{
    uint8_t ui8Control[2] = {0, 0};

    //AM1815_RA_CONTROL1          0x10
    ui8Control[0] = AM1815_BM_STOP_RUN | AM1815_BM_24HOUR | AM1815_BM_OUTB_STATIC | AM1815_BM_RSP_LOW | AM1815_BM_ARST_NO_CLEAR | AM1815_BM_PWR2_PSW | AM1815_BM_WRTC_ENABLED;

    //AM1815_RA_CONTROL2          0x11
    ui8Control[1] = AM1815_BM_RS1E_ENABLED  | AM1815_BM_OUT2S_SLEEP | AM1815_BM_OUT1S_NIRQ;

    spi_setRTC_Data(&(ui8Control[0]), AM1815_RA_CONTROL1, 2);

}/* spi_setRTC_OUT2S_sleep() */

/*============================================================================*/
/*!
    \brief   spi_setRTC_OUT2S_OUTB()
             configure the OUT2S-Flag to OUTB

    \param   None.

    \return  None.
*/
/*============================================================================*/
void spi_setRTC_OUT2S_OUTB()
{

    uint8_t ui8Control[2] = {0, 0};

    //AM1815_RA_OSC_STATUS        0x1D
    ui8Control[0] = AM1815_BM_LKO2_NO_LOCK | AM1815_BM_OF_CLEAR;

    spi_setRTC_Data(&(ui8Control[0]), AM1815_RA_OSC_STATUS, 1);


    ui8Control[0] = 0;
    //AM1815_RA_CONTROL1          0x10
    ui8Control[0] = AM1815_BM_STOP_RUN | AM1815_BM_24HOUR | AM1815_BM_OUTB_STATIC | AM1815_BM_RSP_LOW | AM1815_BM_ARST_NO_CLEAR | AM1815_BM_PWR2_PSW | AM1815_BM_WRTC_ENABLED;

    //AM1815_RA_CONTROL2          0x11
    ui8Control[1] = AM1815_BM_RS1E_ENABLED  | AM1815_BM_OUT2S_OUTB | AM1815_BM_OUT1S_NIRQ;

    spi_setRTC_Data(&(ui8Control[0]), AM1815_RA_CONTROL1, 2);


}/* spi_setRTC_OUT2S_OUTB() */

/*============================================================================*/
/*!
    \brief   spi_setRTC_Data()
             send Data to the RTC

    \param   Pointer of data, address of RTC-Register (write offset will be set), number of Bytes.

    \return  None.
*/
/*============================================================================*/
void spi_setRTC_Data(uint8_t* tmp, uint8_t address, uint8_t nBytes)
{
    /*
     * write the current data
     */

    //wait until all data are send
    while(Buffer128_allData(&Buffer_RTC) == false)
    {
        __bis_SR_register(LPM0_bits + GIE);      // CPU off
        __no_operation();
    }

    Buffer128_clean(&Buffer_RTC);


    Buffer_RTC.data[0] = address | 0x80;        //set address with offset
    //Number of rx data
    //one more Byte, because first byte will be only address
    Buffer_RTC.dataLength = nBytes + 1;

    //copy data from tmp
    memcpy(&(Buffer_RTC.data[1]), &(tmp[0]), nBytes);

    spi_startRTC_communication();

}/* spi_setRTC_Data */


/*============================================================================*/
/*!
    \brief   spi_clearRTC_status()
             clear the Status Register

    \param   None.

    \return  None.
*/
/*============================================================================*/
void spi_clearRTC_status()
{
    //Address of Status Register = 0x0F, one Byte
    uint8_t tmp = 0;

    spi_setRTC_Data(&tmp, AM1815_RA_STATUS, 1);

}/* spi_clearRTC_status */


/*============================================================================*/
/*!
    \brief   spi_startRTC_communication(
             put the data of the Buffer_RTC to SPI, handel the communication in the ISR

    \param   None.

    \return  None.
*/
/*============================================================================*/
void spi_startRTC_communication(void)
{
    /*
     * starts the communication to the RTC
     * send data to the RTC
     */

    if(Buffer128_NotEmpty(&Buffer_RTC) == false)
    {
        while(EUSCI_B_SPI_getInterruptStatus(SPI_RTC_BASE,
                                                SPI_RTC_TRANSMIT_INTERRUPT) == false)
        {
        }

        //set NSS Pin to 0 -> start communication
        SPI_enable_RTC_NSS();

        //send data
        EUSCI_B_SPI_transmitData(SPI_RTC_BASE, ~Buffer_RTC.data[0]);
        //send other Data is finished in the ISR

    }

}/* spi_spi_startRTC_communication() */


/*============================================================================*/
/*!
    \brief   spi_startRTC_communicationLPM()
             put the data of the Buffer_RTC to SPI, handels the communication
             Go to LPM0 and wake up, if the communication is complete!

    \param   None.

    \return  None.
*/
/*============================================================================*/
void spi_startRTC_communicationLPM(void)
{
    /*
     * starts the communication to the RTC
     * send data to the RTC
     */

    if(Buffer128_NotEmpty(&Buffer_RTC) == false)
    {
        while(EUSCI_B_SPI_getInterruptStatus(SPI_RTC_BASE, SPI_RTC_TRANSMIT_INTERRUPT) == false)
        {
        }

        __delay_cycles(50);

        //set NSS Pin to 0 -> start communication
        SPI_enable_RTC_NSS();

        //send data
        EUSCI_B_SPI_transmitData(SPI_RTC_BASE, ~Buffer_RTC.data[0]);    //invert the Date -> levelshifter
        //send other Data is handelt in the ISR

        __bis_SR_register(LPM0_bits + GIE);      // CPU off
        __no_operation();
    }

    /* Errata of the Chip: busy should not be used! */
    /*
    while(EUSCI_B_SPI_isBusy(SPI_RTC_BASE) != 0)
    {

    }
    */

}/* spi_startRTC_communication() */