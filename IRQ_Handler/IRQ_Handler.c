#include <SPI_SPIRIT1.h>
#include <SPI_AM1815.h>
#include <SPI_Sensor.h>
#include "globals.h"
#include "IRQ_Handler.h"



//******************************************************************************
//
//This is the SPI_RTC_VECTOR interrupt vector service routine
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=SPI_RTC_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(SPI_RTC_VECTOR)))
#endif
void SPI_RTC_ISR(void)
{
//	uint8_t ui8Data = 0;

    switch(__even_in_range(SPI_RTC_IV,4))
    {
    //Vector 2 - RXIFG
    case 2:
        //USCI_B TX buffer ready?

        while(!EUSCI_B_SPI_getInterruptStatus(SPI_RTC_BASE, SPI_RTC_TRANSMIT_INTERRUPT))
        {
        }

        Buffer_RTC.data[Buffer_RTC.counter] = EUSCI_B_SPI_receiveData(SPI_RTC_BASE);		//no invert needed -> levelshifter don't invert the "SOMI" data
        Buffer_RTC.counter = Buffer_RTC.counter + 1;

        if(Buffer128_allData(&Buffer_RTC) == true)
        {
        	//no more data to send
			//set NSS Pin to 1 -> end communication
        	SPI_disable_RTC_NSS();
			__bic_SR_register_on_exit(LPM0_bits);      // CPU on

        } else {
        	EUSCI_B_SPI_transmitData(SPI_RTC_BASE, ~Buffer_RTC.data[Buffer_RTC.counter]);		//invert the Date -> levelshifter
        }
        break;

	//Vector 4 - TXIFG
	case 4:
		//Transmit buffer empty
		//Not used -> will be handled in "case 2"
		break;

    default: break;
    }

}


//******************************************************************************
//
//This is the SPI_RF_VECTOR interrupt vector service routine
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=SPI_RF_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(SPI_RF_VECTOR)))
#endif
void SPI_RF_ISR(void)
{
    switch(__even_in_range(SPI_RF_IV,4))
    {
    //Vector 2 - RXIFG
    case 2:
        //USCI_A0 TX buffer ready?

        while(!EUSCI_A_SPI_getInterruptStatus(SPI_RF_BASE, SPI_RF_TRANSMIT_INTERRUPT))
        {
        }

        Buffer_RF.data[Buffer_RF.counter] = EUSCI_A_SPI_receiveData(SPI_RF_BASE);
        Buffer_RF.counter = Buffer_RF.counter + 1;

        if(Buffer128_allData(&Buffer_RF) == true)
        {
        	//no more data to send
			//set NSS Pin to 1 -> end communication
        	SPI_disable_RF_NSS();
        	//wake the Core up
        	__bic_SR_register_on_exit(LPM0_bits);      // CPU on

        } else {
        	EUSCI_A_SPI_transmitData(SPI_RF_BASE, Buffer_RF.data[Buffer_RF.counter]);
        }
        break;

	//Vector 4 - TXIFG
	case 4:
		//Transmit buffer empty
		//Not used -> will be handled in "case 2"
		break;

    default: break;
    }
}


//******************************************************************************
//
//This is the SPI_Sensor_VECTOR interrupt vector service routine
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=SPI_Sensor_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(SPI_Sensor_VECTOR)))
#endif
void SPI_Sensor_ISR(void)
{
    switch(__even_in_range(SPI_Sensor_IV,4))
    {
    //Vector 2 - RXIFG
    case 2:
        //USCI_A0 TX buffer ready?

        while(!EUSCI_A_SPI_getInterruptStatus(SPI_Sensor_BASE, SPI_Sensor_TRANSMIT_INTERRUPT))
        {
        }

        Buffer_Sensor.data[Buffer_Sensor.counter] = EUSCI_A_SPI_receiveData(SPI_Sensor_BASE);
        Buffer_Sensor.counter = Buffer_Sensor.counter + 1;

        if(Buffer128_allData(&Buffer_Sensor) == true)
        {
        	//no more data to send
			//set NSS Pin to 1 -> end communication
        	SPI_enable_Sensor_NSS(0xFF);
        	//wake the Core up
        	__bic_SR_register_on_exit(LPM0_bits);      // CPU on

        } else {
        	EUSCI_A_SPI_transmitData(SPI_Sensor_BASE, Buffer_Sensor.data[Buffer_Sensor.counter]);
        }
        break;

	//Vector 4 - TXIFG
	case 4:
		//Transmit buffer empty
		//Not used -> will be handled in "case 2"
		break;

    default: break;
    }
}

//******************************************************************************
//
//This is the PORT3_VECTOR interrupt vector service routine
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT3_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(PORT3_VECTOR)))
#endif
void Port_3(void)
{
	if(GPIO_getInterruptStatus(SPI_RF_GPIO_0_INT, SPI_RF_PIN_0_INT) == SPI_RF_PIN_0_INT)
	{
		CircularBuffer_In(0xAA, &FIFO_IRQ_RF);


		GPIO_clearInterrupt(SPI_RF_GPIO_0_INT, SPI_RF_PIN_0_INT);
	}
	else if(GPIO_getInterruptStatus(SPI_RF_GPIO_1_INT, SPI_RF_PIN_1_INT) == SPI_RF_PIN_1_INT)
	{

		GPIO_clearInterrupt(SPI_RF_GPIO_1_INT, SPI_RF_PIN_1_INT);
	}
	else if(GPIO_getInterruptStatus(SPI_RF_GPIO_2_INT, SPI_RF_PIN_2_INT) == SPI_RF_PIN_2_INT)
	{

		GPIO_clearInterrupt(SPI_RF_GPIO_2_INT, SPI_RF_PIN_2_INT);
	}
	else
	{
		GPIO_clearInterrupt(GPIO_PORT_P3, GPIO_PIN_ALL8);
	}
}


//******************************************************************************
//
//This is the PORT4_VECTOR interrupt vector service routine
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT4_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(PORT4_VECTOR)))
#endif
void Port_4(void)
{
	if(GPIO_getInterruptStatus(SPI_RF_GPIO_3_INT, SPI_RF_PIN_3_INT) == SPI_RF_PIN_3_INT)
	{
    	//wake the Core up
    	__bic_SR_register_on_exit(LPM0_bits);      // CPU on

		GPIO_clearInterrupt(SPI_RF_GPIO_3_INT, SPI_RF_PIN_3_INT);
	} else {

		GPIO_clearInterrupt(GPIO_PORT_P4, GPIO_PIN_ALL8);
	}
}

//******************************************************************************
//
//This is the PORT1_VECTOR interrupt vector service routine
//
//******************************************************************************
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{

    ++c_freq;
    P1IFG &= ~BIT2;     // reset interrupt flag
}



//******************************************************************************
//
//This is the TIMER1_A3 interrupt vector service routine.
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(TIMER1_A0_VECTOR)))
#endif
void TIMER1_A0_ISR(void)
{
	Timer_A_disableInterrupt(TIMER_A1_BASE);
	Timer_A_disableCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
	Timer_A_clearTimerInterrupt(TIMER_A1_BASE);
	Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
	Timer_A_stop(TIMER_A1_BASE);
	//wake the Core up
	__bic_SR_register_on_exit(LPM0_bits);      // CPU on
}


//******************************************************************************
//
//This is the TIMER1_A3 interrupt vector service routine.
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A1_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(TIMER1_A1_VECTOR)))
#endif
void TIMER1_A1_ISR(void)
{
	Timer_A_disableInterrupt(TIMER_A1_BASE);
	Timer_A_disableCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
	Timer_A_clearTimerInterrupt(TIMER_A1_BASE);
	Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
	Timer_A_stop(TIMER_A1_BASE);
	//wake the Core up
	__bic_SR_register_on_exit(LPM0_bits);      // CPU on
}
