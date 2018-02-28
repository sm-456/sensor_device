/*============================================================================*/
/*! \file   Timer_Delay.c

    \author your name

    \brief  Timer module: This is the source file containing the Timer functionality of the Project.

   \version $Revision$
*/
/*============================================================================*/

/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/


#include "TimerDelay.h"

#include <msp430.h>
#include "driverlib.h"
//#include "globals.h"


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
    \brief   initTimerDelay()
 	 	 	 init a timer

    \param	 None.

	\return  None.
*/
/*============================================================================*/
void initTimerDelay(void)
{

	Timer_A_clearTimerInterrupt(TIMER_A1_BASE);
	Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    //Start timer in up mode sourced by ACLK

    Timer_A_initUpModeParam param = {0};
    param.clockSource = TIMER_A_CLOCKSOURCE_ACLK;
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    param.timerClear = TIMER_A_DO_CLEAR;
    param.startTimer = false;

    param.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;

    Timer_A_initUpMode(TIMER_A1_BASE, &param);

    /*start in the timerDelay_LPM() function */
}/* initTimerDelay() */


/*============================================================================*/
/*!
    \brief   timerDelay_LPM()
 	 	 	 delay the program in milliseconds (in LPM)

    \param	 time in milliseconds.

	\return  None.
*/
/*============================================================================*/
void timerDelay_LPM(uint32_t msek)
{
	/*
	Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
	Timer_A_clearTimerInterrupt(TIMER_A1_BASE);
    Timer_A_enableInterrupt(TIMER_A1_BASE);
    Timer_A_enableCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
	Timer_A_setCompareValue(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, (msek * (CS_getACLK() / 1000)) - 1 );


	Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE);
	*/

	Timer_A_clearTimerInterrupt(TIMER_A1_BASE);
	Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

	if(msek >= 100000)
	{
		Timer_A_initUpModeParam param = {TIMER_A_CLOCKSOURCE_ACLK, TIMER_A_CLOCKSOURCE_DIVIDER_64, (uint16_t) ((msek / 64000) * CS_getACLK()) - 1,
										 TIMER_A_TAIE_INTERRUPT_DISABLE, TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, TIMER_A_DO_CLEAR, false};

		Timer_A_initUpMode(TIMER_A1_BASE, &param);
	} else if (msek >= 10000)
	{
		Timer_A_initUpModeParam param = {TIMER_A_CLOCKSOURCE_ACLK, TIMER_A_CLOCKSOURCE_DIVIDER_10, (uint16_t) ((msek * CS_getACLK()) / 10000) - 1,
										 TIMER_A_TAIE_INTERRUPT_DISABLE, TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, TIMER_A_DO_CLEAR, false};

		Timer_A_initUpMode(TIMER_A1_BASE, &param);
	} else
	{
	    Timer_A_initUpModeParam param = {TIMER_A_CLOCKSOURCE_ACLK, TIMER_A_CLOCKSOURCE_DIVIDER_1, (uint16_t) ((msek * CS_getACLK()) / 1000) - 1,
										 TIMER_A_TAIE_INTERRUPT_DISABLE, TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, TIMER_A_DO_CLEAR, false};

	    Timer_A_initUpMode(TIMER_A1_BASE, &param);
	}

    Timer_A_enableInterrupt(TIMER_A1_BASE);
    Timer_A_enableCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

	Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);

    //Enter LPM0, enable interrupts
    __bis_SR_register(LPM0_bits + GIE);
    //For debugger
    __no_operation();

}/* timerDelay_LPM() */
