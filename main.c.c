//***************************************************************************************
//  MSP430 und RTC AM1815
//
//  Description; MSP430FR5969 communicates with the RTC AM1815 over SPI
//  UCA1 is used. CS = P4.2
//  ACLK = n/a, MCLK = SMCLK = BRCLK =  DCO = 1MHz
//
//
//  P. Moser
//  Hochschule Offenburg
//  September 2016
//  Built with Code Composer Studio v6
//***************************************************************************************

#include <msp430.h>
#include <time.h>
#include "SPI_AM1815.h"
#include "SPI_SPIRIT1.h"
#include "SPI_Sensor.h"
#include "driverlib.h"
#include "globals.h"
#include "IRQ_Handler.h"
#include "TimerDelay.h"
#include <bme280.h>

//#include "bme280_communication.h"




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
// Do not use this section for code or data placement.
// It will get overwritten!
#define SEND_DELAY      40

#define FRAM_ID_SENSORBOARD                 0xC000      //the ID of the Sensorboard is stored in this register (10 Bit)
#define FRAM_DATA_COUNTER                   0xC004      //Number of data

/*  how the data are saved? (example view: FRAM_DATA_SENS2_TEMP)
 *      Address:                        What Data can be find?
 *      FRAM_DATA_SENS2_TEMP            ID of the Sensorboard (Bit 15 to 5) and the ID of the used Sensor (Bit 4 to 0)
 *      FRAM_DATA_SENS2_TEMP + 1        Number (ID) of DataFrame (Bit 15 to 6) and the number FRAM_stored_data of 16Bit values (Bit 5 to 1), Bit 0: "1" -> an other DataFrame will be send
 *      FRAM_DATA_SENS2_TEMP + 2        First Temperature (16Bit)
 *      FRAM_DATA_SENS2_TEMP + ....
 *      FRAM_DATA_SENS2_TEMP + FRAM_stored_data     Last/actual Temperature (16Bit)
 *
 */


//#define FRAM_DATA_SENS1           0xC100  //Data of the Sensor1 (HTS221) are saved here
#define FRAM_REGISTRATION           0xC100
#define FRAM_DATA_COUNTER_M         0xC104  // Counter for soil moisture measurement

//Data of the Sensor2 (BME280) are saved here
#define FRAM_DATA_SENS2_TEMP        0xC200  //Temperature are saved here -> every value has 16Bit
#define FRAM_ID_SENS2_TEMP          0x01    //ID of the Temperature of Sensor2
#define FRAM_DATA_SENS2_PRESS       0xC300  //Pressure -> 16Bit
#define FRAM_ID_SENS2_PRESS         0x02    //ID of the pressure Sensor2
#define FRAM_DATA_SENS2_HUMIDITY    0xC400  //Humidity -> 16Bit
#define FRAM_ID_SENS2_HUMIDITY      0x03    //ID of the HUMIDITY of Sensor2
#define FRAM_DATA_MOISTURE          0xC500  //
#define FRAM_ID_MOISTURE            0x04
#define MOISTURE_COUNTER_MAX         40
#define REGISTRATION_MESSAGE        0x05

#define FRAM_DATA_SENS3             0xCF00  //Data of the Sensor3 (TMP123) are saved here


#define FRAM_MAX_DATA   10  //maximal 60 data samples are possible -> every sample is 2 bytes per sensor -> every hour a the values will be send
                            // don't forget the 4 Byte header

#define RF_onemoreDataFrame 1


uint16_t ID_of_Sensorboard  = 2;    //ID has to be set in code
                                //possible: use the DEVICE ID of the MSP430
                                // or it can be set by PC-Software

uint16_t *pointer_FRAM_ui16 = 0;    //pointer for use of the FRAM-storage
uint8_t *pointer_FRAM_ui8   = 0;    //pointer for use of the FRAM-storage

uint16_t FRAM_stored_data = 0;          //count the number of data into the FRAM
uint16_t FRAM_moisture_counter = 0;      //counter for number of measured data sets, used for soil moisture measurement every x hours

//Buffer for sendData
uint8_t vectcTxBuff[FRAM_MAX_DATA + 4]={};

uint8_t SW101_event = 0;

//Value of the DIPs and Buttons -> a "1" defines pressed
bool DIP1 = 0;
bool DIP2 = 0;
bool DIP3 = 0;
bool DIP4 = 0;
bool Button1 = 0;
bool Button2 = 0;

//defines, if the RF has enough power
bool ePG = 0;   //with 1: enough Power for sending is in the "power source"
bool PG = 0;    //if 1: the needed voltage is there (or more voltage)

//Values of Sensor2 (BME280) are saved in this variables
//only 16 bit of the sensor value is used
    uint16_t pressure_u16 = 0;
    int16_t temperature_s16 = 0;
    uint16_t temperature_u16 = 0;
    uint16_t humidity_u16 = 0;
    uint16_t frequency_u16 = 0;

uint16_t Sensor_Temp_TMP123 = 0;    //the current temperature of the TMP123 Sensor is saved in this variable

uint8_t tmp_ui8 = 0;
uint16_t tmp_ui16 = 0;
uint8_t rtc_date[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t date_test[6] = {0};
uint8_t FRAM_registration_done = 0;
uint16_t offset_time = 0;

time_t t;
struct tm* ts;
uint32_t t_int;

uint8_t registration[4] = {0};

// BME init variables
int8_t rslt = BME280_OK;
struct bme280_dev dev;
struct bme280_data data;
uint8_t settings_sel;
uint8_t rx_buffer[10] = {0};

/*
typedef struct
{
    short daylight;
    long  timezone; // seconds WEST of UTC.  Strange but traditional
    char  tzname[4];
    char  dstname[4];
} TZ;

struct TZ = {0, -3600, 'CET',
*/

/*==============================================================================
                                LOCAL CONSTANTS
 =============================================================================*/

/*==============================================================================
                           LOCAL FUNCTION PROTOTYPES
 =============================================================================*/





/*==============================================================================
  selectVoltage()
 =============================================================================*/
void selectVoltage(uint8_t voltage)
{
    if(voltage >= 0x10) //Voltage not defined/possible
    {
        voltage = VOLTAGE_33;   //maximal possible voltage
    }

    //check the VSEL1 bit
    if((voltage & 0x01) == 0x01)
    {   // set VsSEL1
        pin_setHigh_VSEL1();
    } else
    {   //clear VSEL1
        pin_setLow_VSEL1();
    }

    //check the VSEL2 bit
    if((voltage & 0x02) == 0x02)
    {   // set VsSEL2
        pin_setHigh_VSEL2();
    } else
    {   //clear VSEL2
        pin_setLow_VSEL2();
    }

    //check the VSEL3 bit
    if((voltage & 0x04) == 0x04)
    {   // set VsSEL3
        pin_setHigh_VSEL3();
    } else
    {   //clear VSEL3
        pin_setLow_VSEL3();
    }

    //check the VSEL4 bit
    if((voltage & 0x08) == 0x08)
    {   // set VsSEL4
        pin_setHigh_VSEL4();
    } else
    {   //clear VSEL4
        pin_setLow_VSEL4();
    }
}


/*==============================================================================
  initClocks()
 =============================================================================*/
void initClocks(void)
{
    //Set DCO frequency to 1MHz
    CS_setDCOFreq(CS_DCORSEL_0,CS_DCOFSEL_0);
    //Set ACLK = VLO with frequency divider of 1
    CS_initClockSignal(CS_ACLK,CS_VLOCLK_SELECT,CS_CLOCK_DIVIDER_1);
    //Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK,CS_DCOCLK_SELECT,CS_CLOCK_DIVIDER_1);
    //Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK,CS_DCOCLK_SELECT,CS_CLOCK_DIVIDER_1);
}

/*==============================================================================
  initGpio()
 =============================================================================*/
void initGpio(void)
{
    //declare all PINs as input
    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN_ALL8);
    GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN_ALL8);
    GPIO_setAsInputPin(GPIO_PORT_P3, GPIO_PIN_ALL8);
    GPIO_setAsInputPin(GPIO_PORT_P4, GPIO_PIN_ALL8);
    GPIO_setAsInputPin(GPIO_PORT_PJ, GPIO_PIN_ALL8);

    //define RTC-Pins for "not used"
        // -> all Pins has a pull-up or pull-down resistor

    //define RF-Pins for "not used"
    GPIO_setAsInputPinWithPullDownResistor(SPI_RF_GPIO_0_INT, SPI_RF_PIN_0_INT);
    GPIO_setAsInputPinWithPullDownResistor(SPI_RF_GPIO_1_INT, SPI_RF_PIN_1_INT);
    GPIO_setAsInputPinWithPullDownResistor(SPI_RF_GPIO_2_INT, SPI_RF_PIN_2_INT);
    GPIO_setAsInputPinWithPullDownResistor(SPI_RF_GPIO_3_INT, SPI_RF_PIN_3_INT);
    GPIO_setAsInputPinWithPullDownResistor(SPI_RF_GPIO_NSS, SPI_RF_PIN_NSS);
    GPIO_setAsInputPinWithPullDownResistor(SPI_RF_GPIO_CLK, SPI_RF_PIN_CLK);
    GPIO_setAsInputPinWithPullDownResistor(SPI_RF_GPIO_SxMx, SPI_RF_PIN_SIMO + SPI_RF_PIN_SOMI);

    GPIO_setAsOutputPin(SPI_RF_GPIO_NSS, SPI_RF_PIN_NSS);
    SPI_disable_RF_NSS();

        //SDN-Pin has a PullUp -> but will set as an output too
    GPIO_setAsOutputPin(SPI_RF_SDN);
    SPI_disable_RF();   //disable the RF

    //define Sensor for "not used"
        // define and disable the enable "VCC for Sensor" Pin
        GPIO_setAsOutputPin(SPI_Sensor_GPIO_VCC, SPI_Sensor_Pin_VCC);
        spi_disable_Sensor_VCC();
        // define the Data ready Pin
        GPIO_setAsInputPinWithPullDownResistor(SPI_Sensor1_GPIO_DRDY, SPI_Sensor1_PIN_DRDY);


    //define Oscilator Pins
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_PJ, GPIO_PIN6 + GPIO_PIN7);


    //define "Button" Pins
    GPIO_setAsOutputPin(pin_buttons);
    pin_disable_buttons();

    //define "StepDown" Pins
    GPIO_setAsOutputPin(pin_VSEL1);
    GPIO_setAsOutputPin(pin_VSEL2);
    GPIO_setAsOutputPin(pin_VSEL3);
    GPIO_setAsOutputPin(pin_VSEL4);

    pin_setLow_VSEL1();
    pin_setLow_VSEL2();
    pin_setHigh_VSEL3();
    pin_setLow_VSEL4();

}

/*==============================================================================
  initStartUp()
 =============================================================================*/
void initStartUp(void)
{
    /* Initialize GPIO and clocks */
    initGpio();
    initClocks();
}


/*==============================================================================
  Button_getValue()
 =============================================================================*/
void Button_getValue(void)
{
    //enable the VCC for the Buttons
    pin_enable_buttons();

    //wait a few cycles -> capacity of the Buttons
    __delay_cycles(10);

    // value can be read
    DIP1 = (bool) pin_Value_DIP1();
    DIP2 = (bool) pin_Value_DIP2();
    DIP3 = (bool) pin_Value_DIP3();
    DIP4 = (bool) pin_Value_DIP4();
    Button1 = (bool) ! pin_Value_Button1();
    Button2 = (bool) ! pin_Value_Button2();

    //disable the Dips and Buttons -> save energy
    pin_disable_buttons();
}

/*==============================================================================
  checkPower()
 =============================================================================*/
void checkPower(void)
{
    //power Signal of the external "power source"
    ePG = !((bool) pin_Value_ePG());


    //PowerGood Signal from the used StepDown Converter
    PG = (bool) pin_Value_PG();
}

/*==============================================================================
  getFrequency()
 =============================================================================*/

uint16_t getFrequency(void)
{
    uint16_t temp_freq = 0;
    //spi_enable_Sensor_VCC();    // enable power source
    timerDelay_LPM(50);   // wait 1 second for sensor frequency equalization

    P1DIR &= ~BIT2;     // set P1.2 as input
    P1REN |= BIT2;      // pullup resistors 1.2
    P1OUT |= BIT2;      // pullup mode
    P1IE |= BIT2;       // P1.2 interrupt enabled
    P1IES |= BIT2;      // P1.2 Hi/lo edge
    P1IFG &= ~BIT2;     // P1.2 IFG cleared

    timerDelay_LPM(100);

    P1IE &= ~BIT2;      // P1.2 interrupt disabled
    temp_freq = c_freq*10;
    c_freq = 0;
    return temp_freq;

}

/*==============================================================================
  RTC_createDate()
 =============================================================================*/

uint8_t* RTC_createDate(uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t day, uint8_t month, uint8_t year)
{
   /* 6 Byte time and date array
    * 0x01 Seconds
    * 0x02 Minutes
    * 0x03 Hours
    * 0x04 Date
    * 0x05 Months
    * 0x06 Years
    see data sheet for memory map (pg 54)
    */
    uint8_t* ptr;
    uint8_t rtc_ten;
    uint8_t ones;

    // seconds
    rtc_ten = seconds;
    ones = seconds % 10;
    rtc_ten = rtc_ten - ones;
    rtc_date[0] = ((rtc_ten/10)<<4) | ones | (0x80);  // keep GP0 flag (msb)
    // minutes
    rtc_ten = minutes;
    ones = minutes % 10;
    rtc_ten = rtc_ten - ones;
    rtc_date[1] = ((rtc_ten/10)<<4) | ones;
    // hours
    rtc_ten = hours;
    ones = hours % 10;
    rtc_ten = rtc_ten - ones;
    rtc_date[2] = ((rtc_ten/10)<<4) | ones;
    // day
    rtc_ten = day;
    ones = day % 10;
    rtc_ten = rtc_ten - ones;
    rtc_date[3] = ((rtc_ten/10)<<4) | ones;
    // month
    rtc_ten = month;
    ones = month % 10;
    rtc_ten = rtc_ten - ones;
    rtc_date[4] = ((rtc_ten/10)<<4) | ones;
    // year (2000 + x ?)
    rtc_ten = year;
    ones = year % 10;
    rtc_ten = rtc_ten - ones;
    rtc_date[5] = ((rtc_ten/10)<<4) | ones;

    ptr = &(rtc_date[0]);

    return ptr;
}

/*==============================================================================
  BME SPI initialization
 =============================================================================*/
uint8_t receive_data(uint8_t* rx_buffer);
int send_data(uint8_t* data_pointer, uint8_t bytes);
void rtc_setAlarm(uint32_t t_int);

/*==============================================================================
  main()
 =============================================================================*/


int main(void) {

    //Stop WDT
    WDT_A_hold(WDT_A_BASE);

    //Config MSP430
    initStartUp();

    /*
     * Disable the GPIO power-on default high-impedance mode to activate
     * previously configured port settings
     */
    PMM_unlockLPM5();

    __bis_SR_register(GIE); //Enable Interrupts


    selectVoltage(VOLTAGE_default); //defines the output voltage of the TPS62740

    //init SPI for the RTC
    spi_initMaster_RTC();

    //Init the RTC
    Buffer128_clean(&Buffer_RTC);

    /*
    // BME init
    int8_t rslt = BME280_OK;
    struct bme280_dev dev;
    struct bme280_data data;
    */

    dev.dev_id = 0;
    dev.intf = BME280_SPI_INTF;
    dev.read = user_spi_read;
    dev.write = user_spi_write;
    dev.delay_ms = user_delay_ms;

    // Recommended mode of operation: Indoor navigation
    dev.settings.osr_h = BME280_OVERSAMPLING_1X;
    dev.settings.osr_p = BME280_OVERSAMPLING_1X;
    dev.settings.osr_t = BME280_OVERSAMPLING_1X;
    dev.settings.filter = BME280_FILTER_COEFF_2;
    dev.settings.standby_time = BME280_STANDBY_TIME_1_MS;

    settings_sel = BME280_OSR_PRESS_SEL;
    settings_sel |= BME280_OSR_TEMP_SEL;
    settings_sel |= BME280_OSR_HUM_SEL;
    settings_sel |= BME280_STANDBY_SEL;
    settings_sel |= BME280_FILTER_SEL;

    /* Sensor_0 interface over SPI with native chip select line */
    //uint16_t MOISTURE_COUNTER_MAX = (hours_for_moisture*3600)/cyclic_wakeUpTime;      // when moisture counter checked every loop
    //uint16_t MOISTURE_COUNTER_MAX = (hours_for_moisture*3600)/(cyclic_wakeUpTime*FRAM_DATA_MAX);  // when m. counter checked in send routine
    //uint16_t MOISTURE_COUNTER_MAX = 5;

    //uint8_t test[20] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};

while(1)
{
    uint8_t* date_ptr;
    uint8_t send_moisture = 0;

    tmp_ui8 = spi_getRTC_status();

    if(tmp_ui8 != 8)    // see status register 0x0F, data sheet pg 64: 8 = countdown timer enabled
    {
        tmp_ui8 = spi_getRTC_status();
        //Software Reset of the RTC
        spi_configRTC_key(0x3C);

        timerDelay_LPM(10);     /* wait 10ms */

        spi_configRTC_init();
    }
    spi_clearRTC_status();

    // check Buttons
    Button_getValue();

    //check the PowerGood Pins
    checkPower();


    //read the "GP0"-Flag of the RTC
    // if it is "1" -> RTC was configured
    // if "0" -> first start of the RTC
    if(spi_getRTC_GP0() == 0)
    {   // first start of the RTC

        //Software Reset of the RTC
        spi_configRTC_key(AM1815_BM_KEY_RESET);

        timerDelay_LPM(10);     /* wait 10ms */

        spi_configRTC_init();

        spi_setRTC_GP0(0xFF);

        spi_clearRTC_status();

        //set time and date
        //insert current time (sent from raspberry)
        //date_ptr = RTC_createDate(01,05,14,7,11,17);
        //spi_configRTC_time(date_ptr);

        //clear the segment -> no Data for sending -> counter from saved Data is zero!
        FRAMCtl_write16(&FRAM_stored_data, (uint16_t *) FRAM_DATA_COUNTER, 1);
        FRAMCtl_write16(&FRAM_moisture_counter, (uint16_t *) FRAM_DATA_COUNTER_M, 1);
        FRAMCtl_write8(&FRAM_registration_done, (uint8_t*)FRAM_REGISTRATION,1);
        //clear the DataFrame number
        FRAMCtl_write16(&temperature_u16, (uint16_t *) (FRAM_DATA_SENS2_TEMP + 2), 1);
        FRAMCtl_write16(&pressure_u16, (uint16_t *) (FRAM_DATA_SENS2_PRESS + 2), 1);
        FRAMCtl_write16(&humidity_u16, (uint16_t *) (FRAM_DATA_SENS2_HUMIDITY + 2), 1);

        FRAMCtl_write16(&frequency_u16, (uint16_t *) (FRAM_DATA_MOISTURE + 2), 1);

    } else {
        //flag GP0 is set -> not the first start of the RTC
        // load FRAM counter variables
        //set pointer for use of the FRAM
        pointer_FRAM_ui16 = (uint16_t*) FRAM_DATA_COUNTER;
        FRAM_stored_data = *(pointer_FRAM_ui16);    // load the number of already saved data
        pointer_FRAM_ui16 = (uint16_t*) FRAM_DATA_COUNTER_M;
        FRAM_moisture_counter = *(pointer_FRAM_ui16);   // load number of saved data sets
        pointer_FRAM_ui8 = (uint8_t*) FRAM_REGISTRATION;
        FRAM_registration_done = *(pointer_FRAM_ui8);

    }

    // set the "wakeUp time" (original implementation)
    //spi_configRTC_countdowntimer(cyclic_wakeUpTime, true);

    /*
    spi_getRTC_Data(&date_test[0], AM1815_RA_SECONDS, 1);
    spi_getRTC_Data(&date_test[1], AM1815_RA_MINUTES, 1);
    spi_getRTC_Data(&date_test[2], AM1815_RA_HOURS, 1);
    spi_getRTC_Data(&date_test[3], AM1815_RA_DATE, 1);
    spi_getRTC_Data(&date_test[4], AM1815_RA_MONTH, 1);
    spi_getRTC_Data(&date_test[5], AM1815_RA_YEAR, 1);
    */
    Button_getValue();

    // wait for button press to register device to master
    if(FRAM_registration_done == 0)
    {
        while((Button1 == LOW) && (FRAM_registration_done == 0))
        {
            timerDelay_LPM(20);
            Button_getValue();
            //if(GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN4) == LOW)
                //SW101_event = 1;
        }

        //if(SW101_event == 1)
        if((Button1 == HIGH) && (FRAM_registration_done == 0))
        {
            //SW101_event = 0;
            tmp_ui16 = (((ID_of_Sensorboard & 0x7FF) << 5) + REGISTRATION_MESSAGE);
            registration[0] = (uint8_t)(tmp_ui16>>8);
            registration[1] = (uint8_t)(tmp_ui16 & 0x00FF);
            registration[2] = 0xAA;
            registration[3] = 0xAA;
            send_data(registration, 4);

            tmp_ui8 = receive_data(rx_buffer);

            FRAM_registration_done = 1;

            if(rx_buffer[0] == 0xAA)
            {
                offset_time = ((rx_buffer[7]<<8)|rx_buffer[8]);

                date_ptr = RTC_createDate(rx_buffer[6],rx_buffer[5],rx_buffer[4],rx_buffer[3],rx_buffer[2],rx_buffer[1]);
                spi_configRTC_time(date_ptr);

                //spi_getRTC_Data(&date_test[0], AM1815_RA_SECONDS, 6);
                //date_test[0] &= 0x7F;
                //date_test[1] &= 0x7F;
                spi_getRTC_time(date_test);

                //timerDelay_LPM(10000);
                //spi_getRTC_time(date_test);
                /*
                spi_getRTC_Data(&date_test[1], AM1815_RA_MINUTES, 1);
                spi_getRTC_Data(&date_test[2], AM1815_RA_HOURS, 1);
                spi_getRTC_Data(&date_test[3], AM1815_RA_DATE, 1);
                spi_getRTC_Data(&date_test[4], AM1815_RA_MONTH, 1);
                spi_getRTC_Data(&date_test[5], AM1815_RA_YEAR, 1);
                */
                // alarm:
                //date_ptr = RTC_createDate(ts->tm_sec,ts->tm_min,ts->tm_hour,ts->tm_mday,ts->tm_mon+1,ts->tm_year-30);
                //spi_setRTC_Data(date_ptr, AM1815_RA_SECONDS_ALARM, 5);
                //spi_startRTC_communication();
            }
            FRAMCtl_write8(&FRAM_registration_done, (uint8_t*)FRAM_REGISTRATION,1);
            //spi_configRTC_countdowntimer(tmp_ui16, true);
            //spi_clearRTC_status();
            // set the RTC to sleep
            //spi_setRTC_sleep(0);
            //timerDelay_LPM(5000);
        }
    }
    else
    {
        if(offset_time == 0)
        {
            spi_configRTC_countdowntimer(cyclic_wakeUpTime, true);
        }
        else
        {
            spi_configRTC_countdowntimer(offset_time, true);
            offset_time = 0;
        }


    if(FRAM_MAX_DATA > FRAM_stored_data)
    {

    // get data of the sensor
        spi_enable_Sensor_VCC();    //enable the Voltage for the Sensors

        spi_initMaster_Sensor();    //init the SPI for the Sensors

        rslt = bme280_init(&dev);

        rslt = bme280_set_sensor_settings(settings_sel, &dev);
        rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);

        //timerDelay_LPM(2);
        rslt = bme280_get_sensor_data(BME280_ALL, &data, &dev);
        //timerDelay_LPM(2);
        rslt = bme280_get_sensor_data(BME280_ALL, &data, &dev);
        //rslt = bme280_get_sensor_data(BME280_ALL, &data, &dev);
        //rslt = stream_sensor_data_normal_mode(&dev);

        pressure_u16 = (uint16_t) ((data.pressure+5)/10);  // xxxx,x hPa
        temperature_s16 = (int16_t) (data.temperature+4000);       // xx,xx °C
        humidity_u16 = (uint16_t) ((data.humidity+5)/10);   // xx,xx %

        if(FRAM_moisture_counter >= MOISTURE_COUNTER_MAX)
        {
            selectVoltage(VOLTAGE_33);
            frequency_u16 = (uint16_t) getFrequency();
            //FRAM_moisture_counter = 0;
            selectVoltage(VOLTAGE_default);
            FRAMCtl_write16(&frequency_u16, (uint16_t *) (FRAM_DATA_MOISTURE + 2), 1);
            send_moisture = 1;
        }

        //SPI_getBME280_data(&pressure_u16, &temperature_s16, &humidity_u16);
       // selectVoltage(VOLTAGE_33);
       // tmp_ui16 = (uint16_t) getFrequency(); // frequency test


        spi_disable_Sensor_VCC();   // disconnect the Sensors from the VCC -> save energy
        spi_DeinitMaster_Sensor();  //disable the communication with the Sensors


        selectVoltage(VOLTAGE_default); //defines the output voltage of the TPS62740

        // store them
        if(FRAM_stored_data == 0)
        {
            /* config the new FRAM */
            tmp_ui16 = (((ID_of_Sensorboard & 0x7FF) << 5) + FRAM_ID_SENS2_TEMP);
            FRAMCtl_write16(&tmp_ui16, (uint16_t *) FRAM_DATA_SENS2_TEMP, 1);

            tmp_ui16 = (((ID_of_Sensorboard & 0x7FF) << 5) + FRAM_ID_SENS2_PRESS );
            FRAMCtl_write16(&tmp_ui16, (uint16_t *) FRAM_DATA_SENS2_PRESS, 1);

            tmp_ui16 = ((ID_of_Sensorboard & 0x7FF) << 5) + FRAM_ID_SENS2_HUMIDITY;
            FRAMCtl_write16(&tmp_ui16, (uint16_t *) FRAM_DATA_SENS2_HUMIDITY, 1);

            tmp_ui16 = ((ID_of_Sensorboard & 0x7FF) << 5) + FRAM_ID_MOISTURE;
            FRAMCtl_write16(&tmp_ui16, (uint16_t *) FRAM_DATA_MOISTURE, 1);
        }

        //count the number of stored value up
        FRAM_stored_data ++;
        FRAM_moisture_counter++;

        //Typecast, to make a send possible
        // TODO: offset to -40°C
        temperature_u16 = (uint16_t) temperature_s16;

        /* store data from Sensor 2 (BME280) */
        FRAMCtl_write16(&temperature_u16, (uint16_t *) (FRAM_DATA_SENS2_TEMP + (2 * FRAM_stored_data) + 2), 1);
        FRAMCtl_write16(&pressure_u16, (uint16_t *) (FRAM_DATA_SENS2_PRESS + (2 * FRAM_stored_data) + 2), 1);
        FRAMCtl_write16(&humidity_u16, (uint16_t *) (FRAM_DATA_SENS2_HUMIDITY + (2 * FRAM_stored_data) + 2), 1);
        FRAMCtl_write16(&frequency_u16, (uint16_t *) (FRAM_DATA_MOISTURE + (2 * 1) + 2), 1);
    }

    ePG = 1;
    //send?
    if(((FRAM_stored_data >= FRAM_MAX_DATA)) && (ePG == 1)) //check "count value in the FRAM" and the "external PowerGood" pin
    {   //data has to be send!

        //init SPI for the RF
        spi_initMaster_RF();

        //start the RF-chip
        SPI_enable_RF();

        //set the optimal voltage to transmit data
        selectVoltage(VOLTAGE_22);  //2,2V is needed


        //init the RF-chip -> chip need ~650µS to start
        timerDelay_LPM(1);  //wait 1ms to be save
        spi_init_RF();


        SpiritPktStackRequireAck(S_DISABLE);
        /* put the RF into ready*/
        SpiritCmdStrobeReady();

        //load the data to send
        /* send temp at first */
        SpiritPktBasicSetPayloadLength((uint8_t) (2* FRAM_stored_data) + 4);
//      SpiritPktBasicSetPayloadLength((uint8_t) (2* FRAM_MAX_DATA) + 4);



        /*copy the information of the dataframe into the buffer */
        pointer_FRAM_ui16 = (uint16_t*) (FRAM_DATA_SENS2_TEMP + 2);
        tmp_ui16 = *(pointer_FRAM_ui16);    // load the number of dataFrames
        //make a mask on the dataframe, make + 1 to the "DataFrame_Number" and add the number of "stored data"
        tmp_ui16 = ((tmp_ui16 & 0xFFC0) + 0x40) + (FRAM_stored_data << 1) + RF_onemoreDataFrame;
        FRAMCtl_write16(&tmp_ui16, (uint16_t *) (FRAM_DATA_SENS2_TEMP + 2), 1);


        /* fit the TX FIFO */
        SpiritCmdStrobeFlushTxFifo();

        /* get status of the SPIRIT1 */
        SpiritRefreshStatus();
        if(g_xStatus.MC_STATE != MC_STATE_READY)
        {
            /* set the ready state */
            SpiritCmdStrobeSabort();
            do
            {
                SpiritRefreshStatus();
            }while(g_xStatus.MC_STATE!=MC_STATE_READY);

        }

        /* clear the Irq */
        SpiritIrqClearStatus();

        /* fit the TX FIFO */
        SpiritCmdStrobeFlushTxFifo();

        SpiritSpiWriteLinearFifo(((2* FRAM_stored_data) + 4), (uint8_t *) (FRAM_DATA_SENS2_TEMP));      //the FRAM data can be used direct, without store in an variable
        //SpiritSpiWriteLinearFifo(20,test);
        //SpiritSpiWriteLinearFifo(((2* FRAM_MAX_DATA) + 4), (uint8_t *) (FRAM_DATA_SENS2_TEMP));       //the FRAM data can be used direct, without store in an variable


        /* send the TX command */
        SpiritCmdStrobeTx();



        //wait until, the data are send
        __bis_SR_register(LPM0_bits + GIE);      // CPU off
        __no_operation();

        timerDelay_LPM(SEND_DELAY);

        //Put it in Ready-Mode
        SpiritCmdStrobeSabort();
        /* get status of the SPIRIT1 */
        SpiritRefreshStatus();
        if(g_xStatus.MC_STATE != MC_STATE_READY)
        {
            /* set the ready state */
            SpiritCmdStrobeSabort();
            do
            {
                SpiritRefreshStatus();
            }while(g_xStatus.MC_STATE!=MC_STATE_READY);

        }

        /* clear the Irq */
        SpiritIrqClearStatus();

        /*copy the information of the dataframe into the buffer */
        pointer_FRAM_ui16 = (uint16_t*) (FRAM_DATA_SENS2_PRESS + 2);
        tmp_ui16 = *(pointer_FRAM_ui16);    // load the number of dataFrames
        tmp_ui16 = ((tmp_ui16 & 0xFFC0) + 0x40) + (FRAM_stored_data << 1) + RF_onemoreDataFrame;    //make a mask on the dataframe, make + 1 to the "DataFrame_Number" and add the number of "stored data"
        FRAMCtl_write16(&tmp_ui16, (uint16_t *) (FRAM_DATA_SENS2_PRESS + 2), 1);


        /* fit the TX FIFO */
        SpiritCmdStrobeFlushTxFifo();
        SpiritSpiWriteLinearFifo(((2* FRAM_stored_data) + 4), (uint8_t *) (FRAM_DATA_SENS2_PRESS));     //the FRAM data can be used direct, without store in an variable
//SpiritSpiWriteLinearFifo(((2* FRAM_MAX_DATA) + 4), (uint8_t *) (FRAM_DATA_SENS2_HUMIDITY));       //the FRAM data can be used direct, without store in an variable


        /* send the TX command */
        SpiritCmdStrobeTx();

        //wait until, the data are send
        __bis_SR_register(LPM0_bits + GIE);      // CPU off
        __no_operation();

        timerDelay_LPM(SEND_DELAY);


        /* get status of the SPIRIT1 */
        SpiritRefreshStatus();
        if(g_xStatus.MC_STATE != MC_STATE_READY)
        {
            /* set the ready state */
            SpiritCmdStrobeSabort();
            do
            {
                SpiritRefreshStatus();
            }while(g_xStatus.MC_STATE!=MC_STATE_READY);

        }

        /* clear the Irq */
        SpiritIrqClearStatus();

        if(FRAM_moisture_counter >= MOISTURE_COUNTER_MAX)
        {
            send_moisture = 1;
            FRAM_moisture_counter = 0;
        }
        else
        {
            send_moisture = 0;
        }

        /*copy the information of the dataframe into the buffer */
        pointer_FRAM_ui16 = (uint16_t*) (FRAM_DATA_SENS2_HUMIDITY + 2);
        tmp_ui16 = *(pointer_FRAM_ui16);    // load the number of dataFrames
        tmp_ui16 = ((tmp_ui16 & 0xFFC0) + 0x40) + (FRAM_stored_data << 1) + send_moisture;  //make a mask on the dataframe, make + 1 to the "DataFrame_Number" and add the number of "stored data"
        FRAMCtl_write16(&tmp_ui16, (uint16_t *) (FRAM_DATA_SENS2_HUMIDITY + 2), 1);

        /* fit the TX FIFO */
        SpiritCmdStrobeFlushTxFifo();
        SpiritSpiWriteLinearFifo(((2* FRAM_stored_data) + 4), (uint8_t *) (FRAM_DATA_SENS2_HUMIDITY));      //the FRAM data can be used direct, without store in an variable
//SpiritSpiWriteLinearFifo(((2* FRAM_MAX_DATA) + 4), (uint8_t *) (FRAM_DATA_SENS2_PRESS));      //the FRAM data can be used direct, without store in an variable


        /* send the TX command */
        SpiritCmdStrobeTx();

        //wait until, the data are send
        __bis_SR_register(LPM0_bits + GIE);      // CPU off
        __no_operation();

        timerDelay_LPM(SEND_DELAY);
        if(send_moisture == 1)
        {
            send_moisture = 0;
            //-----------------------------------------------------------------------------
            SpiritRefreshStatus();
                    if(g_xStatus.MC_STATE != MC_STATE_READY)
                    {
                        /* set the ready state */
                        SpiritCmdStrobeSabort();
                        do
                        {
                            SpiritRefreshStatus();
                        }while(g_xStatus.MC_STATE!=MC_STATE_READY);

                    }
                    SpiritPktBasicSetPayloadLength(6);
                    /* clear the Irq */
                    SpiritIrqClearStatus();

                    /*copy the information of the dataframe into the buffer */
                    pointer_FRAM_ui16 = (uint16_t*) (FRAM_DATA_MOISTURE + 2);
                    tmp_ui16 = *(pointer_FRAM_ui16);    // load the number of dataFrames
                    tmp_ui16 = ((tmp_ui16 & 0xFFC0) + 0x40) + (FRAM_stored_data << 1);  //make a mask on the dataframe, make + 1 to the "DataFrame_Number" and add the number of "stored data"
                    FRAMCtl_write16(&tmp_ui16, (uint16_t *) (FRAM_DATA_MOISTURE + 2), 1);

                    /* fit the TX FIFO */
                    SpiritCmdStrobeFlushTxFifo();
                    SpiritSpiWriteLinearFifo(((2* 1) + 4), (uint8_t *) (FRAM_DATA_MOISTURE));     //the FRAM data can be used direct, without store in an variable
            //SpiritSpiWriteLinearFifo(((2* FRAM_MAX_DATA) + 4), (uint8_t *) (FRAM_DATA_SENS2_PRESS));      //the FRAM data can be used direct, without store in an variable


                    /* send the TX command */
                    SpiritCmdStrobeTx();

                    //wait until, the data are send
                    __bis_SR_register(LPM0_bits + GIE);      // CPU off
                    __no_operation();
        //--------------------------------------------------------------------------------------------
        }

        /* all Data are send!!! */

        /* clear the Irq */
        SpiritIrqClearStatus();


        //data was send -> set FRAM_stored_data to zero
        FRAM_stored_data = 0;

        spi_Deinit_RF();
        spi_DeinitMaster_RF();
        SPI_disable_RF();
    }

    if(FRAM_stored_data >= FRAM_MAX_DATA)
    {
        FRAM_stored_data = 0;
    }

    //save the number of stored data
    FRAMCtl_write16(&FRAM_stored_data, (uint16_t *) FRAM_DATA_COUNTER, 1);
    FRAMCtl_write16(&FRAM_moisture_counter, (uint16_t*) FRAM_DATA_COUNTER_M,1);

    //read the Flag "SLST" of the RTC???

    /*
    tmp_ui8 = spi_getRTC_status();
    if(tmp_ui8 != 0)
    {
        // set the "wakeUp time"
        spi_configRTC_countdowntimer(cyclic_wakeUpTime, true);
        tmp_ui8 = spi_getRTC_status();
    }
    */
    // set wakeup alarm:
    //t_int = (uint32_t) time(NULL);
    //rtc_setAlarm(t_int);

    spi_clearRTC_status();
    // set the RTC to sleep
    spi_setRTC_sleep(0);

    //no more power -> power off -> for safety -> shut down the MCU in 1ms
    timerDelay_LPM((20+cyclic_wakeUpTime) * 100);
    } // if/else registration_done
}   //for the while(1), to test the writing and reading of the FRAM

}

uint8_t receive_data(uint8_t* rx_buffer)
{
    uint8_t irq_rx_data_ready = 0;
    uint8_t received_bytes = 0;
   spi_initMaster_RF();
   SPI_enable_RF();
   selectVoltage(VOLTAGE_22);
   timerDelay_LPM(1);  //wait 1ms to be save
   spi_init_RF();
   SpiritPktStackRequireAck(S_DISABLE);
   SpiritCmdStrobeReady();
    SpiritRefreshStatus();
    if(g_xStatus.MC_STATE != MC_STATE_RX)
    {
        do
        {
            SpiritSpiCommandStrobes(COMMAND_RX);
            SpiritRefreshStatus();
            if(g_xStatus.MC_STATE==0x13 || g_xStatus.MC_STATE==0x0)
            {
                SpiritCmdStrobeRx();
            }

        }while(g_xStatus.MC_STATE!=MC_STATE_RX);
    }
    do
    {
        irq_rx_data_ready = SpiritIrqCheckFlag(RX_DATA_READY);
        SpiritRefreshStatus();
        if(g_xStatus.MC_STATE != MC_STATE_RX)
        {
            do
            {
                SpiritCmdStrobeRx();
                SpiritRefreshStatus();
            }while(g_xStatus.MC_STATE!=MC_STATE_RX);
        }
        timerDelay_LPM(1);

    }while(irq_rx_data_ready == 0);

    if(irq_rx_data_ready == 1)
    {
        SpiritIrqClearStatus();
        received_bytes = SpiritLinearFifoReadNumElementsRxFifo();
        SpiritSpiReadLinearFifo(received_bytes, rx_buffer);
        SpiritCmdStrobeFlushRxFifo();
    }
    spi_Deinit_RF();
    spi_DeinitMaster_RF();
    SPI_disable_RF();

    return received_bytes;
}

int send_data(uint8_t* data_pointer, uint8_t bytes)
{
    spi_initMaster_RF();
   SPI_enable_RF();
   selectVoltage(VOLTAGE_22);
   timerDelay_LPM(1);  //wait 1ms to be save
   spi_init_RF();
   SpiritPktStackRequireAck(S_DISABLE);
    SpiritPktBasicSetPayloadLength(bytes);
    SpiritIrqClearStatus();
    SpiritCmdStrobeFlushTxFifo();
    SpiritCmdStrobeReady();
    SpiritRefreshStatus();

    if(g_xStatus.MC_STATE != MC_STATE_READY)
    {
    // set the ready state
        SpiritCmdStrobeSabort();
        do
        {
            SpiritRefreshStatus();
        }while(g_xStatus.MC_STATE!=MC_STATE_READY);
    }
    SpiritIrqClearStatus();
    SpiritCmdStrobeFlushTxFifo();

    SpiritSpiWriteLinearFifo(bytes, data_pointer);

    SpiritCmdStrobeTx();
    //timerDelay_LPM(20);
    __bis_SR_register(LPM0_bits + GIE);      // CPU off
    __no_operation();

    SpiritCmdStrobeSabort();
    SpiritRefreshStatus();

    if(g_xStatus.MC_STATE != MC_STATE_READY)
    {
        // set the ready state
        SpiritCmdStrobeSabort();
        do
        {
            SpiritRefreshStatus();
        }while(g_xStatus.MC_STATE!=MC_STATE_READY);
    }
    SpiritIrqClearStatus();

    spi_Deinit_RF();
    spi_DeinitMaster_RF();
    SPI_disable_RF();

    return 1;
}

void rtc_setAlarm(uint32_t t_int)
{
    uint8_t * date_ptr;
    time_t t = (time_t) (t_int + ((uint32_t) cyclic_wakeUpTime));
    struct tm * ts;
    ts = localtime(&t);
    date_ptr = RTC_createDate(ts->tm_sec,ts->tm_min,ts->tm_hour,ts->tm_mday,ts->tm_mon+1,ts->tm_year-100);
    spi_setRTC_Data(date_ptr, AM1815_RA_SECONDS_ALARM, 5);  //set Alarm

}