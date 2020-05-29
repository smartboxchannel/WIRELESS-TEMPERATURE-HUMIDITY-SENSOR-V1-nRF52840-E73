// ########################################  Thermometer and hygrometer on nRF52840 #################################################### //
//                                                                                                                                       //
//                                                                                                                                       //
//                                                                                                                                       //
//                                                                                                                                       //
//                                                                                                                                       //
//                                                                                                                                       //
//                                                                                                                                       //
// ##################################################################################################################################### //

#ifndef _MYBOARDNRF5_H_
#define _MYBOARDNRF5_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// Number of pins defined in PinDescription array
#define PINS_COUNT           (32u)
#define NUM_DIGITAL_PINS     (32u)
#define NUM_ANALOG_INPUTS    (8u)
#define NUM_ANALOG_OUTPUTS   (8u)


//   LEDs

#define PIN_LED1              (5)
#define LED_BUILTIN           PIN_LED1
#define BLUE_LED              (PIN_LED1)


//   Buttons

#define PIN_BUTTON             (3)


    /*
       Analog ports

       If you change g_APinDescription, replace PIN_AIN0 with
       port numbers mapped by the g_APinDescription Array.
       You can add PIN_AIN0 to the g_APinDescription Array if
       you want provide analog ports MCU independed, you can add
       PIN_AIN0..PIN_AIN7 to your custom g_APinDescription Array
       defined in MyBoardNRF5.cpp
    */
    
static const uint8_t A0  = ADC_A0;
static const uint8_t A1  = ADC_A1;
static const uint8_t A2  = ADC_A2;
static const uint8_t A3  = ADC_A3;
static const uint8_t A4  = ADC_A4;
static const uint8_t A5  = ADC_A5;
static const uint8_t A6  = ADC_A6;
static const uint8_t A7  = ADC_A7;

/*
   Serial interfaces

   RX and TX are required.
   If you have no serial port, use unused pins
   CTS and RTS are optional.
*/
#define PIN_SERIAL_RX       (9)
#define PIN_SERIAL_TX       (10)

/*
   SPI Interfaces

   This is optional

   If SPI is defined MISO, MOSI, SCK are required
   SS is optional and can be used in your sketch.
*/
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (2)
#define PIN_SPI_MOSI         (4)
#define PIN_SPI_SCK          (15)
#define PIN_SPI_SS           (17)


static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

/*
   Wire Interfaces

   This is optional
*/
#define WIRE_INTERFACES_COUNT 2

#define PIN_WIRE_SDA         (30u)
#define PIN_WIRE_SCL         (31u)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#ifdef __cplusplus
}
#endif

#endif
