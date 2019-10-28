#ifndef ARGOS_PROTO_V1_H
#define ARGOS_PROTO_V1_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

#define LEDS_NUMBER    3

#define LED_START      13
#define LED_1          13
#define LED_2          14
#define LED_3          15
#define LED_STOP       15

#define LEDS_ACTIVE_STATE 0

#define LEDS_INV_MASK  LEDS_MASK

#define LEDS_LIST { LED_1, LED_2, LED_3 }

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2
#define BSP_LED_2      LED_3
  // #define BSP_LED_3      LED_3

#define RX_PIN_NUMBER  26
#define TX_PIN_NUMBER  25
#define CTS_PIN_NUMBER 7
#define RTS_PIN_NUMBER 5
#define HWFC           false
  
#define BUTTONS_NUMBER 1

#define BUTTON_START   28
#define BUTTON_1       28
#define BUTTON_2       29
#define BUTTON_3       30
#define BUTTON_4       31
#define BUTTON_STOP    31
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1 }

#define BSP_BUTTON_0   BUTTON_1

#define ADS_MISO_PIN   4 
#define ADS_CSN_PIN    7 
#define ADS_MOSI_PIN   6 
#define ADS_SCK_PIN    5 
#define ADS_RST_PIN    10
#define ADS_DRDY_PIN   3
#define ADS_START_PIN  9
#define ADS_CLKSEL_PIN 11
  

#ifdef __cplusplus
}
#endif

#endif 
