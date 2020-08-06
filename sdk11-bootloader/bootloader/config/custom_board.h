/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

// LEDs definitions for PCA10040
#define LEDS_NUMBER    0

//#define LED_START      17
//#define LED_STOP       20

#define LEDS_LIST {  }


#define LEDS_MASK      0
/* all LEDs are lit when GPIO is low */
#define LEDS_INV_MASK  LEDS_MASK

#define BUTTONS_NUMBER 0

//#define BUTTON_START   13
//#define BUTTON_STOP    16
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_LIST {  }

#define BUTTONS_MASK   0


// Low frequency clock source to be used by the SoftDevice
// https://devzone.nordicsemi.com/f/nordic-q-a/15363/change-clock-source-nrf52832-app-beacon-example
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_RC,              \
                                 .rc_ctiv       = 16,                               \
                                 .rc_temp_ctiv  = 2,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_250_PPM}

#endif // CUSTOM_BOARD_H
