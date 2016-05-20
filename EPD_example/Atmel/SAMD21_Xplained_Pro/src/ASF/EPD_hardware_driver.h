/**
* \file
*
* \brief The SPI, GPIO, PWM, Temperature definitions of COG hardware driver
*
* Copyright (c) 2012-2015 Pervasive Displays Inc. All rights reserved.
*
*  Authors: Pervasive Displays Inc.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef 	DISPLAY_HARDWARE_DRIVCE_H_INCLUDED_
#define 	DISPLAY_HARDWARE_DRIVCE_H_INCLUDED_

#include "conf_EPD.h"
#include <stdio.h>
#define	_BV(bit)   (1 << (bit)) /**< left shift 1 bit */
#define	_HIGH      1            /**< signal high */
#define	_LOW       !_HIGH       /**< signal low */

#define	config_gpio_dir_o(Pin)  ioport_set_pin_dir(Pin, IOPORT_DIR_OUTPUT) /**< set output direction for an IOPORT pin */
#define	config_gpio_dir_i(Pin)  ioport_set_pin_dir(Pin, IOPORT_DIR_INPUT)  /**< set input direction for an IOPORT pin */
#define	set_gpio_high(Pin)      ioport_set_pin_level(Pin,true) /**< set HIGH for an IOPORT pin */
#define	set_gpio_low(Pin)       ioport_set_pin_level(Pin,false)  /**< set LOW for an IOPORT pin */
#define	set_gpio_invert(Pin)    ioport_toggle_pin_level(Pin) /**< toggle the value of an IOPORT pin */
#define	input_get(Pin)          ioport_get_pin_level(Pin)    /**< get current value of an IOPORT pin */



/******************************************************************************
* GPIO Defines
*****************************************************************************/
#define Temper_PIN          EXT3_PIN_3  /**<  SAMD20 EXT3.pin3.   PA02, ADC0    */
#define SPI_CLK_PIN         EXT3_PIN_18 /**<  SAMD20 EXT3.pin18.  PB23, SPI_SCK */
#define EPD_BUSY_PIN        EXT3_PIN_9  /**<  SAMD20 EXT3.pin9.   PA28          */
#define PWM_PIN             EXT3_PIN_7  /**<  SAMD20 EXT3.pin7.   PA12,         */
#define EPD_RST_PIN         EXT3_PIN_6  /**<  SAMD20 EXT3.pin6.   PA15          */
#define EPD_PANELON_PIN     EXT3_PIN_8  /**<  SAMD20 EXT3.pin8.   PA13          */
#define EPD_DISCHARGE_PIN   EXT3_PIN_5  /**<  SAMD20 EXT3.pin5.   PB30          */
#define EPD_BORDER_PIN      EXT3_PIN_4  /**<  SAMD20 EXT3.pin4.   PA03          */
#define SPI_MISO_PIN        EXT3_PIN_17 /**<  SAMD20 EXT3.pin17.  PB16          */
#define SPI_MOSI_PIN        EXT3_PIN_16 /**<  SAMD20 EXT3.pin16.  PB22          */
#define Flash_CS_PIN        EXT3_PIN_10 /**<  SAMD20 EXT3.pin10.  PA27          */
#define EPD_CS_PIN          EXT3_PIN_15 /**<  SAMD20 EXT3.pin15.  PB17          */

#define EPD_IsBusy()        (bool)input_get(EPD_BUSY_PIN)
#define EPD_cs_high()       set_gpio_high(EPD_CS_PIN)
#define EPD_cs_low()        set_gpio_low(EPD_CS_PIN)
#define EPD_flash_cs_high() set_gpio_high(Flash_CS_PIN)
#define EPD_flash_cs_low()  set_gpio_low(Flash_CS_PIN)
#define EPD_rst_high()      set_gpio_high(EPD_RST_PIN)
#define EPD_rst_low()       set_gpio_low(EPD_RST_PIN)
#define EPD_discharge_high()set_gpio_high(EPD_DISCHARGE_PIN)
#define EPD_discharge_low() set_gpio_low(EPD_DISCHARGE_PIN)
#define EPD_Vcc_turn_on()   set_gpio_high(EPD_PANELON_PIN)
#define EPD_Vcc_turn_off()  set_gpio_low(EPD_PANELON_PIN)
#define EPD_border_high()   set_gpio_high(EPD_BORDER_PIN)
#define EPD_border_low()    set_gpio_low(EPD_BORDER_PIN)
#define EPD_pwm_high()      set_gpio_high(PWM_PIN)
#define EPD_pwm_low()       set_gpio_low(PWM_PIN)
#define SPIMISO_low()       set_gpio_low(SPIMISO_PIN)
#define SPIMOSI_low()       set_gpio_low(SPIMOSI_PIN)
#define SPICLK_low()        set_gpio_low(SPICLK_PIN)
//========================================================================================================
#define TC_CLK_Source			GCLK_GENERATOR_1
#define TC_HW                   TC0
#define TC_CONUT                (system_gclk_gen_get_hz(TC_CLK_Source)/1000) /**< Timer/Counter(TC)  */


/**SPI Defines ****************************************************************/
#define SPI_MODULE  EXT3_SPI_MODULE
#define SPI_SERCOM_MUX  EXT3_SPI_SERCOM_MUX_SETTING
#define  EX_SPI_MISO_PAD EXT3_SPI_SERCOM_PINMUX_PAD0
#define  EX_SPI_MOSI_PAD EXT3_SPI_SERCOM_PINMUX_PAD2
#define  EX_SPI_SCK_PAD EXT3_SPI_SERCOM_PINMUX_PAD3
#define SPI_baudrate            COG_SPI_baudrate           /**< the baud rate of SPI */

/**Temperature ADC Defines****************************************************/
#define Temperature_Sensor_ADC  EXT3_ADC_0_CHANNEL /**< MUX selection on Positive ADC input channel 0 */
#define _TempeScaled            2.5f  /**< the scale of temperature circuit */
#define _ADCRefVcc              1.65f /**< ADC ref voltage = VCC/2 = 3.3/2 */
#define _ADCres                 (float)pow(2,12) /**< 2 ^12 */
#define _DegCOffset             1.0f  /**< the offset of SAMD20 MCU */
#define _ADCSampleCount         16    /**< ADC sampling counter */

void epd_spi_init (void);
void epd_spi_attach (void);
void epd_spi_detach (void);
void epd_spi_send (unsigned char Register, unsigned char *Data, unsigned Length);
void epd_spi_send_byte (uint8_t Register, uint8_t Data);
uint8_t epd_spi_read(uint16_t RDATA);
void epd_spi_write (unsigned char Data);
uint8_t epd_spi_write_ex (unsigned char Data);
void sys_delay_ms(unsigned int ms);
void start_EPD_timer(void);
void stop_EPD_timer(void);
uint32_t get_current_time_tick(void);
void set_current_time_tick(uint32_t count);
void PWM_start_toggle(void);
void PWM_stop_toggle(void);
void PWM_run(uint16_t time);
void initialize_temperature(void);
int16_t get_temperature(void);
void EPD_display_hardware_init (void);
uint8_t SPI_R(uint8_t Register, uint8_t Data);
#endif 	//DISPLAY_HARDWARE_DRIVCE_H_INCLUDED_
