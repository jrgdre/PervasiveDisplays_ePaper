/**
 * \file
 *
 * \brief The initialization and configuration of COG hardware driver
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

#include "conf_EPD.h"
void EPD_initialize_gpio(void);
static volatile uint32_t EPD_Counter;
struct tc_module tc_instance;
volatile bool PWM_Run_Flag;
void tc_callback_Handler(struct tc_module *constmodule_inst);

/**
* \brief Set up EPD Timer for 1 mSec interrupts
*
* \note
* desired value: 1mSec
* actual value:  1.000mSec
*/
static void initialize_EPD_timer(void) {

	 struct tc_config config_tc;
	 
	 tc_reset(&tc_instance);
	 tc_get_config_defaults(&config_tc);
	 config_tc.clock_source=TC_CLK_Source;
	 config_tc.clock_prescaler=TC_CLOCK_PRESCALER_DIV8;
	 config_tc.counter_size = TC_COUNTER_SIZE_32BIT;
	 config_tc.reload_action= TC_RELOAD_ACTION_RESYNC;
	 config_tc.count_direction= TC_COUNT_DIRECTION_UP;
	 config_tc.counter_32_bit.value=0;

	 tc_init(&tc_instance, TC_HW, &config_tc);
	 tc_register_callback(&tc_instance, tc_callback_Handler, TC_CALLBACK_CC_CHANNEL0);
	 tc_enable_callback(&tc_instance, TC_CALLBACK_CC_CHANNEL0);
	 tc_set_count_value(&tc_instance, 0);
	 tc_enable(&tc_instance);

	 EPD_Counter=0;
}

/**
* \brief Start Timer
*/
void start_EPD_timer(void) {
	initialize_EPD_timer();
}

/**
* \brief Stop Timer
*/
void stop_EPD_timer(void) {
    tc_stop_counter(&tc_instance);
	tc_disable(&tc_instance);
	
}

/**
* \brief Get current Timer after starting a new one
*/
uint32_t get_current_time_tick(void) {
	EPD_Counter=tc_get_count_value(&tc_instance)/1000;
	return EPD_Counter;
}
/**
* \brief Set PWM pin to Toggle
*/
static void EPD_pwm_Toggle(void) {
	set_gpio_invert(PWM_PIN);

}
/**
* \brief callback Service Routine for TC0 tick counter
*/
void tc_callback_Handler(
struct tc_module *constmodule_inst){
	EPD_Counter++;
    tc_set_count_value(&tc_instance,TC_CONUT); 
   if(PWM_Run_Flag)
   {
     EPD_pwm_Toggle();    
   }
   
}

/**
* \brief Delay mini-seconds
* \param ms The number of mini-seconds
*/
void sys_delay_ms(unsigned int ms) {
	uint32_t curTicks;
	start_EPD_timer();
	curTicks = EPD_Counter;
	while ((EPD_Counter - curTicks) < ms) __WFI();
	stop_EPD_timer();
}


static void Wait_10us(void) {
	//delay_us(10);
}

//******************************************************************
//* PWM  Configuration/Control //PWM output : PD3
//******************************************************************

/**
* \brief The PWM signal starts toggling
*/
void PWM_start_toggle(void) {
    PWM_Run_Flag=true;
}


/**
* \brief The PWM signal stops toggling.
*/
void PWM_stop_toggle(void) {
	PWM_Run_Flag=false;
    EPD_pwm_low();
}

/**
* \brief PWM toggling.
*
* \param ms The interval of PWM toggling (mini seconds)
*/
void PWM_run(uint16_t ms) {
  
	PWM_start_toggle();
	delay_ms(ms);
	PWM_stop_toggle();
	
}

//******************************************************************
//* SPI  Configuration
//******************************************************************
static struct spi_module spi_master_instance;
static bool spi_flag=FALSE;
//#define SLAVE_SELECT_PIN Flash_CS_PIN
/**
* \brief Configure SPI
*/
void epd_spi_init(void) {
	if(spi_flag) return;
	struct spi_config config_spi_master;  
	/* Configure, initialize and enable SERCOM SPI module */
	spi_get_config_defaults(&config_spi_master);

	config_spi_master.mux_setting = SPI_SERCOM_MUX;
	config_spi_master.transfer_mode=SPI_TRANSFER_MODE_0;
    config_spi_master.mode_specific.master.baudrate=SPI_baudrate;
	/* Configure pad 0 for data in */
	config_spi_master.pinmux_pad0 = EX_SPI_MISO_PAD;
	/* Configure pad 1 as unused */
	config_spi_master.pinmux_pad1 =PINMUX_UNUSED;
	/* Configure pad 2 for data out */
	config_spi_master.pinmux_pad2 = EX_SPI_MOSI_PAD;
	/* Configure pad 3 for SCK */
	config_spi_master.pinmux_pad3 = EX_SPI_SCK_PAD;
	spi_init(&spi_master_instance, SPI_MODULE, &config_spi_master);
	spi_enable(&spi_master_instance);
	spi_flag=TRUE;
}

/**
* \brief Initialize SPI
*/
void epd_spi_attach (void) {
	epd_spi_init();

}

/**
* \brief Disable SPI and change to GPIO
*/
void epd_spi_detach (void) {
	spi_disable(&spi_master_instance);
	
    ioport_enable_pin(SPI_CLK_PIN);    
    ioport_enable_pin(SPI_MOSI_PIN);
	ioport_enable_pin(SPI_MISO_PIN);
	
    config_gpio_dir_o(SPI_CLK_PIN);
    config_gpio_dir_o(SPI_MOSI_PIN);
    config_gpio_dir_o(SPI_MISO_PIN);
	
    set_gpio_low(SPI_CLK_PIN);
    set_gpio_low(SPI_MISO_PIN);
    set_gpio_low(SPI_MOSI_PIN);
	spi_flag=FALSE;
	
}

/**
 * \brief Send data to SPI
 *
 * \param Data The data to be sent out
 */
void epd_spi_write (unsigned char Data) {
	//uint16_t retval;
	//spi_master_instance.hw->SPI.CTRLB.bit.RXEN=0;
	spi_write(&spi_master_instance, Data);
	while (!spi_is_write_complete(&spi_master_instance));
	//while (!spi_is_ready_to_read(&spi_master_instance));
	//spi_read(&spi_master_instance, &retval);
	
}

/**
 * \brief SPI synchronous read
 *
 * \param RDATA The data to be read
 */
uint8_t epd_spi_read(uint16_t rdata) {
	//spi_master_instance.hw->SPI.CTRLB.bit.RXEN=1;
	spi_transceive_wait(&spi_master_instance,rdata, (uint16_t *)&rdata);
	return rdata;
}
/**
 * \brief Send data to SPI with time out feature
 *
 * \param Data The data to be sent out
 */
uint8_t epd_spi_write_ex (unsigned char Data) {
	uint8_t cnt=200;
	uint8_t flag=1;
    while (!spi_is_ready_to_write(&spi_master_instance));
   	spi_write(&spi_master_instance, (uint16_t)Data);
	while (!spi_is_write_complete(&spi_master_instance))
	{
        if((cnt--)==0) {
			flag=0;
			break;
		}
	}
	return flag;
}


/**
* \brief SPI command
*
* \param Register The Register Index as SPI Data to COG
* \param Data The Register Data for sending command data to COG
* \return the SPI read value
*/
uint8_t SPI_R(uint8_t Register, uint8_t Data) {
	uint8_t result;
	spi_master_instance.hw->SPI.CTRLB.bit.RXEN=0;
	EPD_cs_low ();
	epd_spi_write (0x70); // header of Register Index
	epd_spi_write (Register);

	EPD_cs_high ();
	Wait_10us ();	
	
	EPD_cs_low ();

	epd_spi_write (0x73); // header of Register Data of read command
	spi_master_instance.hw->SPI.CTRLB.bit.RXEN=1;
	result=epd_spi_read (Data);

	EPD_cs_high ();

	return result;
}


/**
* \brief SPI command if register data is larger than two bytes
*
* \param register_index The Register Index as SPI command to COG
* \param register_data The Register Data for sending command data to COG
* \param length The number of bytes of Register Data which depends on which
* Register Index is selected.
*/
void epd_spi_send (unsigned char register_index, unsigned char *register_data,
               unsigned length) {
	spi_master_instance.hw->SPI.CTRLB.bit.RXEN=0;
	EPD_cs_low ();
	epd_spi_write (0x70); // header of Register Index
	epd_spi_write (register_index);

	EPD_cs_high ();
	Wait_10us ();
	EPD_cs_low ();

	epd_spi_write (0x72); // header of Register Data of write command
	while(length--) {
	   epd_spi_write (*register_data++);
	}
	EPD_cs_high ();
}

/**
* \brief SPI command
*
* \param register_index The Register Index as SPI command to COG
* \param register_data The Register Data for sending command data to COG
*/
void epd_spi_send_byte (uint8_t register_index, uint8_t register_data) {
	spi_master_instance.hw->SPI.CTRLB.bit.RXEN=0;
	EPD_cs_low ();
	epd_spi_write (0x70); // header of Register Index
	epd_spi_write (register_index);

	EPD_cs_high ();
	Wait_10us ();
	EPD_cs_low ();
	epd_spi_write (0x72); // header of Register Data
	epd_spi_write (register_data);
	EPD_cs_high ();
}

//******************************************************************
//* Temperature sensor  Configuration
//******************************************************************
/** ADC instance */
struct adc_module adc_instance;
/**
* \brief ADC trigger conversion
*
* \return the ADC conversion value
*/
static inline uint16_t get_ADC_value(void) {
	/** Conversion value */
    uint16_t g_adc_sample_data;
	adc_start_conversion(&adc_instance);
	do {
		/* Wait for conversion to be done and read out result */
	} while (adc_read(&adc_instance, &g_adc_sample_data) == STATUS_BUSY);	
	return g_adc_sample_data;
}

/**
* \brief Get temperature value from ADC
*
* \return the Celsius temperature
*/
int16_t get_temperature(void) {

	uint8_t	i;
	long ADC_value;
	float Vadc=0.0;
	float degC=0.0;
	adc_enable(&adc_instance);
	for(i=0; i<10; i++)get_ADC_value();
	ADC_value=0;
	for(i=0; i<_ADCSampleCount; i++) {
		ADC_value+=get_ADC_value();
	}
	ADC_value=ADC_value/_ADCSampleCount;
	adc_disable(&adc_instance);
	Vadc=(_ADCRefVcc/_ADCres)*ADC_value*_TempeScaled;
	degC=(100.0f+_DegCOffset)-(float)(((Vadc-1.199f)*1000.0f)/10.77f);
	return   (int16_t)degC;
}

/**
* \brief Initialize the temperature sensor
*/
void initialize_temperature(void) {
	struct adc_config config_adc;
    adc_get_config_defaults(&config_adc);
    config_adc.gain_factor =ADC_GAIN_FACTOR_1X;
    config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV16;
    config_adc.reference = ADC_REFERENCE_INTVCC1;
    config_adc.positive_input = Temperature_Sensor_ADC;
    config_adc.resolution = ADC_RESOLUTION_12BIT;
    adc_init(&adc_instance, ADC, &config_adc);
}

/**
* \brief Configure GPIO
*/
void EPD_initialize_gpio(void) {
	 ioport_init();
	config_gpio_dir_i( EPD_BUSY_PIN);
	config_gpio_dir_o( EPD_CS_PIN);
	config_gpio_dir_o( EPD_RST_PIN);
	config_gpio_dir_o( EPD_PANELON_PIN);
	config_gpio_dir_o( EPD_DISCHARGE_PIN);
	config_gpio_dir_o( EPD_BORDER_PIN);
	config_gpio_dir_o( Flash_CS_PIN);
	config_gpio_dir_o( PWM_PIN);
	config_gpio_dir_i( Temper_PIN);
	EPD_flash_cs_high();
	EPD_border_low();
}

/**
* \brief Initialize the EPD hardware setting
*/
void EPD_display_hardware_init (void) {
	
	EPD_initialize_gpio();	   
	EPD_Vcc_turn_off();		
	initialize_temperature();
	EPD_cs_low();
	EPD_pwm_low();
	EPD_rst_low();
	EPD_discharge_low();
	EPD_border_low();
	//initialize_EPD_timer();
	epd_spi_attach();
	PWM_Run_Flag=false;
}