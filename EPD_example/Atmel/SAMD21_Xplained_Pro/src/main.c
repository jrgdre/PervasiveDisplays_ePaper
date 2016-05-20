/**
* \file
*
* \brief Sample project code for demonstrating Pervasive Displays 1.44", 1.9", 2", 2.6" and 2.7" EPD
*
* Copyright (c) 2012-2015 Pervasive Displays Inc. All rights reserved.
*
* \page License
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. The name of Atmel may not be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* 4. This software may only be redistributed and used in connection with an
*    Atmel microcontroller product.
*
* THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
* EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
**/

/**
* \brief Demonstration for toggling between two images on EPD of PDi extension board
* with Atmel SAM D21 Xplained Pro kit
*
* \par Content
* -# Include the ASF header files (through asf.h)
* -# Include Pervasive_Displays_small_EPD.h: EPD definitions
* -# Include image_data.h: image data array
*/
#include <asf.h>
#include "conf_EPD.h"
#include "image_data.h"

/**
* \brief The main function will toggle between two images on
* corresponding EPD depends on specified EPD size
*/
int main (void) {
	/* Initialize system clock and SAM D21 Xplained pro board */
    system_clock_init();
	delay_init();
	delay_ms(1000);
	/* Initialize EPD hardware */
	EPD_display_init();
   
    system_interrupt_enable_global();

	for(;;) {		
		/* The Image data arrays for each EPD size are defined at image_data.c */
#if(USE_EPD_Type==EPD_144)
		EPD_display_from_pointer(EPD_144,(uint8_t *)&image_array_144_2,(uint8_t *)&image_array_144_1);
#elif(USE_EPD_Type==EPD_200)
		EPD_display_from_pointer(EPD_200,(uint8_t *)&image_array_200_2,(uint8_t *)&image_array_200_1);
#elif(USE_EPD_Type==EPD_270)
		EPD_display_from_pointer(EPD_270,(uint8_t *)&image_array_270_2,(uint8_t *)&image_array_270_1);
#elif(USE_EPD_Type==EPD_190)
		EPD_display_from_pointer(EPD_190,(uint8_t *)&image_array_190_2,(uint8_t *)&image_array_190_1);
#elif(USE_EPD_Type==EPD_260)
		EPD_display_from_pointer(EPD_260,(uint8_t *)&image_array_260_2,(uint8_t *)&image_array_260_1);
#endif

		/* The interval of two images alternatively change is 10 seconds */
		delay_ms(10000);

#if(USE_EPD_Type==EPD_144)
		EPD_display_from_pointer(EPD_144,(uint8_t *)&image_array_144_1,(uint8_t *)&image_array_144_2);
#elif(USE_EPD_Type==EPD_200)
		EPD_display_from_pointer(EPD_200,(uint8_t *)&image_array_200_1,(uint8_t *)&image_array_200_2);
#elif(USE_EPD_Type==EPD_270)
		EPD_display_from_pointer(EPD_270,(uint8_t *)&image_array_270_1,(uint8_t *)&image_array_270_2);
#elif(USE_EPD_Type==EPD_190)
		EPD_display_from_pointer(EPD_190,(uint8_t *)&image_array_190_1,(uint8_t *)&image_array_190_2);
#elif(USE_EPD_Type==EPD_260)
		EPD_display_from_pointer(EPD_260,(uint8_t *)&image_array_260_1,(uint8_t *)&image_array_260_2);
#endif

		/* The interval of two images alternatively change is 10 seconds */
		delay_ms(10000);
	}
}


/**
 * \page - Quick Start Guide
 *
 * This is the quick start guide for the EPD Xplained Pro extension board made by Pervasive Displays Inc.
 * with its small size EPDs on how to setup the kit for Atmel Xplained Pro.
 * The code example in main.c provides the demo of toggling between two images from predefined image array.
 *
 * \note
 * - Released Date: 10 October, 2015.  Version: 1.0 (based on PDI's Released Date: 18 May, 2015.  Version: 2.02 for SAMD21)
 * - Compiled by Atmel Studio ver.6.0.594 with ASF ver.3.27.0
 * - PDi = Pervasive Displays Inc.(PDi) http://www.pervasivedisplays.com
 * - EPD = Electronic Paper Display (Electrophoretic Display)
 * - COG = Chip on Glass, the driver IC on EPD module
 * - COG G1 or G2: G is for generation.
 * - FPL = Front Plane Laminate which is E-Ink material film.
 *   There are Vizplex(V110, EOL already), Aurora Ma(V230) and Aurora Mb(V231) type
 * - PDi offers Aurora_Ma and Aurora_Mb material with G2 COG to the market.
 *   Some customers got our Aurora_Mb+G1 combination. The sample code is also provided.
 * - Basically, the Aurora_Mb+G1 is the replacement for Vizplex+G1.
 * - How to identify the FPL material type of your displays, please visit
 *   http://www.pervasivedisplays.com/products/label_info
 * - For driving PDi's small size EPDs, please read the "COG Driver Interface
 *   Timing" document(hereinafter COG Document) first. It explains the interface
 *   to the COG driver of EPD for a MCU based solution.
 * - COG Document no.: 4P008-00 (for Vizplex+G1)  : http://www.pervasivedisplays.com/LiteratureRetrieve.aspx?ID=138408
 * - COG Document no.: 4P015-00 (for Aurora_Ma+G2): http://www.pervasivedisplays.com/LiteratureRetrieve.aspx?ID=198794
 * - COG Document no.: 4P016-00 (for Aurora_Mb+G1): http://www.pervasivedisplays.com/LiteratureRetrieve.aspx?ID=220874
 * - COG Document no.: 4P018-00 (for Aurora_Mb+G2): http://www.pervasivedisplays.com/LiteratureRetrieve.aspx?ID=220873
 * - This project code supports EPD size: 1.44", 1.9", 2", 2.6" and 2.7"
 * - Supports Atmel Xplained PRO: SAM D21 Xplained PRO
 *
 * \section File_Explanation
 * - <b>image_data:</b>\n
 *   It defines the image arrays of each EPD size. 
 * - <b>conf_EPD.h:</b> (under [config] folder) The EPD configurations.\n
 *   -# USE_EPD_Type: define the demo size of EPD you are connecting
 *   -# Gx_Aurora_xx: define which FPL material with COG driving waveform of the EPD you're connecting
 *   -# COG_SPI_baudrate: SPI speed, G1 works in 4-12MHz, G2 works in 4-20MHz
 * - <b>Pervasive_Displays_small_EPD</b> folder:\n
 *   All of the COG driving waveforms are located in this folder. Logically developer
 *   doesn't need to change the codes in this folder in order to keep correct driving
 *   to the EPDs.\n\n
 *   <b><em>Software architecture:</em></b>\n
 *   [Application (ex. EPD Kit Tool)] <-- [COG interface (<em>EPD_interface</em>)] <--
 *   [COG driving process (<em>EPD_Gx_Aurora_Mx</em> in COG_FPL folder)] <--
 *   [Hardware Driver & GPIO (<em>EPD_hardware_driver</em>)]\n\n
 *    -# <b>EPD_hardware_driver:</b>\n
 *       Most of the COG hardware initialization, GPIO and configuration. User can implement
 *       the driver layer of EPD if some variables need to be adjusted. The provided
 *       settings and functions are Timer, SPI, PWM, temperature and EPD hardware initialization.
 *    -# <b>COG_FPL</b> folder:\n
 *       The driving process for each sub-folder represents the different display module.
 *       - <b>EPD_UpdateMethod_Gx_Aurora_Mx:</b>\n
 *         UpdateMethod: Global update or Partial update. If none, it means both.
 *         Gx: G1 or G2.
 *         Aurora_Mx: Aurora_Ma or Aurora_Mb.
 *    -# <b>EPD_interface:</b>\n
 *       The application interfaces to work with EPD.
 *
 *
 * \section Use_Case
 * -# <b>EPD_display_from_pointer</b>: Load two image data arrays from image_data.c
 *   according to predefined EPD size.
 * -# <b>EPD_display_from_flash</b>:
 *   Load stored image data from flash memory according for predefined EPD size. User
 *   must convert 1 bit bitmap image file to hex data in advance and store in flash
 *   memory. Image converting tool can be downloaded at http://www.pervasivedisplays.com/LiteratureRetrieve.aspx?ID=214756
 *
 * \section Steps
 * -# Ensure the EPD is connected correctly on the EPD Xplained Pro extension board
 * -# Connect the EPD Xplained Pro to the SAMD21 Xplained Pro header marked EXT1
 * -# Connect the SAM D21 Xplained Pro to computer's USB port via USB cable
 * -# Find #define Gx_Aurora_Mx in "conf_EPD.h" file. Change to the correct type of EPD you are connecting.
 * -# Change the USE_EPDXXX to the correct size.
 * -# Close the J2 jumper on board if you are connecting with 1.44" or 2"
 * -# Start debugging to program the project code to Atmel Kit. The EPD will show
 *    two images change alternately every 10 seconds (default).
 *
 *
 * * \section EPD Xplained Pro extension header pins
 * ================================================================
 * |Pin| Function       | Description                             |
 * |---|----------------|-----------------------------------------|
 * | 1 | ID             | Communication line to ID chip           |
 * | 2 | GND            | Ground                                  |
 * | 3 | Temperature    | On board temperature sensor output (ADC)|
 * | 4 | BORDER_CONTROL | Border control pin (GPIO)               |
 * | 5 | DISCHARGE      | EPD discharge when EPD power off (GPIO) |
 * | 6 | /RESET         | Reset signal. Low enable (GPIO)         |
 * | 7 | PWM            | Square wave when EPD power on (PWM)     |
 * | 8 | PANEL_ON       | COG driver power control pin (GPIO)     |
 * | 9 | BUSY           | COG busy pin (GPIO)                     |
 * |10 | FLASH_CS       | On board flash chip select (GPIO)       |
 * |11 |                |                                         |
 * |12 |                |                                         |
 * |13 |                |                                         |
 * |14 |                |                                         |
 * |15 | /EPD_CS        | Chip Select. Low enable (GPIO)          |
 * |16 | SPI_MOSI       | Serial input from host MCU to EPD       |
 * |17 | SPI_MISO       | Serial output from EPD to host MCU      |
 * |18 | SPI_CLK        | Clock for SPI                           |
 * |19 | GND            | Ground                                  |
 * |20 | VCC            | Target supply voltage                   |
 *
 *
 * \section PDi EPD displays
 * ======================================
 * | Size | PDi Model  |   FPL + COG    |
 * |------|------------|----------------|
 * | 1.44 | EK014BS011 | Aurora_Ma + G2 |
 * | 2.0  | EG020BS011 | Aurora_Ma + G2 |
 * | 2.7  | EM027BS013 | Aurora_Ma + G2 |
 * | 1.44 | EK014CS011 | Aurora_Mb + G1 |
 * | 1.9  | EB019CS011 | Aurora Mb + G1 |
 * | 2.0  | EG020CS012 | Aurora_Mb + G1 |
 * | 2.6  | EN026CS011 | Aurora Mb + G1 |
 * | 2.7  | EM027CS011 | Aurora_Mb + G1 |
 * | 1.44 | E1144CS021 | Aurora Mb + G2 |
 * | 1.9  | E1190CS021 | Aurora Mb + G2 |
 * | 2.0  | E1200CS021 | Aurora Mb + G2 |
 * | 2.6  | E1260CS021 | Aurora Mb + G2 |
 * | 2.7  | E1271CS021 | Aurora Mb + G2 |
 *
 * http://www.pervasivedisplays.com/products/label_info
 */