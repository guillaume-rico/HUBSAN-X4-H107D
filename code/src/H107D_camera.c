/* 
Copyright 2016 Guillaume RICO

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "bradwii.h"
#include "H107D_camera.h"
#include "lib_digitalio.h"

extern globalstruct global;
  
#define PIN_H107D_CAMERA_SDIO (DIGITALPORT0 | 5)
#define PIN_H107D_CAMERA_SCK  (DIGITALPORT4 | 6)
#define PIN_H107D_CAMERA_SCS  (DIGITALPORT4 | 7)

/**
 * @brief      Init communication with H107D Camera
 * @param      None.
 * @return     None
 */
void H107D_camera_init(void) 
{
	// Initialize camera frequency
	global.camera_frequency = 0;
	
	// Start sequence
	uint32_t TSdata   = 0b00001000010011000000000000000000;
	uint8_t n=25;

	// Init port 
	SYS->P4_MFP = 0x00000000UL;
	lib_digitalio_initpin(PIN_H107D_CAMERA_SDIO, DIGITALOUTPUT);
	lib_digitalio_initpin(PIN_H107D_CAMERA_SCK, DIGITALOUTPUT);
	lib_digitalio_initpin(PIN_H107D_CAMERA_SCS, DIGITALOUTPUT);
	lib_digitalio_setoutput(PIN_H107D_CAMERA_SCS, DIGITALON);
	lib_digitalio_setoutput(PIN_H107D_CAMERA_SDIO, DIGITALOFF);
	lib_digitalio_setoutput(PIN_H107D_CAMERA_SCK, DIGITALOFF);

	// Send initialisation frame
	lib_digitalio_setoutput(PIN_H107D_CAMERA_SCS, DIGITALOFF);
	while(n--) {
		if(TSdata&0x80000000UL) // MSB first
			lib_digitalio_setoutput(PIN_H107D_CAMERA_SDIO, DIGITALON);
		else 
			lib_digitalio_setoutput(PIN_H107D_CAMERA_SDIO, DIGITALOFF);
		lib_digitalio_setoutput(PIN_H107D_CAMERA_SCK, DIGITALON);
		CLK_SysTickDelay(3);
		lib_digitalio_setoutput(PIN_H107D_CAMERA_SCK, DIGITALOFF);
		CLK_SysTickDelay(1);
		TSdata = TSdata << 1;
	}
	lib_digitalio_setoutput(PIN_H107D_CAMERA_SCS, DIGITALON);
}


/**
 * @brief               Init communication with H107D Camera
 * @param[newfrequency] Frequency to apply.
 * @return              None
 * @note                
    TRAME[0-5] - 6 bits - 0b100010
    TRAME[6-12] - 7 bits - REVERSE((21 - 3 * (Freq - 5725) / 5 ) % 128)
    TRAME[13-18] - 6 bits -
        If Freq < 5765 REVERSE( 60 + ((Freq-5725) / 5) * 2 + 1 - Freq % 2 )
        If Freq >= 5765 REVERSE( 60 + ((Freq-5725) / 5 - 1) * 2 + Freq % 2 )
    TRAME[19-24] - 6 bits - 0b100010
 */
void H107D_camera_update_frequency(uint16_t newfrequency) 
{
    //uint8_t Tosend = 0x88;
    //uint8_t i;
	  uint8_t n=25;
	
		//uint32_t TSdata = 0b10001010101000111101000100000000;
		// case 5725 MHz
	  // Default value used for initialisation
	  uint32_t TSdata   = 0b00001000010011000000000000000000;
	
    if (global.camera_frequency != newfrequency)
    {
			global.camera_frequency = newfrequency;
			// Case 0 is used 
			switch(newfrequency) { 
				case 5725  : TSdata =  0b10001010101000111101000100000000;break;
				case 5730  : TSdata =  0b10001001001001111101000100000000;break;
				case 5735  : TSdata =  0b10001011110000000011000100000000;break;
				case 5740  : TSdata =  0b10001000110001000011000100000000;break;
				case 5745  : TSdata =  0b10001010010000100011000100000000;break;
				case 5750  : TSdata =  0b10001001100001100011000100000000;break;
				case 5755  : TSdata =  0b10001011000000010011000100000000;break;
				case 5760  : TSdata =  0b10001000000001010011000100000000;break;
				case 5765  : TSdata =  0b10001010111111010011000100000000;break;
				case 5770  : TSdata =  0b10001001011110110011000100000000;break;
				case 5775  : TSdata =  0b10001011101111110011000100000000;break;
				case 5780  : TSdata =  0b10001000101110001011000100000000;break;
				case 5785  : TSdata =  0b10001010001111001011000100000000;break;
				case 5790  : TSdata =  0b10001001110110101011000100000000;break;
				case 5795  : TSdata =  0b10001011010111101011000100000000;break;
				case 5800  : TSdata =  0b10001000010110011011000100000000;break;
				case 5805  : TSdata =  0b10001010100111011011000100000000;break;
				case 5810  : TSdata =  0b10001001000110111011000100000000;break;
				case 5815  : TSdata =  0b10001011111011111011000100000000;break;
				case 5820  : TSdata =  0b10001000111010000111000100000000;break;
				case 5825  : TSdata =  0b10001010011011000111000100000000;break;
				case 5830  : TSdata =  0b10001001101010100111000100000000;break;
				case 5835  : TSdata =  0b10001011001011100111000100000000;break;
				case 5840  : TSdata =  0b10001000001010010111000100000000;break;
				case 5845  : TSdata =  0b10001010110011010111000100000000;break;
				case 5850  : TSdata =  0b10001001010010110111000100000000;break;
				case 5855  : TSdata =  0b10001011100011110111000100000000;break;
				case 5860  : TSdata =  0b10001000100010001111000100000000;break;
				case 5865  : TSdata =  0b10001010000011001111000100000000;break;
				case 5870  : TSdata =  0b10001001111100101111000100000000;break;
				case 5875  : TSdata =  0b10001011011101101111000100000000;break;
			}

			// Init SDIO & CLK
			lib_digitalio_setoutput(PIN_H107D_CAMERA_SCS, DIGITALOFF);
			while(n--) {
					if(TSdata&0x80000000UL) // MSB first
							lib_digitalio_setoutput(PIN_H107D_CAMERA_SDIO, DIGITALON);
					else 
							lib_digitalio_setoutput(PIN_H107D_CAMERA_SDIO, DIGITALOFF);
					lib_digitalio_setoutput(PIN_H107D_CAMERA_SCK, DIGITALON);
					CLK_SysTickDelay(3);
					lib_digitalio_setoutput(PIN_H107D_CAMERA_SCK, DIGITALOFF);
					CLK_SysTickDelay(1);
					TSdata = TSdata << 1;
			}
			lib_digitalio_setoutput(PIN_H107D_CAMERA_SCS, DIGITALON);
		}
}
