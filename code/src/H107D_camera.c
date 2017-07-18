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
    
    // Init port 
    lib_digitalio_initpin(PIN_H107D_CAMERA_SDIO, DIGITALOUTPUT);
    lib_digitalio_initpin(PIN_H107D_CAMERA_SCK, DIGITALOUTPUT);
    lib_digitalio_initpin(PIN_H107D_CAMERA_SCS, DIGITALOUTPUT);
    lib_digitalio_setoutput(PIN_H107D_CAMERA_SCS, DIGITALON);
    lib_digitalio_setoutput(PIN_H107D_CAMERA_SDIO, DIGITALOFF);
    lib_digitalio_setoutput(PIN_H107D_CAMERA_SCK, DIGITALOFF);
    
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
    uint8_t Tosend = 0x88;
    uint8_t i;
    // Update frequency only if changed
    if (global.camera_frequency != newfrequency)
    {
        // Chip select
        lib_digitalio_setoutput(PIN_H107D_CAMERA_SCS, DIGITALOFF);
        
        // Init SDIO & CLK
        lib_digitalio_setoutput(PIN_H107D_CAMERA_SCK, DIGITALOFF);
        lib_digitalio_setoutput(PIN_H107D_CAMERA_SDIO, DIGITALOFF);
        
        // Send fixed part
        Tosend = 0x88;
        for (i=0;i<6;i++) {
            if(Tosend&0x80) // MSB first
                lib_digitalio_setoutput(PIN_H107D_CAMERA_SDIO, DIGITALON);
            else 
                lib_digitalio_setoutput(PIN_H107D_CAMERA_SDIO, DIGITALOFF);
            lib_digitalio_setoutput(PIN_H107D_CAMERA_SCK, DIGITALON);
            lib_digitalio_setoutput(PIN_H107D_CAMERA_SCK, DIGITALOFF);
            Tosend = Tosend << 1;
        }
        
        // Send first part 
        // REVERSE((21 - 3 * (Freq - 5725) / 5 ) % 128)
        Tosend = (21 - 3 * (newfrequency - 5725) / 5) % 128;
        for (i=0;i<7;i++) {
            if(Tosend&0x01) // LSB first (Reverse)
                lib_digitalio_setoutput(PIN_H107D_CAMERA_SDIO, DIGITALON);
            else 
                lib_digitalio_setoutput(PIN_H107D_CAMERA_SDIO, DIGITALOFF);
            lib_digitalio_setoutput(PIN_H107D_CAMERA_SCK, DIGITALON);
            lib_digitalio_setoutput(PIN_H107D_CAMERA_SCK, DIGITALOFF);
            Tosend = Tosend >> 1;
        }
        
        // Send second part 
        // If Freq < 5765 REVERSE( 60 + ((Freq-5725) / 5) * 2 + 1 - Freq % 2 )
        // If Freq >= 5765 REVERSE( 60 + ((Freq-5725) / 5 - 1) * 2 + Freq % 2 )
        if (newfrequency < 5765) {
            Tosend = 60 + ((newfrequency-5725) / 5) * 2 + 1 - newfrequency % 2;
        } else {
            Tosend = 60 + ((newfrequency-5725) / 5 - 1) * 2 + newfrequency % 2;
        }
        for (i=0;i<6;i++) {
            if(Tosend&0x01) // LSB first (Reverse)
                lib_digitalio_setoutput(PIN_H107D_CAMERA_SDIO, DIGITALON);
            else 
                lib_digitalio_setoutput(PIN_H107D_CAMERA_SDIO, DIGITALOFF);
            lib_digitalio_setoutput(PIN_H107D_CAMERA_SCK, DIGITALON);
            lib_digitalio_setoutput(PIN_H107D_CAMERA_SCK, DIGITALOFF);
            Tosend = Tosend >> 1;
        }
        
        // Send fixed part
        Tosend = 0x88;
        for (i=0;i<6;i++) {
            if(Tosend&0x80) // MSB first
                lib_digitalio_setoutput(PIN_H107D_CAMERA_SDIO, DIGITALON);
            else 
                lib_digitalio_setoutput(PIN_H107D_CAMERA_SDIO, DIGITALOFF);
            lib_digitalio_setoutput(PIN_H107D_CAMERA_SCK, DIGITALON);
            lib_digitalio_setoutput(PIN_H107D_CAMERA_SCK, DIGITALOFF);
            Tosend = Tosend << 1;
        }
        
        // Return in nominbal state
        lib_digitalio_setoutput(PIN_H107D_CAMERA_SCK, DIGITALOFF);
        lib_digitalio_setoutput(PIN_H107D_CAMERA_SDIO, DIGITALOFF);
        lib_digitalio_setoutput(PIN_H107D_CAMERA_SCS, DIGITALON);
        
        global.camera_frequency = newfrequency;
    }
}
