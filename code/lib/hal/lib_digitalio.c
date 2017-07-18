/* 
Copyright 2013 Brad Quick

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

#include "hal.h"
#include "lib_digitalio.h"

// This code controls the simple digital IO

// Usage:

// #define LEDOUTPUT (DIGITALPORTB | 7)     // defines the reference for pin 7 on PORTB
// lib_digitalio_initpin(LEDOUTPUT,DIGITALOUTPUT | PULLUPRESISTOR); // set the pin as an output (also turns on the pull up resistor)
// lib_digitalio_setoutput(LEDOUTPUT, DIGITALLOW); // turn the output on by pulling it low

// #define PUSHBUTTON (DIGITALPORTB | 4)     // defines the reference for pin 4 on PORTB
// #define INTERRUPT6PORTANDPIN PUSHBUTTON
// lib_digitalio_initpin(PUSHBUTTON,DIGITALINPUT);  // set the pin as an input (also turns on the pull up resistor)
// if (lib_digitalio_getinput(PUSHBUTTON)) {}       // Check the input
// unimplemented below:
// lib_digitalio_setinterruptcallback(PUSHBUTTON,mypushbuttoncallback); // tell the interrupt
// void mypushbuttoncallback(char interruptnumber,char newstate) // call back will get called any time the pin changes
//      {
//      if (newstate==DIGITALON) {}
//      }


static GPIO_TypeDef *lib_digitalio_getport(unsigned char pinnumber)
{
    unsigned char port = pinnumber & 0xf0;

    switch (port) {
        case DIGITALPORTA:
            return GPIOA;
        case DIGITALPORTB:
            return GPIOB;
        case DIGITALPORTC:
            return GPIOC;
        default:
            return NULL;
    }
}

void lib_digitalio_initpin(unsigned char pinnumber, unsigned char output)
{ 
    // set pin pinnumber to be an output if output | DIGITALOUTPUT, othewise set it to be an input
    GPIO_TypeDef *gpio = lib_digitalio_getport(pinnumber);
    gpio_config_t cfg;

    pinnumber &= 0x0f;

    cfg.pin = 1 << pinnumber;
    cfg.speed = Speed_2MHz;
    if (output & DIGITALOUTPUT)
        cfg.mode = Mode_Out_PP;
    else
        cfg.mode = Mode_IN_FLOATING;
    gpioInit(gpio, &cfg);
}

unsigned char lib_digitalio_getinput(unsigned char pinnumber)
{
    GPIO_TypeDef *gpio = lib_digitalio_getport(pinnumber);
    pinnumber &= 0x0f;

    return (gpio->IDR & (1 << pinnumber)) != 0;
}

void lib_digitalio_setoutput(unsigned char pinnumber, unsigned char value)
{
    GPIO_TypeDef *gpio = lib_digitalio_getport(pinnumber);
    pinnumber &= 0x0f;

    if (value)
        gpio->BRR = (1 << pinnumber);
    else
        gpio->BSRR = (1 << pinnumber);
}

void lib_digitalio_setinterruptcallback(unsigned char pinnumber, digitalcallbackfunctptr callback)
{
    // Not implemented, no need on real hardware...
}
