/* 
Copyright 2013 Brad Quick

Some of this code is based on Multiwii code by Alexandre Dubus (www.multiwii.com)

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

/*

The code is for controlling multi-copters.  Many of the ideas in the code come from the Multi-Wii project
(see multiwii.com).  This project doesn't contain all of the features in Multi-Wii, but I believe it incorporates
a number of improvements.

In order to make the code run quickly on 8 bit processors, much of the math is done using fixed point numbers
instead of floating point.  Much pain was taken to write almost the entire code without performing any
division, which is slow. As a result, main loop cycles can take well under 2 milliseconds.

A second advantage is that I believe that this code is more logically layed out and better commented than 
some other multi-copter code.  It is designed to be easy to follow for the guy who wants to understand better how
the code works and maybe wants to modify it.

In general, I didn't include code that I haven't tested myself, therefore many aircraft configurations, control boards,
sensors, etc. aren't yet included in the code.  It should be fairly easy, however for interested coders to add the
components that they need.

If you find the code useful, I'd love to hear from you.  Email me at the address that's shown vertically below:

b         I made my
r         email address
a         vertical so
d         the spam bots
@         won't figure
j         it out.
a         - Thanks.
m
e
s
l
t
a
y
l
o
r
.
c
o
m

*/

// library headers
#include "hal.h"
#include "lib_timers.h"
#include "lib_serial.h"
#include "lib_i2c.h"
#include "lib_digitalio.h"
#include "lib_fp.h"
#if CONTROL_BOARD_TYPE == CONTROL_BOARD_HUBSAN_H107L || CONTROL_BOARD_TYPE == CONTROL_BOARD_HUBSAN_H107D 
#include "lib_adc.h"
#endif

// project file headers
#include "bradwii.h"
#include "rx.h"
#include "serial.h"
#include "output.h"
#include "gyro.h"
#include "accelerometer.h"
#include "imu.h"
#include "baro.h"
#include "compass.h"
#include "eeprom.h"
#include "gps.h"
#include "navigation.h"
#include "pilotcontrol.h"
#include "autotune.h"
#if CONTROL_BOARD_TYPE == CONTROL_BOARD_HUBSAN_H107D 
#include "H107D_camera.h"
#endif

// Data type for stick movement detection to execute accelerometer calibration
typedef enum stickstate_tag {
    STICK_STATE_START,   // No stick movement detected yet
    STICK_STATE_LOW,     // Stick was low recently
    STICK_STATE_HIGH     // Stick was high recently
} stickstate_t;

globalstruct global;            // global variables
usersettingsstruct usersettings;        // user editable variables

fixedpointnum altitudeholddesiredaltitude;
fixedpointnum integratedaltitudeerror;  // for pid control

fixedpointnum integratedangleerror[3];
fixedpointnum filteredgyrorate[3];

// limit pid windup
#define INTEGRATEDANGLEERRORLIMIT FIXEDPOINTCONSTANT(1000)

#if CONTROL_BOARD_TYPE == CONTROL_BOARD_HUBSAN_H107L || CONTROL_BOARD_TYPE == CONTROL_BOARD_HUBSAN_H107D 
// Factor from ADC input voltage to battery voltage
#define FP_BATTERY_VOLTAGE_FACTOR FIXEDPOINTCONSTANT(BATTERY_VOLTAGE_FACTOR)

// If battery voltage gets below this value the LEDs will blink
#define FP_BATTERY_UNDERVOLTAGE_LIMIT FIXEDPOINTCONSTANT(BATTERY_UNDERVOLTAGE_LIMIT)
#endif

// Stick is moved out of middle position towards low
#define FP_RXMOVELOW FIXEDPOINTCONSTANT(-0.2)
// Stick is moved out of middle position towards high
#define FP_RXMOVEHIGH FIXEDPOINTCONSTANT(0.2)

// timesliver is a very small slice of time (.002 seconds or so).  This small value doesn't take much advantage
// of the resolution of fixedpointnum, so we shift timesliver an extra TIMESLIVEREXTRASHIFT bits.
unsigned long timeslivertimer = 0;

// Local functions
static void detectstickcommand(void);


// It all starts here:
int main(void)
{
#if CONTROL_BOARD_TYPE == CONTROL_BOARD_HUBSAN_H107L || CONTROL_BOARD_TYPE == CONTROL_BOARD_HUBSAN_H107D 
    // Static to keep it off the stack
    static bool isbatterylow;         // Set to true while voltage is below limit
    static bool isadcchannelref;      // Set to true if the next ADC result is reference channel
    // Current unfiltered battery voltage [V]. Filtered value is in global.batteryvoltage
    static fixedpointnum batteryvoltage;
    // Current raw battery voltage.
    static fixedpointnum batteryvoltageraw;
    // Current raw bandgap reference voltage.
    static fixedpointnum bandgapvoltageraw;
    // Initial bandgap voltage [V]. We measure this once when there is no load on the battery
    // because the specified tolerance for this is pretty high.
    static fixedpointnum initialbandgapvoltage;
	  uint8_t nbFlash;
		global.started = 0;
#endif
    static bool isfailsafeactive;     // true while we don't get new data from transmitter

    // initialize hardware
	  lib_hal_init();

    // start with default user settings in case there's nothing in eeprom
    defaultusersettings();
    
    // try to load usersettings from eeprom
    readusersettingsfromeeprom();

    // set our LED as a digital output
    lib_digitalio_initpin(LED1_OUTPUT, DIGITALOUTPUT);

    //initialize the libraries that require initialization
    lib_timers_init();
    lib_i2c_init();

#if CONTROL_BOARD_TYPE == CONTROL_BOARD_HUBSAN_H107L || CONTROL_BOARD_TYPE == CONTROL_BOARD_HUBSAN_H107D 
    // extras init for Hubsan X4
    x4_init_leds();
    if(!global.usersettingsfromeeprom) {
        // If nothing found in EEPROM (= data flash on Mini51)
        // use default X4 settings.
        x4_set_usersettings();
        // Indicate that default settings are used and accelerometer
        // calibration will be executed (4 long LED blinks)
        for(uint8_t i=0;i<4;i++) {
            x4_set_leds(X4_LED_ALL);
            lib_timers_delaymilliseconds(100);
            x4_set_leds(X4_LED_NONE);
            lib_timers_delaymilliseconds(100);
        }
    }
#endif
	
    // pause a moment before initializing everything. To make sure everything is powered up
    lib_timers_delaymilliseconds(100); 
		
    // initialize all other modules
    initrx();
    // Give the battery voltage lowpass filter a reasonable starting point.
    global.batteryvoltage = FP_BATTERY_UNDERVOLTAGE_LIMIT;
    lib_adc_init();  // For battery voltage	
    initoutputs();
    initgyro();
    initacc();
    initimu();

#if (CONTROL_BOARD_TYPE == CONTROL_BOARD_HUBSAN_H107L || CONTROL_BOARD_TYPE == CONTROL_BOARD_HUBSAN_H107D )
    x4_set_leds(X4_LED_ALL);
    // Measure internal bandgap voltage now.
    // Battery is probably full and there is no load,
    // so we can expect to have a good external ADC reference
    // voltage now.
    lib_adc_select_channel(LIB_ADC_CHANREF);
    initialbandgapvoltage = 0;
    // Take average of 8 measurements
    for(int i=0;i<8;i++) {
        lib_adc_startconv();
        while(lib_adc_is_busy())
            ;
        initialbandgapvoltage += lib_adc_read_volt();
    }
    initialbandgapvoltage >>= 3;
    bandgapvoltageraw = lib_adc_read_raw();
    // Start first battery voltage measurement
    isadcchannelref = false;
    lib_adc_select_channel(LIB_ADC_CHAN5);
    lib_adc_startconv();
#endif

    // set the default i2c speed to 400 kHz.  If a device needs to slow it down, it can, but it should set it back.
    lib_i2c_setclockspeed(I2C_400_KHZ);

    global.armed = 0;
    global.navigationmode = NAVIGATIONMODEOFF;
    global.failsafetimer = lib_timers_starttimer();
    for (;;) {

        // check to see what switches are activated
        checkcheckboxitems();

#if (MULTIWII_CONFIG_SERIAL_PORTS != NOSERIALPORT)
        // check for config program activity
        serialcheckforaction();
#endif
        calculatetimesliver();

        // run the imu to estimate the current attitude of the aircraft
        imucalculateestimatedattitude();

				if (!global.armed) {
					
					// Throttle low and yaw left
					if (global.rxvalues[THROTTLEINDEX] < FPSTICKLOW && global.rxvalues[YAWINDEX] > FPSTICKX4HIGH ) {
						
						// Default : Level mode 
						global.activecheckboxitems &= ~CHECKBOXMASKFULLACRO;
						global.activecheckboxitems &= ~CHECKBOXMASKSEMIACRO;
						
						// pitch PIDs
						usersettings.pid_pgain[PITCHINDEX] = 300L; // 35L << 3;
						usersettings.pid_igain[PITCHINDEX] = 32L; //4L << 3; 32L
						usersettings.pid_dgain[PITCHINDEX] = 90L; //22L << 2; 88L

						// roll PIDs
						usersettings.pid_pgain[ROLLINDEX] = 300L; //35L << 3;
						usersettings.pid_igain[ROLLINDEX] = 32L; //4L << 3;
						usersettings.pid_dgain[ROLLINDEX] = 90L; //22L << 2;

						// yaw PIDs
						usersettings.pid_pgain[YAWINDEX] = 300L; //100L << 4;
						usersettings.pid_igain[YAWINDEX] = 0L; //0L;
						usersettings.pid_dgain[YAWINDEX] = 90L; //Was 90L 22L << 2;
						
						if (global.rxvalues[ROLLINDEX] < FPSTICKX4LOW) {
							// Pith High (Up) : Accro
							global.activecheckboxitems |= CHECKBOXMASKFULLACRO;
							global.flymode = ACCROFLIGHTMODE;
							/*
							// pitch PIDs
							// 35 << 3 : 280 , 
							usersettings.pid_pgain[PITCHINDEX] = 35L << 3;
							usersettings.pid_igain[PITCHINDEX] = 4L;
							usersettings.pid_dgain[PITCHINDEX] = 22L << 2;

							// roll PIDs
							usersettings.pid_pgain[ROLLINDEX] = 35L << 3;
							usersettings.pid_igain[ROLLINDEX] = 4L;
							usersettings.pid_dgain[ROLLINDEX] = 22L << 2;

							// yaw PIDs
							usersettings.pid_pgain[YAWINDEX] = 100L << 4;
							usersettings.pid_igain[YAWINDEX] = 0L;
							usersettings.pid_dgain[YAWINDEX] = 22L << 3;
							*/
							nbFlash = 3;
						} else if (global.rxvalues[ROLLINDEX] > FPSTICKX4HIGH) {
							// Pitch Low (Down) : Semi Accro
							global.activecheckboxitems |= CHECKBOXMASKSEMIACRO;
							global.flymode = SEMIACCROFLIGHTMODE;
							/*
							// pitch PIDs
							usersettings.pid_pgain[PITCHINDEX] = 35L << 3;
							usersettings.pid_igain[PITCHINDEX] = 4L;
							usersettings.pid_dgain[PITCHINDEX] = 22L << 2;

							// roll PIDs
							usersettings.pid_pgain[ROLLINDEX] = 35L << 3;
							usersettings.pid_igain[ROLLINDEX] = 4L;
							usersettings.pid_dgain[ROLLINDEX] = 22L << 2;

							// yaw PIDs
							usersettings.pid_pgain[YAWINDEX] = 100L << 4;
							usersettings.pid_igain[YAWINDEX] = 0L;
							usersettings.pid_dgain[YAWINDEX] = 22L << 3;
							*/
							nbFlash = 2;
						} else {
							// Level mode
							global.flymode = LEVELFLIGHTMODE;
							
							// pitch PIDs
							usersettings.pid_pgain[PITCHINDEX] = 200L; // 35L << 3;
							usersettings.pid_igain[PITCHINDEX] = 64L; //4L << 3; 32L
							usersettings.pid_dgain[PITCHINDEX] = 100L; //22L << 2; 88L

							// roll PIDs 120 4 90
							usersettings.pid_pgain[ROLLINDEX] = 180L; //35L << 3;
							usersettings.pid_igain[ROLLINDEX] = 64L; //4L << 3;
							usersettings.pid_dgain[ROLLINDEX] = 90L; //22L << 2;

							// yaw PIDs
							usersettings.pid_pgain[YAWINDEX] = 300L; //100L << 4;300
							usersettings.pid_igain[YAWINDEX] = 0L; //0L;
							usersettings.pid_dgain[YAWINDEX] = 15L; //Was 90L 22L << 2;
							
							nbFlash = 1;
						}
						
						// Flash all leds :
						// 1 time for level mode
						// 2 times for semi accro
						// 3 times for accro
						/*
						for(uint8_t i=0;i<nbFlash;i++) {
								x4_set_leds(X4_LED_ALL);
								lib_timers_delaymilliseconds(500);
								x4_set_leds(X4_LED_NONE);
								lib_timers_delaymilliseconds(500);
						}
						*/
						global.started = 0;
						global.armed = 1;
						calibrategyroandaccelerometer(true);
					}
				} else {
					if (global.rxvalues[THROTTLEINDEX] < FPSTICKLOW && global.rxvalues[YAWINDEX] < FPSTICKX4LOW) {
						global.armed = 0;
					} else if (global.rxvalues[THROTTLEINDEX] > FPSTICKLOW) {
						global.started = 1;
					}
				}

        // read the receiver
        readrx();

        // get the angle error.  Angle error is the difference between our current attitude and our desired attitude.
        // It can be set by navigation, or by the pilot, etc.
        fixedpointnum angleerror[3];

        // let the pilot control the aircraft.
        getangleerrorfrompilotinput(angleerror);

        if (global.rxvalues[THROTTLEINDEX] < FPSTICKLOW) {
            // We are probably on the ground. Don't accumnulate error when we can't correct it
            resetpilotcontrol();

            // bleed off integrated error by averaging in a value of zero
            lib_fp_lowpassfilter(&integratedangleerror[ROLLINDEX], 0L, global.timesliver >> TIMESLIVEREXTRASHIFT, FIXEDPOINTONEOVERONEFOURTH, 0);
            lib_fp_lowpassfilter(&integratedangleerror[PITCHINDEX], 0L, global.timesliver >> TIMESLIVEREXTRASHIFT, FIXEDPOINTONEOVERONEFOURTH, 0);
            lib_fp_lowpassfilter(&integratedangleerror[YAWINDEX], 0L, global.timesliver >> TIMESLIVEREXTRASHIFT, FIXEDPOINTONEOVERONEFOURTH, 0);
						filteredgyrorate[ROLLINDEX] = FIXEDPOINTCONSTANT(0L);
						filteredgyrorate[PITCHINDEX] = FIXEDPOINTCONSTANT(0L);
						filteredgyrorate[YAWINDEX] = FIXEDPOINTCONSTANT(0L);
					
				}

        // get the pilot's throttle component
        // convert from fixedpoint -1 to 1 to fixedpoint 0 to 1
        fixedpointnum throttleoutput = (global.rxvalues[THROTTLEINDEX] >> 1) + FIXEDPOINTONEOVERTWO + FPTHROTTLETOMOTOROFFSET;

        // calculate output values.  Output values will range from 0 to 1.0

        // calculate pid outputs based on our angleerrors as inputs
        fixedpointnum pidoutput[3];

        // Gain Scheduling essentialy modifies the gains depending on
        // throttle level. If GAIN_SCHEDULING_FACTOR is 1.0, it multiplies PID outputs by 1.5 when at full throttle,
        // 1.0 when at mid throttle, and .5 when at zero throttle.  This helps
        // eliminate the wobbles when decending at low throttle.
        fixedpointnum gainschedulingmultiplier = lib_fp_multiply(throttleoutput - FIXEDPOINTCONSTANT(.5), FIXEDPOINTCONSTANT(GAIN_SCHEDULING_FACTOR)) + FIXEDPOINTONE;

        for (int x = 0; x < 3; ++x) {
            integratedangleerror[x] += lib_fp_multiply(angleerror[x], global.timesliver);
						//filteredgyrorate[x] = (lib_fp_multiply(global.gyrorate[x], FIXEDPOINTCONSTANT(1)) + lib_fp_multiply(filteredgyrorate[x], FIXEDPOINTCONSTANT(1))) >> 1;
						//lib_fp_lowpassfilter(&filteredgyrorate[x], global.gyrorate[x], global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);

					
            // don't let the integrated error get too high (windup)
            lib_fp_constrain(&integratedangleerror[x], -INTEGRATEDANGLEERRORLIMIT, INTEGRATEDANGLEERRORLIMIT);

            // do the attitude pid
            pidoutput[x] = lib_fp_multiply(angleerror[x], usersettings.pid_pgain[x])
                         - lib_fp_multiply(global.gyrorate[x], usersettings.pid_dgain[x])
                         + (lib_fp_multiply(integratedangleerror[x], usersettings.pid_igain[x]) >> 4);

            // add gain scheduling.  
            pidoutput[x] = lib_fp_multiply(gainschedulingmultiplier, pidoutput[x]);
        }

				// On Hubsan X4 H107L the front right motor
				// rotates clockwise (viewed from top).
				// On the J385 the motors spin in the opposite direction.
				// PID output for yaw has to be reversed
        pidoutput[YAWINDEX] = -pidoutput[YAWINDEX];

        lib_fp_constrain(&throttleoutput, 0, FIXEDPOINTONE);

        // set the final motor outputs
        // if we aren't armed, or if we desire to have the motors stop, AfterFPSTICKLOW  && global.flymode == LEVELFLIGHTMODE
        if (!global.armed || (global.rxvalues[THROTTLEINDEX] < FPSTICKLOW ) || global.started == 0)
            setallmotoroutputs(MIN_MOTOR_OUTPUT);
        else {
            // mix the outputs to create motor values
            setmotoroutput(0, 0, throttleoutput - pidoutput[ROLLINDEX] + pidoutput[PITCHINDEX] - pidoutput[YAWINDEX]);
            setmotoroutput(1, 1, throttleoutput - pidoutput[ROLLINDEX] - pidoutput[PITCHINDEX] + pidoutput[YAWINDEX]);
            setmotoroutput(2, 2, throttleoutput + pidoutput[ROLLINDEX] + pidoutput[PITCHINDEX] + pidoutput[YAWINDEX]);
            setmotoroutput(3, 3, throttleoutput + pidoutput[ROLLINDEX] - pidoutput[PITCHINDEX] - pidoutput[YAWINDEX]);
        }

#if (CONTROL_BOARD_TYPE == CONTROL_BOARD_HUBSAN_H107L || CONTROL_BOARD_TYPE == CONTROL_BOARD_HUBSAN_H107D )
        // Measure battery voltage
        if(!lib_adc_is_busy())
        {
            // What did we just measure?
            // Always alternate between reference channel
            // and battery voltage
            if(isadcchannelref) {
                bandgapvoltageraw = lib_adc_read_raw();
                isadcchannelref = false;
                lib_adc_select_channel(LIB_ADC_CHAN5);
            } else {
                batteryvoltageraw = lib_adc_read_raw();
                isadcchannelref = true;
                lib_adc_select_channel(LIB_ADC_CHANREF);

                // Unfortunately we have to use fixed point division now
                batteryvoltage = (batteryvoltageraw << 12) / (bandgapvoltageraw >> (FIXEDPOINTSHIFT-12));
                // Now we have battery voltage relative to bandgap reference voltage.
                // Multiply by initially measured bandgap voltage to get the voltage at the ADC pin.
                batteryvoltage = lib_fp_multiply(batteryvoltage, initialbandgapvoltage);
                // Now take the voltage divider into account to get battery voltage.
                batteryvoltage = lib_fp_multiply(batteryvoltage, FP_BATTERY_VOLTAGE_FACTOR);

                // Since we measure under load, the voltage is not stable.
                // Apply 0.5 second lowpass filter.
                // Use constant FIXEDPOINTONEOVERONEFOURTH instead of FIXEDPOINTONEOVERONEHALF
                // Because we call this only every other iteration.
                // (...alternatively multiply global.timesliver by two).
                lib_fp_lowpassfilter(&(global.batteryvoltage), batteryvoltage, global.timesliver, FIXEDPOINTONEOVERONEFOURTH, TIMESLIVEREXTRASHIFT);
                // Update state of isbatterylow flag.
                if(global.batteryvoltage < FP_BATTERY_UNDERVOLTAGE_LIMIT)
                    isbatterylow = true;
                else
                    isbatterylow = false;
            }
            // Start next conversion
            lib_adc_startconv();
        } // IF ADC result available

        // Decide what LEDs have to show
        if(isbatterylow) {
            // Highest priority: Battery voltage
            // Blink all LEDs slow
            if(lib_timers_gettimermicroseconds(0) % 500000 > 250000)
                x4_set_leds(X4_LED_ALL);
            else
                x4_set_leds(X4_LED_NONE);
        }
        else if(isfailsafeactive) {
            // Lost contact with TX
            // Blink LEDs fast alternating
            if(lib_timers_gettimermicroseconds(0) % 250000 > 120000)
                x4_set_leds(X4_LED_FR | X4_LED_RL);
            else
                x4_set_leds(X4_LED_FL | X4_LED_RR);
        }
        else if(!global.armed) {
            // Not armed
            // Short blinks
            if(lib_timers_gettimermicroseconds(0) % 500000 > 450000)
                x4_set_leds(X4_LED_ALL);
            else
                x4_set_leds(X4_LED_NONE);
        }
        else {
            // LEDs stay on
            x4_set_leds(X4_LED_ALL);
        }

#endif
    } // Endless loop
} // main()

void calculatetimesliver(void)
{
    // load global.timesliver with the amount of time that has passed since we last went through this loop
    // convert from microseconds to fixedpointnum seconds shifted by TIMESLIVEREXTRASHIFT
    // 4295L is (FIXEDPOINTONE<<FIXEDPOINTSHIFT)*.000001
    global.timesliver = (lib_timers_gettimermicrosecondsandreset(&timeslivertimer) * 4295L) >> (FIXEDPOINTSHIFT - TIMESLIVEREXTRASHIFT);

    // don't allow big jumps in time because of something slowing the update loop down (should never happen anyway)
    if (global.timesliver > (FIXEDPOINTONEFIFTIETH << TIMESLIVEREXTRASHIFT))
        global.timesliver = FIXEDPOINTONEFIFTIETH << TIMESLIVEREXTRASHIFT;
}

void defaultusersettings(void)
{
    global.usersettingsfromeeprom = 0;  // this should get set to one if we read from eeprom

    // set default acro mode rotation rates
    usersettings.maxyawrate = 400L << FIXEDPOINTSHIFT;  // degrees per second
    usersettings.maxpitchandrollrate = 400L << FIXEDPOINTSHIFT; // degrees per second

    // set default PID settings
    for (int x = 0; x < 3; ++x) {
        usersettings.pid_pgain[x] = 15L << 3;   // 1.5 on configurator
        usersettings.pid_igain[x] = 8L; // .008 on configurator
        usersettings.pid_dgain[x] = 8L << 2;    // 8 on configurator
    }

    usersettings.pid_pgain[YAWINDEX] = 30L << 3;        // 3 on configurator

    for (int x = 3; x < NUMPIDITEMS; ++x) {
        usersettings.pid_pgain[x] = 0;
        usersettings.pid_igain[x] = 0;
        usersettings.pid_dgain[x] = 0;
    }

    usersettings.pid_pgain[ALTITUDEINDEX] = 27L << 7;   // 2.7 on configurator
    usersettings.pid_dgain[ALTITUDEINDEX] = 6L << 9;    // 6 on configurator

    usersettings.pid_pgain[NAVIGATIONINDEX] = 25L << 11;        // 2.5 on configurator
    usersettings.pid_dgain[NAVIGATIONINDEX] = 188L << 8;        // .188 on configurator

    // set default configuration checkbox settings.
    for (int x = 0; x < NUMPOSSIBLECHECKBOXES; ++x) {
        usersettings.checkboxconfiguration[x] = 0;
    }
//   usersettings.checkboxconfiguration[CHECKBOXARM]=CHECKBOXMASKAUX1HIGH;
    usersettings.checkboxconfiguration[CHECKBOXHIGHANGLE] = CHECKBOXMASKAUX1LOW;
    usersettings.checkboxconfiguration[CHECKBOXSEMIACRO] = CHECKBOXMASKAUX1HIGH;
    usersettings.checkboxconfiguration[CHECKBOXHIGHRATES] = CHECKBOXMASKAUX1HIGH;
	
		// reset the calibration settings
    for (int x = 0; x < 3; ++x) {
        usersettings.compasszerooffset[x] = 0;
        usersettings.compasscalibrationmultiplier[x] = 1L << FIXEDPOINTSHIFT;
        usersettings.gyrocalibration[x] = 0;
        usersettings.acccalibration[x] = 0;
    }
#if CONTROL_BOARD_TYPE == CONTROL_BOARD_WLT_V202
    usersettings.boundprotocol = 0; // PROTO_NONE
    usersettings.txidsize = 0;
    usersettings.fhsize = 0;
#endif
}

// Executes command based on stick movements.
// Call this only when not armed.
// Currently implemented: accelerometer calibration
static void detectstickcommand(void) {
    // Timeout for stick movements for accelerometer calibration
    static uint32_t stickcommandtimer;
    // Keeps track of roll stick movements while not armed to execute accelerometer calibration.
    static stickstate_t lastrollstickstate = STICK_STATE_START;
    // Counts roll stick movements
    static uint8_t rollmovecounter;

    // Accelerometer calibration (3x back and forth movement of roll stick while
    // throttle is in lowest position)
    if (global.rxvalues[THROTTLEINDEX] < FPSTICKLOW) {
        if (global.rxvalues[ROLLINDEX] < FP_RXMOVELOW) {
            // Stick is now low. What has happened before?
            if(lastrollstickstate == STICK_STATE_START) {
                // We just come from start position, so this is our first movement
                rollmovecounter=1;
                lastrollstickstate = STICK_STATE_LOW;
                // Detected stick movement, so restart timeout.
                stickcommandtimer = lib_timers_starttimer();
            } else if (lastrollstickstate == STICK_STATE_HIGH) {
                // Stick had been high recently, so increment counter
                rollmovecounter++;
                lastrollstickstate = STICK_STATE_LOW;
                // Detected stick movement, so restart timeout.
                stickcommandtimer = lib_timers_starttimer();
            } // else: nothing happened, nothing to do
        } else if (global.rxvalues[ROLLINDEX] > FP_RXMOVEHIGH) {
            // And now the same in opposite direction...
            if(lastrollstickstate == STICK_STATE_START) {
                // We just come from start position
                rollmovecounter=1;
                lastrollstickstate = STICK_STATE_HIGH;
                // Detected stick movement, so restart timeout.
                stickcommandtimer = lib_timers_starttimer();
            } else if (lastrollstickstate == STICK_STATE_LOW) {
                // Stick had been low recently, so increment counter
                rollmovecounter++;
                lastrollstickstate = STICK_STATE_HIGH;
                // Detected stick movement, so restart timeout.
                stickcommandtimer = lib_timers_starttimer();
            } // else: nothing happened, nothing to do
        }

        if(lib_timers_gettimermicroseconds(stickcommandtimer) > 1000000L) {
            // Timeout: last detected stick movement was more than 1 second ago.
            lastrollstickstate = STICK_STATE_START;
        }

        if(rollmovecounter == 6) {
            // Now we had enough movements. Execute calibration.
            calibrategyroandaccelerometer(true);
            // Save in EEPROM
            writeusersettingstoeeprom();
            lastrollstickstate = STICK_STATE_START;
        }
    } // if throttle low
} // checkforstickcommand()
