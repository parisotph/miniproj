/* Main file for the project */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include <process_image.h>
#include <odometry.h>
#include <move.h>
#include <filter.h>

//define the bus to receive measurements from the IR sensors
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int main(void)
{
/***********************************************MAIN INITIALIZATIONS***********************************************/
    halInit();
    chSysInit();
    mpu_init();
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    dac_start();             //starts the speaker
    dcmi_start();            //starts the camera
	po8030_start();
	motors_init();           //inits the motors
/**********************************************THREADS INITIALIZATIONS**********************************************/
	proximity_start();	     // IR sensors
	move_start();	         // PI regulator
	process_image_start();   // camera
	odometry_start();        // odometry
	VL53L0X_start();		 // ToF sensor
	playMelodyStart();	     // Sound
	filter_start();          //filter

    /* Infinite loop. */
    while (1) {
    		chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

