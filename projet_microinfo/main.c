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
#include <VL53L0X.h>
#include <messagebus.h>
#include <i2c_bus.h>

#include <pi_regulator.h>
#include <process_image.h>

enum etat {TOURNE, POURSUIT, REVIENT};
static uint8_t Etat = TOURNE;
static uint16_t speed;

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    i2c_start();
    //start the USB communication
    usb_start();
    /*//starts the camera
    dcmi_start();
	po8030_start();*/
	//inits the motors
	motors_init();

	/*//stars the threads for the pi regulator and the processing of the image
	pi_regulator_start();
	process_image_start();*/

    //VL53L0X_start();



    /* Infinite loop. */
    while (1) {
    	if(Etat == TOURNE){
    		VL53L0X_start();
    		speed = START;
    		right_motor_set_speed(speed);
    		left_motor_set_speed(-speed);
    		/*uint16_t mesure = 0;
    		mesure = VL53L0X_get_dist_mm();
    		if (mesure < D_MAX){
    			Etat = POURSUIT;
    		}*/
    	}

    	/*if(Etat == POURSUIT){
    		speed = STOP;
    		right_motor_set_speed(speed);
    		left_motor_set_speed(speed);
    	}*/

        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
