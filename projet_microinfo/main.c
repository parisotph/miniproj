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
//#include <VL53L0X.h>
//#include <messagebus.h>
#include <i2c_bus.h>

#include <process_image.h>
#include <odometry.h>
#include "move.h"

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
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    //starts the serial communication
    serial_start();
    i2c_start();
    dac_start();
    //start the USB communication
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();
	//inits the motors
	motors_init();
	//odometry_start();
	//messagebus_init(&bus, &bus_lock, &bus_condvar);
	proximity_start();
	//stars the threads for the pi regulator and the processing of the image
	move_start();
	process_image_start();
	odometry_start();

	VL53L0X_start();
	playMelodyStart();
	//move_start();
    /* Infinite loop. */
    while (1) {
    		chThdSleepMilliseconds(1000);
    }
}

/*int16_t get_right_speed(void){
	return right_motor_speed;
}

int16_t get_left_speed(void){
	return left_motor_speed;
}
*/
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

