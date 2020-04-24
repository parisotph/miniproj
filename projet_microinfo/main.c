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
#include <odometry.h>

enum etat {TOURNE, POURSUIT, REVIENT};
static uint8_t Etat = TOURNE;
static uint8_t n = 0;
static int16_t right_motor_speed;
static int16_t left_motor_speed;
static int16_t speed;
static float angle = 0;
/*static float var_angle = 0;
static int32_t r_pos;
static int32_t l_pos;*/

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
	//odometry_start();

	/*//stars the threads for the pi regulator and the processing of the image
	pi_regulator_start();
	process_image_start();*/

	VL53L0X_start();



    /* Infinite loop. */
    while (1) {


    	if(Etat == TOURNE){

    			uint16_t mesure;
    			mesure = VL53L0X_get_dist_mm();
    			chprintf((BaseSequentialStream *)&SD3, "Distance = %d\n", mesure);

    	    //if(Etat == TOURNE){
    		//int16_t angle = 0;

    		if(mesure < D_MAX) {
    			//int16_t teta0;
    			//teta0 = angle;
    			if(n){
    				Etat = POURSUIT;
    			}
    			else{
    				n = 1;
    			}
    		}
    		else{
    			speed = START;
    			right_motor_set_speed(speed);
    			left_motor_set_speed(-speed);
    		}
    		}
    		if(Etat == POURSUIT){
    			speed = STOP;
    			right_motor_set_speed(speed);
    			left_motor_set_speed(speed);
    			//angle = get_teta();

    		}
    		/*else{
    			Etat = TOURNE;
    		}*/
    		/*else{
    			speed = START;
    			right_motor_set_speed(speed);
    			left_motor_set_speed(-speed);
    		}
    	}
    	else{
    		speed = STOP;
    		right_motor_set_speed(speed);
    		left_motor_set_speed(speed);
    	}*/

    	/*speed = START;
    	right_motor_set_speed(speed);
    	left_motor_set_speed(-speed);
    	r_pos = right_motor_get_pos();
    	l_pos = left_motor_get_pos();

    	/*if (angle > 360){
    		angle = 0;
    	}*/
    	chprintf((BaseSequentialStream *)&SD3, "Etat = %d\n", Etat);
    	chThdSleepMilliseconds(1000);
    }
}

int16_t get_right_speed(void){
	return right_motor_speed;
}

int16_t get_left_speed(void){
	return left_motor_speed;
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

