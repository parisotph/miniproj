#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <VL53L0X.h>

//static uint8_t system_state = TURN;
static uint16_t mesure;

void turn_robot(int16_t speed){
	right_motor_set_speed(speed);
	left_motor_set_speed(-speed);
}

void stop_robot(){
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}


//simple PI regulator implementation
int16_t pi_regulator(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD || error < 0){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    //uint16_t mesure;
    //int16_t left_speed;
    //int16_t right_speed;
    //int16_t speed = 0;
    systime_t time;
    uint8_t n= 0;
    uint8_t system_state = TURN;

    while(1){
    	time = chVTGetSystemTime();

    	mesure = VL53L0X_get_dist_mm();



    	if(system_state == TURN){
    			if(mesure < D_MAX){
    			//system_state = PURSUIT;
    				if(n >= 50){
    					system_state = PURSUIT;
    				}

    				else{
    					n++;
    				}

    				//stop_robot()
    			}
    			else{
    				turn_robot(V_TURN);
    			}
    	}


    	if(system_state == PURSUIT){
    			stop_robot();
    	}



    	//chprintf((BaseSequentialStream *)&SD3, "dist = %d<n", system_state);

    	chThdSleepUntilWindowed(time, time + MS2ST(10));
    }

        //100Hz
        //chThdSleepUntilWindowed(time, time + MS2ST(10));
}

uint16_t get_measure(){
	return mesure;
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
