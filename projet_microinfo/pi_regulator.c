#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
//#include <VL53L0X.h>
#include <odometry.h>
#include "sensors/VL53L0X/VL53L0X.h"

//static uint8_t system_state = TURN;
static uint8_t target_captured;
//static uint8_t target_out_range;
static int16_t left_speed;
static int16_t right_speed;
static uint8_t system_state = TURN;



void set_robot(int16_t right_speed, int16_t left_speed){
	right_motor_set_speed(right_speed);
	left_motor_set_speed(left_speed);
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

	//We don't want the robot to stop at a certain distance but we want it to touch the object
	if(fabs(error) < ERROR_THRESHOLD || error < 0){
		return 0;
	}

	if(error < -CAUGHT){
		target_captured = 1;
	}

	/*if(error > OUT_RANGE){
		target_out_range = 1;
	}*/

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

    int16_t speed = 0;
    int16_t speed_correction;
    systime_t time;
    uint8_t n= 0;
    uint16_t mesure;
    uint8_t dist_reached;
    uint8_t angle_reached;
	uint8_t origin_reached;

    while(1){
    		time = chVTGetSystemTime();

    		set_robot(CST_SPEED, -CST_SPEED);

    		chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}

/*int16_t get_right_speed(void){
	return right_speed;
}

int16_t get_left_speed(void){
	return left_speed;
}*/

uint8_t get_system_state(void){
		return system_state;
}

uint8_t get_target_situation(void){
	return target_captured;
}

/*uint8_t get_pursuit_situation(void){
	return target_out_range;
}*/

void reset_pursuit(void){
	target_captured = 0;
	//target_out_range = 0;
}

