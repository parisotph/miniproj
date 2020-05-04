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
#include "audio/play_melody.h"

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
		target_captured = ACHIEVE;
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
    uint16_t n= 0;
    uint16_t first_measure;
    uint16_t second_measure;
    uint8_t dist_reached;
    uint8_t angle_reached;
	uint8_t origin_reached;
	uint8_t line_not_found;

    while(1){
    		time = chVTGetSystemTime();
    		first_measure = VL53L0X_get_dist_mm();

    		switch(system_state){

    		case TURN:
    			if(first_measure < D_MAX){
    				if(n == 500){
    					set_robot(STOP, STOP);
    					playNote(9, 200);
    					chThdSleepMilliseconds(WAIT_TIME);
    					second_measure = VL53L0X_get_dist_mm();
    					if(second_measure <= D_MAX){
    						system_state = PURSUIT;
    					}
    					else{
    						set_robot(CST_SPEED, -CST_SPEED);
    					}
    				}
    				else{
    					n++;
    				}
    			}
    			else{
    				set_robot(CST_SPEED, -CST_SPEED);
    				//set_robot(STOP, STOP);
    			}
    		break;

    		case PURSUIT:
    			//playMelody(IMPOSSIBLE_MISSION, ML_SIMPLE_PLAY, NULL);
    			    			dist_reached = get_dist_condition();
    			if(dist_reached){
    				set_robot(STOP, STOP);
    				system_state = COMEBACK;
    			}
    			else{
    				speed = pi_regulator(get_distance_cm(), GOAL_DISTANCE);
    				speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));
    				if(abs(speed_correction) < ROTATION_THRESHOLD){
    					speed_correction = 0;
    				}
    				right_speed = speed - ROTATION_COEFF * speed_correction;
    				left_speed = speed + ROTATION_COEFF * speed_correction;
    				set_robot(right_speed, left_speed);
    				//set_robot(speed, speed);
    			}
    		break;

    		case COMEBACK:
    			angle_reached = get_angle_condition();
    			if(angle_reached){
    				origin_reached = get_origin_condition();
    				if(origin_reached){
    					set_robot(STOP, STOP);
    					system_state = TURN;
    					reset_map();
    					reset_odometry();
    					reset_pursuit();
    				}
    				else{
    					set_robot(2*CST_SPEED, 2*CST_SPEED);
    				}
    			}
    			else{
    				set_robot(CST_SPEED, -CST_SPEED);
    			}
    		break;

    		default:
    			error_invalid_state();
    		}

    		chThdSleepUntilWindowed(time, time + MS2ST(1));
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

void error_invalid_state(void){
	set_robot(STOP, STOP);
}

