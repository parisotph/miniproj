/*
 * move.c
 *
 *  Created on: 17 avr. 2020
 *      Author: Philippe Parisot and Lucas Bost
 */
#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>
#include <motors.h>
#include <process_image.h>
#include <odometry.h>
#include "sensors/VL53L0X/VL53L0X.h"
#include "audio/play_melody.h"
#include <move.h>
#include "msgbus/messagebus.h"
#include <leds.h>

/*********************************************GLOBAL VARIABLES************************************************/
//declaration of the system state and initialization in TURN mode
static uint8_t system_state = TURN;
/*******************************************END GLOBAL VARIABLES**********************************************/

/*********************************************PRIVATE FUNCTIONS***********************************************/
//set the speeds of the motors
static void set_robot(int16_t right_speed, int16_t left_speed){
	right_motor_set_speed(right_speed);
	left_motor_set_speed(left_speed);
}

//calculaton of the speed with a PI regulator
static int16_t pi_regulator(float distance, float goal){
	float error = 0;
	float speed = 0;
	static float sum_error = 0;
	//computation of the error
	error = distance - goal;
	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
			return 0;
	}
	//if the intruder is closer than 10cm the robot still advance in order to reach it
	if(error < 0){
		return (2*CST_SPEED);
	}
	//computation of the sum of error
	sum_error += error;
	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}
	//computation of the speed;
	speed = KP * error + KI * sum_error;
	//return the corresponding integer value
    return (int16_t)speed;
}

//function to stop the robot if the system state is not valid
static void error_invalid_state(void){
	set_robot(STOP, STOP);
}

static THD_WORKING_AREA(waMove, 256);
static THD_FUNCTION(Move, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    int16_t speed, speed_correction, right_speed, left_speed;
    uint16_t n=0;
    uint16_t first_measure, second_measure;
    uint8_t dist_reached=0, angle_reached=0, origin_reached=0;

    while(1){
    		time = chVTGetSystemTime();
    		first_measure = VL53L0X_get_dist_mm();
    		//switch to control the FSM
    		switch(system_state){

    		case TURN:
    			if(first_measure < D_MAX){
    				//skip the first 500 ms beacause the first measurement by ToF is 0
    				if(n == HALF_SECOND){
    					//stop the robot in front of the intruder
    					set_robot(STOP, STOP);
    					//make a warning sound
    					playNote(NOTE_INTENSITY , NOTE_DURATION);
    					chThdSleepMilliseconds(WAIT_TIME);
    					second_measure = VL53L0X_get_dist_mm();
    					//test if the object is still close to the robot
    					if(second_measure <= D_MAX){
    						//change the state
    						system_state = PURSUIT;
    					}
    					else{
    						//turn the robot at constant speed
    						set_robot(CST_SPEED, -CST_SPEED);
    					}
    				}
    				else{
    					n++;
    				}
    			}
    			else{
    				//turn the robot at constant speed
    				set_robot(CST_SPEED, -CST_SPEED);
    			}
    		break;

    		case PURSUIT:
    			set_body_led(ON);
    			//test condition for distance
    			dist_reached = get_dist_condition();
    			if(dist_reached == ACHIEVE){
    				set_robot(STOP, STOP);
    				system_state = COMEBACK;
    			}
    			else{
    				//speed is given by the regulator
    				speed = pi_regulator(get_distance_cm(), GOAL_DISTANCE);
    				speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));
    				//if the line is nearly in front of the camera the robot doesn't rotate
    				if(abs(speed_correction) < ROTATION_THRESHOLD){
    					speed_correction = 0;
    				}
    				//add correction
    				right_speed = speed - ROTATION_COEFF * speed_correction;
    				left_speed = speed + ROTATION_COEFF * speed_correction;
    				set_robot(right_speed, left_speed);
    			}
    		break;

    		case COMEBACK:
    			set_body_led(OFF);
    			//test condition for alignment with origin
    			angle_reached = get_angle_condition();
    			if(angle_reached == ACHIEVE){
    				//test condition for origin
    				origin_reached = get_origin_condition();
    				if(origin_reached == ACHIEVE){
    					set_robot(STOP, STOP);
    					//return to the beginning state
    					system_state = TURN;
    					//reset elements for odometry
    					reset_map();
    					reset_odometry();
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
    			//in default mode, we call error function
    			error_invalid_state();
    		}
    		chThdSleepUntilWindowed(time, time + MS2ST(1));
    }
}
/*******************************************END PRIVATE FUNCTIONS********************************************/

/*********************************************PUBLIC FUNCTIONS***********************************************/
void move_start(void){
	chThdCreateStatic(waMove, sizeof(waMove), NORMALPRIO, Move, NULL);
}

uint8_t get_system_state(void){
	return system_state;
}
/*******************************************END PUBLIC FUNCTIONS*********************************************/











