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
#include "audio/play_melody.h"
#include <move.h>
#include "msgbus/messagebus.h"
#include <leds.h>
#include <filter.h>

/*********************************************GLOBAL VARIABLES************************************************/
//declaration of the system state and initialization in TURN mode
static uint8_t system_state = TURN;
/*******************************************END GLOBAL VARIABLES**********************************************/

//declaration of the bus
messagebus_t bus1;
MUTEX_DECL(bus1_lock);
CONDVAR_DECL(bus1_condvar);

/*********************************************INTERNAL FUNCTIONS**********************************************/
//set the speeds of the motors
static void set_robot(int16_t right_speed, int16_t left_speed){
	right_motor_set_speed(right_speed);
	left_motor_set_speed(left_speed);
}

//calculation of the speed with a PI regulator
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
		return (3*CST_SPEED);
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

// move thread
static THD_WORKING_AREA(waMove, 256);
static THD_FUNCTION(Move, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    int16_t speed, speed_correction, right_speed, left_speed;
    //counter to skip the first 500ms
    uint8_t cnt1, cnt2, cnt3;
    //uint16_t first_measure, second_measure, edge_measure;
    uint8_t dist_reached=0, angle_reached=0, origin_reached=0;
    /** Inits the Inter Process Communication bus. */
   messagebus_init(&bus1, &bus1_lock, &bus1_condvar);
   messagebus_topic_t *odm_topic = messagebus_find_topic_blocking(&bus1, "/odm");
   odm_msg_t odm_values;

    while(1){
    		time = chVTGetSystemTime();
    		//wait for new values to be published
    		messagebus_topic_wait(odm_topic, &odm_values, sizeof(odm_values));

    		//switch to control the FSM
    		switch(system_state){

    		case TURN:
    			//test if an intruder is close
    			cnt1 = get_cnt1();
    			if(cnt1 >= LIMIT){
    				//stop the robot in front of the intruder
    				set_robot(STOP, STOP);
    				//lights the front LED
    				set_front_led(ON);
    				//makes a warning sound
    				playNote(NOTE_INTENSITY , NOTE_DURATION);
    				//waits 2.5 seconds
    				chThdSleepMilliseconds(WAIT_TIME);
    				cnt3 = get_cnt3();
    				//test if the object is still in the no-go zone
    				if(cnt3 >= LIMIT){
    					//change the state
    					system_state = PURSUIT;
    				}
    			}
    			else{
    				cnt2 = get_cnt2();
    				//if an intruder is in the perimeter, emits a light warning
    				if(cnt2 >= LIMIT){
    					set_front_led(ON);
    				}
    				else{
    					set_front_led(OFF);
    				}
    				set_robot(CST_SPEED, -CST_SPEED);
    			}
    		break;

    		case PURSUIT:
    			//switches off front led
    			set_front_led(OFF);
    			//lights the body led if it's in PURSUIT mode
    			set_body_led(ON);
    			//test condition for distance
    			dist_reached = odm_values.cd1;
    			if(dist_reached == ACHIEVE){
    				set_robot(STOP, STOP);
    				//change the state
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
    			//switches off the body led
    			set_body_led(OFF);
    			//test condition for alignment with origin
    			angle_reached = odm_values.cd2;
    			if(angle_reached == ACHIEVE){
    				//test condition for origin
    				origin_reached = odm_values.cd3;
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
    			//in default mode, we call error function, which stops the robot
    			error_invalid_state();
    		}
    		chThdSleepUntilWindowed(time, time + MS2ST(1));
    }
}
/*******************************************END INTERNAL FUNCTIONS*******************************************/

/*********************************************PUBLIC FUNCTIONS***********************************************/
void move_start(void){
	chThdCreateStatic(waMove, sizeof(waMove), NORMALPRIO, Move, NULL);
}

uint8_t get_system_state(void){
	return system_state;
}
/*******************************************END PUBLIC FUNCTIONS*********************************************/











