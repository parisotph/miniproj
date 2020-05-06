/*
 * odometry.c
 *
 *  Created on: 17 avr. 2020
 *      Author: Philippe Parisot and Lucas Bost
 */
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <math.h>
#include <main.h>
#include <odometry.h>
#include <process_image.h>
#include <motors.h>
#include <move.h>
#include "msgbus/messagebus.h"
#include "sensors/proximity.h"

/*********************************************GLOBAL VARIABLES************************************************/
//declaration of the coordinates of the robot and angle and distance useful variables
static float teta=0, xc=0, yc=0, dteta=0, dc=0, dxc=0, dyc=0, dist=0, distance=0, phi=0, turn_angle=0;
//declaration of the condition of state changes
static uint8_t dist_reached=0, angle_reached=0, origin_reached=0, target_captured=0;
//declaration of the variables using motor positions
static int32_t r_pos=0, l_pos=0, last_r_pos=0, last_l_pos=0;
/*******************************************END GLOBAL VARIABLES**********************************************/

/*********************************************PRIVATE FUNCTIONS***********************************************/
//calculates the variation of angle of the robot and the distance traveled by the center robot
static void variation_calcul(void){
	//get the motors' position (in steps)
	r_pos = right_motor_get_pos();
	l_pos = left_motor_get_pos();
	//we calculate the angle variation knowing it's proportional to the difference of the variation of steps
	//for the right motor r_pos - last_r_pos
	//for the left motor l_pos - last_l_pos
	dteta = TETA_FACTOR * ((r_pos - last_r_pos) - (l_pos - last_l_pos));
	//we calculate the distance traveled by the center of the robot knowing it's proportional
	//to the half of the sum of the variation of steps
	dc = POS_FACTOR * ((r_pos - last_r_pos) + (l_pos - last_l_pos));
	//save the last positions
	last_r_pos = r_pos;
	last_l_pos = l_pos;
}

//update the position, orientation and distance to origin of the robot
static void update_robot(void){
	teta += dteta;
	// teta must be in ]-2pi,2pi[
	if(teta >= 2*PI){
		teta -= 2*PI;
	}
	if(teta <= -2*PI){
		teta += 2*PI;
	}
	//calculates the variation of position using distance traveled by the robot and its
	//orientation
	dxc = dc * cos(teta);
	dyc = dc * sin(teta);
	//updates position and distance
	xc += dxc;
	yc += dyc;
	dist = sqrt(xc*xc + yc*yc);
}

static void processProximity(void){
	//get the values of IR sensors
	uint16_t value1, value2;
	value1 = get_prox(IRA);
	value2 = get_prox(IRB);
	//performs an average
	float mean;
	mean = (value1+value2)/2;
	if(mean > IR_MIN){
		//if the target is close enough, it is captured
		target_captured = ACHIEVE;
	}
}

//odometry thread
static THD_WORKING_AREA(waOdometry, 256);
static THD_FUNCTION(Odometry, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    uint8_t state;

    while(1){
    		time = chVTGetSystemTime();
    		state = get_system_state();
    		//calculates the variation of orientation and distance traveled
    		variation_calcul();
    		//skip the first 500 ms
    		//test if the system is in PURSUIT state
			if(state == PURSUIT){
				//test if the robot is at the edge of the perimeter
				processProximity();
				if(dist >= PERIMETER_RADIUS || target_captured == ACHIEVE){
					//save the distance to origin, angle of the robot position and angle to align with origin
					phi = atan(yc/xc);
					distance = dist;
					turn_angle = PI + phi - teta;
					dist_reached = ACHIEVE;
					//reset the coordinate system to avoids amplification of the error
					reset_map();
				}
				else{
					update_robot();
				}
			}
			//test if the robot is in COMEBACK state
			if(state == COMEBACK){
				update_robot();
				//if the robot is aligned with origin
				if(teta >= turn_angle){
					angle_reached = ACHIEVE;
				}
				//if the robot  has returned to origin
				if(dist >= distance){
					origin_reached = ACHIEVE;
				}
			}
    		chThdSleepUntilWindowed(time, time + MS2ST(1));
    		//chThdSleepMilliseconds(1);
    }
}
/*******************************************END PRIVATE FUNCTIONS********************************************/

/*********************************************PUBLIC FUNCTIONS***********************************************/
//proceeds to a reset of the coordinate system
void reset_map(void){
	teta = 0;
	xc = 0;
	yc = 0;
	dist = 0;
}

//proceeds to a reset of the variables used during odometry
void reset_odometry(void){
	dc = 0;
	dteta = 0;
	dxc = 0;
	dyc = 0;
	phi = 0;
	turn_angle = 0;
	dist_reached = 0;
	angle_reached = 0;
	origin_reached = 0;
	target_captured = 0;
	//reset the steps counters
	right_motor_set_pos(0);
	left_motor_set_pos(0);
}

void odometry_start(void){
	chThdCreateStatic(waOdometry, sizeof(waOdometry), NORMALPRIO, Odometry, NULL);
}

uint8_t get_dist_condition(void){
	return dist_reached;
}

uint8_t get_angle_condition(void){
	return angle_reached;
}

uint8_t get_origin_condition(void){
	return origin_reached;
}
/*******************************************END PUBLIC FUNCTIONS*********************************************/


