/*
 * odometry.c
 *
 *  Created on: 24 avr. 2020
 *      Author: djed&gunners
 */
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <math.h>


#include <main.h>
#include <odometry.h>
#include <pi_regulator.h>
#include <motors.h>

static float teta = 0, xc = 0, yc = 0, dteta = 0, dxc = 0, dyc = 0, dist = 0, dc = 0;
static float phi = 0, turn_angle = 0;
static uint8_t dist_reached = 0, angle_reached = 0, origin_reached = 0;
static int32_t delta, sum;
static int32_t last_r_pos = 0, last_l_pos = 0;
static int32_t r_pos, l_pos;
static int32_t delta_r_pos, delta_l_pos;

void variation_calcul(void){
	r_pos = right_motor_get_pos();
	l_pos = left_motor_get_pos();
	delta_r_pos = r_pos - last_r_pos;
	delta_l_pos = l_pos - last_l_pos;
	delta = delta_r_pos - delta_l_pos;
	sum = delta_r_pos + delta_l_pos;
	dteta = TETA_FACTOR * delta;
	dc = POS_FACTOR * sum;
	last_r_pos = r_pos;
	last_l_pos = l_pos;
}

// update the angle of the robot
void update_robot(void){
	teta += dteta;
	if(teta >= 2*PI){
		teta -= 2*PI;
	}
	if(teta < -2*PI){
		teta += 2*PI;
	}
	dxc = dc*cos(teta);
	dyc = dc*sin(teta);
	xc += dxc;
	yc += dyc;
	dist = sqrt(xc*xc + yc*yc);
}

static THD_WORKING_AREA(waOdometry, 256);
static THD_FUNCTION(Odometry, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;


    uint8_t state;
    uint8_t target_captured;/*, target_out_range;*/
    int16_t right_speed, left_speed;



    //float r_pos, l_pos;

    systime_t time;

    while(1){
    		time = chVTGetSystemTime();
    		variation_calcul();
    		update_robot();


    	// 1 kHz
    		chThdSleepUntilWindowed(time, time + MS2ST(1));
    		//chThdSleepMilliseconds(1);
    }
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

float get_teta(void){
	return teta;
}

float get_dist(void){
	return dist;
}

float get_dteta(void){
	return dteta;
}

int32_t get_delta(void){
	return delta;
}

int32_t get_delta_r_pos(void){
	return delta_r_pos;
}

int32_t get_delta_l_pos(void){
	return delta_l_pos;
}

int32_t get_last_r_pos(void){
	return last_r_pos;
}

int32_t get_last_l_pos(void){
	return last_l_pos;
}

void reset_odometry(void){
	teta = 0;
	xc = 0;
	yc = 0;
	dist = 0;
	dc = 0;
	dteta = 0;
	dxc = 0;
	dyc = 0;
	phi = 0;
	turn_angle = 0;
	dist_reached = 0;
	angle_reached = 0;
	origin_reached = 0;
	right_motor_set_pos(0);
	left_motor_set_pos(0);
}
