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


#include <main.h>
#include <odometry.h>


static int16_t right_speed;
static int16_t left_speed;
static float teta = 0, xc = 0, yc = 0, dteta = 0, dxc = 0, dyc = 0, dist = 0, dc = 0;
static uint8_t dist_reached = 0, angle_reached = 0, origin_reached = 0;

void variation_calcul(int16_t s_r, int16_t s_l){
	dteta = FACTOR * (s_r - s_l);
	dc = T_FACTOR * (s_r + s_l);
	dxc = -dc * sin(dteta);
	dyc = dc * cos(dteta);
}

void reset_coordinate(float angle, float x_pos, float y_pos, float distance){
	angle = 0;
	x_pos = 0;
	y_pos = 0;
	distance = 0;
}

void reset_variables(uint8_t dist_reached, uint8_t angle_reached, uint8_t origin_reached, float turn_angle, float phi){
	dist_reached = 0;
	angle_reached = 0;
	origin_reached = 0;
	turn_angle = 0;
	phi = 0;
}

void update_robot(float angle, float x_pos, float y_pos, float delta_angle, float delta_x_pos, float delta_y_pos){
	angle += delta_angle;
	if(angle >= 2_PI){
		angle -= 2_PI;
	}
	if(angle <= -2_PI){
		angle += 2_PI;
	}
	x_pos += delta_x_pos;
	y_pos += delta_y_pos;
	dist = sqrt(x_pos*x_pos + y_pos*y_pos);
}

static THD_WORKING_AREA(waOdometry, 256);
static THD_FUNCTION(Odometry, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    float phi = 0, turn_angle = 0;
    uint8_t state;
    uint8_t first_time = 1;
    int16_t right_speed, left_speed;

    systime_t time;

    while(1){
    	time = chVTGetSystemTime();

    	variation_calcul(right_speed, left_speed);

    	if(state == TURN){
    		reset_variables(dist_reached, angle_reached, origin_reached);
    		if(first_time){
    			reset_coordinate(teta, xc, yc, dist);
    			first_time = 0;
    		}
    		else{
    			update_robot(teta, xc, yc, dteta, dxc, dyc);
    		}
    	}
    	if(state == PURSUIT){
    		if(dist >= PERIMETER_RADIUS || target_captured){
    			phi = atan(yc/xc);
    			turn_angle = PI + phi - teta;
    			dist_reached = 1;
    		}
    		else{
    			update_robot(teta, xc, yc, dteta, dxc, dyc);
    		}
    	}
    	if(state == COMEBACK){
    		update_robot(teta, xc, yc, dteta, dxc, dyc);
    		if(angle >= turn_angle){
    			angle_reached = 1;
    		}
    		if(dist <= NEAR_ORIGIN){
    			origin_reached = 1;
    		}
    	}

    	chThdSleepUntilWindowed(time, time + US2ST(1));
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
