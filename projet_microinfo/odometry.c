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

static float teta = 0, xc = 0, yc = 0, dteta = 0, dxc = 0, dyc = 0, dist = 0, dc = 0;
static float phi = 0, turn_angle = 0;
static uint8_t dist_reached = 0, angle_reached = 0, origin_reached = 0;
static int32_t delta;

/*void variation_calcul(int16_t s_r, int16_t s_l){
	dteta = FACTOR * (s_r - s_l);
	dc = T_FACTOR * (s_r + s_l);
	dxc = -dc * sin(dteta);
	dyc = dc * cos(dteta);
}*/

void update_robot(void){
	teta += dteta;
	if(teta >= 2*PI){
		teta -= 2*PI;
	}
	if(teta < -2*PI){
		teta += 2*PI;
	}
	/*xc += dxc;
	yc += dyc;
	dist = sqrt(xc*xc + yc*yc);*/
}

static THD_WORKING_AREA(waOdometry, 256);
static THD_FUNCTION(Odometry, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;


    uint8_t state;
    uint8_t target_captured;/*, target_out_range;*/
    int16_t right_speed, left_speed;
    int16_t position_r;
    int16_t position_l;
    //float r_pos, l_pos;


    systime_t time;

    while(1){
    	time = chVTGetSystemTime();
    	position_r = right_motor_get_pos();
    	position_l = left_motor_get_pos();
    	delta = position_r - position_l;
    	dteta = TETA_FACTOR * delta;
    	right_motor_set_pos(0);
    	left_motor_set_pos(0);
    	/*dc = POS_FACTOR * (right_motor_get_pos()+left_motor_get_pos());
    	dxc = -dc*sin(dteta);
    	dyc = dc*cos(dteta);
    	right_motor_set_pos(0);
    	left_motor_set_pos(0);*/
    	state = get_system_state();

    	if(state == TURN){
    		update_robot();
    	}

    	if(state == PURSUIT){
    		/*target_captured = get_target_situation();
    		if(dist >= PERIMETER_RADIUS || target_captured){
    			phi = atan(yc/xc);
    			turn_angle = PI + phi - teta;
    			dist_reached = 1;
    		}
    		else{*/
    			update_robot();
    		//}
    	}

    	/*if(state == COMEBACK){
    		update_robot();
    		if(teta >= turn_angle){
    			angle_reached = 1;
    		}
    		if(dist <= NEAR_ORIGIN){
    			origin_reached = 1;
    		}
    	}*/
    	// 1 kHz
    	chThdSleepUntilWindowed(time, time + MS2ST(4));
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
