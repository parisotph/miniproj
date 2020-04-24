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

#define DELTAT       0.000001
#define WHEELDIST    5.4
#define CONVERT      180
#define PI           3.14159

static int16_t right_speed;
static int16_t left_speed;
static float teta = 0;


static THD_WORKING_AREA(waOdometry, 256);
static THD_FUNCTION(Odometry, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    float dteta;
    systime_t time;

    while(1){
    	time = chVTGetSystemTime();

    	right_speed = get_right_speed();
    	left_speed = get_left_speed;

    	dteta = (CONVERT * DELTAT)/(PI * WHEELDIST) * ((right_speed - left_speed)/2);

    	teta += dteta;

    	chThdSleepUntilWindowed(time, time + US2ST(1));
    }
}

void odometry_start(void){
	chThdCreateStatic(waOdometry, sizeof(waOdometry), NORMALPRIO, Odometry, NULL);
}

float get_teta(void){
	return teta;
}
