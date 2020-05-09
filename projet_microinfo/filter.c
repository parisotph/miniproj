/*
 * filter.c
 *
 *  Created on: 7 mai 2020
 *      Author: Philippe Parisot and Lucas Bost
 */
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <main.h>
#include "sensors/proximity.h"
#include "sensors/VL53L0x/VL53L0X.h"
#include <filter.h>

/*********************************************GLOBAL VARIABLES************************************************/
//declarations of the counters
static uint8_t counter1=0, counter2=0, counter3=0, counter4 = 0, measure_cnt=0;
/*******************************************END GLOBAL VARIABLES**********************************************/

/*********************************************INTERNAL FUNCTIONS**********************************************/
static void reset_counter(void){
	counter1 = 0;
	counter2 = 0;
	counter3 = 0;
	counter4 = 0;
	measure_cnt = 0;
}

static THD_WORKING_AREA(waFilter, 256);
static THD_FUNCTION(Filter, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    uint16_t measure;
    uint8_t n=0;
    float mean;

    while(1){
    	//the filter looks at how many values per sample reach the conditions
    	measure = VL53L0X_get_dist_mm();
    	//skip the first 500 ms
    	if(n == HALF_SECOND){
			if(measure < TOF_THRESHOLD){
				counter1++;
			}
			else{
				if(measure < EDGE){
					counter2++;
				}
			}
			//for the second measurement we take in account the sensor instability
			if(measure < (TOF_THRESHOLD + TOF_ERROR)){
				counter3++;
			}
			//average between the values of the front right and front left IR sensors
			mean = (get_prox(0)+get_prox(7))/2;
			if(mean > IR_THRESHOLD){
				counter4++;
			}
			measure_cnt++;
			//if 6 measures have been done, reset all the counters to make 6 other measurements
			if(measure_cnt == COUNT){
				reset_counter();
			}
    	}
    	else{
    		n++;
    	}
    	//sleep milliseconds to be sure to have differents values for each sensors
    	chThdSleepMilliseconds(100);
    }
}
/*******************************************END INTERNAL FUNCTIONS*******************************************/

/*********************************************PUBLIC FUNCTIONS***********************************************/
void filter_start(void){
	chThdCreateStatic(waFilter, sizeof(waFilter), NORMALPRIO, Filter, NULL);
}

//functions to transmit the values of all counters
uint8_t get_cnt1(void){
	return counter1;
}

uint8_t get_cnt2(void){
	return counter2;
}

uint8_t get_cnt3(void){
	return counter3;
}

uint8_t get_cnt4(void){
	return counter4;
}
/*******************************************END PUBLIC FUNCTIONS*********************************************/

